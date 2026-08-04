#include "ArmorBoneReference.h"

#include "Address.h"
#include "BSSkin.h"
#include "Fo4NiObjectUtils.h"
#include "RE/B/BSFlattenedBoneTree.h"
#include "RE/B/BSGeometry.h"
#include "RE/N/NiAVObject.h"
#include "RE/N/NiNode.h"
#include "RE/N/NiUpdateData.h"
#include "RE/T/TESObjectREFR.h"

#include <cctype>
#include <functional>
#include <unordered_map>
#include <unordered_set>

namespace Smp
{
	namespace
	{
		using ArmorBoneCache = std::unordered_map<std::string, ArmorBoneReference>;
		using ArmorNifCache = std::unordered_map<std::string, ArmorBoneCache>;

		// AddMissingBones nodes survive normal armor detach, so the first capture for
		// an asset must remain authoritative for later equips of that same asset.
		ArmorNifCache& GetArmorNifCache()
		{
			static ArmorNifCache cache;
			return cache;
		}

		std::mutex& GetArmorNifCacheLock()
		{
			static std::mutex lock;
			return lock;
		}

		std::string NormalizeNifPath(const std::string_view a_path)
		{
			std::string normalized(a_path);
			std::ranges::transform(normalized, normalized.begin(), [](const unsigned char a_character) {
				return a_character == '/' ? '\\' : static_cast<char>(std::tolower(a_character));
			});
			return normalized;
		}

		std::string NormalizeBoneCacheName(const std::string_view a_name)
		{
			std::string normalized(a_name);
			std::ranges::transform(normalized, normalized.begin(), [](const unsigned char a_character) {
				return static_cast<char>(std::tolower(a_character));
			});
			return normalized;
		}

		template <class Visitor>
		void ForEachSkin(RE::NiAVObject* a_object, Visitor& a_visitor)
		{
			if (!a_object) {
				return;
			}

			if (auto* geometry = a_object->IsGeometry()) {
				if (geometry->skinInstance) {
					a_visitor(geometry->skinInstance.get());
				}
				return;
			}

			auto* node = a_object->IsNode();
			if (!node) {
				return;
			}
			for (auto& child : node->children) {
				ForEachSkin(child.get(), a_visitor);
			}
		}

		ArmorBoneReference* FindMutableArmorBoneReference(
			std::vector<ArmorBoneReference>& a_references,
			const std::string_view a_name)
		{
			const auto found = std::ranges::find_if(a_references, [a_name](const ArmorBoneReference& a_reference) {
				return NormalizeBoneCacheName(a_reference.name) == NormalizeBoneCacheName(a_name);
			});
			return found != a_references.end() ? std::addressof(*found) : nullptr;
		}

		std::vector<ArmorBoneReference> RestoreCachedArmorBoneReferences(
			const ArmorBoneCache& a_cachedBones,
			RE::NiAVObject* a_skeletonRoot)
		{
			std::vector<ArmorBoneReference> pending;
			pending.reserve(a_cachedBones.size());
			std::unordered_set<std::string> allNames;
			for (const auto& [normalizedName, cachedReference] : a_cachedBones) {
				auto reference = cachedReference;
				reference.resolvedNode.reset();
				reference.isArmorOnly = !NiObject::IsActorSkeletonBoneName(a_skeletonRoot, reference.name);
				reference.parentBoneIsArmorOnly = false;
				reference.createdByUs = false;
				allNames.insert(normalizedName);
				pending.push_back(std::move(reference));
			}

			// The per-NIF cache is keyed by name. Reconstruct parent-before-child
			// order because FinalizeArmorSkinBindings consumes the recipe that way.
			std::vector<ArmorBoneReference> references;
			references.reserve(pending.size());
			std::unordered_set<std::string> emittedNames;
			while (!pending.empty()) {
				bool madeProgress = false;
				for (auto it = pending.begin(); it != pending.end();) {
					const auto parentName = NormalizeBoneCacheName(it->parentBoneName);
					if (!parentName.empty() &&
						allNames.contains(parentName) &&
						!emittedNames.contains(parentName)) {
						++it;
						continue;
					}

					emittedNames.insert(NormalizeBoneCacheName(it->name));
					references.push_back(std::move(*it));
					it = pending.erase(it);
					madeProgress = true;
				}
				if (madeProgress) {
					continue;
				}

				// Preserve cyclic or malformed recipes for diagnostics without
				// letting one bad parent relationship wedge the cache restore.
				for (auto& reference : pending) {
					references.push_back(std::move(reference));
				}
				pending.clear();
			}

			for (auto& reference : references) {
				auto* parentReference = reference.parentBoneName.empty() ?
					nullptr :
					FindMutableArmorBoneReference(references, reference.parentBoneName);
				reference.parentBoneIsArmorOnly =
					parentReference && parentReference->isArmorOnly;
			}
			return references;
		}

		using BoundBone = std::pair<std::string, RE::NiNode*>;
		using BoundBones = std::vector<BoundBone>;

		void CollectBoundBonesFromObject(RE::NiAVObject* a_attachedObject, BoundBones& a_boundBones)
		{
			auto collectSkin = [&](RE::BSSkin::Instance* a_skin) {
				if (!a_skin || a_skin->bones.size() > RE::BSSkin::kMaxExpectedBones) {
					return;
				}
				for (auto* boneObject : a_skin->bones) {
					auto* bone = boneObject ? boneObject->IsNode() : nullptr;
					if (!bone) {
						continue;
					}
					const auto existing = std::ranges::find_if(a_boundBones, [bone](const BoundBone& a_entry) {
						return a_entry.second == bone;
					});
					if (existing == a_boundBones.end()) {
						a_boundBones.emplace_back(std::string(bone->GetName()), bone);
					}
				}
			};
			ForEachSkin(a_attachedObject, collectSkin);
		}

		BoundBones CollectBoundBones(
			RE::NiAVObject* a_attachedObject,
			const std::span<const RE::NiPointer<RE::NiAVObject>> a_additionalAttachedObjects)
		{
			BoundBones boundBones;
			CollectBoundBonesFromObject(a_attachedObject, boundBones);
			for (const auto& attachedObject : a_additionalAttachedObjects) {
				if (attachedObject && attachedObject.get() != a_attachedObject) {
					CollectBoundBonesFromObject(attachedObject.get(), boundBones);
				}
			}
			return boundBones;
		}

		RE::NiNode* FindUniqueBoundBone(const BoundBones& a_boundBones, const std::string_view a_name)
		{
			const auto normalizedName = NormalizeBoneCacheName(a_name);
			RE::NiNode* result = nullptr;
			for (const auto& [name, bone] : a_boundBones) {
				if (NormalizeBoneCacheName(name) != normalizedName) {
					continue;
				}
				if (result && result != bone) {
					return nullptr;
				}
				result = bone;
			}
			return result;
		}

		RE::NiNode* FindPersistentSkeletonNode(
			RE::NiAVObject* a_skeletonRoot,
			RE::NiAVObject* a_actorRoot,
			RE::NiAVObject* a_attachedObject,
			const std::string_view a_name)
		{
			auto findInTree = [&](RE::NiAVObject* a_root) -> RE::NiNode* {
				auto* node = NiObject::GetObjectNodeByName(a_root, a_name);
				if (!node || (a_attachedObject && NiObject::IsDescendantOf(node, a_attachedObject))) {
					return nullptr;
				}
				return node;
			};

			if (auto* node = findInTree(a_skeletonRoot)) {
				return node;
			}
			return a_actorRoot != a_skeletonRoot ? findInTree(a_actorRoot) : nullptr;
		}

		RE::NiNode* FindRootBone(
			RE::BSFlattenedBoneTree* a_root,
			RE::NiAVObject* a_retainedFace,
			const std::string_view a_name)
		{
			const RE::BSFixedString boneName{ std::string(a_name) };
			bool deferAttach = false;
			auto* node = Address::BSFlattenedBoneTreeGetOrCreateBoneNode(
				a_root,
				boneName,
				deferAttach);
			if (!node) {
				// Armor-only nodes are not represented in the flattened bone map.
				auto* object = a_root->GetObjectByName(boneName);
				node = object ? object->IsNode() : nullptr;
			}
			return node &&
					(!a_retainedFace || !NiObject::IsDescendantOf(node, a_retainedFace)) ?
				node :
				nullptr;
		}

		void UpdateWorldTransforms(RE::NiAVObject* a_object, RE::NiUpdateData& a_updateData)
		{
			if (!a_object) {
				return;
			}

			a_object->UpdateWorldData(std::addressof(a_updateData));
			auto* node = a_object->IsNode();
			if (!node) {
				return;
			}
			for (auto& child : node->children) {
				UpdateWorldTransforms(child.get(), a_updateData);
			}
		}

		bool SkinBindingsContainBone(
			const std::span<const RetainedSkinBinding> a_bindings,
			const std::string_view a_name)
		{
			const auto normalizedName = NormalizeBoneCacheName(a_name);
			return std::ranges::any_of(a_bindings, [&](const RetainedSkinBinding& a_binding) {
				return std::ranges::any_of(a_binding.boneNames, [&](const RE::BSFixedString& a_boneName) {
					return !a_boneName.empty() &&
						NormalizeBoneCacheName(a_boneName) == normalizedName;
				});
			});
		}

	}

	std::vector<RetainedSkinBinding> CaptureRetainedSkinBindings(RE::NiAVObject* a_object)
	{
		std::vector<RetainedSkinBinding> bindings;
		if (!a_object) {
			return bindings;
		}

		std::unordered_set<RE::BSSkin::Instance*> capturedSkins;
		std::uint32_t boneSlots = 0;
		std::uint32_t unnamedSlots = 0;
		std::uint32_t oversizedSkins = 0;
		auto captureSkin = [&](RE::BSSkin::Instance* a_skin) {
			if (!a_skin || !capturedSkins.insert(a_skin).second) {
				return;
			}
			if (a_skin->bones.size() > RE::BSSkin::kMaxExpectedBones) {
				++oversizedSkins;
				return;
			}

			RetainedSkinBinding binding;
			binding.skin.reset(a_skin);
			binding.boneNames.reserve(a_skin->bones.size());
			for (auto* boneObject : a_skin->bones) {
				if (!boneObject || boneObject->GetName().empty()) {
					binding.boneNames.emplace_back();
					++unnamedSlots;
					continue;
				}

				binding.boneNames.push_back(boneObject->name);
			}
			boneSlots += static_cast<std::uint32_t>(binding.boneNames.size());
			bindings.push_back(std::move(binding));
		};
		ForEachSkin(a_object, captureSkin);

		spdlog::debug(
			"captured source skin bindings object={} name='{}' instances={} boneSlots={} unnamedSlots={} oversizedSkins={}",
			static_cast<void*>(a_object),
			std::string_view(a_object->GetName()),
			bindings.size(),
			boneSlots,
			unnamedSlots,
			oversizedSkins);
		return bindings;
	}

	std::vector<RetainedSkinBinding> CaptureMainSkinBindings(RE::NiAVObject* a_mainRoot)
	{
		return CaptureRetainedSkinBindings(a_mainRoot);
	}

	RetainedSkinRebindResult RebindRetainedSkinBindings(
		RE::BSFlattenedBoneTree* a_root,
		const std::span<const RetainedSkinBinding> a_bindings)
	{
		RetainedSkinRebindResult result;
		if (!a_root) {
			return result;
		}

		std::unordered_set<RE::BSSkin::Instance*> reboundSkins;
		for (const auto& binding : a_bindings) {
			auto* skin = binding.skin.get();
			if (!skin || !reboundSkins.insert(skin).second) {
				continue;
			}

			++result.instances;
			const auto boneCount = skin->bones.size();
			const auto transformCount = skin->worldTransforms.size();
			result.boneSlots += boneCount;
			if (boneCount > RE::BSSkin::kMaxExpectedBones) {
				++result.boneSizeMismatches;
				continue;
			}
			if (binding.boneNames.size() != boneCount) {
				++result.boneSizeMismatches;
			}
			if (transformCount != boneCount) {
				++result.transformSizeMismatches;
			}

			for (std::uint32_t boneIndex = 0; boneIndex < boneCount; ++boneIndex) {
				const auto* boneName = boneIndex < binding.boneNames.size() ?
					std::addressof(binding.boneNames[boneIndex]) :
					nullptr;
				auto* target = boneName && !boneName->empty() ?
					a_root->GetObjectByName(*boneName) :
					nullptr;
				if (target) {
					skin->bones[boneIndex] = target;
					if (boneIndex < transformCount) {
						skin->worldTransforms[boneIndex] = std::addressof(target->world);
					}
					++result.reboundSlots;
					continue;
				}

				++result.unresolvedSlots;
				result.unnamedSlots += !boneName || boneName->empty() ? 1U : 0U;
			}
		}

		return result;
	}

	std::vector<ArmorBoneReference> CaptureArmorBoneReferences(
		RE::NiAVObject* a_modelRoot,
		RE::NiAVObject* a_skeletonRoot,
		const std::string_view a_nifPath,
		const bool a_includeAllNamedNodes)
	{
		std::vector<ArmorBoneReference> references;
		std::unordered_map<std::string, RE::NiNode*> sourceBones;
		auto* modelRoot = a_modelRoot ? a_modelRoot->IsNode() : nullptr;
		const auto normalizedNifPath = NormalizeNifPath(a_nifPath);
		const auto cacheKey = normalizedNifPath + (a_includeAllNamedNodes ? "|all" : "|skin");
		if (!modelRoot || !a_skeletonRoot || normalizedNifPath.empty()) {
			if (modelRoot && !a_skeletonRoot) {
				spdlog::warn(
					"cannot classify armor bones before attach because the actor skeleton root is unavailable modelRoot={} name='{}'",
					static_cast<void*>(modelRoot),
					std::string_view(modelRoot->GetName()));
			} else if (modelRoot && normalizedNifPath.empty()) {
				spdlog::warn(
					"cannot persist armor bone references before attach because the armor NIF path is unavailable modelRoot={} name='{}'",
					static_cast<void*>(modelRoot),
					std::string_view(modelRoot->GetName()));
			}
			return references;
		}

		ArmorBoneCache cachedRecipe;
		{
			std::scoped_lock lock(GetArmorNifCacheLock());
			const auto cachedNif = GetArmorNifCache().find(cacheKey);
			if (cachedNif != GetArmorNifCache().end()) {
				cachedRecipe = cachedNif->second;
			}
		}
		if (!cachedRecipe.empty()) {
			references = RestoreCachedArmorBoneReferences(cachedRecipe, a_skeletonRoot);
			const auto armorOnlyBones = static_cast<std::uint32_t>(
				std::ranges::count_if(references, [](const ArmorBoneReference& a_reference) {
					return a_reference.isArmorOnly;
				}));
			spdlog::debug(
				"restored {} authoritative named armor node references from cache without traversing skin bindings nif='{}' modelRoot={} name='{}' skeletonRoot={} name='{}' armorOnlyBones={}",
				references.size(),
				a_nifPath,
				static_cast<void*>(modelRoot),
				std::string_view(modelRoot->GetName()),
				static_cast<void*>(a_skeletonRoot),
				std::string_view(a_skeletonRoot->GetName()),
				armorOnlyBones);
			return references;
		}

		std::unordered_set<std::string> skinnedBoneNames;
		std::vector<RE::NiNode*> linkedSkinBones;
		auto captureSkinNames = [&](RE::BSSkin::Instance* a_skin) {
			if (!a_skin || a_skin->bones.size() > RE::BSSkin::kMaxExpectedBones) {
				return;
			}

			for (auto* boneObject : a_skin->bones) {
				auto* bone = boneObject ? boneObject->IsNode() : nullptr;
				if (!bone) {
					continue;
				}

				const auto name = bone->GetName();
				if (!name.empty()) {
					const auto normalizedName = NormalizeBoneCacheName(name);
					skinnedBoneNames.insert(normalizedName);
					if (std::ranges::find(linkedSkinBones, bone) == linkedSkinBones.end()) {
						linkedSkinBones.push_back(bone);
					}
				}
			}
		};
		ForEachSkin(a_modelRoot, captureSkinNames);

		std::uint32_t duplicateNamedNodes = 0;
		std::function<void(RE::NiNode*)> captureNamedNodes = [&](RE::NiNode* a_node) {
			if (!a_node) {
				return;
			}

			for (auto& childObject : a_node->children) {
				auto* child = childObject ? childObject->IsNode() : nullptr;
				if (!child) {
					continue;
				}

				const auto name = child->GetName();
				if (!name.empty()) {
					const auto normalizedName = NormalizeBoneCacheName(name);
					if (!FindMutableArmorBoneReference(references, name)) {
						const auto isArmorOnly = !NiObject::IsActorSkeletonBoneName(a_skeletonRoot, name);
						references.push_back({
							.name = std::string(name),
							.isSkinned = skinnedBoneNames.contains(normalizedName),
							.isArmorOnly = isArmorOnly,
						});
						sourceBones.emplace(normalizedName, child);
					} else {
						++duplicateNamedNodes;
					}
				}

				captureNamedNodes(child);
			}
		};
		if (a_includeAllNamedNodes) {
			captureNamedNodes(modelRoot);
		}

		// Walk each linked skin bone's source-NIF parent chain as well as the
		// regular child hierarchy so unskinned intermediate bones are recorded.
		for (auto* linkedBone : linkedSkinBones) {
			std::vector<RE::NiNode*> sourceChain;
			for (auto* bone = linkedBone; bone && bone != modelRoot; bone = bone->parent) {
				if (!bone->GetName().empty()) {
					sourceChain.push_back(bone);
				}
			}

			for (auto chainIt = sourceChain.rbegin(); chainIt != sourceChain.rend(); ++chainIt) {
				auto* bone = *chainIt;
				const auto name = bone->GetName();
				const auto normalizedName = NormalizeBoneCacheName(name);
				auto* reference = FindMutableArmorBoneReference(references, name);
				if (!reference) {
					references.push_back({
						.name = std::string(name),
						.isSkinned = skinnedBoneNames.contains(normalizedName),
						.isArmorOnly = !NiObject::IsActorSkeletonBoneName(a_skeletonRoot, name),
					});
				} else {
					reference->isSkinned = reference->isSkinned || skinnedBoneNames.contains(normalizedName);
				}
				sourceBones.try_emplace(normalizedName, bone);
			}
		}

		for (auto& reference : references) {
			const auto sourceBone = sourceBones.find(NormalizeBoneCacheName(reference.name));
			if (sourceBone == sourceBones.end()) {
				continue;
			}
			auto* bone = sourceBone->second;
			bool foundParent = false;
			for (auto* parent = bone ? bone->parent : nullptr; parent && parent != modelRoot; parent = parent->parent) {
				auto* parentReference = FindMutableArmorBoneReference(references, parent->GetName());
				if (!parentReference) {
					continue;
				}

				reference.parentBoneName = parentReference->name;
				reference.localToParentBone = NiObject::BuildLocalToAncestor(bone, parent);
				reference.parentBoneIsArmorOnly = parentReference->isArmorOnly;
				foundParent = true;
				break;
			}
			if (!foundParent) {
				reference.localToParentBone = NiObject::IsDescendantOf(bone, modelRoot) ?
					NiObject::BuildLocalToAncestor(bone, modelRoot) :
					bone->local;
			}
		}

		std::uint32_t cacheHits = 0;
		std::uint32_t cacheMisses = 0;
		{
			std::scoped_lock lock(GetArmorNifCacheLock());
			auto& cachedBones = GetArmorNifCache()[cacheKey];
			for (auto& reference : references) {
				const auto normalizedBoneName = NormalizeBoneCacheName(reference.name);
				const auto cached = cachedBones.find(normalizedBoneName);
				if (cached != cachedBones.end()) {
					// The NIF supplies the immutable hierarchy recipe, but whether a
					// bone is actor-owned is a property of this destination skeleton.
					const auto isArmorOnlyForDestination = reference.isArmorOnly;
					reference = cached->second;
					reference.resolvedNode.reset();
					reference.isArmorOnly = isArmorOnlyForDestination;
					reference.parentBoneIsArmorOnly = false;
					reference.createdByUs = false;
					++cacheHits;
				} else {
					auto cachedReference = reference;
					cachedReference.resolvedNode.reset();
					cachedReference.isArmorOnly = false;
					cachedReference.parentBoneIsArmorOnly = false;
					cachedReference.createdByUs = false;
					cachedBones.emplace(normalizedBoneName, std::move(cachedReference));
					++cacheMisses;
				}
			}
		}
		const auto armorOnlyBones = static_cast<std::uint32_t>(std::ranges::count_if(references, [](const ArmorBoneReference& a_reference) {
			return a_reference.isArmorOnly;
		}));

		spdlog::debug(
			"resolved {} persistent named armor node references before attach nif='{}' modelRoot={} name='{}' skeletonRoot={} name='{}' skinnedBones={} linkedSkinBones={} armorOnlyBones={} duplicateNames={} cacheHits={} cacheMisses={}",
			references.size(),
			a_nifPath,
			static_cast<void*>(modelRoot),
			std::string_view(modelRoot->GetName()),
			static_cast<void*>(a_skeletonRoot),
			std::string_view(a_skeletonRoot->GetName()),
			skinnedBoneNames.size(),
			linkedSkinBones.size(),
			armorOnlyBones,
			duplicateNamedNodes,
			cacheHits,
			cacheMisses);
		return references;
	}

	std::uint32_t MaterializeMissingFaceBonesForBake(
		RE::NiNode* a_mainRoot,
		RE::NiNode* a_faceBonesRoot)
	{
		if (!a_mainRoot || !a_faceBonesRoot || a_mainRoot == a_faceBonesRoot) {
			return 0;
		}

		std::unordered_set<std::string> faceSkinBoneNames;
		auto captureFaceSkinNames = [&](RE::BSSkin::Instance* a_skin) {
			if (!a_skin || a_skin->bones.size() > RE::BSSkin::kMaxExpectedBones) {
				return;
			}
			for (auto* bone : a_skin->bones) {
				if (bone && !bone->GetName().empty()) {
					faceSkinBoneNames.insert(std::string(std::string_view(bone->GetName())));
				}
			}
		};
		ForEachSkin(a_faceBonesRoot, captureFaceSkinNames);

		std::vector<RE::NiNode*> missingMainBones;
		auto captureMissingMainBones = [&](RE::BSSkin::Instance* a_skin) {
			if (!a_skin || a_skin->bones.size() > RE::BSSkin::kMaxExpectedBones) {
				return;
			}
			for (auto* boneObject : a_skin->bones) {
				auto* bone = boneObject ? boneObject->IsNode() : nullptr;
				if (!bone || bone->GetName().empty()) {
					continue;
				}

				const auto boneName = std::string(std::string_view(bone->GetName()));
				if (faceSkinBoneNames.contains(boneName) ||
					a_faceBonesRoot->GetObjectByName(bone->GetName())) {
					continue;
				}
				if (std::ranges::find(missingMainBones, bone) == missingMainBones.end()) {
					missingMainBones.push_back(bone);
				}
			}
		};
		ForEachSkin(a_mainRoot, captureMissingMainBones);

		std::uint32_t createdBones = 0;
		for (auto* missingBone : missingMainBones) {
			std::vector<RE::NiNode*> sourceChain;
			for (auto* source = missingBone; source && source != a_mainRoot; source = source->parent) {
				if (!source->GetName().empty()) {
					sourceChain.push_back(source);
				}
			}
			std::ranges::reverse(sourceChain);

			auto* targetParent = a_faceBonesRoot;
			RE::NiNode* previousSource = a_mainRoot;
			for (auto* source : sourceChain) {
				if (auto* existingObject = a_faceBonesRoot->GetObjectByName(source->GetName())) {
					if (auto* existingNode = existingObject->IsNode()) {
						targetParent = existingNode;
						previousSource = source;
						continue;
					}
				}

				auto created = RE::make_nismart<RE::NiNode>(0);
				created->name = source->GetName();
				created->local = NiObject::BuildLocalToAncestor(source, previousSource);
				targetParent->AttachChild(created.get(), false);
				targetParent = created.get();
				previousSource = source;
				++createdBones;
			}
		}

		if (createdBones != 0) {
			NiObject::UpdateWorldData(a_faceBonesRoot, true);
			spdlog::debug(
				"materialized {} main-NIF bone nodes into temporary facebones root main={} faceBones={} missingSkinBones={}",
				createdBones,
				static_cast<void*>(a_mainRoot),
				static_cast<void*>(a_faceBonesRoot),
				missingMainBones.size());
		}
		return createdBones;
	}

	std::uint32_t MaterializeRetainedHeadPartBones(
		RE::Actor* a_actor,
		RE::BSFaceGenNiNode* a_retainedFace,
		RE::BSFlattenedBoneTree* a_root,
		std::vector<ArmorBoneReference>& a_references,
		const std::span<const std::string> a_requiredBoneNames)
	{
		if (!a_root || a_references.empty()) {
			return 0;
		}

		std::unordered_set<std::string> requiredNames;
		for (const auto& name : a_requiredBoneNames) {
			if (!name.empty()) {
				requiredNames.insert(NormalizeBoneCacheName(name));
			}
		}
		for (const auto& reference : a_references) {
			if (reference.isSkinned && !reference.name.empty()) {
				requiredNames.insert(NormalizeBoneCacheName(reference.name));
			}
		}

		// A created target is only useful if its complete source hierarchy exists.
		// Expand the XML/skinned set to include every captured parent recipe.
		bool addedParent = true;
		while (addedParent) {
			addedParent = false;
			for (const auto& reference : a_references) {
				if (reference.parentBoneName.empty() ||
					!requiredNames.contains(NormalizeBoneCacheName(reference.name))) {
					continue;
				}
				addedParent =
					requiredNames.insert(NormalizeBoneCacheName(reference.parentBoneName)).second ||
					addedParent;
			}
		}

		auto* retainedFaceObject = reinterpret_cast<RE::NiAVObject*>(a_retainedFace);
		std::uint32_t reusedNodes = 0;
		for (auto& reference : a_references) {
			if (!requiredNames.contains(NormalizeBoneCacheName(reference.name))) {
				reference.resolvedNode.reset();
				reference.parentBoneIsArmorOnly = false;
				reference.createdByUs = false;
				continue;
			}
			if (reference.resolvedNode &&
				NiObject::IsDescendantOf(reference.resolvedNode.get(), a_root) &&
				(!retainedFaceObject ||
					!NiObject::IsDescendantOf(reference.resolvedNode.get(), retainedFaceObject))) {
				++reusedNodes;
				continue;
			}

			reference.resolvedNode.reset();
			reference.parentBoneIsArmorOnly = false;
			reference.createdByUs = false;
			if (auto* existing = FindRootBone(
					a_root,
					retainedFaceObject,
					reference.name)) {
				reference.resolvedNode.reset(existing);
				++reusedNodes;
			}
		}

		std::uint32_t createdNodes = 0;
		std::vector<RE::NiNode*> updateRoots;
		bool madeProgress = true;
		while (madeProgress) {
			madeProgress = false;
			for (auto& reference : a_references) {
				if (reference.resolvedNode ||
					!requiredNames.contains(NormalizeBoneCacheName(reference.name))) {
					continue;
				}
				if (!reference.isArmorOnly) {
					// Default skeleton nodes must only be supplied by BSBoneMap or the
					// flattened tree. Never reconstruct one from an armor/headpart NIF.
					continue;
				}

				auto* parentReference = reference.parentBoneName.empty() ? nullptr :
					FindMutableArmorBoneReference(a_references, reference.parentBoneName);
				RE::NiNode* expectedParent = nullptr;
				if (reference.parentBoneName.empty()) {
					expectedParent = a_root;
				} else if (parentReference && parentReference->resolvedNode) {
					expectedParent = parentReference->resolvedNode.get();
				}
				if (!expectedParent) {
					continue;
				}

				auto createdBone = RE::make_nismart<RE::NiNode>(0);
				createdBone->name = RE::BSFixedString(reference.name);
				createdBone->local = reference.localToParentBone;
				expectedParent->AttachChild(createdBone.get(), false);

				auto* createdBonePointer = createdBone.get();
				reference.resolvedNode = std::move(createdBone);
				reference.parentBoneIsArmorOnly = parentReference && parentReference->isArmorOnly;
				reference.createdByUs = true;
				if (!reference.parentBoneIsArmorOnly) {
					updateRoots.push_back(createdBonePointer);
				}
				++createdNodes;
				madeProgress = true;
				spdlog::debug(
					"materialized retained-head bone '{}' under '{}' before FaceGen skinning actor={} node={} parent={}",
					reference.name,
					expectedParent->GetName(),
					static_cast<void*>(a_actor),
					static_cast<void*>(createdBonePointer),
					static_cast<void*>(expectedParent));
			}
		}

		RE::NiUpdateData updateData;
		updateData.flags = 1U;
		for (auto* updateRoot : updateRoots) {
			UpdateWorldTransforms(updateRoot, updateData);
		}

		std::uint32_t unresolvedRecipes = 0;
		for (const auto& requiredName : requiredNames) {
			const auto* reference = FindMutableArmorBoneReference(a_references, requiredName);
			unresolvedRecipes += !reference || !reference->resolvedNode ? 1U : 0U;
		}
		spdlog::debug(
			"prepared retained headpart skeleton targets actor={} faceNode={} root={} requiredBones={} recipes={} created={} reused={} unresolved={}",
			static_cast<void*>(a_actor),
			static_cast<void*>(a_retainedFace),
			static_cast<void*>(a_root),
			requiredNames.size(),
			a_references.size(),
			createdNodes,
			reusedNodes,
			unresolvedRecipes);
		return createdNodes;
	}

	bool FinalizeArmorSkinBindings(
		RE::Actor* a_actor,
		RE::NiAVObject* a_attachedObject,
		RE::NiNode* a_skeletonRoot,
		const bool a_firstPerson,
		std::vector<ArmorBoneReference>& a_references,
		const std::span<const RE::NiPointer<RE::NiAVObject>> a_additionalAttachedObjects,
		const bool a_moveBoundBonesToSkeleton,
		const std::span<const RetainedSkinBinding> a_sourceSkinBindings)
	{
		if (!a_attachedObject || a_references.empty()) {
			return false;
		}

		auto* actorRoot = a_actor ? a_actor->Get3D(a_firstPerson) : nullptr;
		if (!actorRoot && a_actor) {
			actorRoot = a_actor->Get3D();
		}
		auto* destinationRoot = a_skeletonRoot ? a_skeletonRoot : actorRoot ? actorRoot->IsNode() : nullptr;
		if (!destinationRoot) {
			spdlog::warn(
				"cannot reconcile named armor nodes because the destination skeleton is unavailable actor={} object={} references={}",
				static_cast<void*>(a_actor),
				static_cast<void*>(a_attachedObject),
				a_references.size());
			return false;
		}

		std::vector<RetainedSkinBinding> capturedBindings(
			a_sourceSkinBindings.begin(),
			a_sourceSkinBindings.end());
		if (!a_moveBoundBonesToSkeleton && capturedBindings.empty()) {
			std::unordered_set<RE::BSSkin::Instance*> capturedSkins;
			auto captureSkin = [&](RE::BSSkin::Instance* a_skin) {
				if (!a_skin ||
					a_skin->bones.size() > RE::BSSkin::kMaxExpectedBones ||
					!capturedSkins.insert(a_skin).second) {
					return;
				}

				RetainedSkinBinding binding;
				binding.skin.reset(a_skin);
				binding.boneNames.reserve(a_skin->bones.size());
				for (auto* boneObject : a_skin->bones) {
					binding.boneNames.push_back(
						boneObject ? boneObject->name : RE::BSFixedString{});
				}
				capturedBindings.push_back(std::move(binding));
			};
			ForEachSkin(a_attachedObject, captureSkin);
			for (const auto& attachedObject : a_additionalAttachedObjects) {
				ForEachSkin(attachedObject.get(), captureSkin);
			}
		}

		const auto boundBones = CollectBoundBones(a_attachedObject, a_additionalAttachedObjects);
		const auto attachedArmorBones = static_cast<std::uint32_t>(std::ranges::count_if(boundBones, [&](const BoundBone& a_entry) {
			auto* reference = FindMutableArmorBoneReference(a_references, a_entry.first);
			return reference && reference->isArmorOnly;
		}));

		for (auto& reference : a_references) {
			reference.resolvedNode.reset();
			reference.createdByUs = false;
		}

		std::uint32_t createdArmorNodes = 0;
		std::uint32_t reusedArmorNodes = 0;
		std::uint32_t reparentedArmorBones = 0;
		for (auto& reference : a_references) {
			if (!reference.isArmorOnly) {
				auto* bone = NiObject::ResolveActorSkeletonBoneNode(destinationRoot, reference.name);
				if (!bone && actorRoot != destinationRoot) {
					bone = NiObject::ResolveActorSkeletonBoneNode(actorRoot, reference.name);
				}
				if (!bone) {
					bone = FindPersistentSkeletonNode(
						destinationRoot,
						actorRoot,
						a_attachedObject,
						reference.name);
				}
				if (!bone) {
					bone = FindUniqueBoundBone(boundBones, reference.name);
				}
				if (bone) {
					reference.resolvedNode.reset(bone);
				} else {
					spdlog::warn(
						"could not resolve actor-owned skeleton bone '{}' actor={} object={}",
						reference.name,
						static_cast<void*>(a_actor),
						static_cast<void*>(a_attachedObject));
				}
				continue;
			}

			auto* parentReference = reference.parentBoneName.empty() ? nullptr :
				FindMutableArmorBoneReference(a_references, reference.parentBoneName);
			auto* expectedParent = parentReference ? parentReference->resolvedNode.get() : destinationRoot;
			if (!expectedParent) {
				spdlog::warn(
					"cannot reconcile armor node '{}' because source parent '{}' was not resolved actor={} object={}",
					reference.name,
					reference.parentBoneName,
					static_cast<void*>(a_actor),
					static_cast<void*>(a_attachedObject));
				continue;
			}
			reference.parentBoneIsArmorOnly = parentReference && parentReference->isArmorOnly;

			auto* boundBone = FindUniqueBoundBone(boundBones, reference.name);
			auto* persistentBone = FindPersistentSkeletonNode(
				destinationRoot,
				actorRoot,
				a_attachedObject,
				reference.name);
			auto* bone = a_moveBoundBonesToSkeleton ?
				(boundBone ? boundBone : persistentBone) :
				persistentBone;

			if (!bone) {
				const auto mainSkinRequiresBone =
					reference.isArmorOnly &&
					SkinBindingsContainBone(capturedBindings, reference.name);
				if (reference.isSkinned &&
					a_moveBoundBonesToSkeleton &&
					!mainSkinRequiresBone) {
					spdlog::debug(
						"skipping unresolved skinned armor node '{}' actor={} object={} because only the engine-owned BSSkin bone may be used",
						reference.name,
						static_cast<void*>(a_actor),
						static_cast<void*>(a_attachedObject));
					continue;
				}
				auto createdBone = RE::make_nismart<RE::NiNode>(0);
				createdBone->name = RE::BSFixedString(reference.name);
				createdBone->local = reference.localToParentBone;
				expectedParent->AttachChild(createdBone.get(), false);
				bone = createdBone.get();
				reference.resolvedNode = std::move(createdBone);
				reference.createdByUs = true;
				++createdArmorNodes;
				spdlog::debug(
					"created persistent armor node '{}' under '{}' actor={} node={} parent={} sourceSkinned={} requiredByCapturedMainSkin={}",
					reference.name,
					expectedParent->GetName(),
					static_cast<void*>(a_actor),
					static_cast<void*>(bone),
					static_cast<void*>(expectedParent),
					reference.isSkinned,
					mainSkinRequiresBone);
			} else {
				reference.resolvedNode.reset(bone);
				if (reference.isArmorOnly) {
					++reusedArmorNodes;
				}
			}

			if (a_moveBoundBonesToSkeleton &&
				bone != expectedParent &&
				bone->parent != expectedParent &&
				!NiObject::IsDescendantOf(expectedParent, bone)) {
				RE::NiPointer<RE::NiAVObject> keepAlive{ bone };
				if (bone->parent) {
					bone->parent->DetachChild(bone);
				}
				expectedParent->AttachChild(bone, false);
				++reparentedArmorBones;
				spdlog::debug(
					"restored armor-only node '{}' under source parent '{}' actor={} bone={} parent={}",
					reference.name,
					expectedParent->GetName(),
					static_cast<void*>(a_actor),
					static_cast<void*>(bone),
					static_cast<void*>(expectedParent));
			}

			bone->local = reference.localToParentBone;
		}

		std::vector<RE::NiNode*> updateRoots;
		for (const auto& reference : a_references) {
			auto* bone = reference.resolvedNode.get();
			if (!reference.isArmorOnly || reference.parentBoneIsArmorOnly || !bone) {
				continue;
			}
			if (std::ranges::find(updateRoots, bone) == updateRoots.end()) {
				updateRoots.push_back(bone);
			}
		}
		RE::NiUpdateData updateData;
		updateData.flags = 1U;
		for (auto* updateRoot : updateRoots) {
			UpdateWorldTransforms(updateRoot, updateData);
		}

		std::uint32_t reboundSkinSlots = 0;
		std::uint32_t unresolvedSkinSlots = 0;
		for (const auto& binding : capturedBindings) {
			auto* skin = binding.skin.get();
			if (!skin ||
				skin->bones.size() != binding.boneNames.size() ||
				skin->worldTransforms.size() != skin->bones.size()) {
				unresolvedSkinSlots += skin ?
					static_cast<std::uint32_t>(skin->bones.size()) :
					static_cast<std::uint32_t>(binding.boneNames.size());
				continue;
			}

			for (std::uint32_t index = 0; index < skin->bones.size(); ++index) {
				const auto& boneName = binding.boneNames[index];
				auto* reference = boneName.empty() ?
					nullptr :
					FindMutableArmorBoneReference(a_references, boneName);
				auto* target = reference ? reference->resolvedNode.get() : nullptr;
				if (!target) {
					++unresolvedSkinSlots;
					continue;
				}

				skin->bones[index] = target;
				skin->worldTransforms[index] = std::addressof(target->world);
				++reboundSkinSlots;
			}
		}

		auto* currentActorRoot = actorRoot ? actorRoot : static_cast<RE::NiAVObject*>(destinationRoot);

		spdlog::debug(
			"finalized persistent armor hierarchy actor={} object={} skeletonRoot={} actorRoot={} references={} attachedArmorOnlyBones={} createdArmorNodes={} reusedArmorNodes={} reparentedArmorBones={} updatedArmorRoots={} sourceSkinBindings={} reboundSkinSlots={} unresolvedSkinSlots={} movedBoundBones={}",
			static_cast<void*>(a_actor),
			static_cast<void*>(a_attachedObject),
			static_cast<void*>(destinationRoot),
			static_cast<void*>(currentActorRoot),
			a_references.size(),
			attachedArmorBones,
			createdArmorNodes,
			reusedArmorNodes,
			reparentedArmorBones,
			updateRoots.size(),
			a_sourceSkinBindings.size(),
			reboundSkinSlots,
			unresolvedSkinSlots,
			a_moveBoundBonesToSkeleton);
		return a_moveBoundBonesToSkeleton || unresolvedSkinSlots == 0;
	}
}
