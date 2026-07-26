#include "ArmorBoneReference.h"

#include "BSSkin.h"
#include "Fo4NiObjectUtils.h"
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

		using BoundBone = std::pair<std::string, RE::NiNode*>;
		using BoundBones = std::vector<BoundBone>;

		BoundBones CollectBoundBones(RE::NiAVObject* a_attachedObject)
		{
			BoundBones boundBones;
			auto collectSkin = [&](RE::BSSkin::Instance* a_skin) {
				if (!a_skin || a_skin->bones.size() > RE::BSSkin::kMaxExpectedBones) {
					return;
				}
				for (auto* boneObject : a_skin->bones) {
					auto* bone = boneObject ? boneObject->IsNode() : nullptr;
					if (!bone) {
						continue;
					}
					const auto existing = std::ranges::find_if(boundBones, [bone](const BoundBone& a_entry) {
						return a_entry.second == bone;
					});
					if (existing == boundBones.end()) {
						boundBones.emplace_back(std::string(bone->GetName()), bone);
					}
				}
			};
			ForEachSkin(a_attachedObject, collectSkin);
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

	}

	std::vector<ArmorBoneReference> CaptureArmorBoneReferences(
		RE::NiAVObject* a_modelRoot,
		RE::NiAVObject* a_skeletonRoot,
		const std::string_view a_nifPath)
	{
		std::vector<ArmorBoneReference> references;
		std::unordered_map<std::string, RE::NiNode*> sourceBones;
		auto* modelRoot = a_modelRoot ? a_modelRoot->IsNode() : nullptr;
		const auto normalizedNifPath = NormalizeNifPath(a_nifPath);
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
						const auto isArmorOnly = a_skeletonRoot->GetObjectByName(RE::BSFixedString(name)) == nullptr;
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
		captureNamedNodes(modelRoot);

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
						.isArmorOnly = a_skeletonRoot->GetObjectByName(RE::BSFixedString(name)) == nullptr,
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
			auto& cachedBones = GetArmorNifCache()[normalizedNifPath];
			for (auto& reference : references) {
				const auto normalizedBoneName = NormalizeBoneCacheName(reference.name);
				const auto cached = cachedBones.find(normalizedBoneName);
				if (cached != cachedBones.end()) {
					reference = cached->second;
					++cacheHits;
				} else {
					auto cachedReference = reference;
					cachedReference.resolvedNode.reset();
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

	void FinalizeArmorSkinBindings(
		RE::Actor* a_actor,
		RE::NiAVObject* a_attachedObject,
		RE::NiNode* a_skeletonRoot,
		const bool a_firstPerson,
		std::vector<ArmorBoneReference>& a_references)
	{
		if (!a_attachedObject || a_references.empty()) {
			return;
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
			return;
		}

		const auto boundBones = CollectBoundBones(a_attachedObject);
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
			auto* persistentBone = FindPersistentSkeletonNode(destinationRoot, actorRoot, a_attachedObject, reference.name);
			auto* bone = reference.isArmorOnly ?
				(boundBone ? boundBone : persistentBone) :
				(persistentBone ? persistentBone : boundBone);

			if (!bone) {
				auto createdBone = RE::make_nismart<RE::NiNode>(0);
				createdBone->name = RE::BSFixedString(reference.name);
				createdBone->local = reference.localToParentBone;
				expectedParent->AttachChild(createdBone.get(), false);
				bone = createdBone.get();
				reference.resolvedNode = std::move(createdBone);
				reference.isArmorOnly = true;
				reference.createdByUs = true;
				++createdArmorNodes;
				spdlog::debug(
					"created persistent unskinned armor node '{}' under '{}' actor={} node={} parent={}",
					reference.name,
					expectedParent->GetName(),
					static_cast<void*>(a_actor),
					static_cast<void*>(bone),
					static_cast<void*>(expectedParent));
			} else {
				reference.resolvedNode.reset(bone);
				if (reference.isArmorOnly) {
					++reusedArmorNodes;
				}
			}

			if (!reference.isArmorOnly) {
				continue;
			}

			if (bone != expectedParent &&
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

		auto* currentActorRoot = actorRoot ? actorRoot : static_cast<RE::NiAVObject*>(destinationRoot);

		spdlog::debug(
			"finalized persistent armor hierarchy actor={} object={} skeletonRoot={} actorRoot={} references={} attachedArmorOnlyBones={} createdArmorNodes={} reusedArmorNodes={} reparentedArmorNodes={} updatedArmorRoots={}",
			static_cast<void*>(a_actor),
			static_cast<void*>(a_attachedObject),
			static_cast<void*>(destinationRoot),
			static_cast<void*>(currentActorRoot),
			a_references.size(),
			attachedArmorBones,
			createdArmorNodes,
			reusedArmorNodes,
			reparentedArmorBones,
			updateRoots.size());
	}
}
