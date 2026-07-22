#include "ArmorBoneReference.h"

#include "BSSkin.h"
#include "Fo4NiObjectUtils.h"
#include "RE/B/BSGeometry.h"
#include "RE/N/NiAVObject.h"
#include "RE/N/NiNode.h"
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

		const ArmorBoneReference* FindArmorBoneReference(
			const std::vector<ArmorBoneReference>& a_references,
			const std::string_view a_name)
		{
			const auto found = std::ranges::find_if(a_references, [a_name](const ArmorBoneReference& a_reference) {
				return NormalizeBoneCacheName(a_reference.name) == NormalizeBoneCacheName(a_name);
			});
			return found != a_references.end() ? std::addressof(*found) : nullptr;
		}

		bool IsObjectInTree(RE::NiAVObject* a_root, const RE::NiAVObject* a_target)
		{
			if (!a_root || !a_target) {
				return false;
			}
			if (a_root == a_target) {
				return true;
			}
			auto* node = a_root->IsNode();
			if (!node) {
				return false;
			}
			for (auto& child : node->children) {
				if (IsObjectInTree(child.get(), a_target)) {
					return true;
				}
			}
			return false;
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

		std::uint32_t RestoreCachedArmorLocals(
			const std::vector<ArmorBoneReference>& a_references,
			RE::NiAVObject* a_currentActorRoot = nullptr)
		{
			std::uint32_t restored = 0;
			for (const auto& reference : a_references) {
				auto* bone = reference.resolvedNode.get();
				if (!reference.isArmorOnly || !bone || (a_currentActorRoot && !IsObjectInTree(a_currentActorRoot, bone))) {
					continue;
				}

				if (!reference.parentBoneName.empty()) {
					const auto* parentReference = FindArmorBoneReference(a_references, reference.parentBoneName);
					if (!parentReference || bone->parent != parentReference->resolvedNode.get()) {
						continue;
					}
				}

				bone->local = reference.localToParentBone;
				bone->world = bone->parent ? bone->parent->world * bone->local : bone->local;
				++restored;
			}
			return restored;
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
		auto captureSkinNames = [&](RE::BSSkin::Instance* a_skin) {
			if (!a_skin || a_skin->bones.size() > RE::BSSkin::kMaxExpectedBones) {
				return;
			}

			for (auto* boneObject : a_skin->bones) {
				auto* bone = boneObject ? boneObject->IsNode() : nullptr;
				if (!bone || !NiObject::IsDescendantOf(bone, modelRoot)) {
					continue;
				}

				const auto name = bone->GetName();
				if (!name.empty()) {
					skinnedBoneNames.insert(NormalizeBoneCacheName(name));
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
				reference.localToParentBone = NiObject::BuildLocalToAncestor(bone, modelRoot);
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
			"resolved {} persistent named armor node references before attach nif='{}' modelRoot={} name='{}' skeletonRoot={} name='{}' skinnedBones={} armorOnlyBones={} duplicateNames={} cacheHits={} cacheMisses={}",
			references.size(),
			a_nifPath,
			static_cast<void*>(modelRoot),
			std::string_view(modelRoot->GetName()),
			static_cast<void*>(a_skeletonRoot),
			std::string_view(a_skeletonRoot->GetName()),
			skinnedBoneNames.size(),
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
				createdBone->world = expectedParent->world * createdBone->local;
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
			bone->world = bone->parent ? bone->parent->world * bone->local : bone->local;
		}
		auto* currentActorRoot = actorRoot ? actorRoot : static_cast<RE::NiAVObject*>(destinationRoot);
		const auto restoredArmorBoneNodes = RestoreCachedArmorLocals(a_references, currentActorRoot);

		spdlog::debug(
			"finalized persistent armor hierarchy actor={} object={} skeletonRoot={} actorRoot={} references={} attachedArmorOnlyBones={} createdArmorNodes={} reusedArmorNodes={} reparentedArmorNodes={} restoredArmorOnlyNodes={}",
			static_cast<void*>(a_actor),
			static_cast<void*>(a_attachedObject),
			static_cast<void*>(destinationRoot),
			static_cast<void*>(currentActorRoot),
			a_references.size(),
			attachedArmorBones,
			createdArmorNodes,
			reusedArmorNodes,
			reparentedArmorBones,
			restoredArmorBoneNodes);
	}

	void RestoreArmorBoneLocalPose(
		RE::NiAVObject* a_attachedObject,
		RE::NiAVObject* a_actorRoot,
		const std::vector<ArmorBoneReference>& a_references)
	{
		if (!a_attachedObject || !a_actorRoot || a_references.empty()) {
			return;
		}

		const auto restored = RestoreCachedArmorLocals(a_references, a_actorRoot);
		spdlog::debug(
			"restored {} cached armor-only local transforms before physics build object={} actorRoot={} references={}",
			restored,
			static_cast<void*>(a_attachedObject),
			static_cast<void*>(a_actorRoot),
			a_references.size());
	}
}
