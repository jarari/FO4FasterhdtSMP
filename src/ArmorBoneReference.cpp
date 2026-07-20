#include "ArmorBoneReference.h"

#include "BSBoneMap.h"
#include "BSSkin.h"
#include "Fo4NiObjectUtils.h"
#include "RE/B/BSGeometry.h"
#include "RE/N/NiAVObject.h"
#include "RE/N/NiNode.h"
#include "RE/T/TESObjectREFR.h"

#include <cctype>
#include <unordered_map>

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

		void UpdateWorldFromLocal(RE::NiAVObject* a_object)
		{
			if (!a_object) {
				return;
			}

			a_object->world = a_object->parent ? a_object->parent->world * a_object->local : a_object->local;
			if (auto* node = a_object->IsNode()) {
				for (auto& child : node->children) {
					UpdateWorldFromLocal(child.get());
				}
			}
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

		std::uint32_t RestoreCachedArmorLocals(
			const BoundBones& a_boundBones,
			const std::vector<ArmorBoneReference>& a_references)
		{
			std::vector<RE::NiNode*> updateRoots;
			std::uint32_t restored = 0;
			for (const auto& reference : a_references) {
				if (!reference.isArmorOnly || reference.parentBoneName.empty()) {
					continue;
				}

				const auto normalizedBoneName = NormalizeBoneCacheName(reference.name);
				const auto normalizedParentName = NormalizeBoneCacheName(reference.parentBoneName);
				for (const auto& [name, bone] : a_boundBones) {
					if (!bone || NormalizeBoneCacheName(name) != normalizedBoneName) {
						continue;
					}
					if (!bone->parent || NormalizeBoneCacheName(bone->parent->GetName()) != normalizedParentName) {
						spdlog::warn(
							"could not restore armor-only bone '{}' local pose because its bound parent is '{}' instead of '{}' bone={} parent={}",
							reference.name,
							bone->parent ? std::string_view(bone->parent->GetName()) : std::string_view{},
							reference.parentBoneName,
							static_cast<void*>(bone),
							static_cast<void*>(bone->parent));
						continue;
					}

					bone->local = reference.localToParentBone;
					++restored;
					if (!reference.parentBoneIsArmorOnly && std::ranges::find(updateRoots, bone) == updateRoots.end()) {
						updateRoots.push_back(bone);
					}
				}
			}

			for (auto* root : updateRoots) {
				UpdateWorldFromLocal(root);
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

		auto captureSkin = [&](RE::BSSkin::Instance* a_skin) {
			if (!a_skin || a_skin->bones.size() > RE::BSSkin::kMaxExpectedBones) {
				return;
			}

			for (auto* boneObject : a_skin->bones) {
				auto* bone = boneObject ? boneObject->IsNode() : nullptr;
				if (!bone || !NiObject::IsDescendantOf(bone, modelRoot)) {
					continue;
				}

				const auto name = bone->GetName();
				if (name.empty() || FindMutableArmorBoneReference(references, name)) {
					continue;
				}

				const auto isArmorOnly = a_skeletonRoot->GetObjectByName(RE::BSFixedString(name)) == nullptr;
				references.push_back({
					.name = std::string(name),
					.isArmorOnly = isArmorOnly,
				});
				sourceBones.emplace(NormalizeBoneCacheName(name), bone);
			}
		};
		ForEachSkin(a_modelRoot, captureSkin);

		for (auto& reference : references) {
			const auto sourceBone = sourceBones.find(NormalizeBoneCacheName(reference.name));
			if (sourceBone == sourceBones.end()) {
				continue;
			}
			auto* bone = sourceBone->second;
			for (auto* parent = bone ? bone->parent : nullptr; parent && parent != modelRoot; parent = parent->parent) {
				auto* parentReference = FindMutableArmorBoneReference(references, parent->GetName());
				if (!parentReference) {
					continue;
				}

				reference.parentBoneName = parentReference->name;
				reference.localToParentBone = NiObject::BuildLocalToAncestor(bone, parent);
				reference.parentBoneIsArmorOnly = parentReference->isArmorOnly;
				break;
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
					cachedBones.emplace(normalizedBoneName, reference);
					++cacheMisses;
				}
			}
		}
		const auto armorOnlyBones = static_cast<std::uint32_t>(std::ranges::count_if(references, [](const ArmorBoneReference& a_reference) {
			return a_reference.isArmorOnly;
		}));

		spdlog::debug(
			"resolved {} persistent armor skin bone references before attach nif='{}' modelRoot={} name='{}' skeletonRoot={} name='{}' armorOnlyBones={} cacheHits={} cacheMisses={}",
			references.size(),
			a_nifPath,
			static_cast<void*>(modelRoot),
			std::string_view(modelRoot->GetName()),
			static_cast<void*>(a_skeletonRoot),
			std::string_view(a_skeletonRoot->GetName()),
			armorOnlyBones,
			cacheHits,
			cacheMisses);
		return references;
	}

	void FinalizeArmorSkinBindings(
		RE::Actor* a_actor,
		RE::NiAVObject* a_attachedObject,
		const bool a_firstPerson,
		std::vector<ArmorBoneReference>& a_references)
	{
		if (!a_attachedObject) {
			return;
		}

		auto* actorRoot = a_actor ? a_actor->Get3D(a_firstPerson) : nullptr;
		const auto boundBones = CollectBoundBones(a_attachedObject);
		const auto attachedArmorBones = static_cast<std::uint32_t>(std::ranges::count_if(boundBones, [&](const BoundBone& a_entry) {
			auto* reference = FindMutableArmorBoneReference(a_references, a_entry.first);
			return reference && reference->isArmorOnly;
		}));

		std::uint32_t reparentedArmorBones = 0;
		for (const auto& reference : a_references) {
			if (!reference.isArmorOnly ||
				reference.parentBoneName.empty() ||
				reference.parentBoneIsArmorOnly) {
				continue;
			}

			auto* expectedParent = FindUniqueBoundBone(boundBones, reference.parentBoneName);
			if (!expectedParent ||
				!actorRoot ||
				!NiObject::IsDescendantOf(expectedParent, actorRoot)) {
				spdlog::warn(
					"could not resolve source actor parent '{}' for armor-only bone '{}' actor={} parent={}",
					reference.parentBoneName,
					reference.name,
					static_cast<void*>(a_actor),
					static_cast<void*>(expectedParent));
				continue;
			}

			const auto normalizedBoneName = NormalizeBoneCacheName(reference.name);
			for (const auto& [name, bone] : boundBones) {
				if (!bone || NormalizeBoneCacheName(name) != normalizedBoneName ||
					bone == expectedParent || NiObject::IsDescendantOf(expectedParent, bone)) {
					continue;
				}

				RE::NiPointer<RE::NiAVObject> keepAlive{ bone };
				if (bone->parent != expectedParent) {
					if (bone->parent) {
						bone->parent->DetachChild(bone);
					}
					expectedParent->AttachChild(bone, false);
				}
				++reparentedArmorBones;
				spdlog::debug(
					"restored armor-only bone '{}' under source actor parent '{}' actor={} bone={} parent={}",
					reference.name,
					reference.parentBoneName,
					static_cast<void*>(a_actor),
					static_cast<void*>(bone),
					static_cast<void*>(expectedParent));
			}
		}
		const auto restoredArmorBoneNodes = RestoreCachedArmorLocals(boundBones, a_references);

		if ((attachedArmorBones > 0 || reparentedArmorBones > 0) && actorRoot) {
			RefreshBoneScatterTable(actorRoot);
		}

		spdlog::debug(
			"finalized vanilla armor skin bindings actor={} object={} references={} attachedArmorOnlyBones={} reparentedArmorOnlyRoots={} restoredArmorOnlyNodes={}",
			static_cast<void*>(a_actor),
			static_cast<void*>(a_attachedObject),
			a_references.size(),
			attachedArmorBones,
			reparentedArmorBones,
			restoredArmorBoneNodes);
	}

	void RestoreArmorBoneLocalPose(
		RE::NiAVObject* a_attachedObject,
		const std::vector<ArmorBoneReference>& a_references)
	{
		if (!a_attachedObject || a_references.empty()) {
			return;
		}

		const auto restored = RestoreCachedArmorLocals(CollectBoundBones(a_attachedObject), a_references);
		spdlog::debug(
			"restored {} cached armor-only local transforms before physics build object={} references={}",
			restored,
			static_cast<void*>(a_attachedObject),
			a_references.size());
	}
}
