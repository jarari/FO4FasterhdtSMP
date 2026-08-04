#include "Fo4NiObjectUtils.h"

#include "Address.h"
#include "ConfigPaths.h"
#include "PhysicsName.h"
#include "RE/B/BSFixedString.h"
#include "RE/B/BSFlattenedBoneTree.h"
#include "RE/B/BSTHashMap.h"
#include "RE/N/NiAVObject.h"
#include "RE/N/NiExtraData.h"
#include "RE/N/NiNode.h"
#include "RE/N/NiStringExtraData.h"
#include "RE/N/NiUpdateData.h"

namespace Smp::NiObject
{
	namespace
	{
		using BoneMap = RE::BSTHashMap<RE::BSFixedString, RE::NiAVObject*>;

		BoneMap* FindDefaultBoneMapInTree(RE::NiAVObject* a_object)
		{
			if (!a_object) {
				return nullptr;
			}
			static const RE::BSFixedString boneMapExtraName{ "BOM" };
			if (auto* extra = a_object->GetExtraData(boneMapExtraName)) {
				return reinterpret_cast<BoneMap*>(
					reinterpret_cast<std::byte*>(extra) + sizeof(RE::NiExtraData));
			}

			auto* node = a_object->IsNode();
			if (!node) {
				return nullptr;
			}
			for (auto& child : node->children) {
				if (auto* bones = FindDefaultBoneMapInTree(child.get())) {
					return bones;
				}
			}
			return nullptr;
		}

		BoneMap* FindDefaultBoneMap(RE::NiAVObject* a_root)
		{
			static const RE::BSFixedString boneMapExtraName{ "BOM" };
			auto* treeRoot = a_root;
			for (auto* current = a_root; current; current = current->parent) {
				treeRoot = current;
				if (auto* extra = current->GetExtraData(boneMapExtraName)) {
					return reinterpret_cast<BoneMap*>(
						reinterpret_cast<std::byte*>(extra) + sizeof(RE::NiExtraData));
				}
			}
			return FindDefaultBoneMapInTree(treeRoot);
		}
	}

	RE::BSFlattenedBoneTree* FindFlattenedBoneTree(RE::NiAVObject* a_root)
	{
		return Address::BSFlattenedBoneTreeFind(a_root, 1);
	}

	RE::BSFlattenedBoneTree::FlattenedBone* FindFlattenedBoneByName(
		RE::BSFlattenedBoneTree* a_tree,
		const std::string_view a_name)
	{
		if (!a_tree || !a_tree->bone || a_name.empty()) {
			return nullptr;
		}

		const auto found = a_tree->boneMap.find(RE::BSFixedString(std::string(a_name)));
		if (found == a_tree->boneMap.end()) {
			return nullptr;
		}

		const auto index = found->second;
		return index >= 0 && index < a_tree->boneCountExpanded ?
			std::addressof(a_tree->bone[index]) :
			nullptr;
	}

	bool IsActorSkeletonBoneName(RE::NiAVObject* a_root, const std::string_view a_name)
	{
		if (!a_root || a_name.empty()) {
			return false;
		}
		if (FindFlattenedBoneByName(FindFlattenedBoneTree(a_root), a_name)) {
			return true;
		}

		// BSBoneMap is the immutable name-to-node map created from skeleton.nif.
		// It includes default deferred/skin nodes that are not guaranteed to have
		// an entry in BSFlattenedBoneTree::boneMap. Armor nodes attached later are
		// never added to BOM, which makes it the correct ownership boundary.
		const RE::BSFixedString boneName{ std::string(a_name) };
		if (auto* bones = FindDefaultBoneMap(a_root)) {
			return bones->find(boneName) != bones->end();
		}
		return false;
	}

	RE::NiNode* ResolveActorSkeletonBoneNode(RE::NiAVObject* a_root, const std::string_view a_name)
	{
		if (!a_root || a_name.empty()) {
			return nullptr;
		}

		const RE::BSFixedString boneName{ std::string(a_name) };
		if (auto* flattened = FindFlattenedBoneTree(a_root);
			FindFlattenedBoneByName(flattened, a_name)) {
			bool deferredAttach = false;
			if (auto* node = Address::BSFlattenedBoneTreeGetOrCreateBoneNode(
					flattened,
					boneName,
					deferredAttach)) {
				return node;
			}
		}

		if (auto* bones = FindDefaultBoneMap(a_root)) {
			const auto found = bones->find(boneName);
			return found != bones->end() && found->second ? found->second->IsNode() : nullptr;
		}
		return nullptr;
	}

	std::optional<std::string> FindStringExtraData(RE::NiAVObject* a_object, const std::string_view a_name)
	{
		if (!a_object || !a_object->extra) {
			return std::nullopt;
		}

		for (auto* extra : *a_object->extra) {
			auto* stringExtra = netimmerse_cast<RE::NiStringExtraData*>(extra);
			if (!stringExtra) {
				continue;
			}

			const std::string_view name(stringExtra->name);
			auto data = ConfigPaths::Trim(std::string(std::string_view(stringExtra->data)));
			if (data.empty() || !PhysicsNamesEqual(name, a_name)) {
				continue;
			}

			return data;
		}

		return std::nullopt;
	}

	void CollectNodePointers(RE::NiAVObject* a_root, std::vector<RE::NiAVObject*>& a_nodes)
	{
		if (!a_root) {
			return;
		}

		a_nodes.push_back(a_root);

		auto* node = a_root->IsNode();
		if (!node) {
			return;
		}

		for (auto& child : node->children) {
			CollectNodePointers(child.get(), a_nodes);
		}
	}

	namespace
	{
		bool IsExcludedNodePointer(RE::NiAVObject* a_object, const std::unordered_set<RE::NiAVObject*>& a_exclusions)
		{
			return a_object && a_exclusions.contains(a_object);
		}
	}

	void CollectNodePointersWithInheritedExclusions(
		RE::NiAVObject* a_root,
		const std::unordered_set<RE::NiAVObject*>& a_explicitExclusions,
		std::unordered_set<RE::NiAVObject*>& a_inheritedExclusions,
		std::vector<RE::NiAVObject*>& a_nodes)
	{
		if (!a_root) {
			return;
		}

		const auto parentExcluded = a_root->parent && IsExcludedNodePointer(a_root->parent, a_inheritedExclusions);
		const auto excluded = IsExcludedNodePointer(a_root, a_explicitExclusions) || parentExcluded;
		if (excluded) {
			a_inheritedExclusions.insert(a_root);
		} else {
			a_nodes.push_back(a_root);
		}

		auto* node = a_root->IsNode();
		if (!node) {
			return;
		}

		for (auto& child : node->children) {
			CollectNodePointersWithInheritedExclusions(child.get(), a_explicitExclusions, a_inheritedExclusions, a_nodes);
		}
	}

	bool IsDescendantOf(RE::NiAVObject* a_object, RE::NiAVObject* a_ancestor)
	{
		if (!a_object || !a_ancestor) {
			return false;
		}

		for (auto* current = a_object; current; current = current->parent) {
			if (current == a_ancestor) {
				return true;
			}
		}
		return false;
	}

	void UpdateWorldData(RE::NiAVObject* a_object, const bool a_dirty)
	{
		if (!a_object) {
			return;
		}

		RE::NiUpdateData updateData;
		updateData.flags = a_dirty ? 1U : 0U;
		a_object->UpdateWorldData(std::addressof(updateData));

		auto* node = a_object->IsNode();
		if (!node) {
			return;
		}

		for (auto& child : node->children) {
			UpdateWorldData(child.get(), a_dirty);
		}
	}

	RE::NiTransform BuildLocalToAncestor(RE::NiNode* a_node, RE::NiNode* a_ancestor)
	{
		if (!a_node) {
			return RE::NiTransform::IDENTITY;
		}

		auto localToAncestor = a_node->local;
		for (auto* parent = a_node->parent; parent && parent != a_ancestor; parent = parent->parent) {
			localToAncestor = parent->local * localToAncestor;
		}
		return localToAncestor;
	}

	RE::NiNode* GetObjectNodeByName(RE::NiAVObject* a_root, const std::string_view a_name)
	{
		if (!a_root || a_name.empty()) {
			return nullptr;
		}

		auto* object = a_root->GetObjectByName(RE::BSFixedString(std::string(a_name)));
		return object ? object->IsNode() : nullptr;
	}

}
