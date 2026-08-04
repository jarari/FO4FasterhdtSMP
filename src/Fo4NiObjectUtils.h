#pragma once

#include "RE/B/BSFlattenedBoneTree.h"
#include "RE/N/NiTransform.h"

#include <optional>
#include <string>
#include <string_view>
#include <unordered_set>
#include <vector>

namespace RE
{
	class NiAVObject;
	class NiNode;
}

namespace Smp::NiObject
{
	RE::BSFlattenedBoneTree* FindFlattenedBoneTree(RE::NiAVObject* a_root);
	RE::BSFlattenedBoneTree::FlattenedBone* FindFlattenedBoneByName(
		RE::BSFlattenedBoneTree* a_tree,
		std::string_view a_name);
	bool IsActorSkeletonBoneName(RE::NiAVObject* a_root, std::string_view a_name);
	RE::NiNode* ResolveActorSkeletonBoneNode(RE::NiAVObject* a_root, std::string_view a_name);
	std::optional<std::string> FindStringExtraData(RE::NiAVObject* a_object, std::string_view a_name);
	void CollectNodePointers(RE::NiAVObject* a_root, std::vector<RE::NiAVObject*>& a_nodes);
	void CollectNodePointersWithInheritedExclusions(
		RE::NiAVObject* a_root,
		const std::unordered_set<RE::NiAVObject*>& a_explicitExclusions,
		std::unordered_set<RE::NiAVObject*>& a_inheritedExclusions,
		std::vector<RE::NiAVObject*>& a_nodes);
	bool IsDescendantOf(RE::NiAVObject* a_object, RE::NiAVObject* a_ancestor);
	void UpdateWorldData(RE::NiAVObject* a_object, bool a_dirty);
	RE::NiTransform BuildLocalToAncestor(RE::NiNode* a_node, RE::NiNode* a_ancestor);
	RE::NiNode* GetObjectNodeByName(RE::NiAVObject* a_root, std::string_view a_name);
}
