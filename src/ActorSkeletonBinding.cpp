#include "ActorSkeletonBinding.h"

#include "Fo4NiObjectUtils.h"
#include "PhysicsName.h"
#include "RE/B/BipedAnim.h"
#include "RE/N/NiAVObject.h"
#include "RE/N/NiNode.h"
#include "RE/T/TESObjectREFR.h"

#include <unordered_map>
#include <unordered_set>

namespace Smp::ActorSkeletonBinding
{
	namespace
	{
		void CollectTrustedNodeNames(
			const std::vector<RE::NiAVObject*>& a_trustedNodes,
			std::unordered_map<std::string, RE::NiAVObject*>& a_nameToNode)
		{
			for (auto* node : a_trustedNodes) {
				const auto nodeName = node ? node->GetName() : std::string_view{};
				if (nodeName.empty()) {
					continue;
				}
				if (!std::ranges::any_of(a_nameToNode, [nodeName](const auto& a_entry) {
						return PhysicsNamesEqual(a_entry.first, nodeName);
					})) {
					a_nameToNode.emplace(std::string(nodeName), node);
				}
			}
		}

		RE::NiAVObject* FindTrustedNodeByName(
			const std::unordered_map<std::string, RE::NiAVObject*>& a_nameToNode,
			const std::string_view a_name)
		{
			if (a_name.empty()) {
				return nullptr;
			}
			for (const auto& [name, node] : a_nameToNode) {
				if (PhysicsNamesEqual(name, a_name)) {
					return node;
				}
			}
			return nullptr;
		}

		RE::NiAVObject* FindNearestTrustedSourceParent(
			RE::NiNode* a_source,
			const std::unordered_map<std::string, RE::NiAVObject*>& a_nameToNode)
		{
			if (!a_source) {
				return nullptr;
			}
			for (auto* sourceParent = a_source->parent; sourceParent; sourceParent = sourceParent->parent) {
				if (auto* trustedParent = FindTrustedNodeByName(a_nameToNode, sourceParent->GetName())) {
					return trustedParent;
				}
			}
			return nullptr;
		}

		void CollectSourceParentMismatchedTrustedNodes(
			RE::NiNode* a_source,
			const std::unordered_map<std::string, RE::NiAVObject*>& a_nameToNode,
			std::unordered_set<RE::NiAVObject*>& a_rejected)
		{
			if (!a_source) {
				return;
			}

			const auto sourceName = a_source->GetName();
			auto* trustedNode = FindTrustedNodeByName(a_nameToNode, sourceName);
			auto* trustedParent = FindNearestTrustedSourceParent(a_source, a_nameToNode);
			if (trustedNode && trustedParent && trustedNode != trustedParent && !NiObject::IsDescendantOf(trustedNode, trustedParent)) {
				std::vector<RE::NiAVObject*> rejectedSubtree;
				NiObject::CollectNodePointers(trustedNode, rejectedSubtree);
				for (auto* rejectedNode : rejectedSubtree) {
					if (rejectedNode) {
						a_rejected.insert(rejectedNode);
					}
				}
				spdlog::debug(
					"removed pre-attach trusted actor candidate '{}' node={} parent={} parentName='{}' because source intended parent is {} parentName='{}'",
					sourceName,
					static_cast<void*>(trustedNode),
					static_cast<void*>(trustedNode->parent),
					trustedNode->parent ? std::string_view(trustedNode->parent->GetName()) : std::string_view{},
					static_cast<void*>(trustedParent),
					std::string_view(trustedParent->GetName()));
			}

			for (auto& child : a_source->children) {
				if (auto* childNode = child ? child->IsNode() : nullptr) {
					CollectSourceParentMismatchedTrustedNodes(childNode, a_nameToNode, a_rejected);
				}
			}
		}

		const std::string* FindTrustedActorNodeName(
			const std::vector<std::string>& a_trustedActorNodeNames,
			const std::string_view a_name)
		{
			if (a_name.empty()) {
				return nullptr;
			}

			const auto found = std::ranges::find_if(a_trustedActorNodeNames, [a_name](const std::string& a_trustedName) {
				return PhysicsNamesEqual(a_trustedName, a_name);
			});
			return found == a_trustedActorNodeNames.end() ? nullptr : std::addressof(*found);
		}

		void CollectPreAttachMergeParentBindings(
			RE::NiNode* a_source,
			const std::vector<std::string>& a_trustedActorNodeNames,
			std::vector<MergeParentBinding>& a_bindings)
		{
			if (!a_source) {
				return;
			}

			const auto sourceName = a_source->GetName();
			if (!sourceName.empty()) {
				for (auto* sourceParent = a_source->parent; sourceParent; sourceParent = sourceParent->parent) {
					const auto sourceParentName = sourceParent->GetName();
					const auto* trustedParentName = FindTrustedActorNodeName(a_trustedActorNodeNames, sourceParentName);
					if (!trustedParentName) {
						continue;
					}

					const auto alreadyRecorded = std::ranges::any_of(a_bindings, [sourceName](const MergeParentBinding& a_binding) {
						return PhysicsNamesEqual(a_binding.sourceName, sourceName);
					});
					if (!alreadyRecorded) {
						a_bindings.push_back({
							.sourceName = std::string(sourceName),
							.parentName = *trustedParentName,
							.localToParent = NiObject::BuildLocalToAncestor(a_source, sourceParent),
							.hasLocalToParent = true,
						});
					}
					break;
				}
			}

			for (auto& child : a_source->children) {
				if (auto* childNode = child ? child->IsNode() : nullptr) {
					CollectPreAttachMergeParentBindings(childNode, a_trustedActorNodeNames, a_bindings);
				}
			}
		}
	}

	std::vector<RE::NiAVObject*> CaptureTrustedActorSkeletonNodesBeforeAttach(
		RE::Actor* a_actor,
		RE::BipedAnim* a_biped,
		RE::NiAVObject* a_sourceObject,
		const bool a_firstPerson)
	{
		std::vector<RE::NiAVObject*> trustedNodes;
		auto* actorRoot = a_actor ? a_actor->Get3D(a_firstPerson) : nullptr;
		if (!actorRoot && a_actor) {
			actorRoot = a_actor->Get3D();
		}
		if (!actorRoot) {
			return trustedNodes;
		}

		std::unordered_set<RE::NiAVObject*> exclusions;
		auto addExclusion = [&exclusions](RE::NiAVObject* a_object) {
			if (a_object) {
				exclusions.insert(a_object);
			}
		};
		addExclusion(a_sourceObject);
		if (a_biped) {
			for (auto index = 0; index < std::to_underlying(RE::BIPED_OBJECT::kTotal); ++index) {
				const auto bipedObject = static_cast<RE::BIPED_OBJECT>(index);
				auto* bipObject = a_biped->GetBipObject(bipedObject);
				if (bipObject && bipObject->partClone) {
					addExclusion(bipObject->partClone.get());
				}
			}
		}

		auto inheritedExclusions = exclusions;
		NiObject::CollectNodePointersWithInheritedExclusions(actorRoot, exclusions, inheritedExclusions, trustedNodes);
		return trustedNodes;
	}

	void PruneTrustedActorSkeletonNodesBySourceParents(
		RE::NiAVObject* a_sourceObject,
		std::vector<RE::NiAVObject*>& a_trustedNodes)
	{
		auto* sourceRoot = a_sourceObject ? a_sourceObject->IsNode() : nullptr;
		if (!sourceRoot || a_trustedNodes.empty()) {
			return;
		}

		std::unordered_map<std::string, RE::NiAVObject*> nameToNode;
		CollectTrustedNodeNames(a_trustedNodes, nameToNode);
		if (nameToNode.empty()) {
			return;
		}

		std::unordered_set<RE::NiAVObject*> rejected;
		CollectSourceParentMismatchedTrustedNodes(sourceRoot, nameToNode, rejected);
		if (rejected.empty()) {
			return;
		}

		const auto before = a_trustedNodes.size();
		std::erase_if(a_trustedNodes, [&rejected](RE::NiAVObject* a_node) {
			return a_node && rejected.contains(a_node);
		});
		spdlog::debug(
			"pruned {} stale pre-attach trusted actor candidates using source parent bindings sourceRoot={} name='{}'",
			before - a_trustedNodes.size(),
			static_cast<void*>(sourceRoot),
			std::string_view(sourceRoot->GetName()));
	}

	std::vector<MergeParentBinding> BuildPreAttachMergeParentBindings(
		RE::NiAVObject* a_sourceObject,
		const std::vector<RE::NiAVObject*>& a_trustedActorSkeletonNodes)
	{
		std::vector<MergeParentBinding> bindings;
		auto* sourceRoot = a_sourceObject ? a_sourceObject->IsNode() : nullptr;
		if (!sourceRoot || a_trustedActorSkeletonNodes.empty()) {
			return bindings;
		}

		std::vector<std::string> trustedActorNodeNames;
		trustedActorNodeNames.reserve(a_trustedActorSkeletonNodes.size());
		for (auto* nodeObject : a_trustedActorSkeletonNodes) {
			const auto nodeName = nodeObject ? nodeObject->GetName() : std::string_view{};
			if (nodeName.empty()) {
				continue;
			}
			if (!std::ranges::any_of(trustedActorNodeNames, [nodeName](const std::string& a_existing) {
					return PhysicsNamesEqual(a_existing, nodeName);
				})) {
				trustedActorNodeNames.emplace_back(nodeName);
			}
		}
		if (trustedActorNodeNames.empty()) {
			return bindings;
		}

		CollectPreAttachMergeParentBindings(sourceRoot, trustedActorNodeNames, bindings);
		if (!bindings.empty()) {
			spdlog::debug(
				"captured {} pre-attach source parent bindings from original armor skeleton root={} name='{}'",
				bindings.size(),
				static_cast<void*>(sourceRoot),
				std::string_view(sourceRoot->GetName()));
			for (const auto& binding : bindings) {
				spdlog::debug(
					"pre-attach source parent binding source='{}' parent='{}' localToParent=({:.3f},{:.3f},{:.3f})",
					binding.sourceName,
					binding.parentName,
					binding.localToParent.translate.x,
					binding.localToParent.translate.y,
					binding.localToParent.translate.z);
			}
		}
		return bindings;
	}
}
