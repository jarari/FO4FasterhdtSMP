#include "Hooks.h"

#include "BSSkin.h"
#include "ConfigPaths.h"
#include "Fo4PhysicsWorld.h"
#include "LifecycleEvents.h"
#include "PhysicsName.h"
#include "SmpConfig.h"
#include "RE/B/BSAnimationGraphManager.h"
#include "RE/B/BGSHeadPart.h"
#include "RE/B/BSGeometry.h"
#include "RE/B/BSUtilities.h"
#include "RE/H/hkArray.h"
#include "RE/H/hkReferencedObject.h"
#include "RE/H/hkRefPtr.h"
#include "RE/M/Main.h"
#include "RE/N/NiCloningProcess.h"
#include "RE/N/NiNode.h"
#include "RE/N/NiPointer.h"
#include "RE/N/NiStringExtraData.h"
#include "RE/T/TESObjectREFR.h"
#include "RE/T/TESNPC.h"

#if defined(_M_X64) && !defined(_AMD64_)
#	define _AMD64_ 1
#endif
#include <Windows.h>
#include <detours.h>

#include <atomic>
#include <optional>
#include <unordered_set>
#include <unordered_map>

namespace Hooks
{
	namespace Addresses
	{
		REL::Relocation<std::uintptr_t> MainOnIdle{ REL::ID{ 633524, 0 } };
		REL::Relocation<std::uintptr_t> BipedAnimApplySkinnedObjects{ REL::ID{ 224320, 0 } };
		REL::Relocation<std::uintptr_t> BipedAnimAttachSkinnedObject{ REL::ID{ 1575810, 2194388 } };
		REL::Relocation<std::uintptr_t> BipedAnimAttachToParent{ REL::ID{ 1370428, 2194378 } };
		REL::Relocation<std::uintptr_t> BipedAnimRemovePart{ REL::ID{ 575576, 2194342 } };
		REL::Relocation<std::uintptr_t> Update3DModel{ REL::ID{ 986782, 2231882 } };
		REL::Relocation<std::uintptr_t> Reset3D{ REL::ID{ 302888, 2229913 } };
		REL::Relocation<std::uintptr_t> BSFaceGenPrepareHeadPart{ REL::ID{ 840416, 2209534 } };
		REL::Relocation<std::uintptr_t> BSFaceGenModelExtraDataSetBoneName{ REL::ID{ 1278503, 0 } };
		REL::Relocation<std::uintptr_t> BSFaceGenModelExtraDataGetBoneName{ REL::ID{ 190712, 0 } };
		REL::Relocation<std::uintptr_t> BSFaceGenNiNodeFixSkinInstances{ REL::ID{ 399655, 0 } };
	}

	using MainOnIdleUpdateHighActorsArraySorted_t = void (*)(RE::Main*, float);
	using MainSwap_t = void (*)(RE::Main*);
	using BipedAnimApplySkinnedObjects_t = RE::NiAVObject* (*)(RE::BipedAnim*, RE::NiNode*, RE::BIPED_OBJECT, bool);
	using BipedAnimAttachSkinnedObject_t = RE::NiAVObject* (*)(RE::BipedAnim*, RE::NiNode*, RE::NiNode*, RE::BIPED_OBJECT, bool);
	using BipedAnimAttachToParent_t = void (*)(RE::NiAVObject*, RE::NiAVObject*, RE::NiAVObject*, RE::BSTSmartPointer<RE::BipedAnim>&, RE::BIPED_OBJECT);
	using BipedAnimRemovePart_t = void (*)(RE::BipedAnim*, RE::BIPOBJECT*, bool);
	using ActorLoad3D_t = RE::NiAVObject* (*)(RE::TESObjectREFR*, bool);
	using Set3D_t = void (*)(RE::TESObjectREFR*, RE::NiAVObject*, bool);
	using OnHeadInitialized_t = void (*)(RE::TESObjectREFR*);
	using Update3DModel_t = void (*)(void*, RE::Actor*, bool);
	using Reset3D_t = void (*)(RE::Actor*, bool, std::uint32_t, bool, std::uint32_t);
	using PrepareHeadPart_t = void (*)(RE::BSFaceGenNiNode*, RE::BGSHeadPart*, const RE::TESNPC*, bool);
	using SetFaceGenBoneName_t = void (*)(void*, std::uint32_t, RE::BSFixedString*);

	MainOnIdleUpdateHighActorsArraySorted_t OriginalMainOnIdleUpdateHighActorsArraySorted{ nullptr };
	MainSwap_t                     OriginalMainSwap{ nullptr };
	BipedAnimApplySkinnedObjects_t OriginalBipedAnimApplySkinnedObjects{ nullptr };
	BipedAnimAttachSkinnedObject_t OriginalBipedAnimAttachSkinnedObject{ nullptr };
	BipedAnimAttachToParent_t      OriginalBipedAnimAttachToParent{ nullptr };
	BipedAnimRemovePart_t          OriginalBipedAnimRemovePart{ nullptr };
	ActorLoad3D_t                  OriginalActorLoad3D{ nullptr };
	ActorLoad3D_t                  OriginalPlayerCharacterLoad3D{ nullptr };
	Set3D_t                        OriginalActorSet3D{ nullptr };
	Set3D_t                        OriginalPlayerCharacterSet3D{ nullptr };
	OnHeadInitialized_t            OriginalActorOnHeadInitialized{ nullptr };
	OnHeadInitialized_t            OriginalPlayerCharacterOnHeadInitialized{ nullptr };
	Update3DModel_t                OriginalUpdate3DModel{ nullptr };
	Reset3D_t                      OriginalReset3D{ nullptr };
	PrepareHeadPart_t              OriginalPrepareHeadPart{ nullptr };
	SetFaceGenBoneName_t           OriginalSetFaceGenBoneName{ nullptr };

	inline constexpr std::size_t kApplySkinnedObjectsPrologueSize = 14;
	inline constexpr std::size_t kAttachSkinnedObjectPrologueSize = 15;
	inline constexpr std::size_t kAttachToParentPrologueSize = 15;
	inline constexpr std::size_t kRemovePartPrologueSize = 15;
	inline constexpr std::size_t kUpdate3DModelPrologueSize = 5;
	inline constexpr std::size_t kReset3DPrologueSize = 5;
	inline constexpr std::uintptr_t kMainOnIdleUpdateHighActorsArraySortedCallOffsetOG = 0x6E4;
	inline constexpr std::uintptr_t kMainOnIdleUpdateHighActorsArraySortedCallOffsetAE = 0x6E4;
	inline constexpr std::uintptr_t kMainOnIdleSwapCallOffsetOG = 0x6EC;
	inline constexpr std::uintptr_t kMainOnIdleSwapCallOffsetAE = 0x6EC;
	inline constexpr std::uintptr_t kGetBoneNameLimitImmediateOffsetOG = 0x6;
	inline constexpr std::uintptr_t kFixSkinInstancesLimitImmediateOffsetOG = 0x91A;
	inline constexpr std::uint32_t kFaceGenModelExtraDataBoneNameLimit = 0x80;
	inline constexpr bool kEnablePrepareHeadPartHook = true;

	std::mutex               BackupNodeLock;
	std::vector<std::string> BackupNodeNames;
	thread_local std::uint32_t ApplySkinnedObjectsDepth{ 0 };
	constexpr std::string_view kPhysicsXmlExtraName = "HDT Skinned Mesh Physics Object";
	std::mutex FaceGenActorLock;
	std::unordered_map<RE::BSFaceGenNiNode*, RE::ActorHandle> FaceGenActorMap;

	struct PendingPreparedHeadPart
	{
		RE::NiPointer<RE::NiAVObject> object;
		RE::BGSHeadPart* headPart{ nullptr };
	};

	std::unordered_map<RE::BSFaceGenNiNode*, std::vector<PendingPreparedHeadPart>> PendingPreparedHeadParts;

	using BackupBoneMap = std::unordered_map<std::string, std::vector<RE::NiPointer<RE::NiAVObject>>>;

	RE::Actor* ResolveActor(RE::BipedAnim* a_biped);

	struct ScopedApplySkinnedObjectsDepth
	{
		ScopedApplySkinnedObjectsDepth()
		{
			++ApplySkinnedObjectsDepth;
		}

		~ScopedApplySkinnedObjectsDepth()
		{
			--ApplySkinnedObjectsDepth;
		}
	};

	std::optional<std::string> FindPhysicsXmlExtraData(RE::NiAVObject* a_object)
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
			const auto data = Smp::ConfigPaths::Trim(std::string(std::string_view(stringExtra->data)));
			if (data.empty() || !Smp::PhysicsNamesEqual(name, kPhysicsXmlExtraName)) {
				continue;
			}

			if (auto path = Smp::ConfigPaths::ResolveExistingConfigPath(data, true)) {
				return path->string();
			}
			spdlog::warn("found '{}' NiStringExtraData on original model object={} but XML path could not be resolved: {}", kPhysicsXmlExtraName, static_cast<void*>(a_object), data);
		}
		return std::nullopt;
	}

	RE::NiPointer<RE::NiAVObject> ClonePhysicsMergeSource(RE::NiNode* a_root)
	{
		if (!a_root) {
			return nullptr;
		}

		RE::NiCloningProcess cloneProcess;
		cloneProcess.appendChar = '$';
		cloneProcess.copyType = RE::NiCloningProcess::CopyType::kCopyExact;
		cloneProcess.scale = { 1.0F, 1.0F, 1.0F };

		auto* clone = a_root->CreateClone(cloneProcess);
		a_root->ProcessClone(cloneProcess);
		return clone ? static_cast<RE::NiAVObject*>(clone->IsNode()) : nullptr;
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

	bool IsExcludedNodePointer(RE::NiAVObject* a_object, const std::unordered_set<RE::NiAVObject*>& a_exclusions)
	{
		return a_object && a_exclusions.contains(a_object);
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
					return Smp::PhysicsNamesEqual(a_entry.first, nodeName);
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
			if (Smp::PhysicsNamesEqual(name, a_name)) {
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

	void CollectSourceParentMismatchedTrustedNodes(
		RE::NiNode* a_source,
		const std::unordered_map<std::string, RE::NiAVObject*>& a_nameToNode,
		std::unordered_set<RE::NiAVObject*>& a_rejected)
	{
		if (!a_source) {
			return;
		}

		const auto sourceName = a_source->GetName();
		auto*      trustedNode = FindTrustedNodeByName(a_nameToNode, sourceName);
		auto*      trustedParent = FindNearestTrustedSourceParent(a_source, a_nameToNode);
		if (trustedNode && trustedParent && trustedNode != trustedParent && !IsDescendantOf(trustedNode, trustedParent)) {
			std::vector<RE::NiAVObject*> rejectedSubtree;
			CollectNodePointers(trustedNode, rejectedSubtree);
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

	std::vector<RE::NiAVObject*> CaptureTrustedActorSkeletonNodesBeforeAttach(
		RE::BipedAnim* a_biped,
		RE::NiAVObject* a_sourceObject,
		const bool a_firstPerson)
	{
		std::vector<RE::NiAVObject*> trustedNodes;
		auto* actor = ResolveActor(a_biped);
		auto* actorRoot = actor ? actor->Get3D(a_firstPerson) : nullptr;
		if (!actorRoot && actor) {
			actorRoot = actor->Get3D();
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
		CollectNodePointersWithInheritedExclusions(actorRoot, exclusions, inheritedExclusions, trustedNodes);
		return trustedNodes;
	}

	const std::string* FindTrustedActorNodeName(
		const std::vector<std::string>& a_trustedActorNodeNames,
		const std::string_view a_name)
	{
		if (a_name.empty()) {
			return nullptr;
		}

		const auto found = std::ranges::find_if(a_trustedActorNodeNames, [a_name](const std::string& a_trustedName) {
			return Smp::PhysicsNamesEqual(a_trustedName, a_name);
		});
		return found == a_trustedActorNodeNames.end() ? nullptr : std::addressof(*found);
	}

	RE::NiTransform BuildLocalToSourceAncestor(RE::NiNode* a_node, RE::NiNode* a_ancestor)
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

	void CollectPreAttachMergeParentBindings(
		RE::NiNode* a_source,
		const std::vector<std::string>& a_trustedActorNodeNames,
		std::vector<Smp::MergeParentBinding>& a_bindings)
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

				const auto alreadyRecorded = std::ranges::any_of(a_bindings, [sourceName](const Smp::MergeParentBinding& a_binding) {
					return Smp::PhysicsNamesEqual(a_binding.sourceName, sourceName);
				});
				if (!alreadyRecorded) {
					a_bindings.push_back({
						.sourceName = std::string(sourceName),
						.parentName = *trustedParentName,
						.localToParent = BuildLocalToSourceAncestor(a_source, sourceParent),
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

	std::vector<Smp::MergeParentBinding> BuildPreAttachMergeParentBindings(
		RE::NiAVObject* a_sourceObject,
		const std::vector<RE::NiAVObject*>& a_trustedActorSkeletonNodes)
	{
		std::vector<Smp::MergeParentBinding> bindings;
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
			const auto duplicate = std::ranges::any_of(trustedActorNodeNames, [nodeName](const std::string& a_existing) {
				return Smp::PhysicsNamesEqual(a_existing, nodeName);
			});
			if (!duplicate) {
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

	std::vector<std::string> GetBackupNodeNames()
	{
		std::scoped_lock lock(BackupNodeLock);
		return BackupNodeNames;
	}

	RE::BSSkin::Instance* GetBackupSkinInstance(RE::NiAVObject* a_root, const std::string& a_nodeName)
	{
		if (!a_root || a_nodeName.empty()) {
			return nullptr;
		}

		auto* object = a_root->GetObjectByName(RE::BSFixedString(a_nodeName));
		auto* geometry = object ? object->IsGeometry() : nullptr;
		return geometry && geometry->skinInstance ? geometry->skinInstance.get() : nullptr;
	}

	BackupBoneMap CaptureBackupBones(RE::NiAVObject* a_root, const std::vector<std::string>& a_nodeNames)
	{
		BackupBoneMap result;
		for (const auto& nodeName : a_nodeNames) {
			auto* skin = GetBackupSkinInstance(a_root, nodeName);
			if (!skin || skin->bones.empty() || skin->bones.size() > RE::BSSkin::kMaxExpectedBones) {
				continue;
			}

			std::vector<RE::NiPointer<RE::NiAVObject>> bones;
			bones.reserve(skin->bones.size());
			for (std::uint32_t index = 0; index < skin->bones.size(); ++index) {
				bones.emplace_back(skin->bones[index]);
			}

			if (!bones.empty()) {
				result.emplace(nodeName, std::move(bones));
			}
		}
		return result;
	}

	void RestoreBackupBones(RE::NiAVObject* a_root, const BackupBoneMap& a_backupBones)
	{
		for (const auto& [nodeName, bones] : a_backupBones) {
			auto* skin = GetBackupSkinInstance(a_root, nodeName);
			if (!skin || skin->bones.size() > RE::BSSkin::kMaxExpectedBones) {
				continue;
			}

			const auto restoreCount = std::min<std::uint32_t>(skin->bones.size(), static_cast<std::uint32_t>(std::min<std::size_t>(bones.size(), RE::BSSkin::kMaxExpectedBones)));
			std::size_t restored = 0;
			for (std::uint32_t index = 0; index < restoreCount; ++index) {
				if (!skin->bones[index] && bones[index]) {
					skin->bones[index] = bones[index].get();
					++restored;
				}
			}

			if (restored > 0) {
				spdlog::debug("restored {} missing skin bones for backup node '{}'", restored, nodeName);
			}
		}
	}

	void ApplyConfig(const Smp::RuntimeSettings& a_settings)
	{
		std::scoped_lock lock(BackupNodeLock);
		BackupNodeNames = a_settings.smp.backupNodeByName;
	}

	RE::Actor* ResolveActor(RE::BipedAnim* a_biped)
	{
		if (!a_biped) {
			return nullptr;
		}

		const auto handle = a_biped->GetRequester();
		auto       ptr = handle.get();
		return ptr ? ptr->As<RE::Actor>() : nullptr;
	}

	RE::BIPED_OBJECT ResolveBipedObject(RE::BipedAnim* a_biped, RE::BIPOBJECT* a_bipObject)
	{
		if (!a_biped || !a_bipObject) {
			return RE::BIPED_OBJECT::kTotal;
		}

		for (auto index = 0; index < std::to_underlying(RE::BIPED_OBJECT::kTotal); ++index) {
			const auto bipedObject = static_cast<RE::BIPED_OBJECT>(index);
			if (a_biped->GetBipObject(bipedObject) == a_bipObject) {
				return bipedObject;
			}
		}

		return RE::BIPED_OBJECT::kTotal;
	}

	bool IsFirstPersonBiped(RE::BipedAnim* a_biped)
	{
		const auto* player = RE::PlayerCharacter::GetSingleton();
		return player && a_biped && player->firstPersonBipedAnim.get() == a_biped;
	}

	template <class T>
	T CreateBranchGateway5(const char* a_name, REL::Relocation<std::uintptr_t>& a_target, const std::size_t a_prologueSize, void* a_hook)
	{
		const auto targetAddress = a_target.address();
		const auto* targetBytes = reinterpret_cast<const std::byte*>(targetAddress);
		auto&      trampoline = REL::GetTrampoline();
		const auto prologueSize = a_prologueSize;

		if (prologueSize >= 5 && targetBytes[0] == std::byte{ 0xFF } && targetBytes[1] == std::byte{ 0x25 }) {
			std::int32_t oldDisp32 = 0;
			std::memcpy(std::addressof(oldDisp32), targetBytes + 2, sizeof(oldDisp32));
			const auto indirectAddress = targetAddress + sizeof(REL::ASM::JMP6) + oldDisp32;
			std::uintptr_t absoluteDest = 0;
			std::memcpy(std::addressof(absoluteDest), reinterpret_cast<const void*>(indirectAddress), sizeof(absoluteDest));

			auto* gateway = static_cast<std::byte*>(trampoline.allocate(sizeof(REL::ASM::JMP14)));
			const REL::ASM::JMP14 chainedJump{ absoluteDest };
			std::memcpy(gateway, std::addressof(chainedJump), sizeof(chainedJump));

			trampoline.write_jmp6(targetAddress, reinterpret_cast<std::uintptr_t>(a_hook));
			spdlog::info("{} found pre-existing 14-byte jmp at {:x}; chaining through {:x}", a_name, targetAddress, absoluteDest);
			spdlog::info("{} branch hook installed at {:x}", a_name, targetAddress);
			return reinterpret_cast<T>(gateway);
		}

		if (prologueSize >= 5 && targetBytes[0] == std::byte{ 0xE9 }) {
			std::int32_t oldRel32 = 0;
			std::memcpy(std::addressof(oldRel32), targetBytes + 1, sizeof(oldRel32));
			const auto absoluteDest = static_cast<std::uintptr_t>(static_cast<std::int64_t>(targetAddress) + 5 + oldRel32);

			auto* gateway = static_cast<std::byte*>(trampoline.allocate(sizeof(REL::ASM::JMP14)));
			const REL::ASM::JMP14 chainedJump{ absoluteDest };
			std::memcpy(gateway, std::addressof(chainedJump), sizeof(chainedJump));

			trampoline.write_jmp5(targetAddress, reinterpret_cast<std::uintptr_t>(a_hook));
			spdlog::info("{} found pre-existing 5-byte jmp at {:x}; chaining through {:x}", a_name, targetAddress, absoluteDest);
			spdlog::info("{} branch hook installed at {:x}", a_name, targetAddress);
			return reinterpret_cast<T>(gateway);
		}

		auto*      gateway = static_cast<std::byte*>(trampoline.allocate(prologueSize + sizeof(REL::ASM::JMP14)));
		std::memcpy(gateway, targetBytes, prologueSize);

		const REL::ASM::JMP14 jumpBack{ targetAddress + prologueSize };
		std::memcpy(gateway + prologueSize, std::addressof(jumpBack), sizeof(jumpBack));
		trampoline.write_jmp5(targetAddress, reinterpret_cast<std::uintptr_t>(a_hook));
		spdlog::info("{} branch hook installed at {:x}", a_name, targetAddress);
		return reinterpret_cast<T>(gateway);
	}

	template <class T>
	T InstallVFuncHook(const char* a_name, REL::Relocation<std::uintptr_t>& a_vtable, const std::size_t a_index, void* a_hook)
	{
		const auto previous = reinterpret_cast<T>(a_vtable.write_vfunc(a_index, reinterpret_cast<std::uintptr_t>(a_hook)));
		if (previous && previous != reinterpret_cast<T>(a_hook)) {
			spdlog::info("{} vfunc hook installed at slot {:x}; previous target={} will be chained", a_name, a_index, reinterpret_cast<void*>(previous));
			return previous;
		}

		if (previous == reinterpret_cast<T>(a_hook)) {
			spdlog::warn("{} vfunc slot {:x} already points to this hook; refusing to use it as the original target", a_name, a_index);
			return nullptr;
		}

		{
			spdlog::info("{} vfunc hook installed at slot {:x}", a_name, a_index);
		}
		return previous;
	}

	void LogRelocationTarget(const char* a_name, const std::uintptr_t a_address)
	{
		const auto module = REX::FModule::GetExecutingModule();
		const auto base = module.GetBaseAddress();
		const auto offset = base != 0 && a_address >= base ? a_address - base : 0;
		spdlog::info("{} resolved at {:x} moduleOffset={:x}", a_name, a_address, offset);
	}

	bool ValidateUInt32Immediate(
		const char* a_name,
		REL::Relocation<std::uintptr_t>& a_target,
		const std::uintptr_t a_offset,
		const std::uint32_t a_expected)
	{
		const auto address = a_target.address() + a_offset;
		std::uint32_t current = 0;
		std::memcpy(std::addressof(current), reinterpret_cast<const void*>(address), sizeof(current));
		if (current != a_expected) {
			spdlog::error(
				"{} validation failed at {:x}: expected immediate {:#x}, found {:#x}",
				a_name,
				address,
				a_expected,
				current);
			return false;
		}

		spdlog::info("{} validated at {:x}: {:#x}", a_name, address, a_expected);
		return true;
	}

	bool ValidateFaceGenBoneNameLimit()
	{
		if (!REX::FModule::IsRuntimeOG()) {
			spdlog::warn("FaceGen bone-name limit validation skipped: AE relocation IDs are not verified");
			return true;
		}

		LogRelocationTarget("BSFaceGenModelExtraData::GetBoneName", Addresses::BSFaceGenModelExtraDataGetBoneName.address());
		LogRelocationTarget("BSFaceGenNiNode::FixSkinInstances", Addresses::BSFaceGenNiNodeFixSkinInstances.address());

		const auto getBoneNameValid = ValidateUInt32Immediate(
			"BSFaceGenModelExtraData::GetBoneName bone-name limit",
			Addresses::BSFaceGenModelExtraDataGetBoneName,
			kGetBoneNameLimitImmediateOffsetOG,
			kFaceGenModelExtraDataBoneNameLimit);
		const auto fixSkinInstancesValid = ValidateUInt32Immediate(
			"BSFaceGenNiNode::FixSkinInstances bone-name limit",
			Addresses::BSFaceGenNiNodeFixSkinInstances,
			kFixSkinInstancesLimitImmediateOffsetOG,
			kFaceGenModelExtraDataBoneNameLimit);
		return getBoneNameValid && fixSkinInstancesValid;
	}

	void LogHookTargets()
	{
		LogRelocationTarget("Main::OnIdle", Addresses::MainOnIdle.address());
		LogRelocationTarget("BipedAnim::ApplySkinnedObjects", Addresses::BipedAnimApplySkinnedObjects.address());
		LogRelocationTarget("BipedAnim::AttachSkinnedObject", Addresses::BipedAnimAttachSkinnedObject.address());
		LogRelocationTarget("BipedAnim::AttachToParent", Addresses::BipedAnimAttachToParent.address());
		LogRelocationTarget("BipedAnim::RemovePart", Addresses::BipedAnimRemovePart.address());
		LogRelocationTarget("AIProcess::Update3DModel", Addresses::Update3DModel.address());
		LogRelocationTarget("Actor::Reset3D", Addresses::Reset3D.address());
		LogRelocationTarget("BSFaceGenUtils::PrepareHeadPart", Addresses::BSFaceGenPrepareHeadPart.address());
		if (REX::FModule::IsRuntimeOG()) {
			LogRelocationTarget("BSFaceGenModelExtraData::SetBoneName", Addresses::BSFaceGenModelExtraDataSetBoneName.address());
		} else {
			spdlog::warn("BSFaceGenModelExtraData::SetBoneName hook skipped: AE relocation ID is not verified");
		}
	}

	void EmitEvent(const Smp::LifecycleEvent& a_event)
	{
		const auto highFrequency =
			a_event.type == Smp::LifecycleEventType::kActorUpdate3DModel ||
			(a_event.type == Smp::LifecycleEventType::kActorSet3D && !a_event.object) ||
			((a_event.type == Smp::LifecycleEventType::kArmorApplySkinnedObjects ||
				 a_event.type == Smp::LifecycleEventType::kArmorAttachSkinnedObject) &&
				a_event.physicsXmlPath.empty()) ||
			a_event.type == Smp::LifecycleEventType::kArmorDetachBegin ||
			a_event.type == Smp::LifecycleEventType::kArmorDetachEnd;
		const auto level = highFrequency ? spdlog::level::trace : spdlog::level::debug;
		spdlog::log(
			level,
			"SMP lifecycle {} actor={} biped={} bipedObject={} object={} source={} xml='{}'",
			Smp::ToString(a_event.type),
			static_cast<void*>(a_event.actor),
			static_cast<void*>(a_event.biped),
			std::to_underlying(a_event.bipedObject),
			static_cast<void*>(a_event.object),
			static_cast<void*>(a_event.sourceObject),
			a_event.physicsXmlPath);

		Smp::NotifyLifecycleEvent(a_event);
	}

	void SeedFaceGenActor(RE::TESObjectREFR* a_ref)
	{
		auto* actor = a_ref ? static_cast<RE::Actor*>(a_ref) : nullptr;
		auto* faceNode = actor ? actor->GetFaceNodeSkinned() : nullptr;
		if (!actor || !faceNode) {
			return;
		}

		auto handle = RE::BSPointerHandleManagerInterface<RE::Actor>::GetHandle(actor);
		if (!handle) {
			return;
		}

		std::scoped_lock lock(FaceGenActorLock);
		std::erase_if(FaceGenActorMap, [actor, faceNode](const auto& a_entry) {
			const auto resolved = a_entry.second.get();
			return !resolved || (resolved.get() == actor && a_entry.first != faceNode);
		});
		FaceGenActorMap[faceNode] = handle;
	}

	RE::Actor* ResolveFaceGenActor(RE::BSFaceGenNiNode* a_faceNode)
	{
		if (!a_faceNode) {
			return nullptr;
		}

		std::scoped_lock lock(FaceGenActorLock);
		const auto found = FaceGenActorMap.find(a_faceNode);
		if (found == FaceGenActorMap.end()) {
			return nullptr;
		}

		auto resolved = found->second.get();
		if (!resolved) {
			FaceGenActorMap.erase(found);
			return nullptr;
		}
		if (resolved->GetFaceNodeSkinned() != a_faceNode) {
			FaceGenActorMap.erase(found);
			return nullptr;
		}
		return resolved.get();
	}

	void EmitPreparedHeadPartEvent(RE::Actor* a_actor, RE::BSFaceGenNiNode* a_faceNode, RE::NiAVObject* a_object, RE::BGSHeadPart* a_headPart)
	{
		if (!a_actor) {
			return;
		}

		EmitEvent({
			.type = Smp::LifecycleEventType::kHeadPrepareHeadPart,
			.actor = a_actor,
			.object = a_object ? a_object : reinterpret_cast<RE::NiAVObject*>(a_faceNode),
			.headPart = a_headPart,
		});
	}

	void DiscardPendingPreparedHeadPartEvents(RE::BSFaceGenNiNode* a_faceNode)
	{
		std::vector<PendingPreparedHeadPart> pending;
		{
			std::scoped_lock lock(FaceGenActorLock);
			auto found = PendingPreparedHeadParts.find(a_faceNode);
			if (found == PendingPreparedHeadParts.end()) {
				return;
			}
			pending = std::move(found->second);
			PendingPreparedHeadParts.erase(found);
		}

		spdlog::debug(
			"discarded {} pending prepared headpart events after head initialization faceNode={}; full current face node scan will rebuild head physics",
			pending.size(),
			static_cast<void*>(a_faceNode));
	}

	RE::NiAVObject* FindPreparedHeadPartObject(RE::BSFaceGenNiNode* a_faceNode, RE::BGSHeadPart* a_headPart)
	{
		auto* faceObject = reinterpret_cast<RE::NiAVObject*>(a_faceNode);
		if (!faceObject || !a_headPart) {
			return faceObject;
		}

		if (!a_headPart->formEditorID.empty()) {
			if (auto* object = RE::BSUtilities::GetObjectByName(faceObject, a_headPart->formEditorID, true, true)) {
				return object;
			}
		}

		const auto modelKey = Smp::ConfigPaths::LowerString(Smp::ConfigPaths::Trim(std::string(a_headPart->ChargenModel.GetModel())));
		if (!modelKey.empty()) {
			if (auto slash = modelKey.find_last_of("\\/"); slash != std::string::npos) {
				auto name = modelKey.substr(slash + 1);
				if (auto dot = name.find_last_of('.'); dot != std::string::npos) {
					name.erase(dot);
				}
				if (!name.empty()) {
					if (auto* object = RE::BSUtilities::GetObjectByName(faceObject, RE::BSFixedString(name), true, true)) {
						return object;
					}
				}
			}
		}

		return faceObject;
	}

	RE::NiAVObject* HookedBipedAnimApplySkinnedObjects(RE::BipedAnim* a_biped, RE::NiNode* a_originalModelRoot, RE::BIPED_OBJECT a_bipedObject, bool a_firstPerson)
	{
		auto* originalModelObject = a_originalModelRoot ? static_cast<RE::NiAVObject*>(a_originalModelRoot) : nullptr;
		auto selectedXml = FindPhysicsXmlExtraData(originalModelObject);
		auto trustedActorSkeletonNodes = selectedXml ?
			CaptureTrustedActorSkeletonNodesBeforeAttach(a_biped, originalModelObject, a_firstPerson) :
			std::vector<RE::NiAVObject*>{};
		if (selectedXml) {
			PruneTrustedActorSkeletonNodesBySourceParents(originalModelObject, trustedActorSkeletonNodes);
		}
		auto preAttachMergeParentBindings = selectedXml ?
			BuildPreAttachMergeParentBindings(originalModelObject, trustedActorSkeletonNodes) :
			std::vector<Smp::MergeParentBinding>{};
		std::vector<RE::NiAVObject*> mergeSearchExclusions;
		if (selectedXml) {
			CollectNodePointers(originalModelObject, mergeSearchExclusions);
			spdlog::debug(
				"pre-scanned armor physics XML {} from original model root={} name='{}'",
				*selectedXml,
				static_cast<void*>(a_originalModelRoot),
				a_originalModelRoot ? std::string_view(a_originalModelRoot->GetName()) : std::string_view{});
		}
		auto mergeSourceObject = selectedXml ? ClonePhysicsMergeSource(a_originalModelRoot) : RE::NiPointer<RE::NiAVObject>{};
		if (mergeSourceObject) {
			auto* mergeSourceNode = mergeSourceObject->IsNode();
			spdlog::debug(
				"preserved armor merge source clone={} name='{}' children={} from original model root={}",
				static_cast<void*>(mergeSourceObject.get()),
				std::string_view(mergeSourceObject->GetName()),
				mergeSourceNode ? mergeSourceNode->children.size() : 0,
				static_cast<void*>(a_originalModelRoot));
		}

		const auto backupNodeNames = GetBackupNodeNames();
		const auto backupBones = backupNodeNames.empty() ? BackupBoneMap{} : CaptureBackupBones(a_originalModelRoot ? static_cast<RE::NiAVObject*>(a_originalModelRoot) : nullptr, backupNodeNames);

		RE::NiAVObject* attachedObject = nullptr;
		{
			ScopedApplySkinnedObjectsDepth scopedDepth;
			attachedObject = OriginalBipedAnimApplySkinnedObjects(a_biped, a_originalModelRoot, a_bipedObject, a_firstPerson);
		}
		if (!backupBones.empty()) {
			RestoreBackupBones(attachedObject, backupBones);
		}

		auto* bipObject = a_biped ? a_biped->GetBipObject(a_bipedObject) : nullptr;
		EmitEvent({
			.type = Smp::LifecycleEventType::kArmorApplySkinnedObjects,
			.actor = ResolveActor(a_biped),
			.biped = a_biped,
			.bipObject = bipObject,
			.bipedObject = a_bipedObject,
			.object = attachedObject,
			.sourceObject = a_originalModelRoot ? static_cast<RE::NiAVObject*>(a_originalModelRoot) : nullptr,
			.mergeSourceObject = mergeSourceObject.get(),
			.trustedActorSkeletonNodes = std::move(trustedActorSkeletonNodes),
			.mergeSearchExclusions = std::move(mergeSearchExclusions),
			.mergeParentBindings = std::move(preAttachMergeParentBindings),
			.physicsXmlPath = selectedXml.value_or(std::string{}),
			.firstPerson = a_firstPerson,
		});
		return attachedObject;
	}

	RE::NiAVObject* HookedBipedAnimAttachSkinnedObject(RE::BipedAnim* a_biped, RE::NiNode* a_destinationRoot, RE::NiNode* a_sourceRoot, RE::BIPED_OBJECT a_bipedObject, bool a_firstPerson)
	{
		auto* sourceObject = a_sourceRoot ? static_cast<RE::NiAVObject*>(a_sourceRoot) : nullptr;
		auto selectedXml = FindPhysicsXmlExtraData(sourceObject);
		auto trustedActorSkeletonNodes = selectedXml ?
			CaptureTrustedActorSkeletonNodesBeforeAttach(a_biped, sourceObject, a_firstPerson) :
			std::vector<RE::NiAVObject*>{};
		if (selectedXml) {
			PruneTrustedActorSkeletonNodesBySourceParents(sourceObject, trustedActorSkeletonNodes);
		}
		auto preAttachMergeParentBindings = selectedXml ?
			BuildPreAttachMergeParentBindings(sourceObject, trustedActorSkeletonNodes) :
			std::vector<Smp::MergeParentBinding>{};
		const auto backupNodeNames = GetBackupNodeNames();
		const auto backupBones = backupNodeNames.empty() ? BackupBoneMap{} : CaptureBackupBones(a_sourceRoot ? static_cast<RE::NiAVObject*>(a_sourceRoot) : static_cast<RE::NiAVObject*>(a_destinationRoot), backupNodeNames);

		auto* attachedObject = OriginalBipedAnimAttachSkinnedObject(a_biped, a_destinationRoot, a_sourceRoot, a_bipedObject, a_firstPerson);
		if (!backupBones.empty()) {
			RestoreBackupBones(attachedObject, backupBones);
		}
		if (ApplySkinnedObjectsDepth > 0) {
			return attachedObject;
		}
		auto* bipObject = a_biped ? a_biped->GetBipObject(a_bipedObject) : nullptr;
		EmitEvent({
			.type = Smp::LifecycleEventType::kArmorAttachSkinnedObject,
			.actor = ResolveActor(a_biped),
			.biped = a_biped,
			.bipObject = bipObject,
			.bipedObject = a_bipedObject,
			.object = attachedObject,
			.sourceObject = bipObject ? bipObject->partClone.get() : nullptr,
			.trustedActorSkeletonNodes = std::move(trustedActorSkeletonNodes),
			.mergeParentBindings = std::move(preAttachMergeParentBindings),
			.destinationRoot = a_destinationRoot,
			.sourceRoot = a_sourceRoot,
			.physicsXmlPath = selectedXml.value_or(std::string{}),
			.firstPerson = a_firstPerson,
		});
		return attachedObject;
	}

	void HookedBipedAnimAttachToParent(RE::NiAVObject* a_parent, RE::NiAVObject* a_attachedObject, RE::NiAVObject* a_sourceObject, RE::BSTSmartPointer<RE::BipedAnim>& a_biped, RE::BIPED_OBJECT a_bipedObject)
	{
		OriginalBipedAnimAttachToParent(a_parent, a_attachedObject, a_sourceObject, a_biped, a_bipedObject);
		if (ApplySkinnedObjectsDepth > 0) {
			return;
		}
		EmitEvent({
			.type = Smp::LifecycleEventType::kArmorAttachToParent,
			.actor = ResolveActor(a_biped.get()),
			.biped = a_biped.get(),
			.bipedObject = a_bipedObject,
			.object = a_attachedObject,
			.sourceObject = a_sourceObject,
			.firstPerson = IsFirstPersonBiped(a_biped.get()),
		});
	}

	void HookedBipedAnimRemovePart(RE::BipedAnim* a_biped, RE::BIPOBJECT* a_bipObject, bool a_queueDetach)
	{
		const auto bipedObject = ResolveBipedObject(a_biped, a_bipObject);
		EmitEvent({
			.type = Smp::LifecycleEventType::kArmorDetachBegin,
			.actor = ResolveActor(a_biped),
			.biped = a_biped,
			.bipObject = a_bipObject,
			.bipedObject = bipedObject,
			.object = a_bipObject ? a_bipObject->partClone.get() : nullptr,
			.firstPerson = IsFirstPersonBiped(a_biped),
			.queueDetach = a_queueDetach,
		});

		OriginalBipedAnimRemovePart(a_biped, a_bipObject, a_queueDetach);

		EmitEvent({
			.type = Smp::LifecycleEventType::kArmorDetachEnd,
			.actor = ResolveActor(a_biped),
			.biped = a_biped,
			.bipObject = a_bipObject,
			.bipedObject = bipedObject,
			.firstPerson = IsFirstPersonBiped(a_biped),
			.queueDetach = a_queueDetach,
		});
	}

	RE::NiAVObject* HookedActorLoad3D(RE::TESObjectREFR* a_ref, bool a_backgroundLoading)
	{
		auto* loaded3D = OriginalActorLoad3D(a_ref, a_backgroundLoading);
		EmitEvent({
			.type = Smp::LifecycleEventType::kActorLoad3D,
			.actor = static_cast<RE::Actor*>(a_ref),
			.object = loaded3D,
		});
		return loaded3D;
	}

	RE::NiAVObject* HookedPlayerCharacterLoad3D(RE::TESObjectREFR* a_ref, bool a_backgroundLoading)
	{
		auto* loaded3D = OriginalPlayerCharacterLoad3D(a_ref, a_backgroundLoading);
		EmitEvent({
			.type = Smp::LifecycleEventType::kActorLoad3D,
			.actor = static_cast<RE::Actor*>(a_ref),
			.object = loaded3D,
		});
		return loaded3D;
	}

	void HookedActorSet3D(RE::TESObjectREFR* a_ref, RE::NiAVObject* a_object, bool a_queue3DTasks)
	{
		OriginalActorSet3D(a_ref, a_object, a_queue3DTasks);
		EmitEvent({
			.type = Smp::LifecycleEventType::kActorSet3D,
			.actor = static_cast<RE::Actor*>(a_ref),
			.object = a_object,
			.queue3DTasks = a_queue3DTasks,
		});
	}

	void HookedPlayerCharacterSet3D(RE::TESObjectREFR* a_ref, RE::NiAVObject* a_object, bool a_queue3DTasks)
	{
		OriginalPlayerCharacterSet3D(a_ref, a_object, a_queue3DTasks);
		EmitEvent({
			.type = Smp::LifecycleEventType::kActorSet3D,
			.actor = static_cast<RE::Actor*>(a_ref),
			.object = a_object,
			.queue3DTasks = a_queue3DTasks,
		});
	}

	void HookedActorOnHeadInitialized(RE::TESObjectREFR* a_ref)
	{
		OriginalActorOnHeadInitialized(a_ref);
		SeedFaceGenActor(a_ref);
		DiscardPendingPreparedHeadPartEvents(a_ref ? a_ref->GetFaceNodeSkinned() : nullptr);
		EmitEvent({
			.type = Smp::LifecycleEventType::kActorHeadInitialized,
			.actor = static_cast<RE::Actor*>(a_ref),
			.object = a_ref ? reinterpret_cast<RE::NiAVObject*>(a_ref->GetFaceNodeSkinned()) : nullptr,
		});
	}

	void HookedPlayerCharacterOnHeadInitialized(RE::TESObjectREFR* a_ref)
	{
		OriginalPlayerCharacterOnHeadInitialized(a_ref);
		SeedFaceGenActor(a_ref);
		DiscardPendingPreparedHeadPartEvents(a_ref ? a_ref->GetFaceNodeSkinned() : nullptr);
		EmitEvent({
			.type = Smp::LifecycleEventType::kActorHeadInitialized,
			.actor = static_cast<RE::Actor*>(a_ref),
			.object = a_ref ? reinterpret_cast<RE::NiAVObject*>(a_ref->GetFaceNodeSkinned()) : nullptr,
		});
	}

	void HookedPrepareHeadPart(RE::BSFaceGenNiNode* a_faceNode, RE::BGSHeadPart* a_headPart, const RE::TESNPC* a_npc, bool a_arg4)
	{
		OriginalPrepareHeadPart(a_faceNode, a_headPart, a_npc, a_arg4);

		auto* actor = ResolveFaceGenActor(a_faceNode);
		auto* preparedObject = FindPreparedHeadPartObject(a_faceNode, a_headPart);
		if (!actor) {
			std::scoped_lock lock(FaceGenActorLock);
			PendingPreparedHeadParts[a_faceNode].push_back({
				.object = preparedObject,
				.headPart = a_headPart,
			});
			spdlog::debug(
				"queued prepared headpart pending actor resolution faceNode={} object={} headPart={} model='{}'",
				static_cast<void*>(a_faceNode),
				static_cast<void*>(preparedObject),
				static_cast<void*>(a_headPart),
				a_headPart ? a_headPart->GetModel() : "");
			return;
		}

		EmitPreparedHeadPartEvent(actor, a_faceNode, preparedObject, a_headPart);
	}

	void HookedSetFaceGenBoneName(void* a_fmd, std::uint32_t a_boneIdx, RE::BSFixedString* a_boneName)
	{
		if (a_boneIdx < kFaceGenModelExtraDataBoneNameLimit) {
			OriginalSetFaceGenBoneName(a_fmd, a_boneIdx, a_boneName);
			return;
		}

		spdlog::debug(
			"dropped out-of-range BSFaceGenModelExtraData::SetBoneName write fmd={} boneIdx={} boneName={}",
			a_fmd,
			a_boneIdx,
			a_boneName && a_boneName->c_str() ? a_boneName->c_str() : "");
	}

	void HookedMainOnIdleUpdateHighActorsArraySorted(RE::Main* a_main, float a_distance)
	{
		OriginalMainOnIdleUpdateHighActorsArraySorted(a_main, a_distance);
		Smp::Fo4PhysicsWorld::GetSingleton()->StepFrame();
	}

	void HookedMainSwap(RE::Main* a_main)
	{
		Smp::Fo4PhysicsWorld::GetSingleton()->WriteBackPrototypeBodies(Smp::WritebackSource::kMainSync);
		OriginalMainSwap(a_main);
	}

	void HookedUpdate3DModel(void* a_middleProcess, RE::Actor* a_actor, bool a_flag)
	{
		OriginalUpdate3DModel(a_middleProcess, a_actor, a_flag);
		EmitEvent({
			.type = Smp::LifecycleEventType::kActorUpdate3DModel,
			.actor = a_actor,
			.object = a_actor ? a_actor->Get3D() : nullptr,
		});
	}

	void HookedReset3D(RE::Actor* a_actor, bool a_reloadAll, std::uint32_t a_additionalFlags, bool a_queueReset, std::uint32_t a_excludeFlags)
	{
		OriginalReset3D(a_actor, a_reloadAll, a_additionalFlags, a_queueReset, a_excludeFlags);
		EmitEvent({
			.type = Smp::LifecycleEventType::kActorReset3D,
			.actor = a_actor,
			.object = a_actor ? a_actor->Get3D() : nullptr,
			.queue3DTasks = a_queueReset,
		});
	}

	bool InstallLifecycleHooks()
	{
		LogHookTargets();
		const auto isOG = REX::FModule::IsRuntimeOG();
		const auto faceGenBoneNameLimitsValid = ValidateFaceGenBoneNameLimit();
		const auto mainFrameCallsite = Addresses::MainOnIdle.address() + (isOG ? kMainOnIdleUpdateHighActorsArraySortedCallOffsetOG : kMainOnIdleUpdateHighActorsArraySortedCallOffsetAE);
		const auto mainSyncCallsite = Addresses::MainOnIdle.address() + (isOG ? kMainOnIdleSwapCallOffsetOG : kMainOnIdleSwapCallOffsetAE);
		LogRelocationTarget("Main::OnIdle frame update callsite", mainFrameCallsite);
		LogRelocationTarget("Main::OnIdle frame sync callsite", mainSyncCallsite);

		if (isOG && !OriginalMainOnIdleUpdateHighActorsArraySorted) {
			OriginalMainOnIdleUpdateHighActorsArraySorted = reinterpret_cast<MainOnIdleUpdateHighActorsArraySorted_t>(
				REL::GetTrampoline().write_call<5>(mainFrameCallsite, reinterpret_cast<std::uintptr_t>(&HookedMainOnIdleUpdateHighActorsArraySorted)));
			spdlog::info("Main::OnIdle frame update call hook installed at {:x}", mainFrameCallsite);
		}
		if (isOG && !OriginalMainSwap) {
			OriginalMainSwap = reinterpret_cast<MainSwap_t>(
				REL::GetTrampoline().write_call<5>(mainSyncCallsite, reinterpret_cast<std::uintptr_t>(&HookedMainSwap)));
			spdlog::info("Main::OnIdle frame sync call hook installed at {:x}", mainSyncCallsite);
		}
		if (isOG && !OriginalBipedAnimApplySkinnedObjects) {
			OriginalBipedAnimApplySkinnedObjects = CreateBranchGateway5<BipedAnimApplySkinnedObjects_t>("BipedAnim::ApplySkinnedObjects", Addresses::BipedAnimApplySkinnedObjects, kApplySkinnedObjectsPrologueSize, reinterpret_cast<void*>(&HookedBipedAnimApplySkinnedObjects));
		}
		if (!OriginalBipedAnimAttachSkinnedObject) {
			OriginalBipedAnimAttachSkinnedObject = CreateBranchGateway5<BipedAnimAttachSkinnedObject_t>("BipedAnim::AttachSkinnedObject", Addresses::BipedAnimAttachSkinnedObject, kAttachSkinnedObjectPrologueSize, reinterpret_cast<void*>(&HookedBipedAnimAttachSkinnedObject));
		}
		if (!OriginalBipedAnimAttachToParent) {
			OriginalBipedAnimAttachToParent = CreateBranchGateway5<BipedAnimAttachToParent_t>("BipedAnim::AttachToParent", Addresses::BipedAnimAttachToParent, kAttachToParentPrologueSize, reinterpret_cast<void*>(&HookedBipedAnimAttachToParent));
		}
		if (!OriginalBipedAnimRemovePart) {
			OriginalBipedAnimRemovePart = CreateBranchGateway5<BipedAnimRemovePart_t>("BipedAnim::RemovePart", Addresses::BipedAnimRemovePart, kRemovePartPrologueSize, reinterpret_cast<void*>(&HookedBipedAnimRemovePart));
		}

		REL::Relocation<std::uintptr_t> actorVTable{ RE::VTABLE::Actor[0] };
		REL::Relocation<std::uintptr_t> playerVTable{ RE::VTABLE::PlayerCharacter[0] };

		if (!OriginalActorLoad3D) {
			OriginalActorLoad3D = InstallVFuncHook<ActorLoad3D_t>("Actor::Load3D", actorVTable, 0x86, reinterpret_cast<void*>(&HookedActorLoad3D));
		}
		if (!OriginalPlayerCharacterLoad3D) {
			OriginalPlayerCharacterLoad3D = InstallVFuncHook<ActorLoad3D_t>("PlayerCharacter::Load3D", playerVTable, 0x86, reinterpret_cast<void*>(&HookedPlayerCharacterLoad3D));
		}
		if (!OriginalActorSet3D) {
			OriginalActorSet3D = InstallVFuncHook<Set3D_t>("Actor::Set3D", actorVTable, 0x88, reinterpret_cast<void*>(&HookedActorSet3D));
		}
		if (!OriginalPlayerCharacterSet3D) {
			OriginalPlayerCharacterSet3D = InstallVFuncHook<Set3D_t>("PlayerCharacter::Set3D", playerVTable, 0x88, reinterpret_cast<void*>(&HookedPlayerCharacterSet3D));
		}
		if (!OriginalActorOnHeadInitialized) {
			OriginalActorOnHeadInitialized = InstallVFuncHook<OnHeadInitialized_t>("Actor::OnHeadInitialized", actorVTable, 0x98, reinterpret_cast<void*>(&HookedActorOnHeadInitialized));
		}
		if (!OriginalPlayerCharacterOnHeadInitialized) {
			OriginalPlayerCharacterOnHeadInitialized = InstallVFuncHook<OnHeadInitialized_t>("PlayerCharacter::OnHeadInitialized", playerVTable, 0x98, reinterpret_cast<void*>(&HookedPlayerCharacterOnHeadInitialized));
		}

		if (!OriginalUpdate3DModel) {
			OriginalUpdate3DModel = CreateBranchGateway5<Update3DModel_t>("AIProcess::Update3DModel", Addresses::Update3DModel, kUpdate3DModelPrologueSize, reinterpret_cast<void*>(&HookedUpdate3DModel));
		}
		if (!OriginalReset3D) {
			OriginalReset3D = CreateBranchGateway5<Reset3D_t>("Actor::Reset3D", Addresses::Reset3D, kReset3DPrologueSize, reinterpret_cast<void*>(&HookedReset3D));
		}
		if (isOG && !OriginalSetFaceGenBoneName) {
			OriginalSetFaceGenBoneName = reinterpret_cast<SetFaceGenBoneName_t>(Addresses::BSFaceGenModelExtraDataSetBoneName.address());
			DetourTransactionBegin();
			DetourUpdateThread(GetCurrentThread());
			const auto detourError = DetourAttach(
				reinterpret_cast<PVOID*>(std::addressof(OriginalSetFaceGenBoneName)),
				reinterpret_cast<PVOID>(&HookedSetFaceGenBoneName));
			const auto commitError = DetourTransactionCommit();
			if (detourError != NO_ERROR || commitError != NO_ERROR) {
				spdlog::error(
					"BSFaceGenModelExtraData::SetBoneName detour failed attachError={} commitError={} target={}",
					detourError,
					commitError,
					reinterpret_cast<void*>(Addresses::BSFaceGenModelExtraDataSetBoneName.address()));
				OriginalSetFaceGenBoneName = nullptr;
			} else {
				spdlog::info("BSFaceGenModelExtraData::SetBoneName detour installed at {:x}", Addresses::BSFaceGenModelExtraDataSetBoneName.address());
			}
		}
		if constexpr (kEnablePrepareHeadPartHook) {
			if (!OriginalPrepareHeadPart) {
				OriginalPrepareHeadPart = reinterpret_cast<PrepareHeadPart_t>(Addresses::BSFaceGenPrepareHeadPart.address());
				DetourTransactionBegin();
				DetourUpdateThread(GetCurrentThread());
				const auto detourError = DetourAttach(
					reinterpret_cast<PVOID*>(std::addressof(OriginalPrepareHeadPart)),
					reinterpret_cast<PVOID>(&HookedPrepareHeadPart));
				const auto commitError = DetourTransactionCommit();
				if (detourError != NO_ERROR || commitError != NO_ERROR) {
					spdlog::error(
						"BSFaceGenUtils::PrepareHeadPart detour failed attachError={} commitError={} target={}",
						detourError,
						commitError,
						reinterpret_cast<void*>(Addresses::BSFaceGenPrepareHeadPart.address()));
					OriginalPrepareHeadPart = nullptr;
				} else {
					spdlog::info("BSFaceGenUtils::PrepareHeadPart detour installed at {:x}", Addresses::BSFaceGenPrepareHeadPart.address());
				}
			}
		} else {
			spdlog::warn("BSFaceGenUtils::PrepareHeadPart detour disabled pending FaceGen crash verification");
		}

		const bool installed =
			(!isOG || (OriginalMainOnIdleUpdateHighActorsArraySorted && OriginalMainSwap)) &&
			(!isOG || OriginalBipedAnimApplySkinnedObjects) &&
			OriginalBipedAnimAttachSkinnedObject &&
			OriginalBipedAnimAttachToParent &&
			OriginalBipedAnimRemovePart &&
			OriginalActorLoad3D &&
			OriginalPlayerCharacterLoad3D &&
			OriginalActorSet3D &&
			OriginalPlayerCharacterSet3D &&
			OriginalActorOnHeadInitialized &&
			OriginalPlayerCharacterOnHeadInitialized &&
			OriginalUpdate3DModel &&
			OriginalReset3D &&
			faceGenBoneNameLimitsValid &&
			(!isOG || OriginalSetFaceGenBoneName) &&
			(!kEnablePrepareHeadPartHook || OriginalPrepareHeadPart);

		spdlog::info("FO4 Faster HDT-SMP lifecycle hooks {}", installed ? "installed" : "failed");
		return installed;
	}
}
