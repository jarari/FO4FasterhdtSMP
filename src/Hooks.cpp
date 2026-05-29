#include "Hooks.h"

#include "BSSkin.h"
#include "ConfigPaths.h"
#include "Fo4PhysicsWorld.h"
#include "LifecycleEvents.h"
#include "PhysicsName.h"
#include "SmpConfig.h"
#include "RE/B/BSGeometry.h"
#include "RE/M/Main.h"
#include "RE/N/NiCloningProcess.h"
#include "RE/N/NiNode.h"
#include "RE/N/NiPointer.h"
#include "RE/N/NiStringExtraData.h"
#include "RE/T/TESObjectREFR.h"

#include <optional>
#include <atomic>
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

	std::mutex               BackupNodeLock;
	std::vector<std::string> BackupNodeNames;
	thread_local std::uint32_t ApplySkinnedObjectsDepth{ 0 };
	std::atomic<std::uint32_t> PreMergeRenameId{ 0xF0000000U };
	constexpr std::string_view kPhysicsXmlExtraName = "HDT Skinned Mesh Physics Object";

	using BackupBoneMap = std::unordered_map<std::string, std::vector<RE::NiPointer<RE::NiAVObject>>>;

	RE::Actor* ResolveActor(RE::BipedAnim* a_biped);
	void LogApplySourceNode(const char* a_phase, RE::NiAVObject* a_root, std::string_view a_name);

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

	std::string MakeReferenceArmorRenamePrefix(const std::uint32_t a_id)
	{
		char buffer[48]{};
		std::snprintf(buffer, sizeof(buffer), "hdtSSEPhysics_AutoRename_Armor_%08X ", a_id);
		return buffer;
	}

	RE::NiNode* FindNodeByName(RE::NiAVObject* a_root, const std::string_view a_name)
	{
		if (!a_root || a_name.empty()) {
			return nullptr;
		}

		if (const auto name = a_root->GetName(); !name.empty() && Smp::PhysicsNamesEqual(name, a_name)) {
			return a_root->IsNode();
		}

		auto* node = a_root->IsNode();
		if (!node) {
			return nullptr;
		}

		for (auto& child : node->children) {
			if (auto* found = FindNodeByName(child.get(), a_name)) {
				return found;
			}
		}

		return nullptr;
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

	void UpdateNodeWorldFromLocal(RE::NiNode* a_node)
	{
		if (!a_node) {
			return;
		}

		if (a_node->parent) {
			a_node->world = a_node->parent->world * a_node->local;
		} else {
			a_node->world = a_node->local;
		}

		for (auto& child : a_node->children) {
			if (auto* childNode = child ? child->IsNode() : nullptr) {
				UpdateNodeWorldFromLocal(childNode);
			}
		}
	}

	void RenameMergedNodeTree(RE::NiNode* a_node, const std::string& a_prefix, std::vector<Smp::LifecycleMergedSkeletonNode>* a_renamedNodes)
	{
		if (!a_node) {
			return;
		}

		const auto originalName = a_node->GetName();
		if (!originalName.empty()) {
			auto renamed = a_prefix;
			renamed += std::string_view(originalName);
			if (a_renamedNodes) {
				a_renamedNodes->push_back({
					.originalName = std::string(originalName),
					.renamedName = renamed,
					.node = a_node,
				});
			}
			a_node->name = renamed.c_str();
		}

		for (auto& child : a_node->children) {
			if (auto* childNode = child ? child->IsNode() : nullptr) {
				RenameMergedNodeTree(childNode, a_prefix, a_renamedNodes);
			}
		}
	}

	RE::NiNode* CloneMergedNodeTree(RE::NiNode* a_source, const std::string& a_prefix, std::vector<Smp::LifecycleMergedSkeletonNode>& a_renamedNodes)
	{
		if (!a_source) {
			return nullptr;
		}

		RE::NiCloningProcess cloneProcess;
		cloneProcess.appendChar = '$';
		cloneProcess.copyType = RE::NiCloningProcess::CopyType::kCopyExact;
		cloneProcess.scale = { 1.0F, 1.0F, 1.0F };

		auto* cloneObject = a_source->CreateClone(cloneProcess);
		a_source->ProcessClone(cloneProcess);
		auto* cloneNode = cloneObject ? cloneObject->IsNode() : nullptr;
		if (!cloneNode) {
			return nullptr;
		}

		RenameMergedNodeTree(a_source, a_prefix, nullptr);
		RenameMergedNodeTree(cloneNode, a_prefix, std::addressof(a_renamedNodes));
		return cloneNode;
	}

	void MergeSourceSkeletonIntoActorPreApply(
		RE::NiNode* a_destination,
		RE::NiNode* a_source,
		RE::NiAVObject* a_destinationRoot,
		const std::string& a_prefix,
		std::vector<Smp::LifecycleMergedSkeletonNode>& a_renamedNodes,
		std::vector<Smp::LifecycleMergedRootNode>& a_mergedRoots)
	{
		if (!a_destination || !a_source || !a_destinationRoot) {
			return;
		}

		for (auto& child : a_source->children) {
			auto* sourceChild = child ? child->IsNode() : nullptr;
			if (!sourceChild) {
				continue;
			}

			const auto sourceName = sourceChild->GetName();
			if (sourceName.empty()) {
				MergeSourceSkeletonIntoActorPreApply(a_destination, sourceChild, a_destinationRoot, a_prefix, a_renamedNodes, a_mergedRoots);
				continue;
			}

			if (auto* destinationChild = FindNodeByName(a_destinationRoot, sourceName)) {
				MergeSourceSkeletonIntoActorPreApply(destinationChild, sourceChild, a_destinationRoot, a_prefix, a_renamedNodes, a_mergedRoots);
				continue;
			}

			auto* clonedChild = CloneMergedNodeTree(sourceChild, a_prefix, a_renamedNodes);
			if (!clonedChild) {
				continue;
			}

			a_destination->AttachChild(clonedChild, false);
			UpdateNodeWorldFromLocal(clonedChild);
			a_mergedRoots.push_back({
				.parent = a_destination,
				.node = clonedChild,
			});

			if (Smp::PhysicsNamesEqual(sourceName, "InariTail_01")) {
				LogApplySourceNode("ArmorApplySkinnedObjects pre-merge clone", clonedChild, sourceName);
			}
			spdlog::debug(
				"pre-merged source skeleton node '{}' as renamed attachment node={} under parent={}",
				sourceName,
				static_cast<void*>(clonedChild),
				static_cast<void*>(a_destination));
		}
	}

	void PreMergeArmorSkeleton(RE::BipedAnim* a_biped, RE::NiNode* a_sourceRoot, const bool a_firstPerson, std::vector<Smp::LifecycleMergedSkeletonNode>& a_renamedNodes, std::vector<Smp::LifecycleMergedRootNode>& a_mergedRoots)
	{
		auto* actor = ResolveActor(a_biped);
		auto* actorRoot = actor ? actor->Get3D(a_firstPerson) : nullptr;
		if (!actorRoot && actor) {
			actorRoot = actor->Get3D();
		}
		auto* actorRootNode = actorRoot ? actorRoot->IsNode() : nullptr;
		if (!actorRootNode || !a_sourceRoot) {
			return;
		}

		const auto prefix = MakeReferenceArmorRenamePrefix(PreMergeRenameId.fetch_add(1, std::memory_order_relaxed));
		UpdateNodeWorldFromLocal(actorRootNode);
		UpdateNodeWorldFromLocal(a_sourceRoot);
		MergeSourceSkeletonIntoActorPreApply(actorRootNode, a_sourceRoot, actorRootNode, prefix, a_renamedNodes, a_mergedRoots);
		spdlog::debug(
			"pre-merged armor skeleton before ApplySkinnedObjects actor={} source={} root={} renamedNodes={} mergedRoots={} prefix='{}'",
			static_cast<void*>(actor),
			static_cast<void*>(a_sourceRoot),
			static_cast<void*>(actorRootNode),
			a_renamedNodes.size(),
			a_mergedRoots.size(),
			prefix);
	}

	void LogApplySourceNode(const char* a_phase, RE::NiAVObject* a_root, const std::string_view a_name)
	{
		auto* node = FindNodeByName(a_root, a_name);
		if (!node) {
			spdlog::info("{} source node '{}' missing root={}", a_phase, a_name, static_cast<void*>(a_root));
			return;
		}

		auto* parent = node->parent;
		const auto parentName = parent ? parent->GetName() : "";
		const auto& local = node->local.translate;
		const auto& world = node->world.translate;
		spdlog::info(
			"{} source node '{}' node={} parent={} parentName='{}' local=({:.3f},{:.3f},{:.3f}) world=({:.3f},{:.3f},{:.3f})",
			a_phase,
			a_name,
			static_cast<void*>(node),
			static_cast<void*>(parent),
			parentName,
			local.x,
			local.y,
			local.z,
			world.x,
			world.y,
			world.z);
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

	void LogHookTargets()
	{
		LogRelocationTarget("Main::OnIdle", Addresses::MainOnIdle.address());
		LogRelocationTarget("BipedAnim::ApplySkinnedObjects", Addresses::BipedAnimApplySkinnedObjects.address());
		LogRelocationTarget("BipedAnim::AttachSkinnedObject", Addresses::BipedAnimAttachSkinnedObject.address());
		LogRelocationTarget("BipedAnim::AttachToParent", Addresses::BipedAnimAttachToParent.address());
		LogRelocationTarget("BipedAnim::RemovePart", Addresses::BipedAnimRemovePart.address());
		LogRelocationTarget("AIProcess::Update3DModel", Addresses::Update3DModel.address());
		LogRelocationTarget("Actor::Reset3D", Addresses::Reset3D.address());
	}

	void EmitEvent(const Smp::LifecycleEvent& a_event)
	{
		const auto highFrequency =
			a_event.type == Smp::LifecycleEventType::kActorUpdate3DModel ||
			(a_event.type == Smp::LifecycleEventType::kActorSet3D && !a_event.object) ||
			((a_event.type == Smp::LifecycleEventType::kArmorDetachBegin || a_event.type == Smp::LifecycleEventType::kArmorDetachEnd) &&
				!a_event.actor &&
				!a_event.object);
		const auto level = highFrequency ? spdlog::level::trace : spdlog::level::debug;
		spdlog::log(
			level,
			"SMP lifecycle {} actor={} biped={} object={} source={} xml='{}'",
			Smp::ToString(a_event.type),
			static_cast<void*>(a_event.actor),
			static_cast<void*>(a_event.biped),
			static_cast<void*>(a_event.object),
			static_cast<void*>(a_event.sourceObject),
			a_event.physicsXmlPath);

		Smp::NotifyLifecycleEvent(a_event);
	}

	RE::NiAVObject* HookedBipedAnimApplySkinnedObjects(RE::BipedAnim* a_biped, RE::NiNode* a_originalModelRoot, RE::BIPED_OBJECT a_bipedObject, bool a_firstPerson)
	{
		auto* originalModelObject = a_originalModelRoot ? static_cast<RE::NiAVObject*>(a_originalModelRoot) : nullptr;
		auto selectedXml = FindPhysicsXmlExtraData(originalModelObject);
		std::vector<RE::NiAVObject*> mergeSearchExclusions;
		std::vector<Smp::LifecycleMergedSkeletonNode> preMergedSkeletonNodes;
		std::vector<Smp::LifecycleMergedRootNode> preMergedRootNodes;
		if (selectedXml) {
			CollectNodePointers(originalModelObject, mergeSearchExclusions);
			spdlog::debug(
				"pre-scanned armor physics XML {} from original model root={} name='{}'",
				*selectedXml,
				static_cast<void*>(a_originalModelRoot),
				a_originalModelRoot ? std::string_view(a_originalModelRoot->GetName()) : std::string_view{});
			LogApplySourceNode("ArmorApplySkinnedObjects pre-read original", originalModelObject, "Root");
			LogApplySourceNode("ArmorApplySkinnedObjects pre-read original", originalModelObject, "Pelvis");
			LogApplySourceNode("ArmorApplySkinnedObjects pre-read original", originalModelObject, "InariTail_01");
			LogApplySourceNode("ArmorApplySkinnedObjects pre-read original", originalModelObject, "InariTail_02");
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
			LogApplySourceNode("ArmorApplySkinnedObjects preserved clone", mergeSourceObject.get(), "Root");
			LogApplySourceNode("ArmorApplySkinnedObjects preserved clone", mergeSourceObject.get(), "Pelvis");
			LogApplySourceNode("ArmorApplySkinnedObjects preserved clone", mergeSourceObject.get(), "InariTail_01");
			LogApplySourceNode("ArmorApplySkinnedObjects preserved clone", mergeSourceObject.get(), "InariTail_02");
		}
		if (selectedXml) {
			PreMergeArmorSkeleton(a_biped, a_originalModelRoot, a_firstPerson, preMergedSkeletonNodes, preMergedRootNodes);
			LogApplySourceNode("ArmorApplySkinnedObjects post-pre-merge original", originalModelObject, "InariTail_01");
			LogApplySourceNode("ArmorApplySkinnedObjects post-pre-merge original", originalModelObject, "InariTail_02");
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
			.mergeSearchExclusions = std::move(mergeSearchExclusions),
			.preMergedSkeletonNodes = std::move(preMergedSkeletonNodes),
			.preMergedRootNodes = std::move(preMergedRootNodes),
			.physicsXmlPath = selectedXml.value_or(std::string{}),
			.firstPerson = a_firstPerson,
		});
		return attachedObject;
	}

	RE::NiAVObject* HookedBipedAnimAttachSkinnedObject(RE::BipedAnim* a_biped, RE::NiNode* a_destinationRoot, RE::NiNode* a_sourceRoot, RE::BIPED_OBJECT a_bipedObject, bool a_firstPerson)
	{
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
			.destinationRoot = a_destinationRoot,
			.sourceRoot = a_sourceRoot,
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
		EmitEvent({
			.type = Smp::LifecycleEventType::kArmorDetachBegin,
			.actor = ResolveActor(a_biped),
			.biped = a_biped,
			.bipObject = a_bipObject,
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
		EmitEvent({
			.type = Smp::LifecycleEventType::kActorHeadInitialized,
			.actor = static_cast<RE::Actor*>(a_ref),
			.object = a_ref ? reinterpret_cast<RE::NiAVObject*>(a_ref->GetFaceNodeSkinned()) : nullptr,
		});
	}

	void HookedPlayerCharacterOnHeadInitialized(RE::TESObjectREFR* a_ref)
	{
		OriginalPlayerCharacterOnHeadInitialized(a_ref);
		EmitEvent({
			.type = Smp::LifecycleEventType::kActorHeadInitialized,
			.actor = static_cast<RE::Actor*>(a_ref),
			.object = a_ref ? reinterpret_cast<RE::NiAVObject*>(a_ref->GetFaceNodeSkinned()) : nullptr,
		});
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
			OriginalReset3D;

		spdlog::info("FO4 Faster HDT-SMP lifecycle hooks {}", installed ? "installed" : "failed");
		return installed;
	}
}
