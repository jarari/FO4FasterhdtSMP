#include "Hooks.h"

#include "Address.h"
#include "ArmorBoneReference.h"
#include "Fo4PhysicsWorld.h"
#include "ImguiLayer.h"
#include "LifecycleEvents.h"
#include "PhysicsXmlSelection.h"
#include "RE/B/BSAnimationGraphManager.h"
#include "RE/B/BGSHeadPart.h"
#include "RE/B/BSGeometry.h"
#include "RE/B/BSModelDB.h"
#include "RE/H/hkArray.h"
#include "RE/H/hkReferencedObject.h"
#include "RE/H/hkRefPtr.h"
#include "RE/M/Main.h"
#include "RE/N/NiNode.h"
#include "RE/P/PlayerCharacter.h"
#include "RE/N/NiStringExtraData.h"
#include "RE/T/TESObjectREFR.h"
#include "RE/T/TESNPC.h"

#include <cctype>
#include <cstddef>
#include <cstring>
#include <initializer_list>
#include <limits>
#include <memory>
#include <optional>
#include <unordered_map>
#include <utility>

namespace Hooks
{
	namespace Address = Smp::Address;

	using MainSwap_t = void (*)(RE::Main*);
	using BipedAnimApplySkinnedObjects_t = RE::NiAVObject* (*)(RE::BipedAnim*, RE::NiNode*, RE::BIPED_OBJECT, bool);
	using BipedAnimAttachSkinnedObject_t = RE::NiAVObject* (*)(RE::BipedAnim*, RE::NiNode*, RE::NiNode*, RE::BIPED_OBJECT, bool);
	using BipedAnimAttachToParent_t = void (*)(RE::NiAVObject*, RE::NiAVObject*, RE::NiAVObject*, RE::BSTSmartPointer<RE::BipedAnim>&, RE::BIPED_OBJECT);
	using BipedAnimRemovePart_t = void (*)(RE::BipedAnim*, RE::BIPOBJECT*, bool);
	using TESObjectREFRFixDisplayedHeadParts_t = void (*)(RE::TESObjectREFR*, RE::BSFaceGenNiNode*, bool, bool);
	using ActorLoad3D_t = RE::NiAVObject* (*)(RE::TESObjectREFR*, bool);
	using Set3D_t = void (*)(RE::TESObjectREFR*, RE::NiAVObject*, bool);
	using OnHeadInitialized_t = void (*)(RE::TESObjectREFR*);
	using Reset3D_t = void (*)(RE::Actor*, bool, std::uint32_t, bool, std::uint32_t);
	using FaceGenSkinAllGeometry_t = void (*)(RE::BSFaceGenNiNode*, RE::NiNode*, bool);
	using FaceGenSkinSingleGeometry_t = void (*)(RE::BSFaceGenNiNode*, RE::NiNode*, RE::BSGeometry*, bool);
	using FaceGenPrepareHeadPart_t = void (*)(RE::BSFaceGenNiNode*, RE::BGSHeadPart*, RE::TESNPC*, bool, bool);
	using BakeChargenMorphs_t = void (*)(RE::TESNPC*, RE::NiNode*, RE::NiNode*, void*);
	using SetFaceGenBoneName_t = void (*)(void*, std::uint32_t, RE::BSFixedString*);
	using LooksMenuUtilsShowLooksMenu_t = void (*)(RE::TESObjectREFR*, std::uint32_t, RE::TESObjectREFR*, RE::TESObjectREFR*, RE::TESObjectREFR*);

	MainSwap_t                     OriginalMainSwap{ nullptr };
	std::uintptr_t                 OriginalFaceGenManagerUpdate{ 0 };
	BipedAnimApplySkinnedObjects_t OriginalBipedAnimApplySkinnedObjects{ nullptr };
	BipedAnimAttachSkinnedObject_t OriginalBipedAnimAttachSkinnedObject{ nullptr };
	BipedAnimAttachToParent_t      OriginalBipedAnimAttachToParent{ nullptr };
	BipedAnimRemovePart_t          OriginalBipedAnimRemovePart{ nullptr };
	TESObjectREFRFixDisplayedHeadParts_t OriginalTESObjectREFRFixDisplayedHeadParts{ nullptr };
	ActorLoad3D_t                  OriginalActorLoad3D{ nullptr };
	ActorLoad3D_t                  OriginalPlayerCharacterLoad3D{ nullptr };
	Set3D_t                        OriginalActorSet3D{ nullptr };
	Set3D_t                        OriginalPlayerCharacterSet3D{ nullptr };
	OnHeadInitialized_t            OriginalActorOnHeadInitialized{ nullptr };
	OnHeadInitialized_t            OriginalPlayerCharacterOnHeadInitialized{ nullptr };
	Reset3D_t                      OriginalReset3D{ nullptr };
	FaceGenSkinAllGeometry_t       OriginalFaceGenSkinAllGeometry{ nullptr };
	FaceGenSkinSingleGeometry_t    OriginalFaceGenSkinSingleGeometry{ nullptr };
	FaceGenPrepareHeadPart_t       OriginalFaceGenPrepareHeadPart{ nullptr };
	BakeChargenMorphs_t            OriginalArmorBakeChargenMorphs{ nullptr };
	BakeChargenMorphs_t            OriginalHeadBakeChargenMorphs{ nullptr };
	SetFaceGenBoneName_t           OriginalSetFaceGenBoneName{ nullptr };
	LooksMenuUtilsShowLooksMenu_t  OriginalLooksMenuUtilsShowLooksMenu{ nullptr };

	inline constexpr std::uint32_t kFaceGenModelExtraDataBoneNameLimit = 0x80;
	inline constexpr std::size_t kFaceGenQueuePointerOffset = 0x3100;
	inline constexpr std::size_t kFaceGenArenaCountOffset = 0x31B0;
	inline constexpr std::size_t kFaceGenStagedDataOffset = 0x31F0;
	inline constexpr std::size_t kFaceGenStagedNpcOffset = 0x31F8;
	inline constexpr std::size_t kFaceGenActiveStateOffset = 0x34A0;
	inline constexpr std::size_t kFaceGenPreloadIndexOffset = 0x3498;
	inline constexpr std::size_t kFaceGenQueueReadIndexOffset = 0x2400;
	inline constexpr std::size_t kFaceGenQueueWriteIndexOffset = 0x2500;

	template <class T>
	T ReadFaceGenField(const void* a_base, const std::size_t a_offset)
	{
		T value{};
		std::memcpy(std::addressof(value), static_cast<const std::byte*>(a_base) + a_offset, sizeof(value));
		return value;
	}

	bool IsFaceGenManagerIdle(const void* a_manager)
	{
		if (!a_manager) {
			return false;
		}

		// These fields and the lockless queue indices have the same layout in the
		// verified OG and AE implementations of UpdatePendingCustomizationTextures.
		const auto queue = ReadFaceGenField<std::uintptr_t>(a_manager, kFaceGenQueuePointerOffset);
		if (!queue) {
			return false;
		}

		const auto queueBase = reinterpret_cast<const void*>(queue);
		const auto readIndex = ReadFaceGenField<std::uint32_t>(queueBase, kFaceGenQueueReadIndexOffset) & 0xFF;
		const auto writeIndex = ReadFaceGenField<std::uint32_t>(queueBase, kFaceGenQueueWriteIndexOffset) & 0xFF;
		const auto queueEmpty = readIndex == writeIndex;
		const auto noCompleteStagedData =
			ReadFaceGenField<std::uintptr_t>(a_manager, kFaceGenStagedDataOffset) == 0 ||
			ReadFaceGenField<std::uintptr_t>(a_manager, kFaceGenStagedNpcOffset) == 0;

		return ReadFaceGenField<std::uint32_t>(a_manager, kFaceGenActiveStateOffset) == 0 &&
		       ReadFaceGenField<std::uint32_t>(a_manager, kFaceGenArenaCountOffset) == 0 &&
		       ReadFaceGenField<std::uint32_t>(a_manager, kFaceGenPreloadIndexOffset) == std::numeric_limits<std::uint32_t>::max() &&
		       noCompleteStagedData && queueEmpty;
	}

	thread_local std::uint32_t ApplySkinnedObjectsDepth{ 0 };
	thread_local RE::NiNode* ApplySkinnedObjectsSkeletonRoot{ nullptr };
	struct IncrementalHeadPartContext
	{
		RE::BSFaceGenNiNode* faceNode{ nullptr };
		RE::NiPointer<RE::NiNode> mainRoot;
		std::string nifPath;
		bool usesMainModel{ false };
		bool isExtraPart{ false };
	};
	thread_local IncrementalHeadPartContext PendingIncrementalHeadPart;
	std::mutex FaceGenActorLock;
	std::unordered_map<RE::BSFaceGenNiNode*, RE::ActorHandle> FaceGenActorMap;
	RE::Actor* ResolveActor(RE::BipedAnim* a_biped);

	RE::Actor* AsActor(RE::TESObjectREFR* a_ref)
	{
		if (!a_ref) {
			return nullptr;
		}
		if (a_ref == RE::PlayerCharacter::GetSingleton()) {
			return static_cast<RE::Actor*>(a_ref);
		}
		return a_ref->IsActor() ? static_cast<RE::Actor*>(a_ref) : nullptr;
	}

	bool IsBipedObjectSlot(const RE::BIPED_OBJECT a_bipedObject)
	{
		const auto index = std::to_underlying(a_bipedObject);
		return index >= 0 && index < std::to_underlying(RE::BIPED_OBJECT::kTotal);
	}

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
		auto path = Smp::PhysicsXmlSelection::FindDirectPhysicsXmlExtraData(
			a_object,
			Smp::PhysicsXmlSelection::DirectXmlLogContext::kOriginalModelObject);
		if (!path) {
			return std::nullopt;
		}
		return path->string();
	}

	struct PreAttachPhysicsContext
	{
		std::optional<std::string> selectedXml;
		std::vector<Smp::ArmorBoneReference> armorBoneReferences;
		std::vector<Smp::RetainedSkinBinding> mainSkinBindings;
	};

	bool UsesFaceBonesModel(const RE::BIPOBJECT* a_bipObject)
	{
		// BipedAnim::AttachSkinnedObject gates LoadFaceBonesModel and
		// BakeChargenMorphs on TESModel::flags bit 0.
		return a_bipObject &&
			a_bipObject->part &&
			(static_cast<std::uint8_t>(a_bipObject->part->flags) & 1U) != 0;
	}

	PreAttachPhysicsContext PreparePreAttachPhysicsContext(
		RE::NiAVObject* a_sourceObject,
		RE::NiAVObject* a_skeletonRoot,
		const std::string_view a_nifPath,
		const char* a_sourceLabel,
		const bool a_usesFaceBonesModel)
	{
		PreAttachPhysicsContext context;
		context.selectedXml = FindPhysicsXmlExtraData(a_sourceObject);
		if (context.selectedXml) {
			context.armorBoneReferences = Smp::CaptureArmorBoneReferences(
				a_sourceObject,
				a_skeletonRoot,
				a_nifPath,
				true);
		}
		if (context.selectedXml && a_usesFaceBonesModel) {
			context.mainSkinBindings = Smp::CaptureMainSkinBindings(a_sourceObject);
		}
		spdlog::debug(
			"pre-scanned armor source xml='{}' from {}={} name='{}' nif='{}' usesFaceBonesModel={} references={} mainSkinInstances={}",
			context.selectedXml.value_or(std::string{}),
			a_sourceLabel,
			static_cast<void*>(a_sourceObject),
			a_sourceObject ? std::string_view(a_sourceObject->GetName()) : std::string_view{},
			a_nifPath,
			a_usesFaceBonesModel,
			context.armorBoneReferences.size(),
			context.mainSkinBindings.size());

		return context;
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

	struct RipRel32Patch
	{
		std::size_t instructionOffset;
		std::size_t displacementOffset;
		std::size_t instructionSize;
	};

	std::int32_t MakeRel32Displacement(const std::uintptr_t a_sourceNext, const std::uintptr_t a_destination)
	{
		const auto displacement = static_cast<std::int64_t>(a_destination) - static_cast<std::int64_t>(a_sourceNext);
		if (displacement < (std::numeric_limits<std::int32_t>::min)() ||
			displacement > (std::numeric_limits<std::int32_t>::max)()) {
			REX::FAIL("rel32 displacement out of range sourceNext={:x} destination={:x}", a_sourceNext, a_destination);
		}

		return static_cast<std::int32_t>(displacement);
	}

	void WriteBranch5(const std::uintptr_t a_source, const std::uintptr_t a_destination)
	{
		auto& trampoline = REL::GetTrampoline();
		const auto branch = trampoline.allocate_branch5(a_destination);
		const REL::ASM::JMP5 assembly{ MakeRel32Displacement(a_source + sizeof(REL::ASM::JMP5), branch) };
		REL::WriteSafeData(a_source, assembly);
	}

	bool ReadExistingBranchTarget(const std::uintptr_t a_targetAddress, const std::byte* a_targetBytes, std::uintptr_t& a_branchTarget)
	{
		if (a_targetBytes[0] == std::byte{ 0xE9 }) {
			std::int32_t oldDisp = 0;
			std::memcpy(std::addressof(oldDisp), a_targetBytes + 1, sizeof(oldDisp));
			a_branchTarget = a_targetAddress + sizeof(REL::ASM::JMP5) + oldDisp;
			return a_branchTarget != 0;
		}

		if (a_targetBytes[0] == std::byte{ 0xFF } && a_targetBytes[1] == std::byte{ 0x25 }) {
			std::int32_t oldDisp = 0;
			std::memcpy(std::addressof(oldDisp), a_targetBytes + 2, sizeof(oldDisp));
			const auto indirectAddress = a_targetAddress + sizeof(REL::ASM::JMP6) + oldDisp;
			std::memcpy(std::addressof(a_branchTarget), reinterpret_cast<const void*>(indirectAddress), sizeof(a_branchTarget));
			return a_branchTarget != 0;
		}

		return false;
	}

	template <class T>
	T CreateBranchGateway5(
		const char* a_name,
		REL::Relocation<std::uintptr_t>& a_target,
		const std::size_t a_prologueSize,
		void* a_hook,
		std::initializer_list<RipRel32Patch> a_ripPatches = {})
	{
		const auto targetAddress = a_target.address();
		auto* targetBytes = reinterpret_cast<const std::byte*>(targetAddress);
		auto& trampoline = REL::GetTrampoline();

		std::uintptr_t existingBranchTarget = 0;
		if (ReadExistingBranchTarget(targetAddress, targetBytes, existingBranchTarget)) {
			auto* gateway = trampoline.allocate<REL::ASM::JMP14>(existingBranchTarget);
			WriteBranch5(targetAddress, reinterpret_cast<std::uintptr_t>(a_hook));
			spdlog::info("{} found existing branch at {:x}; chaining through {:x}", a_name, targetAddress, existingBranchTarget);
			return reinterpret_cast<T>(gateway);
		}

		auto* gateway = static_cast<std::byte*>(trampoline.allocate(a_prologueSize + sizeof(REL::ASM::JMP14)));
		std::memcpy(gateway, targetBytes, a_prologueSize);

		for (const auto& patch : a_ripPatches) {
			std::int32_t oldDisp = 0;
			std::memcpy(std::addressof(oldDisp), targetBytes + patch.displacementOffset, sizeof(oldDisp));
			const auto originalTarget = targetAddress + patch.instructionOffset + patch.instructionSize + oldDisp;
			const auto gatewayNext = reinterpret_cast<std::uintptr_t>(gateway) + patch.instructionOffset + patch.instructionSize;
			const auto newDisp = MakeRel32Displacement(gatewayNext, originalTarget);
			std::memcpy(gateway + patch.displacementOffset, std::addressof(newDisp), sizeof(newDisp));
		}

		const REL::ASM::JMP14 jumpBack{ targetAddress + a_prologueSize };
		std::memcpy(gateway + a_prologueSize, std::addressof(jumpBack), sizeof(jumpBack));
		WriteBranch5(targetAddress, reinterpret_cast<std::uintptr_t>(a_hook));
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
		LogRelocationTarget("Main::OnIdle", Address::MainOnIdle.address());
		LogRelocationTarget("BipedAnim::ApplySkinnedObjects", Address::BipedAnimApplySkinnedObjects.address());
		LogRelocationTarget("BipedAnim::AttachSkinnedObject", Address::BipedAnimAttachSkinnedObject.address());
		LogRelocationTarget("BipedAnim::AttachToParent", Address::BipedAnimAttachToParent.address());
		LogRelocationTarget("BipedAnim::RemovePart", Address::BipedAnimRemovePart.address());
		LogRelocationTarget("TESObjectREFR::FixDisplayedHeadParts", Address::TESObjectREFRFixDisplayedHeadParts.address());
		LogRelocationTarget("Actor::Reset3D", Address::Reset3D.address());
		LogRelocationTarget("BSFaceGenUtils::AddHeadPartOnActor", Address::BSFaceGenAddHeadPartOnActor.address());
		LogRelocationTarget("BipedAnim::AttachSkinnedObject BakeChargenMorphs callsite", Address::BipedAnimAttachSkinnedObjectBakeChargenMorphsCall.address());
		LogRelocationTarget("BSFaceGenPendingHeadData::Attach BakeChargenMorphs callsite", Address::BSFaceGenPendingHeadAttachBakeChargenMorphsCall.address());
		LogRelocationTarget("BSFaceGenUtils::AddHeadPartOnActor PrepareHeadPart callsite", Address::BSFaceGenAddHeadPartOnActorPrepareHeadPartCall.address());
		LogRelocationTarget("BSFaceGenModelExtraData::SetBoneName", Address::BSFaceGenModelExtraDataSetBoneName.address());
		LogRelocationTarget("LooksMenuUtils::ShowLooksMenu", Address::LooksMenuUtilsShowLooksMenu.address());
	}

	void EmitEvent(const Smp::LifecycleEvent& a_event)
	{
		const auto highFrequency =
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
		auto* actor = AsActor(a_ref);
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

	RE::Actor* ResolveFaceGenActor(RE::BSFaceGenNiNode* a_faceNode, RE::NiNode* a_skeleton)
	{
		if (auto* actor = ResolveFaceGenActor(a_faceNode)) {
			return actor;
		}

		const auto userData = a_skeleton ? a_skeleton->userData : 0;
		if (userData != 0 && userData <= (std::numeric_limits<RE::TESFormID>::max)()) {
			return AsActor(RE::TESForm::GetFormByID<RE::TESObjectREFR>(static_cast<RE::TESFormID>(userData)));
		}

		return nullptr;
	}

	void EmitSkinnedHeadGeometryEvent(
		RE::Actor* a_actor,
		RE::BSFaceGenNiNode* a_faceNode,
		RE::NiAVObject* a_object,
		const Smp::LifecycleEventType a_type)
	{
		if (!a_actor || !a_faceNode) {
			return;
		}

		EmitEvent({
			.type = a_type,
			.actor = a_actor,
			.object = a_object ? a_object : reinterpret_cast<RE::NiAVObject*>(a_faceNode),
		});
	}

	RE::NiAVObject* HookedBipedAnimApplySkinnedObjects(RE::BipedAnim* a_biped, RE::NiNode* a_originalModelRoot, RE::BIPED_OBJECT a_bipedObject, bool a_firstPerson)
	{
		// Vanilla also uses this function for non-biped animation objects and passes
		// kNone. GetBipObject performs unchecked array indexing, so only inspect real
		// biped slots and leave animation-object attachment entirely to vanilla.
		if (!IsBipedObjectSlot(a_bipedObject)) {
			ScopedApplySkinnedObjectsDepth scopedDepth;
			return OriginalBipedAnimApplySkinnedObjects(a_biped, a_originalModelRoot, a_bipedObject, a_firstPerson);
		}

		auto* bipObject = a_biped ? a_biped->GetBipObject(a_bipedObject) : nullptr;
		const std::string_view nifPath = bipObject && bipObject->part ? bipObject->part->GetModel() : "";
		auto* originalModelObject = a_originalModelRoot ? static_cast<RE::NiAVObject*>(a_originalModelRoot) : nullptr;
		auto* actor = ResolveActor(a_biped);
		auto* ownershipRoot = actor ? actor->Get3D(a_firstPerson) : nullptr;
		if (!ownershipRoot && actor) {
			ownershipRoot = actor->Get3D();
		}
		auto preAttach = PreparePreAttachPhysicsContext(
			originalModelObject,
			ownershipRoot ? ownershipRoot :
				(a_biped ? static_cast<RE::NiAVObject*>(a_biped->GetRoot()) : nullptr),
			nifPath,
			"original model root",
			UsesFaceBonesModel(bipObject));

		RE::NiAVObject* attachedObject = nullptr;
		ApplySkinnedObjectsSkeletonRoot = nullptr;
		{
			ScopedApplySkinnedObjectsDepth scopedDepth;
			attachedObject = OriginalBipedAnimApplySkinnedObjects(a_biped, a_originalModelRoot, a_bipedObject, a_firstPerson);
		}
		Smp::FinalizeArmorSkinBindings(
			actor,
			attachedObject,
			ApplySkinnedObjectsSkeletonRoot,
			a_firstPerson,
			preAttach.armorBoneReferences,
			{},
			true,
			preAttach.mainSkinBindings);
		EmitEvent({
			.type = Smp::LifecycleEventType::kArmorApplySkinnedObjects,
			.actor = actor,
			.biped = a_biped,
			.bipObject = bipObject,
			.bipedObject = a_bipedObject,
			.object = attachedObject,
			.sourceObject = a_originalModelRoot ? static_cast<RE::NiAVObject*>(a_originalModelRoot) : nullptr,
			.armorBoneReferences = std::move(preAttach.armorBoneReferences),
			.destinationRoot = ApplySkinnedObjectsSkeletonRoot,
			.sourceRoot = a_originalModelRoot,
			.physicsXmlPath = preAttach.selectedXml.value_or(std::string{}),
			.firstPerson = a_firstPerson,
		});
		return attachedObject;
	}

	RE::NiAVObject* HookedBipedAnimAttachSkinnedObject(RE::BipedAnim* a_biped, RE::NiNode* a_modelRoot, RE::NiNode* a_skeletonRoot, RE::BIPED_OBJECT a_bipedObject, bool a_firstPerson)
	{
		const bool nestedApply = ApplySkinnedObjectsDepth > 0;
		if (!IsBipedObjectSlot(a_bipedObject)) {
			auto* attachedObject = OriginalBipedAnimAttachSkinnedObject(a_biped, a_modelRoot, a_skeletonRoot, a_bipedObject, a_firstPerson);
			if (nestedApply) {
				ApplySkinnedObjectsSkeletonRoot = a_skeletonRoot;
			}
			return attachedObject;
		}

		auto* bipObject = a_biped ? a_biped->GetBipObject(a_bipedObject) : nullptr;
		const std::string_view nifPath = bipObject && bipObject->part ? bipObject->part->GetModel() : "";
		auto* modelObject = a_modelRoot ? static_cast<RE::NiAVObject*>(a_modelRoot) : nullptr;
		auto* actor = ResolveActor(a_biped);
		auto* ownershipRoot = actor ? actor->Get3D(a_firstPerson) : nullptr;
		if (!ownershipRoot && actor) {
			ownershipRoot = actor->Get3D();
		}
		PreAttachPhysicsContext preAttach;
		if (!nestedApply) {
			preAttach = PreparePreAttachPhysicsContext(
				modelObject,
				ownershipRoot ? ownershipRoot : static_cast<RE::NiAVObject*>(a_skeletonRoot),
				nifPath,
				"model root",
				UsesFaceBonesModel(bipObject));
		}

		auto* attachedObject = OriginalBipedAnimAttachSkinnedObject(a_biped, a_modelRoot, a_skeletonRoot, a_bipedObject, a_firstPerson);
		if (nestedApply) {
			ApplySkinnedObjectsSkeletonRoot = a_skeletonRoot;
			return attachedObject;
		}
		Smp::FinalizeArmorSkinBindings(
			actor,
			attachedObject,
			a_skeletonRoot,
			a_firstPerson,
			preAttach.armorBoneReferences,
			{},
			true,
			preAttach.mainSkinBindings);
		EmitEvent({
			.type = Smp::LifecycleEventType::kArmorAttachSkinnedObject,
			.actor = actor,
			.biped = a_biped,
			.bipObject = bipObject,
			.bipedObject = a_bipedObject,
			.object = attachedObject,
			.sourceObject = modelObject,
			.armorBoneReferences = std::move(preAttach.armorBoneReferences),
			.destinationRoot = a_skeletonRoot,
			.sourceRoot = a_modelRoot,
			.physicsXmlPath = preAttach.selectedXml.value_or(std::string{}),
			.firstPerson = a_firstPerson,
		});
		return attachedObject;
	}

	void HookedBipedAnimAttachToParent(RE::NiAVObject* a_parent, RE::NiAVObject* a_attachedObject, RE::NiAVObject* a_sourceObject, RE::BSTSmartPointer<RE::BipedAnim>& a_biped, RE::BIPED_OBJECT a_bipedObject)
	{
		OriginalBipedAnimAttachToParent(a_parent, a_attachedObject, a_sourceObject, a_biped, a_bipedObject);
		if (ApplySkinnedObjectsDepth > 0 || !IsBipedObjectSlot(a_bipedObject)) {
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

	void HookedTESObjectREFRFixDisplayedHeadParts(
		RE::TESObjectREFR* a_ref,
		RE::BSFaceGenNiNode* a_faceNode,
		bool a_hideHead,
		bool a_enableAllSegments)
	{
		OriginalTESObjectREFRFixDisplayedHeadParts(a_ref, a_faceNode, a_hideHead, a_enableAllSegments);

		auto* actor = AsActor(a_ref);
		auto* currentFace = actor ? actor->GetFaceNodeSkinned() : nullptr;
		if (!actor || !currentFace || (a_faceNode && a_faceNode != currentFace)) {
			return;
		}
		SeedFaceGenActor(a_ref);
		EmitEvent({
			.type = Smp::LifecycleEventType::kHeadPartVisibilityUpdated,
			.actor = actor,
			.object = reinterpret_cast<RE::NiAVObject*>(currentFace),
		});
	}

	void BeginSkeletonTransitionIfNeeded(RE::TESObjectREFR* a_ref, RE::NiAVObject* a_replacementRoot)
	{
		auto* actor = AsActor(a_ref);
		auto* oldRoot = actor ? actor->Get3D(false) : nullptr;
		if (!actor || !oldRoot || oldRoot == a_replacementRoot) {
			return;
		}

		// This is the last boundary where stale writeback slots and headpart
		// recipes still refer to a live old skeleton.
		auto* faceNode = actor->GetFaceNodeSkinned();
		auto discardedLifecycleEvents = Smp::DiscardQueuedLifecycleEvents(actor);
		const auto transitionStarted = Smp::Fo4PhysicsWorld::GetSingleton()->BeginActorSkeletonTransition(
			actor,
			oldRoot,
			faceNode);
		discardedLifecycleEvents += Smp::DiscardQueuedLifecycleEvents(actor);
		if (!transitionStarted) {
			if (discardedLifecycleEvents > 0) {
				spdlog::trace(
					"discarded {} queued old-root lifecycle events at untracked Set3D boundary actor={} oldRoot={} replacementRoot={}",
					discardedLifecycleEvents,
					static_cast<void*>(actor),
					static_cast<void*>(oldRoot),
					static_cast<void*>(a_replacementRoot));
			}
			return;
		}

		SeedFaceGenActor(a_ref);
		spdlog::debug(
			"observed actor skeleton replacement boundary before Set3D actor={} oldRoot={} replacementRoot={} retainedFace={} discardedQueuedLifecycleEvents={}",
			static_cast<void*>(actor),
			static_cast<void*>(oldRoot),
			static_cast<void*>(a_replacementRoot),
			static_cast<void*>(faceNode),
			discardedLifecycleEvents);
	}

	RE::NiAVObject* HookedActorLoad3D(RE::TESObjectREFR* a_ref, bool a_backgroundLoading)
	{
		auto* loaded3D = OriginalActorLoad3D(a_ref, a_backgroundLoading);
		if (auto* actor = AsActor(a_ref)) {
			Smp::Fo4PhysicsWorld::GetSingleton()->NoteActorSkeletonLoaded(actor, loaded3D);
			EmitEvent({
				.type = Smp::LifecycleEventType::kActorLoad3D,
				.actor = actor,
				.object = loaded3D,
			});
		}
		return loaded3D;
	}

	RE::NiAVObject* HookedPlayerCharacterLoad3D(RE::TESObjectREFR* a_ref, bool a_backgroundLoading)
	{
		auto* loaded3D = OriginalPlayerCharacterLoad3D(a_ref, a_backgroundLoading);
		if (auto* actor = AsActor(a_ref)) {
			Smp::Fo4PhysicsWorld::GetSingleton()->NoteActorSkeletonLoaded(actor, loaded3D);
			EmitEvent({
				.type = Smp::LifecycleEventType::kActorLoad3D,
				.actor = actor,
				.object = loaded3D,
			});
		}
		return loaded3D;
	}

	void HookedActorSet3D(RE::TESObjectREFR* a_ref, RE::NiAVObject* a_object, bool a_queue3DTasks)
	{
		BeginSkeletonTransitionIfNeeded(a_ref, a_object);
		OriginalActorSet3D(a_ref, a_object, a_queue3DTasks);
		if (auto* actor = AsActor(a_ref)) {
			Smp::Fo4PhysicsWorld::GetSingleton()->NoteActorSkeletonLoaded(actor, a_object);
			EmitEvent({
				.type = Smp::LifecycleEventType::kActorSet3D,
				.actor = actor,
				.object = a_object,
				.queue3DTasks = a_queue3DTasks,
			});
		}
	}

	void HookedPlayerCharacterSet3D(RE::TESObjectREFR* a_ref, RE::NiAVObject* a_object, bool a_queue3DTasks)
	{
		BeginSkeletonTransitionIfNeeded(a_ref, a_object);
		OriginalPlayerCharacterSet3D(a_ref, a_object, a_queue3DTasks);
		if (auto* actor = AsActor(a_ref)) {
			Smp::Fo4PhysicsWorld::GetSingleton()->NoteActorSkeletonLoaded(actor, a_object);
			EmitEvent({
				.type = Smp::LifecycleEventType::kActorSet3D,
				.actor = actor,
				.object = a_object,
				.queue3DTasks = a_queue3DTasks,
			});
		}
	}

	void HookedActorOnHeadInitialized(RE::TESObjectREFR* a_ref)
	{
		SeedFaceGenActor(a_ref);
		OriginalActorOnHeadInitialized(a_ref);
		auto* actor = AsActor(a_ref);
		auto* faceNode = actor ? actor->GetFaceNodeSkinned() : nullptr;
		SeedFaceGenActor(a_ref);
		if (actor) {
			EmitEvent({
				.type = Smp::LifecycleEventType::kActorHeadInitialized,
				.actor = actor,
				.object = reinterpret_cast<RE::NiAVObject*>(faceNode),
			});
		}
	}

	void HookedPlayerCharacterOnHeadInitialized(RE::TESObjectREFR* a_ref)
	{
		SeedFaceGenActor(a_ref);
		OriginalPlayerCharacterOnHeadInitialized(a_ref);
		auto* actor = AsActor(a_ref);
		auto* faceNode = actor ? actor->GetFaceNodeSkinned() : nullptr;
		SeedFaceGenActor(a_ref);
		if (actor) {
			EmitEvent({
				.type = Smp::LifecycleEventType::kActorHeadInitialized,
				.actor = actor,
				.object = reinterpret_cast<RE::NiAVObject*>(faceNode),
			});
		}
	}

	void PrepareMissingFaceBones(
		RE::NiNode* a_mainRoot,
		RE::NiNode* a_faceBonesRoot,
		const bool a_allowWithoutXml)
	{
		if (!a_allowWithoutXml && !FindPhysicsXmlExtraData(a_mainRoot)) {
			return;
		}
		Smp::MaterializeMissingFaceBonesForBake(a_mainRoot, a_faceBonesRoot);
	}

	bool IsFaceBonesNifPath(const std::string_view a_path)
	{
		constexpr std::string_view suffix{ "_facebones.nif" };
		if (a_path.size() < suffix.size()) {
			return false;
		}
		return std::ranges::equal(
			a_path.end() - static_cast<std::ptrdiff_t>(suffix.size()),
			a_path.end(),
			suffix.begin(),
			suffix.end(),
			[](const char a_lhs, const char a_rhs) {
				return std::tolower(static_cast<unsigned char>(a_lhs)) == a_rhs;
			});
	}

	void HookedArmorBakeChargenMorphs(
		RE::TESNPC* a_npc,
		RE::NiNode* a_mainRoot,
		RE::NiNode* a_faceBonesRoot,
		void* a_extraData)
	{
		PrepareMissingFaceBones(a_mainRoot, a_faceBonesRoot, false);
		OriginalArmorBakeChargenMorphs(a_npc, a_mainRoot, a_faceBonesRoot, a_extraData);
	}

	void HookedHeadBakeChargenMorphs(
		RE::TESNPC* a_npc,
		RE::NiNode* a_mainRoot,
		RE::NiNode* a_faceBonesRoot,
		void* a_extraData)
	{
		PrepareMissingFaceBones(
			a_mainRoot,
			a_faceBonesRoot,
			PendingIncrementalHeadPart.isExtraPart);
		OriginalHeadBakeChargenMorphs(a_npc, a_mainRoot, a_faceBonesRoot, a_extraData);
	}

	void HookedFaceGenPrepareHeadPart(
		RE::BSFaceGenNiNode* a_faceNode,
		RE::BGSHeadPart* a_headPart,
		RE::TESNPC* a_npc,
		const bool a_useChargenModel,
		const bool a_arg5)
	{
		PendingIncrementalHeadPart = {};
		const auto* modelPath = a_headPart ?
			static_cast<RE::BGSModelMaterialSwap*>(a_headPart)->GetModel() :
			nullptr;
		const auto* chargenModelPath = a_headPart ? a_headPart->ChargenModel.GetModel() : nullptr;
		const auto hasFaceBonesModel = IsFaceBonesNifPath(chargenModelPath ? chargenModelPath : "");
		bool forceMainModel = false;
		if (modelPath && *modelPath) {
			RE::BSModelDB::DBTraits::ArgsType args{};
			args.loadLevel = 3;
			args.performProcess = true;
			args.loadTextures = true;
			RE::NiPointer<RE::NiNode> mainRoot;
			const auto error = RE::BSModelDB::Demand(modelPath, std::addressof(mainRoot), args);
			if (error == RE::BSResource::ErrorCode::kNone && mainRoot) {
				const auto physicsXml = FindPhysicsXmlExtraData(mainRoot.get());
				const auto isExtraPart = a_headPart->IsExtraPart();
				const auto contributesToPhysicsClosure = physicsXml.has_value() || isExtraPart;
				forceMainModel = contributesToPhysicsClosure && a_useChargenModel && hasFaceBonesModel;
				if (contributesToPhysicsClosure) {
					PendingIncrementalHeadPart = {
						.faceNode = a_faceNode,
						.mainRoot = std::move(mainRoot),
						.nifPath = modelPath,
						.usesMainModel = forceMainModel || !a_useChargenModel,
						.isExtraPart = isExtraPart,
					};
				}
				if (forceMainModel) {
					spdlog::debug(
						"using main NIF as authoritative incremental headpart geometry headPart={:08X} main='{}' faceBones='{}'",
						a_headPart->GetFormID(),
						modelPath,
						chargenModelPath);
				}
			} else {
				spdlog::warn(
					"failed to preload authoritative main headpart model headPart={} nif='{}' error={}",
					a_headPart ? a_headPart->GetFormID() : 0,
					modelPath,
					std::to_underlying(error));
			}
		}

		// PrepareHeadPart always uses the chargen morph TRI in either branch. Passing
		// false only selects the main model instead of ChargenModel for the geometry.
		OriginalFaceGenPrepareHeadPart(
			a_faceNode,
			a_headPart,
			a_npc,
			forceMainModel ? false : a_useChargenModel,
			a_arg5);
	}

	void HookedFaceGenSkinAllGeometry(RE::BSFaceGenNiNode* a_faceNode, RE::NiNode* a_skeleton, bool a_arg3)
	{
		auto* physicsWorld = Smp::Fo4PhysicsWorld::GetSingleton();
		// Vanilla only fixes skin instances on direct geometry children. Targets
		// must exist and every retained slot must be safe before that pass because
		// its metadata fallback dereferences the current bone. Rebind again after
		// vanilla to cover nested geometry and verify the final target set.
		auto* transitionActor = physicsWorld->PrepareRetainedFaceForSkeleton(a_faceNode, a_skeleton);
		OriginalFaceGenSkinAllGeometry(a_faceNode, a_skeleton, a_arg3);
		if (transitionActor) {
			physicsWorld->CompleteRetainedFaceSkinning(transitionActor, a_faceNode, a_skeleton);
		}

		auto* actor = transitionActor ? transitionActor : ResolveFaceGenActor(a_faceNode, a_skeleton);
		if (!actor) {
			spdlog::trace(
				"skipped skinned head full-geometry event because actor is unresolved faceNode={} skeleton={}",
				static_cast<void*>(a_faceNode),
				static_cast<void*>(a_skeleton));
			return;
		}

		EmitSkinnedHeadGeometryEvent(
			actor,
			a_faceNode,
			reinterpret_cast<RE::NiAVObject*>(a_faceNode),
			Smp::LifecycleEventType::kHeadSkinAllGeometry);
	}

	void HookedFaceGenSkinSingleGeometry(RE::BSFaceGenNiNode* a_faceNode, RE::NiNode* a_skeleton, RE::BSGeometry* a_geometry, bool a_arg4)
	{
		auto incrementalContext = std::move(PendingIncrementalHeadPart);
		PendingIncrementalHeadPart = {};
		auto* actor = ResolveFaceGenActor(a_faceNode, a_skeleton);
		std::vector<Smp::ArmorBoneReference> mainReferences;
		std::vector<Smp::RetainedSkinBinding> liveMainBindings;
		const auto useMainContext =
			incrementalContext.faceNode == a_faceNode &&
			incrementalContext.mainRoot &&
			!incrementalContext.nifPath.empty() &&
			incrementalContext.usesMainModel;
		if (useMainContext) {
			mainReferences = Smp::CaptureArmorBoneReferences(
				incrementalContext.mainRoot.get(),
				a_skeleton,
				incrementalContext.nifPath,
				false);
			// PrepareHeadPart produced this geometry from the main NIF. Capture its
			// authoritative palette before vanilla FixSkinInstances can rewrite it.
			liveMainBindings = Smp::CaptureMainSkinBindings(a_geometry);
			Smp::FinalizeArmorSkinBindings(
				actor,
				a_geometry,
				a_skeleton,
				false,
				mainReferences,
				{},
				false,
				liveMainBindings);
		}

		OriginalFaceGenSkinSingleGeometry(a_faceNode, a_skeleton, a_geometry, a_arg4);

		if (useMainContext) {
			Smp::FinalizeArmorSkinBindings(
				actor,
				a_geometry,
				a_skeleton,
				false,
				mainReferences,
				{},
				false,
				liveMainBindings);
		}

		if (!actor) {
			spdlog::trace(
				"skipped skinned head single-geometry event because actor is unresolved faceNode={} skeleton={} geometry={}",
				static_cast<void*>(a_faceNode),
				static_cast<void*>(a_skeleton),
				static_cast<void*>(a_geometry));
			return;
		}

		EmitSkinnedHeadGeometryEvent(
			actor,
			a_faceNode,
			a_geometry,
			Smp::LifecycleEventType::kHeadSkinSingleGeometry);
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

	void HookedLooksMenuUtilsShowLooksMenu(
		RE::TESObjectREFR* a_target,
		const std::uint32_t a_editMode,
		RE::TESObjectREFR* a_target2,
		RE::TESObjectREFR* a_swapTarget,
		RE::TESObjectREFR* a_vendor)
	{
		auto* actor = AsActor(a_target);
		if (!actor) {
			actor = RE::PlayerCharacter::GetSingleton();
		}
		if (actor) {
			Smp::Fo4PhysicsWorld::GetSingleton()->NoteCharacterCustomizationTarget(actor);
		}

		OriginalLooksMenuUtilsShowLooksMenu(a_target, a_editMode, a_target2, a_swapTarget, a_vendor);
	}

	void HookedMainSwap(RE::Main* a_main)
	{
		Smp::Fo4PhysicsWorld::GetSingleton()->StepFrame();
		Smp::Fo4PhysicsWorld::GetSingleton()->WriteBackSystems(Smp::WritebackSource::kMainSync);
		Smp::ImguiLayer::RenderFrame();
		OriginalMainSwap(a_main);
	}

	void HookedFaceGenManagerUpdate(void* a_manager, const bool a_forceUpdate)
	{
		if (REX::FModule::IsRuntimeOG()) {
			reinterpret_cast<void (*)(void*)>(OriginalFaceGenManagerUpdate)(a_manager);
		} else {
			reinterpret_cast<void (*)(void*, bool)>(OriginalFaceGenManagerUpdate)(a_manager, a_forceUpdate);
		}

		Smp::Fo4PhysicsWorld::GetSingleton()->NoteFaceGenManagerUpdated(IsFaceGenManagerIdle(a_manager));
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
		const auto mainSyncCallsite = Address::MainOnIdleSwapCall.address();
		const auto faceGenManagerUpdateCallsite = Address::MainSwapUpdatePendingCustomizationTexturesCall.address();
		LogRelocationTarget("Main::OnIdle frame sync callsite", mainSyncCallsite);
		LogRelocationTarget("Main::Swap FaceGen manager update callsite", faceGenManagerUpdateCallsite);

		if (!OriginalMainSwap) {
			OriginalMainSwap = reinterpret_cast<MainSwap_t>(
				REL::GetTrampoline().write_call<5>(mainSyncCallsite, reinterpret_cast<std::uintptr_t>(&HookedMainSwap)));
			spdlog::info("Main::OnIdle frame sync call hook installed at {:x}", mainSyncCallsite);
		}
		if (!OriginalFaceGenManagerUpdate) {
			OriginalFaceGenManagerUpdate = REL::GetTrampoline().write_call<5>(
				faceGenManagerUpdateCallsite,
				reinterpret_cast<std::uintptr_t>(&HookedFaceGenManagerUpdate));
			spdlog::info("Main::Swap FaceGen manager update call hook installed at {:x}", faceGenManagerUpdateCallsite);
		}
		if (!OriginalBipedAnimApplySkinnedObjects) {
			OriginalBipedAnimApplySkinnedObjects = CreateBranchGateway5<BipedAnimApplySkinnedObjects_t>("BipedAnim::ApplySkinnedObjects", Address::BipedAnimApplySkinnedObjects, Address::BipedAnimApplySkinnedObjectsPrologueSize.value(), reinterpret_cast<void*>(&HookedBipedAnimApplySkinnedObjects));
		}
		if (!OriginalBipedAnimAttachSkinnedObject) {
			OriginalBipedAnimAttachSkinnedObject = CreateBranchGateway5<BipedAnimAttachSkinnedObject_t>("BipedAnim::AttachSkinnedObject", Address::BipedAnimAttachSkinnedObject, Address::BipedAnimAttachSkinnedObjectPrologueSize.value(), reinterpret_cast<void*>(&HookedBipedAnimAttachSkinnedObject));
		}
		if (!OriginalBipedAnimAttachToParent) {
			OriginalBipedAnimAttachToParent = CreateBranchGateway5<BipedAnimAttachToParent_t>("BipedAnim::AttachToParent", Address::BipedAnimAttachToParent, Address::BipedAnimAttachToParentPrologueSize.value(), reinterpret_cast<void*>(&HookedBipedAnimAttachToParent));
		}
		if (!OriginalBipedAnimRemovePart) {
			OriginalBipedAnimRemovePart = CreateBranchGateway5<BipedAnimRemovePart_t>("BipedAnim::RemovePart", Address::BipedAnimRemovePart, Address::BipedAnimRemovePartPrologueSize.value(), reinterpret_cast<void*>(&HookedBipedAnimRemovePart));
		}
		if (!OriginalTESObjectREFRFixDisplayedHeadParts) {
			OriginalTESObjectREFRFixDisplayedHeadParts = CreateBranchGateway5<TESObjectREFRFixDisplayedHeadParts_t>(
				"TESObjectREFR::FixDisplayedHeadParts",
				Address::TESObjectREFRFixDisplayedHeadParts,
				Address::TESObjectREFRFixDisplayedHeadPartsPrologueSize.value(),
				reinterpret_cast<void*>(&HookedTESObjectREFRFixDisplayedHeadParts));
		}

		REL::Relocation<std::uintptr_t> actorVTable{ RE::VTABLE::Actor[0] };
		REL::Relocation<std::uintptr_t> playerVTable{ RE::VTABLE::PlayerCharacter[0] };
		REL::Relocation<std::uintptr_t> faceGenVTable{ RE::VTABLE::BSFaceGenNiNode[0] };

		if (!OriginalActorLoad3D) {
			OriginalActorLoad3D = InstallVFuncHook<ActorLoad3D_t>("Actor::Load3D", actorVTable, Address::ActorLoad3DVFuncSlot, reinterpret_cast<void*>(&HookedActorLoad3D));
		}
		if (!OriginalPlayerCharacterLoad3D) {
			OriginalPlayerCharacterLoad3D = InstallVFuncHook<ActorLoad3D_t>("PlayerCharacter::Load3D", playerVTable, Address::ActorLoad3DVFuncSlot, reinterpret_cast<void*>(&HookedPlayerCharacterLoad3D));
		}
		if (!OriginalActorSet3D) {
			OriginalActorSet3D = InstallVFuncHook<Set3D_t>("Actor::Set3D", actorVTable, Address::ActorSet3DVFuncSlot, reinterpret_cast<void*>(&HookedActorSet3D));
		}
		if (!OriginalPlayerCharacterSet3D) {
			OriginalPlayerCharacterSet3D = InstallVFuncHook<Set3D_t>("PlayerCharacter::Set3D", playerVTable, Address::ActorSet3DVFuncSlot, reinterpret_cast<void*>(&HookedPlayerCharacterSet3D));
		}
		if (!OriginalActorOnHeadInitialized) {
			OriginalActorOnHeadInitialized = InstallVFuncHook<OnHeadInitialized_t>("Actor::OnHeadInitialized", actorVTable, Address::ActorOnHeadInitializedVFuncSlot, reinterpret_cast<void*>(&HookedActorOnHeadInitialized));
		}
		if (!OriginalPlayerCharacterOnHeadInitialized) {
			OriginalPlayerCharacterOnHeadInitialized = InstallVFuncHook<OnHeadInitialized_t>("PlayerCharacter::OnHeadInitialized", playerVTable, Address::ActorOnHeadInitializedVFuncSlot, reinterpret_cast<void*>(&HookedPlayerCharacterOnHeadInitialized));
		}
		if (!OriginalFaceGenSkinAllGeometry) {
			OriginalFaceGenSkinAllGeometry = InstallVFuncHook<FaceGenSkinAllGeometry_t>(
				"BSFaceGenNiNode::SkinAllGeometry",
				faceGenVTable,
				Address::BSFaceGenSkinAllGeometryVFuncSlot,
				reinterpret_cast<void*>(&HookedFaceGenSkinAllGeometry));
		}

		if (!OriginalReset3D) {
			OriginalReset3D = CreateBranchGateway5<Reset3D_t>("Actor::Reset3D", Address::Reset3D, Address::Reset3DPrologueSize.value(), reinterpret_cast<void*>(&HookedReset3D));
		}
		if (!OriginalSetFaceGenBoneName) {
			OriginalSetFaceGenBoneName = CreateBranchGateway5<SetFaceGenBoneName_t>(
				"BSFaceGenModelExtraData::SetBoneName",
				Address::BSFaceGenModelExtraDataSetBoneName,
				Address::BSFaceGenModelExtraDataSetBoneNamePrologueSize.value(),
				reinterpret_cast<void*>(&HookedSetFaceGenBoneName));
		}
		if (!OriginalLooksMenuUtilsShowLooksMenu) {
			OriginalLooksMenuUtilsShowLooksMenu = CreateBranchGateway5<LooksMenuUtilsShowLooksMenu_t>(
				"LooksMenuUtils::ShowLooksMenu",
				Address::LooksMenuUtilsShowLooksMenu,
				Address::LooksMenuUtilsShowLooksMenuPrologueSize.value(),
				reinterpret_cast<void*>(&HookedLooksMenuUtilsShowLooksMenu));
		}
		if (!OriginalArmorBakeChargenMorphs) {
			const auto callsite = Address::BipedAnimAttachSkinnedObjectBakeChargenMorphsCall.address();
			OriginalArmorBakeChargenMorphs = reinterpret_cast<BakeChargenMorphs_t>(
				REL::GetTrampoline().write_call<5>(callsite, reinterpret_cast<std::uintptr_t>(&HookedArmorBakeChargenMorphs)));
			spdlog::info("armor BakeChargenMorphs call hook installed at {:x}", callsite);
		}
		if (!OriginalHeadBakeChargenMorphs) {
			const auto callsite = Address::BSFaceGenPendingHeadAttachBakeChargenMorphsCall.address();
			OriginalHeadBakeChargenMorphs = reinterpret_cast<BakeChargenMorphs_t>(
				REL::GetTrampoline().write_call<5>(callsite, reinterpret_cast<std::uintptr_t>(&HookedHeadBakeChargenMorphs)));
			spdlog::info("pending-head BakeChargenMorphs call hook installed at {:x}", callsite);
		}
		if (!OriginalFaceGenPrepareHeadPart) {
			const auto callsite = Address::BSFaceGenAddHeadPartOnActorPrepareHeadPartCall.address();
			OriginalFaceGenPrepareHeadPart = reinterpret_cast<FaceGenPrepareHeadPart_t>(
				REL::GetTrampoline().write_call<5>(callsite, reinterpret_cast<std::uintptr_t>(&HookedFaceGenPrepareHeadPart)));
			spdlog::info("BSFaceGenUtils::AddHeadPartOnActor prepare-headpart call hook installed at {:x}", callsite);
		}
		if (!OriginalFaceGenSkinSingleGeometry) {
			const auto skinSingleCallsite = Address::BSFaceGenAddHeadPartOnActorSkinSingleCall.address();
			OriginalFaceGenSkinSingleGeometry = reinterpret_cast<FaceGenSkinSingleGeometry_t>(
				REL::GetTrampoline().write_call<5>(skinSingleCallsite, reinterpret_cast<std::uintptr_t>(&HookedFaceGenSkinSingleGeometry)));
			spdlog::info("BSFaceGenUtils::AddHeadPartOnActor skin-single call hook installed at {:x}", skinSingleCallsite);
		}

		const bool installed =
			OriginalMainSwap &&
			OriginalFaceGenManagerUpdate &&
			OriginalBipedAnimApplySkinnedObjects &&
			OriginalBipedAnimAttachSkinnedObject &&
			OriginalBipedAnimAttachToParent &&
			OriginalBipedAnimRemovePart &&
			OriginalTESObjectREFRFixDisplayedHeadParts &&
			OriginalActorLoad3D &&
			OriginalPlayerCharacterLoad3D &&
			OriginalActorSet3D &&
			OriginalPlayerCharacterSet3D &&
			OriginalActorOnHeadInitialized &&
			OriginalPlayerCharacterOnHeadInitialized &&
			OriginalFaceGenSkinAllGeometry &&
			OriginalReset3D &&
			OriginalLooksMenuUtilsShowLooksMenu &&
			OriginalSetFaceGenBoneName &&
			OriginalArmorBakeChargenMorphs &&
			OriginalHeadBakeChargenMorphs &&
			OriginalFaceGenPrepareHeadPart &&
			OriginalFaceGenSkinSingleGeometry;

		spdlog::info("FO4 Faster HDT-SMP lifecycle hooks {}", installed ? "installed" : "failed");
		return installed;
	}
}
