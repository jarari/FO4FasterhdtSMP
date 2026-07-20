#include "Hooks.h"

#include "ArmorBoneReference.h"
#include "Fo4PhysicsWorld.h"
#include "ImguiLayer.h"
#include "LifecycleEvents.h"
#include "PhysicsXmlSelection.h"
#include "RE/B/BSAnimationGraphManager.h"
#include "RE/B/BSGeometry.h"
#include "RE/H/hkArray.h"
#include "RE/H/hkReferencedObject.h"
#include "RE/H/hkRefPtr.h"
#include "RE/M/Main.h"
#include "RE/N/NiNode.h"
#include "RE/P/PlayerCharacter.h"
#include "RE/N/NiStringExtraData.h"
#include "RE/T/TESObjectREFR.h"

#if defined(_M_X64) && !defined(_AMD64_)
#	define _AMD64_ 1
#endif
#include <Windows.h>
#include <detours.h>

#include <limits>
#include <optional>
#include <unordered_map>
#include <utility>

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
		REL::Relocation<std::uintptr_t> BSFaceGenAddHeadPartOnActor{ REL::ID{ 913780, 0 } };
		REL::Relocation<std::uintptr_t> BSFaceGenModelExtraDataSetBoneName{ REL::ID{ 1278503, 0 } };
		REL::Relocation<std::uintptr_t> LooksMenuUtilsShowLooksMenu{ REL::ID{ 411372, 2223366 } };
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
	using FaceGenSkinAllGeometry_t = void (*)(RE::BSFaceGenNiNode*, RE::NiNode*, bool);
	using FaceGenSkinSingleGeometry_t = void (*)(RE::BSFaceGenNiNode*, RE::NiNode*, RE::BSGeometry*, bool);
	using SetFaceGenBoneName_t = void (*)(void*, std::uint32_t, RE::BSFixedString*);
	using LooksMenuUtilsShowLooksMenu_t = void (*)(RE::TESObjectREFR*, std::uint32_t, RE::TESObjectREFR*, RE::TESObjectREFR*, RE::TESObjectREFR*);

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
	FaceGenSkinAllGeometry_t       OriginalFaceGenSkinAllGeometry{ nullptr };
	FaceGenSkinSingleGeometry_t    OriginalFaceGenSkinSingleGeometry{ nullptr };
	SetFaceGenBoneName_t           OriginalSetFaceGenBoneName{ nullptr };
	LooksMenuUtilsShowLooksMenu_t  OriginalLooksMenuUtilsShowLooksMenu{ nullptr };

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
	inline constexpr std::uint32_t kFaceGenModelExtraDataBoneNameLimit = 0x80;
	inline constexpr std::uintptr_t kAddHeadPartSkinSingleCallOffsetOG = 0xFD;
	inline constexpr std::size_t kFaceGenSkinAllGeometryVFuncSlot = 0x43;

	thread_local std::uint32_t ApplySkinnedObjectsDepth{ 0 };
	thread_local RE::NiNode* ApplySkinnedObjectsSkeletonRoot{ nullptr };
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
	};

	PreAttachPhysicsContext PreparePreAttachPhysicsContext(
		RE::NiAVObject* a_sourceObject,
		RE::NiAVObject* a_skeletonRoot,
		const std::string_view a_nifPath,
		const char* a_sourceLabel)
	{
		PreAttachPhysicsContext context;
		context.selectedXml = FindPhysicsXmlExtraData(a_sourceObject);
		if (!context.selectedXml) {
			return context;
		}

		context.armorBoneReferences = Smp::CaptureArmorBoneReferences(a_sourceObject, a_skeletonRoot, a_nifPath);
		spdlog::debug(
			"pre-scanned armor physics XML {} from {}={} name='{}' nif='{}'",
			*context.selectedXml,
			a_sourceLabel,
			static_cast<void*>(a_sourceObject),
			a_sourceObject ? std::string_view(a_sourceObject->GetName()) : std::string_view{},
			a_nifPath);

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
		LogRelocationTarget("LooksMenuUtils::ShowLooksMenu", Addresses::LooksMenuUtilsShowLooksMenu.address());
		if (REX::FModule::IsRuntimeOG()) {
			LogRelocationTarget("BSFaceGenUtils::AddHeadPartOnActor", Addresses::BSFaceGenAddHeadPartOnActor.address());
		} else {
			spdlog::warn("BSFaceGenUtils::AddHeadPartOnActor hook skipped: AE relocation ID is not verified");
		}
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
		auto* bipObject = a_biped ? a_biped->GetBipObject(a_bipedObject) : nullptr;
		const std::string_view nifPath = bipObject && bipObject->part ? bipObject->part->GetModel() : "";
		auto* originalModelObject = a_originalModelRoot ? static_cast<RE::NiAVObject*>(a_originalModelRoot) : nullptr;
		auto preAttach = PreparePreAttachPhysicsContext(
			originalModelObject,
			a_biped ? static_cast<RE::NiAVObject*>(a_biped->GetRoot()) : nullptr,
			nifPath,
			"original model root");

		RE::NiAVObject* attachedObject = nullptr;
		ApplySkinnedObjectsSkeletonRoot = nullptr;
		{
			ScopedApplySkinnedObjectsDepth scopedDepth;
			attachedObject = OriginalBipedAnimApplySkinnedObjects(a_biped, a_originalModelRoot, a_bipedObject, a_firstPerson);
		}
		auto* actor = ResolveActor(a_biped);
		Smp::FinalizeArmorSkinBindings(actor, attachedObject, a_firstPerson, preAttach.armorBoneReferences);
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
		auto* bipObject = a_biped ? a_biped->GetBipObject(a_bipedObject) : nullptr;
		const std::string_view nifPath = bipObject && bipObject->part ? bipObject->part->GetModel() : "";
		auto* modelObject = a_modelRoot ? static_cast<RE::NiAVObject*>(a_modelRoot) : nullptr;
		const bool nestedApply = ApplySkinnedObjectsDepth > 0;
		PreAttachPhysicsContext preAttach;
		if (!nestedApply) {
			preAttach = PreparePreAttachPhysicsContext(modelObject, a_skeletonRoot, nifPath, "model root");
		}

		auto* attachedObject = OriginalBipedAnimAttachSkinnedObject(a_biped, a_modelRoot, a_skeletonRoot, a_bipedObject, a_firstPerson);
		if (nestedApply) {
			ApplySkinnedObjectsSkeletonRoot = a_skeletonRoot;
			return attachedObject;
		}
		auto* actor = ResolveActor(a_biped);
		Smp::FinalizeArmorSkinBindings(actor, attachedObject, a_firstPerson, preAttach.armorBoneReferences);
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
		if (auto* actor = AsActor(a_ref)) {
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
		OriginalActorSet3D(a_ref, a_object, a_queue3DTasks);
		if (auto* actor = AsActor(a_ref)) {
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
		OriginalPlayerCharacterSet3D(a_ref, a_object, a_queue3DTasks);
		if (auto* actor = AsActor(a_ref)) {
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

	void HookedFaceGenSkinAllGeometry(RE::BSFaceGenNiNode* a_faceNode, RE::NiNode* a_skeleton, bool a_arg3)
	{
		OriginalFaceGenSkinAllGeometry(a_faceNode, a_skeleton, a_arg3);

		auto* actor = ResolveFaceGenActor(a_faceNode, a_skeleton);
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
		OriginalFaceGenSkinSingleGeometry(a_faceNode, a_skeleton, a_geometry, a_arg4);

		auto* actor = ResolveFaceGenActor(a_faceNode, a_skeleton);
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
			Smp::Fo4PhysicsWorld::GetSingleton()->NoteCharacterCustomizationTarget(actor, a_editMode);
		}

		OriginalLooksMenuUtilsShowLooksMenu(a_target, a_editMode, a_target2, a_swapTarget, a_vendor);
	}

	void HookedMainOnIdleUpdateHighActorsArraySorted(RE::Main* a_main, float a_distance)
	{
		OriginalMainOnIdleUpdateHighActorsArraySorted(a_main, a_distance);
		Smp::Fo4PhysicsWorld::GetSingleton()->StepFrame();
	}

	void HookedMainSwap(RE::Main* a_main)
	{
		Smp::Fo4PhysicsWorld::GetSingleton()->WriteBackPrototypeBodies(Smp::WritebackSource::kMainSync);
		Smp::ImguiLayer::RenderFrame();
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
		REL::Relocation<std::uintptr_t> faceGenVTable{ RE::VTABLE::BSFaceGenNiNode[0] };

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
		if (!OriginalFaceGenSkinAllGeometry) {
			OriginalFaceGenSkinAllGeometry = InstallVFuncHook<FaceGenSkinAllGeometry_t>(
				"BSFaceGenNiNode::SkinAllGeometry",
				faceGenVTable,
				kFaceGenSkinAllGeometryVFuncSlot,
				reinterpret_cast<void*>(&HookedFaceGenSkinAllGeometry));
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
		if (!OriginalLooksMenuUtilsShowLooksMenu) {
			OriginalLooksMenuUtilsShowLooksMenu = reinterpret_cast<LooksMenuUtilsShowLooksMenu_t>(Addresses::LooksMenuUtilsShowLooksMenu.address());
			DetourTransactionBegin();
			DetourUpdateThread(GetCurrentThread());
			const auto detourError = DetourAttach(
				reinterpret_cast<PVOID*>(std::addressof(OriginalLooksMenuUtilsShowLooksMenu)),
				reinterpret_cast<PVOID>(&HookedLooksMenuUtilsShowLooksMenu));
			const auto commitError = DetourTransactionCommit();
			if (detourError != NO_ERROR || commitError != NO_ERROR) {
				spdlog::error(
					"LooksMenuUtils::ShowLooksMenu detour failed attachError={} commitError={} target={}",
					detourError,
					commitError,
					reinterpret_cast<void*>(Addresses::LooksMenuUtilsShowLooksMenu.address()));
				OriginalLooksMenuUtilsShowLooksMenu = nullptr;
			} else {
				spdlog::info("LooksMenuUtils::ShowLooksMenu detour installed at {:x}", Addresses::LooksMenuUtilsShowLooksMenu.address());
			}
		}
		if (isOG && !OriginalFaceGenSkinSingleGeometry) {
			const auto skinSingleCallsite = Addresses::BSFaceGenAddHeadPartOnActor.address() + (isOG ? kAddHeadPartSkinSingleCallOffsetOG : kAddHeadPartSkinSingleCallOffsetOG);
			OriginalFaceGenSkinSingleGeometry = reinterpret_cast<FaceGenSkinSingleGeometry_t>(
				REL::GetTrampoline().write_call<5>(skinSingleCallsite, reinterpret_cast<std::uintptr_t>(&HookedFaceGenSkinSingleGeometry)));
			spdlog::info("BSFaceGenUtils::AddHeadPartOnActor skin-single call hook installed at {:x}", skinSingleCallsite);
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
			OriginalFaceGenSkinAllGeometry &&
			OriginalUpdate3DModel &&
			OriginalReset3D &&
			OriginalLooksMenuUtilsShowLooksMenu &&
			(!isOG || OriginalSetFaceGenBoneName) &&
			(!isOG || OriginalFaceGenSkinSingleGeometry);

		spdlog::info("FO4 Faster HDT-SMP lifecycle hooks {}", installed ? "installed" : "failed");
		return installed;
	}
}
