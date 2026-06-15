#include "Hooks.h"

#include "ActorSkeletonBinding.h"
#include "BSBoneMap.h"
#include "BSSkin.h"
#include "Fo4NiObjectUtils.h"
#include "Fo4PhysicsWorld.h"
#include "ImguiLayer.h"
#include "LifecycleEvents.h"
#include "PhysicsName.h"
#include "PhysicsXmlSelection.h"
#include "SmpConfig.h"
#include "RE/B/BSAnimationGraphManager.h"
#include "RE/B/BSGeometry.h"
#include "RE/H/hkArray.h"
#include "RE/H/hkReferencedObject.h"
#include "RE/H/hkRefPtr.h"
#include "RE/M/Main.h"
#include "RE/N/NiCloningProcess.h"
#include "RE/N/NiNode.h"
#include "RE/N/NiPointer.h"
#include "RE/P/PlayerCharacter.h"
#include "RE/N/NiStringExtraData.h"
#include "RE/T/TESObjectREFR.h"

#if defined(_M_X64) && !defined(_AMD64_)
#	define _AMD64_ 1
#endif
#include <Windows.h>
#include <detours.h>

#include <atomic>
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

	std::mutex               BackupNodeLock;
	std::vector<std::string> BackupNodeNames;
	thread_local std::uint32_t ApplySkinnedObjectsDepth{ 0 };
	std::mutex FaceGenActorLock;
	std::unordered_map<RE::BSFaceGenNiNode*, RE::ActorHandle> FaceGenActorMap;
	std::atomic<std::uint32_t> ArmorMergeId{ 1 };

	using BackupBoneMap = std::unordered_map<std::string, std::vector<RE::NiPointer<RE::NiAVObject>>>;

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
		RE::NiPointer<RE::NiAVObject> mergeSourceObject;
		std::vector<RE::NiAVObject*> trustedActorSkeletonNodes;
		std::vector<Smp::MergeParentBinding> mergeParentBindings;
		std::vector<Smp::MergeRename> mergeRenameMap;
		std::string mergeRenamePrefix;
	};

	RE::NiNode* FindNode(RE::NiAVObject* a_object, const std::string_view a_name)
	{
		if (!a_object || a_name.empty()) {
			return nullptr;
		}

		auto* node = a_object->IsNode();
		if (!node) {
			return nullptr;
		}

		const auto nodeName = node->GetName();
		if (!nodeName.empty() && Smp::PhysicsNamesEqual(nodeName, a_name)) {
			return node;
		}

		for (auto& child : node->children) {
			if (auto* found = FindNode(child.get(), a_name)) {
				return found;
			}
		}
		return nullptr;
	}

	RE::NiNode* GetSkeletonMergeRoot(RE::Actor* a_actor, const bool a_firstPerson)
	{
		auto* actorRoot = a_actor ? a_actor->Get3D(a_firstPerson) : nullptr;
		if (!actorRoot && a_actor) {
			actorRoot = a_actor->Get3D();
		}
		auto* actorRootNode = actorRoot ? actorRoot->IsNode() : nullptr;
		if (!actorRootNode) {
			return nullptr;
		}

		if (auto* rootNode = FindNode(actorRootNode, "Root")) {
			return rootNode;
		}
		return actorRootNode;
	}

	RE::NiNode* FindTrustedActorSkeletonNode(
		const std::vector<RE::NiAVObject*>& a_trustedActorSkeletonNodes,
		const std::string_view a_name)
	{
		if (a_name.empty()) {
			return nullptr;
		}

		for (auto* object : a_trustedActorSkeletonNodes) {
			if (!object) {
				continue;
			}

			auto* node = object->IsNode();
			if (!node) {
				continue;
			}

			const auto nodeName = node->GetName();
			if (!nodeName.empty() && Smp::PhysicsNamesEqual(nodeName, a_name)) {
				return node;
			}
		}
		return nullptr;
	}

	RE::NiNode* FindActorSkeletonDescendantNode(RE::NiNode* a_expectedParent, const std::string_view a_name)
	{
		if (!a_expectedParent || a_name.empty()) {
			return nullptr;
		}

		for (auto& child : a_expectedParent->children) {
			auto* object = child.get();
			if (!object) {
				continue;
			}

			const auto name = object->GetName();
			if (!name.empty() && !Smp::IsAutoRenamedPhysicsName(name) && Smp::PhysicsNamesEqual(name, a_name)) {
				return object->IsNode();
			}

			auto* node = object->IsNode();
			if (!node || (!name.empty() && Smp::IsAutoRenamedPhysicsName(name))) {
				continue;
			}

			if (auto* found = FindActorSkeletonDescendantNode(node, a_name)) {
				return found;
			}
		}
		return nullptr;
	}

	std::string MakeArmorMergePrefix()
	{
		char buffer[48]{};
		std::snprintf(buffer, sizeof(buffer), "hdtSSEPhysics_AutoRename_Armor_%08X ", ArmorMergeId.fetch_add(1, std::memory_order_relaxed));
		return buffer;
	}

	RE::NiPointer<RE::NiAVObject> CloneNodeExact(RE::NiNode* a_source)
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
		return cloneObject ? static_cast<RE::NiAVObject*>(cloneObject->IsNode()) : nullptr;
	}

	void SetNodeName(RE::NiNode* a_node, const std::string& a_name)
	{
		if (a_node) {
			a_node->name = a_name.c_str();
		}
	}

	bool IsClassicHolsteredWeaponBoneName(const std::string_view a_name)
	{
		return a_name.size() >= 3 &&
			a_name[0] == 'V' &&
			a_name[1] == 'H' &&
			a_name[2] == 'W';
	}

	void RenameTree(RE::NiNode* a_root, const std::string_view a_prefix, std::vector<Smp::MergeRename>& a_map)
	{
		if (!a_root) {
			return;
		}

		const auto name = a_root->GetName();
		if (!name.empty()) {
			std::string renamed{ a_prefix };
			renamed += std::string_view(name);
			if (!std::ranges::any_of(a_map, [name](const Smp::MergeRename& a_entry) {
					return Smp::PhysicsNamesEqual(a_entry.sourceName, name);
				})) {
				a_map.push_back({
					.sourceName = std::string(name),
					.renamedName = renamed,
				});
				spdlog::debug("Rename Bone {} -> {}.", std::string_view(name), renamed);
			}
			SetNodeName(a_root, renamed);
		}

		for (auto& child : a_root->children) {
			if (auto* childNode = child ? child->IsNode() : nullptr) {
				RenameTree(childNode, a_prefix, a_map);
			}
		}
	}

	void StripNonNodeChildren(RE::NiNode* a_root)
	{
		if (!a_root) {
			return;
		}

		std::vector<RE::NiAVObject*> detachedChildren;
		for (auto& child : a_root->children) {
			auto* object = child.get();
			if (!object) {
				continue;
			}

			if (auto* node = object->IsNode()) {
				StripNonNodeChildren(node);
			} else {
				detachedChildren.push_back(object);
			}
		}

		for (auto* child : detachedChildren) {
			a_root->DetachChild(child);
		}
	}

	RE::NiNode* CloneNodeTree(RE::NiNode* a_source, const std::string_view a_prefix, std::vector<Smp::MergeRename>& a_map, const bool a_renameSource)
	{
		if (!a_source) {
			return nullptr;
		}

		RE::NiCloningProcess cloneProcess;
		cloneProcess.appendChar = '$';
		cloneProcess.copyType = RE::NiCloningProcess::CopyType::kCopyExact;
		cloneProcess.scale = { 1.0F, 1.0F, 1.0F };

		auto* clone = a_source->CreateClone(cloneProcess);
		a_source->ProcessClone(cloneProcess);
		auto* cloneNode = clone ? clone->IsNode() : nullptr;
		if (!cloneNode) {
			return nullptr;
		}

		if (a_renameSource) {
			RenameTree(a_source, a_prefix, a_map);
		}
		RenameTree(cloneNode, a_prefix, a_map);
		StripNonNodeChildren(cloneNode);
		return cloneNode;
	}

	void DoSkeletonMerge(
		RE::NiNode* a_destination,
		RE::NiNode* a_source,
		const std::string_view a_prefix,
		std::vector<Smp::MergeRename>& a_map,
		RE::NiNode* a_destinationRoot,
		const std::vector<RE::NiAVObject*>& a_trustedActorSkeletonNodes,
		const bool a_renameSource)
	{
		if (!a_destination || !a_source || !a_destinationRoot) {
			return;
		}

		for (auto& child : a_source->children) {
			auto* sourceChild = child ? child->IsNode() : nullptr;
			if (!sourceChild) {
				continue;
			}

			const auto childName = sourceChild->GetName();
			if (childName.empty()) {
				DoSkeletonMerge(a_destination, sourceChild, a_prefix, a_map, a_destinationRoot, a_trustedActorSkeletonNodes, a_renameSource);
				continue;
			}

			if (Smp::PhysicsNamesEqual(childName, "BSFaceGenNiNodeSkinned")) {
				spdlog::debug("Skipping facegen ninode in skeleton merge.");
				continue;
			}
			if (IsClassicHolsteredWeaponBoneName(childName)) {
				spdlog::debug("Skipping Classic Holstered Weapon helper bone '{}' in skeleton merge.", std::string_view(childName));
				continue;
			}

			auto* destinationChild = FindTrustedActorSkeletonNode(a_trustedActorSkeletonNodes, childName);
			if (!destinationChild) {
				destinationChild = FindActorSkeletonDescendantNode(a_destination, childName);
				if (destinationChild) {
					spdlog::debug(
						"using actor skeleton descendant '{}' node={} under expected parent={} parentName='{}' during armor skeleton merge",
						childName,
						static_cast<void*>(destinationChild),
						static_cast<void*>(a_destination),
						std::string_view(a_destination->GetName()));
				}
			}
			if (destinationChild) {
				DoSkeletonMerge(destinationChild, sourceChild, a_prefix, a_map, a_destinationRoot, a_trustedActorSkeletonNodes, a_renameSource);
			} else if (auto* clone = CloneNodeTree(sourceChild, a_prefix, a_map, a_renameSource)) {
				a_destination->AttachChild(clone, false);
			}
		}
	}

	void DoSkeletonMerge(
		RE::NiNode* a_destination,
		RE::NiNode* a_source,
		const std::string_view a_prefix,
		std::vector<Smp::MergeRename>& a_map,
		const std::vector<RE::NiAVObject*>& a_trustedActorSkeletonNodes,
		const bool a_renameSource = true)
	{
		DoSkeletonMerge(a_destination, a_source, a_prefix, a_map, a_destination, a_trustedActorSkeletonNodes, a_renameSource);
	}

	PreAttachPhysicsContext PreparePreAttachPhysicsContext(
		RE::BipedAnim* a_biped,
		RE::NiNode* a_sourceRoot,
		RE::NiAVObject* a_sourceObject,
		const RE::BIPED_OBJECT a_bipedObject,
		const bool a_firstPerson,
		const char* a_sourceLabel)
	{
		PreAttachPhysicsContext context;
		context.selectedXml = FindPhysicsXmlExtraData(a_sourceObject);
		if (!context.selectedXml) {
			return context;
		}

		auto* actor = ResolveActor(a_biped);
		if (auto reusable = Smp::Fo4PhysicsWorld::GetSingleton()->FindReusablePendingArmorMergeState(actor, a_firstPerson, a_bipedObject, *context.selectedXml)) {
			context.mergeSourceObject = std::move(reusable->mergeSourceSnapshot);
			context.mergeParentBindings = std::move(reusable->mergeParentBindings);
			context.mergeRenameMap = std::move(reusable->mergeRenameMap);
			spdlog::trace(
				"reused pending pre-attach armor merge state actor={} bipedObject={} xml='{}' parentBindings={} renameMap={} mergeSource={}",
				static_cast<void*>(actor),
				std::to_underlying(a_bipedObject),
				*context.selectedXml,
				context.mergeParentBindings.size(),
				context.mergeRenameMap.size(),
				static_cast<void*>(context.mergeSourceObject.get()));
			return context;
		}
		context.trustedActorSkeletonNodes = Smp::ActorSkeletonBinding::CaptureTrustedActorSkeletonNodesBeforeAttach(
			actor,
			a_biped,
			a_sourceObject,
			a_firstPerson);
		Smp::ActorSkeletonBinding::PruneTrustedActorSkeletonNodesBySourceParents(a_sourceObject, context.trustedActorSkeletonNodes);
		context.mergeParentBindings = Smp::ActorSkeletonBinding::BuildPreAttachMergeParentBindings(a_sourceObject, context.trustedActorSkeletonNodes);
		context.mergeRenamePrefix = MakeArmorMergePrefix();
		context.mergeSourceObject = CloneNodeExact(a_sourceRoot);
		if (auto* mergeRoot = GetSkeletonMergeRoot(actor, a_firstPerson)) {
			DoSkeletonMerge(mergeRoot, a_sourceRoot, context.mergeRenamePrefix, context.mergeRenameMap, context.trustedActorSkeletonNodes, false);
			Smp::RefreshBoneScatterTable(mergeRoot);
			spdlog::debug(
				"merged pre-attach armor skeleton source={} name='{}' into actor merge root={} name='{}' prefix='{}' renamedBones={}",
				static_cast<void*>(a_sourceRoot),
				a_sourceRoot ? std::string_view(a_sourceRoot->GetName()) : std::string_view{},
				static_cast<void*>(mergeRoot),
				std::string_view(mergeRoot->GetName()),
				context.mergeRenamePrefix,
				context.mergeRenameMap.size());
		}
		spdlog::debug(
			"pre-scanned armor physics XML {} from {}={} name='{}'",
			*context.selectedXml,
			a_sourceLabel,
			static_cast<void*>(a_sourceRoot),
			a_sourceRoot ? std::string_view(a_sourceRoot->GetName()) : std::string_view{});

		return context;
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
		auto* originalModelObject = a_originalModelRoot ? static_cast<RE::NiAVObject*>(a_originalModelRoot) : nullptr;
		auto preAttach = PreparePreAttachPhysicsContext(
			a_biped,
			a_originalModelRoot,
			originalModelObject,
			a_bipedObject,
			a_firstPerson,
			"original model root");

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
			.mergeSourceObject = preAttach.mergeSourceObject.get(),
			.trustedActorSkeletonNodes = std::move(preAttach.trustedActorSkeletonNodes),
			.mergeParentBindings = std::move(preAttach.mergeParentBindings),
			.mergeRenameMap = std::move(preAttach.mergeRenameMap),
			.mergeRenamePrefix = std::move(preAttach.mergeRenamePrefix),
			.physicsXmlPath = preAttach.selectedXml.value_or(std::string{}),
			.firstPerson = a_firstPerson,
		});
		return attachedObject;
	}

	RE::NiAVObject* HookedBipedAnimAttachSkinnedObject(RE::BipedAnim* a_biped, RE::NiNode* a_destinationRoot, RE::NiNode* a_sourceRoot, RE::BIPED_OBJECT a_bipedObject, bool a_firstPerson)
	{
		auto* sourceObject = a_sourceRoot ? static_cast<RE::NiAVObject*>(a_sourceRoot) : nullptr;
		auto preAttach = PreparePreAttachPhysicsContext(
			a_biped,
			a_sourceRoot,
			sourceObject,
			a_bipedObject,
			a_firstPerson,
			"source root");
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
			.mergeSourceObject = preAttach.mergeSourceObject.get(),
			.trustedActorSkeletonNodes = std::move(preAttach.trustedActorSkeletonNodes),
			.mergeParentBindings = std::move(preAttach.mergeParentBindings),
			.mergeRenameMap = std::move(preAttach.mergeRenameMap),
			.destinationRoot = a_destinationRoot,
			.sourceRoot = a_sourceRoot,
			.mergeRenamePrefix = std::move(preAttach.mergeRenamePrefix),
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
