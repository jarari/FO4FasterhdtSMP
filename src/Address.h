#pragma once

#include "REL/Relocation.h"
#include "RE/B/BSFixedString.h"
#include "REX/FModule.h"

#include <cstddef>
#include <cstdint>

namespace RE
{
	class Actor;
	class BSFlattenedBoneTree;
	class NiAVObject;
	class NiNode;
	class NiPoint3;
}

namespace Smp::Address
{
	struct VariantOffset
	{
		std::uintptr_t og;
		std::uintptr_t ae;

		[[nodiscard]] std::uintptr_t value() const noexcept
		{
			return REX::FModule::IsRuntimeOG() ? og : ae;
		}
	};

	struct IDOffset
	{
		REL::ID id;
		VariantOffset offset;

		[[nodiscard]] std::uintptr_t address() const
		{
			return REL::Relocation<std::uintptr_t>{ id }.address() + offset.value();
		}
	};

	using ActorCalculateLOS_t = RE::NiAVObject*(RE::Actor*, const RE::NiPoint3&, RE::NiPoint3&, float);
	using BSFlattenedBoneTreeGetOrCreateBoneNode_t =
		RE::NiNode*(RE::BSFlattenedBoneTree*, const RE::BSFixedString&, bool&);
	using BSFlattenedBoneTreeFind_t = RE::BSFlattenedBoneTree*(RE::NiAVObject*, std::int32_t);

	extern REL::Relocation<ActorCalculateLOS_t*> ActorCalculateLOS;
	extern REL::Relocation<BSFlattenedBoneTreeGetOrCreateBoneNode_t*> BSFlattenedBoneTreeGetOrCreateBoneNode;
	extern REL::Relocation<BSFlattenedBoneTreeFind_t*> BSFlattenedBoneTreeFind;

	extern REL::Relocation<std::uintptr_t> MainOnIdle;
	extern REL::Relocation<std::uintptr_t> BipedAnimApplySkinnedObjects;
	extern REL::Relocation<std::uintptr_t> BipedAnimAttachSkinnedObject;
	extern REL::Relocation<std::uintptr_t> BipedAnimAttachToParent;
	extern REL::Relocation<std::uintptr_t> BipedAnimRemovePart;
	extern REL::Relocation<std::uintptr_t> TESObjectREFRFixDisplayedHeadParts;
	extern REL::Relocation<std::uintptr_t> Reset3D;
	extern REL::Relocation<std::uintptr_t> BSFaceGenAddHeadPartOnActor;
	extern REL::Relocation<std::uintptr_t> BSFaceGenModelExtraDataSetBoneName;
	extern REL::Relocation<std::uintptr_t> LooksMenuUtilsShowLooksMenu;

	extern const IDOffset MainOnIdleSwapCall;
	extern const IDOffset MainSwapUpdatePendingCustomizationTexturesCall;
	extern const IDOffset TESNPCReplaceRefModelLoadBipedPartsCall;
	extern const IDOffset BipedAnimAttachSkinnedObjectBakeChargenMorphsCall;
	extern const IDOffset BSFaceGenPendingHeadAttachBakeChargenMorphsCall;
	extern const IDOffset BSFaceGenAddHeadPartOnActorPrepareHeadPartCall;
	extern const IDOffset BSFaceGenAddHeadPartOnActorSkinSingleCall;

	extern const VariantOffset BipedAnimApplySkinnedObjectsPrologueSize;
	extern const VariantOffset BipedAnimAttachSkinnedObjectPrologueSize;
	extern const VariantOffset BipedAnimAttachToParentPrologueSize;
	extern const VariantOffset BipedAnimRemovePartPrologueSize;
	extern const VariantOffset TESObjectREFRFixDisplayedHeadPartsPrologueSize;
	extern const VariantOffset Reset3DPrologueSize;
	extern const VariantOffset BSFaceGenModelExtraDataSetBoneNamePrologueSize;
	extern const VariantOffset LooksMenuUtilsShowLooksMenuPrologueSize;

	extern const std::size_t ActorLoad3DVFuncSlot;
	extern const std::size_t ActorSet3DVFuncSlot;
	extern const std::size_t ActorOnHeadInitializedVFuncSlot;
	extern const std::size_t BSFaceGenSkinAllGeometryVFuncSlot;
}
