#pragma once

#include "REL/Relocation.h"
#include "REX/FModule.h"

#include <cstddef>
#include <cstdint>

namespace RE
{
	class Actor;
	class NiAVObject;
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

	extern REL::Relocation<ActorCalculateLOS_t*> ActorCalculateLOS;

	extern REL::Relocation<std::uintptr_t> MainOnIdle;
	extern REL::Relocation<std::uintptr_t> BipedAnimApplySkinnedObjects;
	extern REL::Relocation<std::uintptr_t> BipedAnimAttachSkinnedObject;
	extern REL::Relocation<std::uintptr_t> BipedAnimAttachToParent;
	extern REL::Relocation<std::uintptr_t> BipedAnimRemovePart;
	extern REL::Relocation<std::uintptr_t> Update3DModel;
	extern REL::Relocation<std::uintptr_t> Reset3D;
	extern REL::Relocation<std::uintptr_t> BSFaceGenAddHeadPartOnActor;
	extern REL::Relocation<std::uintptr_t> BSFaceGenModelExtraDataSetBoneName;
	extern REL::Relocation<std::uintptr_t> LooksMenuUtilsShowLooksMenu;

	extern const IDOffset MainOnIdleSwapCall;
	extern const IDOffset BSFaceGenAddHeadPartOnActorSkinSingleCall;

	extern const VariantOffset BipedAnimApplySkinnedObjectsPrologueSize;
	extern const VariantOffset BipedAnimAttachSkinnedObjectPrologueSize;
	extern const VariantOffset BipedAnimAttachToParentPrologueSize;
	extern const VariantOffset BipedAnimRemovePartPrologueSize;
	extern const VariantOffset Update3DModelPrologueSize;
	extern const VariantOffset Reset3DPrologueSize;
	extern const VariantOffset BSFaceGenModelExtraDataSetBoneNamePrologueSize;
	extern const VariantOffset LooksMenuUtilsShowLooksMenuPrologueSize;

	extern const std::size_t ActorLoad3DVFuncSlot;
	extern const std::size_t ActorSet3DVFuncSlot;
	extern const std::size_t ActorOnHeadInitializedVFuncSlot;
	extern const std::size_t BSFaceGenSkinAllGeometryVFuncSlot;
}
