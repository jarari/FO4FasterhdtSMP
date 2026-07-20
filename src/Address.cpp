#include "Address.h"

namespace Smp::Address
{
	namespace
	{
		const REL::ID MainOnIdleID{ 633524, 2228917 };
		const REL::ID BSFaceGenAddHeadPartOnActorID{ 913780, 2209545 };
	}

	REL::Relocation<ActorCalculateLOS_t*> ActorCalculateLOS{ REL::ID{ 1324305, 2230201 } };
	REL::Relocation<CreateBoneMap_t*> CreateBoneMap{ REL::ID{ 1131947, 2276147 } };

	REL::Relocation<std::uintptr_t> MainOnIdle{ MainOnIdleID };
	REL::Relocation<std::uintptr_t> BipedAnimApplySkinnedObjects{ REL::ID{ 224320, 2194381 } };
	REL::Relocation<std::uintptr_t> BipedAnimAttachSkinnedObject{ REL::ID{ 1575810, 2194388 } };
	REL::Relocation<std::uintptr_t> BipedAnimAttachToParent{ REL::ID{ 1370428, 2194378 } };
	REL::Relocation<std::uintptr_t> BipedAnimRemovePart{ REL::ID{ 575576, 2194342 } };
	REL::Relocation<std::uintptr_t> Update3DModel{ REL::ID{ 986782, 2231882 } };
	REL::Relocation<std::uintptr_t> Reset3D{ REL::ID{ 302888, 2229913 } };
	REL::Relocation<std::uintptr_t> BSFaceGenAddHeadPartOnActor{ BSFaceGenAddHeadPartOnActorID };
	REL::Relocation<std::uintptr_t> BSFaceGenModelExtraDataSetBoneName{ REL::ID{ 1278503, 2209387 } };
	REL::Relocation<std::uintptr_t> LooksMenuUtilsShowLooksMenu{ REL::ID{ 411372, 2223366 } };

	const IDOffset MainOnIdleSwapCall{
		MainOnIdleID,
		VariantOffset{ 0x6EC, 0xCDC }
	};
	const IDOffset BSFaceGenAddHeadPartOnActorSkinSingleCall{
		BSFaceGenAddHeadPartOnActorID,
		VariantOffset{ 0xFD, 0x107 }
	};

	// Each value ends on an instruction boundary in the corresponding runtime.
	const VariantOffset BipedAnimApplySkinnedObjectsPrologueSize{ 14, 15 };
	const VariantOffset BipedAnimAttachSkinnedObjectPrologueSize{ 15, 16 };
	const VariantOffset BipedAnimAttachToParentPrologueSize{ 15, 15 };
	const VariantOffset BipedAnimRemovePartPrologueSize{ 15, 18 };
	const VariantOffset Update3DModelPrologueSize{ 5, 5 };
	const VariantOffset Reset3DPrologueSize{ 5, 5 };
	const VariantOffset BSFaceGenModelExtraDataSetBoneNamePrologueSize{ 5, 5 };
	const VariantOffset LooksMenuUtilsShowLooksMenuPrologueSize{ 5, 5 };

	const std::size_t ActorLoad3DVFuncSlot = 0x86;
	const std::size_t ActorSet3DVFuncSlot = 0x88;
	const std::size_t ActorOnHeadInitializedVFuncSlot = 0x98;
	const std::size_t BSFaceGenSkinAllGeometryVFuncSlot = 0x43;
}
