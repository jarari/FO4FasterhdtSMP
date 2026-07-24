#pragma once

#include "hdtBoneScaleConstraint.h"

namespace hdt
{
	class Generic6DofConstraint :
		public BoneScaleConstraint,
		public btGeneric6DofSpring2Constraint
	{
	public:
		BT_DECLARE_ALIGNED_ALLOCATOR();

		Generic6DofConstraint(
			SkinnedMeshBone* a_boneA,
			SkinnedMeshBone* a_boneB,
			const btTransform& a_frameInA,
			const btTransform& a_frameInB);
		~Generic6DofConstraint() override = default;

		void scaleConstraint() override;
	};
}
