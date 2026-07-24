#pragma once

#include "hdtBoneScaleConstraint.h"

namespace hdt
{
	class ConeTwistConstraint :
		public BoneScaleConstraint,
		public btConeTwistConstraint
	{
	public:
		BT_DECLARE_ALIGNED_ALLOCATOR();

		ConeTwistConstraint(
			SkinnedMeshBone* a_boneA,
			SkinnedMeshBone* a_boneB,
			const btTransform& a_frameInA,
			const btTransform& a_frameInB);
		~ConeTwistConstraint() override = default;

		void scaleConstraint() override;
	};
}
