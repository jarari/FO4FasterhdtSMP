#pragma once

#include "hdtBoneScaleConstraint.h"

namespace hdt
{
	class StiffSpringConstraint :
		public BoneScaleConstraint,
		public btTypedConstraint
	{
	public:
		BT_DECLARE_ALIGNED_ALLOCATOR();

		StiffSpringConstraint(SkinnedMeshBone* a_boneA, SkinnedMeshBone* a_boneB);
		~StiffSpringConstraint() override = default;

		void scaleConstraint() override;

		float m_minDistance{ 0.0F };
		float m_maxDistance{ 0.0F };
		float m_stiffness{ 0.0F };
		float m_damping{ 0.0F };
		float m_equilibriumPoint{ 0.0F };

	protected:
		void getInfo1(btConstraintInfo1* a_info) override;
		void getInfo2(btConstraintInfo2* a_info) override;
		void setParam([[maybe_unused]] int a_num, [[maybe_unused]] btScalar a_value, [[maybe_unused]] int a_axis = -1) override {}
		[[nodiscard]] btScalar getParam([[maybe_unused]] int a_num, [[maybe_unused]] int a_axis = -1) const override { return 0.0F; }

		float m_oldDiff{ 0.0F };
	};
}
