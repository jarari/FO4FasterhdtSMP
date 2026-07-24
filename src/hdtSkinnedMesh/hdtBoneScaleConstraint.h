#pragma once

#include "hdtBulletHelper.h"
#include "hdtSkinnedMeshBone.h"

namespace hdt
{
	class alignas(16) BoneScaleConstraint :
		public RefObject
	{
	public:
		BoneScaleConstraint(SkinnedMeshBone* a_boneA, SkinnedMeshBone* a_boneB, btTypedConstraint* a_constraint);
		~BoneScaleConstraint() override = default;

		virtual void scaleConstraint() = 0;

		[[nodiscard]] btTypedConstraint* getConstraint() const { return m_constraint; }

		float m_scaleA{ 1.0F };
		float m_scaleB{ 1.0F };
		SkinnedMeshBone* m_boneA{ nullptr };
		SkinnedMeshBone* m_boneB{ nullptr };
		btTypedConstraint* m_constraint{ nullptr };
	};
}
