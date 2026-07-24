#include "hdtBoneScaleConstraint.h"

namespace hdt
{
	BoneScaleConstraint::BoneScaleConstraint(
		SkinnedMeshBone* a_boneA,
		SkinnedMeshBone* a_boneB,
		btTypedConstraint* a_constraint) :
		m_boneA(a_boneA),
		m_boneB(a_boneB),
		m_constraint(a_constraint)
	{}
}
