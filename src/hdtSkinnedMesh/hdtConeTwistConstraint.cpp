#include "hdtConeTwistConstraint.h"

namespace hdt
{
	ConeTwistConstraint::ConeTwistConstraint(
		SkinnedMeshBone* a_boneA,
		SkinnedMeshBone* a_boneB,
		const btTransform& a_frameInA,
		const btTransform& a_frameInB) :
		BoneScaleConstraint(a_boneA, a_boneB, static_cast<btConeTwistConstraint*>(this)),
		btConeTwistConstraint(
			a_boneA->m_rig,
			a_boneB->m_rig,
			btTransform::getIdentity(),
			btTransform::getIdentity())
	{
		setFrames(a_boneA->m_rigToLocal * a_frameInA, a_boneB->m_rigToLocal * a_frameInB);
		enableMotor(false);
	}

	void ConeTwistConstraint::scaleConstraint()
	{
		const auto newScaleA = m_boneA->m_currentTransform.getScale();
		const auto newScaleB = m_boneB->m_currentTransform.getScale();
		if (btFuzzyZero(newScaleA - m_scaleA) && btFuzzyZero(newScaleB - m_scaleB)) {
			return;
		}

		auto frameA = getFrameOffsetA();
		auto frameB = getFrameOffsetB();
		frameA.setOrigin(frameA.getOrigin() * (newScaleA / std::max(m_scaleA, FLT_EPSILON)));
		frameB.setOrigin(frameB.getOrigin() * (newScaleB / std::max(m_scaleB, FLT_EPSILON)));
		setFrames(frameA, frameB);
		m_scaleA = newScaleA;
		m_scaleB = newScaleB;
	}
}
