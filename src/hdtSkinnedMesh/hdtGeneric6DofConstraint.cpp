#include "hdtGeneric6DofConstraint.h"

namespace hdt
{
	namespace
	{
		float WeightedScale(
			const float a_factorA,
			const float a_factorB,
			const float a_inverseMassA,
			const float a_inverseMassB)
		{
			const auto weight = a_inverseMassA + a_inverseMassB;
			return weight > FLT_EPSILON ?
				(a_factorA * a_inverseMassA + a_factorB * a_inverseMassB) / weight :
				(a_factorA + a_factorB) * 0.5F;
		}
	}

	Generic6DofConstraint::Generic6DofConstraint(
		SkinnedMeshBone* a_boneA,
		SkinnedMeshBone* a_boneB,
		const btTransform& a_frameInA,
		const btTransform& a_frameInB) :
		BoneScaleConstraint(a_boneA, a_boneB, static_cast<btTypedConstraint*>(this)),
		btGeneric6DofSpring2Constraint(
			a_boneA->m_rig,
			a_boneB->m_rig,
			btTransform::getIdentity(),
			btTransform::getIdentity(),
			RO_XYZ)
	{
		setFrames(a_boneA->m_rigToLocal * a_frameInA, a_boneB->m_rigToLocal * a_frameInB);
		for (int axis = 0; axis < 6; ++axis) {
			enableSpring(axis, true);
		}
	}

	void Generic6DofConstraint::scaleConstraint()
	{
		const auto newScaleA = m_boneA->m_currentTransform.getScale();
		const auto newScaleB = m_boneB->m_currentTransform.getScale();
		if (btFuzzyZero(newScaleA - m_scaleA) && btFuzzyZero(newScaleB - m_scaleB)) {
			return;
		}

		const auto factorA = newScaleA / std::max(m_scaleA, FLT_EPSILON);
		const auto factorB = newScaleB / std::max(m_scaleB, FLT_EPSILON);
		const auto factor = WeightedScale(
			factorA,
			factorB,
			m_boneA->m_rig.getInvMass(),
			m_boneB->m_rig.getInvMass());
		const auto factorSquared = factor * factor;
		const auto factorCubed = factorSquared * factor;
		const auto factorFifth = factorCubed * factorSquared;

		auto frameA = getFrameOffsetA();
		auto frameB = getFrameOffsetB();
		frameA.setOrigin(frameA.getOrigin() * factorA);
		frameB.setOrigin(frameB.getOrigin() * factorB);
		setFrames(frameA, frameB);

		auto* linear = getTranslationalLimitMotor();
		linear->m_equilibriumPoint *= factor;
		linear->m_springStiffness *= factorCubed;
		linear->m_upperLimit *= factor;
		linear->m_lowerLimit *= factor;
		for (int axis = 0; axis < 3; ++axis) {
			if (auto* angular = getRotationalLimitMotor(axis)) {
				angular->m_springStiffness *= factorFifth;
			}
		}

		m_scaleA = newScaleA;
		m_scaleB = newScaleB;
	}
}
