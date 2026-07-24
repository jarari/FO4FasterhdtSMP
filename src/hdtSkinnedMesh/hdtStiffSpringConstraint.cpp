#include "hdtStiffSpringConstraint.h"

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

	StiffSpringConstraint::StiffSpringConstraint(SkinnedMeshBone* a_boneA, SkinnedMeshBone* a_boneB) :
		BoneScaleConstraint(a_boneA, a_boneB, this),
		btTypedConstraint(MAX_CONSTRAINT_TYPE, a_boneA->m_rig, a_boneB->m_rig)
	{
		const auto distance = a_boneA->m_currentTransform.getOrigin().distance(a_boneB->m_currentTransform.getOrigin());
		m_equilibriumPoint = distance;
		m_minDistance = distance;
		m_maxDistance = distance;
	}

	void StiffSpringConstraint::scaleConstraint()
	{
		const auto newScaleA = m_boneA->m_currentTransform.getScale();
		const auto newScaleB = m_boneB->m_currentTransform.getScale();
		if (btFuzzyZero(newScaleA - m_scaleA) && btFuzzyZero(newScaleB - m_scaleB)) {
			return;
		}

		const auto factor = WeightedScale(
			newScaleA / std::max(m_scaleA, FLT_EPSILON),
			newScaleB / std::max(m_scaleB, FLT_EPSILON),
			m_boneA->m_rig.getInvMass(),
			m_boneB->m_rig.getInvMass());
		const auto factorCubed = factor * factor * factor;
		m_minDistance *= factor;
		m_maxDistance *= factor;
		m_equilibriumPoint *= factor;
		m_stiffness *= factorCubed;
		m_damping *= factorCubed;
		m_scaleA = newScaleA;
		m_scaleB = newScaleB;
	}

	void StiffSpringConstraint::getInfo1(btConstraintInfo1* a_info)
	{
		const auto localA = m_boneA->m_rigToLocal * m_rbA.getWorldTransform();
		const auto localB = m_boneB->m_rigToLocal * m_rbB.getWorldTransform();
		const auto distance = (localA.getOrigin() - localB.getOrigin()).length();
		a_info->m_numConstraintRows = btFuzzyZero(distance) ? 0 : 1;
		a_info->nub = 0;
	}

	void StiffSpringConstraint::getInfo2(btConstraintInfo2* a_info)
	{
		const auto localA = m_boneA->m_rigToLocal * m_rbA.getWorldTransform();
		const auto localB = m_boneB->m_rigToLocal * m_rbB.getWorldTransform();
		const auto deltaVector = localA.getOrigin() - localB.getOrigin();
		const auto distance = deltaVector.length();
		if (btFuzzyZero(distance)) {
			return;
		}

		const auto direction = deltaVector.normalized();
		a_info->m_J1linearAxis[0] = direction[0];
		a_info->m_J1linearAxis[1] = direction[1];
		a_info->m_J1linearAxis[2] = direction[2];
		a_info->m_J2linearAxis[0] = -direction[0];
		a_info->m_J2linearAxis[1] = -direction[1];
		a_info->m_J2linearAxis[2] = -direction[2];

		int currentLimit = 0;
		float currentLimitError = 0.0F;
		if (distance < m_minDistance) {
			currentLimit = 2;
			currentLimitError = distance - m_minDistance;
		} else if (distance > m_maxDistance) {
			currentLimit = 1;
			currentLimitError = distance - m_maxDistance;
		}

		if (currentLimit == 0) {
			const auto delta = distance - m_equilibriumPoint;
			const auto velocity = (delta - m_oldDiff) * a_info->fps;
			auto force = (delta + m_oldDiff) * 0.5F * m_stiffness;
			const auto friction = m_damping * velocity;
			force += force * friction < 0.0F ?
				btClamped(friction, -btFabs(force), btFabs(force)) :
				friction;

			const auto targetVelocity = (a_info->fps / static_cast<btScalar>(a_info->m_numIterations)) * force;
			const auto maxMotorForce = btFabs(force) / a_info->fps;
			const auto motorFactor = getMotorFactor(
				distance,
				m_minDistance,
				m_maxDistance,
				targetVelocity,
				a_info->fps * a_info->erp);
			a_info->m_constraintError[0] =
				motorFactor * targetVelocity * (m_rbA.getInvMass() + m_rbB.getInvMass());
			a_info->m_lowerLimit[0] = -maxMotorForce;
			a_info->m_upperLimit[0] = maxMotorForce;
			m_oldDiff = delta;
			return;
		}

		a_info->m_constraintError[0] = a_info->fps * a_info->erp * currentLimitError;
		if (m_minDistance == m_maxDistance) {
			a_info->m_lowerLimit[0] = -SIMD_INFINITY;
			a_info->m_upperLimit[0] = SIMD_INFINITY;
		} else if (currentLimit == 1) {
			a_info->m_lowerLimit[0] = -SIMD_INFINITY;
			a_info->m_upperLimit[0] = 0.0F;
		} else {
			a_info->m_lowerLimit[0] = 0.0F;
			a_info->m_upperLimit[0] = SIMD_INFINITY;
		}
	}
}
