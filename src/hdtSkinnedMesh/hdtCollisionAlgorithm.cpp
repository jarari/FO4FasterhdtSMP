#include "hdtCollisionAlgorithm.h"

namespace hdt
{
	bool checkSphereSphere(const btVector3& a_lhs, const btVector3& a_rhs, float a_lhsRadius, float a_rhsRadius, CollisionResult& a_result)
	{
		const auto difference = a_lhs - a_rhs;
		const auto distanceSquared = difference.length2();
		const auto radiusSum = a_lhsRadius + a_rhsRadius;

		if (distanceSquared > radiusSum * radiusSum) {
			return false;
		}

		const auto length = btSqrt(distanceSquared);
		a_result.normOnB = btVector3(1.0F, 0.0F, 0.0F);
		if (length > FLT_EPSILON) {
			a_result.normOnB = difference / length;
		}

		a_result.depth = length - radiusSum;
		a_result.posA = a_lhs - a_result.normOnB * a_lhsRadius;
		a_result.posB = a_rhs + a_result.normOnB * a_rhsRadius;
		return true;
	}
}
