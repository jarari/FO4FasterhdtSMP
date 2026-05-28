#pragma once

#include "hdtCollider.h"

namespace hdt
{
	struct CollisionResult
	{
		btVector3 posA;
		btVector3 posB;
		btVector3 normOnB;
		Collider* colliderA{ nullptr };
		Collider* colliderB{ nullptr };
		float depth{ 0.0F };
	};

	bool checkSphereSphere(const btVector3& a_lhs, const btVector3& a_rhs, float a_lhsRadius, float a_rhsRadius, CollisionResult& a_result);

	inline btVector3 BaryCoord(const btVector3& a_lhs, const btVector3& a_mid, const btVector3& a_rhs, const btVector3& a_point)
	{
		const auto lhs = a_lhs - a_point;
		const auto mid = a_mid - a_point;
		const auto rhs = a_rhs - a_point;
		auto x = btCross(lhs, mid).get128();
		const auto y = btCross(mid, rhs).get128();
		const auto z = btCross(rhs, lhs).get128();
		x = _mm_dp_ps(x, x, 0x74);
		x = _mm_or_ps(x, _mm_dp_ps(y, y, 0x71));
		x = _mm_or_ps(x, _mm_dp_ps(z, z, 0x72));
		x = _mm_sqrt_ps(x);
		const auto sum = _mm_dp_ps(_mm_set_ps1(1.0F), x, 0x77);
		return vectorFromM128(_mm_div_ps(x, sum));
	}
}
