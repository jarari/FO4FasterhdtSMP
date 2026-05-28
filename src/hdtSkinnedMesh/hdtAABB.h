#pragma once

#include "hdtBulletHelper.h"

namespace hdt
{
	struct Aabb
	{
		Aabb() { invalidate(); }

		Aabb(__m128 a_min, __m128 a_max) :
			min_(a_min), max_(a_max)
		{}

		bool collideWith(const Aabb& a_rhs) const
		{
			const auto minFlag = _mm_cmplt_ps(a_rhs.max_, min_);
			const auto maxFlag = _mm_cmplt_ps(max_, a_rhs.min_);
			const auto flag = _mm_movemask_ps(_mm_or_ps(minFlag, maxFlag));
			return !(flag & 0x7);
		}

		void invalidate()
		{
			min_ = setAll(FLT_MAX);
			max_ = setAll(-FLT_MAX);
		}

		void merge(const btVector3& a_point)
		{
			min_ = _mm_min_ps(min_, a_point.get128());
			max_ = _mm_max_ps(max_, a_point.get128());
		}

		void merge(const Aabb& a_rhs)
		{
			min_ = _mm_min_ps(min_, a_rhs.min_);
			max_ = _mm_max_ps(max_, a_rhs.max_);
		}

		__m128 min_;
		__m128 max_;
	};

	struct BoundingSphere
	{
		BoundingSphere() = default;

		BoundingSphere(const btVector3& a_center, float a_radius) :
			centerRadius_(a_center.x(), a_center.y(), a_center.z(), a_radius)
		{}

		btVector3 center() const { return centerRadius_; }
		float radius() const { return centerRadius_.w(); }

		Aabb getAabb() const
		{
			const auto center = centerRadius_.get128();
			const auto radius = _mm_shuffle_ps(center, center, _MM_SHUFFLE(3, 3, 3, 3));
			return Aabb(_mm_sub_ps(center, radius), _mm_add_ps(center, radius));
		}

		btVector4 centerRadius_;
	};
}
