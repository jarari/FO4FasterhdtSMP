#pragma once

#include "hdtBulletHelper.h"

namespace hdt
{
	struct alignas(16) Bone
	{
		Bone()
		{
			_mm_store_ps(reserved_, _mm_setzero_ps());
		}

		btMatrix4x3T vertexToWorld_;
		float reserved_[3];
		float marginMultiplier_;
	};
}
