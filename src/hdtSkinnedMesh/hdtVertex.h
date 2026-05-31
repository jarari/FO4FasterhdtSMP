#pragma once

#include "hdtBone.h"

#include <cstring>

namespace hdt
{
	struct alignas(16) Vertex
	{
		Vertex()
		{
			std::memset(this, 0, sizeof(*this));
		}

		Vertex(float a_x, float a_y, float a_z) :
			Vertex()
		{
			skinPos_.setValue(a_x, a_y, a_z);
		}

		U32 getBoneIdx(int a_index) const { return boneIdx_[a_index]; }
		void setBoneIdx(int a_index, U32 a_indexValue) { boneIdx_[a_index] = a_indexValue; }
		void sortWeight();

		btVector3 skinPos_;
		float weight_[4];
		U32 boneIdx_[4];
	};

	struct alignas(16) VertexPos
	{
		void set(const btVector3& a_position, float a_marginMultiplier)
		{
			data_ = a_position.get128();
			data_.m128_f32[3] = a_marginMultiplier;
		}

		void set(const btVector4& a_positionAndMargin)
		{
			data_ = a_positionAndMargin.get128();
		}

		void set(__m128 a_positionAndMargin)
		{
			data_ = a_positionAndMargin;
		}

		btVector3 pos() const { return vectorFromM128(data_); }
		__m128 marginMultiplier4() const { return pshufd<0xFF>(data_); }
		float marginMultiplier() const { return _mm_cvtss_f32(marginMultiplier4()); }

		__m128 data_;
	};
}
