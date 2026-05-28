#pragma once

#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>

#include <atomic>
#include <bit>
#include <cassert>
#include <cfloat>
#include <cstdint>
#include <mutex>
#include <vector>
#include <intrin.h>

#undef min
#undef max

#define HDT_LOCK_GUARD(name, lock) std::lock_guard<decltype(lock)> name(lock)

namespace hdt
{
	using I8 = std::int8_t;
	using I16 = std::int16_t;
	using I32 = std::int32_t;
	using I64 = std::int64_t;

	using U8 = std::uint8_t;
	using U16 = std::uint16_t;
	using U32 = std::uint32_t;
	using U64 = std::uint64_t;

	template <int imm>
	__m128 pshufd(__m128 a_value)
	{
		return _mm_castsi128_ps(_mm_shuffle_epi32(_mm_castps_si128(a_value), imm));
	}

	inline __m128 setAll(float a_value) { return pshufd<0>(_mm_load_ss(&a_value)); }
	inline __m128 setAll0(__m128 a_value) { return pshufd<0>(a_value); }
	inline __m128 setAll1(__m128 a_value) { return pshufd<0x55>(a_value); }
	inline __m128 setAll2(__m128 a_value) { return pshufd<0xAA>(a_value); }
	inline __m128 setAll3(__m128 a_value) { return pshufd<0xFF>(a_value); }

	inline __m128& operator+=(__m128& a_lhs, __m128 a_rhs)
	{
		a_lhs = _mm_add_ps(a_lhs, a_rhs);
		return a_lhs;
	}

	inline __m128& operator-=(__m128& a_lhs, __m128 a_rhs)
	{
		a_lhs = _mm_sub_ps(a_lhs, a_rhs);
		return a_lhs;
	}

	inline __m128& operator*=(__m128& a_lhs, __m128 a_rhs)
	{
		a_lhs = _mm_mul_ps(a_lhs, a_rhs);
		return a_lhs;
	}

	inline __m128& operator+=(__m128& a_lhs, float a_rhs)
	{
		a_lhs = _mm_add_ps(a_lhs, setAll(a_rhs));
		return a_lhs;
	}

	inline __m128& operator-=(__m128& a_lhs, float a_rhs)
	{
		a_lhs = _mm_sub_ps(a_lhs, setAll(a_rhs));
		return a_lhs;
	}

	inline __m128& operator*=(__m128& a_lhs, float a_rhs)
	{
		a_lhs = _mm_mul_ps(a_lhs, setAll(a_rhs));
		return a_lhs;
	}

	inline __m128 cross(__m128 a_lhs, __m128 a_rhs)
	{
		auto lhs = pshufd<_MM_SHUFFLE(3, 0, 2, 1)>(a_lhs);
		auto rhs = pshufd<_MM_SHUFFLE(3, 0, 2, 1)>(a_rhs);

		rhs = _mm_mul_ps(rhs, a_lhs);
		lhs = _mm_mul_ps(lhs, a_rhs);
		rhs = _mm_sub_ps(rhs, lhs);

		return pshufd<_MM_SHUFFLE(3, 0, 2, 1)>(rhs);
	}

	inline float rsqrt(float a_value)
	{
		const auto value = _mm_set_ss(a_value);
		const auto estimate = _mm_rsqrt_ss(value);
		const auto multiplied = _mm_mul_ss(_mm_mul_ss(value, estimate), estimate);
		const auto halfEstimate = _mm_mul_ss(estimate, _mm_set_ss(0.5F));
		const auto refined = _mm_sub_ss(_mm_set_ss(3.0F), multiplied);
		return _mm_cvtss_f32(_mm_mul_ss(halfEstimate, refined));
	}

	template <class T>
	T abs(T a_rhs)
	{
		return a_rhs < 0 ? -a_rhs : a_rhs;
	}

	template <>
	inline float abs(float a_rhs)
	{
		return _mm_cvtss_f32(_mm_andnot_ps(_mm_set_ss(-0.0F), _mm_set_ss(a_rhs)));
	}

	template <class T>
	T min(const T& a_lhs, const T& a_rhs)
	{
		return a_lhs < a_rhs ? a_lhs : a_rhs;
	}

	template <class T>
	T max(const T& a_lhs, const T& a_rhs)
	{
		return a_lhs < a_rhs ? a_rhs : a_lhs;
	}

	inline int aligned(int a_value, int a_alignment) { return (a_value + a_alignment - 1) & -a_alignment; }

	template <int a_alignment>
	int aligned(int a_value)
	{
		return (a_value + a_alignment - 1) & -a_alignment;
	}

	inline U32 aligned2Pow(U32 a_limit)
	{
		return std::bit_floor(a_limit);
	}

	inline btVector3 vectorFromM128(__m128 a_value)
	{
		alignas(16) float values[4];
		_mm_store_ps(values, a_value);
		return btVector3(values[0], values[1], values[2]);
	}

	ATTRIBUTE_ALIGNED16(class)
	btQsTransform
	{
	public:
		BT_DECLARE_ALIGNED_ALLOCATOR();

		btQsTransform() :
			basis_(btQuaternion::getIdentity()), originScale_(0, 0, 0, 1)
		{}

		btQsTransform(const btQuaternion& a_rotation, const btVector3& a_translation, float a_scale = 1.0F) :
			basis_(a_rotation)
		{
#ifdef BT_ALLOW_SSE4
			originScale_.mVec128 = _mm_insert_ps(a_translation.get128(), _mm_set_ss(a_scale), 0x30);
#else
			originScale_ = a_translation;
			originScale_[3] = a_scale;
#endif
		}

		btQsTransform(const btTransform& a_transform, float a_scale = 1.0F) :
			basis_(a_transform.getRotation())
		{
#ifdef BT_ALLOW_SSE4
			originScale_.mVec128 = _mm_insert_ps(a_transform.getOrigin().get128(), _mm_set_ss(a_scale), 0x30);
#else
			originScale_ = a_transform.getOrigin();
			originScale_[3] = a_scale;
#endif
		}

		[[nodiscard]] bool isValid() const { return getScale() > 0.0F; }
		[[nodiscard]] btQuaternion getBasis() const { return basis_; }
		btQuaternion& getBasis() { return basis_; }
		void setBasis(const btQuaternion& a_quaternion) { basis_ = a_quaternion; }
		void setBasis(const btMatrix3x3& a_matrix) { a_matrix.getRotation(basis_); }
		[[nodiscard]] float getScale() const { return originScale_[3]; }
		float& getScale() { return originScale_[3]; }
		[[nodiscard]] float getScaleReg() const { return getScale(); }

		void setScale(float a_scale)
		{
			assert(a_scale > 0.0F);
			originScale_[3] = a_scale;
		}

		[[nodiscard]] btVector3 getOrigin() const { return originScale_; }

		void setOrigin(const btVector3& a_origin)
		{
#ifdef BT_ALLOW_SSE4
			originScale_.mVec128 = _mm_blend_ps(a_origin.get128(), originScale_.get128(), 0b1000);
#else
			const auto scale = getScale();
			originScale_ = a_origin;
			originScale_[3] = scale;
#endif
		}

		void setOrigin(float a_x, float a_y, float a_z)
		{
			originScale_[0] = a_x;
			originScale_[1] = a_y;
			originScale_[2] = a_z;
		}

		[[nodiscard]] btQsTransform operator*(const btQsTransform& a_rhs) const
		{
			return btQsTransform(
				basis_ * a_rhs.basis_,
				getOrigin() + quatRotate(basis_, a_rhs.getOrigin() * getScale()),
				getScale() * a_rhs.getScale());
		}

		[[nodiscard]] btVector3 operator*(const btVector3& a_rhs) const
		{
			return getOrigin() + quatRotate(basis_, a_rhs * getScale());
		}

		void operator*=(const btQsTransform& a_rhs)
		{
			const auto scale = getScale();
			const auto newScale = scale * a_rhs.getScale();
			const auto newOrigin = getOrigin() + quatRotate(basis_, a_rhs.getOrigin() * scale);
			basis_ *= a_rhs.basis_;

#ifdef BT_ALLOW_SSE4
			originScale_.mVec128 = _mm_insert_ps(newOrigin.get128(), _mm_set_ss(newScale), 0x30);
#else
			originScale_ = newOrigin;
			originScale_[3] = newScale;
#endif
		}

		[[nodiscard]] btQsTransform inverse() const
		{
			const auto rotation = basis_.inverse();
			const auto scale = 1.0F / getScale();
			return btQsTransform(rotation, quatRotate(rotation, -getOrigin() * scale), scale);
		}

		[[nodiscard]] btTransform asTransform() const
		{
			return btTransform(basis_, originScale_);
		}

		[[nodiscard]] static btQsTransform getIdentity()
		{
			return btQsTransform();
		}

	private:
		btQuaternion basis_;
		btVector4 originScale_;
	};

	ATTRIBUTE_ALIGNED16(class)
	btMatrix4x3
	{
	public:
		btMatrix4x3() = default;

		btMatrix4x3(const btQsTransform& a_transform)
		{
			reinterpret_cast<btMatrix3x3*>(this)->setRotation(a_transform.getBasis());
			const auto scale = pshufd<0xFF>(a_transform.getOrigin().get128());
			row_[0] = _mm_mul_ps(row_[0], scale);
			row_[1] = _mm_mul_ps(row_[1], scale);
			row_[2] = _mm_mul_ps(row_[2], scale);
			row_[0].m128_f32[3] = a_transform.getOrigin()[0];
			row_[1].m128_f32[3] = a_transform.getOrigin()[1];
			row_[2].m128_f32[3] = a_transform.getOrigin()[2];
		}

		btVector3 operator*(const btVector3& a_rhs) const
		{
#ifdef BT_ALLOW_SSE4
			const auto vector = _mm_blend_ps(a_rhs.get128(), _mm_set_ps1(1.0F), 0x8);
			auto x = _mm_dp_ps(row_[0], vector, 0xF1);
			const auto y = _mm_dp_ps(row_[1], vector, 0xF2);
			const auto z = _mm_dp_ps(row_[2], vector, 0xF4);
			x = _mm_or_ps(x, y);
			x = _mm_or_ps(x, z);
#else
			auto vector = a_rhs.get128();
			vector.m128_f32[3] = 1.0F;
			auto x = _mm_mul_ps(row_[0], vector);
			const auto y = _mm_mul_ps(row_[1], vector);
			auto z = _mm_mul_ps(row_[2], vector);
			x = _mm_hadd_ps(x, y);
			z = _mm_hadd_ps(z, z);
			x = _mm_hadd_ps(x, z);
#endif
			return vectorFromM128(x);
		}

		__m128 mulPack(const btVector3& a_rhs, float a_packW) const
		{
#ifdef BT_ALLOW_SSE4
			const auto vector = _mm_blend_ps(a_rhs.get128(), _mm_set_ps1(1.0F), 0x8);
			auto x = _mm_dp_ps(row_[0], vector, 0xF1);
			const auto y = _mm_dp_ps(row_[1], vector, 0xF2);
			x = _mm_or_ps(x, y);
			auto z = _mm_dp_ps(row_[2], vector, 0xF1);
			z = _mm_unpacklo_ps(z, _mm_set_ss(a_packW));
			x = _mm_movelh_ps(x, z);
#else
			auto vector = a_rhs.get128();
			vector.m128_f32[3] = 1.0F;
			const auto w = _mm_load_ss(&a_packW);
			auto x = _mm_mul_ps(row_[0], vector);
			const auto y = _mm_mul_ps(row_[1], vector);
			auto z = _mm_mul_ps(row_[2], vector);
			x = _mm_hadd_ps(x, y);
			z = _mm_hadd_ps(z, w);
			x = _mm_hadd_ps(x, z);
#endif
			return x;
		}

		__m128 row_[3];
	};

	ATTRIBUTE_ALIGNED16(class)
	btMatrix4x3T
	{
	public:
		btMatrix4x3T() = default;

		btMatrix4x3T(const btQsTransform& a_transform)
		{
			btMatrix3x3 rotation;
			rotation.setRotation(a_transform.getBasis());
			rotation = rotation.transpose();
			const auto scale = pshufd<0xFF>(a_transform.getOrigin().get128());
			col_[0] = vectorFromM128(_mm_mul_ps(rotation[0].get128(), scale));
			col_[1] = vectorFromM128(_mm_mul_ps(rotation[1].get128(), scale));
			col_[2] = vectorFromM128(_mm_mul_ps(rotation[2].get128(), scale));
			col_[3] = a_transform.getOrigin();
		}

		btVector3 operator*(const btVector3& a_rhs) const
		{
			return col_[0] * a_rhs[0] + col_[1] * a_rhs[1] + col_[2] * a_rhs[2] + col_[3];
		}

		btMatrix4x3T operator*(const btMatrix4x3T& a_rhs) const
		{
			btMatrix4x3T result;
			result.col_[0] = col_[0] * a_rhs.col_[0][0] + col_[1] * a_rhs.col_[0][1] + col_[2] * a_rhs.col_[0][2];
			result.col_[1] = col_[0] * a_rhs.col_[1][0] + col_[1] * a_rhs.col_[1][1] + col_[2] * a_rhs.col_[1][2];
			result.col_[2] = col_[0] * a_rhs.col_[2][0] + col_[1] * a_rhs.col_[2][1] + col_[2] * a_rhs.col_[2][2];
			result.col_[3] = *this * a_rhs.col_[3];
			return result;
		}

		btMatrix3x3 basis() const { return reinterpret_cast<const btMatrix3x3*>(this)->transpose(); }
		btTransform toTransform() const { return btTransform(reinterpret_cast<const btMatrix3x3*>(this)->transpose(), col_[3]); }

		btVector3 col_[4];
	};

	class RefObject
	{
	public:
		RefObject() = default;
		virtual ~RefObject() = default;

		std::uint32_t IncRef() const { return ++refCount_; }

		std::uint32_t DecRef() const
		{
			assert(refCount_ > 0);
			return --refCount_;
		}

		void release() const
		{
			if (DecRef() == 0) {
				delete this;
			}
		}

		long getRefCount() const { return refCount_; }

	private:
		mutable std::atomic<std::uint32_t> refCount_{ 0 };
	};

	template <>
	inline btVector3 abs(btVector3 a_rhs)
	{
		return vectorFromM128(_mm_andnot_ps(_mm_set_ps1(-0.0F), a_rhs.get128()));
	}

	template <class T>
	using vectorA16 = std::vector<T>;

	class SpinLock
	{
	public:
		void lock() noexcept
		{
			for (;;) {
				if (!flag_.exchange(true, std::memory_order_acquire)) {
					return;
				}

				while (flag_.load(std::memory_order_relaxed)) {
					_mm_pause();
				}
			}
		}

		void unlock() noexcept
		{
			flag_.store(false, std::memory_order_release);
		}

		bool try_lock() noexcept
		{
			return !flag_.exchange(true, std::memory_order_acquire);
		}

	private:
		std::atomic<bool> flag_{ false };
	};
}
