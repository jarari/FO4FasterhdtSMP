#pragma once

#include "hdtAABB.h"

#include <functional>
#include <utility>

namespace hdt
{
	static constexpr int MaxCollisionPairs = 6024;

	struct alignas(16) Collider
	{
		Collider() = default;
		explicit Collider(int a_index) { vertex_ = a_index; }

		Collider(int a_index0, int a_index1, int a_index2)
		{
			vertices_[0] = a_index0;
			vertices_[1] = a_index1;
			vertices_[2] = a_index2;
		}

		Collider(const Collider& a_rhs) { operator=(a_rhs); }

		Collider& operator=(const Collider& a_rhs)
		{
			auto* destination = reinterpret_cast<__m128i*>(this);
			const auto* source = reinterpret_cast<const __m128i*>(&a_rhs);
			const auto value = _mm_load_si128(source);
			_mm_store_si128(destination, value);
			return *this;
		}

		union
		{
			U32 vertex_;
			U32 vertices_[3];
		};

		float flexible_;
	};

	struct alignas(16) ColliderTree
	{
		ColliderTree()
		{
			aabbAll_.invalidate();
			aabbMe_.invalidate();
		}

		explicit ColliderTree(U32 a_key) :
			key_(a_key)
		{
			aabbAll_.invalidate();
			aabbMe_.invalidate();
		}

		bool empty() const { return children_.empty() && colliders_.empty(); }

		void insertCollider(const U32* a_keys, std::size_t a_keyCount, const Collider& a_collider);
		void exportColliders(vectorA16<Collider>& a_exportTo);
		void remapColliders(Collider* a_start, Aabb* a_startAabb);
		void checkCollisionL(ColliderTree* a_rhs, std::vector<std::pair<ColliderTree*, ColliderTree*>>& a_result);
		void checkCollisionR(ColliderTree* a_rhs, std::vector<std::pair<ColliderTree*, ColliderTree*>>& a_result);
		void clipCollider(const std::function<bool(const Collider&)>& a_func);
		void updateKinematic(const std::function<float(const Collider*)>& a_func);
		void visitColliders(const std::function<void(Collider*)>& a_func);
		void updateAabb();
		void optimize();
		bool collapseCollideL(ColliderTree* a_rhs);
		bool collapseCollideR(ColliderTree* a_rhs);

		Aabb aabbAll_;
		Aabb aabbMe_;
		U32 isKinematic_{ true };
		Collider* colliderBuffer_{ nullptr };
		Aabb* aabb_{ nullptr };
		U32 numCollider_{ 0 };
		U32 dynCollider_{ 0 };
		U32 dynChild_{ 0 };
		vectorA16<ColliderTree> children_;
		vectorA16<Collider> colliders_;
		U32 key_{ 0 };
	};
}
