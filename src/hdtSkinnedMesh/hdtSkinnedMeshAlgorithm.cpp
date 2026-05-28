#include "hdtSkinnedMeshAlgorithm.h"

#include "hdtSkinnedMeshBody.h"

#include <algorithm>
#include <atomic>
#include <memory>

namespace
{
	__m128 CrossProduct(const __m128 a_lhs, const __m128 a_rhs)
	{
		const auto tmp0 = _mm_shuffle_ps(a_lhs, a_lhs, _MM_SHUFFLE(3, 0, 2, 1));
		const auto tmp1 = _mm_shuffle_ps(a_rhs, a_rhs, _MM_SHUFFLE(3, 1, 0, 2));
		const auto tmp2 = _mm_mul_ps(tmp0, a_rhs);
		const auto tmp3 = _mm_mul_ps(tmp0, tmp1);
		const auto tmp4 = _mm_shuffle_ps(tmp2, tmp2, _MM_SHUFFLE(3, 0, 2, 1));
		return _mm_sub_ps(tmp3, tmp4);
	}
}

namespace hdt
{
	template <class T>
	struct CollisionCheckBase
	{
		using ShapeProp0 = PerVertexShape::ShapeProp;
		using ShapeProp1 = typename T::ShapeProp;

		CollisionCheckBase(PerVertexShape* a_lhs, T* a_rhs, CollisionResult* a_results) :
			vertices0(a_lhs->owner_->vertexPositions_.data()),
			vertices1(a_rhs->owner_->vertexPositions_.data()),
			tree0(std::addressof(a_lhs->tree_)),
			tree1(std::addressof(a_rhs->tree_)),
			shapeProp0(std::addressof(a_lhs->shapeProp_)),
			shapeProp1(std::addressof(a_rhs->shapeProp_)),
			results(a_results)
		{}

		VertexPos* vertices0{ nullptr };
		VertexPos* vertices1{ nullptr };
		ColliderTree* tree0{ nullptr };
		ColliderTree* tree1{ nullptr };
		ShapeProp0* shapeProp0{ nullptr };
		ShapeProp1* shapeProp1{ nullptr };
		std::atomic_long numResults{ 0 };
		CollisionResult* results{ nullptr };
	};

	template <class T, bool SwapResults>
	struct CollisionResultWriter :
		public CollisionCheckBase<T>
	{
		template <class... Args>
		explicit CollisionResultWriter(Args&&... a_args) :
			CollisionCheckBase<T>(std::forward<Args>(a_args)...)
		{}

		bool addResult(const CollisionResult& a_result)
		{
			const auto index = this->numResults.fetch_add(1);
			if (index >= SkinnedMeshAlgorithm::MaxCollisionCount) {
				return false;
			}

			if constexpr (SwapResults) {
				this->results[index].posA = a_result.posB;
				this->results[index].posB = a_result.posA;
				this->results[index].colliderA = a_result.colliderB;
				this->results[index].colliderB = a_result.colliderA;
				this->results[index].normOnB = -a_result.normOnB;
				this->results[index].depth = a_result.depth;
			} else {
				this->results[index] = a_result;
			}
			return true;
		}
	};

	template <class T, bool SwapResults>
	struct CollisionChecker;

	template <bool SwapResults>
	struct CollisionChecker<PerVertexShape, SwapResults> :
		public CollisionResultWriter<PerVertexShape, SwapResults>
	{
		template <class... Args>
		explicit CollisionChecker(Args&&... a_args) :
			CollisionResultWriter<PerVertexShape, SwapResults>(std::forward<Args>(a_args)...)
		{}

		bool checkCollide(Collider* a_lhs, Collider* a_rhs, CollisionResult& a_result)
		{
			const auto sphere0 = this->vertices0[a_lhs->vertex_];
			const auto radius0 = sphere0.marginMultiplier() * this->shapeProp0->margin;
			const auto sphere1 = this->vertices1[a_rhs->vertex_];
			const auto radius1 = sphere1.marginMultiplier() * this->shapeProp1->margin;
			const auto result = checkSphereSphere(sphere0.pos(), sphere1.pos(), radius0, radius1, a_result);
			a_result.colliderA = a_lhs;
			a_result.colliderB = a_rhs;
			return result;
		}
	};

	template <bool SwapResults>
	struct CollisionChecker<PerTriangleShape, SwapResults> :
		public CollisionResultWriter<PerTriangleShape, SwapResults>
	{
		template <class... Args>
		explicit CollisionChecker(Args&&... a_args) :
			CollisionResultWriter<PerTriangleShape, SwapResults>(std::forward<Args>(a_args)...)
		{}

		bool checkCollide(Collider* a_lhs, Collider* a_rhs, CollisionResult& a_result)
		{
			const auto sphere = this->vertices0[a_lhs->vertex_];
			const auto radius = sphere.marginMultiplier() * this->shapeProp0->margin;
			const auto point0 = this->vertices1[a_rhs->vertices_[0]];
			const auto point1 = this->vertices1[a_rhs->vertices_[1]];
			const auto point2 = this->vertices1[a_rhs->vertices_[2]];
			auto margin = (point0.marginMultiplier() + point1.marginMultiplier() + point2.marginMultiplier()) / 3.0F;
			auto penetration = this->shapeProp1->penetration * margin;
			margin *= this->shapeProp1->margin;
			if (penetration > -FLT_EPSILON && penetration < FLT_EPSILON) {
				penetration = 0.0F;
			}

			auto edge0 = (point1.pos() - point0.pos()).get128();
			auto edge1 = (point2.pos() - point0.pos()).get128();
			auto rawNormal = CrossProduct(edge0, edge1);
			auto length = _mm_sqrt_ps(_mm_dp_ps(rawNormal, rawNormal, 0x77));
			if (_mm_cvtss_f32(length) < FLT_EPSILON) {
				return false;
			}

			auto normal = _mm_div_ps(rawNormal, length);
			if (penetration < 0.0F) {
				normal = _mm_sub_ps(_mm_setzero_ps(), normal);
				penetration = -penetration;
			}

			auto pointToSphere = (sphere.pos() - point0.pos()).get128();
			const auto distance = _mm_dp_ps(pointToSphere, normal, 0x77);
			auto distanceFromPlane = _mm_cvtss_f32(distance);
			const auto projection = _mm_sub_ps(sphere.pos().get128(), _mm_mul_ps(normal, distance));
			const auto radiusWithMargin = radius + margin;
			const auto insideContactPlane = [&]() {
				if (penetration >= FLT_EPSILON) {
					return distanceFromPlane < radiusWithMargin && distanceFromPlane >= -penetration;
				}
				if (distanceFromPlane < 0.0F) {
					distanceFromPlane = -distanceFromPlane;
					normal = _mm_sub_ps(_mm_setzero_ps(), normal);
				}
				return distanceFromPlane < radiusWithMargin;
			}();
			if (!insideContactPlane) {
				return false;
			}

			pointToSphere = _mm_sub_ps(projection, point0.pos().get128());
			auto bp = _mm_sub_ps(projection, point1.pos().get128());
			auto cp = _mm_sub_ps(projection, point2.pos().get128());
			auto area0 = CrossProduct(bp, cp);
			edge0 = CrossProduct(cp, pointToSphere);
			edge1 = CrossProduct(pointToSphere, bp);
			area0 = _mm_dp_ps(area0, area0, 0x74);
			edge0 = _mm_dp_ps(edge0, edge0, 0x72);
			edge1 = _mm_dp_ps(edge1, edge1, 0x71);
			area0 = _mm_or_ps(_mm_or_ps(area0, edge0), edge1);
			area0 = _mm_sqrt_ps(area0);
			area0 = _mm_add_ps(area0, _mm_shuffle_ps(area0, area0, _MM_SHUFFLE(3, 0, 2, 1)));
			area0 = _mm_cmpgt_ps(area0, length);
			const auto pointInTriangle = _mm_test_all_zeros(_mm_set_epi32(0, -1, -1, -1), _mm_castps_si128(area0));

			a_result.colliderA = a_lhs;
			a_result.colliderB = a_rhs;
			if (!pointInTriangle) {
				return false;
			}

			a_result.normOnB.set128(normal);
			a_result.posA = sphere.pos() - a_result.normOnB * radius;
			a_result.posB.set128(projection);
			a_result.depth = distanceFromPlane - radiusWithMargin;
			return a_result.depth < -FLT_EPSILON;
		}
	};

	template <class T, bool SwapResults = false>
	struct CollisionCheckAlgorithm :
		public CollisionChecker<T, SwapResults>
	{
		template <class... Args>
		explicit CollisionCheckAlgorithm(Args&&... a_args) :
			CollisionChecker<T, SwapResults>(std::forward<Args>(a_args)...)
		{}

		void dispatch(ColliderTree* a_lhs, ColliderTree* a_rhs, std::vector<Aabb*>& a_listA, std::vector<Aabb*>& a_listB, const Aabb& a_refinedB)
		{
			CollisionResult result;
			CollisionResult temp;
			bool hasResult = false;
			auto* beginA = a_lhs->aabb_;
			auto* beginB = a_rhs->aabb_;

			for (auto* aabbA : a_listA) {
				if (!aabbA->collideWith(a_refinedB)) {
					continue;
				}
				for (auto* aabbB : a_listB) {
					if (!aabbA->collideWith(*aabbB)) {
						continue;
					}
					if (this->checkCollide(std::addressof(a_lhs->colliderBuffer_[aabbA - beginA]), std::addressof(a_rhs->colliderBuffer_[aabbB - beginB]), temp)) {
						if (!hasResult || result.depth > temp.depth) {
							hasResult = true;
							result = temp;
						}
					}
				}
			}

			if (hasResult) {
				this->addResult(result);
			}
		}

		int operator()()
		{
			std::vector<std::pair<ColliderTree*, ColliderTree*>> pairs;
			pairs.reserve(this->tree0->numCollider_ + this->tree1->numCollider_);
			this->tree0->checkCollisionL(this->tree1, pairs);
			if (pairs.empty() || pairs.size() > MaxCollisionPairs) {
				return 0;
			}

			for (auto& pair : pairs) {
				if (this->numResults.load() >= SkinnedMeshAlgorithm::MaxCollisionCount) {
					break;
				}

				auto* lhs = pair.first;
				auto* rhs = pair.second;
				auto* beginA = lhs->aabb_;
				auto* beginB = rhs->aabb_;
				const auto sizeA = rhs->isKinematic_ ? lhs->dynCollider_ : lhs->numCollider_;
				const auto sizeB = lhs->isKinematic_ ? rhs->dynCollider_ : rhs->numCollider_;
				auto* endA = beginA + sizeA;
				auto* endB = beginB + sizeB;

				Aabb aabbA;
				auto aabbB = rhs->aabbMe_;
				std::vector<Aabb*> listA;
				std::vector<Aabb*> listB;
				listA.reserve(sizeA);
				listB.reserve(sizeB);

				for (auto* candidate = beginA; candidate < endA; ++candidate) {
					if (candidate->collideWith(aabbB)) {
						listA.push_back(candidate);
						aabbA.merge(*candidate);
					}
				}

				if (!listA.empty()) {
					aabbB.invalidate();
					for (auto* candidate = beginB; candidate < endB; ++candidate) {
						if (candidate->collideWith(aabbA)) {
							listB.push_back(candidate);
							aabbB.merge(*candidate);
						}
					}
				}

				dispatch(lhs, rhs, listA, listB, aabbB);
			}

			return static_cast<int>(this->numResults.load());
		}
	};

	template <class T>
	int CheckCollide(PerVertexShape* a_lhs, T* a_rhs, CollisionResult* a_results)
	{
		return CollisionCheckAlgorithm<T>(a_lhs, a_rhs, a_results)();
	}

	int CheckCollide(PerTriangleShape* a_lhs, PerVertexShape* a_rhs, CollisionResult* a_results)
	{
		return CollisionCheckAlgorithm<PerTriangleShape, true>(a_rhs, a_lhs, a_results)();
	}

	SkinnedMeshAlgorithm::MergeBuffer::MergeBuffer()
	{
		activeCells.reserve(256);
	}

	SkinnedMeshAlgorithm::MergeBuffer::~MergeBuffer()
	{
		std::free(buffer);
		std::free(generations);
	}

	void SkinnedMeshAlgorithm::MergeBuffer::resize(const int a_x, const int a_y)
	{
		mergeStride = a_y;
		const auto needed = a_x * a_y;
		if (needed > mergeSize) {
			std::free(buffer);
			std::free(generations);
			mergeSize = needed;
			buffer = static_cast<CollisionMerge*>(std::malloc(needed * sizeof(CollisionMerge)));
			generations = static_cast<std::uint32_t*>(std::calloc(needed, sizeof(std::uint32_t)));
		}
		if (++currentGen == 0) {
			std::memset(generations, 0, mergeSize * sizeof(std::uint32_t));
			currentGen = 1;
		}
		activeCells.clear();
	}

	SkinnedMeshAlgorithm::CollisionMerge* SkinnedMeshAlgorithm::MergeBuffer::getAndTrack(const int a_x, const int a_y)
	{
		const auto index = a_x * mergeStride + a_y;
		auto* cell = std::addressof(buffer[index]);
		if (generations[index] != currentGen) {
			cell->reset();
			generations[index] = currentGen;
			activeCells.push_back(index);
		}
		return cell;
	}

	template <class T0, class T1>
	void SkinnedMeshAlgorithm::MergeBuffer::doMerge(T0* a_shape0, T1* a_shape1, CollisionResult* a_collisions, const int a_count)
	{
		for (int index = 0; index < a_count; ++index) {
			auto& result = a_collisions[index];
			if (result.depth >= -FLT_EPSILON) {
				break;
			}

			const auto flexible = std::max(result.colliderA->flexible_, result.colliderB->flexible_);
			if (flexible < FLT_EPSILON) {
				continue;
			}

			const auto weight = flexible * result.depth;
			const auto weightSquared = weight * weight;
			const auto normalScaled = result.normOnB * weight * weightSquared;
			const auto posAScaled = result.posA * weightSquared;
			const auto posBScaled = result.posB * weightSquared;

			for (int boneA = 0; boneA < a_shape0->getBonePerCollider(); ++boneA) {
				const auto weightA = a_shape0->getColliderBoneWeight(result.colliderA, boneA);
				const auto boneIndexA = a_shape0->getColliderBoneIndex(result.colliderA, boneA);
				if (boneIndexA < 0 || static_cast<std::size_t>(boneIndexA) >= a_shape0->owner_->skinnedBones_.size() ||
					weightA <= a_shape0->owner_->skinnedBones_[boneIndexA].weightThreshold) {
					continue;
				}

				for (int boneB = 0; boneB < a_shape1->getBonePerCollider(); ++boneB) {
					const auto weightB = a_shape1->getColliderBoneWeight(result.colliderB, boneB);
					const auto boneIndexB = a_shape1->getColliderBoneIndex(result.colliderB, boneB);
					if (boneIndexB < 0 || static_cast<std::size_t>(boneIndexB) >= a_shape1->owner_->skinnedBones_.size() ||
						weightB <= a_shape1->owner_->skinnedBones_[boneIndexB].weightThreshold) {
						continue;
					}

					if (a_shape0->owner_->skinnedBones_[boneIndexA].isKinematic && a_shape1->owner_->skinnedBones_[boneIndexB].isKinematic) {
						continue;
					}

					auto* cell = getAndTrack(boneIndexA, boneIndexB);
					if (cell->weight > FLT_EPSILON && cell->normal.dot(normalScaled) < 0.0F) {
						continue;
					}

					cell->weight += weightSquared;
					cell->normal += normalScaled;
					cell->pos[0] += posAScaled;
					cell->pos[1] += posBScaled;
				}
			}
		}
	}

	void SkinnedMeshAlgorithm::MergeBuffer::apply(SkinnedMeshBody* a_body0, SkinnedMeshBody* a_body1, CollisionDispatcher* a_dispatcher)
	{
		for (const auto flatIndex : activeCells) {
			const auto indexA = flatIndex / mergeStride;
			const auto indexB = flatIndex % mergeStride;
			auto* cell = std::addressof(buffer[flatIndex]);
			if (cell->weight < FLT_EPSILON) {
				continue;
			}

			auto* bone0 = a_body0->skinnedBones_[indexA].ptr;
			auto* bone1 = a_body1->skinnedBones_[indexB].ptr;
			if (!bone0 || !bone1 || bone0 == bone1 ||
				!a_body1->canCollideWith(bone0) ||
				!a_body0->canCollideWith(bone1) ||
				(a_body0->skinnedBones_[indexA].isKinematic && a_body1->skinnedBones_[indexB].isKinematic)) {
				continue;
			}

			const auto invWeight = 1.0F / cell->weight;
			const auto worldA = cell->pos[0] * invWeight;
			const auto worldB = cell->pos[1] * invWeight;
			const auto localA = bone0->m_rig.getWorldTransform().invXform(worldA);
			const auto localB = bone1->m_rig.getWorldTransform().invXform(worldB);
			auto normal = cell->normal * invWeight;
			if (normal.fuzzyZero()) {
				continue;
			}

			const auto depth = -normal.length();
			normal = -normal.normalized();
			if (depth >= -FLT_EPSILON) {
				continue;
			}

			btManifoldPoint point(localA, localB, normal, depth);
			point.m_positionWorldOnA = worldA;
			point.m_positionWorldOnB = worldB;
			point.m_combinedFriction = bone0->m_rig.getFriction() * bone1->m_rig.getFriction();
			point.m_combinedRestitution = bone0->m_rig.getRestitution() * bone1->m_rig.getRestitution();
			point.m_combinedRollingFriction = bone0->m_rig.getRollingFriction() * bone1->m_rig.getRollingFriction();

			auto* manifold = a_dispatcher->getNewManifold(std::addressof(bone0->m_rig), std::addressof(bone1->m_rig));
			manifold->addManifoldPoint(point);
		}
	}

	template <class T0, class T1>
	void SkinnedMeshAlgorithm::processCollision(T0* a_shape0, T1* a_shape1, MergeBuffer& a_merge, CollisionResult* a_collision)
	{
		const auto count = std::min(CheckCollide(a_shape0, a_shape1, a_collision), MaxCollisionCount);
		if (count <= 0) {
			return;
		}

		std::sort(a_collision, a_collision + count, [](const CollisionResult& a_lhs, const CollisionResult& a_rhs) {
			return a_lhs.depth < a_rhs.depth;
		});
		a_merge.doMerge(a_shape0, a_shape1, a_collision, count);
	}

	void SkinnedMeshAlgorithm::processCollision(SkinnedMeshBody* a_body0, SkinnedMeshBody* a_body1, CollisionDispatcher* a_dispatcher)
	{
		if (!a_body0 || !a_body1 || !a_body0->shape_ || !a_body1->shape_) {
			return;
		}

		thread_local MergeBuffer merge;
		thread_local auto collision = std::make_unique<CollisionResult[]>(MaxCollisionCount);
		merge.resize(static_cast<int>(a_body0->skinnedBones_.size()), static_cast<int>(a_body1->skinnedBones_.size()));

		if (a_body0->shape_->asPerTriangleShape() && a_body1->shape_->asPerTriangleShape()) {
			processCollision(a_body0->shape_->asPerTriangleShape(), a_body1->shape_->asPerVertexShape(), merge, collision.get());
			processCollision(a_body0->shape_->asPerVertexShape(), a_body1->shape_->asPerTriangleShape(), merge, collision.get());
		} else if (a_body0->shape_->asPerTriangleShape()) {
			processCollision(a_body0->shape_->asPerTriangleShape(), a_body1->shape_->asPerVertexShape(), merge, collision.get());
		} else if (a_body1->shape_->asPerTriangleShape()) {
			processCollision(a_body0->shape_->asPerVertexShape(), a_body1->shape_->asPerTriangleShape(), merge, collision.get());
		} else {
			processCollision(a_body0->shape_->asPerVertexShape(), a_body1->shape_->asPerVertexShape(), merge, collision.get());
		}

		merge.apply(a_body0, a_body1, a_dispatcher);
	}
}
