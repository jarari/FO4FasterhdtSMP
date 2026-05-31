#include "hdtSkinnedMeshBody.h"

#include "hdtSkinnedMeshShape.h"

#include <algorithm>

#if defined(__AVX2__) || defined(__AVX512F__) || defined(FO4_FASTER_HDTSMP_AVX2) || defined(FO4_FASTER_HDTSMP_AVX512)
#define HDT_USE_FMA_VERTEX_UPDATE 1
#endif

namespace hdt
{
	SkinnedMeshBody::CollisionShape::CollisionShape() :
		aabb(_mm_setzero_ps(), _mm_setzero_ps())
	{
		m_shapeType = CUSTOM_CONCAVE_SHAPE_TYPE;
	}

	void SkinnedMeshBody::CollisionShape::getAabb([[maybe_unused]] const btTransform& a_transform, btVector3& aabbMin, btVector3& aabbMax) const
	{
		aabbMin = vectorFromM128(aabb.min_);
		aabbMax = vectorFromM128(aabb.max_);
	}

	const btVector3& SkinnedMeshBody::CollisionShape::getLocalScaling() const
	{
		static const btVector3 localScaling(1.0F, 1.0F, 1.0F);
		return localScaling;
	}

	SkinnedMeshBody::SkinnedMeshBody()
	{
		m_collisionShape = std::addressof(bulletShape_);
	}

	namespace
	{
		__m128 CalculateVertexState(__m128 a_skinPosition, const Bone& a_bone, __m128 a_weight)
		{
			const auto position = a_bone.vertexToWorld_ * vectorFromM128(a_skinPosition);
			__m128 packed = position.get128();
			packed = _mm_blend_ps(packed, _mm_load_ps(a_bone.reserved_), 0x8);
			return _mm_mul_ps(a_weight, packed);
		}

#ifdef HDT_USE_FMA_VERTEX_UPDATE
		__m128 CalculateVertexStateFMA(__m128 a_skinPosition, const Bone& a_bone, __m128 a_weight)
		{
			const __m128 px = setAll0(a_skinPosition);
			const __m128 py = setAll1(a_skinPosition);
			const __m128 pz = setAll2(a_skinPosition);
			__m128 result = _mm_fmadd_ps(a_bone.vertexToWorld_.col_[2].get128(), pz, a_bone.vertexToWorld_.col_[3].get128());
			result = _mm_fmadd_ps(a_bone.vertexToWorld_.col_[1].get128(), py, result);
			result = _mm_fmadd_ps(a_bone.vertexToWorld_.col_[0].get128(), px, result);
			result = _mm_blend_ps(result, _mm_load_ps(a_bone.reserved_), 0x8);
			return _mm_mul_ps(a_weight, result);
		}
#endif
	}

	void SkinnedMeshBody::internalUpdate()
	{
		if (disabled_) {
			return;
		}

		bones_.resize(skinnedBones_.size());
		for (std::size_t index = 0; index < skinnedBones_.size(); ++index) {
			if (index + 8 < skinnedBones_.size()) {
				_mm_prefetch(reinterpret_cast<const char*>(std::addressof(skinnedBones_[index + 8])), _MM_HINT_T1);
			}
			auto& source = skinnedBones_[index];
			auto& destination = bones_[index];
			if (!source.ptr) {
				continue;
			}

			const auto boneTransform = source.ptr->m_currentTransform;
			destination.vertexToWorld_ = btMatrix4x3T(boneTransform) * source.vertexToBone;
			destination.marginMultiplier_ = source.ptr->m_marginMultipler * boneTransform.getScale();
			destination.reserved_[0] = 0.0F;
			destination.reserved_[1] = 0.0F;
			destination.reserved_[2] = 0.0F;
		}

		vertexPositions_.resize(vertices_.size());
		const auto vertexCount = static_cast<int>(vertices_.size());
		const auto* const __restrict vertices = vertices_.data();
		auto* const __restrict vertexPositions = vertexPositions_.data();
		const auto* const __restrict bones = bones_.data();

		if (!bones_.empty()) {
#ifdef HDT_USE_FMA_VERTEX_UPDATE
			constexpr int prefetchDistance = 6;
			int index = 0;
			for (; index + 1 < vertexCount; index += 2) {
				if (index + prefetchDistance + 1 < vertexCount) {
					_mm_prefetch(reinterpret_cast<const char*>(std::addressof(vertices[index + prefetchDistance])), _MM_HINT_T0);
					_mm_prefetch(reinterpret_cast<const char*>(std::addressof(vertices[index + prefetchDistance + 1])), _MM_HINT_T0);
				}

				const auto& vertex0 = vertices[index];
				const auto skinPosition0 = vertex0.skinPos_.get128();
				const auto weights0 = _mm_load_ps(vertex0.weight_);
				auto blended0 = CalculateVertexStateFMA(skinPosition0, bones[vertex0.getBoneIdx(0)], setAll0(weights0));
				blended0 += CalculateVertexStateFMA(skinPosition0, bones[vertex0.getBoneIdx(1)], setAll1(weights0));
				blended0 += CalculateVertexStateFMA(skinPosition0, bones[vertex0.getBoneIdx(2)], setAll2(weights0));
				blended0 += CalculateVertexStateFMA(skinPosition0, bones[vertex0.getBoneIdx(3)], setAll3(weights0));
				vertexPositions[index].set(blended0);

				const auto& vertex1 = vertices[index + 1];
				const auto skinPosition1 = vertex1.skinPos_.get128();
				const auto weights1 = _mm_load_ps(vertex1.weight_);
				auto blended1 = CalculateVertexStateFMA(skinPosition1, bones[vertex1.getBoneIdx(0)], setAll0(weights1));
				blended1 += CalculateVertexStateFMA(skinPosition1, bones[vertex1.getBoneIdx(1)], setAll1(weights1));
				blended1 += CalculateVertexStateFMA(skinPosition1, bones[vertex1.getBoneIdx(2)], setAll2(weights1));
				blended1 += CalculateVertexStateFMA(skinPosition1, bones[vertex1.getBoneIdx(3)], setAll3(weights1));
				vertexPositions[index + 1].set(blended1);
			}
			for (; index < vertexCount; ++index) {
				const auto& vertex = vertices[index];
				const auto skinPosition = vertex.skinPos_.get128();
				const auto weights = _mm_load_ps(vertex.weight_);
				auto blended = CalculateVertexStateFMA(skinPosition, bones[vertex.getBoneIdx(0)], setAll0(weights));
				blended += CalculateVertexStateFMA(skinPosition, bones[vertex.getBoneIdx(1)], setAll1(weights));
				blended += CalculateVertexStateFMA(skinPosition, bones[vertex.getBoneIdx(2)], setAll2(weights));
				blended += CalculateVertexStateFMA(skinPosition, bones[vertex.getBoneIdx(3)], setAll3(weights));
				vertexPositions[index].set(blended);
			}
#else
			constexpr int prefetchDistance = 8;
			int index = 0;
			for (; index + 3 < vertexCount; index += 4) {
				if (index + prefetchDistance + 3 < vertexCount) {
					_mm_prefetch(reinterpret_cast<const char*>(std::addressof(vertices[index + prefetchDistance])), _MM_HINT_T0);
					_mm_prefetch(reinterpret_cast<const char*>(std::addressof(vertices[index + prefetchDistance + 1])), _MM_HINT_T0);
					_mm_prefetch(reinterpret_cast<const char*>(std::addressof(vertices[index + prefetchDistance + 2])), _MM_HINT_T0);
					_mm_prefetch(reinterpret_cast<const char*>(std::addressof(vertices[index + prefetchDistance + 3])), _MM_HINT_T0);
				}

				for (int lane = 0; lane < 4; ++lane) {
					const auto& vertex = vertices[index + lane];
					const auto skinPosition = vertex.skinPos_.get128();
					const auto weights = _mm_load_ps(vertex.weight_);
					auto blended = CalculateVertexState(skinPosition, bones[vertex.getBoneIdx(0)], setAll0(weights));
					blended += CalculateVertexState(skinPosition, bones[vertex.getBoneIdx(1)], setAll1(weights));
					blended += CalculateVertexState(skinPosition, bones[vertex.getBoneIdx(2)], setAll2(weights));
					blended += CalculateVertexState(skinPosition, bones[vertex.getBoneIdx(3)], setAll3(weights));
					vertexPositions[index + lane].set(blended);
				}
			}
			for (; index < vertexCount; ++index) {
				const auto& vertex = vertices[index];
				const auto skinPosition = vertex.skinPos_.get128();
				const auto weights = _mm_load_ps(vertex.weight_);
				auto blended = CalculateVertexState(skinPosition, bones[vertex.getBoneIdx(0)], setAll0(weights));
				blended += CalculateVertexState(skinPosition, bones[vertex.getBoneIdx(1)], setAll1(weights));
				blended += CalculateVertexState(skinPosition, bones[vertex.getBoneIdx(2)], setAll2(weights));
				blended += CalculateVertexState(skinPosition, bones[vertex.getBoneIdx(3)], setAll3(weights));
				vertexPositions[index].set(blended);
			}
#endif
		}

		if (shape_) {
			shape_->internalUpdate();
			bulletShape_.aabb = shape_->tree_.aabbAll_;
		}
	}

	float SkinnedMeshBody::flexible(const Vertex& a_vertex)
	{
		float result = 0.0F;
		for (int index = 0; index < 4; ++index) {
			if (a_vertex.weight_[index] < FLT_EPSILON) {
				break;
			}
			const auto boneIndex = a_vertex.getBoneIdx(index);
			if (boneIndex < skinnedBones_.size() && !skinnedBones_[boneIndex].isKinematic) {
				result += a_vertex.weight_[index];
			}
		}
		return result;
	}

	int SkinnedMeshBody::addBone(SkinnedMeshBone* a_bone, const btQsTransform& a_verticesToBone, const BoundingSphere& a_boundingSphere)
	{
		auto& skinnedBone = skinnedBones_.emplace_back();
		skinnedBone.ptr = a_bone;
		skinnedBone.vertexToBone = btMatrix4x3T(a_verticesToBone);
		skinnedBone.localBoundingSphere = a_boundingSphere;
		skinnedBone.isKinematic = !a_bone || a_bone->m_rig.isStaticOrKinematicObject();
		return static_cast<int>(skinnedBones_.size() - 1);
	}

	void SkinnedMeshBody::finishBuild()
	{
		bones_.resize(skinnedBones_.size());
		vertexPositions_.resize(vertices_.size());
		if (!shape_) {
			return;
		}

		shape_->clipColliders();
		isKinematic_ = true;
		for (auto& skinnedBone : skinnedBones_) {
			skinnedBone.isKinematic = !skinnedBone.ptr || skinnedBone.ptr->m_rig.isStaticOrKinematicObject();
			if (!skinnedBone.isKinematic) {
				isKinematic_ = false;
			}
		}

		shape_->finishBuild();
		std::vector<std::uint8_t> flags(vertices_.size(), 0);
		shape_->markUsedVertices(flags);

		std::uint32_t usedVertices = 0;
		std::vector<std::uint32_t> remap(vertices_.size(), 0);
		for (std::size_t index = 0; index < vertices_.size(); ++index) {
			if (!flags[index]) {
				continue;
			}
			vertices_[usedVertices] = vertices_[index];
			if (index < vertexPositions_.size()) {
				vertexPositions_[usedVertices] = vertexPositions_[index];
			}
			remap[index] = usedVertices++;
		}

		shape_->remapVertices(remap);
		vertices_.resize(usedVertices);
		vertexPositions_.resize(usedVertices);
		useBoundingSphere_ = shape_->colliders_.size() > 10;
	}

	bool SkinnedMeshBody::canCollideWith(const SkinnedMeshBone* a_bone) const
	{
		if (!canCollideWithBones_.empty()) {
			return std::ranges::find(canCollideWithBones_, a_bone) != canCollideWithBones_.end();
		}
		return std::ranges::find(noCollideWithBones_, a_bone) == noCollideWithBones_.end();
	}

	bool SkinnedMeshBody::canCollideWith(const SkinnedMeshBody* a_body) const
	{
		if (disabled_ || (a_body && a_body->disabled_)) {
			return false;
		}

		if (!a_body || (isKinematic_ && a_body->isKinematic_)) {
			return false;
		}

		switch (shared_) {
		case SharedType::kPublic:
			break;
		case SharedType::kInternal:
			if (actor_ != a_body->actor_) {
				return false;
			}
			break;
		case SharedType::kExternal:
			if (actor_ == a_body->actor_) {
				return false;
			}
			break;
		case SharedType::kPrivate:
			if (actor_ != a_body->actor_ || buildGroup_ == 0 || buildGroup_ != a_body->buildGroup_) {
				return false;
			}
			break;
		default:
			return false;
		}

		if (canCollideWithTags_.empty()) {
			for (const auto& tag : a_body->tags_) {
				if (std::ranges::find(noCollideWithTags_, tag) != noCollideWithTags_.end()) {
					return false;
				}
			}
			return true;
		}

		for (const auto& tag : a_body->tags_) {
			if (std::ranges::find(canCollideWithTags_, tag) != canCollideWithTags_.end()) {
				return true;
			}
		}
		return false;
	}

	void SkinnedMeshBody::updateBoundingSphereAabb()
	{
		if (disabled_) {
			return;
		}

		bulletShape_.aabb.invalidate();
		for (auto& skinnedBone : skinnedBones_) {
			if (!skinnedBone.ptr) {
				continue;
			}
			const auto sphere = skinnedBone.localBoundingSphere;
			const auto transform = skinnedBone.ptr->m_currentTransform;
			skinnedBone.worldBoundingSphere = BoundingSphere(transform * sphere.center(), transform.getScale() * sphere.radius());
			bulletShape_.aabb.merge(skinnedBone.worldBoundingSphere.getAabb());
		}

		if (!useBoundingSphere_) {
			internalUpdate();
		}
	}

	bool SkinnedMeshBody::isBoundingSphereCollided(SkinnedMeshBody* a_rhs)
	{
		return canCollideWith(a_rhs) && a_rhs && a_rhs->canCollideWith(this);
	}
}
