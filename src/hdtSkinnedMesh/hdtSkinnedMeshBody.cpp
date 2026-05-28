#include "hdtSkinnedMeshBody.h"

#include "hdtSkinnedMeshShape.h"

#include <algorithm>

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
		__m128 CalculateVertexState(const btVector3& a_skinPosition, const Bone& a_bone, const float a_weight)
		{
			auto position = a_bone.vertexToWorld_ * a_skinPosition;
			__m128 packed = position.get128();
			packed = _mm_blend_ps(packed, _mm_load_ps(a_bone.reserved_), 0x8);
			return _mm_mul_ps(_mm_set1_ps(a_weight), packed);
		}
	}

	void SkinnedMeshBody::internalUpdate()
	{
		if (disabled_) {
			return;
		}

		bones_.resize(skinnedBones_.size());
		for (std::size_t index = 0; index < skinnedBones_.size(); ++index) {
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
		for (std::size_t index = 0; index < vertices_.size(); ++index) {
			const auto& vertex = vertices_[index];
			__m128 blended = _mm_setzero_ps();
			for (int weightIndex = 0; weightIndex < 4; ++weightIndex) {
				const auto boneIndex = vertex.getBoneIdx(weightIndex);
				if (vertex.weight_[weightIndex] <= FLT_EPSILON || boneIndex >= bones_.size()) {
					continue;
				}
				blended = _mm_add_ps(blended, CalculateVertexState(vertex.skinPos_, bones_[boneIndex], vertex.weight_[weightIndex]));
			}
			alignas(16) float values[4];
			_mm_store_ps(values, blended);
			vertexPositions_[index].set(btVector4(values[0], values[1], values[2], values[3]));
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
