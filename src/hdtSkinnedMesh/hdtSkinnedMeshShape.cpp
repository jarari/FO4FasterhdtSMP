#include "hdtSkinnedMeshShape.h"

#include "hdtSkinnedMeshBody.h"

#include <algorithm>

namespace hdt
{
	SkinnedMeshShape::SkinnedMeshShape(SkinnedMeshBody* a_body, const bool a_attachToOwner) :
		owner_(a_body)
	{
		if (owner_ && a_attachToOwner) {
			owner_->shape_ = RE::BSTSmartPointer<SkinnedMeshShape>(this);
		}
	}

	void SkinnedMeshShape::clipColliders()
	{
		tree_.clipCollider([this](const Collider& a_collider) {
			bool active = false;
			for (int index = 0; index < getBonePerCollider() && !active; ++index) {
				const auto boneIndex = getColliderBoneIndex(std::addressof(a_collider), index);
				if (!owner_ || boneIndex < 0 || static_cast<std::size_t>(boneIndex) >= owner_->skinnedBones_.size()) {
					continue;
				}

				const auto weight = getColliderBoneWeight(std::addressof(a_collider), index);
				active = weight > FLT_EPSILON && weight > owner_->skinnedBones_[boneIndex].weightThreshold;
			}
			return !active;
		});
	}

	PerVertexShape::PerVertexShape(SkinnedMeshBody* a_body, const bool a_attachToOwner) :
		SkinnedMeshShape(a_body, a_attachToOwner)
	{}

	float PerVertexShape::getColliderBoneWeight(const Collider* a_collider, const int a_boneIndex)
	{
		return owner_->vertices_[a_collider->vertex_].weight_[a_boneIndex];
	}

	int PerVertexShape::getColliderBoneIndex(const Collider* a_collider, const int a_boneIndex)
	{
		return static_cast<int>(owner_->vertices_[a_collider->vertex_].getBoneIdx(a_boneIndex));
	}

	void PerVertexShape::finishBuild()
	{
		tree_.optimize();
		tree_.updateKinematic([this](const Collider* a_collider) {
			return owner_->flexible(owner_->vertices_[a_collider->vertex_]);
		});

		owner_->setCollisionFlags(tree_.isKinematic_ ? btCollisionObject::CF_KINEMATIC_OBJECT : 0);
		tree_.exportColliders(colliders_);
		aabbs_.resize(colliders_.size());
		tree_.remapColliders(colliders_.data(), aabbs_.data());
	}

	void PerVertexShape::internalUpdate()
	{
		const auto size = colliders_.size();
		const auto marginFactor = _mm_set1_ps(shapeProp_.margin);
		for (std::size_t index = 0; index < size; ++index) {
			const auto point = owner_->vertexPositions_[colliders_[index].vertex_].data_;
			const auto radius = _mm_mul_ps(_mm_shuffle_ps(point, point, _MM_SHUFFLE(3, 3, 3, 3)), marginFactor);
			aabbs_[index].min_ = _mm_sub_ps(point, radius);
			aabbs_[index].max_ = _mm_add_ps(point, radius);
		}

		tree_.updateAabb();
	}

	void PerVertexShape::autoGen()
	{
		tree_.children_.clear();
		std::vector<U32> keys;
		for (U32 vertexIndex = 0; vertexIndex < owner_->vertices_.size(); ++vertexIndex) {
			keys.clear();
			for (int weightIndex = 0; weightIndex < 4; ++weightIndex) {
				if (owner_->vertices_[vertexIndex].weight_[weightIndex] > FLT_EPSILON) {
					keys.push_back(owner_->vertices_[vertexIndex].getBoneIdx(weightIndex));
				}
			}
			tree_.insertCollider(keys.data(), keys.size(), Collider(vertexIndex));
		}
	}

	void PerVertexShape::markUsedVertices(std::vector<std::uint8_t>& a_flags)
	{
		for (const auto& collider : colliders_) {
			if (collider.vertex_ < a_flags.size()) {
				a_flags[collider.vertex_] = 1;
			}
		}
	}

	void PerVertexShape::remapVertices(const std::vector<std::uint32_t>& a_map)
	{
		for (auto& collider : colliders_) {
			if (collider.vertex_ < a_map.size()) {
				collider.vertex_ = a_map[collider.vertex_];
			}
		}
	}

	PerTriangleShape::PerTriangleShape(SkinnedMeshBody* a_body) :
		SkinnedMeshShape(a_body)
	{}

	float PerTriangleShape::getColliderBoneWeight(const Collider* a_collider, const int a_boneIndex)
	{
		return owner_->vertices_[a_collider->vertices_[a_boneIndex / 4]].weight_[a_boneIndex % 4];
	}

	int PerTriangleShape::getColliderBoneIndex(const Collider* a_collider, const int a_boneIndex)
	{
		return static_cast<int>(owner_->vertices_[a_collider->vertices_[a_boneIndex / 4]].getBoneIdx(a_boneIndex % 4));
	}

	btVector3 PerTriangleShape::baryCoord(const Collider* a_collider, const btVector3& a_point)
	{
		return BaryCoord(
			owner_->vertexPositions_[a_collider->vertices_[0]].pos(),
			owner_->vertexPositions_[a_collider->vertices_[1]].pos(),
			owner_->vertexPositions_[a_collider->vertices_[2]].pos(),
			a_point);
	}

	void PerTriangleShape::finishBuild()
	{
		tree_.optimize();
		tree_.updateKinematic([this](const Collider* a_collider) {
			auto flexible = owner_->flexible(owner_->vertices_[a_collider->vertices_[0]]);
			flexible += owner_->flexible(owner_->vertices_[a_collider->vertices_[1]]);
			flexible += owner_->flexible(owner_->vertices_[a_collider->vertices_[2]]);
			return flexible / 3.0F;
		});

		owner_->setCollisionFlags(tree_.isKinematic_ ? btCollisionObject::CF_KINEMATIC_OBJECT : 0);
		tree_.exportColliders(colliders_);
		aabbs_.resize(colliders_.size());
		tree_.remapColliders(colliders_.data(), aabbs_.data());

		verticesCollision_ = RE::make_smart<PerVertexShape>(owner_, false);
		verticesCollision_->shapeProp_.margin = shapeProp_.margin;
		owner_->shape_ = RE::BSTSmartPointer<SkinnedMeshShape>(this);
		verticesCollision_->autoGen();
		verticesCollision_->clipColliders();
		verticesCollision_->finishBuild();
	}

	void PerTriangleShape::internalUpdate()
	{
		for (std::size_t index = 0; index < colliders_.size(); ++index) {
			const auto& collider = colliders_[index];
			const auto point0 = owner_->vertexPositions_[collider.vertices_[0]].data_;
			const auto point1 = owner_->vertexPositions_[collider.vertices_[1]].data_;
			const auto point2 = owner_->vertexPositions_[collider.vertices_[2]].data_;

			auto min = _mm_min_ps(_mm_min_ps(point0, point1), point2);
			auto max = _mm_max_ps(_mm_max_ps(point0, point1), point2);
			auto margin = _mm_add_ps(_mm_add_ps(point0, point1), point2);
			const auto penetration = _mm_andnot_ps(_mm_set_ss(-0.0F), _mm_set_ss(shapeProp_.penetration));
			alignas(16) float marginValues[4];
			_mm_store_ps(marginValues, margin);
			margin = _mm_max_ss(_mm_set_ss(marginValues[3] * shapeProp_.margin / 3.0F), penetration);
			margin = _mm_shuffle_ps(margin, margin, 0);

			aabbs_[index].min_ = _mm_sub_ps(min, margin);
			aabbs_[index].max_ = _mm_add_ps(max, margin);
		}

		tree_.updateAabb();
	}

	void PerTriangleShape::markUsedVertices(std::vector<std::uint8_t>& a_flags)
	{
		for (const auto& collider : colliders_) {
			for (const auto vertex : collider.vertices_) {
				if (vertex < a_flags.size()) {
					a_flags[vertex] = 1;
				}
			}
		}

		if (verticesCollision_) {
			verticesCollision_->markUsedVertices(a_flags);
		}
	}

	void PerTriangleShape::remapVertices(const std::vector<std::uint32_t>& a_map)
	{
		for (auto& collider : colliders_) {
			for (auto& vertex : collider.vertices_) {
				if (vertex < a_map.size()) {
					vertex = a_map[vertex];
				}
			}
		}

		if (verticesCollision_) {
			verticesCollision_->remapVertices(a_map);
		}
	}

	void PerTriangleShape::addTriangle(const int a_vertex0, const int a_vertex1, const int a_vertex2)
	{
		if (!owner_ ||
			a_vertex0 < 0 || a_vertex1 < 0 || a_vertex2 < 0 ||
			static_cast<std::size_t>(a_vertex0) >= owner_->vertices_.size() ||
			static_cast<std::size_t>(a_vertex1) >= owner_->vertices_.size() ||
			static_cast<std::size_t>(a_vertex2) >= owner_->vertices_.size()) {
			return;
		}

		const Collider collider(a_vertex0, a_vertex1, a_vertex2);
		U32 keys[12]{};
		float weights[12]{};
		int count = 0;
		const Vertex* vertices[3]{
			std::addressof(owner_->vertices_[a_vertex0]),
			std::addressof(owner_->vertices_[a_vertex1]),
			std::addressof(owner_->vertices_[a_vertex2]),
		};

		for (const auto* vertex : vertices) {
			for (int weightIndex = 0; weightIndex < 4; ++weightIndex) {
				const auto weight = vertex->weight_[weightIndex];
				if (weight < FLT_EPSILON) {
					continue;
				}
				const auto bone = vertex->getBoneIdx(weightIndex);
				auto found = -1;
				for (int index = 0; index < count; ++index) {
					if (keys[index] == bone) {
						found = index;
						break;
					}
				}
				if (found >= 0) {
					weights[found] += weight;
				} else if (count < static_cast<int>(std::size(keys))) {
					keys[count] = bone;
					weights[count] = weight;
					++count;
				}
			}
		}

		for (int index = 1; index < count; ++index) {
			const auto weight = weights[index];
			const auto key = keys[index];
			auto destination = index;
			while (destination > 0 && (weights[destination - 1] < weight || (weights[destination - 1] == weight && keys[destination - 1] > key))) {
				weights[destination] = weights[destination - 1];
				keys[destination] = keys[destination - 1];
				--destination;
			}
			weights[destination] = weight;
			keys[destination] = key;
		}

		tree_.insertCollider(keys, count, collider);
	}
}
