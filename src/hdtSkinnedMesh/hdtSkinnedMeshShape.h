#pragma once

#include "hdtCollider.h"
#include "hdtCollisionAlgorithm.h"

namespace hdt
{
	class SkinnedMeshBody;
	class PerVertexShape;
	class PerTriangleShape;

	class SkinnedMeshShape :
		public RE::BSIntrusiveRefCounted
	{
	public:
		BT_DECLARE_ALIGNED_ALLOCATOR();

		explicit SkinnedMeshShape(SkinnedMeshBody* a_body, bool a_attachToOwner = true);
		virtual ~SkinnedMeshShape() = default;

		virtual PerVertexShape* asPerVertexShape() { return nullptr; }
		virtual PerTriangleShape* asPerTriangleShape() { return nullptr; }

		const Aabb& getAabb() const { return tree_.aabbAll_; }

		virtual void clipColliders();
		virtual void finishBuild() = 0;
		virtual void internalUpdate() = 0;
		virtual void markUsedVertices(std::vector<std::uint8_t>& a_flags) = 0;
		virtual void remapVertices(const std::vector<std::uint32_t>& a_map) = 0;

		virtual float getColliderBoneWeight(const Collider* a_collider, int a_boneIndex) = 0;
		virtual int getColliderBoneIndex(const Collider* a_collider, int a_boneIndex) = 0;
		virtual btVector3 baryCoord(const Collider* a_collider, const btVector3& a_point) = 0;
		virtual float baryWeight(const btVector3& a_weight, int a_boneIndex) = 0;
		virtual int getBonePerCollider() = 0;

		SkinnedMeshBody* owner_{ nullptr };
		vectorA16<Aabb> aabbs_;
		vectorA16<Collider> colliders_;
		ColliderTree tree_;
	};

	class PerVertexShape :
		public SkinnedMeshShape
	{
	public:
		explicit PerVertexShape(SkinnedMeshBody* a_body, bool a_attachToOwner = true);
		~PerVertexShape() override = default;

		PerVertexShape* asPerVertexShape() override { return this; }
		void finishBuild() override;
		void internalUpdate() override;
		void markUsedVertices(std::vector<std::uint8_t>& a_flags) override;
		void remapVertices(const std::vector<std::uint32_t>& a_map) override;
		void autoGen();

		int getBonePerCollider() override { return 4; }
		float getColliderBoneWeight(const Collider* a_collider, int a_boneIndex) override;
		int getColliderBoneIndex(const Collider* a_collider, int a_boneIndex) override;
		btVector3 baryCoord(const Collider*, const btVector3&) override { return btVector3(1.0F, 1.0F, 1.0F); }
		float baryWeight(const btVector3&, int) override { return 1.0F; }

		struct ShapeProp
		{
			float margin{ 1.0F };
		} shapeProp_;
	};

	class PerTriangleShape :
		public SkinnedMeshShape
	{
	public:
		explicit PerTriangleShape(SkinnedMeshBody* a_body);
		~PerTriangleShape() override = default;

		PerVertexShape* asPerVertexShape() override { return verticesCollision_.get(); }
		PerTriangleShape* asPerTriangleShape() override { return this; }
		void finishBuild() override;
		void internalUpdate() override;
		void markUsedVertices(std::vector<std::uint8_t>& a_flags) override;
		void remapVertices(const std::vector<std::uint32_t>& a_map) override;
		void addTriangle(int a_vertex0, int a_vertex1, int a_vertex2);

		int getBonePerCollider() override { return 12; }
		float getColliderBoneWeight(const Collider* a_collider, int a_boneIndex) override;
		int getColliderBoneIndex(const Collider* a_collider, int a_boneIndex) override;
		btVector3 baryCoord(const Collider* a_collider, const btVector3& a_point) override;
		float baryWeight(const btVector3& a_weight, int a_boneIndex) override { return a_weight[a_boneIndex / 4]; }

		struct ShapeProp
		{
			float margin{ 1.0F };
			float penetration{ 1.0F };
		} shapeProp_;

		RE::BSTSmartPointer<PerVertexShape> verticesCollision_;
	};
}
