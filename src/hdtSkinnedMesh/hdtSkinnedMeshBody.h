#pragma once

#include "hdtSkinnedMeshBone.h"
#include "hdtVertex.h"

#include <cstdint>
#include <vector>

namespace hdt
{
	class SkinnedMeshShape;

	class SkinnedMeshBody :
		public btCollisionObject,
		public RE::BSIntrusiveRefCounted
	{
	public:
		using btCollisionObject::operator delete;
		using btCollisionObject::operator delete[];
		using btCollisionObject::operator new;
		using btCollisionObject::operator new[];

		SkinnedMeshBody();
		~SkinnedMeshBody() override = default;

		struct CollisionShape :
			public btCollisionShape
		{
			CollisionShape();

			void getAabb(const btTransform& a_transform, btVector3& aabbMin, btVector3& aabbMax) const override;
			void setLocalScaling(const btVector3&) override {}
			const btVector3& getLocalScaling() const override;
			void calculateLocalInertia(btScalar, btVector3& a_inertia) const override { a_inertia.setZero(); }
			const char* getName() const override { return "btSkinnedMeshBody"; }
			btScalar getMargin() const override { return 0.0F; }
			void setMargin(btScalar) override {}

			Aabb aabb;
		} bulletShape_;

		struct SkinnedBone
		{
			btMatrix4x3T vertexToBone;
			BoundingSphere localBoundingSphere;
			BoundingSphere worldBoundingSphere;
			SkinnedMeshBone* ptr{ nullptr };
			float weightThreshold{ 0.0F };
			bool isKinematic{ true };
		};

		enum class SharedType
		{
			kPublic,
			kInternal,
			kExternal,
			kPrivate
		};

		int addBone(SkinnedMeshBone* a_bone, const btQsTransform& a_verticesToBone, const BoundingSphere& a_boundingSphere);
		void finishBuild();
		virtual void internalUpdate();
		float flexible(const Vertex& a_vertex);
		bool canCollideWith(const SkinnedMeshBone* a_bone) const;
		virtual bool canCollideWith(const SkinnedMeshBody* a_body) const;
		void updateBoundingSphereAabb();
		bool isBoundingSphereCollided(SkinnedMeshBody* a_rhs);

		RE::BSFixedString name_;
		RE::Actor* actor_{ nullptr };
		std::uint64_t buildGroup_{ 0 };
		SharedType shared_{ SharedType::kPublic };
		bool disabled_{ false };
		bool isKinematic_{ true };
		bool useBoundingSphere_{ false };
		int disablePriority_{ 0 };
		RE::BSFixedString disableTag_;
		RE::BSTSmartPointer<SkinnedMeshShape> shape_;
		std::vector<SkinnedBone> skinnedBones_;
		std::vector<Bone> bones_;
		std::vector<Vertex> vertices_;
		std::vector<VertexPos> vertexPositions_;
		std::vector<RE::BSFixedString> tags_;
		std::vector<RE::BSFixedString> canCollideWithTags_;
		std::vector<RE::BSFixedString> noCollideWithTags_;
		std::vector<SkinnedMeshBone*> canCollideWithBones_;
		std::vector<SkinnedMeshBone*> noCollideWithBones_;
	};
}
