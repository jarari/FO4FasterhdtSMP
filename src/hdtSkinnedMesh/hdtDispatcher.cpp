#include "hdtDispatcher.h"

#include "hdtSkinnedMeshAlgorithm.h"
#include "hdtSkinnedMeshBody.h"
#include "hdtSkinnedMeshShape.h"

#include <LinearMath/btPoolAllocator.h>

#include <algorithm>

namespace
{
	bool IsSkinnedMesh(const btCollisionObject* a_object)
	{
		return a_object &&
		       a_object->getCollisionShape() &&
		       a_object->getCollisionShape()->getShapeType() == CUSTOM_CONCAVE_SHAPE_TYPE;
	}

	bool BonesCanCollide(const btCollisionObject* a_body0, const btCollisionObject* a_body1)
	{
		auto* bone0 = a_body0 ? static_cast<hdt::SkinnedMeshBone*>(a_body0->getUserPointer()) : nullptr;
		auto* bone1 = a_body1 ? static_cast<hdt::SkinnedMeshBone*>(a_body1->getUserPointer()) : nullptr;
		return !bone0 || !bone1 || (bone0->canCollideWith(bone1) && bone1->canCollideWith(bone0));
	}
}

namespace hdt
{
	CollisionDispatcher::CollisionDispatcher(btCollisionConfiguration* a_collisionConfiguration) :
		btCollisionDispatcher(a_collisionConfiguration)
	{}

	btPersistentManifold* CollisionDispatcher::getNewManifold(const btCollisionObject* a_body0, const btCollisionObject* a_body1)
	{
		std::lock_guard lock(lock_);
		return btCollisionDispatcher::getNewManifold(a_body0, a_body1);
	}

	void CollisionDispatcher::releaseManifold(btPersistentManifold* a_manifold)
	{
		std::lock_guard lock(lock_);
		btCollisionDispatcher::releaseManifold(a_manifold);
	}

	bool CollisionDispatcher::needsCollision(const btCollisionObject* a_body0, const btCollisionObject* a_body1)
	{
		const auto skinned0 = IsSkinnedMesh(a_body0);
		const auto skinned1 = IsSkinnedMesh(a_body1);
		if (skinned0 || skinned1) {
			const auto* shape0 = skinned0 ? static_cast<const SkinnedMeshBody*>(a_body0) : nullptr;
			const auto* shape1 = skinned1 ? static_cast<const SkinnedMeshBody*>(a_body1) : nullptr;
			return shape0 && shape1 && shape0 != shape1 && shape0->canCollideWith(shape1) && shape1->canCollideWith(shape0);
		}

		if (a_body0->isStaticOrKinematicObject() && a_body1->isStaticOrKinematicObject()) {
			return false;
		}
		return BonesCanCollide(a_body0, a_body1) && btCollisionDispatcher::needsCollision(a_body0, a_body1);
	}

	void CollisionDispatcher::dispatchAllCollisionPairs(btOverlappingPairCache* a_pairCache, const btDispatcherInfo& a_dispatchInfo, btDispatcher* a_dispatcher)
	{
		pairs_.clear();
		const auto pairCount = a_pairCache ? a_pairCache->getNumOverlappingPairs() : 0;
		if (pairCount <= 0) {
			return;
		}

		auto* pairs = a_pairCache->getOverlappingPairArrayPtr();
		for (int index = 0; index < pairCount; ++index) {
			auto& pair = pairs[index];
			auto* object0 = static_cast<btCollisionObject*>(pair.m_pProxy0->m_clientObject);
			auto* object1 = static_cast<btCollisionObject*>(pair.m_pProxy1->m_clientObject);
			const auto skinned0 = IsSkinnedMesh(object0);
			const auto skinned1 = IsSkinnedMesh(object1);
			if (!skinned0 && !skinned1) {
				getNearCallback()(pair, *this, a_dispatchInfo);
				continue;
			}

			auto* shape0 = skinned0 ? static_cast<SkinnedMeshBody*>(object0) : nullptr;
			auto* shape1 = skinned1 ? static_cast<SkinnedMeshBody*>(object1) : nullptr;
			if (shape0 && shape1 && shape0 != shape1 && shape0->canCollideWith(shape1) && shape1->canCollideWith(shape0)) {
				pairs_.emplace_back(shape0, shape1);
			}
		}

		for (const auto& [shape0, shape1] : pairs_) {
			if (shape0->shape_ && shape1->shape_ && shape0->shape_->tree_.collapseCollideL(std::addressof(shape1->shape_->tree_))) {
				SkinnedMeshAlgorithm::processCollision(shape0, shape1, this);
			}
		}

		(void)a_dispatcher;
	}

	void CollisionDispatcher::clearAllManifold()
	{
		std::lock_guard lock(lock_);
		for (int index = 0; index < m_manifoldsPtr.size(); ++index) {
			auto* manifold = m_manifoldsPtr[index];
			manifold->~btPersistentManifold();
			if (m_persistentManifoldPoolAllocator->validPtr(manifold)) {
				m_persistentManifoldPoolAllocator->freeMemory(manifold);
			} else {
				btAlignedFree(manifold);
			}
		}
		m_manifoldsPtr.clear();
	}
}
