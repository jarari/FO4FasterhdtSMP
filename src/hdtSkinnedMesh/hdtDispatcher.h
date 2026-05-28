#pragma once

#include "hdtBulletHelper.h"

#include <BulletCollision/CollisionDispatch/btCollisionDispatcher.h>

namespace hdt
{
	class SkinnedMeshBody;

	class CollisionDispatcher :
		public btCollisionDispatcher
	{
	public:
		explicit CollisionDispatcher(btCollisionConfiguration* a_collisionConfiguration);

		btPersistentManifold* getNewManifold(const btCollisionObject* a_body0, const btCollisionObject* a_body1) override;
		void releaseManifold(btPersistentManifold* a_manifold) override;
		bool needsCollision(const btCollisionObject* a_body0, const btCollisionObject* a_body1) override;
		void dispatchAllCollisionPairs(btOverlappingPairCache* a_pairCache, const btDispatcherInfo& a_dispatchInfo, btDispatcher* a_dispatcher) override;

		void clearAllManifold();

	private:
		SpinLock lock_;
		std::vector<std::pair<SkinnedMeshBody*, SkinnedMeshBody*>> pairs_;
	};
}
