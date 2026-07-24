#pragma once

#include "hdtSkinnedMeshSystem.h"

#include <BulletCollision/CollisionDispatch/btSimulationIslandManager.h>
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorldMt.h>

namespace hdt
{
	// Fallout 4 keeps actor/build-group ownership outside the Bullet world, but
	// the simulation behavior itself is the reference hdtSkinnedMeshWorld.
	class SkinnedMeshWorld :
		public btDiscreteDynamicsWorldMt
	{
	public:
		using btDiscreteDynamicsWorldMt::btDiscreteDynamicsWorldMt;
		~SkinnedMeshWorld() override;

		void addSkinnedMeshSystem(SkinnedMeshSystem* a_system);
		void removeSkinnedMeshSystem(SkinnedMeshSystem* a_system);
		void updateConstraintsForBone(SkinnedMeshBone* a_bone);

		int stepSimulation(
			btScalar a_remainingTimeStep,
			int a_maxSubSteps = 1,
			btScalar a_fixedTimeStep = btScalar(1.0F) / btScalar(60.0F)) override;
		int stepReference(btScalar a_remainingTimeStep, btScalar a_fixedTimeStep);

	protected:
		void applyGravity() override;
		void predictUnconstraintMotion(btScalar a_timeStep) override;
		void integrateTransforms(btScalar a_timeStep) override;
		void performDiscreteCollisionDetection() override;
		void calculateSimulationIslands() override;
		void solveConstraints(btContactSolverInfo& a_solverInfo) override;

		std::vector<RE::BSTSmartPointer<SkinnedMeshSystem>> systems_;
	};
}
