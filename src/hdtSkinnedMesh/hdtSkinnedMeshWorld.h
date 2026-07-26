#pragma once

#include "hdtSkinnedMeshSystem.h"

#include <BulletCollision/CollisionDispatch/btSimulationIslandManager.h>
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorldMt.h>

namespace hdt
{
	class SkinnedMeshWorld :
		public btDiscreteDynamicsWorldMt
	{
	public:
		using btDiscreteDynamicsWorldMt::btDiscreteDynamicsWorldMt;
		~SkinnedMeshWorld() override;

		virtual void addSkinnedMeshSystem(SkinnedMeshSystem* a_system);
		virtual void removeSkinnedMeshSystem(SkinnedMeshSystem* a_system);
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

		std::vector<RE::BSTSmartPointer<SkinnedMeshSystem>> m_systems;
	};
}
