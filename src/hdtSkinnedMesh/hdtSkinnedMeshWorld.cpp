#include "hdtSkinnedMeshWorld.h"

#include "hdtDispatcher.h"
#include "hdtSkinnedMeshBody.h"
#include "hdtSkinnedMeshShape.h"

namespace hdt
{
	SkinnedMeshWorld::~SkinnedMeshWorld()
	{
		while (!systems_.empty()) {
			removeSkinnedMeshSystem(systems_.back().get());
		}
	}

	void SkinnedMeshWorld::addSkinnedMeshSystem(SkinnedMeshSystem* a_system)
	{
		if (!a_system ||
			std::ranges::find_if(systems_, [a_system](const auto& a_entry) {
				return a_entry.get() == a_system;
			}) != systems_.end()) {
			return;
		}

		systems_.emplace_back(a_system);
		for (const auto& mesh : a_system->meshes_) {
			if (mesh) {
				addCollisionObject(mesh.get(), 1, 1);
			}
		}
		for (const auto& bone : a_system->bones_) {
			if (!bone) {
				continue;
			}
			bone->m_rig.setActivationState(DISABLE_DEACTIVATION);
			addRigidBody(std::addressof(bone->m_rig), 0, 0);
		}
		for (const auto& group : a_system->constraintGroups_) {
			if (!group) {
				continue;
			}
			for (const auto& constraint : group->m_constraints) {
				if (constraint && constraint->getConstraint()) {
					addConstraint(constraint->getConstraint(), true);
				}
			}
		}
		for (const auto& constraint : a_system->constraints_) {
			if (constraint && constraint->getConstraint()) {
				addConstraint(constraint->getConstraint(), true);
			}
		}

		a_system->readTransform(a_system->prepareForRead(-10.0F));
		a_system->world_ = this;
	}

	void SkinnedMeshWorld::removeSkinnedMeshSystem(SkinnedMeshSystem* a_system)
	{
		const auto found = std::ranges::find_if(systems_, [a_system](const auto& a_entry) {
			return a_entry.get() == a_system;
		});
		if (found == systems_.end()) {
			return;
		}

		for (const auto& group : a_system->constraintGroups_) {
			if (!group) {
				continue;
			}
			for (const auto& constraint : group->m_constraints) {
				if (constraint && constraint->getConstraint()) {
					removeConstraint(constraint->getConstraint());
				}
			}
		}
		for (const auto& mesh : a_system->meshes_) {
			if (mesh) {
				removeCollisionObject(mesh.get());
			}
		}
		for (const auto& constraint : a_system->constraints_) {
			if (constraint && constraint->getConstraint()) {
				removeConstraint(constraint->getConstraint());
			}
		}
		for (const auto& bone : a_system->bones_) {
			if (bone) {
				removeRigidBody(std::addressof(bone->m_rig));
			}
		}

		*found = std::move(systems_.back());
		systems_.pop_back();
		a_system->world_ = nullptr;
	}

	void SkinnedMeshWorld::updateConstraintsForBone(SkinnedMeshBone* a_bone)
	{
		if (!a_bone) {
			return;
		}

		for (int index = 0; index < m_constraints.size(); ++index) {
			auto* constraint = m_constraints[index];
			if (!constraint ||
				(std::addressof(constraint->getRigidBodyA()) != std::addressof(a_bone->m_rig) &&
					std::addressof(constraint->getRigidBodyB()) != std::addressof(a_bone->m_rig))) {
				continue;
			}
			constraint->setEnabled(
				!constraint->getRigidBodyA().isStaticOrKinematicObject() ||
				!constraint->getRigidBodyB().isStaticOrKinematicObject());
		}
	}

	int SkinnedMeshWorld::stepSimulation(
		const btScalar a_remainingTimeStep,
		[[maybe_unused]] const int a_maxSubSteps,
		const btScalar a_fixedTimeStep)
	{
		return stepReference(a_remainingTimeStep, a_fixedTimeStep);
	}

	int SkinnedMeshWorld::stepReference(btScalar a_remainingTimeStep, const btScalar a_fixedTimeStep)
	{
		applyGravity();
		while (a_remainingTimeStep > a_fixedTimeStep) {
			internalSingleStepSimulation(a_fixedTimeStep);
			a_remainingTimeStep -= a_fixedTimeStep;
		}

		constexpr auto minimumPeriod = btScalar(1.0F / 300.0F);
		if (a_remainingTimeStep > minimumPeriod) {
			internalSingleStepSimulation(a_remainingTimeStep);
		}
		clearForces();
		return 0;
	}

	void SkinnedMeshWorld::performDiscreteCollisionDetection()
	{
		BT_PROFILE("performDiscreteCollisionDetection");
		for (const auto& system : systems_) {
			if (system) {
				system->internalUpdate();
			}
		}

		// FO4 owns bones and meshes per actor/build group rather than exclusively
		// through SkinnedMeshSystem, so update directly registered objects too.
		if (systems_.empty()) {
			for (int index = 0; index < m_collisionObjects.size(); ++index) {
				if (auto* rigidBody = btRigidBody::upcast(m_collisionObjects[index])) {
					if (auto* bone = static_cast<SkinnedMeshBone*>(rigidBody->getUserPointer())) {
						bone->internalUpdate();
					}
				}
			}
			for (int index = 0; index < m_collisionObjects.size(); ++index) {
				auto* object = m_collisionObjects[index];
				if (!object || !object->getCollisionShape() ||
					object->getCollisionShape()->getShapeType() != CUSTOM_CONCAVE_SHAPE_TYPE) {
					continue;
				}
				static_cast<SkinnedMeshBody*>(object)->updateBoundingSphereAabb();
			}
		}

		auto& dispatchInfo = getDispatchInfo();
		for (int index = 0; index < m_collisionObjects.size(); ++index) {
			auto* object = m_collisionObjects[index];
			auto* proxy = object ? object->getBroadphaseHandle() : nullptr;
			if (!object || !proxy ||
				(proxy->m_collisionFilterGroup == 0 && proxy->m_collisionFilterMask == 0)) {
				continue;
			}

			btVector3 minimum;
			btVector3 maximum;
			object->getCollisionShape()->getAabb(object->getWorldTransform(), minimum, maximum);
			m_broadphasePairCache->setAabb(proxy, minimum, maximum, m_dispatcher1);
		}

		m_broadphasePairCache->calculateOverlappingPairs(m_dispatcher1);
		if (m_dispatcher1) {
			m_dispatcher1->dispatchAllCollisionPairs(
				m_broadphasePairCache->getOverlappingPairCache(),
				dispatchInfo,
				m_dispatcher1);
		}
	}

	void SkinnedMeshWorld::applyGravity()
	{
		const auto worldGravity = getGravity();
		for (int index = 0; index < m_collisionObjects.size(); ++index) {
			auto* body = btRigidBody::upcast(m_collisionObjects[index]);
			if (!body || body->isStaticOrKinematicObject() ||
				(body->getFlags() & BT_DISABLE_WORLD_GRAVITY)) {
				continue;
			}

			if (const auto* bone = static_cast<SkinnedMeshBone*>(body->getUserPointer())) {
				body->setGravity(worldGravity * std::clamp(bone->m_gravityFactor, 0.0F, 1.0F));
			}
		}
		btDiscreteDynamicsWorldMt::applyGravity();
	}

	void SkinnedMeshWorld::predictUnconstraintMotion(const btScalar a_timeStep)
	{
		struct Update :
			public btIParallelForBody
		{
			btScalar timeStep{ 0.0F };
			btRigidBody** bodies{ nullptr };

			void forLoop(const int a_begin, const int a_end) const override
			{
				for (int index = a_begin; index < a_end; ++index) {
					auto* body = bodies[index];
					if (!body->isStaticOrKinematicObject()) {
						body->applyDamping(timeStep);
					}
					body->predictIntegratedTransform(timeStep, body->getInterpolationWorldTransform());
				}
			}
		};

		if (m_nonStaticRigidBodies.size() == 0) {
			return;
		}

		Update update;
		update.timeStep = a_timeStep;
		update.bodies = std::addressof(m_nonStaticRigidBodies[0]);
		btParallelFor(0, m_nonStaticRigidBodies.size(), 100, update);
	}

	void SkinnedMeshWorld::integrateTransforms(const btScalar a_timeStep)
	{
		for (int index = 0; index < m_collisionObjects.size(); ++index) {
			auto* body = m_collisionObjects[index];
			if (!body || !body->isKinematicObject()) {
				continue;
			}
			btTransformUtil::integrateTransform(
				body->getWorldTransform(),
				body->getInterpolationLinearVelocity(),
				body->getInterpolationAngularVelocity(),
				a_timeStep,
				body->getInterpolationWorldTransform());
			body->setWorldTransform(body->getInterpolationWorldTransform());
		}

		const btVector3 limitMin(-1e9F, -1e9F, -1e9F);
		const btVector3 limitMax(1e9F, 1e9F, 1e9F);
		for (int index = 0; index < m_nonStaticRigidBodies.size(); ++index) {
			auto* body = m_nonStaticRigidBodies[index];
			auto linearVelocity = body->getLinearVelocity();
			linearVelocity.setMax(limitMin);
			linearVelocity.setMin(limitMax);
			body->setLinearVelocity(linearVelocity);
			auto angularVelocity = body->getAngularVelocity();
			angularVelocity.setMax(limitMin);
			angularVelocity.setMin(limitMax);
			body->setAngularVelocity(angularVelocity);
		}

		btDiscreteDynamicsWorldMt::integrateTransforms(a_timeStep);
	}

	void SkinnedMeshWorld::calculateSimulationIslands()
	{
		BT_PROFILE("calculateSimulationIslands");
		getSimulationIslandManager()->updateActivationState(
			getCollisionWorld(),
			getCollisionWorld()->getDispatcher());
		auto* unionFind = std::addressof(getSimulationIslandManager()->getUnionFind());

		for (int index = 0; index < m_predictiveManifolds.size(); ++index) {
			auto* manifold = m_predictiveManifolds[index];
			const auto* object0 = manifold->getBody0();
			const auto* object1 = manifold->getBody1();
			if (object0 && !object0->isStaticOrKinematicObject() &&
				object1 && !object1->isStaticOrKinematicObject()) {
				unionFind->unite(object0->getIslandTag(), object1->getIslandTag());
			}
		}

		for (int index = 0; index < m_constraints.size(); ++index) {
			auto* constraint = m_constraints[index];
			if (!constraint || !constraint->isEnabled()) {
				continue;
			}
			const auto* object0 = std::addressof(constraint->getRigidBodyA());
			const auto* object1 = std::addressof(constraint->getRigidBodyB());
			if (!object0->isStaticOrKinematicObject() && !object1->isStaticOrKinematicObject()) {
				unionFind->unite(object0->getIslandTag(), object1->getIslandTag());
			}
		}

		auto* dispatcher = getCollisionWorld()->getDispatcher();
		const auto manifoldCount = dispatcher ? dispatcher->getNumManifolds() : 0;
		for (int index = 0; index < manifoldCount; ++index) {
			auto* manifold = dispatcher->getManifoldByIndexInternal(index);
			if (!manifold || manifold->getNumContacts() <= 0) {
				continue;
			}
			const auto* object0 = static_cast<const btCollisionObject*>(manifold->getBody0());
			const auto* object1 = static_cast<const btCollisionObject*>(manifold->getBody1());
			if (object0 && !object0->isStaticOrKinematicObject() &&
				object1 && !object1->isStaticOrKinematicObject()) {
				unionFind->unite(object0->getIslandTag(), object1->getIslandTag());
			}
		}

		getSimulationIslandManager()->storeIslandActivationState(getCollisionWorld());
	}

	void SkinnedMeshWorld::solveConstraints(btContactSolverInfo& a_solverInfo)
	{
		BT_PROFILE("solveConstraints");
		if (m_collisionObjects.size() == 0) {
			return;
		}

		for (int index = 0; index < m_constraints.size(); ++index) {
			auto* constraint = m_constraints[index];
			if (constraint && constraint->isEnabled() &&
				constraint->getRigidBodyA().isStaticOrKinematicObject() &&
				constraint->getRigidBodyB().isStaticOrKinematicObject()) {
				constraint->setEnabled(false);
			}
		}

		btDiscreteDynamicsWorldMt::solveConstraints(a_solverInfo);
		static_cast<CollisionDispatcher*>(m_dispatcher1)->clearAllManifold();
	}
}
