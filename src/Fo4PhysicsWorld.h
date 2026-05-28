#pragma once

#include "DefaultBBP.h"
#include "LifecycleEvents.h"

#include <btBulletDynamicsCommon.h>

#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

class btBroadphaseInterface;
class btCollisionDispatcher;
class btCollisionShape;
class btDefaultCollisionConfiguration;
struct btDefaultMotionState;
class btDiscreteDynamicsWorld;
class btRigidBody;
class btSequentialImpulseConstraintSolver;
class btTypedConstraint;

namespace hdt
{
	class CollisionDispatcher;
	class SkinnedMeshBody;
}

namespace Smp
{
	class Fo4SkinnedMeshBone;
	enum class PhysicsConstraintKind;
	struct PhysicsXmlSummary;
	struct RuntimeSettings;

	enum class WritebackSource
	{
		kUnknown,
		kMainSync,
		kCellJobs,
		kPostAnimationGraph
	};

	enum class PrototypeBuildDomain
	{
		kArmor,
		kHead,
		kHair
	};

	class Fo4PhysicsWorld :
		public RE::BSTEventSink<LifecycleEvent>
	{
	public:
		~Fo4PhysicsWorld() override;

		static Fo4PhysicsWorld* GetSingleton();

		void Register();
		bool Initialize();
		void ApplyConfig(const RuntimeSettings& a_settings);
		void Reset();
		void StepFrame();
		void Step(float a_deltaSeconds);
		void UpdateWindLocked();
		void ApplyWindForcesLocked();
		void RecordFrameMetrics(float a_stepMs);
		void RecordWritebackMetric(float a_writebackMs, WritebackSource a_source, bool a_wroteAny, bool a_skippedDuplicate);
		void WriteBackPrototypeBodies(WritebackSource a_source = WritebackSource::kUnknown);
		void WriteBackPrototypeBodies(RE::Actor* a_actor, WritebackSource a_source = WritebackSource::kUnknown);

		RE::BSEventNotifyControl ProcessEvent(const LifecycleEvent& a_event, RE::BSTEventSource<LifecycleEvent>* a_source) override;

	private:
		Fo4PhysicsWorld() = default;
		Fo4PhysicsWorld(const Fo4PhysicsWorld&) = delete;
		Fo4PhysicsWorld(Fo4PhysicsWorld&&) = delete;
		Fo4PhysicsWorld& operator=(const Fo4PhysicsWorld&) = delete;
		Fo4PhysicsWorld& operator=(Fo4PhysicsWorld&&) = delete;

		bool InitializeLocked();
		void ResetLocked();
		void NoteLifecycleCandidate(const LifecycleEvent& a_event);
		void BuildPrototypeForEventLocked(const LifecycleEvent& a_event);
		void BuildHeadPrototypeForEventLocked(const LifecycleEvent& a_event);
		bool IsPrototypeCandidateLocked(const LifecycleEvent& a_event, bool a_requireObject);

		struct PrototypeBody
		{
			RE::Actor* actor{ nullptr };
			RE::NiNode* node{ nullptr };
			std::uint64_t buildGroup{ 0 };
			std::vector<std::uint64_t> buildGroups;
			std::vector<std::pair<std::uint64_t, PrototypeBuildDomain>> buildGroupDomains;
			std::string boneName;
			std::unique_ptr<btCollisionShape> shape;
			std::unique_ptr<btDefaultMotionState> motionState;
			std::unique_ptr<Fo4SkinnedMeshBone> bone;
		};

		struct PrototypeConstraint
		{
			std::uint64_t buildGroup{ 0 };
			PrototypeBuildDomain domain{ PrototypeBuildDomain::kArmor };
			std::string bodyA;
			std::string bodyB;
			PhysicsConstraintKind kind{};
			Fo4SkinnedMeshBone* boneA{ nullptr };
			Fo4SkinnedMeshBone* boneB{ nullptr };
			float scaleA{ 1.0F };
			float scaleB{ 1.0F };
			std::unique_ptr<btTypedConstraint> constraint;
		};

		struct PrototypeMesh
		{
			std::string name;
			RE::BSGeometry* geometry{ nullptr };
			std::uint64_t buildGroup{ 0 };
			PrototypeBuildDomain domain{ PrototypeBuildDomain::kArmor };
			RE::BSTSmartPointer<hdt::SkinnedMeshBody> body;
		};

		struct PrototypeActorState
		{
			RE::Actor* actor{ nullptr };
			RE::ActorHandle actorHandle;
			std::uint64_t nextBuildGroup{ 0 };
			std::uint64_t lastWritebackFrame{ 0 };
			WritebackSource lastWritebackSource{ WritebackSource::kUnknown };
			std::uint32_t resetReadFrames{ 0 };
			std::vector<PrototypeBody> bodies;
			std::vector<PrototypeMesh> meshes;
			std::vector<PrototypeConstraint> constraints;
		};

		struct SuspendedActorCandidate
		{
			RE::ActorHandle actorHandle;
		};

		PrototypeActorState* FindPrototypeStateLocked(RE::Actor* a_actor);
		PrototypeActorState& GetOrCreatePrototypeStateLocked(RE::Actor* a_actor);
		bool IsPrototypeStateValidLocked(PrototypeActorState& a_state);
		void PruneInvalidPrototypeStatesLocked();
		void EnforceActorBudgetLocked();
		void SuspendActorCandidateLocked(RE::Actor* a_actor);
		void TryReactivateSuspendedActorsLocked();
		void ClearPrototypeStateLocked(PrototypeActorState& a_state);
		bool ClearPrototypeGroupsForObjectLocked(PrototypeActorState& a_state, RE::NiAVObject* a_object);
		bool ClearPrototypeGroupsByDomainLocked(PrototypeActorState& a_state, PrototypeBuildDomain a_domain);
		void ClearPrototypeGroupsLocked(PrototypeActorState& a_state, const std::vector<std::uint64_t>& a_buildGroups);
		void ClearAllPrototypeStatesLocked();
		void BuildPrototypeBodiesLocked(PrototypeActorState& a_state, const LifecycleEvent& a_event, const PhysicsXmlSummary& a_summary, const DefaultBBP::NameMap& a_meshNameMap, PrototypeBuildDomain a_domain);
		void BuildPrototypeMeshesLocked(PrototypeActorState& a_state, const PhysicsXmlSummary& a_summary, const LifecycleEvent& a_event, const DefaultBBP::NameMap& a_meshNameMap, std::uint64_t a_buildGroup, PrototypeBuildDomain a_domain);
		void BuildPrototypeConstraintsLocked(PrototypeActorState& a_state, const PhysicsXmlSummary& a_summary, std::uint64_t a_buildGroup, PrototypeBuildDomain a_domain);
		void ScalePrototypeConstraintsLocked(PrototypeActorState& a_state);
		void LogRootConstraintDiagnosticsLocked(std::string_view a_phase, const PrototypeActorState& a_state);
		void UpdateMeshDisableStatesLocked(PrototypeActorState& a_state);

		std::mutex lock_;
		std::unique_ptr<btDefaultCollisionConfiguration> collisionConfiguration_;
		std::unique_ptr<hdt::CollisionDispatcher> dispatcher_;
		std::unique_ptr<btBroadphaseInterface> broadphase_;
		std::unique_ptr<btSequentialImpulseConstraintSolver> solver_;
		std::unique_ptr<btDiscreteDynamicsWorld> dynamicsWorld_;
		std::uint64_t candidateEvents_{ 0 };
		std::uint64_t simulationFrame_{ 1 };
		int maxSubSteps_{ 4 };
		float fixedStepSeconds_{ 1.0F / 60.0F };
		bool useRealTime_{ false };
		float budgetMs_{ 3.5F };
		int sampleSize_{ 5 };
		std::uint32_t metricFrameInterval_{ 60 };
		std::uint32_t metricFrameCounter_{ 0 };
		float averageStepMs_{ 0.0F };
		float averageWritebackMs_{ 0.0F };
		float pendingWritebackMs_{ 0.0F };
		std::uint32_t cellJobsWritebacks_{ 0 };
		std::uint32_t postAnimationWritebacks_{ 0 };
		std::uint32_t duplicateCellJobsWritebacks_{ 0 };
		std::uint32_t duplicatePostAnimationWritebacks_{ 0 };
		std::uint32_t diagnosticFrameBudget_{ 0 };
		int solverIterations_{ 10 };
		float solverErp_{ 0.2F };
		bool disableFirstPersonViewPhysics_{ false };
		bool enableNpcPhysics_{ true };
		bool autoAdjustMaxActors_{ false };
		std::size_t maxActiveActors_{ 4 };
		std::size_t currentMaxActiveActors_{ 4 };
		float maxActorDistance_{ 3000.0F };
		bool windEnabled_{ false };
		bool windUseWeather_{ false };
		float windStrength_{ 0.0F };
		float windDistanceForNoWind_{ 50.0F };
		float windDistanceForMaxWind_{ 3000.0F };
		btVector3 windDirection_{ 1.0F, 0.0F, 0.0F };
		btVector3 currentWind_{ 0.0F, 0.0F, 0.0F };
		std::string prototypePhysicsXml_;
		std::vector<PrototypeActorState> prototypeActors_;
		std::vector<SuspendedActorCandidate> suspendedActors_;
		bool registered_{ false };
	};
}
