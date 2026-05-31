#pragma once

#include "DefaultBBP.h"
#include "LifecycleEvents.h"
#include "RE/N/NiTransform.h"

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

namespace RE
{
	class MenuOpenCloseEvent;
}

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
		public RE::BSTEventSink<LifecycleEvent>,
		public RE::BSTEventSink<RE::MenuOpenCloseEvent>
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
		void ProcessPendingRebuilds();

		RE::BSEventNotifyControl ProcessEvent(const LifecycleEvent& a_event, RE::BSTEventSource<LifecycleEvent>* a_source) override;
		RE::BSEventNotifyControl ProcessEvent(const RE::MenuOpenCloseEvent& a_event, RE::BSTEventSource<RE::MenuOpenCloseEvent>* a_source) override;

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
			RE::NiNode* resetParent{ nullptr };
			std::unique_ptr<RE::NiTransform> resetLocalToParent;
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

		struct PrototypeMergedNode
		{
			std::uint64_t buildGroup{ 0 };
			RE::NiNode* parent{ nullptr };
			RE::NiPointer<RE::NiAVObject> node;
		};

		struct PrototypeArmorRecord
		{
			RE::BIPED_OBJECT bipedObject{ RE::BIPED_OBJECT::kTotal };
			std::string physicsXmlPath;
			DefaultBBP::NameMap meshNameMap;
			std::vector<LifecycleMergedSkeletonNode> preMergedSkeletonNodes;
			std::vector<LifecycleMergedRootNode> preMergedRootNodes;
			std::uint32_t cpuCopyRetryCount{ 0 };
		};

		struct PrototypeBuildResult
		{
			bool cpuCopyPending{ false };
			bool succeeded{ false };
		};

		struct PrototypeActorState
		{
			RE::Actor* actor{ nullptr };
			RE::ActorHandle actorHandle;
			bool firstPerson{ false };
			std::uint64_t nextBuildGroup{ 0 };
			std::uint32_t nextArmorRenameId{ 0 };
			std::uint32_t nextHeadRenameId{ 0 };
			std::uint64_t lastWritebackFrame{ 0 };
			WritebackSource lastWritebackSource{ WritebackSource::kUnknown };
			std::uint32_t resetReadFrames{ 0 };
			float currentWindFactor{ 1.0F };
			bool runtimeSuspended{ false };
			RE::NiPointer<RE::NiAVObject> faceNode;
			std::vector<PrototypeBody> bodies;
			std::vector<PrototypeMesh> meshes;
			std::vector<PrototypeConstraint> constraints;
			std::vector<PrototypeMergedNode> mergedNodes;
			std::vector<PrototypeArmorRecord> armorRecords;

			[[nodiscard]] bool HasRuntime() const
			{
				return !bodies.empty() || !meshes.empty() || !constraints.empty();
			}
		};

		struct SuspendedActorCandidate
		{
			RE::ActorHandle actorHandle;
			bool firstPerson{ false };
			std::vector<PrototypeArmorRecord> armorRecords;
		};

		struct PendingActorRebuild
		{
			RE::ActorHandle actorHandle;
			bool firstPerson{ false };
			std::vector<PrototypeArmorRecord> armorRecords;
			std::uint32_t frameDelay{ 0 };
		};

		struct PendingHeadRebuild
		{
			RE::ActorHandle actorHandle;
			LifecycleEventType type{ LifecycleEventType::kActorHeadInitialized };
			RE::NiPointer<RE::NiAVObject> object;
			RE::BGSHeadPart* headPart{ nullptr };
			std::uint32_t frameDelay{ 0 };
		};

		PrototypeActorState* FindPrototypeStateLocked(RE::Actor* a_actor, bool a_firstPerson);
		PrototypeActorState& GetOrCreatePrototypeStateLocked(RE::Actor* a_actor, bool a_firstPerson);
		bool IsPrototypeStateValidLocked(PrototypeActorState& a_state);
		void PruneInvalidPrototypeStatesLocked();
		void EnforceActorBudgetLocked();
		void SuspendActorCandidateLocked(RE::Actor* a_actor, bool a_firstPerson, std::vector<PrototypeArmorRecord> a_armorRecords = {});
		void TryReactivateSuspendedActorsLocked();
		void TryReactivateSuspendedPrototypeStatesLocked();
		void MarkPendingActorRebuildLocked(RE::Actor* a_actor, bool a_firstPerson, std::vector<PrototypeArmorRecord> a_armorRecords = {});
		void MarkPendingHeadRebuildLocked(const LifecycleEvent& a_event);
		void SchedulePendingRebuildTaskLocked();
		bool HasActiveOrPendingActorRebuildLocked(RE::Actor* a_actor);
		bool HasPendingArmorRecordRebuildLocked(RE::Actor* a_actor);
		std::vector<PrototypeArmorRecord> CollectSuspendedArmorRecordsLocked(const LifecycleEvent& a_event);
		void RecordPrototypeArmorLocked(
			PrototypeActorState& a_state,
			RE::BIPED_OBJECT a_bipedObject,
			std::string a_physicsXmlPath,
			const DefaultBBP::NameMap& a_meshNameMap,
			const std::vector<LifecycleMergedSkeletonNode>& a_preMergedSkeletonNodes,
			const std::vector<LifecycleMergedRootNode>& a_preMergedRootNodes);
		bool RebuildPendingArmorRecordsLocked(RE::Actor* a_actor, bool a_firstPerson, std::vector<PrototypeArmorRecord>& a_armorRecords, std::uint32_t* a_retryDelay = nullptr);
		void TryRebuildPendingActorsLocked(RE::Actor* a_actor = nullptr);
		void TryRebuildPendingHeadsLocked();
		void SuspendPrototypeStatesForCustomizationMenuLocked();
		void ReloadPrototypeStatesForCustomizationMenuLocked();
		void ClearPrototypeStatesForMenuRebuildLocked();
		void SuspendPrototypeRuntimeLocked(PrototypeActorState& a_state);
		void ClearPrototypeStateLocked(PrototypeActorState& a_state, bool a_restoreSkinSlots = true);
		bool ClearPrototypeGroupsForObjectLocked(PrototypeActorState& a_state, RE::NiAVObject* a_object);
		bool ClearPrototypeGroupsForBoneNamesLocked(PrototypeActorState& a_state, std::span<const std::string> a_boneNames, PrototypeBuildDomain a_domain);
		bool ClearPrototypeGroupsByDomainLocked(PrototypeActorState& a_state, PrototypeBuildDomain a_domain);
		void ClearPrototypeGroupsLocked(PrototypeActorState& a_state, const std::vector<std::uint64_t>& a_buildGroups);
		void ClearAllPrototypeStatesLocked();
		void ResumeFromLoadingMenuLocked();
		PrototypeBuildResult BuildPrototypeBodiesLocked(PrototypeActorState& a_state, const LifecycleEvent& a_event, const PhysicsXmlSummary& a_summary, const DefaultBBP::NameMap& a_meshNameMap, PrototypeBuildDomain a_domain);
		void BuildPrototypeMeshesLocked(PrototypeActorState& a_state, const PhysicsXmlSummary& a_summary, const LifecycleEvent& a_event, const DefaultBBP::NameMap& a_meshNameMap, std::uint64_t a_buildGroup, PrototypeBuildDomain a_domain);
		void BuildPrototypeConstraintsLocked(PrototypeActorState& a_state, const PhysicsXmlSummary& a_summary, std::uint64_t a_buildGroup, PrototypeBuildDomain a_domain);
		void ResetPrototypeBuildGroupToCurrentPoseLocked(PrototypeActorState& a_state, std::uint64_t a_buildGroup);
		void ScalePrototypeConstraintsLocked(PrototypeActorState& a_state);
		void LogRootConstraintDiagnosticsLocked(std::string_view a_phase, const PrototypeActorState& a_state);
		void UpdateMeshDisableStatesLocked(PrototypeActorState& a_state);
		void ResetStepClockLocked();

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
		float averageInterval_{ 1.0F / 60.0F };
		float accumulatedInterval_{ 0.0F };
		float currentStepSeconds_{ 1.0F / 60.0F };
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
		bool disableSMPHairWhenWigEquipped_{ true };
		float windDistanceForNoWind_{ 50.0F };
		float windDistanceForMaxWind_{ 3000.0F };
		float windWeatherCooldown_{ 0.0F };
		float windWeatherShortCooldownSeconds_{ 0.5F };
		float windWeatherLongCooldownSeconds_{ 5.0F };
		int windSmoothingSamples_{ 8 };
		bool randomizePerBoneWind_{ true };
		btVector3 windDirection_{ 1.0F, 0.0F, 0.0F };
		btVector3 currentWind_{ 0.0F, 0.0F, 0.0F };
		btVector3 targetWind_{ 0.0F, 0.0F, 0.0F };
		std::string prototypePhysicsXml_;
		std::vector<PrototypeActorState> prototypeActors_;
		std::vector<SuspendedActorCandidate> suspendedActors_;
		std::vector<PendingActorRebuild> pendingActorRebuilds_;
		std::vector<PendingHeadRebuild> pendingHeadRebuilds_;
		std::uint64_t nextPendingRebuildFrame_{ 1 };
		std::uint32_t characterCustomizationMenuDepth_{ 0 };
		std::uint32_t loadingMenuDepth_{ 0 };
		bool loadingPhysicsSuspended_{ false };
		bool customizationCloseReloadQueued_{ false };
		bool pendingRebuildTaskQueued_{ false };
		bool registered_{ false };
	};
}
