#pragma once

#include "DefaultBBP.h"
#include "Fo4SkinnedMeshBone.h"
#include "LifecycleEvents.h"
#include "RE/N/NiTransform.h"

#include <btBulletDynamicsCommon.h>

#include <memory>
#include <mutex>
#include <span>
#include <string>
#include <string_view>
#include <unordered_map>
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
	class NiAVObject;
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
		~Fo4PhysicsWorld() noexcept override;

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
		void DrawBulletVisualization();

		RE::BSEventNotifyControl ProcessEvent(const LifecycleEvent& a_event, RE::BSTEventSource<LifecycleEvent>* a_source) override;
		RE::BSEventNotifyControl ProcessEvent(const RE::MenuOpenCloseEvent& a_event, RE::BSTEventSource<RE::MenuOpenCloseEvent>* a_source) override;

	private:
		struct AsyncStepState;

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
			RE::BIPED_OBJECT bipedObject{ RE::BIPED_OBJECT::kTotal };
			std::vector<std::uint64_t> buildGroups;
			std::vector<std::pair<std::uint64_t, PrototypeBuildDomain>> buildGroupDomains;
			std::vector<std::pair<std::uint64_t, RE::BIPED_OBJECT>> buildGroupBipedObjects;
			std::string boneName;
			std::unique_ptr<btCollisionShape> shape;
			std::unique_ptr<btDefaultMotionState> motionState;
			std::unique_ptr<Fo4SkinnedMeshBone> bone;
			bool inBulletWorld{ false };
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
			bool inBulletWorld{ false };
		};

		struct PrototypeMesh
		{
			std::string name;
			RE::BSGeometry* geometry{ nullptr };
			std::uint64_t buildGroup{ 0 };
			RE::BIPED_OBJECT bipedObject{ RE::BIPED_OBJECT::kTotal };
			PrototypeBuildDomain domain{ PrototypeBuildDomain::kArmor };
			RE::BSTSmartPointer<hdt::SkinnedMeshBody> body;
			bool inBulletWorld{ false };
		};

		struct PrototypeMergedNode
		{
			std::uint64_t buildGroup{ 0 };
			RE::NiNode* parent{ nullptr };
			RE::NiPointer<RE::NiAVObject> node;
			std::string sourceName;
			std::string recordParentName;
			RE::NiTransform localToParent{ RE::NiTransform::IDENTITY };
			RE::NiTransform recordLocalToParent{ RE::NiTransform::IDENTITY };
			bool hasLocalToParent{ false };
			bool hasRecordLocalToParent{ false };
			bool recordMergeParentBinding{ false };
		};

		struct PrototypeArmorRecord
		{
			RE::BIPED_OBJECT bipedObject{ RE::BIPED_OBJECT::kTotal };
			std::string physicsXmlPath;
			DefaultBBP::NameMap meshNameMap;
			RE::NiPointer<RE::NiAVObject> attachedObject;
			RE::NiPointer<RE::NiAVObject> sourceObject;
			RE::NiPointer<RE::NiAVObject> mergeSourceObject;
			std::vector<MergeParentBinding> mergeParentBindings;
			std::vector<MergeRename> mergeRenameMap;
			std::vector<std::uint64_t> buildGroups;
			std::uint32_t cpuCopyRetryCount{ 0 };
		};

		struct PrototypeAttachmentRecord
		{
			RE::BIPED_OBJECT bipedObject{ RE::BIPED_OBJECT::kTotal };
			std::uint32_t generation{ 0 };
			std::string physicsXmlPath;
			RE::NiPointer<RE::NiAVObject> attachedObject;
			RE::NiPointer<RE::NiAVObject> sourceObject;
			std::vector<std::uint64_t> buildGroups;
		};

		struct PrototypeBuildResult
		{
			std::uint64_t buildGroup{ 0 };
			bool cpuCopyPending{ false };
			bool committed{ false };
			bool recordable{ false };
			bool succeeded{ false };
		};

		struct PrototypeBuildGroupRuntime
		{
			std::uint64_t buildGroup{ 0 };
			PrototypeBuildDomain domain{ PrototypeBuildDomain::kArmor };
			RE::BIPED_OBJECT bipedObject{ RE::BIPED_OBJECT::kTotal };
			std::vector<hdt::SkinnedMeshBody*> meshes;
			std::vector<Fo4SkinnedMeshBone*> bones;
			std::vector<btTypedConstraint*> constraints;
		};

		struct PrototypeActorState
		{
			RE::Actor* actor{ nullptr };
			RE::ActorHandle actorHandle;
			bool firstPerson{ false };
			RE::NiPointer<RE::NiAVObject> lastReadRoot;
			bool readInitialized{ false };
			std::uint64_t nextBuildGroup{ 0 };
			std::uint32_t nextAttachmentGeneration{ 0 };
			std::uint64_t lastWritebackFrame{ 0 };
			WritebackSource lastWritebackSource{ WritebackSource::kUnknown };
			std::uint32_t resetReadFrames{ 0 };
			float currentWindFactor{ 1.0F };
			bool runtimeSuspended{ false };
			bool runtimeSoftSuspended{ false };
			RE::NiPointer<RE::NiAVObject> faceNode;
			std::vector<PrototypeBody> bodies;
			std::vector<PrototypeMesh> meshes;
			std::vector<PrototypeConstraint> constraints;
			std::vector<PrototypeMergedNode> mergedNodes;
			std::vector<PrototypeArmorRecord> armorRecords;
			std::vector<PrototypeAttachmentRecord> attachmentRecords;
			std::vector<PrototypeBuildGroupRuntime> runtimes;
			std::vector<Fo4SkinnedMeshBone::SkinSlotRestore> suspendedSkinSlots;

			[[nodiscard]] bool HasRuntime() const
			{
				return !runtimes.empty() || !bodies.empty() || !meshes.empty() || !constraints.empty();
			}

			[[nodiscard]] bool HasActiveRuntime() const
			{
				return HasRuntime() && !runtimeSuspended && !runtimeSoftSuspended;
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
			bool forceArmorRescan{ false };
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
		bool ShouldBuildSuspendedArmorCandidateLocked(const LifecycleEvent& a_event) const;
		void SoftSuspendBuiltRuntimeIfOutOfRangeLocked(PrototypeActorState& a_state, const LifecycleEvent& a_event);
		static void MergePrototypeArmorRecord(std::vector<PrototypeArmorRecord>& a_records, PrototypeArmorRecord a_record);
		PendingActorRebuild* FindPendingActorRebuildLocked(RE::Actor* a_actor, bool a_firstPerson);
		std::vector<PrototypeArmorRecord> CollectQueuedArmorRecordsForAttachLocked(const LifecycleEvent& a_event);
		std::vector<PrototypeArmorRecord> CollectQueuedArmorRecordsForDetachLocked(const LifecycleEvent& a_event);
		void MarkPendingActorRebuildLocked(RE::Actor* a_actor, bool a_firstPerson, std::vector<PrototypeArmorRecord> a_armorRecords = {}, bool a_forceArmorRescan = false, bool a_scheduleImmediately = true, bool a_replaceArmorRecords = false);
		void MarkPendingHeadRebuildLocked(const LifecycleEvent& a_event);
		void SchedulePendingRebuildTaskLocked();
		bool HasActiveOrPendingActorRebuildLocked(RE::Actor* a_actor);
		std::vector<PrototypeArmorRecord> CollectSuspendedArmorRecordsLocked(const LifecycleEvent& a_event);
		void RecordPrototypeArmorLocked(
			PrototypeActorState& a_state,
			RE::BIPED_OBJECT a_bipedObject,
			std::string a_physicsXmlPath,
			const DefaultBBP::NameMap& a_meshNameMap,
			RE::NiAVObject* a_attachedObject = nullptr,
			RE::NiAVObject* a_sourceObject = nullptr,
			RE::NiAVObject* a_mergeSourceObject = nullptr,
			std::vector<MergeRename> a_mergeRenameMap = {},
			std::uint64_t a_buildGroup = 0);
		PrototypeAttachmentRecord* FindPrototypeAttachmentLocked(PrototypeActorState& a_state, RE::BIPED_OBJECT a_bipedObject, RE::NiAVObject* a_object = nullptr, RE::NiAVObject* a_sourceObject = nullptr, std::string_view a_physicsXmlPath = {});
		const PrototypeAttachmentRecord* FindPrototypeAttachmentLocked(const PrototypeActorState& a_state, RE::BIPED_OBJECT a_bipedObject, RE::NiAVObject* a_object = nullptr, RE::NiAVObject* a_sourceObject = nullptr, std::string_view a_physicsXmlPath = {});
		bool IsPrototypeAttachmentCurrentLocked(const PrototypeActorState& a_state, RE::BIPED_OBJECT a_bipedObject, RE::NiAVObject* a_object, RE::NiAVObject* a_sourceObject, std::string_view a_physicsXmlPath);
		void RecordPrototypeAttachmentLocked(PrototypeActorState& a_state, RE::BIPED_OBJECT a_bipedObject, RE::NiAVObject* a_object, RE::NiAVObject* a_sourceObject, std::string a_physicsXmlPath, std::uint64_t a_buildGroup);
		bool RebuildPendingArmorRecordsLocked(RE::Actor* a_actor, PendingActorRebuild& a_pending);
		void TryRebuildPendingActorsLocked(RE::Actor* a_actor = nullptr);
		void TryRebuildPendingHeadsLocked();
		void SuspendPrototypeStatesForCustomizationMenuLocked();
		void ReloadPrototypeStatesForCustomizationMenuLocked();
		void SuspendPrototypeRuntimeLocked(PrototypeActorState& a_state);
		void SoftSuspendPrototypeRuntimeLocked(PrototypeActorState& a_state);
		bool ResumeSoftSuspendedPrototypeRuntimeLocked(PrototypeActorState& a_state);
		void ClearPrototypeStateLocked(PrototypeActorState& a_state, bool a_restoreSkinSlots = true);
		std::uint32_t RestoreSuspendedSkinSlotsLocked(PrototypeActorState& a_state, std::span<const std::uint64_t> a_buildGroups, std::span<const Fo4SkinnedMeshBone::ActiveSkinSlot> a_activeSlots = {});
		std::uint32_t RestoreAllSuspendedSkinSlotsLocked(PrototypeActorState& a_state);
		std::vector<std::uint64_t> CollectPrototypeGroupsForObjectLocked(const PrototypeActorState& a_state, RE::NiAVObject* a_object) const;
		bool ClearPrototypeGroupsForObjectLocked(PrototypeActorState& a_state, RE::NiAVObject* a_object);
		bool ClearPrototypeGroupsForBipedObjectLocked(PrototypeActorState& a_state, RE::BIPED_OBJECT a_bipedObject);
		bool ClearPrototypeGroupsForBoneNamesLocked(PrototypeActorState& a_state, std::span<const std::string> a_boneNames, PrototypeBuildDomain a_domain);
		bool ClearPrototypeGroupsByDomainLocked(PrototypeActorState& a_state, PrototypeBuildDomain a_domain);
		void ClearPrototypeGroupsLocked(PrototypeActorState& a_state, const std::vector<std::uint64_t>& a_buildGroups, bool a_detachMergedNodes = true);
		void ClearAllPrototypeStatesLocked();
		void ResumeFromLoadingMenuLocked();
		float PreparePrototypeActorForReadLocked(PrototypeActorState& a_state, float a_timeStep);
		bool PrototypeBuildGroupHasMeshLocked(const PrototypeActorState& a_state, std::uint64_t a_buildGroup) const;
		bool PrototypeBuildGroupHasBodyLocked(const PrototypeActorState& a_state, std::uint64_t a_buildGroup) const;
		bool PrototypeBuildGroupIsRecordableLocked(const PrototypeActorState& a_state, std::uint64_t a_buildGroup, PrototypeBuildDomain a_domain) const;
		void CommitPrototypeBuildGroupToBulletLocked(PrototypeActorState& a_state, std::uint64_t a_buildGroup);
		PrototypeBuildResult BuildPrototypeBodiesLocked(PrototypeActorState& a_state, const LifecycleEvent& a_event, const PhysicsXmlSummary& a_summary, const DefaultBBP::NameMap& a_meshNameMap, PrototypeBuildDomain a_domain);
		bool BuildPrototypeMeshesLocked(PrototypeActorState& a_state, const PhysicsXmlSummary& a_summary, const LifecycleEvent& a_event, const DefaultBBP::NameMap& a_meshNameMap, std::uint64_t a_buildGroup, PrototypeBuildDomain a_domain, std::span<PrototypeBody> a_stagedBodies, std::vector<PrototypeMesh>& a_stagedMeshes);
		void BuildPrototypeConstraintsLocked(PrototypeActorState& a_state, const PhysicsXmlSummary& a_summary, std::uint64_t a_buildGroup, PrototypeBuildDomain a_domain, std::span<PrototypeBody> a_stagedBodies, std::vector<PrototypeConstraint>& a_stagedConstraints);
		void LogPrototypeActorBulletObjectsLocked(const PrototypeActorState& a_state, std::string_view a_reason) const;
		void ResetPrototypeBuildGroupToCurrentPoseLocked(PrototypeActorState& a_state, std::uint64_t a_buildGroup, std::span<PrototypeBody> a_stagedBodies = {});
		std::uint32_t RefreshPrototypeBuildGroupMeshesLocked(PrototypeActorState& a_state, std::uint64_t a_buildGroup);
		std::uint32_t ResetPrototypeBuildGroupToReferencePoseLocked(PrototypeActorState& a_state, std::uint64_t a_buildGroup);
		std::uint32_t ResetPrototypeRuntimeToReferencePoseLocked(PrototypeActorState& a_state, std::string_view a_reason);
		void ScalePrototypeConstraintsLocked(PrototypeActorState& a_state);
		void ScalePrototypeConstraintsLocked(PrototypeActorState& a_state, const PrototypeBuildGroupRuntime& a_runtime);
		void LogRootConstraintDiagnosticsLocked(std::string_view a_phase, const PrototypeActorState& a_state);
		void UpdateMeshDisableStatesLocked(PrototypeActorState& a_state);
		void ResetStepClockLocked();
		void WaitForAsyncStep();
		void RunSecondStepLocked(float a_deltaSeconds, float a_fixedStepSeconds);

		std::mutex lock_;
		std::unique_ptr<AsyncStepState> asyncStepState_;
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
		float averageMainSyncMs_{ 0.0F };
		float averageStepReadMs_{ 0.0F };
		float averageStepWindMs_{ 0.0F };
		float averageStepBulletMs_{ 0.0F };
		float averageStepCollisionMs_{ 0.0F };
		float pendingWritebackMs_{ 0.0F };
		float pendingMainSyncMs_{ 0.0F };
		float pendingStepReadMs_{ 0.0F };
		float pendingStepWindMs_{ 0.0F };
		float pendingStepBulletMs_{ 0.0F };
		float pendingStepCollisionMs_{ 0.0F };
		std::uint32_t pendingStepCollisionCalls_{ 0 };
		std::uint32_t cellJobsWritebacks_{ 0 };
		std::uint32_t postAnimationWritebacks_{ 0 };
		std::uint32_t mainSyncWritebacks_{ 0 };
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
		bool pendingRebuildTaskQueued_{ false };
		bool registered_{ false };
	};
}
