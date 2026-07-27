#pragma once

#include "Fo4SkinnedMeshSystem.h"
#include "LifecycleEvents.h"
#include "hdtSkinnedMesh/hdtSkinnedMeshWorld.h"

#include <btBulletDynamicsCommon.h>

#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

class btBroadphaseInterface;
class btCollisionDispatcher;
class btCollisionShape;
class btConstraintSolverPoolMt;
class btDefaultCollisionConfiguration;
struct btDefaultMotionState;
class btDiscreteDynamicsWorld;
class btRigidBody;
class btTypedConstraint;

namespace RE
{
	class MenuOpenCloseEvent;
	class NiAVObject;
	class TESObjectARMA;
}

namespace hdt
{
	class BoneScaleConstraint;
	class CollisionDispatcher;
	class SkinnedMeshBody;
}

namespace Smp
{
	class Fo4SkinnedMeshBone;
	enum class PhysicsConstraintKind;
	struct PhysicsXmlSummary;
	struct RuntimeSettings;

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
		void ResetSystems();
		void SetDisabled(bool a_disabled);
		[[nodiscard]] bool IsDisabled();
		void SetProfilerCapture(bool a_enabled, std::uint64_t a_sampleFrames = 240, std::uint64_t a_printFrames = 240);
		void PrintConsoleDetails(bool a_includeItems);
		void PrintConsoleSummary();
		void StepFrame();
		void Step(float a_deltaSeconds);
		void UpdateWindLocked();
		void ApplyWindForcesLocked();
		void RecordFrameMetrics(float a_stepMs);
		void RecordWritebackMetric(float a_writebackMs, WritebackSource a_source, bool a_wroteAny, bool a_skippedDuplicate);
		void WriteBackSystems(WritebackSource a_source = WritebackSource::kUnknown);
		void WriteBackSystems(RE::Actor* a_actor, WritebackSource a_source = WritebackSource::kUnknown);
		void ProcessPendingRebuilds();
		void DrawBulletVisualization();
		void NoteCharacterCustomizationTarget(RE::Actor* a_actor);
		void PrepareActor3DModelUpdate(RE::Actor* a_actor, std::uint16_t a_updateFlags);
		void QueueActor3DModelUpdateCompletion(RE::Actor* a_actor, std::uint16_t a_updateFlags);
		bool ReloadPhysicsFile(RE::Actor* a_actor, RE::TESObjectARMA* a_armorAddon, std::string_view a_physicsFilePath, bool a_persist, bool a_verbose);
		bool SwapPhysicsFile(RE::Actor* a_actor, std::string_view a_oldPhysicsFilePath, std::string_view a_newPhysicsFilePath, bool a_persist, bool a_verbose);
		[[nodiscard]] std::string QueryCurrentPhysicsFile(RE::Actor* a_actor, RE::TESObjectARMA* a_armorAddon, bool a_verbose);
		[[nodiscard]] std::vector<bool> TogglePhysics(RE::Actor* a_actor, std::span<const std::string> a_boneNames, bool a_on);
		void ResetActorPhysics(RE::Actor* a_actor, bool a_full);

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
		void BuildForEventLocked(const LifecycleEvent& a_event);
		bool FinalizeHeadHierarchyForEventLocked(const LifecycleEvent& a_event);
		bool BuildHeadForEventLocked(const LifecycleEvent& a_event);
		bool IsBuildCandidateLocked(const LifecycleEvent& a_event, bool a_requireObject);
		void CompletePendingActor3DModelUpdates();
		void CompleteActor3DModelUpdate(RE::Actor* a_actor, std::uint16_t a_updateFlags);

		#include "Fo4PhysicsWorld.Types.h"
		#include "Fo4PhysicsWorld.PrivateMethods.h"
		#include "Fo4PhysicsWorld.State.h"
	};
}
