#pragma once

#include "ArmorBoneReference.h"
#include "DefaultBBP.h"
#include "Fo4SkinnedMeshBone.h"
#include "RE/N/NiTransform.h"
#include "hdtSkinnedMesh/hdtBoneScaleConstraint.h"
#include "hdtSkinnedMesh/hdtSkinnedMeshBody.h"
#include "hdtSkinnedMesh/hdtSkinnedMeshSystem.h"

#include <btBulletDynamicsCommon.h>

#include <algorithm>
#include <memory>
#include <ranges>
#include <string>
#include <utility>
#include <vector>

namespace Smp
{
	enum class PhysicsConstraintKind;

	enum class WritebackSource
	{
		kUnknown,
		kMainSync,
		kCellJobs,
		kPostAnimationGraph
	};

	enum class BuildDomain
	{
		kArmor,
		kHead,
		kHair
	};

	template <class T>
	class SystemObjectRef
	{
	public:
		// Build records own staged objects. On commit, ownership moves to the
		// SkinnedMeshSystem while this stable pointer remains as FO4 metadata.
		SystemObjectRef() = default;
		SystemObjectRef(const SystemObjectRef&) = delete;
		SystemObjectRef& operator=(const SystemObjectRef&) = delete;

		SystemObjectRef(SystemObjectRef&& a_other) noexcept :
			owner_(std::move(a_other.owner_)),
			object_(std::exchange(a_other.object_, nullptr))
		{}

		SystemObjectRef& operator=(SystemObjectRef&& a_other) noexcept
		{
			if (this != std::addressof(a_other)) {
				owner_ = std::move(a_other.owner_);
				object_ = std::exchange(a_other.object_, nullptr);
			}
			return *this;
		}

		SystemObjectRef& operator=(RE::BSTSmartPointer<T> a_object)
		{
			owner_ = std::move(a_object);
			object_ = owner_.get();
			return *this;
		}

		[[nodiscard]] T* get() const { return object_; }
		[[nodiscard]] T* operator->() const { return object_; }
		[[nodiscard]] explicit operator bool() const { return object_ != nullptr; }

		[[nodiscard]] RE::BSTSmartPointer<T> TakeOwnership()
		{
			return std::move(owner_);
		}

	private:
		RE::BSTSmartPointer<T> owner_;
		T* object_{ nullptr };
	};

	struct BoneRecord
	{
		RE::Actor* actor{ nullptr };
		RE::NiNode* node{ nullptr };
		std::uint64_t buildGroup{ 0 };
		RE::BIPED_OBJECT bipedObject{ RE::BIPED_OBJECT::kTotal };
		std::vector<std::uint64_t> buildGroups;
		std::vector<std::pair<std::uint64_t, BuildDomain>> buildGroupDomains;
		std::vector<std::pair<std::uint64_t, RE::BIPED_OBJECT>> buildGroupBipedObjects;
		std::string boneName;
		std::unique_ptr<btCollisionShape> shape;
		std::unique_ptr<btDefaultMotionState> motionState;
		SystemObjectRef<Fo4SkinnedMeshBone> bone;
		bool meshOnlySkinBone{ false };
	};

	struct ConstraintRecord
	{
		std::uint64_t buildGroup{ 0 };
		BuildDomain domain{ BuildDomain::kArmor };
		std::string sourceKey;
		std::size_t descriptorIndex{ 0 };
		std::string bodyA;
		std::string bodyB;
		PhysicsConstraintKind kind{};
		Fo4SkinnedMeshBone* boneA{ nullptr };
		Fo4SkinnedMeshBone* boneB{ nullptr };
		SystemObjectRef<hdt::BoneScaleConstraint> constraint;

		[[nodiscard]] btTypedConstraint* GetConstraint() const
		{
			return constraint ? constraint->getConstraint() : nullptr;
		}
	};

	struct MeshRecord
	{
		std::string name;
		RE::BSGeometry* geometry{ nullptr };
		std::uint64_t buildGroup{ 0 };
		std::string sourceKey;
		std::size_t descriptorIndex{ 0 };
		RE::BIPED_OBJECT bipedObject{ RE::BIPED_OBJECT::kTotal };
		BuildDomain domain{ BuildDomain::kArmor };
		SystemObjectRef<hdt::SkinnedMeshBody> body;
	};

	struct AttachmentBoneLocalPose
	{
		std::uint64_t buildGroup{ 0 };
		RE::NiPointer<RE::NiAVObject> node;
		RE::NiTransform local{ RE::NiTransform::IDENTITY };
	};

	struct ArmorPhysicsRecord
	{
		RE::BIPED_OBJECT bipedObject{ RE::BIPED_OBJECT::kTotal };
		std::string physicsXmlPath;
		DefaultBBP::NameMap meshNameMap;
		RE::NiPointer<RE::NiAVObject> attachedObject;
		RE::NiPointer<RE::NiAVObject> sourceObject;
		std::vector<ArmorBoneReference> armorBoneReferences;
		std::vector<std::uint64_t> buildGroups;
		std::uint32_t cpuCopyRetryCount{ 0 };
		bool preserveCurrentPose{ false };
	};

	struct AttachmentPhysicsRecord
	{
		RE::BIPED_OBJECT bipedObject{ RE::BIPED_OBJECT::kTotal };
		std::uint32_t generation{ 0 };
		std::string physicsXmlPath;
		RE::NiPointer<RE::NiAVObject> attachedObject;
		RE::NiPointer<RE::NiAVObject> sourceObject;
		std::vector<std::uint64_t> buildGroups;
	};

	struct HeadPartPhysicsRecord
	{
		BuildDomain domain{ BuildDomain::kHead };
		std::string physicsXmlPath;
		RE::NiPointer<RE::NiAVObject> object;
		RE::NiPointer<RE::NiAVObject> sourceObject;
		RE::NiPointer<RE::NiAVObject> sourceRoot;
		std::vector<ArmorBoneReference> boneReferences;
		std::vector<std::string> requiredBoneNames;
		std::uint64_t buildGroup{ 0 };
		bool isHeadPartClosure{ false };
	};

	struct BuildResult
	{
		std::uint64_t buildGroup{ 0 };
		bool cpuCopyPending{ false };
		bool committed{ false };
		bool recordable{ false };
		bool succeeded{ false };
	};

	struct BuildGroupRecord
	{
		std::uint64_t buildGroup{ 0 };
		BuildDomain domain{ BuildDomain::kArmor };
		RE::BIPED_OBJECT bipedObject{ RE::BIPED_OBJECT::kTotal };
		bool pendingResetPhysicsRead{ false };
		bool pendingResetPhysicsWriteback{ false };
	};

	struct ReadPreparation
	{
		float timeStep{ 0.0F };
		RE::NiNode* restoreRoot{ nullptr };
		RE::NiTransform restoreWorld{ RE::NiTransform::IDENTITY };
	};

	class Fo4SkinnedMeshSystem :
		public hdt::SkinnedMeshSystem
	{
	public:
		// FO4 attachments can share bones materialized under the actor's live Root.
		// One actor system keeps each shared rigid body registered exactly once;
		// build-group records remain removal/rebuild metadata only.
		~Fo4SkinnedMeshSystem() override;
		float prepareForRead(float a_timeStep) override;
		[[nodiscard]] const ReadPreparation& GetReadPreparation() const { return readPreparation_; }

		bool AddBone(SystemObjectRef<Fo4SkinnedMeshBone>& a_bone);
		bool AddMesh(SystemObjectRef<hdt::SkinnedMeshBody>& a_mesh);
		bool AddConstraint(SystemObjectRef<hdt::BoneScaleConstraint>& a_constraint);
		[[nodiscard]] bool ContainsBone(const Fo4SkinnedMeshBone* a_bone) const;
		[[nodiscard]] bool ContainsMesh(const hdt::SkinnedMeshBody* a_mesh) const;
		[[nodiscard]] bool ContainsConstraint(const hdt::BoneScaleConstraint* a_constraint) const;
		[[nodiscard]] RE::BSTSmartPointer<Fo4SkinnedMeshBone> ReleaseBone(Fo4SkinnedMeshBone* a_bone);
		[[nodiscard]] RE::BSTSmartPointer<hdt::SkinnedMeshBody> ReleaseMesh(hdt::SkinnedMeshBody* a_mesh);
		[[nodiscard]] RE::BSTSmartPointer<hdt::BoneScaleConstraint> ReleaseConstraint(hdt::BoneScaleConstraint* a_constraint);
		void ClearSystemObjects();
		[[nodiscard]] bool HasPhysics() const;
		[[nodiscard]] bool IsActive() const;
		[[nodiscard]] bool IsInactive() const;

		RE::Actor* actor{ nullptr };
		RE::ActorHandle actorHandle;
		bool firstPerson{ false };
		RE::NiPointer<RE::NiAVObject> lastReadRoot;
		bool readInitialized{ false };
		btQuaternion lastRootRotation{ btQuaternion::getIdentity() };
		bool lastRootRotationInitialized{ false };
		bool clampRotations{ true };
		float rotationSpeedLimit{ 10.0F };
		bool unclampedResets{ true };
		float unclampedResetAngle{ 130.0F };
		std::uint64_t nextBuildGroup{ 0 };
		std::uint32_t nextAttachmentGeneration{ 0 };
		std::uint64_t lastWritebackFrame{ 0 };
		WritebackSource lastWritebackSource{ WritebackSource::kUnknown };
		float currentWindFactor{ 1.0F };
		bool suspended{ false };
		RE::NiPointer<RE::NiAVObject> faceNode;
		std::vector<BoneRecord> bodies;
		std::vector<MeshRecord> meshes;
		std::vector<ConstraintRecord> constraints;
		std::vector<AttachmentBoneLocalPose> attachmentBoneLocalPoses;
		std::vector<ArmorPhysicsRecord> armorRecords;
		std::vector<AttachmentPhysicsRecord> attachmentRecords;
		std::vector<HeadPartPhysicsRecord> headPartRecords;
		std::vector<BuildGroupRecord> buildGroups;
		std::vector<Fo4SkinnedMeshBone::SkinSlotRestore> suspendedSkinSlots;

	private:
		ReadPreparation readPreparation_;
	};
}
