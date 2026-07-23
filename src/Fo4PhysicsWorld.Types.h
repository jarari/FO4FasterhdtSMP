// Included inside Smp::Fo4PhysicsWorld private section.
// Split from Fo4PhysicsWorld.h: nested runtime/build data types.

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
			bool meshOnlySkinBone{ false };
			bool inBulletWorld{ false };
		};

		struct PrototypeConstraint
		{
			std::uint64_t buildGroup{ 0 };
			PrototypeBuildDomain domain{ PrototypeBuildDomain::kArmor };
			std::size_t descriptorIndex{ 0 };
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
			std::size_t descriptorIndex{ 0 };
			RE::BIPED_OBJECT bipedObject{ RE::BIPED_OBJECT::kTotal };
			PrototypeBuildDomain domain{ PrototypeBuildDomain::kArmor };
			RE::BSTSmartPointer<hdt::SkinnedMeshBody> body;
			bool inBulletWorld{ false };
		};

		struct PrototypeAttachmentBoneLocalPose
		{
			std::uint64_t buildGroup{ 0 };
			RE::NiPointer<RE::NiAVObject> node;
			RE::NiTransform local{ RE::NiTransform::IDENTITY };
		};

		struct PrototypeArmorRecord
		{
			RE::BIPED_OBJECT bipedObject{ RE::BIPED_OBJECT::kTotal };
			std::string physicsXmlPath;
			DefaultBBP::NameMap meshNameMap;
			RE::NiPointer<RE::NiAVObject> attachedObject;
			RE::NiPointer<RE::NiAVObject> sourceObject;
			std::vector<ArmorBoneReference> armorBoneReferences;
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

		struct PrototypeHeadPartRecord
		{
			PrototypeBuildDomain domain{ PrototypeBuildDomain::kHead };
			std::string physicsXmlPath;
			RE::NiPointer<RE::NiAVObject> object;
			RE::NiPointer<RE::NiAVObject> sourceObject;
			RE::NiPointer<RE::NiAVObject> sourceRoot;
			std::uint64_t buildGroup{ 0 };
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
			bool pendingResetPhysicsRead{ false };
		};

		struct PrototypeReadPreparation
		{
			float timeStep{ 0.0F };
			RE::NiNode* restoreRoot{ nullptr };
			RE::NiTransform restoreWorld{ RE::NiTransform::IDENTITY };
		};

		struct PrototypeActorState
		{
			PrototypeActorState() = default;
			PrototypeActorState(const PrototypeActorState&) = delete;
			PrototypeActorState& operator=(const PrototypeActorState&) = delete;
			PrototypeActorState(PrototypeActorState&&) noexcept = default;
			PrototypeActorState& operator=(PrototypeActorState&&) noexcept = default;

			RE::Actor* actor{ nullptr };
			RE::ActorHandle actorHandle;
			bool firstPerson{ false };
			RE::NiPointer<RE::NiAVObject> lastReadRoot;
			bool readInitialized{ false };
			btQuaternion lastRootRotation{ btQuaternion::getIdentity() };
			bool lastRootRotationInitialized{ false };
			std::uint64_t nextBuildGroup{ 0 };
			std::uint32_t nextAttachmentGeneration{ 0 };
			std::uint64_t lastWritebackFrame{ 0 };
			WritebackSource lastWritebackSource{ WritebackSource::kUnknown };
			float currentWindFactor{ 1.0F };
			bool runtimeSuspended{ false };
			bool runtimeSoftSuspended{ false };
			RE::NiPointer<RE::NiAVObject> faceNode;
			std::vector<PrototypeBody> bodies;
			std::vector<PrototypeMesh> meshes;
			std::vector<PrototypeConstraint> constraints;
			std::vector<PrototypeAttachmentBoneLocalPose> attachmentBoneLocalPoses;
			std::vector<PrototypeArmorRecord> armorRecords;
			std::vector<PrototypeAttachmentRecord> attachmentRecords;
			std::vector<PrototypeHeadPartRecord> headPartRecords;
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
