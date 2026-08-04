// Included inside Smp::Fo4PhysicsWorld private section.
// Split from Fo4PhysicsWorld.h: event queues owned by the world.

		struct SuspendedActorCandidate
		{
			RE::ActorHandle actorHandle;
			bool firstPerson{ false };
			std::vector<ArmorPhysicsRecord> armorRecords;
		};

		struct PendingActorRebuild
		{
			RE::ActorHandle actorHandle;
			bool firstPerson{ false };
			std::vector<ArmorPhysicsRecord> armorRecords;
			std::uint32_t frameDelay{ 0 };
			bool forceArmorRescan{ false };
		};

		struct PendingHeadRebuild
		{
			RE::ActorHandle actorHandle;
			LifecycleEventType type{ LifecycleEventType::kActorHeadInitialized };
			RE::NiPointer<RE::NiAVObject> object;
			std::uint32_t frameDelay{ 0 };
			std::uint32_t cpuCopyRetryCount{ 0 };
		};

		struct FaceGenRebuildGuard
		{
			RE::ActorHandle actorHandle;
			RE::NiPointer<RE::NiAVObject> latestFaceNode;
			std::uint64_t lastManagerObservation{ 0 };
			std::uint32_t stableIdleObservations{ 0 };
		};

		struct SaveLoadArmorRecord
		{
			RE::BipedAnim* bipedIdentity{ nullptr };
			RE::NiAVObject* objectIdentity{ nullptr };
			ArmorPhysicsRecord record;
		};

		struct SaveLoadActorCandidate
		{
			RE::ActorHandle actorHandle;
			bool firstPerson{ false };
			bool rebuildArmor{ false };
			bool rebuildHead{ false };
			std::vector<SaveLoadArmorRecord> armorRecords;
		};

		struct PendingSkeletonTransition
		{
			RE::ActorHandle actorHandle;
			RE::NiPointer<RE::NiAVObject> oldRoot;
			RE::NiPointer<RE::NiAVObject> retainedFace;
			RE::NiPointer<RE::NiAVObject> newRoot;
			RE::NiPointer<RE::BSFlattenedBoneTree> skeletonRoot;
			std::vector<ArmorBoneReference> headBoneReferences;
			std::vector<std::string> requiredHeadBoneNames;
			std::vector<RetainedSkinBinding> retainedSkinBindings;
			std::uint32_t frameAge{ 0 };
			std::uint32_t loadedFrameAge{ 0 };
			bool armorRebuildQueued{ false };
			bool faceSkinned{ false };
		};

		struct RetainedHeadSkeletonCache
		{
			RE::ActorHandle actorHandle;
			std::uintptr_t retainedFaceIdentity{ 0 };
			std::vector<ArmorBoneReference> headBoneReferences;
			std::vector<std::string> requiredHeadBoneNames;
		};
