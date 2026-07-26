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
			RE::BGSHeadPart* headPart{ nullptr };
			std::uint32_t frameDelay{ 0 };
			std::uint32_t cpuCopyRetryCount{ 0 };
		};
