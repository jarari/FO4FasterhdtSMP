// Included inside Smp::Fo4PhysicsWorld private section.
// Split from Fo4PhysicsWorld.h: private method declarations.

		Fo4SkinnedMeshSystem* FindSystemLocked(RE::Actor* a_actor, bool a_firstPerson);
		Fo4SkinnedMeshSystem& GetOrCreateSystemLocked(RE::Actor* a_actor, bool a_firstPerson);
		bool IsSystemValidLocked(Fo4SkinnedMeshSystem& a_state);
		bool IsActorWithinDistanceLocked(RE::Actor* a_actor) const;
		void PruneInvalidSystemsLocked();
		void EnforceActorBudgetLocked();
		void SuspendActorCandidateLocked(RE::Actor* a_actor, bool a_firstPerson, std::vector<ArmorPhysicsRecord> a_armorRecords = {});
		void TryReactivateSuspendedActorsLocked();
		void TryReactivateInactiveSystemsLocked();
		bool ShouldBuildSuspendedArmorCandidateLocked(const LifecycleEvent& a_event) const;
		void DeactivateBuiltSystemIfInactiveLocked(Fo4SkinnedMeshSystem& a_state, const LifecycleEvent& a_event);
		static void MergeArmorPhysicsRecord(std::vector<ArmorPhysicsRecord>& a_records, ArmorPhysicsRecord a_record);
		static void StripQueuedArmorRuntimePointers(std::vector<ArmorPhysicsRecord>& a_records);
		static std::uint32_t PruneStalePendingHairSlotArmorRecords(std::vector<ArmorPhysicsRecord>& a_records);
		PendingActorRebuild* FindPendingActorRebuildLocked(RE::Actor* a_actor, bool a_firstPerson);
		std::vector<ArmorPhysicsRecord> CollectQueuedArmorRecordsForAttachLocked(const LifecycleEvent& a_event);
		std::vector<ArmorPhysicsRecord> CollectQueuedArmorRecordsForDetachLocked(const LifecycleEvent& a_event);
		void MarkPendingActorRebuildLocked(RE::Actor* a_actor, bool a_firstPerson, std::vector<ArmorPhysicsRecord> a_armorRecords = {}, bool a_forceArmorRescan = false, bool a_scheduleImmediately = true, bool a_replaceArmorRecords = false);
		void MarkPendingHeadRebuildLocked(const LifecycleEvent& a_event);
		bool HasActiveOrPendingActorRebuildLocked(RE::Actor* a_actor);
		bool SoftReloadSystemLocked(Fo4SkinnedMeshSystem& a_state, LifecycleEventType a_reason);
		std::vector<ArmorPhysicsRecord> CollectSuspendedArmorRecordsLocked(const LifecycleEvent& a_event);
		void RecordArmorLocked(
			Fo4SkinnedMeshSystem& a_state,
			RE::BIPED_OBJECT a_bipedObject,
			std::string a_physicsXmlPath,
			const DefaultBBP::NameMap& a_meshNameMap,
			RE::NiAVObject* a_attachedObject = nullptr,
			RE::NiAVObject* a_sourceObject = nullptr,
			std::vector<ArmorBoneReference> a_armorBoneReferences = {},
			std::uint64_t a_buildGroup = 0);
		AttachmentPhysicsRecord* FindAttachmentLocked(Fo4SkinnedMeshSystem& a_state, RE::BIPED_OBJECT a_bipedObject, RE::NiAVObject* a_object = nullptr, RE::NiAVObject* a_sourceObject = nullptr, std::string_view a_physicsXmlPath = {});
		const AttachmentPhysicsRecord* FindAttachmentLocked(const Fo4SkinnedMeshSystem& a_state, RE::BIPED_OBJECT a_bipedObject, RE::NiAVObject* a_object = nullptr, RE::NiAVObject* a_sourceObject = nullptr, std::string_view a_physicsXmlPath = {});
		bool IsAttachmentCurrentLocked(const Fo4SkinnedMeshSystem& a_state, RE::BIPED_OBJECT a_bipedObject, RE::NiAVObject* a_object, RE::NiAVObject* a_sourceObject, std::string_view a_physicsXmlPath);
		void RecordAttachmentLocked(Fo4SkinnedMeshSystem& a_state, RE::BIPED_OBJECT a_bipedObject, RE::NiAVObject* a_object, RE::NiAVObject* a_sourceObject, std::string a_physicsXmlPath, std::uint64_t a_buildGroup);
		bool RebuildPendingArmorRecordsLocked(RE::Actor* a_actor, PendingActorRebuild& a_pending);
		void TryRebuildPendingActorsLocked(RE::Actor* a_actor = nullptr);
		void TryRebuildPendingHeadsLocked();
		bool NoteCharacterCustomizationTargetLocked(RE::Actor* a_actor);
		RE::Actor* ResolveCharacterCustomizationTargetLocked();
		bool IsCharacterCustomizationTargetLocked(RE::Actor* a_actor);
		bool ShouldDeferCharacterCustomizationPhysicsLocked(const LifecycleEvent& a_event);
		FaceGenRebuildGuard* FindFaceGenRebuildGuardLocked(RE::Actor* a_actor);
		void ObserveGuardedHeadEventLocked(const LifecycleEvent& a_event);
		void AdvanceFaceGenRebuildGuardsLocked();
		bool IsActorFaceGenLoadPendingLocked(RE::Actor* a_actor) const;
		void SuspendCharacterCustomizationTargetLocked();
		void ReloadCharacterCustomizationTargetLocked();
		void ClearCharacterCustomizationTargetLocked();
		void SuspendSystemLocked(Fo4SkinnedMeshSystem& a_state);
		void DeactivateSystemLocked(Fo4SkinnedMeshSystem& a_state);
		bool ReactivateSystemLocked(Fo4SkinnedMeshSystem& a_state);
		void ClearSystemLocked(Fo4SkinnedMeshSystem& a_state, bool a_restoreSkinSlots = true);
		std::uint32_t RestoreSuspendedSkinSlotsLocked(Fo4SkinnedMeshSystem& a_state, std::span<const std::uint64_t> a_buildGroups, std::span<const Fo4SkinnedMeshBone::ActiveSkinSlot> a_activeSlots = {});
		std::uint32_t RestoreAllSuspendedSkinSlotsLocked(Fo4SkinnedMeshSystem& a_state);
		std::vector<std::uint64_t> CollectBuildGroupsForObjectLocked(const Fo4SkinnedMeshSystem& a_state, RE::NiAVObject* a_object) const;
		std::vector<std::uint64_t> CollectArmorPhysicsGroupsForBipedObjectLocked(const Fo4SkinnedMeshSystem& a_state, RE::BIPED_OBJECT a_bipedObject, std::uint64_t a_preservedBuildGroup = 0) const;
		std::uint32_t PrunePhysicsRecordsForBipedObjectLocked(Fo4SkinnedMeshSystem& a_state, RE::BIPED_OBJECT a_bipedObject, std::uint64_t a_preservedBuildGroup = 0);
		std::uint32_t ClearStaleHairSlotArmorGroupsLocked(Fo4SkinnedMeshSystem& a_state, RE::BIPED_OBJECT a_bipedObject, std::uint64_t a_preservedBuildGroup, std::string_view a_reason, RE::NiAVObject* a_object = nullptr, std::string_view a_physicsXmlPath = {}, bool a_resetToStoredLocalPose = true);
		std::uint32_t CollectHeadPartGroupsLocked(const Fo4SkinnedMeshSystem& a_state, BuildDomain a_domain, std::vector<std::uint64_t>& a_buildGroups) const;
		bool BuildGroupsIncludeHairSlotArmorLocked(const Fo4SkinnedMeshSystem& a_state, std::span<const std::uint64_t> a_buildGroups) const;
		bool ClearBuildGroupsForObjectLocked(Fo4SkinnedMeshSystem& a_state, RE::NiAVObject* a_object);
		bool ClearBuildGroupsForBipedObjectLocked(Fo4SkinnedMeshSystem& a_state, RE::BIPED_OBJECT a_bipedObject);
		bool ClearBuildGroupsForBoneNamesLocked(Fo4SkinnedMeshSystem& a_state, std::span<const std::string> a_boneNames, BuildDomain a_domain);
		bool ClearBuildGroupsByDomainLocked(Fo4SkinnedMeshSystem& a_state, BuildDomain a_domain);
		void ClearHeadPhysicsTrackingLocked(Fo4SkinnedMeshSystem& a_state, std::string_view a_reason);
		void ClearBuildGroupsLocked(Fo4SkinnedMeshSystem& a_state, const std::vector<std::uint64_t>& a_buildGroups, bool a_resetToStoredLocalPose = true);
		void ClearAllSystemsLocked();
		void NoteSaveLoadActorLocked(const LifecycleEvent& a_event);
		void ResumeFromLoadingMenuLocked();
		bool BuildGroupHasMeshLocked(const Fo4SkinnedMeshSystem& a_state, std::uint64_t a_buildGroup) const;
		bool BuildGroupHasBodyLocked(const Fo4SkinnedMeshSystem& a_state, std::uint64_t a_buildGroup) const;
		bool BuildGroupIsRecordableLocked(const Fo4SkinnedMeshSystem& a_state, std::uint64_t a_buildGroup, BuildDomain a_domain, RE::BIPED_OBJECT a_bipedObject = RE::BIPED_OBJECT::kTotal) const;
		void UpdateBuildGroupMeshesLocked(Fo4SkinnedMeshSystem& a_state, std::uint64_t a_buildGroup);
		void ActivateBuildGroupLocked(Fo4SkinnedMeshSystem& a_state, std::uint64_t a_buildGroup);
		BuildResult BuildSystemObjectsLocked(Fo4SkinnedMeshSystem& a_state, const LifecycleEvent& a_event, const PhysicsXmlSummary& a_summary, const DefaultBBP::NameMap& a_meshNameMap, BuildDomain a_domain, bool a_commitToBullet = true, std::span<const RE::NiPointer<RE::NiAVObject>> a_meshSourceRoots = {}, std::uint64_t a_requestedBuildGroup = 0, std::string_view a_sourceKey = {});
		bool BuildMeshesLocked(Fo4SkinnedMeshSystem& a_state, const PhysicsXmlSummary& a_summary, const LifecycleEvent& a_event, const DefaultBBP::NameMap& a_meshNameMap, std::span<const RE::NiPointer<RE::NiAVObject>> a_meshSourceRoots, std::uint64_t a_buildGroup, std::string_view a_sourceKey, BuildDomain a_domain, std::vector<BoneRecord>& a_stagedBodies, std::vector<MeshRecord>& a_stagedMeshes);
		void BuildConstraintsLocked(Fo4SkinnedMeshSystem& a_state, const PhysicsXmlSummary& a_summary, std::uint64_t a_buildGroup, std::string_view a_sourceKey, BuildDomain a_domain, std::span<BoneRecord> a_stagedBodies, std::vector<ConstraintRecord>& a_stagedConstraints);
		void LogSystemBulletObjectsLocked(const Fo4SkinnedMeshSystem& a_state, std::string_view a_reason) const;
		void ResetBuildGroupToCurrentPoseLocked(Fo4SkinnedMeshSystem& a_state, std::uint64_t a_buildGroup, std::span<BoneRecord> a_stagedBodies = {});
		void ResetBuildGroupsToStoredLocalPoseLocked(Fo4SkinnedMeshSystem& a_state, std::span<const std::uint64_t> a_buildGroups, std::string_view a_reason);
		void LogRootConstraintDiagnosticsLocked(std::string_view a_phase, const Fo4SkinnedMeshSystem& a_state);
		void UpdateMeshDisableStatesLocked(Fo4SkinnedMeshSystem& a_state);
		void ResetStepClockLocked();
		void WaitForAsyncStep();
		void RunSecondStepLocked(float a_deltaSeconds, float a_fixedStepSeconds);
