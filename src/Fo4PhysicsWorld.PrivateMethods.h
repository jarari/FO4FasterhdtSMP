// Included inside Smp::Fo4PhysicsWorld private section.
// Split from Fo4PhysicsWorld.h: private method declarations.

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
		static void StripQueuedArmorRuntimePointers(std::vector<PrototypeArmorRecord>& a_records);
		static bool PrototypeArmorRecordsIncludeHairSlot(std::span<const PrototypeArmorRecord> a_records);
		static std::uint32_t PruneStalePendingHairSlotArmorRecords(std::vector<PrototypeArmorRecord>& a_records);
		PendingActorRebuild* FindPendingActorRebuildLocked(RE::Actor* a_actor, bool a_firstPerson);
		std::vector<PrototypeArmorRecord> CollectQueuedArmorRecordsForAttachLocked(const LifecycleEvent& a_event);
		std::vector<PrototypeArmorRecord> CollectQueuedArmorRecordsForDetachLocked(const LifecycleEvent& a_event);
		void MarkPendingActorRebuildLocked(RE::Actor* a_actor, bool a_firstPerson, std::vector<PrototypeArmorRecord> a_armorRecords = {}, bool a_forceArmorRescan = false, bool a_scheduleImmediately = true, bool a_replaceArmorRecords = false);
		void MarkPendingHeadRebuildLocked(const LifecycleEvent& a_event);
		bool HasActiveOrPendingActorRebuildLocked(RE::Actor* a_actor);
		bool SoftReloadPrototypeStateLocked(PrototypeActorState& a_state, LifecycleEventType a_reason);
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
		std::vector<std::uint64_t> CollectArmorPrototypeGroupsForBipedObjectLocked(const PrototypeActorState& a_state, RE::BIPED_OBJECT a_bipedObject, std::uint64_t a_preservedBuildGroup = 0) const;
		std::uint32_t PrunePrototypeRecordsForBipedObjectLocked(PrototypeActorState& a_state, RE::BIPED_OBJECT a_bipedObject, std::uint64_t a_preservedBuildGroup = 0);
		std::uint32_t ClearStaleHairSlotArmorGroupsLocked(PrototypeActorState& a_state, RE::BIPED_OBJECT a_bipedObject, std::uint64_t a_preservedBuildGroup, std::string_view a_reason, RE::NiAVObject* a_object = nullptr, std::string_view a_physicsXmlPath = {});
		std::uint32_t CollectHeadPartGroupsLocked(const PrototypeActorState& a_state, std::vector<std::uint64_t>& a_buildGroups) const;
		bool HasActiveHairSlotArmorLocked(const PrototypeActorState& a_state) const;
		bool PrototypeBuildGroupsIncludeHairSlotArmorLocked(const PrototypeActorState& a_state, std::span<const std::uint64_t> a_buildGroups) const;
		bool ClearPrototypeGroupsForObjectLocked(PrototypeActorState& a_state, RE::NiAVObject* a_object);
		bool ClearPrototypeGroupsForBipedObjectLocked(PrototypeActorState& a_state, RE::BIPED_OBJECT a_bipedObject);
		bool ClearPrototypeGroupsForBoneNamesLocked(PrototypeActorState& a_state, std::span<const std::string> a_boneNames, PrototypeBuildDomain a_domain);
		bool ClearPrototypeGroupsByDomainLocked(PrototypeActorState& a_state, PrototypeBuildDomain a_domain);
		void ClearHeadPrototypeTrackingLocked(PrototypeActorState& a_state, std::string_view a_reason);
		void ClearPrototypeGroupsLocked(PrototypeActorState& a_state, const std::vector<std::uint64_t>& a_buildGroups, bool a_detachMergedNodes = true);
		void ClearAllPrototypeStatesLocked();
		void ResumeFromLoadingMenuLocked();
		PrototypeReadPreparation PreparePrototypeActorForReadLocked(PrototypeActorState& a_state, float a_timeStep);
		bool PrototypeBuildGroupHasMeshLocked(const PrototypeActorState& a_state, std::uint64_t a_buildGroup) const;
		bool PrototypeBuildGroupHasBodyLocked(const PrototypeActorState& a_state, std::uint64_t a_buildGroup) const;
		bool PrototypeBuildGroupIsRecordableLocked(const PrototypeActorState& a_state, std::uint64_t a_buildGroup, PrototypeBuildDomain a_domain, RE::BIPED_OBJECT a_bipedObject = RE::BIPED_OBJECT::kTotal) const;
		void UpdatePrototypeBuildGroupMeshesLocked(PrototypeActorState& a_state, std::uint64_t a_buildGroup);
		void CommitPrototypeBuildGroupToBulletLocked(PrototypeActorState& a_state, std::uint64_t a_buildGroup);
		PrototypeBuildResult BuildPrototypeBodiesLocked(PrototypeActorState& a_state, const LifecycleEvent& a_event, const PhysicsXmlSummary& a_summary, const DefaultBBP::NameMap& a_meshNameMap, PrototypeBuildDomain a_domain, bool a_commitToBullet = true);
		bool BuildPrototypeMeshesLocked(PrototypeActorState& a_state, const PhysicsXmlSummary& a_summary, const LifecycleEvent& a_event, const DefaultBBP::NameMap& a_meshNameMap, std::uint64_t a_buildGroup, PrototypeBuildDomain a_domain, const std::unordered_set<RE::NiAVObject*>& a_trustedActorSkeletonNodes, const std::vector<RE::NiAVObject*>& a_mergedSkeletonNodes, std::vector<PrototypeBody>& a_stagedBodies, std::vector<PrototypeMesh>& a_stagedMeshes);
		void BuildPrototypeConstraintsLocked(PrototypeActorState& a_state, const PhysicsXmlSummary& a_summary, std::uint64_t a_buildGroup, PrototypeBuildDomain a_domain, std::span<PrototypeBody> a_stagedBodies, std::vector<PrototypeConstraint>& a_stagedConstraints);
		void LogPrototypeActorBulletObjectsLocked(const PrototypeActorState& a_state, std::string_view a_reason) const;
		void ResetPrototypeBuildGroupToCurrentPoseLocked(PrototypeActorState& a_state, std::uint64_t a_buildGroup, std::span<PrototypeBody> a_stagedBodies = {});
		void ResetPrototypeBuildGroupsToStoredLocalPoseLocked(PrototypeActorState& a_state, std::span<const std::uint64_t> a_buildGroups, std::string_view a_reason);
		void ScalePrototypeConstraintsLocked(PrototypeActorState& a_state);
		void ScalePrototypeConstraintsLocked(PrototypeActorState& a_state, const PrototypeBuildGroupRuntime& a_runtime);
		void LogRootConstraintDiagnosticsLocked(std::string_view a_phase, const PrototypeActorState& a_state);
		void UpdateMeshDisableStatesLocked(PrototypeActorState& a_state);
		void ResetStepClockLocked();
		void WaitForAsyncStep();
		void RunSecondStepLocked(float a_deltaSeconds, float a_fixedStepSeconds);
