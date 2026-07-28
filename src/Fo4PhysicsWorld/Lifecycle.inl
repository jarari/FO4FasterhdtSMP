// Included by ../Fo4PhysicsWorld.cpp.
// Split from Fo4PhysicsWorld.cpp. Do not compile this file separately.

namespace Smp
{
	bool Fo4PhysicsWorld::InitializeLocked()
	{
		if (dynamicsWorld_) {
			return true;
		}

		collisionConfiguration_ = std::make_unique<btDefaultCollisionConfiguration>();
		dispatcher_ = std::make_unique<hdt::CollisionDispatcher>(collisionConfiguration_.get());
		broadphase_ = std::make_unique<btDbvtBroadphase>();
		solver_ = std::make_unique<btConstraintSolverPoolMt>(InitializeBulletTaskSchedulerAndGetThreadCount());
		dynamicsWorld_ = std::make_unique<Fo4SkinnedMeshWorld>(
			dispatcher_.get(),
			broadphase_.get(),
			solver_.get(),
			nullptr,
			collisionConfiguration_.get());
		dynamicsWorld_->setGravity(btVector3(0.0F, 0.0F, kGravityAcceleration));
		auto& solverInfo = dynamicsWorld_->getSolverInfo();
		solverInfo.m_numIterations = solverIterations_;
		solverInfo.m_erp = solverErp_;
		solverInfo.m_friction = 0.0F;
		solverInfo.m_splitImpulse = true;
		solverInfo.m_splitImpulsePenetrationThreshold = -0.01F;
		solverInfo.m_erp2 = 0.15F;
		solverInfo.m_globalCfm = 0.001F;
		solverInfo.m_restitutionVelocityThreshold = 0.2F;
		solverInfo.m_solverMode = SOLVER_SIMD;
		solverInfo.m_leastSquaresResidualThreshold = 0.0001F;

		spdlog::info("initialized FO4 Faster HDT-SMP Bullet physics world");
		return true;
	}

	void Fo4PhysicsWorld::ResetLocked()
	{
		ClearAllSystemsLocked();

		if (dynamicsWorld_) {
			for (auto index = dynamicsWorld_->getNumCollisionObjects() - 1; index >= 0; --index) {
				const auto object = dynamicsWorld_->getCollisionObjectArray()[index];
				dynamicsWorld_->removeCollisionObject(object);
			}
		}

		dynamicsWorld_.reset();
		solver_.reset();
		broadphase_.reset();
		dispatcher_.reset();
		collisionConfiguration_.reset();
		suspendedActors_.clear();
		pendingActorRebuilds_.clear();
		pendingHeadRebuilds_.clear();
		pendingSkeletonTransitions_.clear();
		retainedHeadSkeletonCaches_.clear();
		saveLoadActors_.clear();
		loadingMenuDepth_ = 0;
		loadingPhysicsSuspended_ = false;
		saveLoadInProgress_ = false;
		candidateEvents_ = 0;
		simulationFrame_ = 1;
		currentMaxActiveActors_ = maxActiveActors_;
		ResetStepClockLocked();
		metricFrameCounter_ = 0;
		averageStepMs_ = 0.0F;
		averageWritebackMs_ = 0.0F;
		averageMainSyncMs_ = 0.0F;
		averageStepReadMs_ = 0.0F;
		averageStepWindMs_ = 0.0F;
		averageStepBulletMs_ = 0.0F;
		averageStepCollisionMs_ = 0.0F;
		pendingWritebackMs_ = 0.0F;
		pendingMainSyncMs_ = 0.0F;
		pendingStepReadMs_ = 0.0F;
		pendingStepWindMs_ = 0.0F;
		pendingStepBulletMs_ = 0.0F;
		pendingStepCollisionMs_ = 0.0F;
		pendingStepCollisionCalls_ = 0;
		mainSyncWritebacks_ = 0;
		cellJobsWritebacks_ = 0;
		postAnimationWritebacks_ = 0;
		duplicateCellJobsWritebacks_ = 0;
		duplicatePostAnimationWritebacks_ = 0;
		currentWind_.setZero();
		targetWind_.setZero();
		windTime_ = 0.0F;
		windWeatherCooldown_ = 0.0F;
		characterCustomizationMenuDepth_ = 0;
		ClearCharacterCustomizationTargetLocked();
	}

	void Fo4PhysicsWorld::NoteLifecycleCandidate(const LifecycleEvent& a_event)
	{
		std::scoped_lock lock(lock_);
		if (!InitializeLocked()) {
			return;
		}

		if ((a_event.type == LifecycleEventType::kActorLoad3D || a_event.type == LifecycleEventType::kActorSet3D) && a_event.actor) {
			if (const auto* actorState = FindSystemLocked(a_event.actor, a_event.firstPerson);
				actorState && actorState->IsActive()) {
				spdlog::debug(
					"skipping generic {} rebuild for actor={} firstPerson={} because direct armor physics is already active",
					ToString(a_event.type),
					static_cast<void*>(a_event.actor),
					a_event.firstPerson);
				return;
			}
		}

		PruneInvalidSystemsLocked();

		const auto armorAttach = IsArmorAttachCandidate(a_event.type);
		const auto actorArmorAttach = armorAttach && a_event.actor && !a_event.firstPerson;
		const auto actorSkeletonTransitionPending =
			a_event.actor &&
			std::ranges::any_of(pendingSkeletonTransitions_, [&a_event](const PendingSkeletonTransition& a_pending) {
				const auto actor = a_pending.actorHandle.get();
				return actor && actor.get() == a_event.actor;
			});
		if (actorSkeletonTransitionPending) {
			if (actorArmorAttach) {
				auto armorRecords = CollectQueuedArmorRecordsForAttachLocked(a_event);
				MarkPendingActorRebuildLocked(
					a_event.actor,
					a_event.firstPerson,
					std::move(armorRecords),
					false,
					false);
			}
			spdlog::debug(
				"deferred system physics attach candidate {} until actor skeleton transition completion actor={} object={}",
				ToString(a_event.type),
				static_cast<void*>(a_event.actor),
				static_cast<void*>(a_event.object));
			return;
		}
		if (actorArmorAttach && loadingPhysicsSuspended_) {
			auto armorRecords = CollectQueuedArmorRecordsForAttachLocked(a_event);
			const auto queuedRecords = armorRecords.size();
			MarkPendingActorRebuildLocked(a_event.actor, a_event.firstPerson, std::move(armorRecords), false, false);
			ResetStepClockLocked();
			spdlog::debug(
				"queued scoped armor system physics resume for loading-screen attach {} actor={} object={} armorRecords={}",
				ToString(a_event.type),
				static_cast<void*>(a_event.actor),
				static_cast<void*>(a_event.object),
				queuedRecords);
			return;
		}

		if (ShouldDeferCharacterCustomizationPhysicsLocked(a_event)) {
			if (actorArmorAttach) {
				spdlog::debug(
					"deferred scoped armor system physics build for customization target attach {} actor={} object={}",
					ToString(a_event.type),
					static_cast<void*>(a_event.actor),
					static_cast<void*>(a_event.object));
			} else {
				spdlog::debug(
					"deferred system physics attach candidate {} for customization target actor={} object={}",
					ToString(a_event.type),
					static_cast<void*>(a_event.actor),
					static_cast<void*>(a_event.object));
			}
			return;
		}

		++candidateEvents_;
		const auto candidateLogLevel =
			armorAttach && a_event.physicsXmlPath.empty() ?
				spdlog::level::trace :
				spdlog::level::debug;
		spdlog::log(
			candidateLogLevel,
			"physics world observed attach candidate #{} {} actor={} object={}",
			candidateEvents_,
			ToString(a_event.type),
			static_cast<void*>(a_event.actor),
			static_cast<void*>(a_event.object));

		if (!IsBuildCandidateLocked(a_event, true)) {
			auto armorRecords = CollectSuspendedArmorRecordsLocked(a_event);
			if (!armorRecords.empty()) {
				spdlog::debug(
					"captured {} suspended armor records for skipped system physics candidate {} actor={}",
					armorRecords.size(),
					ToString(a_event.type),
					static_cast<void*>(a_event.actor));
				SuspendActorCandidateLocked(a_event.actor, a_event.firstPerson, std::move(armorRecords));
			}
			return;
		}

		BuildForEventLocked(a_event);
		if ((a_event.type == LifecycleEventType::kActorLoad3D || a_event.type == LifecycleEventType::kActorSet3D) && a_event.actor) {
			const auto* actorState = FindSystemLocked(a_event.actor, a_event.firstPerson);
			if (!actorState || !actorState->IsActive()) {
				auto armorRecords = actorState ? actorState->armorRecords : std::vector<ArmorPhysicsRecord>{};
				if (armorRecords.empty()) {
					armorRecords = CollectSuspendedArmorRecordsLocked(a_event);
				}
				if (!armorRecords.empty()) {
					MarkPendingActorRebuildLocked(a_event.actor, a_event.firstPerson, std::move(armorRecords));
					spdlog::debug(
						"queued retry for {} actor={} firstPerson={} because initial generic actor build found no active runtime",
						ToString(a_event.type),
						static_cast<void*>(a_event.actor),
						a_event.firstPerson);
				} else {
					spdlog::debug(
						"skipping pending retry for {} actor={} firstPerson={} because no SMP armor records are tracked",
						ToString(a_event.type),
						static_cast<void*>(a_event.actor),
						a_event.firstPerson);
				}
			}
		}
	}

	void Fo4PhysicsWorld::BuildForEventLocked(const LifecycleEvent& a_event)
	{
		auto* loader = PhysicsXmlLoader::GetSingleton();
		const auto armorAttach = IsArmorAttachCandidate(a_event.type);

		const auto buildSelection = [&](const ArmorPhysicsXmlBuildCandidate& a_candidate) {
			auto* a_object = a_candidate.object;
			const auto& a_selection = a_candidate.selection;
			auto selectedXml = a_selection.path.string();
			if (a_event.actor) {
				if (auto overridePath = Papyrus::ResolvePhysicsFileOverride(a_event.actor->GetFormID(), selectedXml)) {
					spdlog::debug(
						"applying persistent DynamicHDT physics-file override actor={:08X} original='{}' override='{}'",
						a_event.actor->GetFormID(),
						selectedXml,
						*overridePath);
					selectedXml = std::move(*overridePath);
				}
			}
			if (!a_object || selectedXml.empty()) {
				return false;
			}

			spdlog::info(
				"loading system physics XML {} for actor={} object={}",
				selectedXml,
				static_cast<void*>(a_event.actor),
				static_cast<void*>(a_object));
			const auto selectedSummary = loader->LoadSummary(selectedXml);
			if (!selectedSummary) {
				spdlog::warn("skipping system physics candidate because selected XML failed to load: {}", selectedXml);
				return false;
			}

			auto scopedEvent = a_event;
			scopedEvent.object = a_object;
			scopedEvent.physicsXmlPath = selectedXml;
			if (a_candidate.sourceObject) {
				scopedEvent.sourceObject = a_candidate.sourceObject;
			}
			if (a_candidate.sourceRoot) {
				scopedEvent.sourceRoot = a_candidate.sourceRoot;
			}
			if (a_candidate.bipObject) {
				scopedEvent.bipObject = a_candidate.bipObject;
			}
			if (a_candidate.bipedObject != RE::BIPED_OBJECT::kTotal) {
				scopedEvent.bipedObject = a_candidate.bipedObject;
			}
			if (!scopedEvent.biped) {
				scopedEvent.biped = ResolveEventBiped(scopedEvent);
			}
			auto& actorState = GetOrCreateSystemLocked(a_event.actor, a_event.firstPerson);
			std::vector<std::uint64_t> staleArmorBuildGroups;
			const auto scopedArmorBuild = armorAttach || scopedEvent.bipedObject != RE::BIPED_OBJECT::kTotal;
			const auto hairSlotArmorBuild = scopedArmorBuild && IsHairBipedObject(scopedEvent.bipedObject);
			if (scopedArmorBuild) {
				if (IsAttachmentCurrentLocked(actorState, scopedEvent.bipedObject, a_object, scopedEvent.sourceObject, selectedXml)) {
					spdlog::debug(
						"skipping duplicate armor system attach actor={} bipedObject={} object={} xml='{}'",
						static_cast<void*>(a_event.actor),
						std::to_underlying(scopedEvent.bipedObject),
						static_cast<void*>(a_object),
						selectedXml);
					return false;
				}
				if (const auto* attachment = FindAttachmentLocked(actorState, scopedEvent.bipedObject, a_object, scopedEvent.sourceObject, selectedXml)) {
					for (const auto buildGroup : attachment->buildGroups) {
						if (buildGroup != 0 && std::ranges::find(staleArmorBuildGroups, buildGroup) == staleArmorBuildGroups.end()) {
							staleArmorBuildGroups.push_back(buildGroup);
						}
					}
				}
				for (const auto buildGroup : CollectBuildGroupsForObjectLocked(actorState, a_object)) {
					if (buildGroup != 0 && std::ranges::find(staleArmorBuildGroups, buildGroup) == staleArmorBuildGroups.end()) {
						staleArmorBuildGroups.push_back(buildGroup);
					}
				}
				if (hairSlotArmorBuild) {
					std::vector<std::uint64_t> staleHeadBuildGroups;
					const auto replacedHeadParts = CollectHeadPartGroupsLocked(actorState, BuildDomain::kHair, staleHeadBuildGroups);
					if (replacedHeadParts > 0) {
						spdlog::debug(
							"hair-slot armor attach is replacing tracked hair headpart groups actor={} bipedObject={} object={} xml='{}' hairRecords={} groups={}",
							static_cast<void*>(a_event.actor),
							std::to_underlying(scopedEvent.bipedObject),
							static_cast<void*>(a_object),
							selectedXml,
							replacedHeadParts,
							staleHeadBuildGroups.size());
					}
					if (!staleHeadBuildGroups.empty()) {
						ClearBuildGroupsLocked(actorState, staleHeadBuildGroups);
					}
				}
				if (!hairSlotArmorBuild && !staleArmorBuildGroups.empty()) {
					spdlog::debug(
						"clearing stale armor system groups before rebuild actor={} bipedObject={} object={} count={}",
						static_cast<void*>(a_event.actor),
						std::to_underlying(scopedEvent.bipedObject),
						static_cast<void*>(a_object),
						staleArmorBuildGroups.size());
					ClearBuildGroupsLocked(actorState, staleArmorBuildGroups);
					staleArmorBuildGroups.clear();
				}
				if (hairSlotArmorBuild) {
					ClearStaleHairSlotArmorGroupsLocked(actorState, scopedEvent.bipedObject, 0, "armor-attach-prebuild", a_object, selectedXml);
					staleArmorBuildGroups.clear();
				}
			}
			auto buildResult = BuildSystemObjectsLocked(actorState, scopedEvent, *selectedSummary, a_selection.meshNameMap, BuildDomain::kArmor, !hairSlotArmorBuild);
			if (buildResult.succeeded) {
				if (hairSlotArmorBuild) {
					ActivateBuildGroupLocked(actorState, buildResult.buildGroup);
					buildResult.committed = true;
					LogSystemBulletObjectsLocked(actorState, "after-system-build-commit");
				}
				RecordAttachmentLocked(actorState, scopedEvent.bipedObject, a_object, scopedEvent.sourceObject, selectedXml, buildResult.buildGroup);
				RecordArmorLocked(
					actorState,
					scopedEvent.bipedObject,
					selectedXml,
					a_selection.meshNameMap,
					a_object,
					scopedEvent.sourceObject,
					scopedEvent.armorBoneReferences,
					buildResult.buildGroup);
				if (hairSlotArmorBuild) {
					const auto remainingHairSlotArmorGroups = CollectArmorPhysicsGroupsForBipedObjectLocked(actorState, scopedEvent.bipedObject).size();
					spdlog::debug(
						"hair-slot armor ownership after attach commit actor={} bipedObject={} buildGroup={} object={} xml='{}' remainingArmorGroups={}",
						static_cast<void*>(a_event.actor),
						std::to_underlying(scopedEvent.bipedObject),
						buildResult.buildGroup,
						static_cast<void*>(a_object),
						selectedXml,
						remainingHairSlotArmorGroups);
				}
			} else if (buildResult.buildGroup != 0 && BuildGroupIsRecordableLocked(actorState, buildResult.buildGroup, BuildDomain::kArmor)) {
				ClearBuildGroupsLocked(actorState, std::vector<std::uint64_t>{ buildResult.buildGroup });
				spdlog::debug(
					"rolled back incomplete armor system build group actor={} bipedObject={} object={} buildGroup={} xml='{}' pendingCpuCopy={}",
					static_cast<void*>(a_event.actor),
					std::to_underlying(scopedEvent.bipedObject),
					static_cast<void*>(a_object),
					buildResult.buildGroup,
					selectedXml,
					buildResult.cpuCopyPending);
			}
			if (scopedArmorBuild && buildResult.cpuCopyPending && a_event.actor) {
				MarkPendingActorRebuildLocked(a_event.actor, a_event.firstPerson, std::vector<ArmorPhysicsRecord>{
					ArmorPhysicsRecord{
						.bipedObject = scopedEvent.bipedObject,
						.physicsXmlPath = selectedXml,
						.meshNameMap = a_selection.meshNameMap,
						.attachedObject = a_object,
						.sourceObject = scopedEvent.sourceObject,
						.armorBoneReferences = scopedEvent.armorBoneReferences,
						.cpuCopyRetryCount = 1,
					},
				});
			}
			if (scopedArmorBuild) {
				DeactivateBuiltSystemIfInactiveLocked(actorState, scopedEvent);
			}
			return buildResult.succeeded;
		};

		std::optional<ArmorPhysicsXmlSelection> discoveredXml;
		if (armorAttach && !a_event.physicsXmlPath.empty()) {
			discoveredXml = ArmorPhysicsXmlSelection{ .path = a_event.physicsXmlPath };
		}
		const auto selectedXml = discoveredXml ? discoveredXml->path.string() : (armorAttach ? std::string{} : fallbackPhysicsXml_);
		std::vector<ArmorPhysicsXmlBuildCandidate> equippedArmorCandidates;
		if (!armorAttach) {
			CollectEquippedArmorPhysicsXmlSelections(a_event, equippedArmorCandidates);
		}
		if (selectedXml.empty() && equippedArmorCandidates.empty()) {
			if (armorAttach) {
				spdlog::trace(
					"armor attach candidate has no pre-scanned XML actor={} object={} sourceObject={} sourceRoot={} destinationRoot={} bipObject={} model='{}' armorAddon={} preScannedXml='{}'",
					static_cast<void*>(a_event.actor),
					static_cast<void*>(a_event.object),
					static_cast<void*>(a_event.sourceObject),
					static_cast<void*>(a_event.sourceRoot),
					static_cast<void*>(a_event.destinationRoot),
					static_cast<void*>(a_event.bipObject),
					(a_event.bipObject && a_event.bipObject->part) ? a_event.bipObject->part->GetModel() : "",
					static_cast<void*>(a_event.bipObject ? a_event.bipObject->armorAddon : nullptr),
					a_event.physicsXmlPath);
			}
			return;
		}

		if (!armorAttach) {
			if (auto* actorState = FindSystemLocked(a_event.actor, a_event.firstPerson)) {
				ClearSystemLocked(*actorState);
				std::erase_if(systems_, [](const auto& a_state) {
					return !a_state->suspended && !a_state->HasPhysics();
				});
			}
		}

		if (!armorAttach && !equippedArmorCandidates.empty()) {
			std::uint32_t builtEquipped = 0;
			for (const auto& candidate : equippedArmorCandidates) {
				if (buildSelection(candidate)) {
					++builtEquipped;
				}
			}
			spdlog::debug(
				"processed equipped armor system physics rescan actor={} candidates={} built={}",
				static_cast<void*>(a_event.actor),
				equippedArmorCandidates.size(),
				builtEquipped);
			std::erase_if(systems_, [](const auto& a_state) {
				return !a_state->suspended && !a_state->HasPhysics();
			});
			if (selectedXml.empty()) {
				return;
			}
		}

		const Smp::DefaultBBP::NameMap emptyMeshNameMap;
		buildSelection(ArmorPhysicsXmlBuildCandidate{
			.object = a_event.object,
			.selection = ArmorPhysicsXmlSelection{
				.path = selectedXml,
				.meshNameMap = discoveredXml ? discoveredXml->meshNameMap : emptyMeshNameMap,
			},
			.bipObject = a_event.bipObject,
			.bipedObject = a_event.bipedObject,
			.sourceObject = a_event.sourceObject,
			.sourceRoot = a_event.sourceRoot,
		});
		std::erase_if(systems_, [](const auto& a_state) {
			return !a_state->suspended && !a_state->HasPhysics();
		});
	}

	bool Fo4PhysicsWorld::FinalizeHeadHierarchyForEventLocked(const LifecycleEvent& a_event)
	{
		if (!a_event.actor || a_event.firstPerson) {
			return false;
		}

		auto* faceNode = a_event.actor->GetFaceNodeSkinned();
		auto* actorObject = a_event.actor->Get3D(false);
		auto* destinationRoot = actorObject ? actorObject->IsNode() : nullptr;
		if (!faceNode || !destinationRoot) {
			return false;
		}

		std::vector<HeadPhysicsXmlBuildCandidate> candidates;
		CollectLiveHeadPartCandidates(
			a_event.actor,
			reinterpret_cast<RE::NiAVObject*>(faceNode),
			candidates);

		std::uint32_t finalized = 0;
		for (auto& candidate : candidates) {
			finalized += FinalizeLiveHeadPartCandidate(a_event.actor, candidate) ? 1U : 0U;
		}
		spdlog::debug(
			"finalized live headpart hierarchy without physics build actor={} faceNode={} candidates={} finalized={}",
			static_cast<void*>(a_event.actor),
			static_cast<void*>(faceNode),
			candidates.size(),
			finalized);
		return finalized > 0;
	}

	bool Fo4PhysicsWorld::BuildHeadForEventLocked(const LifecycleEvent& a_event)
	{
		auto* faceNode = a_event.actor ? a_event.actor->GetFaceNodeSkinned() : nullptr;
		if (!faceNode) {
			spdlog::debug("skipping head physics candidate {} actor={} because no skinned face node is available", ToString(a_event.type), static_cast<void*>(a_event.actor));
			return false;
		}
		auto* faceObject = reinterpret_cast<RE::NiAVObject*>(faceNode);

		auto& actorState = GetOrCreateSystemLocked(a_event.actor, a_event.firstPerson);
		struct PendingPoseCacheCleanup
		{
			std::vector<AttachmentBoneLocalPose>& poses;

			~PendingPoseCacheCleanup()
			{
				std::erase_if(poses, [](const AttachmentBoneLocalPose& a_pose) {
					return a_pose.buildGroup == kPapyrusHeadPoseCacheBuildGroup;
				});
			}
		} pendingPoseCacheCleanup{ actorState.attachmentBoneLocalPoses };
		const auto faceNodeChanged = actorState.faceNode && actorState.faceNode.get() != faceObject;
		if (faceNodeChanged) {
			ClearHeadPhysicsTrackingLocked(actorState, "face-node-replacement");
			spdlog::debug(
				"cleared stale head/hair system physics after face node replacement actor={} oldFaceNode={} newFaceNode={}",
				static_cast<void*>(a_event.actor),
				static_cast<void*>(actorState.faceNode.get()),
				static_cast<void*>(faceObject));
		}
		actorState.faceNode = faceObject;

		const auto suppressHairHeadParts =
			HasActiveHairSlotArmorLocked(actorState) ||
			(disableSMPHairWhenWigEquipped_ && HasEquippedHairSlotObject(a_event.actor));

		const auto touchedHeadGeometry = a_event.type == LifecycleEventType::kHeadSkinSingleGeometry;
		const auto touchedObjectValid = !touchedHeadGeometry || !a_event.object || IsObjectInTree(faceObject, a_event.object);
		if (touchedHeadGeometry && !touchedObjectValid) {
			ClearHeadPhysicsTrackingLocked(actorState, "stale-touched-headpart");
			spdlog::debug(
				"discarded stale touched headpart object for actor={} object={} faceNode={}; rebuilding full current face node",
				static_cast<void*>(a_event.actor),
				static_cast<void*>(a_event.object),
				static_cast<void*>(faceObject));
		}

		const auto hairKeys = BuildHairHeadpartKeys(a_event.actor);
		std::vector<HeadPhysicsXmlBuildCandidate> candidates;
		CollectLiveHeadPartCandidates(a_event.actor, faceObject, candidates);
		CollectHeadPhysicsXmlSelections(
			touchedHeadGeometry && touchedObjectValid && a_event.object ? a_event.object : faceObject,
			hairKeys,
			candidates);

		std::size_t candidateRecipeCount = 0;
		std::vector<ArmorBoneReference> cachedBoneReferences;
		for (const auto& candidate : candidates) {
			candidateRecipeCount += candidate.boneReferences.size();
			for (const auto& reference : candidate.boneReferences) {
				MergeBoneReferenceRecipe(cachedBoneReferences, reference);
			}
		}
		if (actorState.actorHandle && !cachedBoneReferences.empty()) {
			const auto faceIdentity = reinterpret_cast<std::uintptr_t>(faceObject);
			std::erase_if(
				retainedHeadSkeletonCaches_,
				[&](const RetainedHeadSkeletonCache& a_cached) {
					const auto actor = a_cached.actorHandle.get();
					return !actor ||
						(actor.get() == a_event.actor &&
							a_cached.retainedFaceIdentity != faceIdentity);
				});
			auto cached = std::ranges::find_if(
				retainedHeadSkeletonCaches_,
				[&](const RetainedHeadSkeletonCache& a_cached) {
					const auto actor = a_cached.actorHandle.get();
					return actor &&
						actor.get() == a_event.actor &&
						a_cached.retainedFaceIdentity == faceIdentity;
				});
			if (cached == retainedHeadSkeletonCaches_.end()) {
				retainedHeadSkeletonCaches_.push_back({
					.actorHandle = actorState.actorHandle,
					.retainedFaceIdentity = faceIdentity,
				});
				cached = std::prev(retainedHeadSkeletonCaches_.end());
			}
			cached->headBoneReferences = std::move(cachedBoneReferences);
			cached->requiredHeadBoneNames.clear();
			spdlog::debug(
				"cached retained-head skeleton recipes actor={} faceNode={} candidateRecipes={} cachedRecipes={} before hair suppression",
				static_cast<void*>(a_event.actor),
				static_cast<void*>(faceNode),
				candidateRecipeCount,
				cached->headBoneReferences.size());
		}

		if (suppressHairHeadParts) {
			std::vector<std::uint64_t> removedGroups;
			const auto removedHeadParts = CollectHeadPartGroupsLocked(actorState, BuildDomain::kHair, removedGroups);
			if (!removedGroups.empty()) {
				ClearBuildGroupsLocked(actorState, removedGroups);
			}
			const auto removedCandidates = std::erase_if(candidates, [](const HeadPhysicsXmlBuildCandidate& a_candidate) {
				return a_candidate.domain == BuildDomain::kHair;
			});
			spdlog::debug(
				"suppressed hair headpart physics while preserving non-hair headparts actor={} faceNode={} removedRecords={} removedGroups={} removedCandidates={}",
				static_cast<void*>(a_event.actor),
				static_cast<void*>(faceNode),
				removedHeadParts,
				removedGroups.size(),
				removedCandidates);
		}

		std::vector<std::uint64_t> staleClosureGroups;
		for (const auto& record : actorState.headPartRecords) {
			if (!record.isHeadPartClosure || record.buildGroup == 0) {
				continue;
			}
			const auto recordXmlKey = ConfigPaths::LowerString(record.physicsXmlPath);
			const auto stillCurrent = std::ranges::any_of(candidates, [&](const HeadPhysicsXmlBuildCandidate& a_candidate) {
				return a_candidate.isHeadPartClosure &&
					a_candidate.domain == record.domain &&
					a_candidate.object == record.object.get() &&
					std::ranges::any_of(a_candidate.paths, [&](const std::filesystem::path& a_path) {
						return ConfigPaths::LowerString(a_path.string()) == recordXmlKey;
					});
			});
			if (!stillCurrent && std::ranges::find(staleClosureGroups, record.buildGroup) == staleClosureGroups.end()) {
				staleClosureGroups.push_back(record.buildGroup);
			}
		}
		if (!staleClosureGroups.empty()) {
			ClearBuildGroupsLocked(actorState, staleClosureGroups);
			spdlog::debug(
				"cleared {} stale headpart closure build groups for actor={} before rebuilding current headpart state",
				staleClosureGroups.size(),
				static_cast<void*>(a_event.actor));
		}

		if (candidates.empty()) {
			if (!actorState.headPartRecords.empty()) {
				if (!touchedHeadGeometry) {
					ClearBuildGroupsByDomainLocked(actorState, BuildDomain::kHead);
					ClearBuildGroupsByDomainLocked(actorState, BuildDomain::kHair);
					actorState.headPartRecords.clear();
				} else if (touchedObjectValid && a_event.object) {
					std::vector<std::uint64_t> removedGroups;
					for (const auto& record : actorState.headPartRecords) {
						if (record.object.get() == a_event.object && record.buildGroup != 0 && std::ranges::find(removedGroups, record.buildGroup) == removedGroups.end()) {
							removedGroups.push_back(record.buildGroup);
						}
					}
					ClearBuildGroupsLocked(actorState, removedGroups);
				}
			}
			spdlog::debug(
				"head physics candidate {} actor={} faceNode={} object={} found no XML/defaultBBP head/hair subtrees hairKeys={}",
				ToString(a_event.type),
				static_cast<void*>(a_event.actor),
				static_cast<void*>(faceNode),
				static_cast<void*>(a_event.object),
				hairKeys.size());
			std::erase_if(systems_, [](const auto& a_state) {
				return !a_state->suspended && !a_state->HasPhysics();
			});
			return false;
		}

		auto* loader = PhysicsXmlLoader::GetSingleton();
		std::uint32_t built = 0;
		std::uint32_t skippedExisting = 0;
		std::uint32_t skippedDuplicateXml = 0;
		bool cpuCopyPending = false;
		struct ScannedPhysicsFile
		{
			BuildDomain domain;
			std::string pathKey;
			RE::NiAVObject* object;
			bool isHeadPartClosure;
		};
		std::vector<ScannedPhysicsFile> scannedPhysicsFiles;
		for (auto& candidate : candidates) {
			std::vector<std::pair<std::filesystem::path, std::string>> selectedXmls;
			for (const auto& path : candidate.paths) {
				const auto selectedXml = path.string();
				if (selectedXml.empty()) {
					continue;
				}
				const auto selectedXmlKey = ConfigPaths::LowerString(selectedXml);
				const auto duplicatePhysicsFile = std::ranges::any_of(scannedPhysicsFiles, [&](const auto& a_entry) {
					if (a_entry.domain != candidate.domain || a_entry.pathKey != selectedXmlKey) {
						return false;
					}
					if (a_entry.isHeadPartClosure && candidate.isHeadPartClosure) {
						return a_entry.object == candidate.object;
					}
					return true;
				});
				if (duplicatePhysicsFile) {
					++skippedDuplicateXml;
					continue;
				}
				const auto trackedByAnotherObject = std::ranges::any_of(actorState.headPartRecords, [&](const HeadPartPhysicsRecord& a_record) {
					return a_record.object.get() != candidate.object &&
						a_record.domain == candidate.domain &&
						a_record.isHeadPartClosure == candidate.isHeadPartClosure &&
						!candidate.isHeadPartClosure &&
						a_record.buildGroup != 0 &&
						ConfigPaths::LowerString(a_record.physicsXmlPath) == selectedXmlKey;
				});
				if (trackedByAnotherObject) {
					++skippedDuplicateXml;
					continue;
				}
				scannedPhysicsFiles.push_back({
					.domain = candidate.domain,
					.pathKey = selectedXmlKey,
					.object = candidate.object,
					.isHeadPartClosure = candidate.isHeadPartClosure,
				});
				selectedXmls.emplace_back(path, std::move(selectedXmlKey));
			}
			if (selectedXmls.empty()) {
				continue;
			}

			std::uint64_t trackedBuildGroup = 0;
			const auto candidateAlreadyBuilt = std::ranges::all_of(selectedXmls, [&](const auto& a_selectedXml) {
				const auto record = std::ranges::find_if(actorState.headPartRecords, [&](const HeadPartPhysicsRecord& a_record) {
					return a_record.object.get() == candidate.object &&
						a_record.sourceObject.get() == candidate.sourceObject.get() &&
						a_record.domain == candidate.domain &&
						a_record.isHeadPartClosure == candidate.isHeadPartClosure &&
						a_record.buildGroup != 0 &&
						ConfigPaths::LowerString(a_record.physicsXmlPath) == a_selectedXml.second;
				});
				if (record == actorState.headPartRecords.end()) {
					return false;
				}
				if (trackedBuildGroup == 0) {
					trackedBuildGroup = record->buildGroup;
				}
				return record->buildGroup == trackedBuildGroup;
			});
			if (candidateAlreadyBuilt) {
				skippedExisting += static_cast<std::uint32_t>(selectedXmls.size());
				continue;
			}

			PhysicsXmlSummary mergedSummary;
			std::vector<std::filesystem::path> loadedPaths;
			std::string sourceKey;
			for (const auto& [path, pathKey] : selectedXmls) {
				spdlog::info(
					"loading {} system physics XML {} for actor={} object={}",
					BuildDomainName(candidate.domain),
					path.string(),
					static_cast<void*>(a_event.actor),
					static_cast<void*>(candidate.object));
				const auto selectedSummary = loader->LoadSummary(path.string());
				if (!selectedSummary) {
					spdlog::warn(
						"skipping {} physics XML because it failed to load: {}",
						BuildDomainName(candidate.domain),
						path.string());
					continue;
				}
				MergePhysicsSummary(mergedSummary, *selectedSummary);
				loadedPaths.push_back(path);
				if (!sourceKey.empty()) {
					sourceKey.push_back('|');
				}
				sourceKey.append(pathKey);
			}
			if (loadedPaths.empty()) {
				continue;
			}

			std::vector<std::uint64_t> replacedGroups;
			for (const auto& record : actorState.headPartRecords) {
				if (record.object.get() == candidate.object &&
					record.domain == candidate.domain &&
					record.isHeadPartClosure == candidate.isHeadPartClosure &&
					record.buildGroup != 0 &&
					std::ranges::find(replacedGroups, record.buildGroup) == replacedGroups.end()) {
					replacedGroups.push_back(record.buildGroup);
				}
			}
			if (!replacedGroups.empty()) {
				ClearBuildGroupsLocked(actorState, replacedGroups);
				spdlog::debug(
					"cleared {} partial {} headpart build groups for actor={} object={} before single-pass rebuild",
					replacedGroups.size(),
					BuildDomainName(candidate.domain),
					static_cast<void*>(a_event.actor),
					static_cast<void*>(candidate.object));
			}

			auto scopedEvent = a_event;
			scopedEvent.object = candidate.object;
			scopedEvent.sourceObject = candidate.sourceObject.get();
			scopedEvent.sourceRoot = candidate.sourceRoot ? candidate.sourceRoot->IsNode() : nullptr;
			if (candidate.destinationRoot) {
				scopedEvent.destinationRoot = candidate.destinationRoot.get();
			}
			if (FinalizeLiveHeadPartCandidate(a_event.actor, candidate)) {
				scopedEvent.armorBoneReferences = candidate.boneReferences;
			}
			const auto requestedBuildGroup =
				candidate.paths.size() > 1 || !candidate.boneReferences.empty() ?
				++actorState.nextBuildGroup :
				0;
			const auto buildResult = BuildSystemObjectsLocked(
				actorState,
				scopedEvent,
				mergedSummary,
				candidate.meshNameMap,
				candidate.domain,
				true,
				candidate.meshSourceRoots,
				requestedBuildGroup,
				sourceKey);
			cpuCopyPending = cpuCopyPending || buildResult.cpuCopyPending;
			if (buildResult.succeeded) {
				auto storedBoneReferences = candidate.boneReferences;
				for (auto& reference : storedBoneReferences) {
					ResetBoneReferenceRuntimeState(reference);
				}
				for (const auto& loadedPath : loadedPaths) {
					actorState.headPartRecords.push_back({
						.domain = candidate.domain,
						.physicsXmlPath = loadedPath.string(),
						.object = candidate.object,
						.sourceObject = candidate.sourceObject,
						.sourceRoot = candidate.sourceRoot,
						.boneReferences = storedBoneReferences,
						.requiredBoneNames = mergedSummary.boneNames,
						.buildGroup = buildResult.buildGroup,
						.isHeadPartClosure = candidate.isHeadPartClosure,
					});
				}
				++built;
			}
		}

		spdlog::debug(
			"processed head physics candidate actor={} faceNode={} candidates={} built={} skippedExisting={} skippedDuplicateXml={} trackedHeadParts={} hairKeys={} pendingCpuCopy={}",
			static_cast<void*>(a_event.actor),
			static_cast<void*>(faceNode),
			candidates.size(),
			built,
			skippedExisting,
			skippedDuplicateXml,
			actorState.headPartRecords.size(),
			hairKeys.size(),
			cpuCopyPending);
		std::erase_if(systems_, [](const auto& a_state) {
			return !a_state->suspended && !a_state->HasPhysics();
		});
		return cpuCopyPending;
	}

	bool Fo4PhysicsWorld::IsBuildCandidateLocked(const LifecycleEvent& a_event, const bool a_requireObject)
	{
		if (a_requireObject && !a_event.object) {
			spdlog::trace("skipping system physics candidate {} with null object", ToString(a_event.type));
			return false;
		}

		const auto* player = RE::PlayerCharacter::GetSingleton();
		if (!a_event.actor) {
			spdlog::trace("skipping system physics candidate {} with null actor", ToString(a_event.type));
			return false;
		}

		if (IsIgnoredFirstPersonEvent(a_event, disableFirstPersonViewPhysics_)) {
			spdlog::debug("skipping first-person system physics candidate {}", ToString(a_event.type));
			return false;
		}

		if (a_event.actor == player) {
			return true;
		}

		if (!enableNpcPhysics_) {
			spdlog::trace("skipping system physics candidate {} for non-player actor={} because NPC physics is disabled", ToString(a_event.type), static_cast<void*>(a_event.actor));
			return false;
		}

		const auto buildSuspendedArmorCandidate = ShouldBuildSuspendedArmorCandidateLocked(a_event);
		const auto activeActors = static_cast<std::size_t>(std::ranges::count_if(systems_, [](const auto& a_state) {
			return a_state->IsActive();
		}));
		if (FindSystemLocked(a_event.actor, a_event.firstPerson) == nullptr && activeActors >= currentMaxActiveActors_) {
			if (buildSuspendedArmorCandidate) {
				spdlog::debug(
					"allowing out-of-budget SMP armor candidate {} for actor={} to build before inactive-system deactivation ({}/{})",
					ToString(a_event.type),
					static_cast<void*>(a_event.actor),
					activeActors,
					currentMaxActiveActors_);
			} else {
				spdlog::debug(
					"skipping system physics candidate {} for actor={} because active actor budget is full ({}/{})",
					ToString(a_event.type),
					static_cast<void*>(a_event.actor),
					activeActors,
					currentMaxActiveActors_);
				SuspendActorCandidateLocked(a_event.actor, a_event.firstPerson);
				return false;
			}
		}

		if (!IsActorWithinDistanceLocked(a_event.actor)) {
			if (buildSuspendedArmorCandidate) {
				spdlog::debug(
					"allowing out-of-range SMP armor candidate {} for actor={} to build before inactive-system deactivation maxActorDistance={}",
					ToString(a_event.type),
					static_cast<void*>(a_event.actor),
					maxActorDistance_);
				return true;
			}
			spdlog::trace(
				"skipping system physics candidate {} for actor={} beyond maxActorDistance={}",
				ToString(a_event.type),
				static_cast<void*>(a_event.actor),
				maxActorDistance_);
			SuspendActorCandidateLocked(a_event.actor, a_event.firstPerson);
			return false;
		}

		if (!IsActorInReferenceCullView(a_event.actor, a_event.object, a_event.firstPerson)) {
			if (buildSuspendedArmorCandidate) {
				spdlog::debug(
					"allowing inactive-view SMP armor candidate {} for actor={} to build before inactive-system deactivation",
					ToString(a_event.type),
					static_cast<void*>(a_event.actor));
				return true;
			}
			spdlog::trace(
				"skipping system physics candidate {} for actor={} because reference view culler marks it inactive",
				ToString(a_event.type),
				static_cast<void*>(a_event.actor));
			SuspendActorCandidateLocked(a_event.actor, a_event.firstPerson);
			return false;
		}

		return true;
	}
}
