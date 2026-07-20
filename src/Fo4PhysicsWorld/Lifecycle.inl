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
		solver_ = std::make_unique<btSequentialImpulseConstraintSolver>();
		dynamicsWorld_ = std::make_unique<PrototypeDynamicsWorld>(dispatcher_.get(), broadphase_.get(), solver_.get(), collisionConfiguration_.get());
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
		ClearAllPrototypeStatesLocked();

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
		loadingMenuDepth_ = 0;
		loadingPhysicsSuspended_ = false;
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
			if (const auto* actorState = FindPrototypeStateLocked(a_event.actor, a_event.firstPerson);
				actorState && actorState->HasActiveRuntime()) {
				spdlog::debug(
					"skipping generic {} rebuild for actor={} firstPerson={} because direct armor physics is already active",
					ToString(a_event.type),
					static_cast<void*>(a_event.actor),
					a_event.firstPerson);
				return;
			}
		}

		PruneInvalidPrototypeStatesLocked();

		const auto armorAttach = IsArmorAttachCandidate(a_event.type);
		const auto actorArmorAttach = armorAttach && a_event.actor && !a_event.firstPerson;
		if (actorArmorAttach && loadingPhysicsSuspended_) {
			auto armorRecords = CollectQueuedArmorRecordsForAttachLocked(a_event);
			const auto queuedRecords = armorRecords.size();
			MarkPendingActorRebuildLocked(a_event.actor, a_event.firstPerson, std::move(armorRecords), false, false);
			ResetStepClockLocked();
			spdlog::debug(
				"queued scoped armor prototype physics resume for loading-screen attach {} actor={} object={} armorRecords={}",
				ToString(a_event.type),
				static_cast<void*>(a_event.actor),
				static_cast<void*>(a_event.object),
				queuedRecords);
			return;
		}

		if (DeferCharacterCustomizationLifecycleLocked(a_event, true, false)) {
			if (actorArmorAttach) {
				spdlog::debug(
					"deferred scoped armor prototype physics resume for customization target attach {} actor={} object={} armorRecords={}",
					ToString(a_event.type),
					static_cast<void*>(a_event.actor),
					static_cast<void*>(a_event.object),
					characterCustomizationArmorRecords_.size());
			} else {
				spdlog::debug(
					"deferred prototype physics attach candidate {} for customization target actor={} object={}",
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

		if (!IsPrototypeCandidateLocked(a_event, true)) {
			auto armorRecords = CollectSuspendedArmorRecordsLocked(a_event);
			if (!armorRecords.empty()) {
				spdlog::debug(
					"captured {} suspended armor records for skipped prototype physics candidate {} actor={}",
					armorRecords.size(),
					ToString(a_event.type),
					static_cast<void*>(a_event.actor));
				SuspendActorCandidateLocked(a_event.actor, a_event.firstPerson, std::move(armorRecords));
			}
			return;
		}

		BuildPrototypeForEventLocked(a_event);
		if ((a_event.type == LifecycleEventType::kActorLoad3D || a_event.type == LifecycleEventType::kActorSet3D) && a_event.actor) {
			const auto* actorState = FindPrototypeStateLocked(a_event.actor, a_event.firstPerson);
			if (!actorState || !actorState->HasActiveRuntime()) {
				auto armorRecords = actorState ? actorState->armorRecords : std::vector<PrototypeArmorRecord>{};
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

	void Fo4PhysicsWorld::BuildPrototypeForEventLocked(const LifecycleEvent& a_event)
	{
		auto* loader = PhysicsXmlLoader::GetSingleton();
		const auto armorAttach = IsArmorAttachCandidate(a_event.type);

		const auto buildSelection = [&](const ArmorPhysicsXmlBuildCandidate& a_candidate) {
			auto* a_object = a_candidate.object;
			const auto& a_selection = a_candidate.selection;
			const auto selectedXml = a_selection.path.string();
			if (!a_object || selectedXml.empty()) {
				return false;
			}

			spdlog::info(
				"loading prototype physics XML {} for actor={} object={}",
				selectedXml,
				static_cast<void*>(a_event.actor),
				static_cast<void*>(a_object));
			const auto selectedSummary = loader->LoadSummary(selectedXml);
			if (!selectedSummary) {
				spdlog::warn("skipping prototype physics candidate because selected XML failed to load: {}", selectedXml);
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
			auto& actorState = GetOrCreatePrototypeStateLocked(a_event.actor, a_event.firstPerson);
			std::vector<std::uint64_t> staleArmorBuildGroups;
			const auto scopedArmorBuild = armorAttach || scopedEvent.bipedObject != RE::BIPED_OBJECT::kTotal;
			const auto hairSlotArmorBuild = scopedArmorBuild && IsHairBipedObject(scopedEvent.bipedObject);
			if (scopedArmorBuild) {
				if (IsPrototypeAttachmentCurrentLocked(actorState, scopedEvent.bipedObject, a_object, scopedEvent.sourceObject, selectedXml)) {
					spdlog::debug(
						"skipping duplicate armor prototype attach actor={} bipedObject={} object={} xml='{}'",
						static_cast<void*>(a_event.actor),
						std::to_underlying(scopedEvent.bipedObject),
						static_cast<void*>(a_object),
						selectedXml);
					return false;
				}
				if (const auto* attachment = FindPrototypeAttachmentLocked(actorState, scopedEvent.bipedObject, a_object, scopedEvent.sourceObject, selectedXml)) {
					for (const auto buildGroup : attachment->buildGroups) {
						if (buildGroup != 0 && std::ranges::find(staleArmorBuildGroups, buildGroup) == staleArmorBuildGroups.end()) {
							staleArmorBuildGroups.push_back(buildGroup);
						}
					}
				}
				for (const auto buildGroup : CollectPrototypeGroupsForObjectLocked(actorState, a_object)) {
					if (buildGroup != 0 && std::ranges::find(staleArmorBuildGroups, buildGroup) == staleArmorBuildGroups.end()) {
						staleArmorBuildGroups.push_back(buildGroup);
					}
				}
				if (hairSlotArmorBuild) {
					std::vector<std::uint64_t> staleHeadBuildGroups;
					const auto replacedHeadParts = CollectHeadPartGroupsLocked(actorState, staleHeadBuildGroups);
					if (replacedHeadParts > 0) {
						spdlog::debug(
							"hair-slot armor attach is replacing tracked head/hair prototype groups actor={} bipedObject={} object={} xml='{}' headPartRecords={} groups={}",
							static_cast<void*>(a_event.actor),
							std::to_underlying(scopedEvent.bipedObject),
							static_cast<void*>(a_object),
							selectedXml,
							replacedHeadParts,
							staleHeadBuildGroups.size());
					}
					if (!staleHeadBuildGroups.empty()) {
						ClearPrototypeGroupsLocked(actorState, staleHeadBuildGroups, true);
					}
				}
				if (!hairSlotArmorBuild && !staleArmorBuildGroups.empty()) {
					spdlog::debug(
						"clearing stale armor prototype groups before rebuild actor={} bipedObject={} object={} count={}",
						static_cast<void*>(a_event.actor),
						std::to_underlying(scopedEvent.bipedObject),
						static_cast<void*>(a_object),
						staleArmorBuildGroups.size());
					ClearPrototypeGroupsLocked(actorState, staleArmorBuildGroups);
					staleArmorBuildGroups.clear();
				}
				if (hairSlotArmorBuild) {
					ClearStaleHairSlotArmorGroupsLocked(actorState, scopedEvent.bipedObject, 0, "armor-attach-prebuild", a_object, selectedXml);
					staleArmorBuildGroups.clear();
				}
			}
			auto buildResult = BuildPrototypeBodiesLocked(actorState, scopedEvent, *selectedSummary, a_selection.meshNameMap, PrototypeBuildDomain::kArmor, !hairSlotArmorBuild);
			if (buildResult.succeeded) {
				if (hairSlotArmorBuild) {
					CommitPrototypeBuildGroupToBulletLocked(actorState, buildResult.buildGroup);
					buildResult.committed = true;
					LogPrototypeActorBulletObjectsLocked(actorState, "after-prototype-build-commit");
				}
				RecordPrototypeAttachmentLocked(actorState, scopedEvent.bipedObject, a_object, scopedEvent.sourceObject, selectedXml, buildResult.buildGroup);
				RecordPrototypeArmorLocked(
					actorState,
					scopedEvent.bipedObject,
					selectedXml,
					a_selection.meshNameMap,
					a_object,
					scopedEvent.sourceObject,
					scopedEvent.armorBoneReferences,
					buildResult.buildGroup);
				if (hairSlotArmorBuild) {
					const auto remainingHairSlotArmorGroups = CollectArmorPrototypeGroupsForBipedObjectLocked(actorState, scopedEvent.bipedObject).size();
					spdlog::debug(
						"hair-slot armor ownership after attach commit actor={} bipedObject={} buildGroup={} object={} xml='{}' remainingArmorGroups={}",
						static_cast<void*>(a_event.actor),
						std::to_underlying(scopedEvent.bipedObject),
						buildResult.buildGroup,
						static_cast<void*>(a_object),
						selectedXml,
						remainingHairSlotArmorGroups);
				}
			} else if (buildResult.buildGroup != 0 && PrototypeBuildGroupIsRecordableLocked(actorState, buildResult.buildGroup, PrototypeBuildDomain::kArmor)) {
				ClearPrototypeGroupsLocked(actorState, std::vector<std::uint64_t>{ buildResult.buildGroup });
				spdlog::debug(
					"rolled back incomplete armor prototype build group actor={} bipedObject={} object={} buildGroup={} xml='{}' pendingCpuCopy={}",
					static_cast<void*>(a_event.actor),
					std::to_underlying(scopedEvent.bipedObject),
					static_cast<void*>(a_object),
					buildResult.buildGroup,
					selectedXml,
					buildResult.cpuCopyPending);
			}
			if (scopedArmorBuild && buildResult.cpuCopyPending && a_event.actor) {
				MarkPendingActorRebuildLocked(a_event.actor, a_event.firstPerson, std::vector<PrototypeArmorRecord>{
					PrototypeArmorRecord{
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
				SoftSuspendBuiltRuntimeIfOutOfRangeLocked(actorState, scopedEvent);
			}
			return buildResult.succeeded;
		};

		std::optional<ArmorPhysicsXmlSelection> discoveredXml;
		if (armorAttach && !a_event.physicsXmlPath.empty()) {
			discoveredXml = ArmorPhysicsXmlSelection{ .path = a_event.physicsXmlPath };
		}
		const auto selectedXml = discoveredXml ? discoveredXml->path.string() : (armorAttach ? std::string{} : prototypePhysicsXml_);
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
			if (loader->HasPrototype()) {
				loader->LoadPrototype({});
			}
			return;
		}

		if (!armorAttach) {
			if (auto* actorState = FindPrototypeStateLocked(a_event.actor, a_event.firstPerson)) {
				ClearPrototypeStateLocked(*actorState);
				std::erase_if(prototypeActors_, [](const PrototypeActorState& a_state) {
					return !a_state.runtimeSuspended && !a_state.HasRuntime();
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
				"processed equipped armor prototype physics rescan actor={} candidates={} built={}",
				static_cast<void*>(a_event.actor),
				equippedArmorCandidates.size(),
				builtEquipped);
			std::erase_if(prototypeActors_, [](const PrototypeActorState& a_state) {
				return !a_state.runtimeSuspended && !a_state.HasRuntime();
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
		std::erase_if(prototypeActors_, [](const PrototypeActorState& a_state) {
			return !a_state.runtimeSuspended && !a_state.HasRuntime();
		});
	}

	void Fo4PhysicsWorld::BuildHeadPrototypeForEventLocked(const LifecycleEvent& a_event)
	{
		auto* faceNode = a_event.actor ? a_event.actor->GetFaceNodeSkinned() : nullptr;
		if (!faceNode) {
			spdlog::debug("skipping head physics candidate {} actor={} because no skinned face node is available", ToString(a_event.type), static_cast<void*>(a_event.actor));
			return;
		}
		auto* faceObject = reinterpret_cast<RE::NiAVObject*>(faceNode);

		auto& actorState = GetOrCreatePrototypeStateLocked(a_event.actor, a_event.firstPerson);
		const auto faceNodeChanged = actorState.faceNode && actorState.faceNode.get() != faceObject;
		if (faceNodeChanged) {
			ClearHeadPrototypeTrackingLocked(actorState, "face-node-replacement");
			spdlog::debug(
				"cleared stale head/hair prototype physics after face node replacement actor={} oldFaceNode={} newFaceNode={}",
				static_cast<void*>(a_event.actor),
				static_cast<void*>(actorState.faceNode.get()),
				static_cast<void*>(faceObject));
		}
		actorState.faceNode = faceObject;

		const auto suppressHeadForHairArmor =
			HasActiveHairSlotArmorLocked(actorState) ||
			(disableSMPHairWhenWigEquipped_ && HasEquippedHairSlotObject(a_event.actor));
		if (suppressHeadForHairArmor) {
			std::vector<std::uint64_t> removedGroups;
			const auto removedHeadParts = CollectHeadPartGroupsLocked(actorState, removedGroups);
			if (!removedGroups.empty()) {
				ClearPrototypeGroupsLocked(actorState, removedGroups);
			}
			spdlog::debug(
				"skipping head/hair prototype rebuild while hair-slot armor is active actor={} faceNode={} removedHeadPartRecords={} removedGroups={}",
				static_cast<void*>(a_event.actor),
				static_cast<void*>(faceNode),
				removedHeadParts,
				removedGroups.size());
			std::erase_if(prototypeActors_, [](const PrototypeActorState& a_state) {
				return !a_state.runtimeSuspended && !a_state.HasRuntime();
			});
			return;
		}

		const auto touchedHeadGeometry = a_event.type == LifecycleEventType::kHeadSkinSingleGeometry;
		const auto touchedObjectValid = !touchedHeadGeometry || !a_event.object || IsObjectInTree(faceObject, a_event.object);
		if (touchedHeadGeometry && !touchedObjectValid) {
			ClearHeadPrototypeTrackingLocked(actorState, "stale-touched-headpart");
			spdlog::debug(
				"discarded stale touched headpart object for actor={} object={} faceNode={}; rebuilding full current face node",
				static_cast<void*>(a_event.actor),
				static_cast<void*>(a_event.object),
				static_cast<void*>(faceObject));
		}

		const auto hairKeys = BuildHairHeadpartKeys(a_event.actor);
		std::vector<HeadPhysicsXmlBuildCandidate> candidates;
		const auto headPartIsHair = a_event.headPart && a_event.headPart->type.get() == RE::BGSHeadPart::HeadPartType::kHair;
		CollectHeadPhysicsXmlSelections(touchedHeadGeometry && touchedObjectValid && a_event.object ? a_event.object : faceObject, hairKeys, candidates, headPartIsHair);
		if (candidates.empty()) {
			if (!actorState.headPartRecords.empty()) {
				if (!touchedHeadGeometry) {
					ClearPrototypeGroupsByDomainLocked(actorState, PrototypeBuildDomain::kHead);
					ClearPrototypeGroupsByDomainLocked(actorState, PrototypeBuildDomain::kHair);
					actorState.headPartRecords.clear();
				} else if (touchedObjectValid && a_event.object) {
					std::vector<std::uint64_t> removedGroups;
					for (const auto& record : actorState.headPartRecords) {
						if (record.object.get() == a_event.object && record.buildGroup != 0 && std::ranges::find(removedGroups, record.buildGroup) == removedGroups.end()) {
							removedGroups.push_back(record.buildGroup);
						}
					}
					ClearPrototypeGroupsLocked(actorState, removedGroups);
				}
			}
			spdlog::debug(
				"head physics candidate {} actor={} faceNode={} object={} found no XML/defaultBBP head/hair subtrees hairKeys={}",
				ToString(a_event.type),
				static_cast<void*>(a_event.actor),
				static_cast<void*>(faceNode),
				static_cast<void*>(a_event.object),
				hairKeys.size());
			std::erase_if(prototypeActors_, [](const PrototypeActorState& a_state) {
				return !a_state.runtimeSuspended && !a_state.HasRuntime();
			});
			return;
		}

		auto* loader = PhysicsXmlLoader::GetSingleton();
		std::uint32_t built = 0;
		std::uint32_t skippedExisting = 0;
		std::uint32_t skippedDuplicateXml = 0;
		std::vector<std::pair<PrototypeBuildDomain, std::string>> scannedPhysicsFiles;
		for (const auto& candidate : candidates) {
			const auto selectedXml = candidate.path.string();
			if (selectedXml.empty()) {
				continue;
			}
			const auto selectedXmlKey = ConfigPaths::LowerString(selectedXml);
			const auto duplicatePhysicsFile = std::ranges::any_of(scannedPhysicsFiles, [&](const auto& a_entry) {
				return a_entry.first == candidate.domain && a_entry.second == selectedXmlKey;
			});
			if (duplicatePhysicsFile) {
				++skippedDuplicateXml;
				spdlog::debug(
					"previous head part generated {} physics system for XML {}, skipping duplicate object={}",
					PrototypeDomainName(candidate.domain),
					selectedXml,
					static_cast<void*>(candidate.object));
				continue;
			}
			scannedPhysicsFiles.push_back({ candidate.domain, selectedXmlKey });

			const auto existing = std::ranges::find_if(actorState.headPartRecords, [&](const PrototypeHeadPartRecord& a_record) {
				return a_record.object.get() == candidate.object &&
					a_record.sourceObject.get() == candidate.sourceObject.get() &&
					a_record.domain == candidate.domain &&
					ConfigPaths::LowerString(a_record.physicsXmlPath) == selectedXmlKey;
			});
			if (existing != actorState.headPartRecords.end() && existing->buildGroup != 0) {
				++skippedExisting;
				spdlog::debug(
					"geometry is already added as {} head part actor={} object={} XML={} buildGroup={}",
					PrototypeDomainName(candidate.domain),
					static_cast<void*>(a_event.actor),
					static_cast<void*>(candidate.object),
					selectedXml,
					existing->buildGroup);
				continue;
			}

			const auto duplicateTrackedXml = std::ranges::any_of(actorState.headPartRecords, [&](const PrototypeHeadPartRecord& a_record) {
				return a_record.object.get() != candidate.object &&
					a_record.domain == candidate.domain &&
					!a_record.physicsXmlPath.empty() &&
					ConfigPaths::LowerString(a_record.physicsXmlPath) == selectedXmlKey &&
					a_record.buildGroup != 0;
			});
			if (duplicateTrackedXml) {
				++skippedDuplicateXml;
				spdlog::debug(
					"previous tracked head part generated {} physics system for XML {}, skipping duplicate object={}",
					PrototypeDomainName(candidate.domain),
					selectedXml,
					static_cast<void*>(candidate.object));
				continue;
			}

			std::vector<std::uint64_t> replacedGroups;
			for (const auto& record : actorState.headPartRecords) {
				if (record.object.get() == candidate.object && record.buildGroup != 0 && std::ranges::find(replacedGroups, record.buildGroup) == replacedGroups.end()) {
					replacedGroups.push_back(record.buildGroup);
				}
			}
			if (!replacedGroups.empty()) {
				ClearPrototypeGroupsLocked(actorState, replacedGroups);
				spdlog::debug(
					"cleared stale tracked {} headpart groups={} for actor={} object={} before rebuilding XML {}",
					PrototypeDomainName(candidate.domain),
					replacedGroups.size(),
					static_cast<void*>(a_event.actor),
					static_cast<void*>(candidate.object),
					selectedXml);
			}

			spdlog::info(
				"loading {} prototype physics XML {} for actor={} object={}",
				PrototypeDomainName(candidate.domain),
				selectedXml,
				static_cast<void*>(a_event.actor),
				static_cast<void*>(candidate.object));
			const auto selectedSummary = loader->LoadSummary(selectedXml);
			if (!selectedSummary) {
				spdlog::warn("skipping {} physics candidate because selected XML failed to load: {}", PrototypeDomainName(candidate.domain), selectedXml);
				continue;
			}

			auto scopedEvent = a_event;
			scopedEvent.object = candidate.object;
			scopedEvent.sourceObject = candidate.sourceObject.get();
			scopedEvent.sourceRoot = candidate.sourceRoot ? candidate.sourceRoot->IsNode() : nullptr;
			scopedEvent.cloneSourceBeforeTraversal = candidate.cloneSourceBeforeTraversal;
			const auto buildResult = BuildPrototypeBodiesLocked(actorState, scopedEvent, *selectedSummary, candidate.meshNameMap, candidate.domain);
			if (buildResult.succeeded) {
				actorState.headPartRecords.push_back({
					.domain = candidate.domain,
					.physicsXmlPath = selectedXml,
					.object = candidate.object,
					.sourceObject = candidate.sourceObject,
					.sourceRoot = candidate.sourceRoot,
					.buildGroup = buildResult.buildGroup,
				});
				++built;
			}
		}

		spdlog::debug(
			"processed head physics candidate actor={} faceNode={} candidates={} built={} skippedExisting={} skippedDuplicateXml={} trackedHeadParts={} hairKeys={}",
			static_cast<void*>(a_event.actor),
			static_cast<void*>(faceNode),
			candidates.size(),
			built,
			skippedExisting,
			skippedDuplicateXml,
			actorState.headPartRecords.size(),
			hairKeys.size());
		std::erase_if(prototypeActors_, [](const PrototypeActorState& a_state) {
			return !a_state.runtimeSuspended && !a_state.HasRuntime();
		});
	}

	bool Fo4PhysicsWorld::IsPrototypeCandidateLocked(const LifecycleEvent& a_event, const bool a_requireObject)
	{
		if (a_requireObject && !a_event.object) {
			spdlog::trace("skipping prototype physics candidate {} with null object", ToString(a_event.type));
			return false;
		}

		const auto* player = RE::PlayerCharacter::GetSingleton();
		if (!a_event.actor) {
			spdlog::trace("skipping prototype physics candidate {} with null actor", ToString(a_event.type));
			return false;
		}

		if (IsIgnoredFirstPersonEvent(a_event, disableFirstPersonViewPhysics_)) {
			spdlog::debug("skipping first-person prototype physics candidate {}", ToString(a_event.type));
			return false;
		}

		if (a_event.actor == player) {
			return true;
		}

		if (!enableNpcPhysics_) {
			spdlog::trace("skipping prototype physics candidate {} for non-player actor={} because NPC physics is disabled", ToString(a_event.type), static_cast<void*>(a_event.actor));
			return false;
		}

		const auto buildSuspendedArmorCandidate = ShouldBuildSuspendedArmorCandidateLocked(a_event);
		const auto activeActors = static_cast<std::size_t>(std::ranges::count_if(prototypeActors_, [](const PrototypeActorState& a_state) {
			return a_state.HasActiveRuntime();
		}));
		if (FindPrototypeStateLocked(a_event.actor, a_event.firstPerson) == nullptr && activeActors >= currentMaxActiveActors_) {
			if (buildSuspendedArmorCandidate) {
				spdlog::debug(
					"allowing out-of-budget SMP armor candidate {} for actor={} to build directly into soft suspension ({}/{})",
					ToString(a_event.type),
					static_cast<void*>(a_event.actor),
					activeActors,
					currentMaxActiveActors_);
			} else {
				spdlog::debug(
					"skipping prototype physics candidate {} for actor={} because active actor budget is full ({}/{})",
					ToString(a_event.type),
					static_cast<void*>(a_event.actor),
					activeActors,
					currentMaxActiveActors_);
				SuspendActorCandidateLocked(a_event.actor, a_event.firstPerson);
				return false;
			}
		}

		if (!IsActorInReferenceCullView(a_event.actor, a_event.object, a_event.firstPerson)) {
			if (buildSuspendedArmorCandidate) {
				spdlog::debug(
					"allowing inactive-view SMP armor candidate {} for actor={} to build directly into soft suspension",
					ToString(a_event.type),
					static_cast<void*>(a_event.actor));
				return true;
			}
			spdlog::trace(
				"skipping prototype physics candidate {} for actor={} because reference view culler marks it inactive",
				ToString(a_event.type),
				static_cast<void*>(a_event.actor));
			SuspendActorCandidateLocked(a_event.actor, a_event.firstPerson);
			return false;
		}

		return true;
	}
}
