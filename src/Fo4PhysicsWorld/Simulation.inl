// Included by ../Fo4PhysicsWorld.cpp.
// Split from Fo4PhysicsWorld.cpp. Do not compile this file separately.

namespace Smp
{
	void Fo4PhysicsWorld::StepFrame()
	{
		WaitForAsyncStep();
		DrainQueuedLifecycleEvents();
		ProcessPendingRebuilds();
		{
			std::scoped_lock lock(lock_);
			if (disabled_) {
				ResetStepClockLocked();
				return;
			}
		}

		auto delta = fixedStepSeconds_;
		if (const auto timer = RE::BSTimer::GetSingleton()) {
			delta = useRealTime_ ? timer->realTimeDelta : timer->delta;
		}
		if (delta <= 0.0F || !std::isfinite(delta)) {
			return;
		}

		if (IsGamePaused()) {
			std::scoped_lock lock(lock_);
			ResetStepClockLocked();
			return;
		}

		float remainingTimeStep = 0.0F;
		{
			std::scoped_lock lock(lock_);
			if (loadingPhysicsSuspended_) {
				ResumeFromLoadingMenuLocked();
				return;
			}

			accumulatedInterval_ += delta;
			averageInterval_ += (delta - averageInterval_) * 0.125F;
			currentStepSeconds_ = std::min(averageInterval_, fixedStepSeconds_);

			if (accumulatedInterval_ * 2.0F <= currentStepSeconds_) {
				return;
			}

			remainingTimeStep = std::min(
				accumulatedInterval_,
				currentStepSeconds_ * static_cast<float>(std::max(maxSubSteps_, 1)));
			accumulatedInterval_ = 0.0F;
		}

		const auto start = Clock::now();
		Step(remainingTimeStep);
		RecordFrameMetrics(ElapsedMs(start, Clock::now()));
	}

	void Fo4PhysicsWorld::Step(const float a_deltaSeconds)
	{
		WaitForAsyncStep();
		std::scoped_lock lock(lock_);
		if (!dynamicsWorld_ || a_deltaSeconds <= 0.0F) {
			return;
		}

		PruneInvalidPrototypeStatesLocked();
		TryReactivateSuspendedActorsLocked();
		TryReactivateSuspendedPrototypeStatesLocked();

		const auto* player = RE::PlayerCharacter::GetSingleton();
		const auto skipFirstPersonPlayerPhysics = disableFirstPersonViewPhysics_ && IsPlayerFirstPersonView();

		struct ActorReadTask
		{
			PrototypeActorState* actorState{ nullptr };
			float readDelta{ 0.0F };
			RE::NiNode* restoreRoot{ nullptr };
			RE::NiTransform restoreWorld{ RE::NiTransform::IDENTITY };
		};
		const auto processReadTask = [this](const ActorReadTask& a_task) {
			auto& actorState = *a_task.actorState;
			if (!actorState.runtimes.empty()) {
				for (const auto& runtime : actorState.runtimes) {
					for (auto* bone : runtime.bones) {
						if (bone) {
							bone->readTransform(a_task.readDelta);
						}
					}
					ScalePrototypeConstraintsLocked(actorState, runtime);
				}
			} else {
				for (auto& prototypeBody : actorState.bodies) {
					if (prototypeBody.bone) {
						prototypeBody.bone->readTransform(a_task.readDelta);
					}
				}
				ScalePrototypeConstraintsLocked(actorState);
			}
		};

		auto phaseStart = Clock::now();
		std::vector<ActorReadTask> readTasks;
		readTasks.reserve(prototypeActors_.size());
		for (auto& actorState : prototypeActors_) {
			if (actorState.runtimeSoftSuspended) {
				continue;
			}
			if (skipFirstPersonPlayerPhysics && actorState.actor == player) {
				actorState.currentWindFactor = 0.0F;
				continue;
			}
			UpdateMeshDisableStatesLocked(actorState);
			for (auto& runtime : actorState.runtimes) {
				if (!runtime.pendingResetPhysicsRead) {
					continue;
				}

				ResetPrototypeBuildGroupToCurrentPoseLocked(actorState, runtime.buildGroup);
				runtime.pendingResetPhysicsRead = false;
				spdlog::debug(
					"performed one-shot reset physics read for newly committed prototype build group actor={} buildGroup={} domain={} bipedObject={}",
					static_cast<void*>(actorState.actor),
					runtime.buildGroup,
					PrototypeDomainName(runtime.domain),
					std::to_underlying(runtime.bipedObject));
			}
			const auto preparation = PreparePrototypeActorForReadLocked(actorState, a_deltaSeconds);
			readTasks.push_back({
				.actorState = std::addressof(actorState),
				.readDelta = preparation.timeStep,
				.restoreRoot = preparation.restoreRoot,
				.restoreWorld = preparation.restoreWorld,
			});
		}
		tbb::parallel_for(std::size_t{ 0 }, readTasks.size(), [&](const std::size_t a_index) {
			processReadTask(readTasks[a_index]);
		});
		for (const auto& readTask : readTasks) {
			if (readTask.restoreRoot) {
				readTask.restoreRoot->world = readTask.restoreWorld;
				for (auto& child : readTask.restoreRoot->children) {
					if (child) {
						UpdateTransformUpDown(child.get(), true);
					}
				}
			}
		}
		const auto readMs = ElapsedMs(phaseStart, Clock::now());

		phaseStart = Clock::now();
		UpdateWindLocked();
		ApplyWindForcesLocked();
		const auto windMs = ElapsedMs(phaseStart, Clock::now());

		const auto fixedStepSeconds = std::clamp(currentStepSeconds_, kMinimumStepSeconds, fixedStepSeconds_);
		const auto maximumStepSeconds = std::max(fixedStepSeconds, fixedStepSeconds * static_cast<float>(std::max(maxSubSteps_, 1)));
		const auto clampedDelta = std::clamp(a_deltaSeconds, kMinimumStepSeconds, maximumStepSeconds);

		pendingStepReadMs_ += readMs;
		pendingStepWindMs_ += windMs;
		if (!asyncStepState_) {
			asyncStepState_ = std::make_unique<AsyncStepState>();
		}
		asyncStepState_->tasks.run([this, clampedDelta, fixedStepSeconds]() {
			RunSecondStepLocked(clampedDelta, fixedStepSeconds);
		});
	}

	void Fo4PhysicsWorld::RunSecondStepLocked(const float a_deltaSeconds, const float a_fixedStepSeconds)
	{
		std::scoped_lock lock(lock_);
		if (!dynamicsWorld_ || a_deltaSeconds <= 0.0F || a_fixedStepSeconds <= 0.0F) {
			return;
		}

		ResetFrameCollisionProfile();
		auto phaseStart = Clock::now();
		const auto translationOffset = ApplyTranslationOffset(*dynamicsWorld_);
		if (auto* world = static_cast<PrototypeDynamicsWorld*>(dynamicsWorld_.get())) {
			world->StepReference(a_deltaSeconds, a_fixedStepSeconds);
		}
		RestoreTranslationOffset(*dynamicsWorld_, translationOffset);
		if (!translationOffset.fuzzyZero()) {
			RefreshSkinnedMeshWorldState(*dynamicsWorld_);
		}
		const auto bulletMs = ElapsedMs(phaseStart, Clock::now());
		std::uint32_t collisionCalls = 0;
		const auto collisionMs = ConsumeFrameCollisionProfile(collisionCalls);

		pendingStepBulletMs_ += bulletMs;
		pendingStepCollisionMs_ += collisionMs;
		pendingStepCollisionCalls_ += collisionCalls;

		++simulationFrame_;
		if (simulationFrame_ == 0) {
			simulationFrame_ = 1;
		}
		PhysicsProfiler::AdvanceFrame();
	}

	void Fo4PhysicsWorld::LogRootConstraintDiagnosticsLocked(const std::string_view a_phase, const PrototypeActorState& a_state)
	{
		const PrototypeConstraint* selected = nullptr;
		for (const auto& prototypeConstraint : a_state.constraints) {
			if (!prototypeConstraint.constraint || prototypeConstraint.kind != PhysicsConstraintKind::kGeneric) {
				continue;
			}

			const auto& bodyA = prototypeConstraint.constraint->getRigidBodyA();
			const auto& bodyB = prototypeConstraint.constraint->getRigidBodyB();
			if (bodyA.isStaticOrKinematicObject() != bodyB.isStaticOrKinematicObject()) {
				selected = std::addressof(prototypeConstraint);
				break;
			}
			if (!selected) {
				selected = std::addressof(prototypeConstraint);
			}
		}
		if (!selected || !selected->constraint) {
			return;
		}

		auto* generic = static_cast<btGeneric6DofSpring2Constraint*>(selected->constraint.get());
		const auto& bodyA = generic->getRigidBodyA();
		const auto& bodyB = generic->getRigidBodyB();
		const auto bodyATransform = bodyA.getWorldTransform();
		const auto bodyBTransform = bodyB.getWorldTransform();
		const auto anchorA = bodyATransform * generic->getFrameOffsetA();
		const auto anchorB = bodyBTransform * generic->getFrameOffsetB();
		const auto bodyAOrigin = bodyATransform.getOrigin();
		const auto bodyBOrigin = bodyBTransform.getOrigin();
		const auto anchorAOrigin = anchorA.getOrigin();
		const auto anchorBOrigin = anchorB.getOrigin();
		const auto anchorDelta = anchorBOrigin - anchorAOrigin;

		spdlog::info(
			"prototype root constraint diagnostic {} actor={} bodies='{}'/'{}' enabled={} bodyAkin={} bodyBkin={} bodyA=({:.3f},{:.3f},{:.3f}) bodyB=({:.3f},{:.3f},{:.3f}) anchorA=({:.3f},{:.3f},{:.3f}) anchorB=({:.3f},{:.3f},{:.3f}) anchorDelta=({:.3f},{:.3f},{:.3f}) velA=({:.3f},{:.3f},{:.3f}) velB=({:.3f},{:.3f},{:.3f})",
			a_phase,
			static_cast<void*>(a_state.actor),
			selected->bodyA,
			selected->bodyB,
			generic->isEnabled(),
			bodyA.isStaticOrKinematicObject(),
			bodyB.isStaticOrKinematicObject(),
			bodyAOrigin.x(),
			bodyAOrigin.y(),
			bodyAOrigin.z(),
			bodyBOrigin.x(),
			bodyBOrigin.y(),
			bodyBOrigin.z(),
			anchorAOrigin.x(),
			anchorAOrigin.y(),
			anchorAOrigin.z(),
			anchorBOrigin.x(),
			anchorBOrigin.y(),
			anchorBOrigin.z(),
			anchorDelta.x(),
			anchorDelta.y(),
			anchorDelta.z(),
			bodyA.getLinearVelocity().x(),
			bodyA.getLinearVelocity().y(),
			bodyA.getLinearVelocity().z(),
			bodyB.getLinearVelocity().x(),
			bodyB.getLinearVelocity().y(),
			bodyB.getLinearVelocity().z());
	}

	void Fo4PhysicsWorld::UpdateMeshDisableStatesLocked(PrototypeActorState& a_state)
	{
		struct DisableGroup
		{
			RE::BSFixedString tag;
			std::vector<hdt::SkinnedMeshBody*> bodies;
		};

		std::vector<RE::BSFixedString> activeTags;
		std::vector<DisableGroup> disableGroups;
		const auto disableHairForWig = disableSMPHairWhenWigEquipped_ && HasEquippedHairSlotObject(a_state.actor);

		for (auto& prototypeMesh : a_state.meshes) {
			auto* body = prototypeMesh.body.get();
			if (!body) {
				continue;
			}

			body->disabled_ = false;
			if (disableHairForWig && prototypeMesh.domain == PrototypeBuildDomain::kHair) {
				body->disabled_ = true;
				continue;
			}
			if (body->disableTag_.empty()) {
				for (const auto& tag : body->tags_) {
					if (std::ranges::find(activeTags, tag) == activeTags.end()) {
						activeTags.push_back(tag);
					}
				}
				continue;
			}

			const auto foundGroup = std::ranges::find_if(disableGroups, [body](const DisableGroup& a_group) {
				return a_group.tag == body->disableTag_;
			});
			if (foundGroup != disableGroups.end()) {
				foundGroup->bodies.push_back(body);
			} else {
				auto& group = disableGroups.emplace_back();
				group.tag = body->disableTag_;
				group.bodies.push_back(body);
			}
		}

		for (auto& group : disableGroups) {
			if (std::ranges::find(activeTags, group.tag) != activeTags.end()) {
				for (auto* body : group.bodies) {
					body->disabled_ = true;
				}
				continue;
			}

			std::ranges::sort(group.bodies, [](const hdt::SkinnedMeshBody* a_lhs, const hdt::SkinnedMeshBody* a_rhs) {
				if (a_lhs->disablePriority_ != a_rhs->disablePriority_) {
					return a_lhs->disablePriority_ > a_rhs->disablePriority_;
				}
				return a_lhs < a_rhs;
			});
			for (auto* body : group.bodies) {
				body->disabled_ = true;
			}
			if (!group.bodies.empty()) {
				group.bodies.front()->disabled_ = false;
			}
		}
	}

	void Fo4PhysicsWorld::UpdateWindLocked()
	{
		if (!windEnabled_ || windStrength_ <= 0.0F) {
			currentWind_.setZero();
			targetWind_.setZero();
			return;
		}

		auto direction = windDirection_;
		auto strength = windStrength_;
		if (windUseWeather_) {
			const auto* sky = RE::Sky::GetSingleton();
			auto* player = RE::PlayerCharacter::GetSingleton();
			auto* cell = player ? player->GetParentCell() : nullptr;
			if (!IsWeatherWindSkyValid(sky) || !player || !cell || !cell->IsExterior() || !cell->worldSpace) {
				ClearWindState(currentWind_, targetWind_, windWeatherCooldown_, windWeatherLongCooldownSeconds_);
				return;
			}

			windWeatherCooldown_ -= currentStepSeconds_;
			if (windWeatherCooldown_ <= 0.0F) {
				direction = WindDirectionFromFo4SkyAngle(sky->windAngle);
				strength *= std::max(sky->windSpeed, 0.0F);
				targetWind_ = direction * strength * kGameUnitsPerMeter;
				windWeatherCooldown_ = windWeatherShortCooldownSeconds_;
			}
		} else {
			targetWind_ = direction * strength * kGameUnitsPerMeter;
		}

		const auto smoothingSamples = static_cast<float>(std::max(windSmoothingSamples_, 1));
		if (smoothingSamples <= 1.0F) {
			currentWind_ = targetWind_;
		} else {
			currentWind_ += (targetWind_ - currentWind_) / smoothingSamples;
		}
	}

	void Fo4PhysicsWorld::ApplyWindForcesLocked()
	{
		if (currentWind_.length2() <= SIMD_EPSILON) {
			for (auto& actorState : prototypeActors_) {
				actorState.currentWindFactor = 0.0F;
			}
			return;
		}

		for (auto& actorState : prototypeActors_) {
			if (actorState.runtimeSoftSuspended) {
				actorState.currentWindFactor = 0.0F;
				continue;
			}
			if (windUseWeather_ && !IsActorWeatherWindCellValid(actorState.actor)) {
				actorState.currentWindFactor = 0.0F;
				continue;
			}

			const auto targetActorWindScale = ResolveActorWindObstructionFactor(actorState.actor, currentWind_, windDistanceForNoWind_, windDistanceForMaxWind_);
			actorState.currentWindFactor += (targetActorWindScale - actorState.currentWindFactor) / static_cast<float>(std::max(windSmoothingSamples_, 1));
			if (std::abs(actorState.currentWindFactor - targetActorWindScale) <= 0.001F) {
				actorState.currentWindFactor = targetActorWindScale;
			}
			const auto actorWindScale = std::clamp(actorState.currentWindFactor, 0.0F, 1.0F);
			if (actorWindScale <= 0.0F) {
				continue;
			}

			const auto actorWind = currentWind_ * actorWindScale;
			for (auto& prototypeBody : actorState.bodies) {
				if (!prototypeBody.bone || prototypeBody.bone->m_windFactor <= 0.0F || prototypeBody.bone->m_rig.isStaticOrKinematicObject()) {
					continue;
				}

				const auto boneWind = randomizePerBoneWind_ ?
					actorWind * StableWindVariation(prototypeBody.boneName) :
					actorWind;
				prototypeBody.bone->m_rig.applyCentralForce(boneWind * prototypeBody.bone->m_windFactor);
			}
		}
	}

	void Fo4PhysicsWorld::RecordFrameMetrics(const float a_stepMs)
	{
		std::scoped_lock lock(lock_);
		if (prototypeActors_.empty()) {
			pendingWritebackMs_ = 0.0F;
			pendingMainSyncMs_ = 0.0F;
			pendingStepReadMs_ = 0.0F;
			pendingStepWindMs_ = 0.0F;
			pendingStepBulletMs_ = 0.0F;
			pendingStepCollisionMs_ = 0.0F;
			pendingStepCollisionCalls_ = 0;
			currentMaxActiveActors_ = maxActiveActors_;
			metricFrameCounter_ = 0;
			averageStepMs_ = 0.0F;
			averageWritebackMs_ = 0.0F;
			averageMainSyncMs_ = 0.0F;
			averageStepReadMs_ = 0.0F;
			averageStepWindMs_ = 0.0F;
			averageStepBulletMs_ = 0.0F;
			averageStepCollisionMs_ = 0.0F;
			return;
		}

		const auto sampleWeight = static_cast<float>(sampleSize_);
		averageStepMs_ = ((averageStepMs_ * (sampleWeight - 1.0F)) + std::max(a_stepMs, 0.0F)) / sampleWeight;
		averageWritebackMs_ = ((averageWritebackMs_ * (sampleWeight - 1.0F)) + std::max(pendingWritebackMs_, 0.0F)) / sampleWeight;
		averageMainSyncMs_ = ((averageMainSyncMs_ * (sampleWeight - 1.0F)) + std::max(pendingMainSyncMs_, 0.0F)) / sampleWeight;
		averageStepReadMs_ = ((averageStepReadMs_ * (sampleWeight - 1.0F)) + std::max(pendingStepReadMs_, 0.0F)) / sampleWeight;
		averageStepWindMs_ = ((averageStepWindMs_ * (sampleWeight - 1.0F)) + std::max(pendingStepWindMs_, 0.0F)) / sampleWeight;
		averageStepBulletMs_ = ((averageStepBulletMs_ * (sampleWeight - 1.0F)) + std::max(pendingStepBulletMs_, 0.0F)) / sampleWeight;
		averageStepCollisionMs_ = ((averageStepCollisionMs_ * (sampleWeight - 1.0F)) + std::max(pendingStepCollisionMs_, 0.0F)) / sampleWeight;

		pendingWritebackMs_ = 0.0F;
		pendingMainSyncMs_ = 0.0F;
		pendingStepReadMs_ = 0.0F;
		pendingStepWindMs_ = 0.0F;
		pendingStepBulletMs_ = 0.0F;
		pendingStepCollisionMs_ = 0.0F;

		++metricFrameCounter_;
		if (metricFrameCounter_ < metricFrameInterval_) {
			return;
		}
		metricFrameCounter_ = 0;
		const auto collisionCalls = pendingStepCollisionCalls_;
		pendingStepCollisionCalls_ = 0;

		std::size_t bodyCount = 0;
		std::size_t meshCount = 0;
		std::size_t activeActors = 0;
		for (const auto& actorState : prototypeActors_) {
			bodyCount += actorState.bodies.size();
			meshCount += actorState.meshes.size();
			if (actorState.HasActiveRuntime()) {
				++activeActors;
			}
		}

		const auto totalMs = averageStepMs_ + averageWritebackMs_;
		if (autoAdjustMaxActors_) {
			if (totalMs > budgetMs_ && currentMaxActiveActors_ > 1) {
				--currentMaxActiveActors_;
			} else if (totalMs < budgetMs_ && currentMaxActiveActors_ < maxActiveActors_) {
				const auto averagePerActor = activeActors > 0 ? totalMs / static_cast<float>(activeActors) : 0.0F;
				const auto headroom = budgetMs_ - totalMs;
				const auto canAdd = averagePerActor > 0.0F ? static_cast<std::size_t>(std::max(headroom / averagePerActor, 0.0F)) : 2U;
				currentMaxActiveActors_ += std::clamp<std::size_t>(canAdd, 0, 2);
				currentMaxActiveActors_ = std::min(currentMaxActiveActors_, maxActiveActors_);
			}
		} else {
			currentMaxActiveActors_ = maxActiveActors_;
		}
		EnforceActorBudgetLocked();

		if (totalMs > budgetMs_) {
			spdlog::debug(
				"[SMP Metrics] activeActors={} actorCap={}/{} bodies={} meshes={} avgFrameImpact={:.3f}ms budget={:.3f}ms step={:.3f}ms sync={:.3f}ms writeback={:.3f}ms stepRead={:.3f}ms stepWind={:.3f}ms stepBullet={:.3f}ms collision={:.3f}ms collisionCalls={} writes(mainSync/cellJobs/postAnim)={}/{}/{} duplicateSkips(cellJobs/postAnim)={}/{}",
				activeActors,
				currentMaxActiveActors_,
				maxActiveActors_,
				bodyCount,
				meshCount,
				totalMs,
				budgetMs_,
				averageStepMs_,
				averageMainSyncMs_,
				averageWritebackMs_,
				averageStepReadMs_,
				averageStepWindMs_,
				averageStepBulletMs_,
				averageStepCollisionMs_,
				collisionCalls,
				mainSyncWritebacks_,
				cellJobsWritebacks_,
				postAnimationWritebacks_,
				duplicateCellJobsWritebacks_,
				duplicatePostAnimationWritebacks_);
		} else {
			spdlog::trace(
				"[SMP Metrics] activeActors={} actorCap={}/{} bodies={} meshes={} avgFrameImpact={:.3f}ms budget={:.3f}ms step={:.3f}ms sync={:.3f}ms writeback={:.3f}ms stepRead={:.3f}ms stepWind={:.3f}ms stepBullet={:.3f}ms collision={:.3f}ms collisionCalls={} writes(mainSync/cellJobs/postAnim)={}/{}/{} duplicateSkips(cellJobs/postAnim)={}/{}",
				activeActors,
				currentMaxActiveActors_,
				maxActiveActors_,
				bodyCount,
				meshCount,
				totalMs,
				budgetMs_,
				averageStepMs_,
				averageMainSyncMs_,
				averageWritebackMs_,
				averageStepReadMs_,
				averageStepWindMs_,
				averageStepBulletMs_,
				averageStepCollisionMs_,
				collisionCalls,
				mainSyncWritebacks_,
				cellJobsWritebacks_,
				postAnimationWritebacks_,
				duplicateCellJobsWritebacks_,
				duplicatePostAnimationWritebacks_);
		}

		mainSyncWritebacks_ = 0;
		cellJobsWritebacks_ = 0;
		postAnimationWritebacks_ = 0;
		duplicateCellJobsWritebacks_ = 0;
		duplicatePostAnimationWritebacks_ = 0;
	}

	void Fo4PhysicsWorld::RecordWritebackMetric(
		const float a_writebackMs,
		const WritebackSource a_source,
		const bool a_wroteAny,
		const bool a_skippedDuplicate)
	{
		std::scoped_lock lock(lock_);
		if (a_wroteAny) {
			pendingWritebackMs_ += std::max(a_writebackMs, 0.0F);
			if (a_source == WritebackSource::kMainSync) {
				pendingMainSyncMs_ += std::max(a_writebackMs, 0.0F);
				++mainSyncWritebacks_;
			}
			IncrementWritebackCounter(a_source, cellJobsWritebacks_, postAnimationWritebacks_);
		}
		if (a_skippedDuplicate) {
			IncrementWritebackCounter(a_source, duplicateCellJobsWritebacks_, duplicatePostAnimationWritebacks_);
		}
	}

	void Fo4PhysicsWorld::WriteBackPrototypeBodies(const WritebackSource a_source)
	{
		const auto start = Clock::now();
		bool wroteAny = false;
		bool skippedDuplicate = false;
		WaitForAsyncStep();
		{
			std::scoped_lock lock(lock_);
			if (loadingPhysicsSuspended_) {
				ResetStepClockLocked();
				return;
			}

			PruneInvalidPrototypeStatesLocked();
			const auto* player = RE::PlayerCharacter::GetSingleton();
			const auto skipFirstPersonPlayerPhysics = disableFirstPersonViewPhysics_ && IsPlayerFirstPersonView();

			for (auto& actorState : prototypeActors_) {
				if (actorState.runtimeSoftSuspended) {
					continue;
				}
				if (skipFirstPersonPlayerPhysics && actorState.actor == player) {
					continue;
				}
				const auto resetReadPending = std::ranges::any_of(actorState.runtimes, [](const PrototypeBuildGroupRuntime& a_runtime) {
						return a_runtime.pendingResetPhysicsRead;
					});
				if (resetReadPending) {
					skippedDuplicate = true;
					spdlog::trace(
						"skipping prototype writeback until pending reset physics read completes actor={}",
						static_cast<void*>(actorState.actor));
					continue;
				}
				if (!CanWriteBackFrame(actorState.lastWritebackFrame, a_source, simulationFrame_)) {
					skippedDuplicate = true;
					continue;
				}
				actorState.lastWritebackFrame = simulationFrame_;
				actorState.lastWritebackSource = a_source;
				std::vector<RE::NiNode*> writtenNodes;
				const auto writeBody = [&](PrototypeBody& prototypeBody) {
					if (!prototypeBody.bone || prototypeBody.bone->m_rig.isKinematicObject()) {
						return;
					}
					if (prototypeBody.node && std::ranges::find(writtenNodes, prototypeBody.node) != writtenNodes.end()) {
						skippedDuplicate = true;
						spdlog::warn(
							"skipping duplicate dynamic prototype writeback actor={} node={} nodeName='{}' buildGroup={} bone='{}' source={}",
							static_cast<void*>(actorState.actor),
							static_cast<void*>(prototypeBody.node),
							std::string_view(prototypeBody.node->GetName()),
							prototypeBody.buildGroup,
							prototypeBody.boneName,
							WritebackSourceName(a_source));
						return;
					}
					prototypeBody.bone->writeTransform();
					if (prototypeBody.node) {
						writtenNodes.push_back(prototypeBody.node);
					}
					wroteAny = true;
				};
				if (!actorState.runtimes.empty()) {
					for (const auto& runtime : actorState.runtimes) {
						for (auto* bone : runtime.bones) {
							auto body = std::ranges::find_if(actorState.bodies, [bone](const PrototypeBody& a_body) {
								return a_body.bone.get() == bone;
							});
							if (body == actorState.bodies.end()) {
								continue;
							}
							writeBody(*body);
						}
					}
				} else {
					for (auto& prototypeBody : actorState.bodies) {
						writeBody(prototypeBody);
					}
				}
			}
		}
		RecordWritebackMetric(ElapsedMs(start, Clock::now()), a_source, wroteAny, skippedDuplicate);
	}

	void Fo4PhysicsWorld::WriteBackPrototypeBodies(RE::Actor* a_actor, const WritebackSource a_source)
	{
		const auto start = Clock::now();
		bool wroteAny = false;
		bool skippedDuplicate = false;
		WaitForAsyncStep();
		{
			std::scoped_lock lock(lock_);
			if (loadingPhysicsSuspended_) {
				ResetStepClockLocked();
				return;
			}

			PruneInvalidPrototypeStatesLocked();
			const auto* player = RE::PlayerCharacter::GetSingleton();
			const auto skipFirstPersonPlayerPhysics = disableFirstPersonViewPhysics_ && IsPlayerFirstPersonView();

			for (auto& actorState : prototypeActors_) {
				if (actorState.actor != a_actor) {
					continue;
				}
				if (actorState.runtimeSoftSuspended) {
					continue;
				}
				if (skipFirstPersonPlayerPhysics && actorState.actor == player) {
					continue;
				}

				const auto resetReadPending = std::ranges::any_of(actorState.runtimes, [](const PrototypeBuildGroupRuntime& a_runtime) {
						return a_runtime.pendingResetPhysicsRead;
					});
				if (resetReadPending) {
					skippedDuplicate = true;
					spdlog::trace(
						"skipping targeted prototype writeback until pending reset physics read completes actor={}",
						static_cast<void*>(actorState.actor));
					continue;
				}
				if (CanWriteBackFrame(actorState.lastWritebackFrame, a_source, simulationFrame_)) {
					actorState.lastWritebackFrame = simulationFrame_;
					actorState.lastWritebackSource = a_source;
					std::vector<RE::NiNode*> writtenNodes;
					const auto writeBody = [&](PrototypeBody& prototypeBody) {
						if (!prototypeBody.bone || prototypeBody.bone->m_rig.isKinematicObject()) {
							return;
						}
						if (prototypeBody.node && std::ranges::find(writtenNodes, prototypeBody.node) != writtenNodes.end()) {
							skippedDuplicate = true;
							spdlog::warn(
								"skipping duplicate dynamic prototype writeback actor={} node={} nodeName='{}' buildGroup={} bone='{}' source={}",
								static_cast<void*>(actorState.actor),
								static_cast<void*>(prototypeBody.node),
								std::string_view(prototypeBody.node->GetName()),
								prototypeBody.buildGroup,
								prototypeBody.boneName,
								WritebackSourceName(a_source));
							return;
						}
						prototypeBody.bone->writeTransform();
						if (prototypeBody.node) {
							writtenNodes.push_back(prototypeBody.node);
						}
						wroteAny = true;
					};
					if (!actorState.runtimes.empty()) {
						for (const auto& runtime : actorState.runtimes) {
							for (auto* bone : runtime.bones) {
								auto body = std::ranges::find_if(actorState.bodies, [bone](const PrototypeBody& a_body) {
									return a_body.bone.get() == bone;
								});
								if (body == actorState.bodies.end()) {
									continue;
								}
								writeBody(*body);
							}
						}
					} else {
						for (auto& prototypeBody : actorState.bodies) {
							writeBody(prototypeBody);
						}
					}
				} else {
					skippedDuplicate = true;
				}
			}
		}
		RecordWritebackMetric(ElapsedMs(start, Clock::now()), a_source, wroteAny, skippedDuplicate);
	}

	void Fo4PhysicsWorld::ProcessPendingRebuilds()
	{
		WaitForAsyncStep();
		std::scoped_lock lock(lock_);
		if (loadingPhysicsSuspended_) {
			return;
		}

		if (pendingActorRebuilds_.empty() && pendingHeadRebuilds_.empty()) {
			return;
		}

		TryRebuildPendingActorsLocked();
		TryRebuildPendingHeadsLocked();
	}
}
