// Included by ../Fo4PhysicsWorld.cpp.
// Split from Fo4PhysicsWorld.cpp. Do not compile this file separately.

namespace Smp
{
	struct Fo4PhysicsWorld::AsyncStepState
	{
		tbb::task_group tasks;
	};

	Fo4PhysicsWorld::~Fo4PhysicsWorld() noexcept
	{
		try {
			Reset();
		} catch (...) {
		}
	}

	Fo4PhysicsWorld* Fo4PhysicsWorld::GetSingleton()
	{
		static Fo4PhysicsWorld singleton;
		return std::addressof(singleton);
	}

	void Fo4PhysicsWorld::Register()
	{
		if (registered_) {
			return;
		}
		if (Initialize()) {
			GetLifecycleEventSource().RegisterSink(static_cast<RE::BSTEventSink<LifecycleEvent>*>(this));
			if (auto* ui = RE::UI::GetSingleton()) {
				ui->RegisterSink<RE::MenuOpenCloseEvent>(static_cast<RE::BSTEventSink<RE::MenuOpenCloseEvent>*>(this));
			} else {
				spdlog::warn("FO4 Faster HDT-SMP could not register UI menu sink; LooksMenu rebuild deferral will be unavailable");
			}
			registered_ = true;
		}
	}

	bool Fo4PhysicsWorld::Initialize()
	{
		WaitForAsyncStep();
		std::scoped_lock lock(lock_);
		return InitializeLocked();
	}

	void Fo4PhysicsWorld::ApplyConfig(const RuntimeSettings& a_settings)
	{
		WaitForAsyncStep();
		std::scoped_lock lock(lock_);
		solverIterations_ = a_settings.solver.numIterations;
		solverErp_ = a_settings.solver.erp;
		maxSubSteps_ = a_settings.solver.maxSubSteps;
		fixedStepSeconds_ = 1.0F / static_cast<float>(a_settings.solver.minFps);
		ResetStepClockLocked();
		useRealTime_ = a_settings.smp.useRealTime;
		budgetMs_ = std::max(a_settings.smp.budgetMs, 0.1F);
		sampleSize_ = std::max(a_settings.smp.sampleSize, 1);
		metricFrameInterval_ = static_cast<std::uint32_t>(std::max(a_settings.solver.minFps, 1));
		clampRotations_ = a_settings.smp.clampRotations;
		rotationSpeedLimit_ = std::max(a_settings.smp.rotationSpeedLimit, 0.0F);
		unclampedResets_ = a_settings.smp.unclampedResets;
		unclampedResetAngle_ = std::max(a_settings.smp.unclampedResetAngle, 0.0F);
		disableFirstPersonViewPhysics_ = a_settings.smp.disableFirstPersonViewPhysics;
		disableSMPHairWhenWigEquipped_ = a_settings.smp.disableSMPHairWhenWigEquipped;
		enableNpcPhysics_ = a_settings.smp.enableNpcPhysics;
		autoAdjustMaxActors_ = a_settings.smp.autoAdjustMaxActors;
		maxActiveActors_ = static_cast<std::size_t>(std::max(a_settings.smp.maxActiveActors, 1));
		currentMaxActiveActors_ = maxActiveActors_;
		maxActorDistance_ = std::max(a_settings.smp.maxActorDistance, 0.0F);
		fallbackPhysicsXml_.clear();
		if (!a_settings.smp.fallbackPhysicsXml.empty()) {
			if (auto resolved = ConfigPaths::ResolveExistingConfigPath(a_settings.smp.fallbackPhysicsXml, true)) {
				fallbackPhysicsXml_ = resolved->string();
			} else {
				spdlog::warn("physics XML fallback disabled because configured path is missing or not XML: {}", a_settings.smp.fallbackPhysicsXml);
			}
		}
		windEnabled_ = a_settings.wind.enabled;
		windUseWeather_ = a_settings.wind.useWeather;
		windStrength_ = std::max(a_settings.wind.windStrength, 0.0F);
		windDistanceForNoWind_ = std::max(a_settings.wind.distanceForNoWind, 0.0F);
		windDistanceForMaxWind_ = std::max(a_settings.wind.distanceForMaxWind, windDistanceForNoWind_);
		windWeatherShortCooldownSeconds_ = std::max(a_settings.wind.weatherShortCooldownSeconds, 0.0F);
		windWeatherLongCooldownSeconds_ = std::max(a_settings.wind.weatherLongCooldownSeconds, windWeatherShortCooldownSeconds_);
		windSmoothingSamples_ = std::max(a_settings.wind.smoothingSamples, 1);
		randomizePerBoneWind_ = a_settings.wind.randomizePerBoneWind;
		windDirection_ = btVector3(a_settings.wind.directionX, a_settings.wind.directionY, a_settings.wind.directionZ);
		if (windDirection_.length2() > SIMD_EPSILON) {
			windDirection_.normalize();
		} else {
			windDirection_ = btVector3(1.0F, 0.0F, 0.0F);
		}

		if (dynamicsWorld_) {
			auto& solverInfo = dynamicsWorld_->getSolverInfo();
			solverInfo.m_numIterations = solverIterations_;
			solverInfo.m_erp = solverErp_;
		}
		EnforceActorBudgetLocked();
	}

	void Fo4PhysicsWorld::DrawBulletVisualization()
	{
		WaitForAsyncStep();
		std::scoped_lock lock(lock_);
		Smp::BulletVisualization::DrawWorld(dynamicsWorld_.get());
	}

	void Fo4PhysicsWorld::Reset()
	{
		WaitForAsyncStep();
		std::scoped_lock lock(lock_);
		ResetLocked();
	}

	void Fo4PhysicsWorld::BeginSaveLoad()
	{
		WaitForAsyncStep();

		std::size_t clearedSystems = 0;
		std::size_t preservedHeadCaches = 0;
		std::size_t dirtyActors = 0;
		std::size_t discardedEvents = 0;
		{
			std::scoped_lock lock(lock_);
			saveLoadInProgress_ = true;
			loadingPhysicsSuspended_ = true;
			loadingMenuDepth_ = 0;
			saveLoadActors_.clear();
			discardedEvents = DiscardAllQueuedLifecycleEvents();

			std::vector<RetainedHeadSkeletonCache> retainedHeadCaches;
			for (const auto& actorStatePointer : systems_) {
				const auto& actorState = *actorStatePointer;
				auto* actor = actorState.actor;
				auto* faceNode = actor && !actorState.firstPerson ? actor->GetFaceNodeSkinned() : nullptr;
				if (!actor || !faceNode || actorState.headPartRecords.empty() || !actorState.actorHandle) {
					continue;
				}

				RetainedHeadSkeletonCache cache{
					.actorHandle = actorState.actorHandle,
					.retainedFaceIdentity = reinterpret_cast<std::uintptr_t>(faceNode),
				};
				for (const auto& record : actorState.headPartRecords) {
					for (const auto& boneName : record.requiredBoneNames) {
						MergePhysicsName(cache.requiredHeadBoneNames, boneName);
					}
					for (const auto& reference : record.boneReferences) {
						MergeBoneReferenceRecipe(cache.headBoneReferences, reference);
					}
				}
				if (!cache.headBoneReferences.empty()) {
					retainedHeadCaches.push_back(std::move(cache));
				}
			}

			clearedSystems = systems_.size();
			ClearAllSystemsLocked();
			retainedHeadSkeletonCaches_ = std::move(retainedHeadCaches);
			preservedHeadCaches = retainedHeadSkeletonCaches_.size();
			pendingActorRebuilds_.clear();
			pendingHeadRebuilds_.clear();
			pendingSkeletonTransitions_.clear();
			characterCustomizationMenuDepth_ = 0;
			characterCustomizationTarget_.reset();

			if (auto* player = RE::PlayerCharacter::GetSingleton()) {
				NoteSaveLoadActorLocked(LifecycleEvent{
					.type = LifecycleEventType::kActorSet3D,
					.actor = player,
					.object = player->Get3D(false),
					.firstPerson = false,
				});
			}

			if (dynamicsWorld_) {
				dynamicsWorld_->clearForces();
			}
			dirtyActors = saveLoadActors_.size();
			ResetStepClockLocked();
		}

		spdlog::debug(
			"began save-load physics transaction clearedSystems={} preservedHeadCaches={} dirtyActors={} discardedQueuedLifecycleEvents={}",
			clearedSystems,
			preservedHeadCaches,
			dirtyActors,
			discardedEvents);
	}

	void Fo4PhysicsWorld::ResetSystems()
	{
		WaitForAsyncStep();
		std::vector<std::pair<RE::Actor*, RE::ActorHandle>> actors;
		{
			std::scoped_lock lock(lock_);
			for (auto& actorStatePointer : systems_) {
				auto& actorState = *actorStatePointer;
				for (auto& runtime : actorState.buildGroups) {
					runtime.pendingResetPhysicsRead = true;
					runtime.pendingResetPhysicsWriteback = true;
				}
				if (actorState.actor &&
					std::ranges::none_of(actors, [&](const auto& a_entry) {
						return a_entry.first == actorState.actor;
					})) {
					actors.emplace_back(actorState.actor, actorState.actorHandle);
				}
			}
			ResetStepClockLocked();
		}

		for (const auto& [actor, handle] : actors) {
			const auto resolved = handle.get();
			if (resolved && resolved.get() == actor) {
				ResetActorPhysics(resolved.get(), true);
			}
		}
	}

	void Fo4PhysicsWorld::SetDisabled(const bool a_disabled)
	{
		WaitForAsyncStep();
		std::scoped_lock lock(lock_);
		disabled_ = a_disabled;
		ResetStepClockLocked();
	}

	bool Fo4PhysicsWorld::IsDisabled()
	{
		std::scoped_lock lock(lock_);
		return disabled_;
	}

	void Fo4PhysicsWorld::SetProfilerCapture(
		const bool a_enabled,
		const std::uint64_t a_sampleFrames,
		const std::uint64_t a_printFrames)
	{
		WaitForAsyncStep();
		std::scoped_lock lock(lock_);
		PhysicsProfiler::SetCapture(a_enabled, a_sampleFrames, a_printFrames);
	}

	void Fo4PhysicsWorld::WaitForAsyncStep()
	{
		if (asyncStepState_) {
			asyncStepState_->tasks.wait();
		}
	}

	void Fo4PhysicsWorld::ResetStepClockLocked()
	{
		averageInterval_ = fixedStepSeconds_;
		accumulatedInterval_ = 0.0F;
		currentStepSeconds_ = fixedStepSeconds_;
	}
}
