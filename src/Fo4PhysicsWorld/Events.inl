// Included by ../Fo4PhysicsWorld.cpp.
// Split from Fo4PhysicsWorld.cpp. Do not compile this file separately.

namespace Smp
{
	RE::BSEventNotifyControl Fo4PhysicsWorld::ProcessEvent(const LifecycleEvent& a_event, RE::BSTEventSource<LifecycleEvent>*)
	{
		WaitForAsyncStep();
		if (IsAttachCandidate(a_event.type)) {
			if (a_event.type == LifecycleEventType::kActorSet3D && !a_event.object) {
				std::scoped_lock lock(lock_);
				bool queuedSoftReload = false;
				for (auto& actorState : prototypeActors_) {
					if (actorState.actor != a_event.actor) {
						continue;
					}
					queuedSoftReload = SoftReloadPrototypeStateLocked(actorState, a_event.type) || queuedSoftReload;
				}
				std::erase_if(prototypeActors_, [](const PrototypeActorState& a_state) {
					return !a_state.actor && !a_state.HasRuntime() && a_state.armorRecords.empty();
				});
				if (queuedSoftReload) {
					spdlog::debug("physics world soft-reloaded actor state for null Set3D actor={}", static_cast<void*>(a_event.actor));
				} else {
					spdlog::trace("physics world ignored null Set3D for untracked actor={}", static_cast<void*>(a_event.actor));
				}
				return RE::BSEventNotifyControl::kContinue;
			}
			NoteLifecycleCandidate(a_event);
		} else if (IsArmorDetachCandidate(a_event.type)) {
			if (!a_event.actor) {
				spdlog::trace(
					"skipping unactionable armor detach candidate {} actor={} biped={} object={}",
					ToString(a_event.type),
					static_cast<void*>(a_event.actor),
					static_cast<void*>(a_event.biped),
					static_cast<void*>(a_event.object));
				return RE::BSEventNotifyControl::kContinue;
			}

			if (IsIgnoredFirstPersonEvent(a_event, disableFirstPersonViewPhysics_)) {
				spdlog::debug("skipping first-person prototype physics detach candidate {}", ToString(a_event.type));
				return RE::BSEventNotifyControl::kContinue;
			}

			std::scoped_lock lock(lock_);
			if (a_event.firstPerson) {
				spdlog::debug(
					"ignored first-person armor detach full prototype rebuild candidate {} actor={} object={}",
					ToString(a_event.type),
					static_cast<void*>(a_event.actor),
					static_cast<void*>(a_event.object));
				return RE::BSEventNotifyControl::kContinue;
			}
			auto* actorState = FindPrototypeStateLocked(a_event.actor, a_event.firstPerson);
			if (!actorState) {
				spdlog::trace(
					"ignored untracked armor detach candidate {} actor={} object={}",
					ToString(a_event.type),
					static_cast<void*>(a_event.actor),
					static_cast<void*>(a_event.object));
				return RE::BSEventNotifyControl::kContinue;
			}

			bool cleared = false;
			const auto bipedObject = ResolveEventBipedObject(a_event);
			if (a_event.object) {
				cleared = ClearPrototypeGroupsForObjectLocked(*actorState, a_event.object);
			}
			if (!cleared && bipedObject != RE::BIPED_OBJECT::kTotal) {
				cleared = ClearPrototypeGroupsForBipedObjectLocked(*actorState, bipedObject);
			}
			if (cleared && IsHairBipedObject(bipedObject)) {
				MarkPendingHeadRebuildLocked(LifecycleEvent{
					.type = LifecycleEventType::kActorHeadInitialized,
					.actor = a_event.actor,
					.object = a_event.actor->GetFaceNodeSkinned() ? reinterpret_cast<RE::NiAVObject*>(a_event.actor->GetFaceNodeSkinned()) : nullptr,
					.firstPerson = a_event.firstPerson,
				});
				spdlog::debug(
					"queued headpart prototype rebuild after hair-slot armor detach actor={} bipedObject={}",
					static_cast<void*>(a_event.actor),
					std::to_underlying(bipedObject));
			}
			std::erase_if(prototypeActors_, [](const PrototypeActorState& a_state) {
				return !a_state.runtimeSuspended && !a_state.HasRuntime() && a_state.armorRecords.empty();
			});
			ResetStepClockLocked();
			const auto detachLogLevel = cleared ? spdlog::level::debug : spdlog::level::trace;
			spdlog::log(
				detachLogLevel,
				"processed scoped armor prototype physics detach after {} actor={} object={} cleared={} customizationActive={}",
				ToString(a_event.type),
				static_cast<void*>(a_event.actor),
				static_cast<void*>(a_event.object),
				cleared,
				characterCustomizationMenuDepth_ > 0);
			spdlog::log(
				detachLogLevel,
				"physics world observed armor detach candidate {} actor={} object={}",
				ToString(a_event.type),
				static_cast<void*>(a_event.actor),
				static_cast<void*>(a_event.object));
		} else if (IsResetCandidate(a_event.type)) {
			std::scoped_lock lock(lock_);
			bool queuedSoftReload = false;
			for (auto& actorState : prototypeActors_) {
				if (actorState.actor != a_event.actor) {
					continue;
				}
				queuedSoftReload = SoftReloadPrototypeStateLocked(actorState, a_event.type) || queuedSoftReload;
			}
			std::erase_if(prototypeActors_, [](const PrototypeActorState& a_state) {
				return !a_state.actor && !a_state.HasRuntime() && a_state.armorRecords.empty();
			});
			const auto deferForCustomization = characterCustomizationMenuDepth_ > 0;
			if (!deferForCustomization) {
				PruneInvalidPrototypeStatesLocked();
			}
			bool rebuilt = false;
			if (!deferForCustomization && InitializeLocked() && IsPrototypeCandidateLocked(a_event, true)) {
				BuildPrototypeForEventLocked(a_event);
				rebuilt = FindPrototypeStateLocked(a_event.actor, a_event.firstPerson) != nullptr;
			}
			if (!rebuilt && queuedSoftReload) {
				if (characterCustomizationMenuDepth_ == 0) {
					MarkPendingActorRebuildLocked(a_event.actor, a_event.firstPerson);
				}
			}
			spdlog::debug(
				"physics world observed rebuild/reset candidate {} actor={} object={} queuedSoftReload={}",
				ToString(a_event.type),
				static_cast<void*>(a_event.actor),
				static_cast<void*>(a_event.object),
				queuedSoftReload);
		} else if (a_event.type == LifecycleEventType::kActorUpdate3DModel) {
			std::scoped_lock lock(lock_);
			if (characterCustomizationMenuDepth_ == 0 && (HasActiveOrPendingActorRebuildLocked(a_event.actor) || !pendingHeadRebuilds_.empty())) {
				SchedulePendingRebuildTaskLocked();
			}
			spdlog::trace(
				"physics world observed per-frame update candidate {} actor={} object={}; pending rebuilds run through the F4SE task queue",
				ToString(a_event.type),
				static_cast<void*>(a_event.actor),
				static_cast<void*>(a_event.object));
		} else if (IsHeadCandidate(a_event.type)) {
			std::scoped_lock lock(lock_);
			PruneInvalidPrototypeStatesLocked();
			MarkPendingHeadRebuildLocked(a_event);
			ResetStepClockLocked();
			spdlog::debug(
				"queued head physics rebuild candidate {} actor={} object={} for deferred main-frame processing",
				ToString(a_event.type),
				static_cast<void*>(a_event.actor),
				static_cast<void*>(a_event.object));
		}

		return RE::BSEventNotifyControl::kContinue;
	}

	RE::BSEventNotifyControl Fo4PhysicsWorld::ProcessEvent(const RE::MenuOpenCloseEvent& a_event, RE::BSTEventSource<RE::MenuOpenCloseEvent>*)
	{
		WaitForAsyncStep();
		const auto menuName = LowerMenuName(a_event.menuName);
		if (menuName == "loadingmenu") {
			std::scoped_lock lock(lock_);
			if (a_event.opening) {
				++loadingMenuDepth_;
				loadingPhysicsSuspended_ = true;
				ResetStepClockLocked();
				if (dynamicsWorld_) {
					dynamicsWorld_->clearForces();
				}
				spdlog::debug("loading menu '{}' opened; prototype physics suspended until game resumes", std::string_view(a_event.menuName));
			} else {
				if (loadingMenuDepth_ > 0) {
					--loadingMenuDepth_;
				}
				spdlog::debug(
					"loading menu '{}' closed; prototype physics reset will run when game resumes depth={}",
					std::string_view(a_event.menuName),
					loadingMenuDepth_);
			}
			return RE::BSEventNotifyControl::kContinue;
		}

		if (menuName != "looksmenu") {
			return RE::BSEventNotifyControl::kContinue;
		}

		std::scoped_lock lock(lock_);
		if (a_event.opening) {
			const auto wasClosed = characterCustomizationMenuDepth_ == 0;
			++characterCustomizationMenuDepth_;
			if (wasClosed) {
				pendingActorRebuilds_.clear();
				pendingHeadRebuilds_.clear();
				pendingRebuildTaskQueued_ = false;
				SuspendPrototypeStatesForCustomizationMenuLocked();
			}
			spdlog::debug(
				"character customization menu '{}' opened; prototype physics suspended for active actors",
				std::string_view(a_event.menuName));
		} else {
			if (characterCustomizationMenuDepth_ > 0) {
				--characterCustomizationMenuDepth_;
			}
			if (characterCustomizationMenuDepth_ == 0) {
				ReloadPrototypeStatesForCustomizationMenuLocked();
				std::vector<RE::Actor*> actors;
				for (const auto& actorState : prototypeActors_) {
					if (actorState.actor && std::ranges::find(actors, actorState.actor) == actors.end()) {
						actors.push_back(actorState.actor);
					}
				}
				for (auto& actorState : prototypeActors_) {
					if (actorState.actor) {
						ClearHeadPrototypeTrackingLocked(actorState, "customization-menu-closed");
						actorState.faceNode = nullptr;
					}
				}
				for (auto* actor : actors) {
					MarkPendingHeadRebuildLocked(LifecycleEvent{
						.type = LifecycleEventType::kActorHeadInitialized,
						.actor = actor,
						.object = actor->GetFaceNodeSkinned() ? reinterpret_cast<RE::NiAVObject*>(actor->GetFaceNodeSkinned()) : nullptr,
						.firstPerson = false,
					});
				}
			}
			ResetStepClockLocked();
			spdlog::debug(
				"character customization menu '{}' closed; prototype physics reloaded from tracked armor records",
				std::string_view(a_event.menuName));
		}

		return RE::BSEventNotifyControl::kContinue;
	}
}
