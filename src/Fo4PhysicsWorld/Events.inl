// Included by ../Fo4PhysicsWorld.cpp.
// Split from Fo4PhysicsWorld.cpp. Do not compile this file separately.

namespace Smp
{
	RE::BSEventNotifyControl Fo4PhysicsWorld::ProcessEvent(const LifecycleEvent& a_event, RE::BSTEventSource<LifecycleEvent>*)
	{
		WaitForAsyncStep();
		{
			std::scoped_lock lock(lock_);
			if (saveLoadInProgress_) {
				NoteSaveLoadActorLocked(a_event);
				spdlog::trace(
					"quarantined save-load lifecycle {} actor={} biped={} object={}; actor will be rebuilt from live state",
					ToString(a_event.type),
					static_cast<void*>(a_event.actor),
					static_cast<void*>(a_event.biped),
					static_cast<void*>(a_event.object));
				return RE::BSEventNotifyControl::kContinue;
			}
		}

		if (IsAttachCandidate(a_event.type)) {
			if (a_event.type == LifecycleEventType::kActorSet3D && !a_event.object) {
				std::scoped_lock lock(lock_);
				bool queuedSoftReload = false;
				for (auto& actorStatePointer : systems_) {
					auto& actorState = *actorStatePointer;
					if (actorState.actor != a_event.actor) {
						continue;
					}
					queuedSoftReload = SoftReloadSystemLocked(actorState, a_event.type) || queuedSoftReload;
				}
				std::erase_if(systems_, [](const auto& a_state) {
					return !a_state->actor && !a_state->HasPhysics() && a_state->armorRecords.empty();
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
				spdlog::debug("skipping first-person system physics detach candidate {}", ToString(a_event.type));
				return RE::BSEventNotifyControl::kContinue;
			}

			std::scoped_lock lock(lock_);
			if (a_event.firstPerson) {
				spdlog::debug(
					"ignored first-person armor detach full system rebuild candidate {} actor={} object={}",
					ToString(a_event.type),
					static_cast<void*>(a_event.actor),
					static_cast<void*>(a_event.object));
				return RE::BSEventNotifyControl::kContinue;
			}
			if (ShouldDeferCharacterCustomizationPhysicsLocked(a_event)) {
				return RE::BSEventNotifyControl::kContinue;
			}

			auto* liveBiped = a_event.actor->GetBiped(a_event.firstPerson).get();
			if (a_event.biped && a_event.biped != liveBiped) {
				spdlog::debug(
					"ignored stale armor detach from replaced biped actor={} eventBiped={} liveBiped={} bipedObject={} object={}",
					static_cast<void*>(a_event.actor),
					static_cast<void*>(a_event.biped),
					static_cast<void*>(liveBiped),
					std::to_underlying(ResolveEventBipedObject(a_event)),
					static_cast<void*>(a_event.object));
				return RE::BSEventNotifyControl::kContinue;
			}

			const auto bipedObject = ResolveEventBipedObject(a_event);
			auto* liveBipObject =
				liveBiped && bipedObject != RE::BIPED_OBJECT::kTotal ?
					liveBiped->GetBipObject(bipedObject) :
					nullptr;
			auto* liveObject = liveBipObject ? liveBipObject->partClone.get() : nullptr;
			auto* pending = FindPendingActorRebuildLocked(a_event.actor, a_event.firstPerson);
			auto* actorState = FindSystemLocked(a_event.actor, a_event.firstPerson);
			std::uint32_t hairVisibilityChanges = 0;
			if (a_event.type == LifecycleEventType::kArmorDetachEnd && !a_event.firstPerson) {
				std::erase_if(actorHairVisibilityStates_, [](const ActorHairVisibilityState& a_state) {
					return !a_state.actorHandle.get();
				});
				auto visibilityState = std::ranges::find_if(actorHairVisibilityStates_, [&](const ActorHairVisibilityState& a_state) {
					const auto actor = a_state.actorHandle.get();
					return actor && actor.get() == a_event.actor;
				});
				if (visibilityState != actorHairVisibilityStates_.end()) {
					auto* faceNode = a_event.actor->GetFaceNodeSkinned();
					if (!faceNode || visibilityState->faceIdentity != reinterpret_cast<std::uintptr_t>(faceNode)) {
						actorHairVisibilityStates_.erase(visibilityState);
					} else {
						hairVisibilityChanges = RefreshHairSourceVisibilityStates(visibilityState->sources);
					}
				}
			}
			if (!a_event.object && a_event.type == LifecycleEventType::kArmorDetachBegin) {
				spdlog::trace(
					"deferred null-object armor detach begin actor={} biped={} bipedObject={} queueDetach={} until post-remove live-slot validation",
					static_cast<void*>(a_event.actor),
					static_cast<void*>(a_event.biped),
					std::to_underlying(bipedObject),
					a_event.queueDetach);
				return RE::BSEventNotifyControl::kContinue;
			}
			if (!a_event.object &&
				a_event.type == LifecycleEventType::kArmorDetachEnd &&
				bipedObject != RE::BIPED_OBJECT::kTotal &&
				liveObject &&
				hairVisibilityChanges == 0) {
				spdlog::debug(
					"ignored stale null-object armor detach end actor={} biped={} bipedObject={} liveObject={} because the live slot is populated",
					static_cast<void*>(a_event.actor),
					static_cast<void*>(liveBiped),
					std::to_underlying(bipedObject),
					static_cast<void*>(liveObject));
				return RE::BSEventNotifyControl::kContinue;
			}
			const auto queuedVisibilityRebuild = hairVisibilityChanges != 0;
			if (queuedVisibilityRebuild) {
				MarkPendingHeadRebuildLocked(LifecycleEvent{
					.type = LifecycleEventType::kActorHeadInitialized,
					.actor = a_event.actor,
					.object = a_event.actor->GetFaceNodeSkinned() ? reinterpret_cast<RE::NiAVObject*>(a_event.actor->GetFaceNodeSkinned()) : nullptr,
					.firstPerson = a_event.firstPerson,
				});
				spdlog::debug(
					"queued headpart visibility rebuild after armor detach changed live hair sources actor={} reportedBipedObject={} changedSources={}",
					static_cast<void*>(a_event.actor),
					std::to_underlying(bipedObject),
					hairVisibilityChanges);
			}

			if (pending) {
				const auto before = pending->armorRecords.size();
				auto remainingRecords = CollectQueuedArmorRecordsForDetachLocked(a_event);
				const auto after = remainingRecords.size();
				pending->armorRecords = std::move(remainingRecords);
				if (before != after) {
					spdlog::debug(
						"invalidated pending armor rebuild records after detach actor={} firstPerson={} bipedObject={} removed={} remaining={}",
						static_cast<void*>(a_event.actor),
						a_event.firstPerson,
						std::to_underlying(ResolveEventBipedObject(a_event)),
						before - after,
						after);
				}
				if (pending->armorRecords.empty() && !pending->forceArmorRescan) {
					std::erase_if(pendingActorRebuilds_, [&](const PendingActorRebuild& a_pending) {
						const auto resolvedActor = a_pending.actorHandle.get();
						return resolvedActor && resolvedActor.get() == a_event.actor && a_pending.firstPerson == a_event.firstPerson;
					});
				}
			}

			if (!actorState) {
				spdlog::trace(
					"ignored untracked armor detach candidate {} actor={} object={}",
					ToString(a_event.type),
					static_cast<void*>(a_event.actor),
					static_cast<void*>(a_event.object));
				return RE::BSEventNotifyControl::kContinue;
			}

			bool cleared = false;
			bool clearedHairSlotArmor = IsHairBipedObject(bipedObject);
			if (a_event.object) {
				auto buildGroups = CollectBuildGroupsForObjectLocked(*actorState, a_event.object);
				clearedHairSlotArmor = clearedHairSlotArmor || BuildGroupsIncludeHairSlotArmorLocked(*actorState, buildGroups);
				if (!buildGroups.empty()) {
					ClearBuildGroupsLocked(*actorState, buildGroups);
					cleared = true;
				}
			}
			if (!cleared &&
				bipedObject != RE::BIPED_OBJECT::kTotal &&
				!liveObject) {
				cleared = ClearBuildGroupsForBipedObjectLocked(*actorState, bipedObject);
			}
			const auto hairSlotVisibilityChanged =
				a_event.type == LifecycleEventType::kArmorDetachEnd && IsHairBipedObject(bipedObject);
			if (!queuedVisibilityRebuild && ((cleared && clearedHairSlotArmor) || hairSlotVisibilityChanged)) {
				MarkPendingHeadRebuildLocked(LifecycleEvent{
					.type = LifecycleEventType::kActorHeadInitialized,
					.actor = a_event.actor,
					.object = a_event.actor->GetFaceNodeSkinned() ? reinterpret_cast<RE::NiAVObject*>(a_event.actor->GetFaceNodeSkinned()) : nullptr,
					.firstPerson = a_event.firstPerson,
				});
				spdlog::debug(
					"queued headpart visibility rebuild after hair-slot armor detach actor={} bipedObject={} clearedPhysics={}",
					static_cast<void*>(a_event.actor),
					std::to_underlying(bipedObject),
					cleared);
			}
			std::erase_if(systems_, [](const auto& a_state) {
				return !a_state->suspended && !a_state->HasPhysics() && a_state->armorRecords.empty();
			});
			ResetStepClockLocked();
			const auto detachLogLevel = cleared ? spdlog::level::debug : spdlog::level::trace;
			spdlog::log(
				detachLogLevel,
				"processed scoped armor system physics detach after {} actor={} object={} cleared={} customizationActive={}",
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
			const auto deferForCustomization = ShouldDeferCharacterCustomizationPhysicsLocked(a_event);
			bool queuedSoftReload = false;
			for (auto& actorStatePointer : systems_) {
				auto& actorState = *actorStatePointer;
				if (actorState.actor != a_event.actor) {
					continue;
				}
				if (deferForCustomization) {
					continue;
				}
				queuedSoftReload = SoftReloadSystemLocked(actorState, a_event.type) || queuedSoftReload;
			}
			std::erase_if(systems_, [](const auto& a_state) {
				return !a_state->actor && !a_state->HasPhysics() && a_state->armorRecords.empty();
			});
			if (!deferForCustomization) {
				PruneInvalidSystemsLocked();
			}
			bool rebuilt = false;
			if (!deferForCustomization && InitializeLocked() && IsBuildCandidateLocked(a_event, true)) {
				BuildForEventLocked(a_event);
				rebuilt = FindSystemLocked(a_event.actor, a_event.firstPerson) != nullptr;
			}
			if (!rebuilt && queuedSoftReload) {
				if (!deferForCustomization) {
					MarkPendingActorRebuildLocked(a_event.actor, a_event.firstPerson);
				}
			}
			spdlog::debug(
				"physics world observed rebuild/reset candidate {} actor={} object={} queuedSoftReload={}",
				ToString(a_event.type),
				static_cast<void*>(a_event.actor),
				static_cast<void*>(a_event.object),
				queuedSoftReload);
		} else if (IsHeadCandidate(a_event.type)) {
			std::scoped_lock lock(lock_);
			if (ShouldDeferCharacterCustomizationPhysicsLocked(a_event)) {
				const auto finalized = FinalizeHeadHierarchyForEventLocked(a_event);
				ObserveGuardedHeadEventLocked(a_event);
				spdlog::debug(
					"finalized live head hierarchy and absorbed physics build {} for guarded customization target actor={} object={} finalized={}",
					ToString(a_event.type),
					static_cast<void*>(a_event.actor),
					static_cast<void*>(a_event.object),
					finalized);
				return RE::BSEventNotifyControl::kContinue;
			}
			PruneInvalidSystemsLocked();
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
				spdlog::debug("loading menu '{}' opened; system physics suspended until game resumes", std::string_view(a_event.menuName));
			} else {
				if (loadingMenuDepth_ > 0) {
					--loadingMenuDepth_;
				}
				spdlog::debug(
					"loading menu '{}' closed; system physics reset will run when game resumes depth={}",
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
				SuspendCharacterCustomizationTargetLocked();
			}
			spdlog::debug(
				"character customization menu '{}' opened; target-scoped system physics suspension active actor={}",
				std::string_view(a_event.menuName),
				static_cast<void*>(ResolveCharacterCustomizationTargetLocked()));
		} else {
			if (characterCustomizationMenuDepth_ > 0) {
				--characterCustomizationMenuDepth_;
			}
			if (characterCustomizationMenuDepth_ == 0) {
				ReloadCharacterCustomizationTargetLocked();
			}
			ResetStepClockLocked();
			spdlog::debug(
				"character customization menu '{}' closed; target-scoped FaceGen idle rebuild guard started",
				std::string_view(a_event.menuName));
		}

		return RE::BSEventNotifyControl::kContinue;
	}
}
