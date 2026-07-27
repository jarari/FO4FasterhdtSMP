// Included by ../Fo4PhysicsWorld.cpp.
// Split from Fo4PhysicsWorld.cpp. Do not compile this file separately.

namespace Smp
{
	void Fo4PhysicsWorld::PrepareActor3DModelUpdate(RE::Actor* a_actor, const std::uint16_t a_updateFlags)
	{
		if (!a_actor) {
			return;
		}

		WaitForAsyncStep();
		std::scoped_lock lock(lock_);

		std::uint32_t clearedStates = 0;
		for (auto& actorStatePointer : systems_) {
			auto& actorState = *actorStatePointer;
			if (actorState.actor != a_actor || actorState.firstPerson) {
				continue;
			}

			ClearSystemLocked(actorState);
			++clearedStates;
		}
		std::erase_if(systems_, [](const auto& a_state) {
			return !a_state->suspended && !a_state->HasPhysics() && a_state->armorRecords.empty();
		});

		const auto discardedSuspended = std::erase_if(
			suspendedActors_,
			[a_actor](const SuspendedActorCandidate& a_candidate) {
				const auto actor = a_candidate.actorHandle.get();
				return actor && actor.get() == a_actor && !a_candidate.firstPerson;
			});
		const auto discardedActorRebuilds = std::erase_if(
			pendingActorRebuilds_,
			[a_actor](const PendingActorRebuild& a_pending) {
				const auto actor = a_pending.actorHandle.get();
				return actor && actor.get() == a_actor && !a_pending.firstPerson;
			});
		const auto discardedHeadRebuilds = std::erase_if(
			pendingHeadRebuilds_,
			[a_actor](const PendingHeadRebuild& a_pending) {
				const auto actor = a_pending.actorHandle.get();
				return actor && actor.get() == a_actor;
			});
		const auto discardedModelUpdates = std::erase_if(
			pendingActor3DModelUpdates_,
			[a_actor](const PendingActor3DModelUpdate& a_pending) {
				const auto actor = a_pending.actorHandle.get();
				return actor && actor.get() == a_actor;
			});

		ResetStepClockLocked();
		spdlog::debug(
			"prepared actor 3D model update actor={} flags=0x{:X} clearedStates={} discardedSuspended={} discardedActorRebuilds={} discardedHeadRebuilds={} discardedModelUpdates={}",
			static_cast<void*>(a_actor),
			a_updateFlags,
			clearedStates,
			discardedSuspended,
			discardedActorRebuilds,
			discardedHeadRebuilds,
			discardedModelUpdates);
	}

	void Fo4PhysicsWorld::QueueActor3DModelUpdateCompletion(RE::Actor* a_actor, const std::uint16_t a_updateFlags)
	{
		if (!a_actor) {
			return;
		}

		WaitForAsyncStep();
		std::scoped_lock lock(lock_);
		auto actorHandle = RE::BSPointerHandleManagerInterface<RE::Actor>::GetHandle(a_actor);
		if (!actorHandle) {
			return;
		}

		for (auto& pending : pendingActor3DModelUpdates_) {
			const auto actor = pending.actorHandle.get();
			if (actor && actor.get() == a_actor) {
				pending.updateFlags |= a_updateFlags;
				return;
			}
		}

		pendingActor3DModelUpdates_.push_back({
			.actorHandle = actorHandle,
			.updateFlags = a_updateFlags,
		});
	}

	void Fo4PhysicsWorld::CompletePendingActor3DModelUpdates()
	{
		std::vector<PendingActor3DModelUpdate> pendingUpdates;
		{
			std::scoped_lock lock(lock_);
			pendingUpdates.swap(pendingActor3DModelUpdates_);
		}

		for (const auto& pending : pendingUpdates) {
			const auto actor = pending.actorHandle.get();
			if (actor) {
				CompleteActor3DModelUpdate(actor.get(), pending.updateFlags);
			}
		}
	}

	void Fo4PhysicsWorld::CompleteActor3DModelUpdate(RE::Actor* a_actor, const std::uint16_t a_updateFlags)
	{
		if (!a_actor) {
			return;
		}

		std::scoped_lock lock(lock_);
		if (!InitializeLocked()) {
			return;
		}

		// ReEquipAll attachment events were drained before this completion and
		// recorded their XML/bone context in the existing pending rebuild. Keep
		// those records; the forced rescan is only the fallback when none exist.
		MarkPendingActorRebuildLocked(a_actor, false, {}, true, true, false);
		MarkPendingHeadRebuildLocked(LifecycleEvent{
			.type = LifecycleEventType::kActorHeadInitialized,
			.actor = a_actor,
			.firstPerson = false,
		});
		ResetStepClockLocked();
		spdlog::debug(
			"completed actor 3D model update at main-frame sync actor={} flags=0x{:X} forceArmorRescan=true headReloadQueued=true",
			static_cast<void*>(a_actor),
			a_updateFlags);
	}

	void Fo4PhysicsWorld::NoteCharacterCustomizationTarget(RE::Actor* a_actor)
	{
		WaitForAsyncStep();
		std::scoped_lock lock(lock_);
		NoteCharacterCustomizationTargetLocked(a_actor);
		if (characterCustomizationMenuDepth_ > 0) {
			SuspendCharacterCustomizationTargetLocked();
		}
	}

	bool Fo4PhysicsWorld::NoteCharacterCustomizationTargetLocked(RE::Actor* a_actor)
	{
		if (!a_actor) {
			return false;
		}

		auto handle = RE::BSPointerHandleManagerInterface<RE::Actor>::GetHandle(a_actor);
		if (!handle) {
			return false;
		}

		characterCustomizationTarget_ = handle;
		spdlog::debug(
			"captured character customization target actor={} menuDepth={}",
			static_cast<void*>(a_actor),
			characterCustomizationMenuDepth_);
		return true;
	}

	RE::Actor* Fo4PhysicsWorld::ResolveCharacterCustomizationTargetLocked()
	{
		if (!characterCustomizationTarget_) {
			return nullptr;
		}

		auto resolved = characterCustomizationTarget_.get();
		if (!resolved) {
			characterCustomizationTarget_.reset();
			return nullptr;
		}

		return resolved.get();
	}

	bool Fo4PhysicsWorld::IsCharacterCustomizationTargetLocked(RE::Actor* a_actor)
	{
		if (!a_actor) {
			return false;
		}

		return ResolveCharacterCustomizationTargetLocked() == a_actor;
	}

	bool Fo4PhysicsWorld::ShouldDeferCharacterCustomizationPhysicsLocked(const LifecycleEvent& a_event)
	{
		if (characterCustomizationMenuDepth_ == 0 || !a_event.actor || a_event.firstPerson) {
			return false;
		}

		auto* target = ResolveCharacterCustomizationTargetLocked();
		if (!target) {
			NoteCharacterCustomizationTargetLocked(a_event.actor);
			target = a_event.actor;
			SuspendCharacterCustomizationTargetLocked();
			spdlog::debug(
				"using first LooksMenu lifecycle event as customization target fallback actor={} type={}",
				static_cast<void*>(a_event.actor),
				ToString(a_event.type));
		}

		if (target != a_event.actor) {
			return false;
		}

		ResetStepClockLocked();
		spdlog::debug(
			"deferred target character customization physics build {} actor={}",
			ToString(a_event.type),
			static_cast<void*>(a_event.actor));
		return true;
	}

	void Fo4PhysicsWorld::SuspendCharacterCustomizationTargetLocked()
	{
		auto* target = ResolveCharacterCustomizationTargetLocked();
		if (!target) {
			return;
		}

		const auto discardedSuspendedCandidates = std::erase_if(
			suspendedActors_,
			[target](const SuspendedActorCandidate& a_candidate) {
				const auto actor = a_candidate.actorHandle.get();
				return actor && actor.get() == target && !a_candidate.firstPerson;
			});
		std::uint32_t suspendedStates = 0;
		std::uint32_t discardedArmorRecords = 0;
		for (auto& actorStatePointer : systems_) {
			auto& actorState = *actorStatePointer;
			if (actorState.actor != target || actorState.firstPerson) {
				continue;
			}
			if (actorState.HasPhysics() && !actorState.suspended) {
				SuspendSystemLocked(actorState);
				++suspendedStates;
			}
			discardedArmorRecords += static_cast<std::uint32_t>(actorState.armorRecords.size());
			actorState.armorRecords.clear();
		}

		if (suspendedStates > 0 || discardedArmorRecords > 0 || discardedSuspendedCandidates > 0) {
			ResetStepClockLocked();
		}
		spdlog::debug(
			"suspended target system physics for character customization actor={} suspendedStates={} discardedArmorRecords={} discardedSuspendedCandidates={} trackedStates={}",
			static_cast<void*>(target),
			suspendedStates,
			discardedArmorRecords,
			discardedSuspendedCandidates,
			systems_.size());
	}

	void Fo4PhysicsWorld::ReloadCharacterCustomizationTargetLocked()
	{
		auto* target = ResolveCharacterCustomizationTargetLocked();
		if (!target) {
			ClearCharacterCustomizationTargetLocked();
			spdlog::debug("skipping character customization target reload because target actor is unresolved");
			return;
		}

		if (!InitializeLocked()) {
			ClearCharacterCustomizationTargetLocked();
			return;
		}

		std::uint32_t clearedStates = 0;
		std::uint32_t skippedFirstPerson = 0;
		for (auto& actorStatePointer : systems_) {
			auto& actorState = *actorStatePointer;
			if (actorState.actor != target) {
				continue;
			}
			if (actorState.firstPerson) {
				++skippedFirstPerson;
				continue;
			}

			ClearSystemLocked(actorState);
			++clearedStates;
		}

		std::erase_if(suspendedActors_, [target](const SuspendedActorCandidate& a_candidate) {
			const auto actor = a_candidate.actorHandle.get();
			return actor && actor.get() == target && !a_candidate.firstPerson;
		});
		std::erase_if(systems_, [](const auto& a_state) {
			return !a_state->suspended && !a_state->HasPhysics() && a_state->armorRecords.empty();
		});

		MarkPendingActorRebuildLocked(target, false, {}, true, true, true);
		MarkPendingHeadRebuildLocked(LifecycleEvent{
			.type = LifecycleEventType::kActorHeadInitialized,
			.actor = target,
			.firstPerson = false,
		});
		ResetStepClockLocked();
		spdlog::debug(
			"queued live-state system physics reload after character customization actor={} forceArmorRescan=true headReloadQueued=true clearedStates={} skippedFirstPerson={}",
			static_cast<void*>(target),
			clearedStates,
			skippedFirstPerson);
		ClearCharacterCustomizationTargetLocked();
	}

	void Fo4PhysicsWorld::ClearCharacterCustomizationTargetLocked()
	{
		characterCustomizationTarget_.reset();
	}

	void Fo4PhysicsWorld::SuspendSystemLocked(Fo4SkinnedMeshSystem& a_state)
	{
		std::vector<std::uint64_t> buildGroups;
		for (const auto& runtime : a_state.buildGroups) {
			if (runtime.buildGroup != 0 && std::ranges::find(buildGroups, runtime.buildGroup) == buildGroups.end()) {
				buildGroups.push_back(runtime.buildGroup);
			}
		}
		if (buildGroups.empty()) {
			for (const auto& body : a_state.bodies) {
				for (const auto buildGroup : body.buildGroups) {
					if (buildGroup != 0 && std::ranges::find(buildGroups, buildGroup) == buildGroups.end()) {
						buildGroups.push_back(buildGroup);
					}
				}
				if (body.buildGroup != 0 && std::ranges::find(buildGroups, body.buildGroup) == buildGroups.end()) {
					buildGroups.push_back(body.buildGroup);
				}
			}
		}
		auto* actorRoot = a_state.actor ? a_state.actor->Get3D(a_state.firstPerson) : nullptr;
		if (!actorRoot && a_state.actor && !a_state.firstPerson) {
			actorRoot = a_state.actor->Get3D();
		}
		if (actorRoot) {
			ResetBuildGroupsToStoredLocalPoseLocked(a_state, buildGroups, "suspend");
		}
		if (dispatcher_) {
			dispatcher_->clearAllManifold();
		}

		if (dynamicsWorld_ && a_state.m_world == dynamicsWorld_.get()) {
			dynamicsWorld_->removeSkinnedMeshSystem(std::addressof(a_state));
		}

		const auto meshCount = a_state.meshes.size();
		const auto constraintCount = a_state.constraints.size();
		const auto bodyCount = a_state.bodies.size();
		std::uint32_t capturedSkinSlots = 0;
		for (auto& boneRecord : a_state.bodies) {
			if (!boneRecord.bone) {
				continue;
			}
			const auto before = a_state.suspendedSkinSlots.size();
			boneRecord.bone->CollectSkinWorldTransformRestoreSlots(a_state.suspendedSkinSlots);
			capturedSkinSlots += static_cast<std::uint32_t>(a_state.suspendedSkinSlots.size() - before);
		}
		// BoneRecord owns each rigid body's shape and motion state. Destroy the
		// system-owned Bullet objects before releasing those record dependencies.
		a_state.ClearSystemObjects();
		a_state.meshes.clear();
		a_state.constraints.clear();
		a_state.bodies.clear();
		a_state.buildGroups.clear();
		a_state.lastWritebackFrame = 0;
		a_state.lastWritebackSource = WritebackSource::kUnknown;
		a_state.suspended = true;
		spdlog::debug(
			"suspended system runtime for actor={} buildGroups={} bodies={} meshes={} constraints={} capturedSkinSlots={} cachedAttachmentBonePoses={} armorRecords={}",
			static_cast<void*>(a_state.actor),
			buildGroups.size(),
			bodyCount,
			meshCount,
			constraintCount,
			capturedSkinSlots,
			a_state.attachmentBoneLocalPoses.size(),
			a_state.armorRecords.size());
	}

	void Fo4PhysicsWorld::DeactivateSystemLocked(Fo4SkinnedMeshSystem& a_state)
	{
		if (!a_state.HasPhysics() || a_state.IsInactive()) {
			return;
		}

		std::vector<std::uint64_t> buildGroups;
		for (const auto& runtime : a_state.buildGroups) {
			if (runtime.buildGroup != 0 && std::ranges::find(buildGroups, runtime.buildGroup) == buildGroups.end()) {
				buildGroups.push_back(runtime.buildGroup);
			}
		}
		ResetBuildGroupsToStoredLocalPoseLocked(a_state, buildGroups, "deactivate");

		const auto wasActive = a_state.m_world == dynamicsWorld_.get();
		const auto removedConstraints = wasActive ? static_cast<std::uint32_t>(a_state.constraints.size()) : 0U;
		const auto removedMeshes = wasActive ? static_cast<std::uint32_t>(a_state.meshes.size()) : 0U;
		const auto removedBodies = wasActive ? static_cast<std::uint32_t>(a_state.bodies.size()) : 0U;
		if (dynamicsWorld_ && a_state.m_world == dynamicsWorld_.get()) {
			dynamicsWorld_->removeSkinnedMeshSystem(std::addressof(a_state));
		}

		a_state.lastWritebackFrame = 0;
		a_state.lastWritebackSource = WritebackSource::kUnknown;
		a_state.currentWindFactor = 0.0F;
		a_state.suspended = false;
		spdlog::debug(
			"deactivated system actor={} buildGroups={} removedBodies={} removedMeshes={} removedConstraints={} retainedBodies={} retainedMeshes={} retainedConstraints={} trackedBuildGroups={} armorRecords={}",
			static_cast<void*>(a_state.actor),
			buildGroups.size(),
			removedBodies,
			removedMeshes,
			removedConstraints,
			a_state.bodies.size(),
			a_state.meshes.size(),
			a_state.constraints.size(),
			a_state.buildGroups.size(),
			a_state.armorRecords.size());
	}

	bool Fo4PhysicsWorld::ReactivateSystemLocked(Fo4SkinnedMeshSystem& a_state)
	{
		if (!a_state.IsInactive() || !dynamicsWorld_) {
			return false;
		}

		std::vector<std::uint64_t> buildGroups;
		for (const auto& runtime : a_state.buildGroups) {
			if (runtime.buildGroup != 0 && std::ranges::find(buildGroups, runtime.buildGroup) == buildGroups.end()) {
				buildGroups.push_back(runtime.buildGroup);
			}
		}
		if (buildGroups.empty()) {
			return false;
		}

		dynamicsWorld_->addSkinnedMeshSystem(std::addressof(a_state));

		if (a_state.m_world != dynamicsWorld_.get()) {
			return false;
		}

		a_state.suspended = false;
		a_state.lastWritebackFrame = 0;
		a_state.lastWritebackSource = WritebackSource::kUnknown;
		a_state.currentWindFactor = 1.0F;
		spdlog::debug(
			"reactivated system actor={} buildGroups={} bodies={} meshes={} constraints={}",
			static_cast<void*>(a_state.actor),
			buildGroups.size(),
			a_state.bodies.size(),
			a_state.meshes.size(),
			a_state.constraints.size());
		return true;
	}

	std::uint32_t Fo4PhysicsWorld::RestoreSuspendedSkinSlotsLocked(
		Fo4SkinnedMeshSystem& a_state,
		const std::span<const std::uint64_t> a_buildGroups,
		const std::span<const Fo4SkinnedMeshBone::ActiveSkinSlot> a_activeSlots)
	{
		if (a_buildGroups.empty() || a_state.suspendedSkinSlots.empty()) {
			return 0;
		}

		const auto containsGroup = [&a_buildGroups](const std::uint64_t a_buildGroup) {
			return a_buildGroup != 0 && std::ranges::find(a_buildGroups, a_buildGroup) != a_buildGroups.end();
		};
		const auto hasActiveSlot = [&a_activeSlots](RE::BSSkin::Instance* a_skin, const std::uint32_t a_index) {
			return std::ranges::any_of(a_activeSlots, [a_skin, a_index](const Fo4SkinnedMeshBone::ActiveSkinSlot& a_slot) {
				return a_slot.skin == a_skin && a_slot.index == a_index;
			});
		};
		const auto hasActiveSkin = [&a_activeSlots](RE::BSSkin::Instance* a_skin) {
			return std::ranges::any_of(a_activeSlots, [a_skin](const Fo4SkinnedMeshBone::ActiveSkinSlot& a_slot) {
				return a_slot.skin == a_skin;
			});
		};
		const auto hasRetainedSuspendedSlot = [&a_state, &containsGroup](const Fo4SkinnedMeshBone::SkinSlotRestore& a_slot) {
			return std::ranges::any_of(a_state.suspendedSkinSlots, [&a_slot, &containsGroup](const Fo4SkinnedMeshBone::SkinSlotRestore& a_other) {
				return a_other.buildGroup != a_slot.buildGroup &&
					!containsGroup(a_other.buildGroup) &&
					a_other.skin.get() == a_slot.skin.get() &&
					a_other.index == a_slot.index;
			});
		};
		const auto hasRetainedSuspendedSkin = [&a_state, &containsGroup](const Fo4SkinnedMeshBone::SkinSlotRestore& a_slot) {
			return std::ranges::any_of(a_state.suspendedSkinSlots, [&a_slot, &containsGroup](const Fo4SkinnedMeshBone::SkinSlotRestore& a_other) {
				return a_other.buildGroup != a_slot.buildGroup &&
					!containsGroup(a_other.buildGroup) &&
					a_other.skin.get() == a_slot.skin.get();
			});
		};

		std::uint32_t restored = 0;
		for (const auto& slot : a_state.suspendedSkinSlots) {
			if (!containsGroup(slot.buildGroup) || !slot.skin) {
				continue;
			}

			if (!hasActiveSlot(slot.skin.get(), slot.index) && !hasRetainedSuspendedSlot(slot)) {
				if (slot.index < slot.skin->bones.size() &&
					slot.skin->bones[slot.index] == slot.reboundBone.get() &&
					slot.originalBone) {
					slot.skin->bones[slot.index] = slot.originalBone;
					++restored;
				}
				if (slot.index < slot.skin->worldTransforms.size() &&
					slot.skin->worldTransforms[slot.index] == slot.reboundWorldTransform &&
					slot.originalWorldTransform) {
					slot.skin->worldTransforms[slot.index] = slot.originalWorldTransform;
					++restored;
				}
			}

			if (!hasActiveSkin(slot.skin.get()) &&
				!hasRetainedSuspendedSkin(slot) &&
				slot.originalRootNode &&
				slot.skin->rootNode != slot.originalRootNode) {
				slot.skin->rootNode = slot.originalRootNode;
				++restored;
			}
		}

		const auto erased = std::erase_if(a_state.suspendedSkinSlots, [&containsGroup](const Fo4SkinnedMeshBone::SkinSlotRestore& a_slot) {
			return containsGroup(a_slot.buildGroup);
		});
		if (restored > 0 || erased > 0) {
			spdlog::debug(
				"restored {} suspended system skin slot fields and erased {} cached slots for actor={}",
				restored,
				erased,
				static_cast<void*>(a_state.actor));
		}
		return restored;
	}

	std::uint32_t Fo4PhysicsWorld::RestoreAllSuspendedSkinSlotsLocked(Fo4SkinnedMeshSystem& a_state)
	{
		std::vector<std::uint64_t> buildGroups;
		for (const auto& slot : a_state.suspendedSkinSlots) {
			if (slot.buildGroup != 0 && std::ranges::find(buildGroups, slot.buildGroup) == buildGroups.end()) {
				buildGroups.push_back(slot.buildGroup);
			}
		}
		return RestoreSuspendedSkinSlotsLocked(a_state, buildGroups);
	}

	void Fo4PhysicsWorld::ClearSystemLocked(Fo4SkinnedMeshSystem& a_state, const bool a_restoreSkinSlots)
	{
		if (a_restoreSkinSlots && !a_state.attachmentBoneLocalPoses.empty()) {
			std::vector<std::uint64_t> buildGroups;
			for (const auto& localPose : a_state.attachmentBoneLocalPoses) {
				if (localPose.buildGroup != 0 && std::ranges::find(buildGroups, localPose.buildGroup) == buildGroups.end()) {
					buildGroups.push_back(localPose.buildGroup);
				}
			}
			ResetBuildGroupsToStoredLocalPoseLocked(a_state, buildGroups, "clear-state");
		}
		if (dispatcher_) {
			dispatcher_->clearAllManifold();
		}

		if (dynamicsWorld_ && a_state.m_world == dynamicsWorld_.get()) {
			dynamicsWorld_->removeSkinnedMeshSystem(std::addressof(a_state));
		}
		if (!a_state.meshes.empty()) {
			spdlog::debug("cleared {} system physics mesh bodies for actor={}", a_state.meshes.size(), static_cast<void*>(a_state.actor));
		}

		if (!a_state.constraints.empty()) {
			spdlog::debug("cleared {} system physics constraints for actor={}", a_state.constraints.size(), static_cast<void*>(a_state.actor));
		}
		a_state.buildGroups.clear();

		if (a_restoreSkinSlots) {
			for (auto& boneRecord : a_state.bodies) {
				if (!boneRecord.bone) {
					continue;
				}
				for (const auto buildGroup : boneRecord.buildGroups) {
					boneRecord.bone->RemoveSkinWorldTransformsForBuildGroup(buildGroup);
				}
				if (boneRecord.buildGroup != 0) {
					boneRecord.bone->RemoveSkinWorldTransformsForBuildGroup(boneRecord.buildGroup);
				}
			}
			RestoreAllSuspendedSkinSlotsLocked(a_state);
		} else if (!a_state.bodies.empty()) {
			spdlog::debug(
				"skipped restoring system skin slots while clearing actor={} for model rebuild",
				static_cast<void*>(a_state.actor));
			a_state.suspendedSkinSlots.clear();
		}

		if (!a_state.bodies.empty()) {
			spdlog::debug("cleared {} system physics bodies for actor={}", a_state.bodies.size(), static_cast<void*>(a_state.actor));
		}
		// Keep record-owned collision shapes and motion states alive until the
		// system-owned constraints, meshes, and bones have been destroyed.
		a_state.ClearSystemObjects();
		a_state.meshes.clear();
		a_state.constraints.clear();
		a_state.bodies.clear();
		a_state.attachmentBoneLocalPoses.clear();
		a_state.nextBuildGroup = 0;
		a_state.nextAttachmentGeneration = 0;
		a_state.lastReadRoot = nullptr;
		a_state.readInitialized = false;
		a_state.lastRootRotation = btQuaternion::getIdentity();
		a_state.lastRootRotationInitialized = false;
		a_state.lastWritebackFrame = 0;
		a_state.lastWritebackSource = WritebackSource::kUnknown;
		a_state.currentWindFactor = 1.0F;
		a_state.suspended = false;
		a_state.faceNode = nullptr;
		a_state.armorRecords.clear();
		a_state.attachmentRecords.clear();
		a_state.headPartRecords.clear();
		a_state.buildGroups.clear();
		a_state.suspendedSkinSlots.clear();
	}

	std::vector<std::uint64_t> Fo4PhysicsWorld::CollectBuildGroupsForObjectLocked(const Fo4SkinnedMeshSystem& a_state, RE::NiAVObject* a_object) const
	{
		std::vector<std::uint64_t> buildGroups;
		const auto appendGroup = [&buildGroups](const std::uint64_t a_buildGroup) {
			if (a_buildGroup != 0 && std::ranges::find(buildGroups, a_buildGroup) == buildGroups.end()) {
				buildGroups.push_back(a_buildGroup);
			}
		};
		if (!a_object) {
			return buildGroups;
		}
		if (a_state.actor) {
			auto* primaryRoot = a_state.actor->Get3D(a_state.firstPerson);
			auto* thirdPersonRoot = a_state.actor->Get3D(false);
			auto* firstPersonRoot = a_state.actor->Get3D(true);
			if (a_object == primaryRoot || a_object == thirdPersonRoot || a_object == firstPersonRoot || a_object == a_state.faceNode.get()) {
				spdlog::debug(
					"refusing object-scoped system clear from broad actor object={} actor={}; waiting for attachment/biped scoped clear",
					static_cast<void*>(a_object),
					static_cast<void*>(a_state.actor));
				return buildGroups;
			}
		}

		for (const auto& record : a_state.attachmentRecords) {
			const auto matchesAttachment =
				record.attachedObject &&
				(record.attachedObject.get() == a_object ||
					IsObjectInTree(record.attachedObject.get(), a_object) ||
					IsObjectInTree(a_object, record.attachedObject.get()));
			const auto matchesSource =
				record.sourceObject &&
				(record.sourceObject.get() == a_object ||
					IsObjectInTree(record.sourceObject.get(), a_object) ||
					IsObjectInTree(a_object, record.sourceObject.get()));
			if (!matchesAttachment && !matchesSource) {
				continue;
			}

			for (const auto buildGroup : record.buildGroups) {
				appendGroup(buildGroup);
			}
		}
		if (!buildGroups.empty()) {
			return buildGroups;
		}

		for (const auto& meshRecord : a_state.meshes) {
			if (meshRecord.buildGroup == 0 || !meshRecord.geometry || !IsObjectInTree(a_object, meshRecord.geometry)) {
				continue;
			}

			appendGroup(meshRecord.buildGroup);
		}

		for (const auto& boneRecord : a_state.bodies) {
			if (!boneRecord.node || !IsNodeInTree(a_object, boneRecord.node)) {
				continue;
			}

			if (boneRecord.buildGroups.empty()) {
				appendGroup(boneRecord.buildGroup);
				continue;
			}
			for (const auto buildGroup : boneRecord.buildGroups) {
				appendGroup(buildGroup);
			}
		}

		return buildGroups;
	}

	std::vector<std::uint64_t> Fo4PhysicsWorld::CollectArmorPhysicsGroupsForBipedObjectLocked(
		const Fo4SkinnedMeshSystem& a_state,
		const RE::BIPED_OBJECT a_bipedObject,
		const std::uint64_t a_preservedBuildGroup) const
	{
		std::vector<std::uint64_t> buildGroups;
		if (a_bipedObject == RE::BIPED_OBJECT::kTotal) {
			return buildGroups;
		}

		const auto appendGroup = [&buildGroups, a_preservedBuildGroup](const std::uint64_t a_buildGroup) {
			if (a_buildGroup == 0 || a_buildGroup == a_preservedBuildGroup) {
				return;
			}
			if (std::ranges::find(buildGroups, a_buildGroup) == buildGroups.end()) {
				buildGroups.push_back(a_buildGroup);
			}
		};

		const auto groupStillOwnedByBiped = [&a_state, a_bipedObject](const std::uint64_t a_buildGroup) {
			if (a_buildGroup == 0) {
				return false;
			}

			bool foundCurrentOwner = false;
			for (const auto& runtime : a_state.buildGroups) {
				if (runtime.buildGroup != a_buildGroup || runtime.domain != BuildDomain::kArmor) {
					continue;
				}
				foundCurrentOwner = true;
				if (runtime.bipedObject == a_bipedObject) {
					return true;
				}
			}
			for (const auto& meshRecord : a_state.meshes) {
				if (meshRecord.buildGroup != a_buildGroup || meshRecord.domain != BuildDomain::kArmor) {
					continue;
				}
				foundCurrentOwner = true;
				if (meshRecord.bipedObject == a_bipedObject) {
					return true;
				}
			}

			return !foundCurrentOwner;
		};

		for (const auto& record : a_state.armorRecords) {
			if (record.bipedObject != a_bipedObject) {
				continue;
			}
			for (const auto buildGroup : record.buildGroups) {
				if (groupStillOwnedByBiped(buildGroup)) {
					appendGroup(buildGroup);
				}
			}
		}

		for (const auto& record : a_state.attachmentRecords) {
			if (record.bipedObject != a_bipedObject) {
				continue;
			}
			for (const auto buildGroup : record.buildGroups) {
				if (groupStillOwnedByBiped(buildGroup)) {
					appendGroup(buildGroup);
				}
			}
		}

		for (const auto& runtime : a_state.buildGroups) {
			if (runtime.domain == BuildDomain::kArmor && runtime.bipedObject == a_bipedObject) {
				appendGroup(runtime.buildGroup);
			}
		}

		for (const auto& meshRecord : a_state.meshes) {
			if (meshRecord.domain == BuildDomain::kArmor && meshRecord.bipedObject == a_bipedObject) {
				appendGroup(meshRecord.buildGroup);
			}
		}

		return buildGroups;
	}

	std::uint32_t Fo4PhysicsWorld::PrunePhysicsRecordsForBipedObjectLocked(
		Fo4SkinnedMeshSystem& a_state,
		const RE::BIPED_OBJECT a_bipedObject,
		const std::uint64_t a_preservedBuildGroup)
	{
		if (a_bipedObject == RE::BIPED_OBJECT::kTotal) {
			return 0;
		}

		const auto hasPreservedGroup = [a_preservedBuildGroup](const std::vector<std::uint64_t>& a_buildGroups) {
			return a_preservedBuildGroup != 0 &&
				std::ranges::find(a_buildGroups, a_preservedBuildGroup) != a_buildGroups.end();
		};
		const auto removedAttachments = std::erase_if(a_state.attachmentRecords, [a_bipedObject, &hasPreservedGroup](const AttachmentPhysicsRecord& a_record) {
			return a_record.bipedObject == a_bipedObject && !hasPreservedGroup(a_record.buildGroups);
		});
		const auto removedArmorRecords = std::erase_if(a_state.armorRecords, [a_bipedObject, &hasPreservedGroup](const ArmorPhysicsRecord& a_record) {
			return a_record.bipedObject == a_bipedObject && !hasPreservedGroup(a_record.buildGroups);
		});

		return static_cast<std::uint32_t>(removedAttachments + removedArmorRecords);
	}

	std::uint32_t Fo4PhysicsWorld::ClearStaleHairSlotArmorGroupsLocked(
		Fo4SkinnedMeshSystem& a_state,
		const RE::BIPED_OBJECT a_bipedObject,
		const std::uint64_t a_preservedBuildGroup,
		const std::string_view a_reason,
		RE::NiAVObject* a_object,
		const std::string_view a_physicsXmlPath,
		const bool a_resetToStoredLocalPose)
	{
		if (!IsHairBipedObject(a_bipedObject)) {
			return 0;
		}

		auto buildGroups = CollectArmorPhysicsGroupsForBipedObjectLocked(a_state, a_bipedObject, a_preservedBuildGroup);
		for (const auto buildGroup : buildGroups) {
			spdlog::debug(
				"clearing stale hair-slot armor system group actor={} bipedObject={} buildGroup={} preservedBuildGroup={} object={} xml='{}' reason={}",
				static_cast<void*>(a_state.actor),
				std::to_underlying(a_bipedObject),
				buildGroup,
				a_preservedBuildGroup,
				static_cast<void*>(a_object),
				a_physicsXmlPath,
				a_reason);
		}
		if (!buildGroups.empty()) {
			ClearBuildGroupsLocked(a_state, buildGroups, a_resetToStoredLocalPose);
		}

		const auto removedRecords = PrunePhysicsRecordsForBipedObjectLocked(a_state, a_bipedObject, a_preservedBuildGroup);
		if (!buildGroups.empty() || removedRecords > 0) {
			spdlog::debug(
				"cleared stale hair-slot armor ownership actor={} bipedObject={} groups={} records={} preservedBuildGroup={} object={} xml='{}' reason={}",
				static_cast<void*>(a_state.actor),
				std::to_underlying(a_bipedObject),
				buildGroups.size(),
				removedRecords,
				a_preservedBuildGroup,
				static_cast<void*>(a_object),
				a_physicsXmlPath,
				a_reason);
		}

		return static_cast<std::uint32_t>(buildGroups.size());
	}

	std::uint32_t Fo4PhysicsWorld::CollectHeadPartGroupsLocked(
		const Fo4SkinnedMeshSystem& a_state,
		const BuildDomain a_domain,
		std::vector<std::uint64_t>& a_buildGroups) const
	{
		std::uint32_t matchedRecords = 0;
		for (const auto& record : a_state.headPartRecords) {
			if (record.domain != a_domain || record.buildGroup == 0) {
				continue;
			}

			++matchedRecords;
			if (std::ranges::find(a_buildGroups, record.buildGroup) == a_buildGroups.end()) {
				a_buildGroups.push_back(record.buildGroup);
			}
		}
		return matchedRecords;
	}

	bool Fo4PhysicsWorld::HasActiveHairSlotArmorLocked(const Fo4SkinnedMeshSystem& a_state) const
	{
		if (std::ranges::any_of(a_state.armorRecords, [&](const ArmorPhysicsRecord& a_record) {
				return IsHairBipedObject(a_record.bipedObject) &&
					std::ranges::any_of(a_record.buildGroups, [&](const std::uint64_t a_buildGroup) {
						return BuildGroupHasBodyLocked(a_state, a_buildGroup) || BuildGroupHasMeshLocked(a_state, a_buildGroup);
					});
			})) {
			return true;
		}
		return std::ranges::any_of(a_state.buildGroups, [](const BuildGroupRecord& a_runtime) {
			return a_runtime.domain == BuildDomain::kArmor && IsHairBipedObject(a_runtime.bipedObject);
		});
	}

	bool Fo4PhysicsWorld::BuildGroupsIncludeHairSlotArmorLocked(
		const Fo4SkinnedMeshSystem& a_state,
		const std::span<const std::uint64_t> a_buildGroups) const
	{
		if (a_buildGroups.empty()) {
			return false;
		}

		const auto containsGroup = [&a_buildGroups](const std::uint64_t a_buildGroup) {
			return a_buildGroup != 0 && std::ranges::find(a_buildGroups, a_buildGroup) != a_buildGroups.end();
		};

		if (std::ranges::any_of(a_state.armorRecords, [&](const ArmorPhysicsRecord& a_record) {
				return IsHairBipedObject(a_record.bipedObject) && std::ranges::any_of(a_record.buildGroups, containsGroup);
			})) {
			return true;
		}
		if (std::ranges::any_of(a_state.attachmentRecords, [&](const AttachmentPhysicsRecord& a_record) {
				return IsHairBipedObject(a_record.bipedObject) && std::ranges::any_of(a_record.buildGroups, containsGroup);
			})) {
			return true;
		}
		if (std::ranges::any_of(a_state.buildGroups, [&](const BuildGroupRecord& a_runtime) {
				return containsGroup(a_runtime.buildGroup) && a_runtime.domain == BuildDomain::kArmor && IsHairBipedObject(a_runtime.bipedObject);
			})) {
			return true;
		}
		if (std::ranges::any_of(a_state.meshes, [&](const MeshRecord& a_mesh) {
				return containsGroup(a_mesh.buildGroup) && a_mesh.domain == BuildDomain::kArmor && IsHairBipedObject(a_mesh.bipedObject);
			})) {
			return true;
		}
		return std::ranges::any_of(a_state.bodies, [&](const BoneRecord& a_body) {
			if (!a_body.buildGroupDomains.empty() || !a_body.buildGroupBipedObjects.empty()) {
				for (const auto& [buildGroup, domain] : a_body.buildGroupDomains) {
					if (containsGroup(buildGroup) && domain == BuildDomain::kArmor) {
						const auto biped = std::ranges::find_if(a_body.buildGroupBipedObjects, [buildGroup](const auto& a_entry) {
							return a_entry.first == buildGroup;
						});
						if (biped != a_body.buildGroupBipedObjects.end() && IsHairBipedObject(biped->second)) {
							return true;
						}
					}
				}
				return false;
			}
			return containsGroup(a_body.buildGroup) && IsHairBipedObject(a_body.bipedObject);
		});
	}

	bool Fo4PhysicsWorld::ClearBuildGroupsForObjectLocked(Fo4SkinnedMeshSystem& a_state, RE::NiAVObject* a_object)
	{
		auto buildGroups = CollectBuildGroupsForObjectLocked(a_state, a_object);
		if (buildGroups.empty()) {
			return false;
		}

		ClearBuildGroupsLocked(a_state, buildGroups);
		return true;
	}

	bool Fo4PhysicsWorld::ClearBuildGroupsForBipedObjectLocked(Fo4SkinnedMeshSystem& a_state, const RE::BIPED_OBJECT a_bipedObject)
	{
		if (a_bipedObject == RE::BIPED_OBJECT::kTotal) {
			return false;
		}

		const auto attachmentCount = static_cast<std::size_t>(std::ranges::count_if(a_state.attachmentRecords, [a_bipedObject](const AttachmentPhysicsRecord& a_record) {
			return a_record.bipedObject == a_bipedObject && !a_record.buildGroups.empty();
		}));
		const auto armorRecordCount = static_cast<std::size_t>(std::ranges::count_if(a_state.armorRecords, [a_bipedObject](const ArmorPhysicsRecord& a_record) {
			return a_record.bipedObject == a_bipedObject;
		}));
		if (attachmentCount > 1 || armorRecordCount > 1) {
			spdlog::warn(
				"refusing biped-wide system clear actor={} bipedObject={} attachments={} armorRecords={} because multiple same-slot systems are tracked",
				static_cast<void*>(a_state.actor),
				std::to_underlying(a_bipedObject),
				attachmentCount,
				armorRecordCount);
			return false;
		}

		std::vector<std::uint64_t> buildGroups;
		for (const auto& attachment : a_state.attachmentRecords) {
			if (attachment.bipedObject == a_bipedObject) {
				buildGroups.insert(buildGroups.end(), attachment.buildGroups.begin(), attachment.buildGroups.end());
			}
		}
		const auto appendGroup = [&buildGroups](const std::uint64_t a_buildGroup) {
			if (a_buildGroup != 0 && std::ranges::find(buildGroups, a_buildGroup) == buildGroups.end()) {
				buildGroups.push_back(a_buildGroup);
			}
		};

		for (const auto& runtime : a_state.buildGroups) {
			if (runtime.bipedObject == a_bipedObject) {
				appendGroup(runtime.buildGroup);
			}
		}

		for (const auto& meshRecord : a_state.meshes) {
			if (meshRecord.bipedObject == a_bipedObject) {
				appendGroup(meshRecord.buildGroup);
			}
		}

		for (const auto& boneRecord : a_state.bodies) {
			if (!boneRecord.buildGroupBipedObjects.empty()) {
				for (const auto& [buildGroup, bipedObject] : boneRecord.buildGroupBipedObjects) {
					if (bipedObject == a_bipedObject) {
						appendGroup(buildGroup);
					}
				}
			} else if (boneRecord.bipedObject == a_bipedObject) {
				appendGroup(boneRecord.buildGroup);
			}
		}

		if (buildGroups.empty()) {
			return false;
		}

		ClearBuildGroupsLocked(a_state, buildGroups);
		std::erase_if(a_state.armorRecords, [a_bipedObject](const ArmorPhysicsRecord& a_record) {
			return a_record.bipedObject == a_bipedObject;
		});
		return true;
	}

	bool Fo4PhysicsWorld::ClearBuildGroupsForBoneNamesLocked(Fo4SkinnedMeshSystem& a_state, const std::span<const std::string> a_boneNames, const BuildDomain a_domain)
	{
		if (a_boneNames.empty()) {
			return false;
		}
		if (a_domain == BuildDomain::kArmor) {
			spdlog::warn(
				"refusing to clear armor system groups by bone names actor={} names={} because actor skeleton bones may be shared across armor XMLs",
				static_cast<void*>(a_state.actor),
				a_boneNames.size());
			return false;
		}

		std::vector<std::uint64_t> buildGroups;
		const auto appendGroup = [&buildGroups](const std::uint64_t a_buildGroup) {
			if (a_buildGroup != 0 && std::ranges::find(buildGroups, a_buildGroup) == buildGroups.end()) {
				buildGroups.push_back(a_buildGroup);
			}
		};
		for (const auto& boneRecord : a_state.bodies) {
			if (boneRecord.boneName.empty()) {
				continue;
			}
			const auto nameMatched = std::ranges::any_of(a_boneNames, [&boneRecord](const std::string& a_boneName) {
				return PhysicsNamesEqual(boneRecord.boneName, a_boneName);
			});
			if (!nameMatched) {
				continue;
			}

			if (!boneRecord.buildGroupDomains.empty()) {
				for (const auto& [buildGroup, domain] : boneRecord.buildGroupDomains) {
					if (domain == a_domain) {
						appendGroup(buildGroup);
					}
				}
			} else if (boneRecord.buildGroup != 0) {
				appendGroup(boneRecord.buildGroup);
			}
		}

		if (buildGroups.empty()) {
			return false;
		}

		ClearBuildGroupsLocked(a_state, buildGroups);
		return true;
	}

	bool Fo4PhysicsWorld::ClearBuildGroupsByDomainLocked(Fo4SkinnedMeshSystem& a_state, const BuildDomain a_domain)
	{
		std::vector<std::uint64_t> buildGroups;
		for (const auto& runtime : a_state.buildGroups) {
			if (runtime.buildGroup != 0 && runtime.domain == a_domain && std::ranges::find(buildGroups, runtime.buildGroup) == buildGroups.end()) {
				buildGroups.push_back(runtime.buildGroup);
			}
		}
		for (const auto& meshRecord : a_state.meshes) {
			if (meshRecord.buildGroup != 0 && meshRecord.domain == a_domain && std::ranges::find(buildGroups, meshRecord.buildGroup) == buildGroups.end()) {
				buildGroups.push_back(meshRecord.buildGroup);
			}
		}
		for (const auto& constraintRecord : a_state.constraints) {
			if (constraintRecord.buildGroup != 0 && constraintRecord.domain == a_domain && std::ranges::find(buildGroups, constraintRecord.buildGroup) == buildGroups.end()) {
				buildGroups.push_back(constraintRecord.buildGroup);
			}
		}
		for (const auto& boneRecord : a_state.bodies) {
			for (const auto& [buildGroup, domain] : boneRecord.buildGroupDomains) {
				if (buildGroup != 0 && domain == a_domain && std::ranges::find(buildGroups, buildGroup) == buildGroups.end()) {
					buildGroups.push_back(buildGroup);
				}
			}
		}

		if (buildGroups.empty()) {
			return false;
		}

		ClearBuildGroupsLocked(a_state, buildGroups);
		return true;
	}

	void Fo4PhysicsWorld::ClearHeadPhysicsTrackingLocked(Fo4SkinnedMeshSystem& a_state, const std::string_view a_reason)
	{
		const auto clearedHead = ClearBuildGroupsByDomainLocked(a_state, BuildDomain::kHead);
		const auto clearedHair = ClearBuildGroupsByDomainLocked(a_state, BuildDomain::kHair);

		const auto recordCount = a_state.headPartRecords.size();
		a_state.headPartRecords.clear();
		spdlog::debug(
			"cleared head/hair system tracking actor={} reason={} clearedHead={} clearedHair={} headPartRecords={}",
			static_cast<void*>(a_state.actor),
			a_reason,
			clearedHead,
			clearedHair,
			recordCount);
	}

	void Fo4PhysicsWorld::ClearBuildGroupsLocked(
		Fo4SkinnedMeshSystem& a_state,
		const std::vector<std::uint64_t>& a_buildGroups,
		const bool a_resetToStoredLocalPose)
	{
		if (a_buildGroups.empty()) {
			return;
		}
		if (a_resetToStoredLocalPose) {
			ResetBuildGroupsToStoredLocalPoseLocked(a_state, a_buildGroups, "clear-groups");
		}

		const auto containsGroup = [&a_buildGroups](const std::uint64_t a_buildGroup) {
			return std::ranges::find(a_buildGroups, a_buildGroup) != a_buildGroups.end();
		};
		const auto allGroupsRemoved = [&containsGroup](const BoneRecord& a_body) {
			return !a_body.buildGroups.empty() ?
				std::ranges::all_of(a_body.buildGroups, containsGroup) :
				containsGroup(a_body.buildGroup);
		};
		for (auto& constraint : a_state.constraints) {
			if (!constraint.constraint || !containsGroup(constraint.buildGroup)) {
				continue;
			}
			if (dynamicsWorld_ && a_state.m_world == dynamicsWorld_.get()) {
				dynamicsWorld_->removeConstraint(constraint.GetConstraint());
			}
			if (auto released = a_state.ReleaseConstraint(constraint.constraint.get())) {
				constraint.constraint = std::move(released);
			}
		}

		for (auto& mesh : a_state.meshes) {
			if (!mesh.body || !containsGroup(mesh.buildGroup)) {
				continue;
			}
			if (dynamicsWorld_ && a_state.m_world == dynamicsWorld_.get()) {
				dynamicsWorld_->removeCollisionObject(mesh.body.get());
			}
			if (auto released = a_state.ReleaseMesh(mesh.body.get())) {
				mesh.body = std::move(released);
			}
		}

		for (auto& body : a_state.bodies) {
			if (!body.bone || !allGroupsRemoved(body)) {
				continue;
			}
			if (dynamicsWorld_ && a_state.m_world == dynamicsWorld_.get()) {
				dynamicsWorld_->removeRigidBody(std::addressof(body.bone->m_rig));
			}
			if (auto released = a_state.ReleaseBone(body.bone.get())) {
				body.bone = std::move(released);
			}
		}

		std::vector<Fo4SkinnedMeshBone::ActiveSkinSlot> activeSkinSlots;
		for (const auto& boneRecord : a_state.bodies) {
			if (allGroupsRemoved(boneRecord) || !boneRecord.bone) {
				continue;
			}
			boneRecord.bone->CollectSkinWorldTransformSlots(activeSkinSlots);
		}
		for (const auto& suspendedSlot : a_state.suspendedSkinSlots) {
			if (!suspendedSlot.skin || containsGroup(suspendedSlot.buildGroup)) {
				continue;
			}
			activeSkinSlots.push_back({
				.skin = suspendedSlot.skin.get(),
				.index = suspendedSlot.index,
				.buildGroup = suspendedSlot.buildGroup,
			});
		}

		for (auto& boneRecord : a_state.bodies) {
			if (!boneRecord.bone) {
				continue;
			}

			for (const auto buildGroup : a_buildGroups) {
				boneRecord.bone->RemoveSkinWorldTransformsForBuildGroup(buildGroup, activeSkinSlots);
			}
		}
		RestoreSuspendedSkinSlotsLocked(a_state, a_buildGroups, activeSkinSlots);

		const auto meshCount = std::erase_if(a_state.meshes, [&containsGroup](const MeshRecord& a_mesh) {
			return containsGroup(a_mesh.buildGroup);
		});
		const auto constraintCount = std::erase_if(a_state.constraints, [&containsGroup](const ConstraintRecord& a_constraint) {
			return containsGroup(a_constraint.buildGroup);
		});
		const auto runtimeCount = std::erase_if(a_state.buildGroups, [&containsGroup](const BuildGroupRecord& a_runtime) {
			return containsGroup(a_runtime.buildGroup);
		});

		for (auto& body : a_state.bodies) {
			if (body.buildGroups.empty() && body.buildGroup != 0) {
				body.buildGroups.push_back(body.buildGroup);
			}
			if (body.buildGroupDomains.empty() && body.buildGroup != 0) {
				body.buildGroupDomains.push_back({ body.buildGroup, BuildDomain::kArmor });
			}
			if (body.buildGroupBipedObjects.empty() && body.buildGroup != 0) {
				body.buildGroupBipedObjects.push_back({ body.buildGroup, body.bipedObject });
			}
			std::erase_if(body.buildGroups, [&containsGroup](const std::uint64_t a_buildGroup) {
				return containsGroup(a_buildGroup);
			});
			std::erase_if(body.buildGroupDomains, [&containsGroup](const auto& a_entry) {
				return containsGroup(a_entry.first);
			});
			std::erase_if(body.buildGroupBipedObjects, [&containsGroup](const auto& a_entry) {
				return containsGroup(a_entry.first);
			});
			if (!body.buildGroups.empty()) {
				body.buildGroup = body.buildGroups.front();
				const auto biped = std::ranges::find_if(body.buildGroupBipedObjects, [&body](const auto& a_entry) {
					return a_entry.first == body.buildGroup;
				});
				body.bipedObject = biped != body.buildGroupBipedObjects.end() ? biped->second : RE::BIPED_OBJECT::kTotal;
			}
		}
		const auto bodyCount = std::erase_if(a_state.bodies, [](const BoneRecord& a_body) {
			return a_body.buildGroups.empty();
		});
		if (!a_state.HasPhysics() && dynamicsWorld_ && a_state.m_world == dynamicsWorld_.get()) {
			dynamicsWorld_->removeSkinnedMeshSystem(std::addressof(a_state));
			a_state.ClearSystemObjects();
		}
		const auto localPoseCount = std::erase_if(a_state.attachmentBoneLocalPoses, [&containsGroup](const AttachmentBoneLocalPose& a_pose) {
			return containsGroup(a_pose.buildGroup);
		});
		std::erase_if(a_state.attachmentRecords, [&containsGroup](AttachmentPhysicsRecord& a_record) {
			std::erase_if(a_record.buildGroups, [&containsGroup](const std::uint64_t a_buildGroup) {
				return containsGroup(a_buildGroup);
			});
			if (!a_record.buildGroups.empty()) {
				return false;
			}
			a_record.attachedObject = nullptr;
			a_record.sourceObject = nullptr;
			return true;
		});
		std::erase_if(a_state.headPartRecords, [&containsGroup](HeadPartPhysicsRecord& a_record) {
			if (a_record.buildGroup == 0 || !containsGroup(a_record.buildGroup)) {
				return false;
			}
			a_record.object = nullptr;
			a_record.sourceObject = nullptr;
			a_record.sourceRoot = nullptr;
			return true;
		});
		const auto armorRecordCount = std::erase_if(a_state.armorRecords, [&containsGroup](ArmorPhysicsRecord& a_record) {
			if (a_record.buildGroups.empty()) {
				return false;
			}
			std::erase_if(a_record.buildGroups, [&containsGroup](const std::uint64_t a_buildGroup) {
				return containsGroup(a_buildGroup);
			});
			if (!a_record.buildGroups.empty()) {
				return false;
			}
			a_record.attachedObject = nullptr;
			a_record.sourceObject = nullptr;
			return true;
		});
		if (!a_state.HasPhysics()) {
		}

		spdlog::debug(
			"cleared system physics groups={} buildGroups={} bodies={} meshes={} constraints={} attachmentBoneLocalPoses={} armorRecords={} for actor={}",
			a_buildGroups.size(),
			runtimeCount,
			bodyCount,
			meshCount,
			constraintCount,
			localPoseCount,
			armorRecordCount,
			static_cast<void*>(a_state.actor));
	}

	void Fo4PhysicsWorld::ClearAllSystemsLocked()
	{
		for (auto& actorState : systems_) {
			ClearSystemLocked(*actorState);
		}
		systems_.clear();
		suspendedActors_.clear();
	}

	void Fo4PhysicsWorld::ResumeFromLoadingMenuLocked()
	{
		std::size_t resetBodies = 0;
		for (auto& actorStatePointer : systems_) {
			auto& actorState = *actorStatePointer;
			if (actorState.IsInactive()) {
				continue;
			}
			actorState.lastWritebackFrame = 0;
			actorState.lastWritebackSource = WritebackSource::kUnknown;
			actorState.currentWindFactor = 1.0F;
			resetBodies += std::ranges::count_if(actorState.bodies, [](const BoneRecord& a_body) {
				return static_cast<bool>(a_body.bone);
			});
			actorState.readTransform(0.0F);
		}
		if (dynamicsWorld_) {
			dynamicsWorld_->clearForces();
		}
		loadingPhysicsSuspended_ = false;
		loadingMenuDepth_ = 0;
		ResetStepClockLocked();
		spdlog::debug("loading menu resume reset {} system physics bodies to current node poses", resetBodies);
	}

	bool Fo4PhysicsWorld::BuildGroupHasMeshLocked(const Fo4SkinnedMeshSystem& a_state, const std::uint64_t a_buildGroup) const
	{
		if (a_buildGroup == 0) {
			return false;
		}

		return std::ranges::any_of(a_state.meshes, [a_buildGroup](const MeshRecord& a_mesh) {
			return a_mesh.buildGroup == a_buildGroup && a_mesh.body;
		});
	}

	bool Fo4PhysicsWorld::BuildGroupHasBodyLocked(const Fo4SkinnedMeshSystem& a_state, const std::uint64_t a_buildGroup) const
	{
		if (a_buildGroup == 0) {
			return false;
		}

		return std::ranges::any_of(a_state.bodies, [a_buildGroup](const BoneRecord& a_body) {
			return BodyHasBuildGroup(a_body, a_buildGroup) && a_body.bone;
		});
	}

	bool Fo4PhysicsWorld::BuildGroupIsRecordableLocked(
		const Fo4SkinnedMeshSystem& a_state,
		const std::uint64_t a_buildGroup,
		const BuildDomain a_domain,
		const RE::BIPED_OBJECT a_bipedObject) const
	{
		(void)a_domain;
		(void)a_bipedObject;

		if (!BuildGroupHasBodyLocked(a_state, a_buildGroup)) {
			return false;
		}

		return true;
	}

	void Fo4PhysicsWorld::UpdateBuildGroupMeshesLocked(Fo4SkinnedMeshSystem& a_state, const std::uint64_t a_buildGroup)
	{
		if (a_buildGroup == 0) {
			return;
		}

		for (auto& meshRecord : a_state.meshes) {
			if (!meshRecord.body || meshRecord.buildGroup != a_buildGroup) {
				continue;
			}

			meshRecord.body->internalUpdate();
		}
	}

	void Fo4PhysicsWorld::ActivateBuildGroupLocked(Fo4SkinnedMeshSystem& a_state, const std::uint64_t a_buildGroup)
	{
		if (!dynamicsWorld_ || a_buildGroup == 0) {
			return;
		}

		const auto activateSystem = a_state.m_world == nullptr;
		std::uint32_t committedMeshes = 0;
		for (auto& meshRecord : a_state.meshes) {
			if (!meshRecord.body || a_state.ContainsMesh(meshRecord.body.get()) || meshRecord.buildGroup != a_buildGroup) {
				continue;
			}

			if (!a_state.AddMesh(meshRecord.body)) {
				continue;
			}
			if (!activateSystem) {
				dynamicsWorld_->addCollisionObject(meshRecord.body.get(), 1, 1);
			}
			++committedMeshes;
		}

		std::uint32_t committedBodies = 0;
		for (auto& boneRecord : a_state.bodies) {
			if (!boneRecord.bone || a_state.ContainsBone(boneRecord.bone.get()) || !BodyHasBuildGroup(boneRecord, a_buildGroup)) {
				continue;
			}

			if (!a_state.AddBone(boneRecord.bone)) {
				continue;
			}
			if (!activateSystem) {
				// hdtSMP bones are solver/constraint bodies; mesh collisions are handled by the custom dispatcher.
				dynamicsWorld_->addRigidBody(std::addressof(boneRecord.bone->m_rig), 0, 0);
			}
			++committedBodies;
		}

		std::uint32_t committedConstraints = 0;
		for (auto& constraintRecord : a_state.constraints) {
			if (!constraintRecord.constraint || a_state.ContainsConstraint(constraintRecord.constraint.get()) || constraintRecord.buildGroup != a_buildGroup) {
				continue;
			}

			if (!a_state.AddConstraint(constraintRecord.constraint)) {
				continue;
			}
			if (!activateSystem) {
				dynamicsWorld_->addConstraint(constraintRecord.GetConstraint(), true);
			}
			++committedConstraints;
		}

		if (committedBodies > 0 || committedMeshes > 0 || committedConstraints > 0) {
			if (activateSystem) {
				dynamicsWorld_->addSkinnedMeshSystem(std::addressof(a_state));
			}
			auto runtime = std::ranges::find_if(a_state.buildGroups, [a_buildGroup](const BuildGroupRecord& a_runtime) {
				return a_runtime.buildGroup == a_buildGroup;
			});
			if (runtime == a_state.buildGroups.end()) {
				BuildGroupRecord newRuntime;
				newRuntime.buildGroup = a_buildGroup;
				newRuntime.pendingResetPhysicsRead = true;
				newRuntime.pendingResetPhysicsWriteback = true;
				if (const auto mesh = std::ranges::find_if(a_state.meshes, [a_buildGroup](const MeshRecord& a_mesh) {
						return a_mesh.buildGroup == a_buildGroup;
					});
					mesh != a_state.meshes.end()) {
					newRuntime.domain = mesh->domain;
					newRuntime.bipedObject = mesh->bipedObject;
				} else if (const auto constraint = std::ranges::find_if(a_state.constraints, [a_buildGroup](const ConstraintRecord& a_constraint) {
						return a_constraint.buildGroup == a_buildGroup;
					});
					constraint != a_state.constraints.end()) {
					newRuntime.domain = constraint->domain;
				}
				a_state.buildGroups.push_back(newRuntime);
				runtime = std::prev(a_state.buildGroups.end());
			}
			for (auto& meshRecord : a_state.meshes) {
				if (meshRecord.buildGroup != a_buildGroup || !meshRecord.body) {
					continue;
				}
				runtime->domain = meshRecord.domain;
				runtime->bipedObject = meshRecord.bipedObject;
			}
			for (auto& boneRecord : a_state.bodies) {
				if (!BodyHasBuildGroup(boneRecord, a_buildGroup) || !boneRecord.bone) {
					continue;
				}
				if (runtime->bipedObject == RE::BIPED_OBJECT::kTotal) {
					const auto biped = std::ranges::find_if(boneRecord.buildGroupBipedObjects, [a_buildGroup](const auto& a_entry) {
						return a_entry.first == a_buildGroup;
					});
					runtime->bipedObject = biped != boneRecord.buildGroupBipedObjects.end() ? biped->second : boneRecord.bipedObject;
				}
				const auto domain = std::ranges::find_if(boneRecord.buildGroupDomains, [a_buildGroup](const auto& a_entry) {
					return a_entry.first == a_buildGroup;
				});
				if (domain != boneRecord.buildGroupDomains.end()) {
					runtime->domain = domain->second;
				}
			}
			for (auto& constraintRecord : a_state.constraints) {
				if (constraintRecord.buildGroup != a_buildGroup || !constraintRecord.constraint) {
					continue;
				}
				runtime->domain = constraintRecord.domain;
			}
			ResetBuildGroupToCurrentPoseLocked(a_state, a_buildGroup);
			UpdateBuildGroupMeshesLocked(a_state, a_buildGroup);
			spdlog::debug(
				"committed system build group to Bullet actor={} buildGroup={} domain={} bipedObject={} bodies={} meshes={} constraints={}",
				static_cast<void*>(a_state.actor),
				a_buildGroup,
				BuildDomainName(runtime->domain),
				std::to_underlying(runtime->bipedObject),
				committedBodies,
				committedMeshes,
				committedConstraints);
		}
	}
}
