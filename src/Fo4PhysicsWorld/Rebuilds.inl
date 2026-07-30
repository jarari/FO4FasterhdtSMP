// Included by ../Fo4PhysicsWorld.cpp.
// Split from Fo4PhysicsWorld.cpp. Do not compile this file separately.

namespace Smp
{
	Fo4SkinnedMeshSystem* Fo4PhysicsWorld::FindSystemLocked(RE::Actor* a_actor, const bool a_firstPerson)
	{
		const auto found = std::ranges::find_if(systems_, [a_actor, a_firstPerson](const auto& a_state) {
			return a_state->actor == a_actor && a_state->firstPerson == a_firstPerson;
		});
		return found == systems_.end() ? nullptr : found->get();
	}

	Fo4SkinnedMeshSystem& Fo4PhysicsWorld::GetOrCreateSystemLocked(RE::Actor* a_actor, const bool a_firstPerson)
	{
		if (auto* state = FindSystemLocked(a_actor, a_firstPerson)) {
			return *state;
		}

		auto state = RE::make_smart<Fo4SkinnedMeshSystem>();
		state->actor = a_actor;
		state->actorHandle = RE::BSPointerHandleManagerInterface<RE::Actor>::GetHandle(a_actor);
		state->firstPerson = a_firstPerson;
		systems_.push_back(state);
		return *state;
	}

	bool Fo4PhysicsWorld::IsSystemValidLocked(Fo4SkinnedMeshSystem& a_state)
	{
		if (!a_state.actor || !a_state.actorHandle) {
			spdlog::debug("dropping system physics state with missing actor handle actor={}", static_cast<void*>(a_state.actor));
			return false;
		}

		auto resolvedActor = a_state.actorHandle.get();
		if (!resolvedActor || resolvedActor.get() != a_state.actor) {
			spdlog::debug("dropping system physics state with stale actor handle actor={}", static_cast<void*>(a_state.actor));
			return false;
		}

		auto* root = resolvedActor->Get3D(a_state.firstPerson);
		if (!root && !a_state.firstPerson) {
			root = resolvedActor->Get3D();
		}
		if (!root) {
			if (!a_state.armorRecords.empty() || a_state.HasPhysics() || a_state.suspended) {
				if (!a_state.suspended || a_state.HasPhysics()) {
					spdlog::debug(
						"suspending system physics state for actor={} firstPerson={} with no current 3D; preserved armorRecords={} hadRuntime={}",
						static_cast<void*>(a_state.actor),
						a_state.firstPerson,
						a_state.armorRecords.size(),
						a_state.HasPhysics());
					SuspendSystemLocked(a_state);
				}
				return true;
			}
			spdlog::debug(
				"dropping system physics state for actor={} firstPerson={} with no current 3D",
				static_cast<void*>(a_state.actor),
				a_state.firstPerson);
			return false;
		}

		const auto* player = RE::PlayerCharacter::GetSingleton();
		if (a_state.actor != player) {
			if (!enableNpcPhysics_) {
				spdlog::debug("dropping system physics state for actor={} because NPC physics is disabled", static_cast<void*>(a_state.actor));
				return false;
			}

			if (!IsActorWithinDistanceLocked(a_state.actor)) {
				if (a_state.IsActive()) {
					spdlog::debug(
						"deactivating system physics state for actor={} beyond maxActorDistance={}",
						static_cast<void*>(a_state.actor),
						maxActorDistance_);
					DeactivateSystemLocked(a_state);
				}
				return true;
			}

			if (!IsActorInReferenceCullView(a_state.actor, root, a_state.firstPerson)) {
				if (a_state.IsActive()) {
					spdlog::debug(
						"deactivating system physics state for actor={} because reference view culler marks it inactive",
						static_cast<void*>(a_state.actor));
					DeactivateSystemLocked(a_state);
				}
				return true;
			}
		}

		return true;
	}

	bool Fo4PhysicsWorld::IsActorWithinDistanceLocked(RE::Actor* a_actor) const
	{
		const auto* player = RE::PlayerCharacter::GetSingleton();
		if (!a_actor || !player || a_actor == player || maxActorDistance_ <= 0.0F) {
			return true;
		}

		const auto maxDistanceSquared = maxActorDistance_ * maxActorDistance_;
		return DistanceSquared(a_actor->GetPosition(), player->GetPosition()) <= maxDistanceSquared;
	}

	void Fo4PhysicsWorld::PruneInvalidSystemsLocked()
	{
		for (auto& actorState : systems_) {
			if (!IsSystemValidLocked(*actorState)) {
				ClearSystemLocked(*actorState);
				actorState->actor = nullptr;
				actorState->actorHandle.reset();
			}
		}

		std::erase_if(systems_, [](const auto& a_state) {
			return !a_state->actor && !a_state->HasPhysics();
		});
	}

	void Fo4PhysicsWorld::EnforceActorBudgetLocked()
	{
		auto activeActors = static_cast<std::size_t>(std::ranges::count_if(systems_, [](const auto& a_state) {
			return a_state->IsActive();
		}));
		if (activeActors <= currentMaxActiveActors_) {
			return;
		}

		const auto* player = RE::PlayerCharacter::GetSingleton();
		while (activeActors > currentMaxActiveActors_) {
			auto victim = systems_.end();
			auto victimDistanceSquared = -1.0F;
			for (auto it = systems_.begin(); it != systems_.end(); ++it) {
				if (!(*it)->IsActive()) {
					continue;
				}
				auto resolvedActor = (*it)->actorHandle.get();
				if (!resolvedActor || resolvedActor.get() != (*it)->actor) {
					continue;
				}
				if (player && (*it)->actor == player) {
					continue;
				}

				auto distanceSquared = std::numeric_limits<float>::max();
				if (player) {
					distanceSquared = DistanceSquared(resolvedActor->GetPosition(), player->GetPosition());
				}

				if (victim == systems_.end() || distanceSquared > victimDistanceSquared) {
					victim = it;
					victimDistanceSquared = distanceSquared;
				}
			}

			if (victim == systems_.end()) {
				return;
			}

			spdlog::debug(
				"deactivating system physics state for actor={} because active actor budget shrank to {}",
				static_cast<void*>((*victim)->actor),
				currentMaxActiveActors_);
			DeactivateSystemLocked(**victim);
			--activeActors;
		}
	}

	bool Fo4PhysicsWorld::ShouldBuildSuspendedArmorCandidateLocked(const LifecycleEvent& a_event) const
	{
		return IsArmorAttachCandidate(a_event.type) && !a_event.physicsXmlPath.empty();
	}

	void Fo4PhysicsWorld::DeactivateBuiltSystemIfInactiveLocked(Fo4SkinnedMeshSystem& a_state, const LifecycleEvent& a_event)
	{
		if (!a_state.IsActive()) {
			return;
		}

		const auto* player = RE::PlayerCharacter::GetSingleton();
		if (a_state.actor == player) {
			return;
		}

		bool shouldSuspend = false;
		const char* reason = nullptr;
		if (!IsActorWithinDistanceLocked(a_state.actor)) {
			shouldSuspend = true;
			reason = "distance";
		} else if (!IsActorInReferenceCullView(a_state.actor, a_event.object, a_event.firstPerson)) {
			shouldSuspend = true;
			reason = "view";
		}
		const auto activeActors = static_cast<std::size_t>(std::ranges::count_if(systems_, [](const auto& a_candidate) {
			return a_candidate->IsActive();
		}));
		if (!shouldSuspend && activeActors > currentMaxActiveActors_) {
			shouldSuspend = true;
			reason = "budget";
		}

		if (!shouldSuspend) {
			return;
		}

		std::vector<std::uint64_t> buildGroups;
		for (const auto& runtime : a_state.buildGroups) {
			if (runtime.buildGroup != 0 && std::ranges::find(buildGroups, runtime.buildGroup) == buildGroups.end()) {
				buildGroups.push_back(runtime.buildGroup);
			}
		}

		spdlog::debug(
			"deactivating freshly built armor system actor={} reason={} activeActors={} actorCap={} event={} buildGroups={}",
			static_cast<void*>(a_state.actor),
			reason ? reason : "unknown",
			activeActors,
			currentMaxActiveActors_,
			ToString(a_event.type),
			buildGroups.size());
		DeactivateSystemLocked(a_state);
	}

	void Fo4PhysicsWorld::SuspendActorCandidateLocked(
		RE::Actor* a_actor,
		const bool a_firstPerson,
		std::vector<ArmorPhysicsRecord> a_armorRecords)
	{
		const auto* player = RE::PlayerCharacter::GetSingleton();
		if (!a_actor || a_actor == player || !enableNpcPhysics_) {
			return;
		}
		if (a_armorRecords.empty()) {
			spdlog::trace(
				"skipping empty suspended system physics candidate actor={} firstPerson={}",
				static_cast<void*>(a_actor),
				a_firstPerson);
			return;
		}

		for (auto& candidate : suspendedActors_) {
			const auto resolvedActor = candidate.actorHandle.get();
			if (resolvedActor && resolvedActor.get() == a_actor) {
				candidate.firstPerson = a_firstPerson;
				StripQueuedArmorRuntimePointers(a_armorRecords);
				candidate.armorRecords = std::move(a_armorRecords);
				return;
			}
		}

		auto handle = RE::BSPointerHandleManagerInterface<RE::Actor>::GetHandle(a_actor);
		if (!handle) {
			return;
		}

		StripQueuedArmorRuntimePointers(a_armorRecords);
		suspendedActors_.push_back({
			.actorHandle = handle,
			.firstPerson = a_firstPerson,
			.armorRecords = std::move(a_armorRecords),
		});
		spdlog::debug(
			"suspended system physics candidate actor={} firstPerson={} armorRecords={} until view/budget allows rebuild",
			static_cast<void*>(a_actor),
			a_firstPerson,
			suspendedActors_.back().armorRecords.size());
	}

	void Fo4PhysicsWorld::TryReactivateSuspendedActorsLocked()
	{
		if (suspendedActors_.empty()) {
			return;
		}

		if (!enableNpcPhysics_) {
			suspendedActors_.clear();
			return;
		}

		for (auto it = suspendedActors_.begin(); it != suspendedActors_.end();) {
			const auto activeActors = static_cast<std::size_t>(std::ranges::count_if(systems_, [](const auto& a_state) {
				return a_state->IsActive();
			}));
			if (activeActors >= currentMaxActiveActors_) {
				return;
			}

			auto resolvedActor = it->actorHandle.get();
			if (!resolvedActor) {
				it = suspendedActors_.erase(it);
				continue;
			}

			auto* actor = resolvedActor.get();
			auto* existingState = actor ? FindSystemLocked(actor, it->firstPerson) : nullptr;
			if (!actor) {
				it = suspendedActors_.erase(it);
				continue;
			}
			if (existingState) {
				if ((existingState->suspended || existingState->IsInactive()) && !it->armorRecords.empty()) {
					for (auto& record : it->armorRecords) {
						MergeArmorPhysicsRecord(existingState->armorRecords, std::move(record));
					}
					spdlog::debug(
						"merged suspended armor records into inactive system state actor={} firstPerson={} armorRecords={}",
						static_cast<void*>(actor),
						it->firstPerson,
						existingState->armorRecords.size());
				}
				it = suspendedActors_.erase(it);
				continue;
			}

			auto* root = actor->Get3D(it->firstPerson);
			if (!root && !it->firstPerson) {
				root = actor->Get3D();
			}
			if (!root) {
				++it;
				continue;
			}
			if (!IsActorWithinDistanceLocked(actor)) {
				++it;
				continue;
			}
			if (!IsActorInReferenceCullView(actor, root, it->firstPerson)) {
				++it;
				continue;
			}

			if (it->armorRecords.empty()) {
				it = suspendedActors_.erase(it);
				continue;
			}

			PendingActorRebuild pending{
				.actorHandle = it->actorHandle,
				.firstPerson = it->firstPerson,
				.armorRecords = std::move(it->armorRecords),
			};
			if (!RebuildPendingArmorRecordsLocked(actor, pending)) {
				it->armorRecords = std::move(pending.armorRecords);
				++it;
				continue;
			}

			spdlog::debug(
				"reactivated suspended system physics candidate actor={} firstPerson={} from preserved armor records",
				static_cast<void*>(actor),
				it->firstPerson);
			it = suspendedActors_.erase(it);
		}
	}

	void Fo4PhysicsWorld::TryReactivateInactiveSystemsLocked()
	{
		if (!enableNpcPhysics_) {
			return;
		}

		const auto* player = RE::PlayerCharacter::GetSingleton();
		for (auto& actorStatePointer : systems_) {
			auto& actorState = *actorStatePointer;
			const auto needsReactivation = actorState.IsInactive();
			const auto needsRebuild = actorState.suspended && !actorState.armorRecords.empty();
			if ((!needsReactivation && !needsRebuild) || actorState.actor == player) {
				continue;
			}

			const auto activeActors = static_cast<std::size_t>(std::ranges::count_if(systems_, [](const auto& a_state) {
				return a_state->IsActive();
			}));
			if (activeActors >= currentMaxActiveActors_) {
				return;
			}

			auto resolvedActor = actorState.actorHandle.get();
			if (!resolvedActor || resolvedActor.get() != actorState.actor) {
				continue;
			}

			auto* actor = resolvedActor.get();
			auto* root = actor->Get3D(actorState.firstPerson);
			if (!root && !actorState.firstPerson) {
				root = actor->Get3D();
			}
			if (!root) {
				continue;
			}
			if (!IsActorWithinDistanceLocked(actor)) {
				continue;
			}
			if (!IsActorInReferenceCullView(actor, root, actorState.firstPerson)) {
				continue;
			}

			if (needsReactivation && ReactivateSystemLocked(actorState)) {
				continue;
			}
			if (needsReactivation && !actorState.armorRecords.empty()) {
				actorState.suspended = true;
			}

			if (!actorState.suspended || actorState.armorRecords.empty()) {
				continue;
			}

			auto armorRecords = actorState.armorRecords;
			PendingActorRebuild pending{
				.actorHandle = actorState.actorHandle,
				.firstPerson = actorState.firstPerson,
				.armorRecords = std::move(armorRecords),
			};
			if (!RebuildPendingArmorRecordsLocked(actor, pending)) {
				if (actorState.HasPhysics()) {
					actorState.suspended = false;
				}
				continue;
			}

			actorState.suspended = false;
			spdlog::debug(
				"reactivated inactive system physics state actor={} firstPerson={} from preserved armor records",
				static_cast<void*>(actor),
				actorState.firstPerson);
		}
	}

	void Fo4PhysicsWorld::MergeArmorPhysicsRecord(std::vector<ArmorPhysicsRecord>& a_records, ArmorPhysicsRecord a_record)
	{
		if (a_record.bipedObject == RE::BIPED_OBJECT::kTotal || a_record.physicsXmlPath.empty()) {
			return;
		}
		auto appendBuildGroups = [](std::vector<std::uint64_t>& a_target, const std::vector<std::uint64_t>& a_source) {
			for (const auto buildGroup : a_source) {
				if (buildGroup != 0 && std::ranges::find(a_target, buildGroup) == a_target.end()) {
					a_target.push_back(buildGroup);
				}
			}
		};
		auto runtimeObjectsCompatible = [](const ArmorPhysicsRecord& a_lhs, const ArmorPhysicsRecord& a_rhs) {
			return (!a_lhs.attachedObject || !a_rhs.attachedObject || a_lhs.attachedObject.get() == a_rhs.attachedObject.get()) &&
				(!a_lhs.sourceObject || !a_rhs.sourceObject || a_lhs.sourceObject.get() == a_rhs.sourceObject.get());
		};
		const auto normalizedXml = ResourceFile::ComparisonKey(a_record.physicsXmlPath);
		auto existing = std::ranges::find_if(a_records, [&a_record, &normalizedXml](const ArmorPhysicsRecord& a_existing) {
			return a_existing.bipedObject == a_record.bipedObject &&
				a_existing.attachedObject.get() == a_record.attachedObject.get() &&
				a_existing.sourceObject.get() == a_record.sourceObject.get() &&
				ResourceFile::ComparisonKey(a_existing.physicsXmlPath) == normalizedXml;
		});
		if (existing == a_records.end()) {
			existing = std::ranges::find_if(a_records, [&a_record, &normalizedXml, &runtimeObjectsCompatible](const ArmorPhysicsRecord& a_existing) {
				return a_existing.bipedObject == a_record.bipedObject &&
					ResourceFile::ComparisonKey(a_existing.physicsXmlPath) == normalizedXml &&
					runtimeObjectsCompatible(a_record, a_existing);
			});
		}
		if (existing != a_records.end()) {
			if (!a_record.attachedObject && existing->attachedObject) {
				a_record.attachedObject = existing->attachedObject;
			}
			if (!a_record.sourceObject && existing->sourceObject) {
				a_record.sourceObject = existing->sourceObject;
			}
			if (!existing->armorBoneReferences.empty()) {
				a_record.armorBoneReferences = existing->armorBoneReferences;
			}
			appendBuildGroups(a_record.buildGroups, existing->buildGroups);
			const auto moveMergedHairSlotOwnerToBack = IsHairBipedObject(a_record.bipedObject);
			*existing = std::move(a_record);
			if (moveMergedHairSlotOwnerToBack && std::next(existing) != a_records.end()) {
				auto mergedRecord = std::move(*existing);
				a_records.erase(existing);
				a_records.push_back(std::move(mergedRecord));
			}
		} else {
			a_records.push_back(std::move(a_record));
		}
	}

	void Fo4PhysicsWorld::StripQueuedArmorRuntimePointers(std::vector<ArmorPhysicsRecord>& a_records)
	{
		for (auto& record : a_records) {
			record.attachedObject = nullptr;
			record.sourceObject = nullptr;
			record.buildGroups.clear();
		}
	}

	std::uint32_t Fo4PhysicsWorld::PruneStalePendingHairSlotArmorRecords(std::vector<ArmorPhysicsRecord>& a_records)
	{
		std::uint32_t removed = 0;
		for (auto it = a_records.begin(); it != a_records.end();) {
			if (!IsHairBipedObject(it->bipedObject)) {
				++it;
				continue;
			}

			const auto bipedObject = it->bipedObject;
			const auto hasLaterSlotOwner = std::ranges::any_of(std::next(it), a_records.end(), [bipedObject](const ArmorPhysicsRecord& a_record) {
				return a_record.bipedObject == bipedObject;
			});
			if (!hasLaterSlotOwner) {
				++it;
				continue;
			}

			it = a_records.erase(it);
			++removed;
		}
		return removed;
	}

	Fo4PhysicsWorld::PendingActorRebuild* Fo4PhysicsWorld::FindPendingActorRebuildLocked(RE::Actor* a_actor, const bool a_firstPerson)
	{
		if (!a_actor) {
			return nullptr;
		}

		const auto found = std::ranges::find_if(pendingActorRebuilds_, [a_actor, a_firstPerson](const PendingActorRebuild& a_pending) {
			const auto resolvedActor = a_pending.actorHandle.get();
			return resolvedActor && resolvedActor.get() == a_actor && a_pending.firstPerson == a_firstPerson;
		});
		return found != pendingActorRebuilds_.end() ? std::addressof(*found) : nullptr;
	}

	std::vector<ArmorPhysicsRecord> Fo4PhysicsWorld::CollectQueuedArmorRecordsForAttachLocked(const LifecycleEvent& a_event)
	{
		std::vector<ArmorPhysicsRecord> records;
		if (auto* pending = FindPendingActorRebuildLocked(a_event.actor, a_event.firstPerson)) {
			for (auto& record : pending->armorRecords) {
				MergeArmorPhysicsRecord(records, record);
			}
		} else if (auto* actorState = FindSystemLocked(a_event.actor, a_event.firstPerson)) {
			for (auto& record : actorState->armorRecords) {
				MergeArmorPhysicsRecord(records, record);
			}
		}

		for (auto& record : CollectSuspendedArmorRecordsLocked(a_event)) {
			MergeArmorPhysicsRecord(records, std::move(record));
		}
		return records;
	}

	std::vector<ArmorPhysicsRecord> Fo4PhysicsWorld::CollectQueuedArmorRecordsForDetachLocked(const LifecycleEvent& a_event)
	{
		std::vector<ArmorPhysicsRecord> records;
		if (auto* pending = FindPendingActorRebuildLocked(a_event.actor, a_event.firstPerson)) {
			records = pending->armorRecords;
		} else if (auto* actorState = FindSystemLocked(a_event.actor, a_event.firstPerson)) {
			records = actorState->armorRecords;
		}
		const auto detachedBipedObject = ResolveEventBipedObject(a_event);
		if (detachedBipedObject != RE::BIPED_OBJECT::kTotal) {
			std::erase_if(records, [detachedBipedObject, object = a_event.object](const ArmorPhysicsRecord& a_record) {
				if (a_record.bipedObject != detachedBipedObject) {
					return false;
				}
				const auto recordHasPhysicsObject = a_record.attachedObject || a_record.sourceObject;
				if (!object || !recordHasPhysicsObject) {
					return true;
				}
				const auto matchesAttached =
					a_record.attachedObject &&
					(a_record.attachedObject.get() == object ||
						IsObjectInTree(a_record.attachedObject.get(), object) ||
						IsObjectInTree(object, a_record.attachedObject.get()));
				const auto matchesSource =
					a_record.sourceObject &&
					(a_record.sourceObject.get() == object ||
						IsObjectInTree(a_record.sourceObject.get(), object) ||
						IsObjectInTree(object, a_record.sourceObject.get()));
				return matchesAttached || matchesSource;
			});
		}
		return records;
	}

	void Fo4PhysicsWorld::MarkPendingActorRebuildLocked(
		RE::Actor* a_actor,
		const bool a_firstPerson,
		std::vector<ArmorPhysicsRecord> a_armorRecords,
		const bool a_forceArmorRescan,
		const bool a_scheduleImmediately,
		const bool a_replaceArmorRecords)
	{
		if (!a_actor) {
			return;
		}

		auto handle = RE::BSPointerHandleManagerInterface<RE::Actor>::GetHandle(a_actor);
		if (!handle) {
			return;
		}

		const auto requestedDelay = std::ranges::any_of(a_armorRecords, [](const ArmorPhysicsRecord& a_record) {
			return a_record.cpuCopyRetryCount > 0;
		}) ? kCpuCopyPendingRetryDelayTasks : 0U;
		const auto rebuildDelay = a_forceArmorRescan ? std::max(requestedDelay, kArmorChangeRebuildDelayTasks) : requestedDelay;
		const auto frameDelay = std::max(rebuildDelay, a_scheduleImmediately ? 0U : 1U);
		StripQueuedArmorRuntimePointers(a_armorRecords);
		if (auto* pending = FindPendingActorRebuildLocked(a_actor, a_firstPerson)) {
			if (a_replaceArmorRecords) {
				pending->armorRecords = std::move(a_armorRecords);
			} else {
				for (auto& record : a_armorRecords) {
					MergeArmorPhysicsRecord(pending->armorRecords, std::move(record));
				}
			}
			const auto prunedHairSlotRecords = PruneStalePendingHairSlotArmorRecords(pending->armorRecords);
			pending->frameDelay = std::max(pending->frameDelay, frameDelay);
			pending->forceArmorRescan = pending->forceArmorRescan || a_forceArmorRescan;
			spdlog::debug(
				"updated pending system physics rebuild for actor={} firstPerson={} armorRecords={} forceArmorRescan={} scheduleImmediately={} replaceArmorRecords={}",
				static_cast<void*>(a_actor),
				a_firstPerson,
				pending->armorRecords.size(),
				pending->forceArmorRescan,
				a_scheduleImmediately,
				a_replaceArmorRecords);
			if (prunedHairSlotRecords > 0) {
				spdlog::debug(
					"pruned stale pending hair-slot armor records actor={} firstPerson={} records={} remaining={}",
					static_cast<void*>(a_actor),
					a_firstPerson,
					prunedHairSlotRecords,
					pending->armorRecords.size());
			}
			return;
		}

		const auto prunedHairSlotRecords = PruneStalePendingHairSlotArmorRecords(a_armorRecords);
		pendingActorRebuilds_.push_back({
			.actorHandle = handle,
			.firstPerson = a_firstPerson,
			.armorRecords = std::move(a_armorRecords),
			.frameDelay = frameDelay,
			.forceArmorRescan = a_forceArmorRescan,
		});
		spdlog::debug(
			"queued pending system physics rebuild for actor={} firstPerson={} armorRecords={} forceArmorRescan={} scheduleImmediately={} replaceArmorRecords={}",
			static_cast<void*>(a_actor),
			a_firstPerson,
			pendingActorRebuilds_.back().armorRecords.size(),
			a_forceArmorRescan,
			a_scheduleImmediately,
			a_replaceArmorRecords);
		if (prunedHairSlotRecords > 0) {
			spdlog::debug(
				"pruned stale pending hair-slot armor records actor={} firstPerson={} records={} remaining={}",
				static_cast<void*>(a_actor),
				a_firstPerson,
				prunedHairSlotRecords,
				pendingActorRebuilds_.back().armorRecords.size());
		}
	}

	bool Fo4PhysicsWorld::SoftReloadSystemLocked(Fo4SkinnedMeshSystem& a_state, const LifecycleEventType a_reason)
	{
		auto* actor = a_state.actor;
		if (!actor) {
			if (auto resolved = a_state.actorHandle.get()) {
				actor = resolved.get();
			}
		}
		if (!actor) {
			return false;
		}

		auto armorRecords = a_state.armorRecords;
		const auto hadRuntime = a_state.HasPhysics();
		if (hadRuntime) {
			SuspendSystemLocked(a_state);
		}

		if (armorRecords.empty() && !hadRuntime) {
			return false;
		}

		const auto forceArmorRescan = armorRecords.empty();
		MarkPendingActorRebuildLocked(actor, a_state.firstPerson, std::move(armorRecords), forceArmorRescan, true, true);
		ResetStepClockLocked();
		spdlog::debug(
			"soft-reloaded system physics state after {} actor={} firstPerson={} hadRuntime={} forceArmorRescan={}",
			ToString(a_reason),
			static_cast<void*>(actor),
			a_state.firstPerson,
			hadRuntime,
			forceArmorRescan);
		return true;
	}

	void Fo4PhysicsWorld::MarkPendingHeadRebuildLocked(const LifecycleEvent& a_event)
	{
		if (!a_event.actor) {
			return;
		}

		auto handle = RE::BSPointerHandleManagerInterface<RE::Actor>::GetHandle(a_event.actor);
		if (!handle) {
			return;
		}

		for (auto& pending : pendingHeadRebuilds_) {
			auto resolvedActor = pending.actorHandle.get();
			if (resolvedActor && resolvedActor.get() == a_event.actor) {
				pending.type = LifecycleEventType::kActorHeadInitialized;
				pending.object = nullptr;
				pending.frameDelay = std::max(pending.frameDelay, kHeadInitializedRebuildDelayFrames);
				pending.cpuCopyRetryCount = 0;
				return;
			}
		}

		pendingHeadRebuilds_.push_back({
			.actorHandle = handle,
			.type = LifecycleEventType::kActorHeadInitialized,
			.frameDelay = kHeadInitializedRebuildDelayFrames,
		});
	}

	bool Fo4PhysicsWorld::HasActiveOrPendingActorRebuildLocked(RE::Actor* a_actor)
	{
		if (!a_actor) {
			return false;
		}

		if (std::ranges::any_of(systems_, [a_actor](const auto& a_state) {
			return a_state->actor == a_actor;
		})) {
			return true;
		}

		return std::ranges::any_of(pendingActorRebuilds_, [a_actor](const PendingActorRebuild& a_pending) {
			const auto resolvedActor = a_pending.actorHandle.get();
			return resolvedActor && resolvedActor.get() == a_actor;
		});
	}

	std::vector<ArmorPhysicsRecord> Fo4PhysicsWorld::CollectSuspendedArmorRecordsLocked(const LifecycleEvent& a_event)
	{
		std::vector<ArmorPhysicsRecord> records;
		if (!IsArmorAttachCandidate(a_event.type) || a_event.physicsXmlPath.empty() || !a_event.object) {
			return records;
		}

		const auto bipedObject = ResolveEventBipedObject(a_event);
		if (bipedObject == RE::BIPED_OBJECT::kTotal) {
			return records;
		}

		records.push_back({
			.bipedObject = bipedObject,
			.physicsXmlPath = a_event.physicsXmlPath,
			.attachedObject = a_event.object,
			.sourceObject = a_event.sourceObject,
			.armorBoneReferences = a_event.armorBoneReferences,
		});
		return records;
	}

	void Fo4PhysicsWorld::RecordArmorLocked(
		Fo4SkinnedMeshSystem& a_state,
		const RE::BIPED_OBJECT a_bipedObject,
		std::string a_physicsXmlPath,
		const DefaultBBP::NameMap& a_meshNameMap,
		RE::NiAVObject* a_attachedObject,
		RE::NiAVObject* a_sourceObject,
		std::vector<ArmorBoneReference> a_armorBoneReferences,
		const std::uint64_t a_buildGroup)
	{
		if (a_bipedObject == RE::BIPED_OBJECT::kTotal || a_physicsXmlPath.empty()) {
			return;
		}

		const auto normalizedXml = ResourceFile::ComparisonKey(a_physicsXmlPath);
		const auto appendBuildGroup = [](std::vector<std::uint64_t>& a_buildGroups, const std::uint64_t a_buildGroup) {
			if (a_buildGroup != 0 && std::ranges::find(a_buildGroups, a_buildGroup) == a_buildGroups.end()) {
				a_buildGroups.push_back(a_buildGroup);
			}
		};
		auto existing = std::ranges::find_if(a_state.armorRecords, [&](const ArmorPhysicsRecord& a_record) {
			return a_record.bipedObject == a_bipedObject &&
				a_record.attachedObject.get() == a_attachedObject &&
				a_record.sourceObject.get() == a_sourceObject &&
				ResourceFile::ComparisonKey(a_record.physicsXmlPath) == normalizedXml;
		});
		if (existing != a_state.armorRecords.end()) {
			existing->physicsXmlPath = std::move(a_physicsXmlPath);
			existing->meshNameMap = a_meshNameMap;
			existing->attachedObject = a_attachedObject;
			existing->sourceObject = a_sourceObject;
			if (existing->armorBoneReferences.empty()) {
				existing->armorBoneReferences = std::move(a_armorBoneReferences);
			}
			appendBuildGroup(existing->buildGroups, a_buildGroup);
		} else {
			std::vector<std::uint64_t> buildGroups;
			appendBuildGroup(buildGroups, a_buildGroup);
			a_state.armorRecords.push_back({
				.bipedObject = a_bipedObject,
				.physicsXmlPath = std::move(a_physicsXmlPath),
				.meshNameMap = a_meshNameMap,
				.attachedObject = a_attachedObject,
				.sourceObject = a_sourceObject,
				.armorBoneReferences = std::move(a_armorBoneReferences),
				.buildGroups = std::move(buildGroups),
			});
		}
	}

	AttachmentPhysicsRecord* Fo4PhysicsWorld::FindAttachmentLocked(
		Fo4SkinnedMeshSystem& a_state,
		const RE::BIPED_OBJECT a_bipedObject,
		RE::NiAVObject* a_object,
		RE::NiAVObject* a_sourceObject,
		const std::string_view a_physicsXmlPath)
	{
		if (a_bipedObject == RE::BIPED_OBJECT::kTotal) {
			return nullptr;
		}

		const auto normalizedXml =
			a_physicsXmlPath.empty() ? std::string{} : ResourceFile::ComparisonKey(a_physicsXmlPath);
		const auto found = std::ranges::find_if(a_state.attachmentRecords, [&](const AttachmentPhysicsRecord& a_record) {
			if (a_record.bipedObject != a_bipedObject) {
				return false;
			}
			if (a_object && a_record.attachedObject.get() != a_object) {
				return false;
			}
			if (a_sourceObject && a_record.sourceObject && a_record.sourceObject.get() != a_sourceObject) {
				return false;
			}
			if (!normalizedXml.empty() && ResourceFile::ComparisonKey(a_record.physicsXmlPath) != normalizedXml) {
				return false;
			}
			return true;
		});
		return found != a_state.attachmentRecords.end() ? std::addressof(*found) : nullptr;
	}

	const AttachmentPhysicsRecord* Fo4PhysicsWorld::FindAttachmentLocked(
		const Fo4SkinnedMeshSystem& a_state,
		const RE::BIPED_OBJECT a_bipedObject,
		RE::NiAVObject* a_object,
		RE::NiAVObject* a_sourceObject,
		const std::string_view a_physicsXmlPath)
	{
		if (a_bipedObject == RE::BIPED_OBJECT::kTotal) {
			return nullptr;
		}

		const auto normalizedXml =
			a_physicsXmlPath.empty() ? std::string{} : ResourceFile::ComparisonKey(a_physicsXmlPath);
		const auto found = std::ranges::find_if(a_state.attachmentRecords, [&](const AttachmentPhysicsRecord& a_record) {
			if (a_record.bipedObject != a_bipedObject) {
				return false;
			}
			if (a_object && a_record.attachedObject.get() != a_object) {
				return false;
			}
			if (a_sourceObject && a_record.sourceObject && a_record.sourceObject.get() != a_sourceObject) {
				return false;
			}
			if (!normalizedXml.empty() && ResourceFile::ComparisonKey(a_record.physicsXmlPath) != normalizedXml) {
				return false;
			}
			return true;
		});
		return found != a_state.attachmentRecords.end() ? std::addressof(*found) : nullptr;
	}

	bool Fo4PhysicsWorld::IsAttachmentCurrentLocked(
		const Fo4SkinnedMeshSystem& a_state,
		const RE::BIPED_OBJECT a_bipedObject,
		RE::NiAVObject* a_object,
		RE::NiAVObject* a_sourceObject,
		const std::string_view a_physicsXmlPath)
	{
		const auto* record = FindAttachmentLocked(a_state, a_bipedObject, a_object, a_sourceObject, a_physicsXmlPath);
		if (!record || record->buildGroups.empty() || record->physicsXmlPath.empty()) {
			return false;
		}

		const auto sameXml = ResourceFile::PathsEqual(record->physicsXmlPath, a_physicsXmlPath);
		if (!sameXml) {
			return false;
		}
		if (a_object && record->attachedObject.get() != a_object) {
			return false;
		}
		if (a_sourceObject && record->sourceObject && record->sourceObject.get() != a_sourceObject) {
			return false;
		}

		return std::ranges::any_of(record->buildGroups, [&](const std::uint64_t a_buildGroup) {
			if (std::ranges::any_of(a_state.meshes, [a_buildGroup](const MeshRecord& a_mesh) {
					return a_mesh.buildGroup == a_buildGroup;
				})) {
				return true;
			}
			return std::ranges::any_of(a_state.bodies, [a_buildGroup](const BoneRecord& a_body) {
				return BodyHasBuildGroup(a_body, a_buildGroup);
			});
		});
	}

	void Fo4PhysicsWorld::RecordAttachmentLocked(
		Fo4SkinnedMeshSystem& a_state,
		const RE::BIPED_OBJECT a_bipedObject,
		RE::NiAVObject* a_object,
		RE::NiAVObject* a_sourceObject,
		std::string a_physicsXmlPath,
		const std::uint64_t a_buildGroup)
	{
		if (a_bipedObject == RE::BIPED_OBJECT::kTotal || a_physicsXmlPath.empty() || a_buildGroup == 0) {
			return;
		}

		auto* record = FindAttachmentLocked(a_state, a_bipedObject, a_object, a_sourceObject, a_physicsXmlPath);
		if (!record) {
			a_state.attachmentRecords.push_back({
				.bipedObject = a_bipedObject,
				.generation = ++a_state.nextAttachmentGeneration,
			});
			record = std::addressof(a_state.attachmentRecords.back());
		}

		record->physicsXmlPath = std::move(a_physicsXmlPath);
		record->attachedObject = a_object;
		record->sourceObject = a_sourceObject;
		if (std::ranges::find(record->buildGroups, a_buildGroup) == record->buildGroups.end()) {
			record->buildGroups.push_back(a_buildGroup);
		}
	}

	bool Fo4PhysicsWorld::RebuildPendingArmorRecordsLocked(RE::Actor* a_actor, PendingActorRebuild& a_pending)
	{
		if (!a_actor) {
			a_pending.armorRecords.clear();
			return true;
		}

		auto* biped = a_actor->GetBiped(a_pending.firstPerson).get();
		if (!biped && !a_pending.firstPerson) {
			biped = a_actor->GetBiped().get();
		}
		if (!biped) {
			return false;
		}

		if (a_pending.armorRecords.size() > 1) {
			auto recordScore = [](const ArmorPhysicsRecord& a_record) {
				return a_record.armorBoneReferences.size();
			};
			auto fillMissingRecordState = [](ArmorPhysicsRecord& a_target, const ArmorPhysicsRecord& a_source) {
				if (!a_target.attachedObject && a_source.attachedObject) {
					a_target.attachedObject = a_source.attachedObject;
				}
				if (!a_target.sourceObject && a_source.sourceObject) {
					a_target.sourceObject = a_source.sourceObject;
				}
				if (a_target.meshNameMap.empty() && !a_source.meshNameMap.empty()) {
					a_target.meshNameMap = a_source.meshNameMap;
				}
				if (a_target.armorBoneReferences.empty() && !a_source.armorBoneReferences.empty()) {
					a_target.armorBoneReferences = a_source.armorBoneReferences;
				}
				for (const auto buildGroup : a_source.buildGroups) {
					if (buildGroup != 0 && std::ranges::find(a_target.buildGroups, buildGroup) == a_target.buildGroups.end()) {
						a_target.buildGroups.push_back(buildGroup);
					}
				}
			};
			auto runtimeObjectsCompatible = [](const ArmorPhysicsRecord& a_lhs, const ArmorPhysicsRecord& a_rhs) {
				return (!a_lhs.attachedObject || !a_rhs.attachedObject || a_lhs.attachedObject.get() == a_rhs.attachedObject.get()) &&
					(!a_lhs.sourceObject || !a_rhs.sourceObject || a_lhs.sourceObject.get() == a_rhs.sourceObject.get());
			};

			std::vector<ArmorPhysicsRecord> mergedRecords;
			for (auto& record : a_pending.armorRecords) {
				if (record.bipedObject == RE::BIPED_OBJECT::kTotal || record.physicsXmlPath.empty()) {
					continue;
				}

				const auto normalizedXml = ResourceFile::ComparisonKey(record.physicsXmlPath);
				auto existing = std::ranges::find_if(mergedRecords, [&](const ArmorPhysicsRecord& a_existing) {
					return a_existing.bipedObject == record.bipedObject &&
						ResourceFile::ComparisonKey(a_existing.physicsXmlPath) == normalizedXml &&
						runtimeObjectsCompatible(record, a_existing);
				});
				if (existing == mergedRecords.end()) {
					mergedRecords.push_back(std::move(record));
					continue;
				}

				if (recordScore(record) > recordScore(*existing)) {
					fillMissingRecordState(record, *existing);
					*existing = std::move(record);
				} else {
					fillMissingRecordState(*existing, record);
				}
			}
			if (mergedRecords.size() != a_pending.armorRecords.size()) {
				spdlog::debug(
					"collapsed duplicate pending armor records actor={} firstPerson={} before rebuild oldCount={} newCount={}",
					static_cast<void*>(a_actor),
					a_pending.firstPerson,
					a_pending.armorRecords.size(),
					mergedRecords.size());
			}
			a_pending.armorRecords = std::move(mergedRecords);
		}
		if (const auto prunedHairSlotRecords = PruneStalePendingHairSlotArmorRecords(a_pending.armorRecords);
			prunedHairSlotRecords > 0) {
			spdlog::debug(
				"pruned stale pending hair-slot armor records before rebuild actor={} firstPerson={} records={} remaining={}",
				static_cast<void*>(a_actor),
				a_pending.firstPerson,
				prunedHairSlotRecords,
				a_pending.armorRecords.size());
		}

		auto* loader = PhysicsXmlLoader::GetSingleton();
		for (auto it = a_pending.armorRecords.begin(); it != a_pending.armorRecords.end();) {
			auto& record = *it;
			if (record.bipedObject == RE::BIPED_OBJECT::kTotal || record.physicsXmlPath.empty()) {
				it = a_pending.armorRecords.erase(it);
				continue;
			}

			auto* bipObject = biped->GetBipObject(record.bipedObject);
			auto* partClone = bipObject ? bipObject->partClone.get() : nullptr;
			if (!partClone) {
				++it;
				continue;
			}
			const auto hadQueuedRuntimeObject = record.attachedObject || record.sourceObject;
			if (hadQueuedRuntimeObject &&
				(record.attachedObject.get() != partClone ||
					(record.sourceObject && record.sourceObject.get() != partClone))) {
				if (auto* logger = spdlog::default_logger_raw(); logger && logger->should_log(spdlog::level::debug)) {
					spdlog::debug(
						"rebasing pending armor rebuild onto current biped partClone actor={} bipedObject={} xml='{}' queuedObject={} queuedSource={} currentPartClone={}",
						static_cast<void*>(a_actor),
						std::to_underlying(record.bipedObject),
						record.physicsXmlPath,
						static_cast<void*>(record.attachedObject.get()),
						static_cast<void*>(record.sourceObject.get()),
						static_cast<void*>(partClone));
				}
			}
			auto* rebuildObject = partClone;
			auto* rebuildSourceObject = partClone;
			auto* rebuildSourceRoot = partClone->IsNode();

			const auto selectedSummary = loader->LoadSummary(record.physicsXmlPath);
			if (!selectedSummary) {
				spdlog::warn(
					"dropping pending customization resume armor slot because XML failed to load actor={} bipedObject={} xml='{}'",
					static_cast<void*>(a_actor),
					std::to_underlying(record.bipedObject),
					record.physicsXmlPath);
				it = a_pending.armorRecords.erase(it);
				continue;
			}

			const LifecycleEvent resumeEvent{
				.type = LifecycleEventType::kArmorApplySkinnedObjects,
				.actor = a_actor,
				.biped = biped,
				.bipObject = bipObject,
				.bipedObject = record.bipedObject,
				.object = rebuildObject,
				.sourceObject = rebuildSourceObject,
				.armorBoneReferences = record.armorBoneReferences,
				.sourceRoot = rebuildSourceRoot,
				.physicsXmlPath = record.physicsXmlPath,
				.firstPerson = a_pending.firstPerson,
			};

			auto& actorState = GetOrCreateSystemLocked(a_actor, a_pending.firstPerson);
			std::vector<std::uint64_t> staleArmorBuildGroups;
			std::vector<AttachmentBoneLocalPose> preservedPoseCache;
			const auto cacheCurrentPoseForGroups = [&](const std::span<const std::uint64_t> a_buildGroups) {
				if (!record.preserveCurrentPose || a_buildGroups.empty()) {
					return;
				}
				for (const auto& localPose : actorState.attachmentBoneLocalPoses) {
					if (!localPose.node ||
						std::ranges::find(a_buildGroups, localPose.buildGroup) == a_buildGroups.end() ||
						std::ranges::any_of(preservedPoseCache, [&](const AttachmentBoneLocalPose& a_cached) {
							return a_cached.node.get() == localPose.node.get();
						})) {
						continue;
					}
					auto cachedPose = localPose;
					cachedPose.buildGroup = kPapyrusArmorPoseCacheBuildGroup;
					preservedPoseCache.push_back(std::move(cachedPose));
				}
			};
			const auto restoreCachedPoses = [&]() {
				for (const auto& cachedPose : preservedPoseCache) {
					const auto alreadyCached = std::ranges::any_of(actorState.attachmentBoneLocalPoses, [&](const AttachmentBoneLocalPose& a_existing) {
						return a_existing.buildGroup == kPapyrusArmorPoseCacheBuildGroup && a_existing.node.get() == cachedPose.node.get();
					});
					if (!alreadyCached) {
						actorState.attachmentBoneLocalPoses.push_back(cachedPose);
					}
				}
			};
			const auto hairSlotArmorBuild = IsHairBipedObject(record.bipedObject);
			if (const auto* attachment = FindAttachmentLocked(actorState, record.bipedObject, rebuildObject, rebuildSourceObject, record.physicsXmlPath)) {
				for (const auto buildGroup : attachment->buildGroups) {
					if (buildGroup != 0 && std::ranges::find(staleArmorBuildGroups, buildGroup) == staleArmorBuildGroups.end()) {
						staleArmorBuildGroups.push_back(buildGroup);
					}
				}
			}
			for (const auto buildGroup : CollectBuildGroupsForObjectLocked(actorState, rebuildObject)) {
				if (buildGroup != 0 && std::ranges::find(staleArmorBuildGroups, buildGroup) == staleArmorBuildGroups.end()) {
					staleArmorBuildGroups.push_back(buildGroup);
				}
			}
			if (hairSlotArmorBuild) {
				std::vector<std::uint64_t> staleHeadBuildGroups;
				const auto replacedHeadParts = CollectHeadPartGroupsLocked(actorState, BuildDomain::kHair, staleHeadBuildGroups);
				if (replacedHeadParts > 0) {
					spdlog::debug(
						"pending hair-slot armor rebuild is replacing tracked hair headpart groups actor={} bipedObject={} object={} xml='{}' hairRecords={} groups={}",
						static_cast<void*>(a_actor),
						std::to_underlying(record.bipedObject),
						static_cast<void*>(rebuildObject),
						record.physicsXmlPath,
						replacedHeadParts,
						staleHeadBuildGroups.size());
				}
				if (!staleHeadBuildGroups.empty()) {
					ClearBuildGroupsLocked(actorState, staleHeadBuildGroups);
				}
			}
			if (!hairSlotArmorBuild && !staleArmorBuildGroups.empty()) {
				cacheCurrentPoseForGroups(staleArmorBuildGroups);
				ClearBuildGroupsLocked(actorState, staleArmorBuildGroups, !record.preserveCurrentPose);
				restoreCachedPoses();
				const auto clearedCount = staleArmorBuildGroups.size();
				staleArmorBuildGroups.clear();
				spdlog::debug(
					"cleared stale system groups before pending armor rebuild actor={} bipedObject={} object={} xml='{}' groups={}",
					static_cast<void*>(a_actor),
					std::to_underlying(record.bipedObject),
					static_cast<void*>(rebuildObject),
					record.physicsXmlPath,
					clearedCount);
			}
			if (hairSlotArmorBuild) {
				const auto staleHairBuildGroups = CollectArmorPhysicsGroupsForBipedObjectLocked(actorState, record.bipedObject);
				cacheCurrentPoseForGroups(staleHairBuildGroups);
				ClearStaleHairSlotArmorGroupsLocked(
					actorState,
					record.bipedObject,
					0,
					"pending-armor-rebuild-prebuild",
					rebuildObject,
					record.physicsXmlPath,
					!record.preserveCurrentPose);
				restoreCachedPoses();
				staleArmorBuildGroups.clear();
			}
			spdlog::debug(
				"rebuilding pending armor system physics actor={} bipedObject={} object={} xml='{}' stagedStaleGroups={}",
				static_cast<void*>(a_actor),
				std::to_underlying(record.bipedObject),
				static_cast<void*>(rebuildObject),
				record.physicsXmlPath,
				staleArmorBuildGroups.size());
			auto buildResult = BuildSystemObjectsLocked(actorState, resumeEvent, *selectedSummary, record.meshNameMap, BuildDomain::kArmor, !hairSlotArmorBuild);
			if (buildResult.succeeded) {
				if (hairSlotArmorBuild) {
					ActivateBuildGroupLocked(actorState, buildResult.buildGroup);
					buildResult.committed = true;
					LogSystemBulletObjectsLocked(actorState, "after-system-build-commit");
				} else if (!staleArmorBuildGroups.empty()) {
					ClearBuildGroupsLocked(actorState, staleArmorBuildGroups);
				}
				RecordAttachmentLocked(actorState, record.bipedObject, rebuildObject, rebuildSourceObject, record.physicsXmlPath, buildResult.buildGroup);
				RecordArmorLocked(
					actorState,
					record.bipedObject,
					record.physicsXmlPath,
					record.meshNameMap,
					rebuildObject,
					rebuildSourceObject,
					record.armorBoneReferences,
					buildResult.buildGroup);
				if (hairSlotArmorBuild) {
					const auto remainingHairSlotArmorGroups = CollectArmorPhysicsGroupsForBipedObjectLocked(actorState, record.bipedObject).size();
					spdlog::debug(
						"hair-slot armor ownership after pending commit actor={} bipedObject={} buildGroup={} object={} xml='{}' remainingArmorGroups={}",
						static_cast<void*>(a_actor),
						std::to_underlying(record.bipedObject),
						buildResult.buildGroup,
						static_cast<void*>(rebuildObject),
						record.physicsXmlPath,
						remainingHairSlotArmorGroups);
				}
				DeactivateBuiltSystemIfInactiveLocked(actorState, resumeEvent);
			} else if (buildResult.buildGroup != 0 && BuildGroupIsRecordableLocked(actorState, buildResult.buildGroup, BuildDomain::kArmor)) {
				ClearBuildGroupsLocked(actorState, std::vector<std::uint64_t>{ buildResult.buildGroup });
				spdlog::debug(
					"rolled back incomplete pending armor system build group actor={} bipedObject={} object={} buildGroup={} xml='{}' pendingCpuCopy={}",
					static_cast<void*>(a_actor),
					std::to_underlying(record.bipedObject),
					static_cast<void*>(rebuildObject),
					buildResult.buildGroup,
					record.physicsXmlPath,
					buildResult.cpuCopyPending);
			}
			if (buildResult.cpuCopyPending) {
				if (record.cpuCopyRetryCount < kCpuCopyPendingMaxRetries) {
					++record.cpuCopyRetryCount;
					a_pending.frameDelay = std::max(a_pending.frameDelay, kCpuCopyPendingRetryDelayTasks);
					spdlog::debug(
						"retrying pending system mesh CPU copy actor={} bipedObject={} object={} attempt={}/{} delayTasks={}",
						static_cast<void*>(a_actor),
						std::to_underlying(record.bipedObject),
						static_cast<void*>(rebuildObject),
						record.cpuCopyRetryCount,
						kCpuCopyPendingMaxRetries,
						a_pending.frameDelay);
					++it;
					continue;
				}
				spdlog::warn(
					"giving up pending system mesh CPU copy actor={} bipedObject={} object={} attempts={} xml='{}'",
					static_cast<void*>(a_actor),
					std::to_underlying(record.bipedObject),
					static_cast<void*>(rebuildObject),
					record.cpuCopyRetryCount,
					record.physicsXmlPath);
			}
			if (record.preserveCurrentPose) {
				std::erase_if(actorState.attachmentBoneLocalPoses, [](const AttachmentBoneLocalPose& a_pose) {
					return a_pose.buildGroup == kPapyrusArmorPoseCacheBuildGroup;
				});
			}
			it = a_pending.armorRecords.erase(it);
		}

		return a_pending.armorRecords.empty();
	}

	void Fo4PhysicsWorld::TryRebuildPendingActorsLocked(RE::Actor* a_actor)
	{
		if (pendingActorRebuilds_.empty()) {
			return;
		}
		if (!InitializeLocked()) {
			return;
		}

		for (auto it = pendingActorRebuilds_.begin(); it != pendingActorRebuilds_.end();) {
			auto resolvedActor = it->actorHandle.get();
			if (!resolvedActor) {
				it = pendingActorRebuilds_.erase(it);
				continue;
			}

			auto* actor = resolvedActor.get();
			if (!actor) {
				it = pendingActorRebuilds_.erase(it);
				continue;
			}
			if (a_actor && actor != a_actor) {
				++it;
				continue;
			}
			if (characterCustomizationMenuDepth_ > 0 && IsCharacterCustomizationTargetLocked(actor)) {
				++it;
				continue;
			}

			if (it->frameDelay > 0) {
				--it->frameDelay;
				++it;
				continue;
			}

			bool fullActorRebuild = false;
			if (it->forceArmorRescan) {
				auto* root = actor->Get3D(it->firstPerson);
				if (!root && !it->firstPerson) {
					root = actor->Get3D();
				}
				auto* biped = actor->GetBiped(it->firstPerson).get();
				if (!biped && !it->firstPerson) {
					biped = actor->GetBiped().get();
				}
				if (!root || !biped) {
					++it;
					continue;
				}

				if (auto* existingState = FindSystemLocked(actor, it->firstPerson)) {
					ClearSystemLocked(*existingState);
					std::erase_if(systems_, [](const auto& a_state) {
						return !a_state->suspended && !a_state->HasPhysics() && a_state->armorRecords.empty();
					});
				}

				it->forceArmorRescan = false;
				if (it->armorRecords.empty()) {
					const LifecycleEvent rebuildEvent{
						.type = LifecycleEventType::kActorSet3D,
						.actor = actor,
						.biped = biped,
						.object = root,
						.firstPerson = it->firstPerson,
					};
					if (IsBuildCandidateLocked(rebuildEvent, true)) {
						spdlog::debug(
							"processing full actor system physics rebuild for actor={} root={} firstPerson={} with no queued armor records",
							static_cast<void*>(actor),
							static_cast<void*>(root),
							it->firstPerson);
						BuildForEventLocked(rebuildEvent);
					}
					it = pendingActorRebuilds_.erase(it);
					continue;
				}
				spdlog::debug(
					"processing full actor system physics rebuild for actor={} root={} firstPerson={} armorRecords={}",
					static_cast<void*>(actor),
					static_cast<void*>(root),
					it->firstPerson,
					it->armorRecords.size());
				fullActorRebuild = true;
			}

			if (!it->armorRecords.empty()) {
				if (!RebuildPendingArmorRecordsLocked(actor, *it)) {
					++it;
					continue;
				}
				if (fullActorRebuild) {
					LogActorSkeletonHierarchy(actor, it->firstPerson, "after-full-actor-system-rebuild");
					if (const auto* rebuiltState = FindSystemLocked(actor, it->firstPerson)) {
						LogSystemBulletObjectsLocked(*rebuiltState, "after-full-actor-system-rebuild");
					}
				}
				it = pendingActorRebuilds_.erase(it);
				continue;
			}

			if (auto* existingState = FindSystemLocked(actor, it->firstPerson);
				existingState && existingState->IsActive()) {
				it = pendingActorRebuilds_.erase(it);
				continue;
			}

			auto* root = actor->Get3D(it->firstPerson);
			if (!root && !it->firstPerson) {
				root = actor->Get3D();
			}
			if (!root) {
				++it;
				continue;
			}
			auto* biped = actor->GetBiped(it->firstPerson).get();
			if (!biped && !it->firstPerson) {
				biped = actor->GetBiped().get();
			}
			const LifecycleEvent rebuildEvent{
				.type = LifecycleEventType::kActorSet3D,
				.actor = actor,
				.biped = biped,
				.object = root,
				.firstPerson = it->firstPerson,
			};
			if (IsBuildCandidateLocked(rebuildEvent, true)) {
				spdlog::debug(
					"processing pending system physics rebuild for actor={} root={} firstPerson={}",
					static_cast<void*>(actor),
					static_cast<void*>(root),
					it->firstPerson);
				BuildForEventLocked(rebuildEvent);
			}
			const auto* rebuiltState = FindSystemLocked(actor, it->firstPerson);
			const auto rebuiltRuntime = rebuiltState && rebuiltState->IsActive();
			if (rebuiltRuntime) {
				it = pendingActorRebuilds_.erase(it);
				continue;
			}
			if (rebuildEvent.biped) {
				spdlog::debug(
					"dropping pending system physics rebuild for actor={} firstPerson={} because no early SMP armor records are tracked",
					static_cast<void*>(actor),
					it->firstPerson);
				it = pendingActorRebuilds_.erase(it);
				continue;
			}

			++it;
		}
	}

	void Fo4PhysicsWorld::TryRebuildPendingHeadsLocked()
	{
		if (pendingHeadRebuilds_.empty()) {
			return;
		}
		if (!InitializeLocked()) {
			return;
		}

		for (auto it = pendingHeadRebuilds_.begin(); it != pendingHeadRebuilds_.end();) {
			auto resolvedActor = it->actorHandle.get();
			if (!resolvedActor) {
				it = pendingHeadRebuilds_.erase(it);
				continue;
			}

			auto* actor = resolvedActor.get();
			if (!actor) {
				it = pendingHeadRebuilds_.erase(it);
				continue;
			}
			if (characterCustomizationMenuDepth_ > 0 && IsCharacterCustomizationTargetLocked(actor)) {
				++it;
				continue;
			}
			auto* biped = actor->GetBiped(false).get();
			if (!biped) {
				biped = actor->GetBiped().get();
			}
			if (biped) {
				for (auto pendingActorIt = pendingActorRebuilds_.begin(); pendingActorIt != pendingActorRebuilds_.end();) {
					const auto pendingActor = pendingActorIt->actorHandle.get();
					if (!pendingActor || pendingActor.get() != actor || pendingActorIt->firstPerson) {
						++pendingActorIt;
						continue;
					}

					const auto removed = std::erase_if(
						pendingActorIt->armorRecords,
						[biped](const ArmorPhysicsRecord& a_record) {
							if (a_record.bipedObject == RE::BIPED_OBJECT::kTotal) {
								return true;
							}
							const auto* bipObject = biped->GetBipObject(a_record.bipedObject);
							return !bipObject || !bipObject->partClone;
						});
					if (removed > 0) {
						spdlog::debug(
							"discarded pending armor records absent from the live biped before head rebuild actor={} removed={} remaining={}",
							static_cast<void*>(actor),
							removed,
							pendingActorIt->armorRecords.size());
					}
					if (pendingActorIt->armorRecords.empty() && !pendingActorIt->forceArmorRescan) {
						pendingActorIt = pendingActorRebuilds_.erase(pendingActorIt);
						continue;
					}
					++pendingActorIt;
				}
			}
			const auto actorRebuildPending = std::ranges::any_of(pendingActorRebuilds_, [actor](const PendingActorRebuild& a_pending) {
				const auto resolvedPendingActor = a_pending.actorHandle.get();
				return resolvedPendingActor && resolvedPendingActor.get() == actor;
			});
			if (actorRebuildPending) {
				++it;
				continue;
			}

			if (it->frameDelay > 0) {
				--it->frameDelay;
				++it;
				continue;
			}

			auto* root = actor->Get3D(false);
			if (!root) {
				root = actor->Get3D();
			}
			auto* faceNode = actor->GetFaceNodeSkinned();
			if (!root || !faceNode) {
				++it;
				continue;
			}

			const LifecycleEvent headEvent{
				.type = it->type,
				.actor = actor,
				.object = it->object ? it->object.get() : reinterpret_cast<RE::NiAVObject*>(faceNode),
				.firstPerson = false,
			};
			bool cpuCopyPending = false;
			if (IsBuildCandidateLocked(headEvent, false)) {
				spdlog::debug(
					"processing pending head physics rebuild for actor={} root={} faceNode={}",
					static_cast<void*>(actor),
					static_cast<void*>(root),
					static_cast<void*>(faceNode));
				cpuCopyPending = BuildHeadForEventLocked(headEvent);
			}
			if (cpuCopyPending) {
				if (it->cpuCopyRetryCount < kCpuCopyPendingMaxRetries) {
					++it->cpuCopyRetryCount;
					it->frameDelay = std::max(it->frameDelay, kCpuCopyPendingRetryDelayTasks);
					spdlog::debug(
						"retrying pending head mesh CPU copy actor={} object={} attempt={}/{} delayTasks={}",
						static_cast<void*>(actor),
						static_cast<void*>(headEvent.object),
						it->cpuCopyRetryCount,
						kCpuCopyPendingMaxRetries,
						it->frameDelay);
					++it;
					continue;
				}
				spdlog::warn(
					"giving up pending head mesh CPU copy actor={} object={} attempts={}",
					static_cast<void*>(actor),
					static_cast<void*>(headEvent.object),
					it->cpuCopyRetryCount);
			}
			it = pendingHeadRebuilds_.erase(it);
		}
	}
}
