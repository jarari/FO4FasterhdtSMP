// Included by ../Fo4PhysicsWorld.cpp.
// Split from Fo4PhysicsWorld.cpp. Do not compile this file separately.

namespace Smp
{
	Fo4PhysicsWorld::PrototypeActorState* Fo4PhysicsWorld::FindPrototypeStateLocked(RE::Actor* a_actor, const bool a_firstPerson)
	{
		const auto found = std::ranges::find_if(prototypeActors_, [a_actor, a_firstPerson](const PrototypeActorState& a_state) {
			return a_state.actor == a_actor && a_state.firstPerson == a_firstPerson;
		});
		return found == prototypeActors_.end() ? nullptr : std::addressof(*found);
	}

	Fo4PhysicsWorld::PrototypeActorState& Fo4PhysicsWorld::GetOrCreatePrototypeStateLocked(RE::Actor* a_actor, const bool a_firstPerson)
	{
		if (auto* state = FindPrototypeStateLocked(a_actor, a_firstPerson)) {
			return *state;
		}

		auto& state = prototypeActors_.emplace_back();
		state.actor = a_actor;
		state.actorHandle = RE::BSPointerHandleManagerInterface<RE::Actor>::GetHandle(a_actor);
		state.firstPerson = a_firstPerson;
		return state;
	}

	bool Fo4PhysicsWorld::IsPrototypeStateValidLocked(PrototypeActorState& a_state)
	{
		if (!a_state.actor || !a_state.actorHandle) {
			spdlog::debug("dropping prototype physics state with missing actor handle actor={}", static_cast<void*>(a_state.actor));
			return false;
		}

		auto resolvedActor = a_state.actorHandle.get();
		if (!resolvedActor || resolvedActor.get() != a_state.actor) {
			spdlog::debug("dropping prototype physics state with stale actor handle actor={}", static_cast<void*>(a_state.actor));
			return false;
		}

		auto* root = resolvedActor->Get3D(a_state.firstPerson);
		if (!root && !a_state.firstPerson) {
			root = resolvedActor->Get3D();
		}
		if (!root) {
			if (!a_state.armorRecords.empty() || a_state.HasRuntime() || a_state.runtimeSuspended) {
				if (!a_state.runtimeSuspended || a_state.HasRuntime()) {
					spdlog::debug(
						"suspending prototype physics state for actor={} firstPerson={} with no current 3D; preserved armorRecords={} hadRuntime={}",
						static_cast<void*>(a_state.actor),
						a_state.firstPerson,
						a_state.armorRecords.size(),
						a_state.HasRuntime());
					SuspendPrototypeRuntimeLocked(a_state);
				}
				return true;
			}
			spdlog::debug(
				"dropping prototype physics state for actor={} firstPerson={} with no current 3D",
				static_cast<void*>(a_state.actor),
				a_state.firstPerson);
			return false;
		}

		const auto* player = RE::PlayerCharacter::GetSingleton();
		if (a_state.actor != player) {
			if (!enableNpcPhysics_) {
				spdlog::debug("dropping prototype physics state for actor={} because NPC physics is disabled", static_cast<void*>(a_state.actor));
				return false;
			}

			if (!IsActorInReferenceCullView(a_state.actor, root, a_state.firstPerson)) {
				if (a_state.HasRuntime() && !a_state.runtimeSoftSuspended) {
					spdlog::debug(
						"soft-suspending prototype physics state for actor={} because reference view culler marks it inactive",
						static_cast<void*>(a_state.actor));
					SoftSuspendPrototypeRuntimeLocked(a_state);
				}
				return true;
			}
		}

		return true;
	}

	void Fo4PhysicsWorld::PruneInvalidPrototypeStatesLocked()
	{
		for (auto& actorState : prototypeActors_) {
			if (!IsPrototypeStateValidLocked(actorState)) {
				ClearPrototypeStateLocked(actorState);
				actorState.actor = nullptr;
				actorState.actorHandle.reset();
			}
		}

		std::erase_if(prototypeActors_, [](const PrototypeActorState& a_state) {
			return !a_state.actor && !a_state.HasRuntime();
		});
	}

	void Fo4PhysicsWorld::EnforceActorBudgetLocked()
	{
		auto activeActors = static_cast<std::size_t>(std::ranges::count_if(prototypeActors_, [](const PrototypeActorState& a_state) {
			return a_state.HasActiveRuntime();
		}));
		if (activeActors <= currentMaxActiveActors_) {
			return;
		}

		const auto* player = RE::PlayerCharacter::GetSingleton();
		while (activeActors > currentMaxActiveActors_) {
			auto victim = prototypeActors_.end();
			auto victimDistanceSquared = -1.0F;
			for (auto it = prototypeActors_.begin(); it != prototypeActors_.end(); ++it) {
				if (!it->HasActiveRuntime()) {
					continue;
				}
				auto resolvedActor = it->actorHandle.get();
				if (!resolvedActor || resolvedActor.get() != it->actor) {
					continue;
				}
				if (player && it->actor == player) {
					continue;
				}

				auto distanceSquared = std::numeric_limits<float>::max();
				if (player) {
					distanceSquared = DistanceSquared(resolvedActor->GetPosition(), player->GetPosition());
				}

				if (victim == prototypeActors_.end() || distanceSquared > victimDistanceSquared) {
					victim = it;
					victimDistanceSquared = distanceSquared;
				}
			}

			if (victim == prototypeActors_.end()) {
				return;
			}

			spdlog::debug(
				"soft-suspending prototype physics state for actor={} because active actor budget shrank to {}",
				static_cast<void*>(victim->actor),
				currentMaxActiveActors_);
			SoftSuspendPrototypeRuntimeLocked(*victim);
			--activeActors;
		}
	}

	bool Fo4PhysicsWorld::ShouldBuildSuspendedArmorCandidateLocked(const LifecycleEvent& a_event) const
	{
		return IsArmorAttachCandidate(a_event.type) && !a_event.physicsXmlPath.empty();
	}

	void Fo4PhysicsWorld::SoftSuspendBuiltRuntimeIfOutOfRangeLocked(PrototypeActorState& a_state, const LifecycleEvent& a_event)
	{
		if (!a_state.HasActiveRuntime()) {
			return;
		}

		const auto* player = RE::PlayerCharacter::GetSingleton();
		if (a_state.actor == player) {
			return;
		}

		bool shouldSuspend = false;
		const char* reason = nullptr;
		if (!IsActorInReferenceCullView(a_state.actor, a_event.object, a_event.firstPerson)) {
			shouldSuspend = true;
			reason = "view";
		}
		const auto activeActors = static_cast<std::size_t>(std::ranges::count_if(prototypeActors_, [](const PrototypeActorState& a_candidate) {
			return a_candidate.HasActiveRuntime();
		}));
		if (!shouldSuspend && activeActors > currentMaxActiveActors_) {
			shouldSuspend = true;
			reason = "budget";
		}

		if (!shouldSuspend) {
			return;
		}

		std::vector<std::uint64_t> buildGroups;
		for (const auto& runtime : a_state.runtimes) {
			if (runtime.buildGroup != 0 && std::ranges::find(buildGroups, runtime.buildGroup) == buildGroups.end()) {
				buildGroups.push_back(runtime.buildGroup);
			}
		}

		spdlog::debug(
			"soft-suspending freshly built armor prototype runtime actor={} reason={} activeActors={} actorCap={} event={} buildGroups={}",
			static_cast<void*>(a_state.actor),
			reason ? reason : "unknown",
			activeActors,
			currentMaxActiveActors_,
			ToString(a_event.type),
			buildGroups.size());
		SoftSuspendPrototypeRuntimeLocked(a_state);
	}

	void Fo4PhysicsWorld::SuspendActorCandidateLocked(
		RE::Actor* a_actor,
		const bool a_firstPerson,
		std::vector<PrototypeArmorRecord> a_armorRecords)
	{
		const auto* player = RE::PlayerCharacter::GetSingleton();
		if (!a_actor || a_actor == player || !enableNpcPhysics_) {
			return;
		}
		if (a_armorRecords.empty()) {
			spdlog::trace(
				"skipping empty suspended prototype physics candidate actor={} firstPerson={}",
				static_cast<void*>(a_actor),
				a_firstPerson);
			return;
		}

		for (auto& candidate : suspendedActors_) {
			const auto resolvedActor = candidate.actorHandle.get();
			if (resolvedActor && resolvedActor.get() == a_actor) {
				candidate.firstPerson = a_firstPerson;
				candidate.armorRecords = std::move(a_armorRecords);
				return;
			}
		}

		auto handle = RE::BSPointerHandleManagerInterface<RE::Actor>::GetHandle(a_actor);
		if (!handle) {
			return;
		}

		suspendedActors_.push_back({
			.actorHandle = handle,
			.firstPerson = a_firstPerson,
			.armorRecords = std::move(a_armorRecords),
		});
		spdlog::debug(
			"suspended prototype physics candidate actor={} firstPerson={} armorRecords={} until view/budget allows rebuild",
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
			const auto activeActors = static_cast<std::size_t>(std::ranges::count_if(prototypeActors_, [](const PrototypeActorState& a_state) {
				return a_state.HasActiveRuntime();
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
			auto* existingState = actor ? FindPrototypeStateLocked(actor, it->firstPerson) : nullptr;
			if (!actor) {
				it = suspendedActors_.erase(it);
				continue;
			}
			if (existingState) {
				if ((existingState->runtimeSuspended || existingState->runtimeSoftSuspended) && !it->armorRecords.empty()) {
					for (auto& record : it->armorRecords) {
						MergePrototypeArmorRecord(existingState->armorRecords, std::move(record));
					}
					spdlog::debug(
						"merged suspended armor records into soft-suspended prototype state actor={} firstPerson={} armorRecords={}",
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
				"reactivated suspended prototype physics candidate actor={} firstPerson={} from preserved armor records",
				static_cast<void*>(actor),
				it->firstPerson);
			it = suspendedActors_.erase(it);
		}
	}

	void Fo4PhysicsWorld::TryReactivateSuspendedPrototypeStatesLocked()
	{
		if (!enableNpcPhysics_) {
			return;
		}

		const auto* player = RE::PlayerCharacter::GetSingleton();
		for (auto& actorState : prototypeActors_) {
			const auto needsSoftResume = actorState.runtimeSoftSuspended;
			const auto needsRebuild = actorState.runtimeSuspended && !actorState.armorRecords.empty();
			if ((!needsSoftResume && !needsRebuild) || actorState.actor == player) {
				continue;
			}

			const auto activeActors = static_cast<std::size_t>(std::ranges::count_if(prototypeActors_, [](const PrototypeActorState& a_state) {
				return a_state.HasActiveRuntime();
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
			if (!IsActorInReferenceCullView(actor, root, actorState.firstPerson)) {
				continue;
			}

			if (needsSoftResume && ResumeSoftSuspendedPrototypeRuntimeLocked(actorState)) {
				continue;
			}
			if (needsSoftResume && !actorState.armorRecords.empty()) {
				actorState.runtimeSuspended = true;
			}

			if (!actorState.runtimeSuspended || actorState.armorRecords.empty()) {
				continue;
			}

			auto armorRecords = actorState.armorRecords;
			PendingActorRebuild pending{
				.actorHandle = actorState.actorHandle,
				.firstPerson = actorState.firstPerson,
				.armorRecords = std::move(armorRecords),
			};
			if (!RebuildPendingArmorRecordsLocked(actor, pending)) {
				if (actorState.HasRuntime()) {
					actorState.runtimeSuspended = false;
				}
				continue;
			}

			actorState.runtimeSuspended = false;
			spdlog::debug(
				"reactivated soft-suspended prototype physics state actor={} firstPerson={} from preserved armor records",
				static_cast<void*>(actor),
				actorState.firstPerson);
		}
	}

	void Fo4PhysicsWorld::MergePrototypeArmorRecord(std::vector<PrototypeArmorRecord>& a_records, PrototypeArmorRecord a_record)
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
		const auto normalizedXml = ConfigPaths::LowerString(a_record.physicsXmlPath);
		auto existing = std::ranges::find_if(a_records, [&a_record, &normalizedXml](const PrototypeArmorRecord& a_existing) {
			return a_existing.bipedObject == a_record.bipedObject &&
				a_existing.attachedObject.get() == a_record.attachedObject.get() &&
				a_existing.sourceObject.get() == a_record.sourceObject.get() &&
				ConfigPaths::LowerString(a_existing.physicsXmlPath) == normalizedXml;
		});
		if (existing == a_records.end()) {
			existing = std::ranges::find_if(a_records, [&a_record, &normalizedXml](const PrototypeArmorRecord& a_existing) {
				return a_existing.bipedObject == a_record.bipedObject &&
					ConfigPaths::LowerString(a_existing.physicsXmlPath) == normalizedXml;
			});
		}
		if (existing != a_records.end()) {
			const auto recordHasRuntimeObject = a_record.attachedObject || a_record.sourceObject || a_record.mergeSourceObject;
			const auto sameRuntimeObjects =
				a_record.attachedObject.get() == existing->attachedObject.get() &&
				a_record.sourceObject.get() == existing->sourceObject.get() &&
				a_record.mergeSourceObject.get() == existing->mergeSourceObject.get();
			const auto mayInheritRecordedMergeState = !recordHasRuntimeObject || sameRuntimeObjects;
			if (!a_record.attachedObject && existing->attachedObject) {
				a_record.attachedObject = existing->attachedObject;
			}
			if (!a_record.sourceObject && existing->sourceObject) {
				a_record.sourceObject = existing->sourceObject;
			}
			if (!a_record.mergeSourceObject && existing->mergeSourceObject) {
				a_record.mergeSourceObject = existing->mergeSourceObject;
			}
			if (mayInheritRecordedMergeState && a_record.mergeParentBindings.empty() && !existing->mergeParentBindings.empty()) {
				a_record.mergeParentBindings = existing->mergeParentBindings;
			}
			if (mayInheritRecordedMergeState && a_record.mergeRenameMap.empty() && !existing->mergeRenameMap.empty()) {
				a_record.mergeRenameMap = existing->mergeRenameMap;
			}
			if (!mayInheritRecordedMergeState && (a_record.mergeParentBindings.empty() || a_record.mergeRenameMap.empty()) && (!existing->mergeParentBindings.empty() || !existing->mergeRenameMap.empty())) {
				spdlog::debug(
					"not inheriting recorded armor merge state for fresh runtime object actorRecord bipedObject={} xml='{}' object={} source={} existingObject={} existingSource={}",
					std::to_underlying(a_record.bipedObject),
					a_record.physicsXmlPath,
					static_cast<void*>(a_record.attachedObject.get()),
					static_cast<void*>(a_record.sourceObject.get()),
					static_cast<void*>(existing->attachedObject.get()),
					static_cast<void*>(existing->sourceObject.get()));
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

	std::uint32_t Fo4PhysicsWorld::PruneStalePendingHairSlotArmorRecords(std::vector<PrototypeArmorRecord>& a_records)
	{
		std::uint32_t removed = 0;
		for (auto it = a_records.begin(); it != a_records.end();) {
			if (!IsHairBipedObject(it->bipedObject)) {
				++it;
				continue;
			}

			const auto bipedObject = it->bipedObject;
			const auto hasLaterSlotOwner = std::ranges::any_of(std::next(it), a_records.end(), [bipedObject](const PrototypeArmorRecord& a_record) {
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

	bool Fo4PhysicsWorld::PrototypeArmorRecordsIncludeHairSlot(const std::span<const PrototypeArmorRecord> a_records)
	{
		return std::ranges::any_of(a_records, [](const PrototypeArmorRecord& a_record) {
			return IsHairBipedObject(a_record.bipedObject);
		});
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

	std::vector<Fo4PhysicsWorld::PrototypeArmorRecord> Fo4PhysicsWorld::CollectQueuedArmorRecordsForAttachLocked(const LifecycleEvent& a_event)
	{
		std::vector<PrototypeArmorRecord> records;
		if (auto* pending = FindPendingActorRebuildLocked(a_event.actor, a_event.firstPerson)) {
			for (auto& record : pending->armorRecords) {
				MergePrototypeArmorRecord(records, record);
			}
		} else if (auto* actorState = FindPrototypeStateLocked(a_event.actor, a_event.firstPerson)) {
			for (auto& record : actorState->armorRecords) {
				MergePrototypeArmorRecord(records, record);
			}
		}

		for (auto& record : CollectSuspendedArmorRecordsLocked(a_event)) {
			MergePrototypeArmorRecord(records, std::move(record));
		}
		return records;
	}

	std::vector<Fo4PhysicsWorld::PrototypeArmorRecord> Fo4PhysicsWorld::CollectQueuedArmorRecordsForDetachLocked(const LifecycleEvent& a_event)
	{
		std::vector<PrototypeArmorRecord> records;
		if (auto* pending = FindPendingActorRebuildLocked(a_event.actor, a_event.firstPerson)) {
			records = pending->armorRecords;
		} else if (auto* actorState = FindPrototypeStateLocked(a_event.actor, a_event.firstPerson)) {
			records = actorState->armorRecords;
		}
		const auto detachedBipedObject = ResolveEventBipedObject(a_event);
		if (detachedBipedObject != RE::BIPED_OBJECT::kTotal) {
			std::erase_if(records, [detachedBipedObject, object = a_event.object](const PrototypeArmorRecord& a_record) {
				if (a_record.bipedObject != detachedBipedObject) {
					return false;
				}
				if (!object) {
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
		std::vector<PrototypeArmorRecord> a_armorRecords,
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

		const auto requestedDelay = std::ranges::any_of(a_armorRecords, [](const PrototypeArmorRecord& a_record) {
			return a_record.cpuCopyRetryCount > 0;
		}) ? kCpuCopyPendingRetryDelayTasks : 0U;
		const auto rebuildDelay = a_forceArmorRescan ? std::max(requestedDelay, kArmorChangeRebuildDelayTasks) : requestedDelay;
		const auto frameDelay = std::max(rebuildDelay, a_scheduleImmediately ? 0U : 1U);
		if (auto* pending = FindPendingActorRebuildLocked(a_actor, a_firstPerson)) {
			if (a_replaceArmorRecords) {
				pending->armorRecords = std::move(a_armorRecords);
			} else {
				for (auto& record : a_armorRecords) {
					MergePrototypeArmorRecord(pending->armorRecords, std::move(record));
				}
			}
			const auto prunedHairSlotRecords = PruneStalePendingHairSlotArmorRecords(pending->armorRecords);
			pending->frameDelay = std::max(pending->frameDelay, frameDelay);
			pending->forceArmorRescan = pending->forceArmorRescan || a_forceArmorRescan;
			spdlog::debug(
				"updated pending prototype physics rebuild for actor={} firstPerson={} armorRecords={} forceArmorRescan={} scheduleImmediately={} replaceArmorRecords={}",
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
			"queued pending prototype physics rebuild for actor={} firstPerson={} armorRecords={} forceArmorRescan={} scheduleImmediately={} replaceArmorRecords={}",
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

	bool Fo4PhysicsWorld::SoftReloadPrototypeStateLocked(PrototypeActorState& a_state, const LifecycleEventType a_reason)
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
		const auto hadRuntime = a_state.HasRuntime();
		if (hadRuntime) {
			SuspendPrototypeRuntimeLocked(a_state);
		}

		if (armorRecords.empty() && !hadRuntime) {
			return false;
		}

		const auto forceArmorRescan = armorRecords.empty();
		MarkPendingActorRebuildLocked(actor, a_state.firstPerson, std::move(armorRecords), forceArmorRescan, true, true);
		ResetStepClockLocked();
		spdlog::debug(
			"soft-reloaded prototype physics state after {} actor={} firstPerson={} hadRuntime={} forceArmorRescan={}",
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

		if (auto* pendingActorRebuild = FindPendingActorRebuildLocked(a_event.actor, false);
			pendingActorRebuild && PrototypeArmorRecordsIncludeHairSlot(pendingActorRebuild->armorRecords)) {
			const auto removed = std::erase_if(pendingHeadRebuilds_, [&](const PendingHeadRebuild& a_pending) {
				const auto resolvedActor = a_pending.actorHandle.get();
				return resolvedActor && resolvedActor.get() == a_event.actor;
			});
			spdlog::debug(
				"discarded head physics rebuild for actor={} while hair-slot armor rebuild is pending removedPendingHeads={}",
				static_cast<void*>(a_event.actor),
				removed);
			return;
		}

		auto handle = RE::BSPointerHandleManagerInterface<RE::Actor>::GetHandle(a_event.actor);
		if (!handle) {
			return;
		}

		for (auto& pending : pendingHeadRebuilds_) {
			auto resolvedActor = pending.actorHandle.get();
			if (resolvedActor && resolvedActor.get() == a_event.actor && pending.type == a_event.type && pending.object.get() == a_event.object) {
				pending.frameDelay = std::max(pending.frameDelay, kHeadInitializedRebuildDelayFrames);
				pending.headPart = a_event.headPart;
				return;
			}
		}

		pendingHeadRebuilds_.push_back({
			.actorHandle = handle,
			.type = a_event.type,
			.object = a_event.object,
			.headPart = a_event.headPart,
			.frameDelay = kHeadInitializedRebuildDelayFrames,
		});
	}

	bool Fo4PhysicsWorld::HasActiveOrPendingActorRebuildLocked(RE::Actor* a_actor)
	{
		if (!a_actor) {
			return false;
		}

		if (std::ranges::any_of(prototypeActors_, [a_actor](const PrototypeActorState& a_state) {
			return a_state.actor == a_actor;
		})) {
			return true;
		}

		return std::ranges::any_of(pendingActorRebuilds_, [a_actor](const PendingActorRebuild& a_pending) {
			const auto resolvedActor = a_pending.actorHandle.get();
			return resolvedActor && resolvedActor.get() == a_actor;
		});
	}

	std::vector<Fo4PhysicsWorld::PrototypeArmorRecord> Fo4PhysicsWorld::CollectSuspendedArmorRecordsLocked(const LifecycleEvent& a_event)
	{
		std::vector<PrototypeArmorRecord> records;
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
			.mergeSourceObject = a_event.mergeSourceObject,
			.mergeParentBindings = a_event.mergeParentBindings,
			.mergeRenameMap = a_event.mergeRenameMap,
		});
		return records;
	}

	void Fo4PhysicsWorld::RecordPrototypeArmorLocked(
		PrototypeActorState& a_state,
		const RE::BIPED_OBJECT a_bipedObject,
		std::string a_physicsXmlPath,
		const DefaultBBP::NameMap& a_meshNameMap,
		RE::NiAVObject* a_attachedObject,
		RE::NiAVObject* a_sourceObject,
		RE::NiAVObject* a_mergeSourceObject,
		std::vector<MergeRename> a_mergeRenameMap,
		const std::uint64_t a_buildGroup)
	{
		if (a_bipedObject == RE::BIPED_OBJECT::kTotal || a_physicsXmlPath.empty()) {
			return;
		}

		std::vector<MergeParentBinding> mergeParentBindings;
		std::vector<MergeRename> mergeRenameMap = std::move(a_mergeRenameMap);
		if (a_buildGroup != 0) {
			for (const auto& mergedNode : a_state.mergedNodes) {
				auto* node = mergedNode.node ? mergedNode.node->IsNode() : nullptr;
				if (mergedNode.buildGroup == a_buildGroup) {
					for (const auto& rename : mergedNode.subtreeRenameMap) {
						if (rename.sourceName.empty() || rename.renamedName.empty()) {
							continue;
						}
						const auto alreadyMapped = std::ranges::any_of(mergeRenameMap, [&rename](const MergeRename& a_entry) {
							return PhysicsNamesEqual(a_entry.sourceName, rename.sourceName);
						});
						if (!alreadyMapped) {
							mergeRenameMap.push_back(rename);
						}
					}
				}
				if (mergedNode.buildGroup == a_buildGroup && node && !mergedNode.sourceName.empty()) {
					const auto nodeName = node->GetName();
					const auto alreadyMapped = std::ranges::any_of(mergeRenameMap, [&mergedNode](const MergeRename& a_entry) {
						return PhysicsNamesEqual(a_entry.sourceName, mergedNode.sourceName);
					});
					if (!alreadyMapped && !nodeName.empty() && !PhysicsNamesEqual(mergedNode.sourceName, nodeName)) {
						mergeRenameMap.push_back({
							.sourceName = mergedNode.sourceName,
							.renamedName = std::string(nodeName),
						});
					}
				}
				const auto parentName = !mergedNode.recordParentName.empty() ?
					mergedNode.recordParentName :
					(mergedNode.parent ? std::string(mergedNode.parent->GetName()) : std::string{});
				if (mergedNode.buildGroup != a_buildGroup || !mergedNode.recordMergeParentBinding || !node || parentName.empty()) {
					continue;
				}

				auto sourceName = mergedNode.sourceName;
				if (sourceName.empty()) {
					continue;
				}

				const auto& recordLocal = mergedNode.hasRecordLocalToParent ?
					mergedNode.recordLocalToParent :
					(mergedNode.hasLocalToParent ? mergedNode.localToParent : node->local);
				mergeParentBindings.push_back({
					.sourceName = std::move(sourceName),
					.parentName = parentName,
					.localToParent = recordLocal,
					.hasLocalToParent = true,
				});
			}
		}
		for (const auto& binding : mergeParentBindings) {
			spdlog::debug(
				"recorded armor merge parent binding actor={} bipedObject={} xml='{}' source='{}' parent='{}' buildGroup={} localToParent=({:.3f},{:.3f},{:.3f})",
				static_cast<void*>(a_state.actor),
				std::to_underlying(a_bipedObject),
				a_physicsXmlPath,
				binding.sourceName,
				binding.parentName,
				a_buildGroup,
				binding.localToParent.translate.x,
				binding.localToParent.translate.y,
				binding.localToParent.translate.z);
		}

		const auto normalizedXml = ConfigPaths::LowerString(a_physicsXmlPath);
		const auto appendBuildGroup = [](std::vector<std::uint64_t>& a_buildGroups, const std::uint64_t a_buildGroup) {
			if (a_buildGroup != 0 && std::ranges::find(a_buildGroups, a_buildGroup) == a_buildGroups.end()) {
				a_buildGroups.push_back(a_buildGroup);
			}
		};
		auto existing = std::ranges::find_if(a_state.armorRecords, [&](const PrototypeArmorRecord& a_record) {
			return a_record.bipedObject == a_bipedObject &&
				a_record.attachedObject.get() == a_attachedObject &&
				a_record.sourceObject.get() == a_sourceObject &&
				ConfigPaths::LowerString(a_record.physicsXmlPath) == normalizedXml;
		});
		if (existing == a_state.armorRecords.end()) {
			existing = std::ranges::find_if(a_state.armorRecords, [&](const PrototypeArmorRecord& a_record) {
				return a_record.bipedObject == a_bipedObject &&
					ConfigPaths::LowerString(a_record.physicsXmlPath) == normalizedXml;
			});
		}
		if (existing != a_state.armorRecords.end()) {
			existing->physicsXmlPath = std::move(a_physicsXmlPath);
			existing->meshNameMap = a_meshNameMap;
			existing->attachedObject = a_attachedObject;
			existing->sourceObject = a_sourceObject;
			if (a_mergeSourceObject) {
				existing->mergeSourceObject = a_mergeSourceObject;
			}
			existing->mergeParentBindings = std::move(mergeParentBindings);
			existing->mergeRenameMap = std::move(mergeRenameMap);
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
				.mergeSourceObject = a_mergeSourceObject,
				.mergeParentBindings = std::move(mergeParentBindings),
				.mergeRenameMap = std::move(mergeRenameMap),
				.buildGroups = std::move(buildGroups),
			});
		}
	}

	Fo4PhysicsWorld::PrototypeAttachmentRecord* Fo4PhysicsWorld::FindPrototypeAttachmentLocked(
		PrototypeActorState& a_state,
		const RE::BIPED_OBJECT a_bipedObject,
		RE::NiAVObject* a_object,
		RE::NiAVObject* a_sourceObject,
		const std::string_view a_physicsXmlPath)
	{
		if (a_bipedObject == RE::BIPED_OBJECT::kTotal) {
			return nullptr;
		}

		const auto normalizedXml = a_physicsXmlPath.empty() ? std::string{} : ConfigPaths::LowerString(std::string(a_physicsXmlPath));
		const auto found = std::ranges::find_if(a_state.attachmentRecords, [&](const PrototypeAttachmentRecord& a_record) {
			if (a_record.bipedObject != a_bipedObject) {
				return false;
			}
			if (a_object && a_record.attachedObject.get() != a_object) {
				return false;
			}
			if (a_sourceObject && a_record.sourceObject && a_record.sourceObject.get() != a_sourceObject) {
				return false;
			}
			if (!normalizedXml.empty() && ConfigPaths::LowerString(a_record.physicsXmlPath) != normalizedXml) {
				return false;
			}
			return true;
		});
		return found != a_state.attachmentRecords.end() ? std::addressof(*found) : nullptr;
	}

	const Fo4PhysicsWorld::PrototypeAttachmentRecord* Fo4PhysicsWorld::FindPrototypeAttachmentLocked(
		const PrototypeActorState& a_state,
		const RE::BIPED_OBJECT a_bipedObject,
		RE::NiAVObject* a_object,
		RE::NiAVObject* a_sourceObject,
		const std::string_view a_physicsXmlPath)
	{
		if (a_bipedObject == RE::BIPED_OBJECT::kTotal) {
			return nullptr;
		}

		const auto normalizedXml = a_physicsXmlPath.empty() ? std::string{} : ConfigPaths::LowerString(std::string(a_physicsXmlPath));
		const auto found = std::ranges::find_if(a_state.attachmentRecords, [&](const PrototypeAttachmentRecord& a_record) {
			if (a_record.bipedObject != a_bipedObject) {
				return false;
			}
			if (a_object && a_record.attachedObject.get() != a_object) {
				return false;
			}
			if (a_sourceObject && a_record.sourceObject && a_record.sourceObject.get() != a_sourceObject) {
				return false;
			}
			if (!normalizedXml.empty() && ConfigPaths::LowerString(a_record.physicsXmlPath) != normalizedXml) {
				return false;
			}
			return true;
		});
		return found != a_state.attachmentRecords.end() ? std::addressof(*found) : nullptr;
	}

	bool Fo4PhysicsWorld::IsPrototypeAttachmentCurrentLocked(
		const PrototypeActorState& a_state,
		const RE::BIPED_OBJECT a_bipedObject,
		RE::NiAVObject* a_object,
		RE::NiAVObject* a_sourceObject,
		const std::string_view a_physicsXmlPath)
	{
		const auto* record = FindPrototypeAttachmentLocked(a_state, a_bipedObject, a_object, a_sourceObject, a_physicsXmlPath);
		if (!record || record->buildGroups.empty() || record->physicsXmlPath.empty()) {
			return false;
		}

		const auto sameXml = ConfigPaths::LowerString(record->physicsXmlPath) == ConfigPaths::LowerString(std::string(a_physicsXmlPath));
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
			if (std::ranges::any_of(a_state.meshes, [a_buildGroup](const PrototypeMesh& a_mesh) {
					return a_mesh.buildGroup == a_buildGroup;
				})) {
				return true;
			}
			return std::ranges::any_of(a_state.bodies, [a_buildGroup](const PrototypeBody& a_body) {
				return PrototypeBodyHasBuildGroup(a_body, a_buildGroup);
			});
		});
	}

	void Fo4PhysicsWorld::RecordPrototypeAttachmentLocked(
		PrototypeActorState& a_state,
		const RE::BIPED_OBJECT a_bipedObject,
		RE::NiAVObject* a_object,
		RE::NiAVObject* a_sourceObject,
		std::string a_physicsXmlPath,
		const std::uint64_t a_buildGroup)
	{
		if (a_bipedObject == RE::BIPED_OBJECT::kTotal || a_physicsXmlPath.empty() || a_buildGroup == 0) {
			return;
		}

		auto* record = FindPrototypeAttachmentLocked(a_state, a_bipedObject, a_object, a_sourceObject, a_physicsXmlPath);
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
			auto recordScore = [](const PrototypeArmorRecord& a_record) {
				auto score = 0;
				if (!a_record.mergeRenameMap.empty()) {
					score += 8;
				}
				if (!a_record.mergeParentBindings.empty()) {
					score += 2;
				}
				return score;
			};
			auto fillMissingRecordState = [](PrototypeArmorRecord& a_target, const PrototypeArmorRecord& a_source) {
				const auto targetHasRuntimeObject = a_target.attachedObject || a_target.sourceObject || a_target.mergeSourceObject;
				const auto sameRuntimeObjects =
					a_target.attachedObject.get() == a_source.attachedObject.get() &&
					a_target.sourceObject.get() == a_source.sourceObject.get() &&
					a_target.mergeSourceObject.get() == a_source.mergeSourceObject.get();
				const auto mayInheritRecordedMergeState = !targetHasRuntimeObject || sameRuntimeObjects;
				if (!a_target.attachedObject && a_source.attachedObject) {
					a_target.attachedObject = a_source.attachedObject;
				}
				if (!a_target.sourceObject && a_source.sourceObject) {
					a_target.sourceObject = a_source.sourceObject;
				}
				if (!a_target.mergeSourceObject && a_source.mergeSourceObject) {
					a_target.mergeSourceObject = a_source.mergeSourceObject;
				}
				if (a_target.meshNameMap.empty() && !a_source.meshNameMap.empty()) {
					a_target.meshNameMap = a_source.meshNameMap;
				}
				if (mayInheritRecordedMergeState && a_target.mergeParentBindings.empty() && !a_source.mergeParentBindings.empty()) {
					a_target.mergeParentBindings = a_source.mergeParentBindings;
				}
				if (mayInheritRecordedMergeState && a_target.mergeRenameMap.empty() && !a_source.mergeRenameMap.empty()) {
					a_target.mergeRenameMap = a_source.mergeRenameMap;
				}
				if (!mayInheritRecordedMergeState && (a_target.mergeParentBindings.empty() || a_target.mergeRenameMap.empty()) && (!a_source.mergeParentBindings.empty() || !a_source.mergeRenameMap.empty())) {
					spdlog::debug(
						"not merging stale pending armor merge state for fresh runtime object bipedObject={} xml='{}' object={} source={} staleObject={} staleSource={}",
						std::to_underlying(a_target.bipedObject),
						a_target.physicsXmlPath,
						static_cast<void*>(a_target.attachedObject.get()),
						static_cast<void*>(a_target.sourceObject.get()),
						static_cast<void*>(a_source.attachedObject.get()),
						static_cast<void*>(a_source.sourceObject.get()));
				}
				for (const auto buildGroup : a_source.buildGroups) {
					if (buildGroup != 0 && std::ranges::find(a_target.buildGroups, buildGroup) == a_target.buildGroups.end()) {
						a_target.buildGroups.push_back(buildGroup);
					}
				}
			};

			std::vector<PrototypeArmorRecord> mergedRecords;
			for (auto& record : a_pending.armorRecords) {
				if (record.bipedObject == RE::BIPED_OBJECT::kTotal || record.physicsXmlPath.empty()) {
					continue;
				}

				const auto normalizedXml = ConfigPaths::LowerString(record.physicsXmlPath);
				auto existing = std::ranges::find_if(mergedRecords, [&](const PrototypeArmorRecord& a_existing) {
					return a_existing.bipedObject == record.bipedObject &&
						ConfigPaths::LowerString(a_existing.physicsXmlPath) == normalizedXml;
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
			auto* rebuildObject = record.attachedObject && IsProbablyValidNiObject(record.attachedObject.get()) ? record.attachedObject.get() : partClone;
			auto* rebuildSourceObject = record.sourceObject && IsProbablyValidNiObject(record.sourceObject.get()) ? record.sourceObject.get() : partClone;
			auto* rebuildSourceRoot = rebuildSourceObject ? rebuildSourceObject->IsNode() : partClone->IsNode();

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
				.mergeSourceObject = record.mergeSourceObject.get(),
				.mergeParentBindings = record.mergeParentBindings,
				.mergeRenameMap = record.mergeRenameMap,
				.sourceRoot = rebuildSourceRoot,
				.physicsXmlPath = record.physicsXmlPath,
				.firstPerson = a_pending.firstPerson,
			};

			auto& actorState = GetOrCreatePrototypeStateLocked(a_actor, a_pending.firstPerson);
			std::vector<std::uint64_t> staleArmorBuildGroups;
			const auto hairSlotArmorBuild = IsHairBipedObject(record.bipedObject);
			if (const auto* attachment = FindPrototypeAttachmentLocked(actorState, record.bipedObject, rebuildObject, rebuildSourceObject, record.physicsXmlPath)) {
				for (const auto buildGroup : attachment->buildGroups) {
					if (buildGroup != 0 && std::ranges::find(staleArmorBuildGroups, buildGroup) == staleArmorBuildGroups.end()) {
						staleArmorBuildGroups.push_back(buildGroup);
					}
				}
			}
			for (const auto buildGroup : CollectPrototypeGroupsForObjectLocked(actorState, rebuildObject)) {
				if (buildGroup != 0 && std::ranges::find(staleArmorBuildGroups, buildGroup) == staleArmorBuildGroups.end()) {
					staleArmorBuildGroups.push_back(buildGroup);
				}
			}
			if (hairSlotArmorBuild) {
				std::vector<std::uint64_t> staleHeadBuildGroups;
				const auto replacedHeadParts = CollectHeadPartGroupsLocked(actorState, staleHeadBuildGroups);
				if (replacedHeadParts > 0) {
					spdlog::debug(
						"pending hair-slot armor rebuild is replacing tracked head/hair prototype groups actor={} bipedObject={} object={} xml='{}' headPartRecords={} groups={}",
						static_cast<void*>(a_actor),
						std::to_underlying(record.bipedObject),
						static_cast<void*>(rebuildObject),
						record.physicsXmlPath,
						replacedHeadParts,
						staleHeadBuildGroups.size());
				}
				if (!staleHeadBuildGroups.empty()) {
					ClearPrototypeGroupsLocked(actorState, staleHeadBuildGroups, true);
				}
			}
			if (!hairSlotArmorBuild && !staleArmorBuildGroups.empty()) {
				ClearPrototypeGroupsLocked(actorState, staleArmorBuildGroups, record.mergeRenameMap.empty());
				const auto clearedCount = staleArmorBuildGroups.size();
				staleArmorBuildGroups.clear();
				spdlog::debug(
					"cleared stale prototype groups before pending armor rebuild actor={} bipedObject={} object={} xml='{}' groups={}",
					static_cast<void*>(a_actor),
					std::to_underlying(record.bipedObject),
					static_cast<void*>(rebuildObject),
					record.physicsXmlPath,
					clearedCount);
			}
			if (hairSlotArmorBuild) {
				ClearStaleHairSlotArmorGroupsLocked(actorState, record.bipedObject, 0, "pending-armor-rebuild-prebuild", rebuildObject, record.physicsXmlPath);
				staleArmorBuildGroups.clear();
			}
			spdlog::debug(
				"rebuilding pending armor prototype physics actor={} bipedObject={} object={} xml='{}' stagedStaleGroups={}",
				static_cast<void*>(a_actor),
				std::to_underlying(record.bipedObject),
				static_cast<void*>(rebuildObject),
				record.physicsXmlPath,
				staleArmorBuildGroups.size());
			for (const auto& binding : record.mergeParentBindings) {
				spdlog::debug(
					"pending armor rebuild merge parent binding actor={} bipedObject={} xml='{}' source='{}' parent='{}' hasLocal={} localToParent=({:.3f},{:.3f},{:.3f})",
					static_cast<void*>(a_actor),
					std::to_underlying(record.bipedObject),
					record.physicsXmlPath,
					binding.sourceName,
					binding.parentName,
					binding.hasLocalToParent,
					binding.localToParent.translate.x,
					binding.localToParent.translate.y,
					binding.localToParent.translate.z);
			}
			for (const auto& rename : record.mergeRenameMap) {
				spdlog::debug(
					"pending armor rebuild rename map actor={} bipedObject={} xml='{}' source='{}' renamed='{}'",
					static_cast<void*>(a_actor),
					std::to_underlying(record.bipedObject),
					record.physicsXmlPath,
					rename.sourceName,
					rename.renamedName);
			}
			auto buildResult = BuildPrototypeBodiesLocked(actorState, resumeEvent, *selectedSummary, record.meshNameMap, PrototypeBuildDomain::kArmor, !hairSlotArmorBuild);
			if (buildResult.succeeded) {
				if (hairSlotArmorBuild) {
					CommitPrototypeBuildGroupToBulletLocked(actorState, buildResult.buildGroup);
					buildResult.committed = true;
					LogPrototypeActorBulletObjectsLocked(actorState, "after-prototype-build-commit");
				} else if (!staleArmorBuildGroups.empty()) {
					ClearPrototypeGroupsLocked(actorState, staleArmorBuildGroups);
				}
				RecordPrototypeAttachmentLocked(actorState, record.bipedObject, rebuildObject, rebuildSourceObject, record.physicsXmlPath, buildResult.buildGroup);
				RecordPrototypeArmorLocked(
					actorState,
					record.bipedObject,
					record.physicsXmlPath,
					record.meshNameMap,
					rebuildObject,
					rebuildSourceObject,
					record.mergeSourceObject.get(),
					record.mergeRenameMap,
					buildResult.buildGroup);
				if (hairSlotArmorBuild) {
					const auto remainingHairSlotArmorGroups = CollectArmorPrototypeGroupsForBipedObjectLocked(actorState, record.bipedObject).size();
					spdlog::debug(
						"hair-slot armor ownership after pending commit actor={} bipedObject={} buildGroup={} object={} xml='{}' remainingArmorGroups={}",
						static_cast<void*>(a_actor),
						std::to_underlying(record.bipedObject),
						buildResult.buildGroup,
						static_cast<void*>(rebuildObject),
						record.physicsXmlPath,
						remainingHairSlotArmorGroups);
				}
				SoftSuspendBuiltRuntimeIfOutOfRangeLocked(actorState, resumeEvent);
			} else if (buildResult.buildGroup != 0 && PrototypeBuildGroupIsRecordableLocked(actorState, buildResult.buildGroup, PrototypeBuildDomain::kArmor)) {
				ClearPrototypeGroupsLocked(actorState, std::vector<std::uint64_t>{ buildResult.buildGroup });
				spdlog::debug(
					"rolled back incomplete pending armor prototype build group actor={} bipedObject={} object={} buildGroup={} xml='{}' pendingCpuCopy={}",
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
						"retrying pending prototype mesh CPU copy actor={} bipedObject={} object={} attempt={}/{} delayTasks={}",
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
					"giving up pending prototype mesh CPU copy actor={} bipedObject={} object={} attempts={} xml='{}'",
					static_cast<void*>(a_actor),
					std::to_underlying(record.bipedObject),
					static_cast<void*>(rebuildObject),
					record.cpuCopyRetryCount,
					record.physicsXmlPath);
			}
			it = a_pending.armorRecords.erase(it);
		}

		return a_pending.armorRecords.empty();
	}

	void Fo4PhysicsWorld::TryRebuildPendingActorsLocked(RE::Actor* a_actor)
	{
		if (characterCustomizationMenuDepth_ > 0) {
			return;
		}
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

				std::vector<RE::NiAVObject*> trackedMergedObjects;
				if (auto* existingState = FindPrototypeStateLocked(actor, it->firstPerson)) {
					trackedMergedObjects.reserve(existingState->mergedNodes.size());
					for (const auto& mergedNode : existingState->mergedNodes) {
						if (mergedNode.node) {
							trackedMergedObjects.push_back(mergedNode.node.get());
						}
					}
					const auto prunedStaleMergedNodes = DetachStaleArmorMergedNodes(root, trackedMergedObjects, actor, "full-actor-rebuild-before-clear");
					if (prunedStaleMergedNodes > 0) {
						spdlog::debug(
							"pruned {} stale untracked armor merge nodes before full actor prototype rebuild actor={} firstPerson={}",
							prunedStaleMergedNodes,
							static_cast<void*>(actor),
							it->firstPerson);
					}
					ClearPrototypeStateLocked(*existingState);
					std::erase_if(prototypeActors_, [](const PrototypeActorState& a_state) {
						return !a_state.runtimeSuspended && !a_state.HasRuntime() && a_state.armorRecords.empty();
					});
				} else {
					const auto prunedStaleMergedNodes = DetachStaleArmorMergedNodes(root, trackedMergedObjects, actor, "full-actor-rebuild-no-state");
					if (prunedStaleMergedNodes > 0) {
						spdlog::debug(
							"pruned {} stale armor merge nodes before stateless full actor prototype rebuild actor={} firstPerson={}",
							prunedStaleMergedNodes,
							static_cast<void*>(actor),
							it->firstPerson);
					}
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
					if (IsPrototypeCandidateLocked(rebuildEvent, true)) {
						spdlog::debug(
							"processing full actor prototype physics rebuild for actor={} root={} firstPerson={} with no queued armor records",
							static_cast<void*>(actor),
							static_cast<void*>(root),
							it->firstPerson);
						BuildPrototypeForEventLocked(rebuildEvent);
					}
					it = pendingActorRebuilds_.erase(it);
					continue;
				}
				spdlog::debug(
					"processing full actor prototype physics rebuild for actor={} root={} firstPerson={} armorRecords={}",
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
					LogActorSkeletonHierarchy(actor, it->firstPerson, "after-full-actor-prototype-rebuild");
					if (const auto* rebuiltState = FindPrototypeStateLocked(actor, it->firstPerson)) {
						LogPrototypeActorBulletObjectsLocked(*rebuiltState, "after-full-actor-prototype-rebuild");
					}
				}
				it = pendingActorRebuilds_.erase(it);
				continue;
			}

			if (auto* existingState = FindPrototypeStateLocked(actor, it->firstPerson);
				existingState && existingState->HasActiveRuntime()) {
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
			if (IsPrototypeCandidateLocked(rebuildEvent, true)) {
				spdlog::debug(
					"processing pending prototype physics rebuild for actor={} root={} firstPerson={}",
					static_cast<void*>(actor),
					static_cast<void*>(root),
					it->firstPerson);
				BuildPrototypeForEventLocked(rebuildEvent);
			}
			const auto* rebuiltState = FindPrototypeStateLocked(actor, it->firstPerson);
			const auto rebuiltRuntime = rebuiltState && rebuiltState->HasActiveRuntime();
			if (rebuiltRuntime) {
				it = pendingActorRebuilds_.erase(it);
				continue;
			}
			if (rebuildEvent.biped) {
				spdlog::debug(
					"dropping pending prototype physics rebuild for actor={} firstPerson={} because no early SMP armor records are tracked",
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
		if (characterCustomizationMenuDepth_ > 0 || pendingHeadRebuilds_.empty()) {
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
			const auto actorRebuildPending = std::ranges::any_of(pendingActorRebuilds_, [actor](const PendingActorRebuild& a_pending) {
				const auto resolvedPendingActor = a_pending.actorHandle.get();
				return resolvedPendingActor && resolvedPendingActor.get() == actor;
			});
			if (actorRebuildPending) {
				const auto hairSlotArmorRebuildPending = std::ranges::any_of(pendingActorRebuilds_, [actor](const PendingActorRebuild& a_pending) {
					const auto resolvedPendingActor = a_pending.actorHandle.get();
					return resolvedPendingActor &&
						resolvedPendingActor.get() == actor &&
						PrototypeArmorRecordsIncludeHairSlot(a_pending.armorRecords);
				});
				if (hairSlotArmorRebuildPending) {
					spdlog::debug(
						"dropping pending head physics rebuild for actor={} because a hair-slot armor rebuild owns the slot",
						static_cast<void*>(actor));
					it = pendingHeadRebuilds_.erase(it);
					continue;
				}
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
				.headPart = it->headPart,
				.firstPerson = false,
			};
			if (IsPrototypeCandidateLocked(headEvent, false)) {
				spdlog::debug(
					"processing pending head physics rebuild for actor={} root={} faceNode={}",
					static_cast<void*>(actor),
					static_cast<void*>(root),
					static_cast<void*>(faceNode));
				BuildHeadPrototypeForEventLocked(headEvent);
			}
			it = pendingHeadRebuilds_.erase(it);
		}
	}
}
