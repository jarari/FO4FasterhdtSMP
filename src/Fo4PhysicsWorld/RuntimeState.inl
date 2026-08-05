// Included by ../Fo4PhysicsWorld.cpp.
// Split from Fo4PhysicsWorld.cpp. Do not compile this file separately.

namespace Smp
{
	bool Fo4PhysicsWorld::BeginActorSkeletonTransition(
		RE::Actor* a_actor,
		RE::NiAVObject* a_oldRoot,
		RE::BSFaceGenNiNode* a_retainedFace)
	{
		if (!a_actor || !a_oldRoot) {
			return false;
		}

		WaitForAsyncStep();
		std::scoped_lock lock(lock_);

		const auto resolvesActor = [a_actor](const RE::ActorHandle& a_handle) {
			const auto actor = a_handle.get();
			return actor && actor.get() == a_actor;
		};
		const auto hasTrackedState =
			std::ranges::any_of(systems_, [a_actor](const auto& a_state) {
				return a_state->actor == a_actor && !a_state->firstPerson;
			}) ||
			std::ranges::any_of(suspendedActors_, [&](const SuspendedActorCandidate& a_candidate) {
				return !a_candidate.firstPerson && resolvesActor(a_candidate.actorHandle);
			}) ||
			std::ranges::any_of(pendingActorRebuilds_, [&](const PendingActorRebuild& a_pending) {
				return !a_pending.firstPerson && resolvesActor(a_pending.actorHandle);
			}) ||
			std::ranges::any_of(pendingHeadRebuilds_, [&](const PendingHeadRebuild& a_pending) {
				return resolvesActor(a_pending.actorHandle);
			}) ||
			std::ranges::any_of(pendingSkeletonTransitions_, [&](const PendingSkeletonTransition& a_pending) {
				return resolvesActor(a_pending.actorHandle);
			}) ||
			std::ranges::any_of(retainedHeadSkeletonCaches_, [&](const RetainedHeadSkeletonCache& a_cached) {
				return resolvesActor(a_cached.actorHandle);
			});
		if (!hasTrackedState) {
			return false;
		}

		auto actorHandle = a_actor == RE::PlayerCharacter::GetSingleton() ?
			RE::PlayerCharacter::GetPlayerHandle() :
			RE::BSPointerHandleManagerInterface<RE::Actor>::GetHandle(a_actor);
		if (!actorHandle) {
			return false;
		}

		std::vector<ArmorPhysicsRecord> armorRecords;
		std::vector<ArmorBoneReference> headBoneReferences;
		std::vector<std::string> requiredHeadBoneNames;
		RE::NiPointer<RE::NiAVObject> retainedFace{ reinterpret_cast<RE::NiAVObject*>(a_retainedFace) };
		auto retainedSkinBindings = CaptureRetainedSkinBindings(retainedFace.get());

		const auto retainedFaceIdentity = reinterpret_cast<std::uintptr_t>(retainedFace.get());
		std::erase_if(
			retainedHeadSkeletonCaches_,
			[&](const RetainedHeadSkeletonCache& a_cached) {
				const auto actor = a_cached.actorHandle.get();
				return !actor ||
					(actor.get() == a_actor &&
						retainedFaceIdentity != 0 &&
						a_cached.retainedFaceIdentity != retainedFaceIdentity);
			});
		for (const auto& cached : retainedHeadSkeletonCaches_) {
			const auto actor = cached.actorHandle.get();
			if (!actor ||
				actor.get() != a_actor ||
				cached.retainedFaceIdentity != retainedFaceIdentity) {
				continue;
			}
			for (const auto& boneName : cached.requiredHeadBoneNames) {
				MergePhysicsName(requiredHeadBoneNames, boneName);
			}
			for (const auto& reference : cached.headBoneReferences) {
				MergeBoneReferenceRecipe(headBoneReferences, reference);
			}
		}

		std::uint32_t retainedHeadRecords = 0;
		std::uint32_t clearedStates = 0;
		for (auto& actorStatePointer : systems_) {
			auto& actorState = *actorStatePointer;
			if (actorState.actor != a_actor || actorState.firstPerson) {
				continue;
			}

			retainedHeadRecords += static_cast<std::uint32_t>(actorState.headPartRecords.size());
			for (const auto& record : actorState.armorRecords) {
				MergeArmorPhysicsRecord(armorRecords, record);
			}
			for (const auto& record : actorState.headPartRecords) {
				for (const auto& boneName : record.requiredBoneNames) {
					MergePhysicsName(requiredHeadBoneNames, boneName);
				}
				for (const auto& reference : record.boneReferences) {
					MergeBoneReferenceRecipe(headBoneReferences, reference);
				}
			}

			ClearSystemLocked(actorState);
			++clearedStates;
		}
		std::erase_if(systems_, [a_actor](const auto& a_state) {
			return a_state->actor == a_actor &&
				!a_state->firstPerson &&
				!a_state->suspended &&
				!a_state->HasPhysics() &&
				a_state->armorRecords.empty();
		});

		for (const auto& candidate : suspendedActors_) {
			if (!candidate.firstPerson && resolvesActor(candidate.actorHandle)) {
				for (const auto& record : candidate.armorRecords) {
					MergeArmorPhysicsRecord(armorRecords, record);
				}
			}
		}
		const auto discardedSuspended = std::erase_if(
			suspendedActors_,
			[a_actor](const SuspendedActorCandidate& a_candidate) {
				const auto actor = a_candidate.actorHandle.get();
				return actor && actor.get() == a_actor && !a_candidate.firstPerson;
			});
		for (const auto& pending : pendingActorRebuilds_) {
			if (!pending.firstPerson && resolvesActor(pending.actorHandle)) {
				for (const auto& record : pending.armorRecords) {
					MergeArmorPhysicsRecord(armorRecords, record);
				}
			}
		}
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
		if (retainedFace && !headBoneReferences.empty()) {
			const auto cached = std::ranges::find_if(
				retainedHeadSkeletonCaches_,
				[&](const RetainedHeadSkeletonCache& a_cached) {
					const auto actor = a_cached.actorHandle.get();
					return actor &&
						actor.get() == a_actor &&
						a_cached.retainedFaceIdentity ==
							reinterpret_cast<std::uintptr_t>(retainedFace.get());
				});
			if (cached != retainedHeadSkeletonCaches_.end()) {
				cached->headBoneReferences = headBoneReferences;
				cached->requiredHeadBoneNames = requiredHeadBoneNames;
			} else {
				retainedHeadSkeletonCaches_.push_back({
					.actorHandle = actorHandle,
					.retainedFaceIdentity = reinterpret_cast<std::uintptr_t>(retainedFace.get()),
					.headBoneReferences = headBoneReferences,
					.requiredHeadBoneNames = requiredHeadBoneNames,
				});
			}
		}
		for (const auto& pending : pendingSkeletonTransitions_) {
			if (resolvesActor(pending.actorHandle)) {
				for (const auto& record : pending.armorRecords) {
					MergeArmorPhysicsRecord(armorRecords, record);
				}
			}
		}
		const auto supersededTransitions = std::erase_if(
			pendingSkeletonTransitions_,
			[a_actor](const PendingSkeletonTransition& a_pending) {
				const auto actor = a_pending.actorHandle.get();
				return actor && actor.get() == a_actor;
			});

		pendingSkeletonTransitions_.push_back({
			.actorHandle = actorHandle,
			.oldRoot = a_oldRoot,
			.retainedFace = retainedFace,
			.armorRecords = std::move(armorRecords),
			.headBoneReferences = std::move(headBoneReferences),
			.requiredHeadBoneNames = std::move(requiredHeadBoneNames),
			.retainedSkinBindings = std::move(retainedSkinBindings),
			.faceSkinned = !retainedFace,
		});

		ResetStepClockLocked();
		spdlog::debug(
			"began actor skeleton transition actor={} oldRoot={} retainedFace={} clearedStates={} retainedArmorRecords={} retainedHeadRecords={} retainedBoneRecipes={} requiredHeadBones={} retainedSkinInstances={} discardedSuspended={} discardedActorRebuilds={} discardedHeadRebuilds={} supersededTransitions={}",
			static_cast<void*>(a_actor),
			static_cast<void*>(a_oldRoot),
			static_cast<void*>(retainedFace.get()),
			clearedStates,
			pendingSkeletonTransitions_.back().armorRecords.size(),
			retainedHeadRecords,
			pendingSkeletonTransitions_.back().headBoneReferences.size(),
			pendingSkeletonTransitions_.back().requiredHeadBoneNames.size(),
			pendingSkeletonTransitions_.back().retainedSkinBindings.size(),
			discardedSuspended,
			discardedActorRebuilds,
			discardedHeadRebuilds,
			supersededTransitions);
		return true;
	}

	void Fo4PhysicsWorld::NoteActorSkeletonLoaded(RE::Actor* a_actor, RE::NiAVObject* a_newRoot)
	{
		if (!a_actor || !a_newRoot) {
			return;
		}

		std::scoped_lock lock(lock_);
		for (auto& pending : pendingSkeletonTransitions_) {
			const auto actor = pending.actorHandle.get();
			if (actor && actor.get() == a_actor) {
				pending.newRoot = a_newRoot;
				pending.skeletonRoot = NiObject::FindFlattenedBoneTree(a_newRoot);
				pending.loadedFrameAge = 0;
				if (pending.skeletonRoot) {
					spdlog::debug(
						"associated actor skeleton transition with loaded root actor={} oldRoot={} newRoot={} skeletonRoot={} retainedFace={}",
						static_cast<void*>(a_actor),
						static_cast<void*>(pending.oldRoot.get()),
						static_cast<void*>(a_newRoot),
						static_cast<void*>(pending.skeletonRoot.get()),
						static_cast<void*>(pending.retainedFace.get()));
				} else {
					spdlog::error(
						"loaded actor skeleton transition has no BSFlattenedBoneTree actor={} oldRoot={} newRoot={} retainedFace={}",
						static_cast<void*>(a_actor),
						static_cast<void*>(pending.oldRoot.get()),
						static_cast<void*>(a_newRoot),
						static_cast<void*>(pending.retainedFace.get()));
				}
				return;
			}
		}
	}

	RE::Actor* Fo4PhysicsWorld::PrepareRetainedFaceForSkeleton(
		RE::BSFaceGenNiNode* a_faceNode,
		RE::NiNode* a_skeleton)
	{
		if (!a_faceNode || !a_skeleton) {
			return nullptr;
		}

		WaitForAsyncStep();
		std::scoped_lock lock(lock_);
		for (auto& pending : pendingSkeletonTransitions_) {
			if (pending.retainedFace.get() != reinterpret_cast<RE::NiAVObject*>(a_faceNode)) {
				continue;
			}

			const auto actor = pending.actorHandle.get();
			if (!actor) {
				continue;
			}
			auto* currentRoot = actor->Get3D(false);
			if (!pending.newRoot ||
				!pending.skeletonRoot ||
				currentRoot != pending.newRoot.get() ||
				a_skeleton != pending.newRoot.get()) {
				spdlog::warn(
					"ignored retained-face bone materialization for mismatched transition parameters actor={} currentRoot={} transitionRoot={} skeletonRoot={} skeleton={} faceNode={}",
					static_cast<void*>(actor.get()),
					static_cast<void*>(currentRoot),
					static_cast<void*>(pending.newRoot.get()),
					static_cast<void*>(pending.skeletonRoot.get()),
					static_cast<void*>(a_skeleton),
					static_cast<void*>(a_faceNode));
				return nullptr;
			}
			auto* root = pending.skeletonRoot.get();

			const auto createdBones = MaterializeRetainedHeadPartBones(
				actor.get(),
				a_faceNode,
				root,
				pending.headBoneReferences,
				pending.requiredHeadBoneNames);
			const auto preRebindResult = RebindRetainedSkinBindings(
				root,
				pending.retainedSkinBindings,
				pending.oldRoot.get());
			const auto unresolvedRequiredPhysicsBones = CountUnresolvedRequiredPhysicsBones(
				root,
				pending.requiredHeadBoneNames);
			pending.physicsReady = unresolvedRequiredPhysicsBones == 0;
			if (preRebindResult.unresolvedSlots > 0 ||
				preRebindResult.boneSizeMismatches > 0 ||
				preRebindResult.transformSizeMismatches > 0) {
				spdlog::warn(
					"prepared retained face with unresolved pre-vanilla bindings actor={} faceNode={} root={} skeleton={} createdBones={} recipes={} requiredBones={} unresolvedRequiredPhysicsBones={} physicsReady={} instances={} boneSlots={} reboundSlots={} unresolvedSlots={} unresolvedOldRootSlots={} unnamedSlots={} boneSizeMismatches={} transformSizeMismatches={}",
					static_cast<void*>(actor.get()),
					static_cast<void*>(a_faceNode),
					static_cast<void*>(root),
					static_cast<void*>(a_skeleton),
					createdBones,
					pending.headBoneReferences.size(),
					pending.requiredHeadBoneNames.size(),
					unresolvedRequiredPhysicsBones,
					pending.physicsReady,
					preRebindResult.instances,
					preRebindResult.boneSlots,
					preRebindResult.reboundSlots,
					preRebindResult.unresolvedSlots,
					preRebindResult.unresolvedOldRootSlots,
					preRebindResult.unnamedSlots,
					preRebindResult.boneSizeMismatches,
					preRebindResult.transformSizeMismatches);
			} else {
				spdlog::debug(
					"prepared retained face for replacement skeleton actor={} faceNode={} root={} skeleton={} createdBones={} recipes={} requiredBones={} unresolvedRequiredPhysicsBones={} physicsReady={} instances={} boneSlots={} reboundSlots={}",
					static_cast<void*>(actor.get()),
					static_cast<void*>(a_faceNode),
					static_cast<void*>(root),
					static_cast<void*>(a_skeleton),
					createdBones,
					pending.headBoneReferences.size(),
					pending.requiredHeadBoneNames.size(),
					unresolvedRequiredPhysicsBones,
					pending.physicsReady,
					preRebindResult.instances,
					preRebindResult.boneSlots,
					preRebindResult.reboundSlots);
			}
			return actor.get();
		}
		return nullptr;
	}

	void Fo4PhysicsWorld::CompleteRetainedFaceSkinning(
		RE::Actor* a_actor,
		RE::BSFaceGenNiNode* a_faceNode,
		RE::NiNode* a_skeleton)
	{
		if (!a_actor || !a_faceNode || !a_skeleton) {
			return;
		}

		std::scoped_lock lock(lock_);
		for (auto& pending : pendingSkeletonTransitions_) {
			const auto actor = pending.actorHandle.get();
			if (!actor ||
				actor.get() != a_actor ||
				pending.retainedFace.get() != reinterpret_cast<RE::NiAVObject*>(a_faceNode) ||
				!pending.newRoot ||
				!pending.skeletonRoot) {
				continue;
			}
			auto* currentRoot = actor->Get3D(false);
			if (currentRoot != pending.newRoot.get() ||
				a_skeleton != pending.newRoot.get()) {
				spdlog::warn(
					"could not complete retained-face skinning because transition parameters changed actor={} currentRoot={} transitionRoot={} skeletonRoot={} skeleton={} faceNode={}",
					static_cast<void*>(a_actor),
					static_cast<void*>(currentRoot),
					static_cast<void*>(pending.newRoot.get()),
					static_cast<void*>(pending.skeletonRoot.get()),
					static_cast<void*>(a_skeleton),
					static_cast<void*>(a_faceNode));
				continue;
			}
			auto* root = pending.skeletonRoot.get();

			const auto rebindResult = RebindRetainedSkinBindings(
				root,
				pending.retainedSkinBindings,
				pending.oldRoot.get());
			pending.faceSkinned =
				rebindResult.unresolvedOldRootSlots == 0 &&
				rebindResult.boneSizeMismatches == 0 &&
				rebindResult.transformSizeMismatches == 0;
			pending.physicsReady =
				CountUnresolvedRequiredPhysicsBones(root, pending.requiredHeadBoneNames) == 0;
			if (rebindResult.unresolvedSlots > 0 ||
				rebindResult.boneSizeMismatches > 0 ||
				rebindResult.transformSizeMismatches > 0) {
				spdlog::warn(
					"completed retained-face skinning with unresolved bindings actor={} faceNode={} root={} skeleton={} physicsReady={} instances={} boneSlots={} reboundSlots={} unresolvedSlots={} unresolvedOldRootSlots={} unnamedSlots={} boneSizeMismatches={} transformSizeMismatches={}",
					static_cast<void*>(a_actor),
					static_cast<void*>(a_faceNode),
					static_cast<void*>(root),
					static_cast<void*>(a_skeleton),
					pending.physicsReady,
					rebindResult.instances,
					rebindResult.boneSlots,
					rebindResult.reboundSlots,
					rebindResult.unresolvedSlots,
					rebindResult.unresolvedOldRootSlots,
					rebindResult.unnamedSlots,
					rebindResult.boneSizeMismatches,
					rebindResult.transformSizeMismatches);
			} else {
				spdlog::debug(
					"completed retained-face skinning on replacement skeleton actor={} faceNode={} root={} skeleton={} instances={} boneSlots={} reboundSlots={}",
					static_cast<void*>(a_actor),
					static_cast<void*>(a_faceNode),
					static_cast<void*>(root),
					static_cast<void*>(a_skeleton),
					rebindResult.instances,
					rebindResult.boneSlots,
					rebindResult.reboundSlots);
			}
			return;
		}
	}

	void Fo4PhysicsWorld::CompletePendingSkeletonTransitions()
	{
		std::scoped_lock lock(lock_);
		std::erase_if(
			retainedHeadSkeletonCaches_,
			[](const RetainedHeadSkeletonCache& a_cached) {
				return !a_cached.actorHandle.get();
			});
		for (auto it = pendingSkeletonTransitions_.begin(); it != pendingSkeletonTransitions_.end();) {
			++it->frameAge;
			if (it->newRoot) {
				++it->loadedFrameAge;
			}

			const auto actor = it->actorHandle.get();
			if (!actor) {
				it = pendingSkeletonTransitions_.erase(it);
				continue;
			}

			auto* currentRoot = actor->Get3D(false);
			if (!it->newRoot || !currentRoot) {
				if (it->frameAge < kSkeletonTransitionLoadTimeoutFrames) {
					++it;
					continue;
				}
				spdlog::debug(
					"expired unloaded actor skeleton transition actor={} oldRoot={} newRoot={} frameAge={}",
					static_cast<void*>(actor.get()),
					static_cast<void*>(it->oldRoot.get()),
					static_cast<void*>(it->newRoot.get()),
					it->frameAge);
				it = pendingSkeletonTransitions_.erase(it);
				continue;
			}
			if (currentRoot != it->newRoot.get()) {
				spdlog::debug(
					"discarded superseded actor skeleton transition actor={} transitionRoot={} currentRoot={}",
					static_cast<void*>(actor.get()),
					static_cast<void*>(it->newRoot.get()),
					static_cast<void*>(currentRoot));
				it = pendingSkeletonTransitions_.erase(it);
				continue;
			}

			auto* currentFace = reinterpret_cast<RE::NiAVObject*>(actor->GetFaceNodeSkinned());
			if (it->retainedFace &&
				currentFace &&
				currentFace != it->retainedFace.get() &&
				IsObjectInTree(it->newRoot.get(), currentFace)) {
				spdlog::debug(
					"actor skeleton transition replaced rather than retained the face actor={} oldFace={} newFace={}",
					static_cast<void*>(actor.get()),
					static_cast<void*>(it->retainedFace.get()),
					static_cast<void*>(currentFace));
				const auto replacedFaceIdentity =
					reinterpret_cast<std::uintptr_t>(it->retainedFace.get());
				std::erase_if(
					retainedHeadSkeletonCaches_,
					[&](const RetainedHeadSkeletonCache& a_cached) {
						const auto cachedActor = a_cached.actorHandle.get();
						return cachedActor &&
							cachedActor.get() == actor.get() &&
							a_cached.retainedFaceIdentity == replacedFaceIdentity;
					});
				it->retainedFace = nullptr;
				it->faceSkinned = true;
			}

			const auto unresolvedRequiredPhysicsBones = CountUnresolvedRequiredPhysicsBones(
				it->skeletonRoot.get(),
				it->requiredHeadBoneNames);
			it->physicsReady = unresolvedRequiredPhysicsBones == 0;
			if (!it->physicsReady) {
				if (it->loadedFrameAge >= kRetainedFaceSkinningTimeoutFrames && !it->physicsTimeoutLogged) {
					it->physicsTimeoutLogged = true;
					spdlog::warn(
						"actor skeleton transition still lacks required physics bones actor={} newRoot={} loadedFrameAge={} unresolvedRequiredPhysicsBones={}; retaining old skeleton and waiting without clearing physics",
						static_cast<void*>(actor.get()),
						static_cast<void*>(it->newRoot.get()),
						it->loadedFrameAge,
						unresolvedRequiredPhysicsBones);
				}
				++it;
				continue;
			}

			if (it->retainedFace && !it->faceSkinned && it->skeletonRoot) {
				const auto retryResult = RebindRetainedSkinBindings(
					it->skeletonRoot.get(),
					it->retainedSkinBindings,
					it->oldRoot.get());
				it->faceSkinned =
					retryResult.unresolvedOldRootSlots == 0 &&
					retryResult.boneSizeMismatches == 0 &&
					retryResult.transformSizeMismatches == 0;
				if (it->faceSkinned) {
					spdlog::debug(
						"retained face no longer depends on old skeleton actor={} oldRoot={} newRoot={} unresolvedSlots={} unresolvedOldRootSlots={}",
						static_cast<void*>(actor.get()),
						static_cast<void*>(it->oldRoot.get()),
						static_cast<void*>(it->newRoot.get()),
						retryResult.unresolvedSlots,
						retryResult.unresolvedOldRootSlots);
				}
			}

			// Physics readiness is independent from complete retained-face migration.
			// Rebuild against the current skeleton as soon as every physics-required
			// bone exists, while retaining oldRoot for any remaining live skin slots.
			if (!it->rebuildsQueued) {
				const auto retainedArmorRecords = it->armorRecords.size();
				MarkPendingActorRebuildLocked(
					actor.get(),
					false,
					std::move(it->armorRecords),
					true,
					true,
					true);
				MarkPendingHeadRebuildLocked(LifecycleEvent{
					.type = LifecycleEventType::kActorHeadInitialized,
					.actor = actor.get(),
					.object = reinterpret_cast<RE::NiAVObject*>(actor->GetFaceNodeSkinned()),
					.firstPerson = false,
				});
				it->rebuildsQueued = true;
				ResetStepClockLocked();
				spdlog::debug(
					"released actor physics rebuild after replacement skeleton became physics-ready actor={} oldRoot={} newRoot={} retainedArmorRecords={} requiredPhysicsBones={} faceSkinned={} headReloadQueued=true",
					static_cast<void*>(actor.get()),
					static_cast<void*>(it->oldRoot.get()),
					static_cast<void*>(it->newRoot.get()),
					retainedArmorRecords,
					it->requiredHeadBoneNames.size(),
					it->faceSkinned);
			}

			const auto faceReady = !it->retainedFace || it->faceSkinned;
			if (!faceReady) {
				if (it->loadedFrameAge >= kRetainedFaceSkinningTimeoutFrames && !it->faceTimeoutLogged) {
					it->faceTimeoutLogged = true;
					spdlog::warn(
						"retained face still references old skeleton after migration timeout actor={} oldRoot={} newRoot={} retainedFace={} loadedFrameAge={}; physics remains active and old skeleton stays retained",
						static_cast<void*>(actor.get()),
						static_cast<void*>(it->oldRoot.get()),
						static_cast<void*>(it->newRoot.get()),
						static_cast<void*>(it->retainedFace.get()),
						it->loadedFrameAge);
				}
				++it;
				continue;
			}

			spdlog::debug(
				"completed actor skeleton transition actor={} oldRoot={} newRoot={} physicsReady=true faceReleased=true rebuildsQueued={}",
				static_cast<void*>(actor.get()),
				static_cast<void*>(it->oldRoot.get()),
				static_cast<void*>(it->newRoot.get()),
				it->rebuildsQueued);
			it = pendingSkeletonTransitions_.erase(it);
		}
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
		if (!a_event.actor || a_event.firstPerson) {
			return false;
		}

		auto* guard = FindFaceGenRebuildGuardLocked(a_event.actor);
		if (characterCustomizationMenuDepth_ == 0 && !guard) {
			return false;
		}

		auto* target = ResolveCharacterCustomizationTargetLocked();
		if (!target && characterCustomizationMenuDepth_ > 0) {
			NoteCharacterCustomizationTargetLocked(a_event.actor);
			target = a_event.actor;
			SuspendCharacterCustomizationTargetLocked();
			spdlog::debug(
				"using first LooksMenu lifecycle event as customization target fallback actor={} type={}",
				static_cast<void*>(a_event.actor),
				ToString(a_event.type));
		}

		if (target != a_event.actor && !guard) {
			return false;
		}

		auto& armorRecords = guard ? guard->armorRecords : characterCustomizationArmorRecords_;
		const auto bipedObject = ResolveEventBipedObject(a_event);
		if ((a_event.type == LifecycleEventType::kArmorApplySkinnedObjects ||
			 a_event.type == LifecycleEventType::kArmorAttachSkinnedObject) &&
			bipedObject != RE::BIPED_OBJECT::kTotal) {
			// ApplySkinnedObjects can replace the source root with a different live
			// attachment and does not carry arbitrary SMP XML extra data across. Treat
			// this as the authoritative owner change for the biped slot, then retain
			// the pre-attach recipe supplied by the hook when one exists.
			auto attachedRecords = CollectSuspendedArmorRecordsLocked(a_event);
			const auto sameRetainedOwner = a_event.object && std::ranges::any_of(armorRecords, [&](const ArmorPhysicsRecord& a_record) {
				return a_record.bipedObject == bipedObject &&
				       (a_record.attachedObject.get() == a_event.object || a_record.sourceObject.get() == a_event.object);
			});
			if (!attachedRecords.empty() || (a_event.object && !sameRetainedOwner)) {
				std::erase_if(armorRecords, [bipedObject](const ArmorPhysicsRecord& a_record) {
					return a_record.bipedObject == bipedObject;
				});
			}
			for (auto& record : attachedRecords) {
				MergeArmorPhysicsRecord(armorRecords, std::move(record));
			}
		} else if (a_event.type == LifecycleEventType::kArmorDetachEnd &&
			bipedObject != RE::BIPED_OBJECT::kTotal) {
			auto* liveBiped = a_event.actor->GetBiped(a_event.firstPerson).get();
			auto* liveBipObject = liveBiped ? liveBiped->GetBipObject(bipedObject) : nullptr;
			if (!liveBipObject || !liveBipObject->partClone) {
				std::erase_if(armorRecords, [bipedObject](const ArmorPhysicsRecord& a_record) {
					return a_record.bipedObject == bipedObject;
				});
			}
		}

		ResetStepClockLocked();
		spdlog::debug(
			"deferred target character customization physics build {} actor={} retainedArmorRecords={}",
			ToString(a_event.type),
			static_cast<void*>(a_event.actor),
			armorRecords.size());
		return true;
	}

	Fo4PhysicsWorld::FaceGenRebuildGuard* Fo4PhysicsWorld::FindFaceGenRebuildGuardLocked(RE::Actor* a_actor)
	{
		if (!a_actor) {
			return nullptr;
		}

		const auto found = std::ranges::find_if(faceGenRebuildGuards_, [a_actor](const FaceGenRebuildGuard& a_guard) {
			const auto actor = a_guard.actorHandle.get();
			return actor && actor.get() == a_actor;
		});
		return found != faceGenRebuildGuards_.end() ? std::addressof(*found) : nullptr;
	}

	void Fo4PhysicsWorld::ObserveGuardedHeadEventLocked(const LifecycleEvent& a_event)
	{
		auto* guard = FindFaceGenRebuildGuardLocked(a_event.actor);
		if (!guard) {
			return;
		}

		auto* faceNode = a_event.actor ? a_event.actor->GetFaceNodeSkinned() : nullptr;
		guard->latestFaceNode = reinterpret_cast<RE::NiAVObject*>(faceNode);
		guard->lastManagerObservation = faceGenManagerObservation_;
		guard->stableIdleObservations = 0;
		spdlog::trace(
			"absorbed FaceGen head replacement during actor-scoped rebuild guard actor={} faceNode={} type={}",
			static_cast<void*>(a_event.actor),
			static_cast<void*>(faceNode),
			ToString(a_event.type));
	}

	bool Fo4PhysicsWorld::IsActorFaceGenLoadPendingLocked(RE::Actor* a_actor) const
	{
		return a_actor && a_actor->currentProcess && a_actor->currentProcess->high &&
		       a_actor->currentProcess->high->faceGenLoadPending;
	}

	void Fo4PhysicsWorld::AdvanceFaceGenRebuildGuardsLocked()
	{
		for (auto it = faceGenRebuildGuards_.begin(); it != faceGenRebuildGuards_.end();) {
			auto resolvedActor = it->actorHandle.get();
			if (!resolvedActor) {
				it = faceGenRebuildGuards_.erase(it);
				continue;
			}

			auto* actor = resolvedActor.get();
			if (characterCustomizationMenuDepth_ > 0 && IsCharacterCustomizationTargetLocked(actor)) {
				it->stableIdleObservations = 0;
				it->lastManagerObservation = faceGenManagerObservation_;
				++it;
				continue;
			}

			auto* currentFaceNode = reinterpret_cast<RE::NiAVObject*>(actor->GetFaceNodeSkinned());
			if (!currentFaceNode || currentFaceNode != it->latestFaceNode.get()) {
				it->latestFaceNode = currentFaceNode;
				it->stableIdleObservations = 0;
				it->lastManagerObservation = faceGenManagerObservation_;
				++it;
				continue;
			}

			if (!faceGenManagerIdle_ || IsActorFaceGenLoadPendingLocked(actor)) {
				it->stableIdleObservations = 0;
				it->lastManagerObservation = faceGenManagerObservation_;
				++it;
				continue;
			}

			if (it->lastManagerObservation == faceGenManagerObservation_) {
				++it;
				continue;
			}

			it->lastManagerObservation = faceGenManagerObservation_;
			++it->stableIdleObservations;
			if (it->stableIdleObservations < kFaceGenStableIdleObservations) {
				++it;
				continue;
			}

			const auto retainedArmorRecordCount = it->armorRecords.size();
			MarkPendingActorRebuildLocked(actor, false, std::move(it->armorRecords), true, true, true);
			MarkPendingHeadRebuildLocked(LifecycleEvent{
				.type = LifecycleEventType::kActorHeadInitialized,
				.actor = actor,
				.object = currentFaceNode,
				.firstPerson = false,
			});
			ResetStepClockLocked();
			spdlog::debug(
				"released actor-scoped FaceGen rebuild guard actor={} faceNode={} idleObservations={} armorRecords={} forceArmorRescan=true headReloadQueued=true",
				static_cast<void*>(actor),
				static_cast<void*>(currentFaceNode),
				it->stableIdleObservations,
				retainedArmorRecordCount);
			if (IsCharacterCustomizationTargetLocked(actor)) {
				ClearCharacterCustomizationTargetLocked();
			}
			it = faceGenRebuildGuards_.erase(it);
		}
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
		std::uint32_t retainedArmorRecords = 0;
		for (auto& actorStatePointer : systems_) {
			auto& actorState = *actorStatePointer;
			if (actorState.actor != target || actorState.firstPerson) {
				continue;
			}
			if (actorState.HasPhysics() && !actorState.suspended) {
				SuspendSystemLocked(actorState);
				++suspendedStates;
			}
			for (auto& record : actorState.armorRecords) {
				MergeArmorPhysicsRecord(characterCustomizationArmorRecords_, record);
			}
			retainedArmorRecords += static_cast<std::uint32_t>(actorState.armorRecords.size());
			actorState.armorRecords.clear();
		}

		if (suspendedStates > 0 || retainedArmorRecords > 0 || discardedSuspendedCandidates > 0) {
			ResetStepClockLocked();
		}
		spdlog::debug(
			"suspended target system physics for character customization actor={} suspendedStates={} retainedArmorRecords={} discardedSuspendedCandidates={} trackedStates={}",
			static_cast<void*>(target),
			suspendedStates,
			retainedArmorRecords,
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

		std::vector<ArmorPhysicsRecord> armorRecords;
		for (auto& record : characterCustomizationArmorRecords_) {
			MergeArmorPhysicsRecord(armorRecords, std::move(record));
		}
		characterCustomizationArmorRecords_.clear();

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

			for (auto& record : actorState.armorRecords) {
				MergeArmorPhysicsRecord(armorRecords, record);
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

		const auto resolvesTarget = [target](const RE::ActorHandle& a_handle) {
			const auto actor = a_handle.get();
			return actor && actor.get() == target;
		};
		const auto discardedActorRebuilds = std::erase_if(
			pendingActorRebuilds_,
			[&](const PendingActorRebuild& a_pending) {
				return !a_pending.firstPerson && resolvesTarget(a_pending.actorHandle);
			});
		const auto discardedHeadRebuilds = std::erase_if(
			pendingHeadRebuilds_,
			[&](const PendingHeadRebuild& a_pending) {
				return resolvesTarget(a_pending.actorHandle);
			});

		auto* guard = FindFaceGenRebuildGuardLocked(target);
		if (!guard) {
			faceGenRebuildGuards_.push_back({
				.actorHandle = characterCustomizationTarget_,
			});
			guard = std::addressof(faceGenRebuildGuards_.back());
		}
		guard->latestFaceNode = reinterpret_cast<RE::NiAVObject*>(target->GetFaceNodeSkinned());
		for (auto& record : armorRecords) {
			MergeArmorPhysicsRecord(guard->armorRecords, std::move(record));
		}
		guard->lastManagerObservation = faceGenManagerObservation_;
		guard->stableIdleObservations = 0;
		ResetStepClockLocked();
		spdlog::debug(
			"entered actor-scoped FaceGen idle rebuild guard after character customization actor={} faceNode={} armorRecords={} clearedStates={} skippedFirstPerson={} discardedActorRebuilds={} discardedHeadRebuilds={}",
			static_cast<void*>(target),
			static_cast<void*>(guard->latestFaceNode.get()),
			guard->armorRecords.size(),
			clearedStates,
			skippedFirstPerson,
			discardedActorRebuilds,
			discardedHeadRebuilds);
	}

	void Fo4PhysicsWorld::ClearCharacterCustomizationTargetLocked()
	{
		characterCustomizationTarget_.reset();
		characterCustomizationArmorRecords_.clear();
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
		retainedHeadSkeletonCaches_.clear();
	}

	void Fo4PhysicsWorld::NoteSaveLoadActorLocked(const LifecycleEvent& a_event)
	{
		if (!a_event.actor) {
			return;
		}

		const auto actor3DReady =
			(a_event.type == LifecycleEventType::kActorLoad3D ||
			 a_event.type == LifecycleEventType::kActorSet3D) &&
			a_event.object;
		const auto headReady = !a_event.firstPerson && IsHeadCandidate(a_event.type);
		const auto armorAttach =
			!a_event.firstPerson &&
			IsArmorAttachCandidate(a_event.type) &&
			a_event.biped &&
			a_event.object &&
			!a_event.physicsXmlPath.empty();
		if (!actor3DReady && !headReady && !armorAttach) {
			return;
		}

		auto actorHandle = a_event.actor == RE::PlayerCharacter::GetSingleton() ?
			RE::PlayerCharacter::GetPlayerHandle() :
			RE::BSPointerHandleManagerInterface<RE::Actor>::GetHandle(a_event.actor);
		if (!actorHandle) {
			return;
		}

		auto found = std::ranges::find_if(
			saveLoadActors_,
			[&](const SaveLoadActorCandidate& a_candidate) {
				const auto actor = a_candidate.actorHandle.get();
				return actor &&
					actor.get() == a_event.actor &&
					a_candidate.firstPerson == a_event.firstPerson;
			});
		if (found == saveLoadActors_.end()) {
			saveLoadActors_.push_back({
				.actorHandle = actorHandle,
				.firstPerson = a_event.firstPerson,
			});
			found = std::prev(saveLoadActors_.end());
		}

		found->rebuildArmor = found->rebuildArmor || actor3DReady || armorAttach;
		found->rebuildHead = found->rebuildHead || actor3DReady || headReady;
		if (!armorAttach) {
			return;
		}

		auto armorRecords = CollectSuspendedArmorRecordsLocked(a_event);
		StripQueuedArmorRuntimePointers(armorRecords);
		for (auto& record : armorRecords) {
			const auto normalizedXml = ResourceFile::ComparisonKey(record.physicsXmlPath);
			auto existing = std::ranges::find_if(
				found->armorRecords,
				[&](const SaveLoadArmorRecord& a_existing) {
					return a_existing.bipedIdentity == a_event.biped &&
						a_existing.objectIdentity == a_event.object &&
						a_existing.record.bipedObject == record.bipedObject &&
						ResourceFile::ComparisonKey(a_existing.record.physicsXmlPath) == normalizedXml;
				});
			if (existing != found->armorRecords.end()) {
				existing->record = std::move(record);
				continue;
			}

			found->armorRecords.push_back({
				.bipedIdentity = a_event.biped,
				.objectIdentity = a_event.object,
				.record = std::move(record),
			});
		}
	}

	void Fo4PhysicsWorld::NoteFaceGenManagerUpdated(const bool a_idle)
	{
		WaitForAsyncStep();
		std::scoped_lock lock(lock_);
		faceGenManagerIdle_ = a_idle;
		++faceGenManagerObservation_;
		AdvanceFaceGenRebuildGuardsLocked();
	}

	void Fo4PhysicsWorld::ResumeFromLoadingMenuLocked()
	{
		if (saveLoadInProgress_) {
			std::size_t queuedArmorRebuilds = 0;
			std::size_t queuedHeadRebuilds = 0;
			std::size_t skippedUnreadyArmorActors = 0;
			std::size_t acceptedArmorRecords = 0;
			std::size_t rejectedArmorRecords = 0;
			for (const auto& candidate : saveLoadActors_) {
				const auto resolvedActor = candidate.actorHandle.get();
				if (!resolvedActor) {
					++skippedUnreadyArmorActors;
					continue;
				}

				auto* actor = resolvedActor.get();
				auto* root = actor->Get3D(candidate.firstPerson);
				auto* biped = actor->GetBiped(candidate.firstPerson).get();
				if (candidate.rebuildArmor && root && biped) {
					std::vector<ArmorPhysicsRecord> currentArmorRecords;
					for (const auto& captured : candidate.armorRecords) {
						if (captured.bipedIdentity != biped ||
							captured.record.bipedObject == RE::BIPED_OBJECT::kTotal) {
							++rejectedArmorRecords;
							continue;
						}
						auto* bipObject = biped->GetBipObject(captured.record.bipedObject);
						auto* partClone = bipObject ? bipObject->partClone.get() : nullptr;
						if (!partClone || partClone != captured.objectIdentity) {
							++rejectedArmorRecords;
							continue;
						}
						MergeArmorPhysicsRecord(currentArmorRecords, captured.record);
						++acceptedArmorRecords;
					}
					MarkPendingActorRebuildLocked(
						actor,
						candidate.firstPerson,
						std::move(currentArmorRecords),
						true,
						true,
						true);
					++queuedArmorRebuilds;
				} else if (candidate.rebuildArmor) {
					++skippedUnreadyArmorActors;
				}

				auto* faceNode = !candidate.firstPerson ? actor->GetFaceNodeSkinned() : nullptr;
				if (candidate.rebuildHead && root && faceNode) {
					MarkPendingHeadRebuildLocked(LifecycleEvent{
						.type = LifecycleEventType::kActorHeadInitialized,
						.actor = actor,
						.object = reinterpret_cast<RE::NiAVObject*>(faceNode),
						.firstPerson = false,
					});
					++queuedHeadRebuilds;
				}
			}

			saveLoadActors_.clear();
			saveLoadInProgress_ = false;
			loadingPhysicsSuspended_ = false;
			loadingMenuDepth_ = 0;
			if (dynamicsWorld_) {
				dynamicsWorld_->clearForces();
			}
			ResetStepClockLocked();
			spdlog::debug(
				"completed save-load physics transaction from live actor state armorRebuilds={} headRebuilds={} acceptedArmorRecords={} rejectedArmorRecords={} skippedUnreadyArmorActors={}",
				queuedArmorRebuilds,
				queuedHeadRebuilds,
				acceptedArmorRecords,
				rejectedArmorRecords,
				skippedUnreadyArmorActors);
			return;
		}

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
