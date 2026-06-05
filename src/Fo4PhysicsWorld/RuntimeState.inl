// Included by ../Fo4PhysicsWorld.cpp.
// Split from Fo4PhysicsWorld.cpp. Do not compile this file separately.

namespace Smp
{
	void Fo4PhysicsWorld::SuspendPrototypeStatesForCustomizationMenuLocked()
	{
		if (prototypeActors_.empty()) {
			return;
		}

		std::uint32_t suspendedStates = 0;
		for (auto& actorState : prototypeActors_) {
			if (!actorState.HasRuntime()) {
				continue;
			}
			SuspendPrototypeRuntimeLocked(actorState);
			++suspendedStates;
		}

		pendingActorRebuilds_.clear();
		pendingHeadRebuilds_.clear();
		suspendedActors_.clear();
		ResetStepClockLocked();
		spdlog::debug(
			"suspended {} prototype actor states for character customization menu; trackedStates={}",
			suspendedStates,
			prototypeActors_.size());
	}

	void Fo4PhysicsWorld::ReloadPrototypeStatesForCustomizationMenuLocked()
	{
		if (prototypeActors_.empty()) {
			return;
		}

		if (!InitializeLocked()) {
			return;
		}

		struct CustomizationReloadActor
		{
			RE::Actor* actor{ nullptr };
			std::vector<PrototypeArmorRecord> armorRecords;
		};

		std::vector<CustomizationReloadActor> actors;
		std::uint32_t clearedStates = 0;
		std::uint32_t skippedFirstPerson = 0;
		for (auto& actorState : prototypeActors_) {
			if (actorState.firstPerson) {
				++skippedFirstPerson;
				continue;
			}

			auto* actor = actorState.actor;
			if (!actor) {
				if (auto resolved = actorState.actorHandle.get()) {
					actor = resolved.get();
				}
			}
			if (!actor) {
				continue;
			}

			auto actorReload = std::ranges::find_if(actors, [actor](const CustomizationReloadActor& a_entry) {
				return a_entry.actor == actor;
			});
			if (actorReload == actors.end()) {
				actors.push_back({ .actor = actor });
				actorReload = std::prev(actors.end());
			}
			std::vector<PrototypeArmorRecord> stateArmorRecords;
			for (auto record : actorState.armorRecords) {
				record.buildGroups.clear();
				record.cpuCopyRetryCount = 0;
				MergePrototypeArmorRecord(actorReload->armorRecords, record);
				MergePrototypeArmorRecord(stateArmorRecords, std::move(record));
			}
			ClearPrototypeStateLocked(actorState);
			actorState.armorRecords = std::move(stateArmorRecords);
			actorState.faceNode = nullptr;
			++clearedStates;
		}

		std::erase_if(prototypeActors_, [](const PrototypeActorState& a_state) {
			return !a_state.runtimeSuspended && !a_state.HasRuntime() && a_state.armorRecords.empty();
		});
		for (auto& actorReload : actors) {
			const auto hairSlotArmorQueued = PrototypeArmorRecordsIncludeHairSlot(actorReload.armorRecords);
			MarkPendingActorRebuildLocked(actorReload.actor, false, std::move(actorReload.armorRecords), true, true, true);
			if (hairSlotArmorQueued) {
				spdlog::debug(
					"skipping customization head reload for actor={} because queued hair-slot armor owns the slot",
					static_cast<void*>(actorReload.actor));
			} else {
				MarkPendingHeadRebuildLocked(LifecycleEvent{
					.type = LifecycleEventType::kActorHeadInitialized,
					.actor = actorReload.actor,
					.object = actorReload.actor->GetFaceNodeSkinned() ? reinterpret_cast<RE::NiAVObject*>(actorReload.actor->GetFaceNodeSkinned()) : nullptr,
					.firstPerson = false,
				});
			}
		}
		ResetStepClockLocked();
		spdlog::debug(
			"queued full prototype physics reload after character customization; actors={} clearedStates={} skippedFirstPerson={}",
			actors.size(),
			clearedStates,
			skippedFirstPerson);
	}

	void Fo4PhysicsWorld::SuspendPrototypeRuntimeLocked(PrototypeActorState& a_state)
	{
		if (dispatcher_) {
			dispatcher_->clearAllManifold();
		}

		if (dynamicsWorld_) {
			for (auto& prototypeConstraint : a_state.constraints) {
				if (prototypeConstraint.constraint && prototypeConstraint.inBulletWorld) {
					dynamicsWorld_->removeConstraint(prototypeConstraint.constraint.get());
					prototypeConstraint.inBulletWorld = false;
				}
			}

			for (auto& prototypeMesh : a_state.meshes) {
				if (prototypeMesh.body && prototypeMesh.inBulletWorld) {
					dynamicsWorld_->removeCollisionObject(prototypeMesh.body.get());
					prototypeMesh.inBulletWorld = false;
				}
			}

			for (auto& prototypeBody : a_state.bodies) {
				if (prototypeBody.bone && prototypeBody.inBulletWorld) {
					dynamicsWorld_->removeRigidBody(std::addressof(prototypeBody.bone->m_rig));
					prototypeBody.inBulletWorld = false;
				}
			}
		}

		const auto meshCount = a_state.meshes.size();
		const auto constraintCount = a_state.constraints.size();
		const auto bodyCount = a_state.bodies.size();
		std::uint32_t capturedSkinSlots = 0;
		for (auto& prototypeBody : a_state.bodies) {
			if (!prototypeBody.bone) {
				continue;
			}
			const auto before = a_state.suspendedSkinSlots.size();
			prototypeBody.bone->CollectSkinWorldTransformRestoreSlots(a_state.suspendedSkinSlots);
			capturedSkinSlots += static_cast<std::uint32_t>(a_state.suspendedSkinSlots.size() - before);
		}
		a_state.meshes.clear();
		a_state.constraints.clear();
		a_state.bodies.clear();
		a_state.runtimes.clear();
		a_state.lastWritebackFrame = 0;
		a_state.lastWritebackSource = WritebackSource::kUnknown;
		a_state.runtimeSuspended = true;
		a_state.runtimeSoftSuspended = false;
		spdlog::debug(
			"suspended prototype runtime for actor={} bodies={} meshes={} constraints={} capturedSkinSlots={} preservedMergedNodes={} armorRecords={}",
			static_cast<void*>(a_state.actor),
			bodyCount,
			meshCount,
			constraintCount,
			capturedSkinSlots,
			a_state.mergedNodes.size(),
			a_state.armorRecords.size());
	}

	void Fo4PhysicsWorld::SoftSuspendPrototypeRuntimeLocked(PrototypeActorState& a_state)
	{
		if (!a_state.HasRuntime() || a_state.runtimeSoftSuspended) {
			return;
		}

		std::vector<std::uint64_t> buildGroups;
		for (const auto& runtime : a_state.runtimes) {
			if (runtime.buildGroup != 0 && std::ranges::find(buildGroups, runtime.buildGroup) == buildGroups.end()) {
				buildGroups.push_back(runtime.buildGroup);
			}
		}
		ResetPrototypeBuildGroupsToStoredLocalPoseLocked(a_state, buildGroups, "soft-suspend");

		std::uint32_t removedConstraints = 0;
		std::uint32_t removedMeshes = 0;
		std::uint32_t removedBodies = 0;
		if (dynamicsWorld_) {
			for (auto& prototypeConstraint : a_state.constraints) {
				if (prototypeConstraint.constraint && prototypeConstraint.inBulletWorld) {
					dynamicsWorld_->removeConstraint(prototypeConstraint.constraint.get());
					prototypeConstraint.inBulletWorld = false;
					++removedConstraints;
				}
			}

			for (auto& prototypeMesh : a_state.meshes) {
				if (prototypeMesh.body && prototypeMesh.inBulletWorld) {
					dynamicsWorld_->removeCollisionObject(prototypeMesh.body.get());
					prototypeMesh.inBulletWorld = false;
					++removedMeshes;
				}
			}

			for (auto& prototypeBody : a_state.bodies) {
				if (prototypeBody.bone && prototypeBody.inBulletWorld) {
					dynamicsWorld_->removeRigidBody(std::addressof(prototypeBody.bone->m_rig));
					prototypeBody.inBulletWorld = false;
					++removedBodies;
				}
			}
		}

		a_state.lastWritebackFrame = 0;
		a_state.lastWritebackSource = WritebackSource::kUnknown;
		a_state.currentWindFactor = 0.0F;
		a_state.runtimeSuspended = false;
		a_state.runtimeSoftSuspended = true;
		spdlog::debug(
			"soft-suspended prototype runtime for actor={} buildGroups={} removedBodies={} removedMeshes={} removedConstraints={} retainedBodies={} retainedMeshes={} retainedConstraints={} runtimes={} armorRecords={}",
			static_cast<void*>(a_state.actor),
			buildGroups.size(),
			removedBodies,
			removedMeshes,
			removedConstraints,
			a_state.bodies.size(),
			a_state.meshes.size(),
			a_state.constraints.size(),
			a_state.runtimes.size(),
			a_state.armorRecords.size());
	}

	bool Fo4PhysicsWorld::ResumeSoftSuspendedPrototypeRuntimeLocked(PrototypeActorState& a_state)
	{
		if (!a_state.runtimeSoftSuspended || !dynamicsWorld_) {
			return false;
		}

		std::vector<std::uint64_t> buildGroups;
		for (const auto& runtime : a_state.runtimes) {
			if (runtime.buildGroup != 0 && std::ranges::find(buildGroups, runtime.buildGroup) == buildGroups.end()) {
				buildGroups.push_back(runtime.buildGroup);
			}
		}
		if (buildGroups.empty()) {
			return false;
		}

		for (const auto buildGroup : buildGroups) {
			CommitPrototypeBuildGroupToBulletLocked(a_state, buildGroup);
		}

		const auto hasResumedObject = [&a_state, &buildGroups](const auto& a_collection) {
			return std::ranges::any_of(a_collection, [&buildGroups](const auto& a_object) {
				if (!a_object.inBulletWorld) {
					return false;
				}
				if constexpr (requires { a_object.buildGroup; }) {
					return std::ranges::find(buildGroups, a_object.buildGroup) != buildGroups.end();
				} else {
					return std::ranges::any_of(a_object.buildGroups, [&buildGroups](const std::uint64_t a_buildGroup) {
						return std::ranges::find(buildGroups, a_buildGroup) != buildGroups.end();
					});
				}
			});
		};
		if (!hasResumedObject(a_state.bodies) && !hasResumedObject(a_state.meshes) && !hasResumedObject(a_state.constraints)) {
			return false;
		}

		a_state.runtimeSoftSuspended = false;
		a_state.runtimeSuspended = false;
		a_state.lastWritebackFrame = 0;
		a_state.lastWritebackSource = WritebackSource::kUnknown;
		a_state.currentWindFactor = 1.0F;
		spdlog::debug(
			"resumed soft-suspended prototype runtime for actor={} buildGroups={} bodies={} meshes={} constraints={}",
			static_cast<void*>(a_state.actor),
			buildGroups.size(),
			a_state.bodies.size(),
			a_state.meshes.size(),
			a_state.constraints.size());
		return true;
	}

	std::uint32_t Fo4PhysicsWorld::RestoreSuspendedSkinSlotsLocked(
		PrototypeActorState& a_state,
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
					slot.skin->bones[slot.index] = slot.originalBone.get();
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
				slot.skin->rootNode != slot.originalRootNode.get()) {
				slot.skin->rootNode = slot.originalRootNode.get();
				++restored;
			}
		}

		const auto erased = std::erase_if(a_state.suspendedSkinSlots, [&containsGroup](const Fo4SkinnedMeshBone::SkinSlotRestore& a_slot) {
			return containsGroup(a_slot.buildGroup);
		});
		if (restored > 0 || erased > 0) {
			spdlog::debug(
				"restored {} suspended prototype skin slot fields and erased {} cached slots for actor={}",
				restored,
				erased,
				static_cast<void*>(a_state.actor));
		}
		return restored;
	}

	std::uint32_t Fo4PhysicsWorld::RestoreAllSuspendedSkinSlotsLocked(PrototypeActorState& a_state)
	{
		std::vector<std::uint64_t> buildGroups;
		for (const auto& slot : a_state.suspendedSkinSlots) {
			if (slot.buildGroup != 0 && std::ranges::find(buildGroups, slot.buildGroup) == buildGroups.end()) {
				buildGroups.push_back(slot.buildGroup);
			}
		}
		return RestoreSuspendedSkinSlotsLocked(a_state, buildGroups);
	}

	void Fo4PhysicsWorld::ClearPrototypeStateLocked(PrototypeActorState& a_state, const bool a_restoreSkinSlots)
	{
		if (dispatcher_) {
			dispatcher_->clearAllManifold();
		}

		if (dynamicsWorld_) {
			for (auto& prototypeConstraint : a_state.constraints) {
				if (prototypeConstraint.constraint && prototypeConstraint.inBulletWorld) {
					dynamicsWorld_->removeConstraint(prototypeConstraint.constraint.get());
					prototypeConstraint.inBulletWorld = false;
				}
			}

			for (auto& prototypeMesh : a_state.meshes) {
				if (prototypeMesh.body && prototypeMesh.inBulletWorld) {
					dynamicsWorld_->removeCollisionObject(prototypeMesh.body.get());
					prototypeMesh.inBulletWorld = false;
				}
			}
		}
		if (!a_state.meshes.empty()) {
			spdlog::debug("cleared {} prototype physics mesh bodies for actor={}", a_state.meshes.size(), static_cast<void*>(a_state.actor));
		}
		a_state.meshes.clear();

		if (!a_state.constraints.empty()) {
			spdlog::debug("cleared {} prototype physics constraints for actor={}", a_state.constraints.size(), static_cast<void*>(a_state.actor));
		}
		a_state.constraints.clear();
		a_state.runtimes.clear();

		if (dynamicsWorld_) {
			for (auto& prototypeBody : a_state.bodies) {
				if (prototypeBody.bone && prototypeBody.inBulletWorld) {
					dynamicsWorld_->removeRigidBody(std::addressof(prototypeBody.bone->m_rig));
					prototypeBody.inBulletWorld = false;
				}
			}
		}

		if (a_restoreSkinSlots) {
			for (auto& prototypeBody : a_state.bodies) {
				if (!prototypeBody.bone) {
					continue;
				}
				for (const auto buildGroup : prototypeBody.buildGroups) {
					prototypeBody.bone->RemoveSkinWorldTransformsForBuildGroup(buildGroup);
				}
				if (prototypeBody.buildGroup != 0) {
					prototypeBody.bone->RemoveSkinWorldTransformsForBuildGroup(prototypeBody.buildGroup);
				}
			}
			RestoreAllSuspendedSkinSlotsLocked(a_state);
		} else if (!a_state.bodies.empty()) {
			spdlog::debug(
				"skipped restoring prototype skin slots while clearing actor={} for model rebuild",
				static_cast<void*>(a_state.actor));
			a_state.suspendedSkinSlots.clear();
		}

		if (!a_state.bodies.empty()) {
			spdlog::debug("cleared {} prototype physics bodies for actor={}", a_state.bodies.size(), static_cast<void*>(a_state.actor));
		}
		a_state.bodies.clear();
		for (auto& mergedNode : a_state.mergedNodes) {
			if (mergedNode.parent && mergedNode.node) {
				mergedNode.parent->DetachChild(mergedNode.node.get());
			}
		}
		a_state.mergedNodes.clear();
		a_state.nextBuildGroup = 0;
		a_state.nextAttachmentGeneration = 0;
		a_state.lastReadRoot = nullptr;
		a_state.readInitialized = false;
		a_state.lastRootRotation = btQuaternion::getIdentity();
		a_state.lastRootRotationInitialized = false;
		a_state.lastWritebackFrame = 0;
		a_state.lastWritebackSource = WritebackSource::kUnknown;
		a_state.currentWindFactor = 1.0F;
		a_state.runtimeSuspended = false;
		a_state.runtimeSoftSuspended = false;
		a_state.faceNode = nullptr;
		a_state.attachmentRecords.clear();
		a_state.headPartRecords.clear();
		a_state.runtimes.clear();
		a_state.suspendedSkinSlots.clear();
	}

	std::vector<std::uint64_t> Fo4PhysicsWorld::CollectPrototypeGroupsForObjectLocked(const PrototypeActorState& a_state, RE::NiAVObject* a_object) const
	{
		std::vector<std::uint64_t> buildGroups;
		if (!a_object) {
			return buildGroups;
		}
		if (a_state.actor) {
			auto* primaryRoot = a_state.actor->Get3D(a_state.firstPerson);
			auto* thirdPersonRoot = a_state.actor->Get3D(false);
			auto* firstPersonRoot = a_state.actor->Get3D(true);
			if (a_object == primaryRoot || a_object == thirdPersonRoot || a_object == firstPersonRoot || a_object == a_state.faceNode.get()) {
				spdlog::debug(
					"refusing object-scoped prototype clear from broad actor object={} actor={}; waiting for attachment/biped scoped clear",
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
				if (buildGroup != 0 && std::ranges::find(buildGroups, buildGroup) == buildGroups.end()) {
					buildGroups.push_back(buildGroup);
				}
			}
		}
		if (!buildGroups.empty()) {
			return buildGroups;
		}

		for (const auto& prototypeMesh : a_state.meshes) {
			if (prototypeMesh.buildGroup == 0 || !prototypeMesh.geometry || !IsObjectInTree(a_object, prototypeMesh.geometry)) {
				continue;
			}

			if (std::ranges::find(buildGroups, prototypeMesh.buildGroup) == buildGroups.end()) {
				buildGroups.push_back(prototypeMesh.buildGroup);
			}
		}

		for (const auto& prototypeBody : a_state.bodies) {
			if (prototypeBody.buildGroup == 0 || !prototypeBody.node || !IsNodeInTree(a_object, prototypeBody.node)) {
				continue;
			}

			if (std::ranges::find(buildGroups, prototypeBody.buildGroup) == buildGroups.end()) {
				buildGroups.push_back(prototypeBody.buildGroup);
			}
		}

		return buildGroups;
	}

	std::vector<std::uint64_t> Fo4PhysicsWorld::CollectArmorPrototypeGroupsForBipedObjectLocked(
		const PrototypeActorState& a_state,
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

		for (const auto& record : a_state.armorRecords) {
			if (record.bipedObject != a_bipedObject) {
				continue;
			}
			for (const auto buildGroup : record.buildGroups) {
				appendGroup(buildGroup);
			}
		}

		for (const auto& record : a_state.attachmentRecords) {
			if (record.bipedObject != a_bipedObject) {
				continue;
			}
			for (const auto buildGroup : record.buildGroups) {
				appendGroup(buildGroup);
			}
		}

		for (const auto& runtime : a_state.runtimes) {
			if (runtime.domain == PrototypeBuildDomain::kArmor && runtime.bipedObject == a_bipedObject) {
				appendGroup(runtime.buildGroup);
			}
		}

		for (const auto& prototypeMesh : a_state.meshes) {
			if (prototypeMesh.domain == PrototypeBuildDomain::kArmor && prototypeMesh.bipedObject == a_bipedObject) {
				appendGroup(prototypeMesh.buildGroup);
			}
		}

		for (const auto& prototypeBody : a_state.bodies) {
			const auto visitBodyBuildGroup = [&](const std::uint64_t a_buildGroup) {
				if (a_buildGroup == 0) {
					return;
				}

				auto domain = PrototypeBuildDomain::kArmor;
				if (const auto domainEntry = std::ranges::find_if(prototypeBody.buildGroupDomains, [a_buildGroup](const auto& a_entry) {
						return a_entry.first == a_buildGroup;
					});
					domainEntry != prototypeBody.buildGroupDomains.end()) {
					domain = domainEntry->second;
				}
				if (domain != PrototypeBuildDomain::kArmor) {
					return;
				}

				auto bipedObject = prototypeBody.bipedObject;
				if (const auto bipedEntry = std::ranges::find_if(prototypeBody.buildGroupBipedObjects, [a_buildGroup](const auto& a_entry) {
						return a_entry.first == a_buildGroup;
					});
					bipedEntry != prototypeBody.buildGroupBipedObjects.end()) {
					bipedObject = bipedEntry->second;
				}
				if (bipedObject == a_bipedObject) {
					appendGroup(a_buildGroup);
				}
			};

			if (!prototypeBody.buildGroups.empty()) {
				for (const auto buildGroup : prototypeBody.buildGroups) {
					visitBodyBuildGroup(buildGroup);
				}
			} else {
				visitBodyBuildGroup(prototypeBody.buildGroup);
			}
		}

		return buildGroups;
	}

	std::uint32_t Fo4PhysicsWorld::PrunePrototypeRecordsForBipedObjectLocked(
		PrototypeActorState& a_state,
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
		const auto removedAttachments = std::erase_if(a_state.attachmentRecords, [a_bipedObject, &hasPreservedGroup](const PrototypeAttachmentRecord& a_record) {
			return a_record.bipedObject == a_bipedObject && !hasPreservedGroup(a_record.buildGroups);
		});
		const auto removedArmorRecords = std::erase_if(a_state.armorRecords, [a_bipedObject, &hasPreservedGroup](const PrototypeArmorRecord& a_record) {
			return a_record.bipedObject == a_bipedObject && !hasPreservedGroup(a_record.buildGroups);
		});

		return static_cast<std::uint32_t>(removedAttachments + removedArmorRecords);
	}

	std::uint32_t Fo4PhysicsWorld::ClearStaleHairSlotArmorGroupsLocked(
		PrototypeActorState& a_state,
		const RE::BIPED_OBJECT a_bipedObject,
		const std::uint64_t a_preservedBuildGroup,
		const std::string_view a_reason,
		RE::NiAVObject* a_object,
		const std::string_view a_physicsXmlPath)
	{
		if (!IsHairBipedObject(a_bipedObject)) {
			return 0;
		}

		auto buildGroups = CollectArmorPrototypeGroupsForBipedObjectLocked(a_state, a_bipedObject, a_preservedBuildGroup);
		for (const auto buildGroup : buildGroups) {
			spdlog::debug(
				"clearing stale hair-slot armor prototype group actor={} bipedObject={} buildGroup={} preservedBuildGroup={} object={} xml='{}' reason={}",
				static_cast<void*>(a_state.actor),
				std::to_underlying(a_bipedObject),
				buildGroup,
				a_preservedBuildGroup,
				static_cast<void*>(a_object),
				a_physicsXmlPath,
				a_reason);
		}
		if (!buildGroups.empty()) {
			ClearPrototypeGroupsLocked(a_state, buildGroups, true);
		}

		const auto removedRecords = PrunePrototypeRecordsForBipedObjectLocked(a_state, a_bipedObject, a_preservedBuildGroup);
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
		const PrototypeActorState& a_state,
		std::vector<std::uint64_t>& a_buildGroups) const
	{
		std::uint32_t matchedRecords = 0;
		for (const auto& record : a_state.headPartRecords) {
			if (record.buildGroup == 0) {
				continue;
			}

			++matchedRecords;
			if (std::ranges::find(a_buildGroups, record.buildGroup) == a_buildGroups.end()) {
				a_buildGroups.push_back(record.buildGroup);
			}
		}
		return matchedRecords;
	}

	bool Fo4PhysicsWorld::HasActiveHairSlotArmorLocked(const PrototypeActorState& a_state) const
	{
		if (std::ranges::any_of(a_state.armorRecords, [&](const PrototypeArmorRecord& a_record) {
				return IsHairBipedObject(a_record.bipedObject) &&
					std::ranges::any_of(a_record.buildGroups, [&](const std::uint64_t a_buildGroup) {
						return PrototypeBuildGroupHasBodyLocked(a_state, a_buildGroup) || PrototypeBuildGroupHasMeshLocked(a_state, a_buildGroup);
					});
			})) {
			return true;
		}
		return std::ranges::any_of(a_state.runtimes, [](const PrototypeBuildGroupRuntime& a_runtime) {
			return a_runtime.domain == PrototypeBuildDomain::kArmor && IsHairBipedObject(a_runtime.bipedObject);
		});
	}

	bool Fo4PhysicsWorld::PrototypeBuildGroupsIncludeHairSlotArmorLocked(
		const PrototypeActorState& a_state,
		const std::span<const std::uint64_t> a_buildGroups) const
	{
		if (a_buildGroups.empty()) {
			return false;
		}

		const auto containsGroup = [&a_buildGroups](const std::uint64_t a_buildGroup) {
			return a_buildGroup != 0 && std::ranges::find(a_buildGroups, a_buildGroup) != a_buildGroups.end();
		};

		if (std::ranges::any_of(a_state.armorRecords, [&](const PrototypeArmorRecord& a_record) {
				return IsHairBipedObject(a_record.bipedObject) && std::ranges::any_of(a_record.buildGroups, containsGroup);
			})) {
			return true;
		}
		if (std::ranges::any_of(a_state.attachmentRecords, [&](const PrototypeAttachmentRecord& a_record) {
				return IsHairBipedObject(a_record.bipedObject) && std::ranges::any_of(a_record.buildGroups, containsGroup);
			})) {
			return true;
		}
		if (std::ranges::any_of(a_state.runtimes, [&](const PrototypeBuildGroupRuntime& a_runtime) {
				return containsGroup(a_runtime.buildGroup) && a_runtime.domain == PrototypeBuildDomain::kArmor && IsHairBipedObject(a_runtime.bipedObject);
			})) {
			return true;
		}
		if (std::ranges::any_of(a_state.meshes, [&](const PrototypeMesh& a_mesh) {
				return containsGroup(a_mesh.buildGroup) && a_mesh.domain == PrototypeBuildDomain::kArmor && IsHairBipedObject(a_mesh.bipedObject);
			})) {
			return true;
		}
		return std::ranges::any_of(a_state.bodies, [&](const PrototypeBody& a_body) {
			if (!a_body.buildGroupDomains.empty() || !a_body.buildGroupBipedObjects.empty()) {
				for (const auto& [buildGroup, domain] : a_body.buildGroupDomains) {
					if (containsGroup(buildGroup) && domain == PrototypeBuildDomain::kArmor) {
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

	bool Fo4PhysicsWorld::ClearPrototypeGroupsForObjectLocked(PrototypeActorState& a_state, RE::NiAVObject* a_object)
	{
		auto buildGroups = CollectPrototypeGroupsForObjectLocked(a_state, a_object);
		if (buildGroups.empty()) {
			return false;
		}

		ClearPrototypeGroupsLocked(a_state, buildGroups);
		return true;
	}

	bool Fo4PhysicsWorld::ClearPrototypeGroupsForBipedObjectLocked(PrototypeActorState& a_state, const RE::BIPED_OBJECT a_bipedObject)
	{
		if (a_bipedObject == RE::BIPED_OBJECT::kTotal) {
			return false;
		}

		const auto attachmentCount = static_cast<std::size_t>(std::ranges::count_if(a_state.attachmentRecords, [a_bipedObject](const PrototypeAttachmentRecord& a_record) {
			return a_record.bipedObject == a_bipedObject && !a_record.buildGroups.empty();
		}));
		const auto armorRecordCount = static_cast<std::size_t>(std::ranges::count_if(a_state.armorRecords, [a_bipedObject](const PrototypeArmorRecord& a_record) {
			return a_record.bipedObject == a_bipedObject;
		}));
		if (attachmentCount > 1 || armorRecordCount > 1) {
			spdlog::warn(
				"refusing biped-wide prototype clear actor={} bipedObject={} attachments={} armorRecords={} because multiple same-slot systems are tracked",
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

		for (const auto& runtime : a_state.runtimes) {
			if (runtime.bipedObject == a_bipedObject) {
				appendGroup(runtime.buildGroup);
			}
		}

		for (const auto& prototypeMesh : a_state.meshes) {
			if (prototypeMesh.bipedObject == a_bipedObject) {
				appendGroup(prototypeMesh.buildGroup);
			}
		}

		for (const auto& prototypeBody : a_state.bodies) {
			if (!prototypeBody.buildGroupBipedObjects.empty()) {
				for (const auto& [buildGroup, bipedObject] : prototypeBody.buildGroupBipedObjects) {
					if (bipedObject == a_bipedObject) {
						appendGroup(buildGroup);
					}
				}
			} else if (prototypeBody.bipedObject == a_bipedObject) {
				appendGroup(prototypeBody.buildGroup);
			}
		}

		if (buildGroups.empty()) {
			return false;
		}

		ClearPrototypeGroupsLocked(a_state, buildGroups);
		std::erase_if(a_state.armorRecords, [a_bipedObject](const PrototypeArmorRecord& a_record) {
			return a_record.bipedObject == a_bipedObject;
		});
		return true;
	}

	bool Fo4PhysicsWorld::ClearPrototypeGroupsForBoneNamesLocked(PrototypeActorState& a_state, const std::span<const std::string> a_boneNames, const PrototypeBuildDomain a_domain)
	{
		if (a_boneNames.empty()) {
			return false;
		}
		if (a_domain == PrototypeBuildDomain::kArmor) {
			spdlog::warn(
				"refusing to clear armor prototype groups by bone names actor={} names={} because actor skeleton bones may be shared across armor XMLs",
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
		for (const auto& prototypeBody : a_state.bodies) {
			if (prototypeBody.boneName.empty()) {
				continue;
			}
			const auto nameMatched = std::ranges::any_of(a_boneNames, [&prototypeBody](const std::string& a_boneName) {
				return PhysicsNamesEqual(prototypeBody.boneName, a_boneName);
			});
			if (!nameMatched) {
				continue;
			}

			if (!prototypeBody.buildGroupDomains.empty()) {
				for (const auto& [buildGroup, domain] : prototypeBody.buildGroupDomains) {
					if (domain == a_domain) {
						appendGroup(buildGroup);
					}
				}
			} else if (prototypeBody.buildGroup != 0) {
				appendGroup(prototypeBody.buildGroup);
			}
		}

		if (buildGroups.empty()) {
			return false;
		}

		ClearPrototypeGroupsLocked(a_state, buildGroups);
		return true;
	}

	bool Fo4PhysicsWorld::ClearPrototypeGroupsByDomainLocked(PrototypeActorState& a_state, const PrototypeBuildDomain a_domain)
	{
		std::vector<std::uint64_t> buildGroups;
		for (const auto& runtime : a_state.runtimes) {
			if (runtime.buildGroup != 0 && runtime.domain == a_domain && std::ranges::find(buildGroups, runtime.buildGroup) == buildGroups.end()) {
				buildGroups.push_back(runtime.buildGroup);
			}
		}
		for (const auto& prototypeMesh : a_state.meshes) {
			if (prototypeMesh.buildGroup != 0 && prototypeMesh.domain == a_domain && std::ranges::find(buildGroups, prototypeMesh.buildGroup) == buildGroups.end()) {
				buildGroups.push_back(prototypeMesh.buildGroup);
			}
		}
		for (const auto& prototypeConstraint : a_state.constraints) {
			if (prototypeConstraint.buildGroup != 0 && prototypeConstraint.domain == a_domain && std::ranges::find(buildGroups, prototypeConstraint.buildGroup) == buildGroups.end()) {
				buildGroups.push_back(prototypeConstraint.buildGroup);
			}
		}
		for (const auto& prototypeBody : a_state.bodies) {
			for (const auto& [buildGroup, domain] : prototypeBody.buildGroupDomains) {
				if (buildGroup != 0 && domain == a_domain && std::ranges::find(buildGroups, buildGroup) == buildGroups.end()) {
					buildGroups.push_back(buildGroup);
				}
			}
		}

		if (buildGroups.empty()) {
			return false;
		}

		ClearPrototypeGroupsLocked(a_state, buildGroups);
		return true;
	}

	void Fo4PhysicsWorld::ClearHeadPrototypeTrackingLocked(PrototypeActorState& a_state, const std::string_view a_reason)
	{
		const auto clearedHead = ClearPrototypeGroupsByDomainLocked(a_state, PrototypeBuildDomain::kHead);
		const auto clearedHair = ClearPrototypeGroupsByDomainLocked(a_state, PrototypeBuildDomain::kHair);

		std::uint32_t detachedNodeCount = 0;
		auto detachHeadRenamedChildren = [&detachedNodeCount](auto&& a_self, RE::NiNode* a_node) -> void {
			if (!a_node) {
				return;
			}

			auto index = a_node->children.size();
			while (index > 0) {
				--index;
				auto* child = a_node->children[index].get();
				if (!child) {
					continue;
				}

				const auto childName = child->GetName();
				if (!childName.empty() && StartsWithInsensitive(childName, "hdtSSEPhysics_AutoRename_Head_")) {
					RE::NiPointer<RE::NiAVObject> keepAlive{ child };
					a_node->DetachChild(child);
					++detachedNodeCount;
					continue;
				}

				if (auto* childNode = child->IsNode()) {
					a_self(a_self, childNode);
				}
			}
		};

		if (auto* actor = a_state.actor) {
			if (auto* root = actor->Get3D(a_state.firstPerson)) {
				if (auto* rootNode = root->IsNode()) {
					detachHeadRenamedChildren(detachHeadRenamedChildren, rootNode);
				}
			}
			if (auto* root = actor->Get3D(false)) {
				if (auto* rootNode = root->IsNode()) {
					detachHeadRenamedChildren(detachHeadRenamedChildren, rootNode);
				}
			}
		}

		const auto mergedNodeCount = std::erase_if(a_state.mergedNodes, [](PrototypeMergedNode& a_node) {
			auto* node = a_node.node ? a_node.node->IsNode() : nullptr;
			if (!node) {
				return false;
			}
			const auto name = node->GetName();
			if (!name.empty() && StartsWithInsensitive(name, "hdtSSEPhysics_AutoRename_Head_")) {
				if (a_node.parent && node->parent == a_node.parent) {
					a_node.parent->DetachChild(a_node.node.get());
				}
				a_node.node = nullptr;
				a_node.parent = nullptr;
				return true;
			}
			return false;
		});

		const auto recordCount = a_state.headPartRecords.size();
		a_state.headPartRecords.clear();
		spdlog::debug(
			"cleared head/hair prototype tracking actor={} reason={} clearedHead={} clearedHair={} detachedHeadNodes={} mergedNodes={} headPartRecords={}",
			static_cast<void*>(a_state.actor),
			a_reason,
			clearedHead,
			clearedHair,
			detachedNodeCount,
			mergedNodeCount,
			recordCount);
	}

	void Fo4PhysicsWorld::ClearPrototypeGroupsLocked(PrototypeActorState& a_state, const std::vector<std::uint64_t>& a_buildGroups, const bool a_detachMergedNodes)
	{
		if (a_buildGroups.empty()) {
			return;
		}

		const auto containsGroup = [&a_buildGroups](const std::uint64_t a_buildGroup) {
			return std::ranges::find(a_buildGroups, a_buildGroup) != a_buildGroups.end();
		};
		const auto allGroupsRemoved = [&containsGroup](const PrototypeBody& a_body) {
			return !a_body.buildGroups.empty() ?
				std::ranges::all_of(a_body.buildGroups, containsGroup) :
				containsGroup(a_body.buildGroup);
		};
		if (dynamicsWorld_) {
			for (const auto& runtime : a_state.runtimes) {
				if (!containsGroup(runtime.buildGroup)) {
					continue;
				}

				for (auto* constraint : runtime.constraints) {
					if (!constraint) {
						continue;
					}
					dynamicsWorld_->removeConstraint(constraint);
					for (auto& prototypeConstraint : a_state.constraints) {
						if (prototypeConstraint.constraint.get() == constraint) {
							prototypeConstraint.inBulletWorld = false;
							break;
						}
					}
				}

				for (auto* mesh : runtime.meshes) {
					if (!mesh) {
						continue;
					}
					dynamicsWorld_->removeCollisionObject(mesh);
					for (auto& prototypeMesh : a_state.meshes) {
						if (prototypeMesh.body.get() == mesh) {
							prototypeMesh.inBulletWorld = false;
							break;
						}
					}
				}

				for (auto& prototypeBody : a_state.bodies) {
					if (prototypeBody.bone &&
						prototypeBody.inBulletWorld &&
						allGroupsRemoved(prototypeBody) &&
						std::ranges::find(runtime.bones, prototypeBody.bone.get()) != runtime.bones.end()) {
						dynamicsWorld_->removeRigidBody(std::addressof(prototypeBody.bone->m_rig));
						prototypeBody.inBulletWorld = false;
					}
				}
			}

			for (auto& prototypeConstraint : a_state.constraints) {
				if (prototypeConstraint.constraint && prototypeConstraint.inBulletWorld && containsGroup(prototypeConstraint.buildGroup)) {
					dynamicsWorld_->removeConstraint(prototypeConstraint.constraint.get());
					prototypeConstraint.inBulletWorld = false;
				}
			}

			for (auto& prototypeMesh : a_state.meshes) {
				if (prototypeMesh.body && prototypeMesh.inBulletWorld && containsGroup(prototypeMesh.buildGroup)) {
					dynamicsWorld_->removeCollisionObject(prototypeMesh.body.get());
					prototypeMesh.inBulletWorld = false;
				}
			}

			for (auto& prototypeBody : a_state.bodies) {
				if (prototypeBody.bone && prototypeBody.inBulletWorld && allGroupsRemoved(prototypeBody)) {
					dynamicsWorld_->removeRigidBody(std::addressof(prototypeBody.bone->m_rig));
					prototypeBody.inBulletWorld = false;
				}
			}
		}

		std::vector<Fo4SkinnedMeshBone::ActiveSkinSlot> activeSkinSlots;
		for (const auto& prototypeBody : a_state.bodies) {
			if (allGroupsRemoved(prototypeBody) || !prototypeBody.bone) {
				continue;
			}
			prototypeBody.bone->CollectSkinWorldTransformSlots(activeSkinSlots);
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

		for (auto& prototypeBody : a_state.bodies) {
			if (!prototypeBody.bone) {
				continue;
			}

			for (const auto buildGroup : a_buildGroups) {
				prototypeBody.bone->RemoveSkinWorldTransformsForBuildGroup(buildGroup, activeSkinSlots);
			}
		}
		RestoreSuspendedSkinSlotsLocked(a_state, a_buildGroups, activeSkinSlots);

		const auto meshCount = std::erase_if(a_state.meshes, [&containsGroup](const PrototypeMesh& a_mesh) {
			return containsGroup(a_mesh.buildGroup);
		});
		const auto constraintCount = std::erase_if(a_state.constraints, [&containsGroup](const PrototypeConstraint& a_constraint) {
			return containsGroup(a_constraint.buildGroup);
		});
		const auto runtimeCount = std::erase_if(a_state.runtimes, [&containsGroup](const PrototypeBuildGroupRuntime& a_runtime) {
			return containsGroup(a_runtime.buildGroup);
		});

		for (auto& body : a_state.bodies) {
			if (body.buildGroups.empty() && body.buildGroup != 0) {
				body.buildGroups.push_back(body.buildGroup);
			}
			if (body.buildGroupDomains.empty() && body.buildGroup != 0) {
				body.buildGroupDomains.push_back({ body.buildGroup, PrototypeBuildDomain::kArmor });
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
		const auto bodyCount = std::erase_if(a_state.bodies, [](const PrototypeBody& a_body) {
			return a_body.buildGroups.empty();
		});
		const auto mergedNodeCount = std::erase_if(a_state.mergedNodes, [&a_state, &containsGroup, a_detachMergedNodes](PrototypeMergedNode& a_node) {
			if (!containsGroup(a_node.buildGroup)) {
				return false;
			}
			auto* node = a_node.node ? a_node.node->IsNode() : nullptr;
			const auto nodeStillReferencedByKeptGroup = node && std::ranges::any_of(a_state.mergedNodes, [&](const PrototypeMergedNode& a_other) {
				return std::addressof(a_other) != std::addressof(a_node) &&
					!containsGroup(a_other.buildGroup) &&
					a_other.node.get() == a_node.node.get();
			});
			if (a_detachMergedNodes && !nodeStillReferencedByKeptGroup && a_node.parent && node && node->parent == a_node.parent) {
				a_node.parent->DetachChild(a_node.node.get());
			}
			return true;
		});
		std::erase_if(a_state.attachmentRecords, [&containsGroup](PrototypeAttachmentRecord& a_record) {
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
		std::erase_if(a_state.headPartRecords, [&containsGroup](PrototypeHeadPartRecord& a_record) {
			if (a_record.buildGroup == 0 || !containsGroup(a_record.buildGroup)) {
				return false;
			}
			a_record.object = nullptr;
			a_record.sourceObject = nullptr;
			a_record.sourceRoot = nullptr;
			return true;
		});
		const auto armorRecordCount = std::erase_if(a_state.armorRecords, [&containsGroup](PrototypeArmorRecord& a_record) {
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
			a_record.mergeSourceObject = nullptr;
			return true;
		});
		if (!a_state.HasRuntime()) {
			a_state.runtimeSoftSuspended = false;
		}

		spdlog::debug(
			"cleared prototype physics groups={} runtimes={} bodies={} meshes={} constraints={} mergedNodes={} armorRecords={} for actor={}",
			a_buildGroups.size(),
			runtimeCount,
			bodyCount,
			meshCount,
			constraintCount,
			mergedNodeCount,
			armorRecordCount,
			static_cast<void*>(a_state.actor));
	}

	void Fo4PhysicsWorld::ClearAllPrototypeStatesLocked()
	{
		for (auto& actorState : prototypeActors_) {
			ClearPrototypeStateLocked(actorState);
		}
		prototypeActors_.clear();
		suspendedActors_.clear();
	}

	void Fo4PhysicsWorld::ResumeFromLoadingMenuLocked()
	{
		std::size_t resetBodies = 0;
		for (auto& actorState : prototypeActors_) {
			if (actorState.runtimeSoftSuspended) {
				continue;
			}
			actorState.lastWritebackFrame = 0;
			actorState.lastWritebackSource = WritebackSource::kUnknown;
			actorState.currentWindFactor = 1.0F;
			if (!actorState.runtimes.empty()) {
				for (const auto& runtime : actorState.runtimes) {
					for (auto* bone : runtime.bones) {
						if (!bone) {
							continue;
						}
						bone->readTransform(0.0F);
						++resetBodies;
					}
					ScalePrototypeConstraintsLocked(actorState, runtime);
				}
			} else {
				for (auto& prototypeBody : actorState.bodies) {
					if (!prototypeBody.bone) {
						continue;
					}
					prototypeBody.bone->readTransform(0.0F);
					++resetBodies;
				}
				ScalePrototypeConstraintsLocked(actorState);
			}
		}
		if (dynamicsWorld_) {
			dynamicsWorld_->clearForces();
		}
		loadingPhysicsSuspended_ = false;
		loadingMenuDepth_ = 0;
		ResetStepClockLocked();
		spdlog::debug("loading menu resume reset {} prototype physics bodies to current node poses", resetBodies);
	}

	float Fo4PhysicsWorld::PreparePrototypeActorForReadLocked(PrototypeActorState& a_state, float a_timeStep)
	{
		auto* actorRoot = a_state.actor ? a_state.actor->Get3D(a_state.firstPerson) : nullptr;
		if (!actorRoot && a_state.actor && !a_state.firstPerson) {
			actorRoot = a_state.actor->Get3D();
		}
		auto* skeletonRoot = actorRoot ? actorRoot->IsNode() : nullptr;
		if (!skeletonRoot) {
			return a_timeStep;
		}

		auto* topRoot = static_cast<RE::NiAVObject*>(skeletonRoot);
		while (topRoot && topRoot->parent) {
			topRoot = topRoot->parent;
		}

		if (a_state.lastReadRoot && a_state.lastReadRoot.get() != topRoot) {
			a_timeStep = 0.0F;
			a_state.lastRootRotationInitialized = false;
		}
		if (!a_state.readInitialized) {
			a_timeStep = 0.0F;
			a_state.readInitialized = true;
			a_state.lastRootRotationInitialized = false;
		}

		if (a_timeStep <= 0.0F) {
			UpdateTransformUpDown(skeletonRoot, true);
			a_state.lastRootRotation = Fo4Transform::ToBulletTransform(skeletonRoot->world).getRotation();
			if (a_state.lastRootRotation.length2() <= FLT_EPSILON) {
				a_state.lastRootRotation = btQuaternion::getIdentity();
			} else {
				a_state.lastRootRotation.normalize();
			}
			a_state.lastRootRotationInitialized = true;
			a_state.lastReadRoot = topRoot;
			return 0.0F;
		}

		auto newRootRotation = Fo4Transform::ToBulletTransform(skeletonRoot->world).getRotation();
		if (newRootRotation.length2() <= FLT_EPSILON) {
			newRootRotation = btQuaternion::getIdentity();
		} else {
			newRootRotation.normalize();
		}
		if (!a_state.lastRootRotationInitialized || a_state.lastRootRotation.length2() <= FLT_EPSILON) {
			a_state.lastRootRotation = newRootRotation;
			a_state.lastRootRotationInitialized = true;
		} else {
			btVector3 rotationAxis;
			btScalar rotationAngle = 0.0F;
			btTransformUtil::calculateDiffAxisAngleQuaternion(a_state.lastRootRotation, newRootRotation, rotationAxis, rotationAngle);
			if (clampRotations_ && rotationSpeedLimit_ > 0.0F) {
				const auto limit = rotationSpeedLimit_ * a_timeStep;
				if (rotationAngle < -limit || rotationAngle > limit) {
					rotationAngle = btClamped(rotationAngle, -limit, limit);
					auto clampedRotation = btQuaternion(rotationAxis, rotationAngle) * a_state.lastRootRotation;
					clampedRotation.normalize();
					auto clampedWorld = skeletonRoot->world;
					clampedWorld.rotate = Fo4Transform::ToNiTransform(btTransform(clampedRotation), skeletonRoot->world.scale).rotate;
					clampedWorld.translate = skeletonRoot->world.translate;
					skeletonRoot->world = clampedWorld;
					if (skeletonRoot->parent) {
						skeletonRoot->local = skeletonRoot->parent->world.Invert() * skeletonRoot->world;
					} else {
						skeletonRoot->local = skeletonRoot->world;
					}
					for (auto& child : skeletonRoot->children) {
						if (child) {
							UpdateTransformUpDown(child.get(), true);
						}
					}
					a_state.lastRootRotation = clampedRotation;
				} else {
					a_state.lastRootRotation = newRootRotation;
				}
			} else if (unclampedResets_ && unclampedResetAngle_ > 0.0F) {
				const auto limit = unclampedResetAngle_ * a_timeStep;
				if (rotationAngle < -limit || rotationAngle > limit) {
					UpdateTransformUpDown(skeletonRoot, true);
					a_state.lastRootRotation = Fo4Transform::ToBulletTransform(skeletonRoot->world).getRotation();
					if (a_state.lastRootRotation.length2() <= FLT_EPSILON) {
						a_state.lastRootRotation = btQuaternion::getIdentity();
					} else {
						a_state.lastRootRotation.normalize();
					}
					a_state.lastReadRoot = topRoot;
					return 0.0F;
				}
				a_state.lastRootRotation = newRootRotation;
			} else {
				a_state.lastRootRotation = newRootRotation;
			}
		}

		a_state.lastReadRoot = topRoot;
		return a_timeStep;
	}

	bool Fo4PhysicsWorld::PrototypeBuildGroupHasMeshLocked(const PrototypeActorState& a_state, const std::uint64_t a_buildGroup) const
	{
		if (a_buildGroup == 0) {
			return false;
		}

		return std::ranges::any_of(a_state.meshes, [a_buildGroup](const PrototypeMesh& a_mesh) {
			return a_mesh.buildGroup == a_buildGroup && a_mesh.body;
		});
	}

	bool Fo4PhysicsWorld::PrototypeBuildGroupHasBodyLocked(const PrototypeActorState& a_state, const std::uint64_t a_buildGroup) const
	{
		if (a_buildGroup == 0) {
			return false;
		}

		return std::ranges::any_of(a_state.bodies, [a_buildGroup](const PrototypeBody& a_body) {
			return PrototypeBodyHasBuildGroup(a_body, a_buildGroup) && a_body.bone;
		});
	}

	bool Fo4PhysicsWorld::PrototypeBuildGroupIsRecordableLocked(
		const PrototypeActorState& a_state,
		const std::uint64_t a_buildGroup,
		const PrototypeBuildDomain a_domain,
		const RE::BIPED_OBJECT a_bipedObject) const
	{
		(void)a_domain;
		(void)a_bipedObject;

		if (!PrototypeBuildGroupHasBodyLocked(a_state, a_buildGroup)) {
			return false;
		}

		return true;
	}

	void Fo4PhysicsWorld::UpdatePrototypeBuildGroupMeshesLocked(PrototypeActorState& a_state, const std::uint64_t a_buildGroup)
	{
		if (a_buildGroup == 0) {
			return;
		}

		for (auto& prototypeMesh : a_state.meshes) {
			if (!prototypeMesh.body || prototypeMesh.buildGroup != a_buildGroup) {
				continue;
			}

			prototypeMesh.body->internalUpdate();
		}
	}

	void Fo4PhysicsWorld::CommitPrototypeBuildGroupToBulletLocked(PrototypeActorState& a_state, const std::uint64_t a_buildGroup)
	{
		if (!dynamicsWorld_ || a_buildGroup == 0) {
			return;
		}

		std::uint32_t committedMeshes = 0;
		for (auto& prototypeMesh : a_state.meshes) {
			if (prototypeMesh.inBulletWorld || !prototypeMesh.body || prototypeMesh.buildGroup != a_buildGroup) {
				continue;
			}

			dynamicsWorld_->addCollisionObject(prototypeMesh.body.get(), 1, 1);
			prototypeMesh.inBulletWorld = true;
			++committedMeshes;
		}

		std::uint32_t committedBodies = 0;
		for (auto& prototypeBody : a_state.bodies) {
			if (prototypeBody.inBulletWorld || !prototypeBody.bone || !PrototypeBodyHasBuildGroup(prototypeBody, a_buildGroup)) {
				continue;
			}

			// hdtSMP bones are solver/constraint bodies; mesh collisions are handled by the custom dispatcher.
			dynamicsWorld_->addRigidBody(std::addressof(prototypeBody.bone->m_rig), 0, 0);
			prototypeBody.inBulletWorld = true;
			++committedBodies;
		}

		std::uint32_t committedConstraints = 0;
		for (auto& prototypeConstraint : a_state.constraints) {
			if (prototypeConstraint.inBulletWorld || !prototypeConstraint.constraint || prototypeConstraint.buildGroup != a_buildGroup) {
				continue;
			}

			dynamicsWorld_->addConstraint(prototypeConstraint.constraint.get(), true);
			prototypeConstraint.inBulletWorld = true;
			++committedConstraints;
		}

		if (committedBodies > 0 || committedMeshes > 0 || committedConstraints > 0) {
			auto runtime = std::ranges::find_if(a_state.runtimes, [a_buildGroup](const PrototypeBuildGroupRuntime& a_runtime) {
				return a_runtime.buildGroup == a_buildGroup;
			});
			if (runtime == a_state.runtimes.end()) {
				PrototypeBuildGroupRuntime newRuntime;
				newRuntime.buildGroup = a_buildGroup;
				newRuntime.pendingResetPhysicsRead = true;
				if (const auto mesh = std::ranges::find_if(a_state.meshes, [a_buildGroup](const PrototypeMesh& a_mesh) {
						return a_mesh.buildGroup == a_buildGroup;
					});
					mesh != a_state.meshes.end()) {
					newRuntime.domain = mesh->domain;
					newRuntime.bipedObject = mesh->bipedObject;
				} else if (const auto constraint = std::ranges::find_if(a_state.constraints, [a_buildGroup](const PrototypeConstraint& a_constraint) {
						return a_constraint.buildGroup == a_buildGroup;
					});
					constraint != a_state.constraints.end()) {
					newRuntime.domain = constraint->domain;
				}
				a_state.runtimes.push_back(newRuntime);
				runtime = std::prev(a_state.runtimes.end());
			}
			runtime->meshes.clear();
			runtime->bones.clear();
			runtime->constraints.clear();
			for (auto& prototypeMesh : a_state.meshes) {
				if (prototypeMesh.buildGroup != a_buildGroup || !prototypeMesh.body) {
					continue;
				}
				runtime->meshes.push_back(prototypeMesh.body.get());
				runtime->domain = prototypeMesh.domain;
				runtime->bipedObject = prototypeMesh.bipedObject;
			}
			for (auto& prototypeBody : a_state.bodies) {
				if (!PrototypeBodyHasBuildGroup(prototypeBody, a_buildGroup) || !prototypeBody.bone) {
					continue;
				}
				runtime->bones.push_back(prototypeBody.bone.get());
				if (runtime->bipedObject == RE::BIPED_OBJECT::kTotal) {
					const auto biped = std::ranges::find_if(prototypeBody.buildGroupBipedObjects, [a_buildGroup](const auto& a_entry) {
						return a_entry.first == a_buildGroup;
					});
					runtime->bipedObject = biped != prototypeBody.buildGroupBipedObjects.end() ? biped->second : prototypeBody.bipedObject;
				}
				const auto domain = std::ranges::find_if(prototypeBody.buildGroupDomains, [a_buildGroup](const auto& a_entry) {
					return a_entry.first == a_buildGroup;
				});
				if (domain != prototypeBody.buildGroupDomains.end()) {
					runtime->domain = domain->second;
				}
			}
			for (auto& prototypeConstraint : a_state.constraints) {
				if (prototypeConstraint.buildGroup != a_buildGroup || !prototypeConstraint.constraint) {
					continue;
				}
				runtime->constraints.push_back(prototypeConstraint.constraint.get());
				runtime->domain = prototypeConstraint.domain;
			}
			ResetPrototypeBuildGroupToCurrentPoseLocked(a_state, a_buildGroup);
			UpdatePrototypeBuildGroupMeshesLocked(a_state, a_buildGroup);
			spdlog::debug(
				"committed prototype build group to Bullet actor={} buildGroup={} domain={} bipedObject={} bodies={} meshes={} constraints={}",
				static_cast<void*>(a_state.actor),
				a_buildGroup,
				PrototypeDomainName(runtime->domain),
				std::to_underlying(runtime->bipedObject),
				runtime->bones.size(),
				runtime->meshes.size(),
				runtime->constraints.size());
		}
	}
}
