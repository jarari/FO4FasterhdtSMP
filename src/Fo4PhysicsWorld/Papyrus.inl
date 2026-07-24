// Included by ../Fo4PhysicsWorld.cpp.
// Split from Fo4PhysicsWorld.cpp. Do not compile this file separately.

namespace
{
	RE::BipedAnim* ResolveActorBiped(RE::Actor* a_actor, const bool a_firstPerson)
	{
		if (!a_actor) {
			return nullptr;
		}
		if (const auto& biped = a_actor->GetBiped(a_firstPerson)) {
			return biped.get();
		}
		if (!a_firstPerson) {
			return a_actor->GetBiped().get();
		}
		return nullptr;
	}

	bool BipedObjectUsesArmorAddon(
		RE::Actor* a_actor,
		const bool a_firstPerson,
		const RE::BIPED_OBJECT a_bipedObject,
		const RE::TESObjectARMA* a_armorAddon)
	{
		if (!a_actor || !a_armorAddon || a_bipedObject == RE::BIPED_OBJECT::kTotal) {
			return false;
		}

		auto* biped = ResolveActorBiped(a_actor, a_firstPerson);
		auto* bipObject = biped ? biped->GetBipObject(a_bipedObject) : nullptr;
		return bipObject && bipObject->armorAddon == a_armorAddon;
	}

	void LogDynamicHdtResult(const bool a_verbose, const std::string& a_message)
	{
		if (!a_verbose) {
			return;
		}
		spdlog::info("[DynamicHDT] {}", a_message);
		if (auto* console = RE::ConsoleLog::GetSingleton()) {
			console->Log("[DynamicHDT] {}", a_message);
		}
	}
}

namespace Smp
{
	bool Fo4PhysicsWorld::ReloadPhysicsFile(
		RE::Actor* a_actor,
		RE::TESObjectARMA* a_armorAddon,
		const std::string_view a_physicsFilePath,
		const bool a_persist,
		const bool a_verbose)
	{
		if (!a_actor || !a_armorAddon) {
			return false;
		}

		WaitForAsyncStep();
		bool actorFound = false;
		bool armorAddonFound = false;
		bool succeeded = false;
		std::string oldPhysicsFilePath;
		{
			std::scoped_lock lock(lock_);
			PruneInvalidPrototypeStatesLocked();

			for (const bool firstPerson : { false, true }) {
				auto* actorState = FindPrototypeStateLocked(a_actor, firstPerson);
				if (!actorState) {
					continue;
				}
				actorFound = true;

				auto record = std::ranges::find_if(actorState->armorRecords, [&](const PrototypeArmorRecord& a_record) {
					return BipedObjectUsesArmorAddon(a_actor, actorState->firstPerson, a_record.bipedObject, a_armorAddon);
				});
				if (record == actorState->armorRecords.end()) {
					continue;
				}

				armorAddonFound = true;
				if (record->physicsXmlPath == a_physicsFilePath) {
					LogDynamicHdtResult(a_verbose, "Physics file paths are identical; replacement was skipped.");
					succeeded = true;
					break;
				}

				oldPhysicsFilePath = record->physicsXmlPath;
				record->physicsXmlPath = a_physicsFilePath;
				auto queuedRecord = *record;
				queuedRecord.buildGroups.clear();
				queuedRecord.cpuCopyRetryCount = 0;
				queuedRecord.preserveCurrentPose = true;
				MarkPendingActorRebuildLocked(
					a_actor,
					actorState->firstPerson,
					std::vector<PrototypeArmorRecord>{ std::move(queuedRecord) });
				succeeded = true;
				LogDynamicHdtResult(a_verbose, std::format("Physics file path switched to \"{}\".", a_physicsFilePath));
				break;
			}
			if (succeeded && !oldPhysicsFilePath.empty()) {
				TryRebuildPendingActorsLocked(a_actor);
			}
		}

		if (a_persist && succeeded && !oldPhysicsFilePath.empty()) {
			Papyrus::RegisterPhysicsFileOverride(
				a_actor->GetFormID(),
				std::move(oldPhysicsFilePath),
				std::string(a_physicsFilePath));
		}

		LogDynamicHdtResult(
			a_verbose,
			std::format(
				"Character ({:08X}) {}, ArmorAddon ({:08X}) {}.",
				a_actor->GetFormID(),
				actorFound ? "found" : "not found",
				a_armorAddon->GetFormID(),
				armorAddonFound ? "found" : "not found"));
		return succeeded;
	}

	bool Fo4PhysicsWorld::SwapPhysicsFile(
		RE::Actor* a_actor,
		const std::string_view a_oldPhysicsFilePath,
		const std::string_view a_newPhysicsFilePath,
		const bool a_persist,
		const bool a_verbose)
	{
		if (!a_actor) {
			return false;
		}

		WaitForAsyncStep();
		bool actorFound = false;
		bool physicsFileFound = false;
		bool succeeded = false;
		{
			std::scoped_lock lock(lock_);
			PruneInvalidPrototypeStatesLocked();

			for (const bool firstPerson : { false, true }) {
				auto* actorState = FindPrototypeStateLocked(a_actor, firstPerson);
				if (!actorState) {
					continue;
				}
				actorFound = true;

				auto record = std::ranges::find_if(actorState->armorRecords, [&](const PrototypeArmorRecord& a_record) {
					return a_record.physicsXmlPath == a_oldPhysicsFilePath;
				});
				if (record == actorState->armorRecords.end()) {
					continue;
				}

				physicsFileFound = true;
				if (record->physicsXmlPath == a_newPhysicsFilePath) {
					LogDynamicHdtResult(a_verbose, "Physics file paths are identical; replacement was skipped.");
					succeeded = true;
					break;
				}

				record->physicsXmlPath = a_newPhysicsFilePath;
				auto queuedRecord = *record;
				queuedRecord.buildGroups.clear();
				queuedRecord.cpuCopyRetryCount = 0;
				queuedRecord.preserveCurrentPose = true;
				MarkPendingActorRebuildLocked(
					a_actor,
					actorState->firstPerson,
					std::vector<PrototypeArmorRecord>{ std::move(queuedRecord) });
				succeeded = true;
				break;
			}
			if (succeeded && physicsFileFound && a_oldPhysicsFilePath != a_newPhysicsFilePath) {
				TryRebuildPendingActorsLocked(a_actor);
			}
		}

		if (a_persist) {
			Papyrus::RegisterPhysicsFileOverride(
				a_actor->GetFormID(),
				std::string(a_oldPhysicsFilePath),
				std::string(a_newPhysicsFilePath));
		}

		LogDynamicHdtResult(
			a_verbose,
			std::format(
				"Character ({:08X}) {}, physics file path {}.",
				a_actor->GetFormID(),
				actorFound ? "found" : "not found",
				physicsFileFound ? "found" : "not found"));
		return succeeded;
	}

	std::string Fo4PhysicsWorld::QueryCurrentPhysicsFile(
		RE::Actor* a_actor,
		RE::TESObjectARMA* a_armorAddon,
		const bool a_verbose)
	{
		if (!a_actor || !a_armorAddon) {
			return {};
		}

		WaitForAsyncStep();
		bool actorFound = false;
		bool armorAddonFound = false;
		std::string physicsFilePath;
		{
			std::scoped_lock lock(lock_);
			PruneInvalidPrototypeStatesLocked();

			for (const bool firstPerson : { false, true }) {
				const auto* actorState = FindPrototypeStateLocked(a_actor, firstPerson);
				if (!actorState) {
					continue;
				}
				actorFound = true;

				const auto record = std::ranges::find_if(actorState->armorRecords, [&](const PrototypeArmorRecord& a_record) {
					return BipedObjectUsesArmorAddon(a_actor, actorState->firstPerson, a_record.bipedObject, a_armorAddon);
				});
				if (record == actorState->armorRecords.end()) {
					continue;
				}

				armorAddonFound = true;
				physicsFilePath = record->physicsXmlPath;
				break;
			}
		}

		LogDynamicHdtResult(
			a_verbose,
			std::format(
				"Character ({:08X}) {}, ArmorAddon ({:08X}) {}.",
				a_actor->GetFormID(),
				actorFound ? "found" : "not found",
				a_armorAddon->GetFormID(),
				armorAddonFound ? "found" : "not found"));
		return physicsFilePath;
	}

	std::vector<bool> Fo4PhysicsWorld::TogglePhysics(
		RE::Actor* a_actor,
		const std::span<const std::string> a_boneNames,
		const bool a_on)
	{
		std::vector<bool> result(a_boneNames.size(), false);
		if (!a_actor || a_boneNames.empty()) {
			return result;
		}

		WaitForAsyncStep();
		std::scoped_lock lock(lock_);
		PruneInvalidPrototypeStatesLocked();

		for (std::size_t nameIndex = 0; nameIndex < a_boneNames.size(); ++nameIndex) {
			bool foundAny = false;
			for (auto& actorState : prototypeActors_) {
				auto* stateActor = actorState.actor;
				if (!stateActor) {
					if (const auto resolved = actorState.actorHandle.get()) {
						stateActor = resolved.get();
					}
				}
				if (stateActor != a_actor) {
					continue;
				}

				for (auto& body : actorState.bodies) {
					if (!body.bone || !PhysicsNamesEqual(body.boneName, a_boneNames[nameIndex])) {
						continue;
					}

					auto& rigidBody = body.bone->m_rig;
					const auto currentlyDynamic = !rigidBody.isStaticOrKinematicObject();
					if (!std::exchange(foundAny, true)) {
						result[nameIndex] = currentlyDynamic;
					}
					if (currentlyDynamic == a_on || (a_on && rigidBody.getInvMass() <= 0.0F)) {
						continue;
					}

					const auto flags = rigidBody.getCollisionFlags();
					rigidBody.setCollisionFlags(
						a_on ?
							(flags & ~btCollisionObject::CF_KINEMATIC_OBJECT) :
							(flags | btCollisionObject::CF_KINEMATIC_OBJECT));
					const btVector3 zero(0.0F, 0.0F, 0.0F);
					rigidBody.clearForces();
					rigidBody.setLinearVelocity(zero);
					rigidBody.setAngularVelocity(zero);
					rigidBody.setInterpolationLinearVelocity(zero);
					rigidBody.setInterpolationAngularVelocity(zero);
					rigidBody.activate(true);

					for (auto& constraint : actorState.constraints) {
						if (!constraint.constraint) {
							continue;
						}
						auto& constraintBodyA = constraint.constraint->getRigidBodyA();
						auto& constraintBodyB = constraint.constraint->getRigidBodyB();
						if (std::addressof(constraintBodyA) != std::addressof(rigidBody) &&
							std::addressof(constraintBodyB) != std::addressof(rigidBody)) {
							continue;
						}
						const auto bothKinematic =
							constraintBodyA.isStaticOrKinematicObject() &&
							constraintBodyB.isStaticOrKinematicObject();
						constraint.constraint->setEnabled(!bothKinematic);
					}
				}
			}
		}
		return result;
	}

	void Fo4PhysicsWorld::ResetActorPhysics(RE::Actor* a_actor, const bool a_full)
	{
		if (!a_actor) {
			return;
		}

		WaitForAsyncStep();
		std::scoped_lock lock(lock_);
		PruneInvalidPrototypeStatesLocked();

		bool queuedActorRebuild = false;
		bool queuedHeadRebuild = false;
		for (const bool firstPerson : { false, true }) {
			auto* actorState = FindPrototypeStateLocked(a_actor, firstPerson);
			if (!actorState) {
				continue;
			}

			if (a_full) {
				auto armorRecords = actorState->armorRecords;
				for (auto& record : armorRecords) {
					record.buildGroups.clear();
					record.cpuCopyRetryCount = 0;
					record.preserveCurrentPose = false;
				}
				MarkPendingActorRebuildLocked(
					a_actor,
					actorState->firstPerson,
					std::move(armorRecords),
					true,
					true,
					true);
				queuedActorRebuild = true;
			} else {
				auto armorRecords = actorState->armorRecords;
				for (auto& record : armorRecords) {
					record.buildGroups.clear();
					record.cpuCopyRetryCount = 0;
					record.preserveCurrentPose = true;
				}
				if (!armorRecords.empty()) {
					MarkPendingActorRebuildLocked(
						a_actor,
						actorState->firstPerson,
						std::move(armorRecords),
						false,
						true,
						true);
					queuedActorRebuild = true;
				}
			}

			if (!actorState->firstPerson && (a_full || !actorState->headPartRecords.empty())) {
				if (!a_full) {
					std::vector<std::uint64_t> headBuildGroups;
					CollectHeadPartGroupsLocked(*actorState, headBuildGroups);
					if (!headBuildGroups.empty()) {
						std::vector<PrototypeAttachmentBoneLocalPose> cachedHeadPoses;
						for (const auto& localPose : actorState->attachmentBoneLocalPoses) {
							if (!localPose.node ||
								std::ranges::find(headBuildGroups, localPose.buildGroup) == headBuildGroups.end() ||
								std::ranges::any_of(cachedHeadPoses, [&](const PrototypeAttachmentBoneLocalPose& a_cached) {
									return a_cached.node.get() == localPose.node.get();
								})) {
								continue;
							}
							auto cachedPose = localPose;
							cachedPose.buildGroup = kPapyrusHeadPoseCacheBuildGroup;
							cachedHeadPoses.push_back(std::move(cachedPose));
						}
						ClearPrototypeGroupsLocked(*actorState, headBuildGroups, false);
						actorState->attachmentBoneLocalPoses.insert(
							actorState->attachmentBoneLocalPoses.end(),
							cachedHeadPoses.begin(),
							cachedHeadPoses.end());
					}
				}
				MarkPendingHeadRebuildLocked(LifecycleEvent{
					.type = LifecycleEventType::kActorHeadInitialized,
					.actor = a_actor,
					.object = a_actor->GetFaceNodeSkinned() ?
						reinterpret_cast<RE::NiAVObject*>(a_actor->GetFaceNodeSkinned()) :
						nullptr,
					.firstPerson = false,
				});
				for (auto& pendingHead : pendingHeadRebuilds_) {
					const auto pendingActor = pendingHead.actorHandle.get();
					if (pendingActor && pendingActor.get() == a_actor) {
						pendingHead.frameDelay = 0;
					}
				}
				queuedHeadRebuild = true;
			}
		}
		if (queuedActorRebuild) {
			TryRebuildPendingActorsLocked(a_actor);
		}
		if (queuedHeadRebuild) {
			TryRebuildPendingHeadsLocked();
		}
		ResetStepClockLocked();
	}
}
