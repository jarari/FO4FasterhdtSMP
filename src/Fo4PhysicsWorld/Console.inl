// Included by ../Fo4PhysicsWorld.cpp.
// Split from Fo4PhysicsWorld.cpp. Do not compile this file separately.

namespace Smp
{
	namespace
	{
		const char* ConsolePhysicsState(const bool a_hasPhysics, const bool a_active)
		{
			return !a_hasPhysics ?
				"has no physics system" :
				a_active ? "has active physics system" : "has inactive physics system";
		}

		std::string ConsoleObjectName(const RE::NiAVObject* a_object, const std::string_view a_fallback)
		{
			if (a_object && !a_object->name.empty()) {
				return a_object->name.c_str();
			}
			if (!a_fallback.empty()) {
				return std::string(a_fallback);
			}
			return "unk_name";
		}
	}

	void Fo4PhysicsWorld::PrintConsoleDetails(const bool a_includeItems)
	{
		WaitForAsyncStep();
		std::scoped_lock lock(lock_);
		PruneInvalidPrototypeStatesLocked();

		auto* console = RE::ConsoleLog::GetSingleton();
		if (!console) {
			return;
		}

		std::vector<const PrototypeActorState*> orderedStates;
		orderedStates.reserve(prototypeActors_.size());
		for (const auto& actorState : prototypeActors_) {
			orderedStates.push_back(std::addressof(actorState));
		}
		std::ranges::stable_sort(orderedStates, [](const auto* a_lhs, const auto* a_rhs) {
			return a_lhs->HasActiveRuntime() < a_rhs->HasActiveRuntime();
		});

		for (const auto* actorState : orderedStates) {
			const auto* actor = actorState->actor;
			const auto* baseObject = actor ? actor->GetNPC() : nullptr;
			const auto ownerName = baseObject ? RE::TESFullName::GetFullName(*baseObject) : std::string_view{};
			const auto active = actorState->HasActiveRuntime();

			const char* state = "Unseen by player";
			if (active) {
				state = actor && actor->IsPlayerRef() ? "Is player character" : "Is near player";
			} else if (actorState->runtimeSoftSuspended) {
				state = "Deactivated for performance";
			} else if (actorState->runtimeSuspended) {
				state = "Not in scene";
			}

			console->Log(
				"[HDT-SMP] {} skeleton - owner {} (refr formid {:08x}, base formid {:08x}) - {}",
				active ? "active" : "inactive",
				ownerName.empty() ? "unk_name" : ownerName,
				actor ? actor->GetFormID() : 0,
				baseObject ? baseObject->GetFormID() : 0,
				state);

			if (!a_includeItems) {
				continue;
			}

			const auto groupIsActive = [&](const std::uint64_t a_buildGroup) {
				return active &&
					a_buildGroup != 0 &&
					std::ranges::any_of(actorState->runtimes, [&](const PrototypeBuildGroupRuntime& a_runtime) {
						return a_runtime.buildGroup == a_buildGroup;
					});
			};

			for (const auto& armor : actorState->armorRecords) {
				const auto hasPhysics = !armor.physicsXmlPath.empty() || !armor.buildGroups.empty();
				const auto itemActive = std::ranges::any_of(armor.buildGroups, groupIsActive);
				console->Log(
					"[HDT-SMP] -- tracked armor addon {}, {}",
					ConsoleObjectName(armor.attachedObject.get(), armor.physicsXmlPath),
					ConsolePhysicsState(hasPhysics, itemActive));

				if (hasPhysics) {
					for (const auto& mesh : actorState->meshes) {
						if (std::ranges::find(armor.buildGroups, mesh.buildGroup) != armor.buildGroups.end()) {
							console->Log("[HDT-SMP] ---- has collision mesh {}", mesh.name);
						}
					}
				}
			}

			for (const auto& headPart : actorState->headPartRecords) {
				const auto hasPhysics = !headPart.physicsXmlPath.empty() || headPart.buildGroup != 0;
				const auto itemActive = groupIsActive(headPart.buildGroup);
				console->Log(
					"[HDT-SMP] -- tracked headpart {}, {}",
					ConsoleObjectName(headPart.object.get(), headPart.physicsXmlPath),
					ConsolePhysicsState(hasPhysics, itemActive));

				if (hasPhysics) {
					for (const auto& mesh : actorState->meshes) {
						if (mesh.buildGroup == headPart.buildGroup) {
							console->Log("[HDT-SMP] ---- has collision mesh {}", mesh.name);
						}
					}
				}
			}
		}
	}

	void Fo4PhysicsWorld::PrintConsoleSummary()
	{
		WaitForAsyncStep();
		std::scoped_lock lock(lock_);
		PruneInvalidPrototypeStatesLocked();

		std::size_t activeSkeletons = 0;
		std::size_t armors = 0;
		std::size_t headParts = 0;
		std::size_t activeArmors = 0;
		std::size_t activeHeadParts = 0;
		std::size_t activeCollisionMeshes = 0;

		for (const auto& actorState : prototypeActors_) {
			const auto active = actorState.HasActiveRuntime();
			if (active) {
				++activeSkeletons;
			}

			const auto groupIsActive = [&](const std::uint64_t a_buildGroup) {
				return active &&
					a_buildGroup != 0 &&
					std::ranges::any_of(actorState.runtimes, [&](const PrototypeBuildGroupRuntime& a_runtime) {
						return a_runtime.buildGroup == a_buildGroup;
					});
			};

			armors += actorState.armorRecords.size();
			for (const auto& armor : actorState.armorRecords) {
				if (std::ranges::any_of(armor.buildGroups, groupIsActive)) {
					++activeArmors;
				}
			}

			headParts += actorState.headPartRecords.size();
			for (const auto& headPart : actorState.headPartRecords) {
				if (groupIsActive(headPart.buildGroup)) {
					++activeHeadParts;
				}
			}

			if (active) {
				activeCollisionMeshes += static_cast<std::size_t>(std::ranges::count_if(
					actorState.meshes,
					[](const PrototypeMesh& a_mesh) {
						return a_mesh.inBulletWorld && a_mesh.body;
					}));
			}
		}

		if (auto* console = RE::ConsoleLog::GetSingleton()) {
			console->Log("[HDT-SMP] tracked skeletons: {}", prototypeActors_.size());
			console->Log("[HDT-SMP] active skeletons: {}", activeSkeletons);
			console->Log("[HDT-SMP] tracked armor addons: {}", armors);
			console->Log("[HDT-SMP] tracked head parts: {}", headParts);
			console->Log("[HDT-SMP] active armor addons: {}", activeArmors);
			console->Log("[HDT-SMP] active head parts: {}", activeHeadParts);
			console->Log("[HDT-SMP] active collision meshes: {}", activeCollisionMeshes);
		}
	}
}
