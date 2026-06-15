// Included by ../Fo4PhysicsWorld.cpp.
// Split from Fo4PhysicsWorld.cpp. Do not compile this file separately.

namespace Smp
{
	Fo4PhysicsWorld::PrototypeBuildResult Fo4PhysicsWorld::BuildPrototypeBodiesLocked(
		PrototypeActorState& a_state,
		const LifecycleEvent& a_event,
		const PhysicsXmlSummary& a_summary,
		const DefaultBBP::NameMap& a_meshNameMap,
		const PrototypeBuildDomain a_domain,
		const bool a_commitToBullet)
	{
		struct BuildTiming
		{
			float actorTreePrepMs{ 0.0F };
			float cloneMergeMs{ 0.0F };
			float referencePoseMs{ 0.0F };
			float xmlSkinResolveMs{ 0.0F };
			float bulletBodyMs{ 0.0F };
			float meshBuildMs{ 0.0F };
			float bindCommitConstraintMs{ 0.0F };
		};

		const auto buildTimingStart = Clock::now();
		BuildTiming timing;
		auto logBuildTiming = [&](const char* a_reason, const std::uint64_t a_buildGroup = 0) {
			spdlog::debug(
				"prototype build timing actor={} domain={} bipedObject={} buildGroup={} reason={} totalMs={:.3f} actorTreePrepMs={:.3f} cloneMergeMs={:.3f} referencePoseMs={:.3f} xmlSkinResolveMs={:.3f} bulletBodyMs={:.3f} meshBuildMs={:.3f} bindCommitConstraintMs={:.3f}",
				static_cast<void*>(a_state.actor),
				PrototypeDomainName(a_domain),
				std::to_underlying(a_event.bipedObject),
				a_buildGroup,
				a_reason,
				ElapsedMs(buildTimingStart, Clock::now()),
				timing.actorTreePrepMs,
				timing.cloneMergeMs,
				timing.referencePoseMs,
				timing.xmlSkinResolveMs,
				timing.bulletBodyMs,
				timing.meshBuildMs,
				timing.bindCommitConstraintMs);
		};

		a_state.runtimeSuspended = false;
		a_state.runtimeSoftSuspended = false;
		auto meshNames = BuildMeshMatchNames(a_summary, a_meshNameMap);
		if (a_summary.boneNames.empty() && meshNames.empty()) {
			spdlog::debug("prototype physics XML has no named bones or mesh descriptors to match");
			logBuildTiming("empty-xml");
			return {};
		}

		std::vector<MatchedSkinBone> matchedBones;
		auto phaseStart = Clock::now();
		auto* skeletonSearchRoot = ResolveSkeletonSearchRoot(a_event);
		const auto actorSkeletonSearchExclusions = BuildBipedPartCloneExclusions(a_event);
		const auto knownArmorNodes = BuildKnownArmorNodeSet(a_event);
		auto* actorRoot = a_event.actor ? a_event.actor->Get3D(a_event.firstPerson) : nullptr;
		if (!actorRoot && a_event.actor) {
			actorRoot = a_event.actor->Get3D();
		}
		auto* actorRootNode = actorRoot ? actorRoot->IsNode() : nullptr;
		if (actorRootNode) {
			UpdateTransformUpDown(actorRootNode, true);
		}
		timing.actorTreePrepMs += ElapsedMs(phaseStart, Clock::now());
		if (a_domain == PrototypeBuildDomain::kArmor && !a_summary.meshDescriptors.empty()) {
			auto preflightExtraction = ExtractSkinnedMeshes(a_event.object, meshNames);
			auto preflightCpuCopyPending = HasPendingCpuCopyExtraction(preflightExtraction);
			auto preflightPendingMatchedGeometries = preflightCpuCopyPending ? preflightExtraction.stats.matchedGeometries : 0U;
			auto preflightPendingVertexCopies = preflightCpuCopyPending ? preflightExtraction.stats.pendingVertexCopies : 0U;
			auto preflightPendingIndexCopies = preflightCpuCopyPending ? preflightExtraction.stats.pendingIndexCopies : 0U;
			if (preflightExtraction.meshes.empty()) {
				const std::array fallbackRoots{
					a_event.mergeSourceObject,
					a_event.sourceObject,
					static_cast<RE::NiAVObject*>(a_event.sourceRoot),
				};
				for (auto* root : fallbackRoots) {
					if (!root || root == a_event.object) {
						continue;
					}

					auto fallbackExtraction = ExtractSkinnedMeshes(root, meshNames);
					if (!HasPendingCpuCopyExtraction(fallbackExtraction)) {
						continue;
					}

					preflightCpuCopyPending = true;
					preflightPendingMatchedGeometries += fallbackExtraction.stats.matchedGeometries;
					preflightPendingVertexCopies += fallbackExtraction.stats.pendingVertexCopies;
					preflightPendingIndexCopies += fallbackExtraction.stats.pendingIndexCopies;
				}
			}
			if (preflightCpuCopyPending) {
				spdlog::debug(
					"prototype armor mesh extraction preflight delayed for pending CPU copy before merge/reference pose actor={} object={} matched={} pendingVertexCopies={} pendingIndexCopies={}",
					static_cast<void*>(a_state.actor),
					static_cast<void*>(a_event.object),
					preflightPendingMatchedGeometries,
					preflightPendingVertexCopies,
					preflightPendingIndexCopies);
				PrototypeBuildResult result;
				result.cpuCopyPending = true;
				logBuildTiming("cpu-copy-pending-preflight");
				return result;
			}
		}
		std::vector<MergedSkeletonNode> mergedSkeletonNodes;
		std::vector<MergedRootNode> mergedRootNodes;
		std::vector<SavedNodeLocalPose> savedBuildPoses;
		RE::NiPointer<RE::NiAVObject> preservedSourceClone;
		phaseStart = Clock::now();
		const auto trustedActorSkeletonNodes = BuildTrustedActorSkeletonNodeSet(a_event);
		if (!trustedActorSkeletonNodes.empty()) {
			spdlog::debug(
				"using pre-attach trusted actor skeleton node set actor={} nodes={}",
				static_cast<void*>(a_event.actor),
				trustedActorSkeletonNodes.size());
		}
		std::unordered_set<std::string> reservedMergedSourceNames;
		if (a_domain == PrototypeBuildDomain::kArmor) {
			reservedMergedSourceNames.reserve(a_event.mergeRenameMap.size());
			for (const auto& rename : a_event.mergeRenameMap) {
				const auto key = NormalizeActorLookupName(rename.sourceName);
				if (!key.empty()) {
					reservedMergedSourceNames.insert(key);
				}
			}
		}
		const auto actorSkeletonLookupMode =
			a_domain == PrototypeBuildDomain::kArmor && !trustedActorSkeletonNodes.empty() ?
				ActorSkeletonLookupMode::kAuthoritativeOnly :
				ActorSkeletonLookupMode::kPermissiveNonArmor;
		const auto actorSkeletonLookup = BuildActorSkeletonLookup(actorRoot, actorSkeletonSearchExclusions, knownArmorNodes, trustedActorSkeletonNodes, actorSkeletonLookupMode);
		const auto mergePrefix = !a_event.mergeRenamePrefix.empty() ? a_event.mergeRenamePrefix :
			(a_domain == PrototypeBuildDomain::kHead || a_domain == PrototypeBuildDomain::kHair) ?
				MakeReferenceHeadRenamePrefix(PrototypeHeadRenameId.fetch_add(1, std::memory_order_relaxed)) :
				MakeReferenceArmorRenamePrefix(PrototypeArmorRenameId.fetch_add(1, std::memory_order_relaxed));
		auto* mergeSourceObject = a_event.mergeSourceObject ? a_event.mergeSourceObject : a_event.sourceObject;
		auto* sourceRoot = mergeSourceObject ? mergeSourceObject->IsNode() : a_event.sourceRoot;
		const auto smpClonedPrefix = MakeReferenceSmpClonedPrefix(mergePrefix);
		if (sourceRoot && a_event.preserveMergeSourceNames) {
			preservedSourceClone = CloneNodeExact(sourceRoot);
			if (auto* clonedRoot = preservedSourceClone ? preservedSourceClone->IsNode() : nullptr) {
				sourceRoot = clonedRoot;
			}
		}
		const auto usePreMergedRenameMap = a_domain == PrototypeBuildDomain::kArmor && !a_event.mergeRenameMap.empty();
		if (usePreMergedRenameMap) {
			RegisterMergedRenameMapNodes(
				actorRoot,
				trustedActorSkeletonNodes,
				a_event.mergeRenameMap,
				a_event.mergeParentBindings,
				mergedSkeletonNodes,
				mergedRootNodes);
			RestorePreMergedRenameMapLocalPoseFromSource(mergedSkeletonNodes, sourceRoot, a_event.mergeParentBindings);
			if (sourceRoot && mergedSkeletonNodes.size() < a_event.mergeRenameMap.size()) {
				spdlog::debug(
					"pre-merged armor rename map only resolved {}/{} nodes; rebuilding missing armor skeleton from preserved source={} sourceName='{}'",
					mergedSkeletonNodes.size(),
					a_event.mergeRenameMap.size(),
					static_cast<void*>(sourceRoot),
					std::string_view(sourceRoot->GetName()));
			}
		}
		if ((!usePreMergedRenameMap || (sourceRoot && mergedSkeletonNodes.size() < a_event.mergeRenameMap.size())) && sourceRoot) {
			UpdateNodeWorldFromLocal(sourceRoot);
			auto* liveCloneParent = actorRootNode;
			if (!liveCloneParent) {
				liveCloneParent = a_event.object ? a_event.object->IsNode() : nullptr;
			}
			if (!liveCloneParent) {
				liveCloneParent = sourceRoot->parent ? sourceRoot->parent : sourceRoot;
			}

			const auto sourceRootName = sourceRoot->GetName();
			const auto sourceRootIsActorBone = FindTrustedActorSkeletonNodeForSource(actorRoot, sourceRoot, actorSkeletonLookup, actorSkeletonSearchExclusions, knownArmorNodes, trustedActorSkeletonNodes, actorSkeletonLookupMode, reservedMergedSourceNames) != nullptr;
			if (sourceRoot == liveCloneParent && !sourceRootIsActorBone && !sourceRootName.empty() && IsReferencedXmlBoneName(a_summary, sourceRootName) && sourceRoot->parent) {
				liveCloneParent = sourceRoot->parent;
			}
			if (!sourceRootName.empty() && !sourceRootIsActorBone && HasRelevantXmlDescendant(sourceRoot, a_summary)) {
				spdlog::debug(
					"selectively cloning relevant armor bones from source root '{}' node={} under merge destination parent={} parentName='{}' prefix='{}'",
					sourceRootName,
					static_cast<void*>(sourceRoot),
					static_cast<void*>(liveCloneParent),
					std::string_view(liveCloneParent->GetName()),
					smpClonedPrefix);
			}
			CloneSourceSkeletonIntoPartTree(
				liveCloneParent,
				sourceRoot,
				actorRoot,
				actorSkeletonLookup,
				actorSkeletonSearchExclusions,
				knownArmorNodes,
				trustedActorSkeletonNodes,
				actorSkeletonLookupMode,
				reservedMergedSourceNames,
				smpClonedPrefix,
				a_summary,
				a_event.mergeParentBindings,
				mergedSkeletonNodes,
				mergedRootNodes);
			if (!mergedRootNodes.empty()) {
				RefreshBoneScatterTable(actorRoot);
			}
		}
		if (actorRootNode) {
			UpdateTransformUpDown(actorRootNode, true);
		}
		auto* skeletonLookupRoot = actorRootNode ? static_cast<RE::NiAVObject*>(actorRootNode) : skeletonSearchRoot;
		const auto skeletonLookupExclusions = BuildSkeletonLookupExclusions(actorSkeletonSearchExclusions, mergedRootNodes);
		const auto skeletonLookupKnownArmorNodes = BuildKnownArmorNodeSet(a_event, std::addressof(mergedRootNodes));
		timing.cloneMergeMs += ElapsedMs(phaseStart, Clock::now());
		if (actorRootNode) {
			phaseStart = Clock::now();
			if (!ApplyHavokReferencePose(
					a_event.actor,
					actorRootNode,
					skeletonLookupExclusions,
					skeletonLookupKnownArmorNodes,
					trustedActorSkeletonNodes,
					actorSkeletonLookupMode,
					savedBuildPoses)) {
				UpdateTransformUpDown(actorRootNode, true);
				spdlog::debug(
					"prototype physics build used current actor pose because Havok reference pose was unavailable actor={} root={}",
					static_cast<void*>(a_event.actor),
					static_cast<void*>(actorRootNode));
			}
			timing.referencePoseMs += ElapsedMs(phaseStart, Clock::now());
		}
		phaseStart = Clock::now();
		CollectMatchedSkinBones(a_event.object, a_summary.boneNames, meshNames, matchedBones);
		ResolveExplicitXmlBonesFromMergedSkeleton(
			matchedBones,
			a_summary,
			a_summary.boneNames,
			mergedSkeletonNodes,
			skeletonLookupRoot,
			actorSkeletonLookup,
			sourceRoot,
			a_event.object,
			a_event.sourceObject,
			mergeSourceObject,
			skeletonLookupExclusions,
			skeletonLookupKnownArmorNodes,
			trustedActorSkeletonNodes,
			actorSkeletonLookupMode,
			reservedMergedSourceNames);
		ResolveMatchedSkinBonesFromSkeleton(
			matchedBones,
			a_summary,
			mergedSkeletonNodes,
			skeletonLookupRoot,
			actorSkeletonLookup,
			sourceRoot,
			a_event.object,
			a_event.sourceObject,
			mergeSourceObject,
			skeletonLookupExclusions,
			skeletonLookupKnownArmorNodes,
			trustedActorSkeletonNodes,
			actorSkeletonLookupMode,
			reservedMergedSourceNames);
		if (a_domain == PrototypeBuildDomain::kArmor) {
			const auto removedRawArmorBones = std::erase_if(matchedBones, [&](const MatchedSkinBone& a_matchedBone) {
				const auto remove = IsUnresolvedArmorOwnedMatchedBone(
					a_matchedBone,
					actorRoot,
					sourceRoot,
					a_event.object,
					a_event.sourceObject,
					mergeSourceObject,
					skeletonLookupExclusions,
					skeletonLookupKnownArmorNodes);
				if (remove) {
					spdlog::debug(
						"dropping unresolved armor-owned skin bone '{}' node={} nodeName='{}'; no actor-root fallback will target raw partClone/source nodes",
						a_matchedBone.name,
						static_cast<void*>(a_matchedBone.node),
						a_matchedBone.node ? std::string_view(a_matchedBone.node->GetName()) : std::string_view{});
				}
				return remove;
			});
			if (removedRawArmorBones > 0) {
				spdlog::debug("dropped {} unresolved armor-owned skin bones before prototype body creation", removedRawArmorBones);
			}
		}
		timing.xmlSkinResolveMs += ElapsedMs(phaseStart, Clock::now());
		if (matchedBones.empty()) {
			spdlog::debug(
				"prototype physics XML matched no skin bones for {} actor={} object={}",
				ToString(a_event.type),
				static_cast<void*>(a_event.actor),
				static_cast<void*>(a_event.object));
			RestoreSavedLocalPoses(savedBuildPoses, actorRootNode);
			DetachMergedRootNodes(mergedRootNodes);
			logBuildTiming("no-matched-bones");
			return {};
		}

		std::uint32_t created = 0;
		std::uint32_t dynamicBodies = 0;
		std::uint32_t kinematicBodies = 0;
		std::uint32_t matchedUnderActorRoot = 0;
		std::uint32_t matchedUnderAttachedObject = 0;
		std::uint64_t buildGroup = 0;
		const auto hadActorRuntimeBeforeBuild = a_state.HasRuntime();
		for (const auto& matchedBone : matchedBones) {
			if (IsNodeInTree(actorRoot, matchedBone.node)) {
				++matchedUnderActorRoot;
			}
			if (IsNodeInTree(a_event.object, matchedBone.node)) {
				++matchedUnderAttachedObject;
			}
		}
		if (a_domain != PrototypeBuildDomain::kArmor) {
			for (const auto& prototypeMesh : a_state.meshes) {
				if (prototypeMesh.domain == a_domain && prototypeMesh.geometry && IsObjectInTree(a_event.object, prototypeMesh.geometry)) {
					buildGroup = prototypeMesh.buildGroup;
					break;
				}
			}
			for (const auto& prototypeBody : a_state.bodies) {
				if (buildGroup != 0) {
					break;
				}
				if (prototypeBody.node && IsNodeInTree(a_event.object, prototypeBody.node)) {
					const auto existingGroup = std::ranges::find_if(prototypeBody.buildGroupDomains, [a_domain](const auto& a_entry) {
						return a_entry.second == a_domain;
					});
					if (existingGroup != prototypeBody.buildGroupDomains.end()) {
						buildGroup = existingGroup->first;
						break;
					}
				}
			}
			for (const auto& matchedBone : matchedBones) {
				if (buildGroup != 0) {
					break;
				}
				const auto existing = std::ranges::find_if(a_state.bodies, [&matchedBone](const PrototypeBody& a_body) {
					return a_body.node == matchedBone.node;
				});
				if (existing == a_state.bodies.end()) {
					continue;
				}
				const auto existingGroup = std::ranges::find_if(existing->buildGroupDomains, [a_domain](const auto& a_entry) {
					return a_entry.second == a_domain;
				});
				if (existingGroup != existing->buildGroupDomains.end()) {
					buildGroup = existingGroup->first;
				}
			}
		}
		bool createdBuildGroup = false;
		if (buildGroup == 0) {
			buildGroup = ++a_state.nextBuildGroup;
			createdBuildGroup = true;
		}
		PrototypeBuildResult result;
		result.buildGroup = buildGroup;
		struct BuildGroupRollbackGuard
		{
			Fo4PhysicsWorld* world{ nullptr };
			PrototypeActorState* state{ nullptr };
			std::vector<MergedRootNode>* mergedRoots{ nullptr };
			std::uint64_t buildGroup{ 0 };
			bool active{ false };

			~BuildGroupRollbackGuard()
			{
				if (!active || !world || !state || buildGroup == 0) {
					return;
				}

				if (mergedRoots) {
					DetachMergedRootNodes(*mergedRoots);
				}
				world->ClearPrototypeGroupsLocked(*state, std::vector<std::uint64_t>{ buildGroup });
			}

			void Dismiss()
			{
				active = false;
			}
		};
		BuildGroupRollbackGuard rollbackGuard{
			.world = this,
			.state = std::addressof(a_state),
			.mergedRoots = std::addressof(mergedRootNodes),
			.buildGroup = buildGroup,
			.active = createdBuildGroup,
		};
		std::vector<PrototypeBody> stagedBodies;

		phaseStart = Clock::now();
		for (auto& matchedBone : matchedBones) {
			const auto sameBuildBody = std::ranges::find_if(a_state.bodies, [&matchedBone, buildGroup](const PrototypeBody& a_body) {
				return PrototypeBodyHasBuildGroup(a_body, buildGroup) && a_body.node == matchedBone.node && a_body.bone;
			});
			if (sameBuildBody != a_state.bodies.end()) {
				continue;
			}
			const auto sameStagedBody = std::ranges::find_if(stagedBodies, [&matchedBone, buildGroup](const PrototypeBody& a_body) {
				return PrototypeBodyHasBuildGroup(a_body, buildGroup) && a_body.node == matchedBone.node && a_body.bone;
			});
			if (sameStagedBody != stagedBodies.end()) {
				continue;
			}

			if (a_domain == PrototypeBuildDomain::kArmor && matchedBone.meshOnlySkinBoneCandidate) {
				if (auto* logger = spdlog::default_logger_raw(); logger && logger->should_log(spdlog::level::debug)) {
					spdlog::debug(
						"deferring armor mesh-only skin bone '{}' actor={} node={} nodeName='{}' buildGroup={} to mesh deformer fallback",
						matchedBone.name,
						static_cast<void*>(a_event.actor),
						static_cast<void*>(matchedBone.node),
						matchedBone.node ? std::string_view(matchedBone.node->GetName()) : std::string_view{},
						buildGroup);
				}
				continue;
			}

			if (a_domain != PrototypeBuildDomain::kArmor) {
				const auto existing = std::ranges::find_if(a_state.bodies, [&matchedBone](const PrototypeBody& a_body) {
					return a_body.node == matchedBone.node && a_body.bone;
				});
				if (existing != a_state.bodies.end()) {
					AddPrototypeBodyBuildGroup(*existing, buildGroup, a_domain, a_event.bipedObject);
					continue;
				}
			}

			const auto* descriptor = FindBoneDescriptor(a_summary, matchedBone.name);
			auto fallbackDescriptor = a_summary.defaultBoneDescriptor.value_or(PhysicsBoneDescriptor{});
			fallbackDescriptor.name = matchedBone.name;
			const auto& boneDescriptor = descriptor ? *descriptor : fallbackDescriptor;
			auto shape = CreateCollisionShape(boneDescriptor);
			btVector3 localInertia(0.0F, 0.0F, 0.0F);
			const auto mass = matchedBone.useActorKinematicBody ? 0.0F : std::max(boneDescriptor.mass, 0.0F);
			if (boneDescriptor.hasLocalInertia) {
				localInertia = btVector3(
					std::max(boneDescriptor.localInertia.x, 0.0F),
					std::max(boneDescriptor.localInertia.y, 0.0F),
					std::max(boneDescriptor.localInertia.z, 0.0F));
			} else if (mass > 0.0F) {
				shape->calculateLocalInertia(mass, localInertia);
			}

			const auto localToRig = ToBulletTransform(boneDescriptor.centerOfMassTransform);
			const auto& initialWorld = matchedBone.node->world;
			auto motionState = std::make_unique<btDefaultMotionState>(Smp::Fo4Transform::ToBulletTransform(initialWorld) * localToRig);
			btRigidBody::btRigidBodyConstructionInfo constructionInfo(mass, motionState.get(), shape.get(), localInertia);
			auto bone = std::make_unique<Fo4SkinnedMeshBone>(RE::BSFixedString(matchedBone.name), matchedBone.node, constructionInfo);
			bone->m_localToRig = localToRig;
			bone->m_rigToLocal = localToRig.inverse();
			bone->m_marginMultipler = boneDescriptor.marginMultiplier;
			bone->m_gravityFactor = std::clamp(boneDescriptor.gravityFactor, 0.0F, 1.0F);
			bone->m_windFactor = std::max(boneDescriptor.windFactor, 0.0F);
			for (const auto& boneName : boneDescriptor.canCollideWithBones) {
				bone->m_canCollideWithBone.emplace_back(boneName.c_str());
			}
			for (const auto& boneName : boneDescriptor.noCollideWithBones) {
				bone->m_noCollideWithBone.emplace_back(boneName.c_str());
			}
			bone->m_rig.setDamping(std::max(boneDescriptor.linearDamping, 0.0F), std::max(boneDescriptor.angularDamping, 0.0F));
			bone->m_rig.setFriction(std::max(boneDescriptor.friction, 0.0F));
			bone->m_rig.setRollingFriction(std::max(boneDescriptor.rollingFriction, 0.0F));
			bone->m_rig.setRestitution(std::max(boneDescriptor.restitution, 0.0F));
			bone->m_rig.setGravity(btVector3(0.0F, 0.0F, kGravityAcceleration * bone->m_gravityFactor));
			if (mass <= 0.0F) {
				++kinematicBodies;
			} else {
				++dynamicBodies;
			}
			bone->readTransform(0.0F);
			bone->m_rig.setActivationState(DISABLE_DEACTIVATION);
			if (auto* logger = spdlog::default_logger_raw(); logger && logger->should_log(spdlog::level::debug)) {
				spdlog::debug(
					"staged prototype body writeback target actor={} bone='{}' node={} nodeName='{}' sourceNode={} sourceName='{}' buildGroup={} mass={:.4f}",
					static_cast<void*>(a_event.actor),
					matchedBone.name,
					static_cast<void*>(matchedBone.node),
					matchedBone.node ? std::string_view(matchedBone.node->GetName()) : std::string_view{},
					static_cast<void*>(matchedBone.sourceNode),
					matchedBone.sourceNode ? std::string_view(matchedBone.sourceNode->GetName()) : std::string_view{},
					buildGroup,
					mass);
			}

			PrototypeBody prototypeBody;
			prototypeBody.actor = a_event.actor;
			prototypeBody.node = matchedBone.node;
			prototypeBody.buildGroup = buildGroup;
			prototypeBody.bipedObject = a_event.bipedObject;
			AddPrototypeBodyBuildGroup(prototypeBody, buildGroup, a_domain, a_event.bipedObject);
			prototypeBody.boneName = std::move(matchedBone.name);
			prototypeBody.shape = std::move(shape);
			prototypeBody.motionState = std::move(motionState);
			prototypeBody.bone = std::move(bone);
			stagedBodies.push_back(std::move(prototypeBody));
			++created;
		}
		timing.bulletBodyMs += ElapsedMs(phaseStart, Clock::now());

		std::ranges::stable_sort(stagedBodies, [](const PrototypeBody& a_lhs, const PrototypeBody& a_rhs) {
			const auto lhsDepth = a_lhs.bone ? a_lhs.bone->GetDepth() : 0x7fffffff;
			const auto rhsDepth = a_rhs.bone ? a_rhs.bone->GetDepth() : 0x7fffffff;
			return lhsDepth < rhsDepth;
		});

		ResetPrototypeBuildGroupToCurrentPoseLocked(a_state, buildGroup, stagedBodies);
		std::vector<PrototypeMesh> stagedMeshes;
		std::vector<RE::NiAVObject*> mergedSkeletonNodeObjects;
		mergedSkeletonNodeObjects.reserve(mergedSkeletonNodes.size());
		for (const auto& mergedNode : mergedSkeletonNodes) {
			if (mergedNode.node) {
				mergedSkeletonNodeObjects.push_back(mergedNode.node);
			}
		}
		phaseStart = Clock::now();
		const auto cpuCopyPending = BuildPrototypeMeshesLocked(a_state, a_summary, a_event, a_meshNameMap, buildGroup, a_domain, trustedActorSkeletonNodes, mergedSkeletonNodeObjects, stagedBodies, stagedMeshes);
		timing.meshBuildMs += ElapsedMs(phaseStart, Clock::now());
		const auto bodyOnlyArmorBuild = a_domain == PrototypeBuildDomain::kArmor;
		if (cpuCopyPending) {
			spdlog::debug(
				"prototype build group actor={} buildGroup={} is waiting for mesh CPU copy; staged Bullet rigid bodies were not committed",
				static_cast<void*>(a_state.actor),
				buildGroup);
			result.cpuCopyPending = true;
			RestoreSavedLocalPoses(savedBuildPoses, actorRootNode);
			logBuildTiming("cpu-copy-pending", buildGroup);
			return result;
		}
		if (bodyOnlyArmorBuild && stagedMeshes.empty() && !PrototypeBuildGroupHasMeshLocked(a_state, buildGroup)) {
			spdlog::debug(
				"allowing body-only armor prototype build actor={} bipedObject={} buildGroup={} because no skinned mesh body was decoded",
				static_cast<void*>(a_state.actor),
				std::to_underlying(a_event.bipedObject),
				buildGroup);
		}
		phaseStart = Clock::now();
		for (auto& stagedBody : stagedBodies) {
			a_state.bodies.push_back(std::move(stagedBody));
		}
		std::ranges::stable_sort(a_state.bodies, [](const PrototypeBody& a_lhs, const PrototypeBody& a_rhs) {
			const auto lhsDepth = a_lhs.bone ? a_lhs.bone->GetDepth() : 0x7fffffff;
			const auto rhsDepth = a_rhs.bone ? a_rhs.bone->GetDepth() : 0x7fffffff;
			return lhsDepth < rhsDepth;
		});
		for (auto& stagedMesh : stagedMeshes) {
			a_state.meshes.push_back(std::move(stagedMesh));
		}
		for (auto& mergedRoot : mergedRootNodes) {
			std::vector<MergeRename> subtreeRenameMap;
			for (const auto& renamedNode : mergedSkeletonNodes) {
				if (!renamedNode.node || renamedNode.originalName.empty() || renamedNode.renamedName.empty()) {
					continue;
				}
				auto* rootNode = mergedRoot.node ? mergedRoot.node->IsNode() : nullptr;
				if (renamedNode.node == rootNode || IsObjectDescendantOf(renamedNode.node, rootNode)) {
					subtreeRenameMap.push_back({
						.sourceName = renamedNode.originalName,
						.renamedName = renamedNode.renamedName,
					});
				}
			}
			a_state.mergedNodes.push_back({
				.buildGroup = buildGroup,
				.parent = mergedRoot.parent,
				.node = mergedRoot.node,
				.sourceName = mergedRoot.originalName,
				.recordParentName = mergedRoot.recordParentName,
				.localToParent = mergedRoot.localToParent,
				.recordLocalToParent = mergedRoot.recordLocalToParent,
				.hasLocalToParent = mergedRoot.hasLocalToParent,
				.hasRecordLocalToParent = mergedRoot.hasRecordLocalToParent,
				.recordMergeParentBinding = mergedRoot.recordMergeParentBinding,
				.subtreeRenameMap = std::move(subtreeRenameMap),
			});
		}
		mergedRootNodes.clear();
		for (auto& matchedBone : matchedBones) {
			auto body = std::ranges::find_if(a_state.bodies, [&matchedBone, buildGroup](const PrototypeBody& a_body) {
				return PrototypeBodyHasBuildGroup(a_body, buildGroup) && a_body.node == matchedBone.node && a_body.bone;
			});
			if (body == a_state.bodies.end() && !matchedBone.name.empty()) {
				body = std::ranges::find_if(a_state.bodies, [&matchedBone, buildGroup](const PrototypeBody& a_body) {
					return PrototypeBodyHasBuildGroup(a_body, buildGroup) && PhysicsNamesEqual(a_body.boneName, matchedBone.name) && a_body.bone;
				});
			}
			if (body == a_state.bodies.end()) {
				continue;
			}

			for (auto& skinWorld : matchedBone.skinWorldTransforms) {
				body->bone->AddSkinWorldTransform(
					skinWorld.skin.get(),
					skinWorld.index,
					buildGroup,
					skinWorld.originalBone,
					skinWorld.originalWorldTransform,
					skinWorld.originalRootNode);
			}
		}
		RestoreSavedLocalPoses(savedBuildPoses, actorRootNode);
		ResetPrototypeBuildGroupToCurrentPoseLocked(a_state, buildGroup);
		std::vector<PrototypeConstraint> stagedConstraints;
		BuildPrototypeConstraintsLocked(a_state, a_summary, buildGroup, a_domain, {}, stagedConstraints);
		for (auto& stagedConstraint : stagedConstraints) {
			a_state.constraints.push_back(std::move(stagedConstraint));
		}
		if (a_commitToBullet) {
			CommitPrototypeBuildGroupToBulletLocked(a_state, buildGroup);
			result.committed = true;
		} else {
			spdlog::debug(
				"staged prototype build group for delayed Bullet commit actor={} buildGroup={} domain={} bipedObject={}",
				static_cast<void*>(a_state.actor),
				buildGroup,
				PrototypeDomainName(a_domain),
				std::to_underlying(a_event.bipedObject));
		}
		timing.bindCommitConstraintMs += ElapsedMs(phaseStart, Clock::now());
		if (a_commitToBullet && actorRoot && spdlog::default_logger_raw() && spdlog::default_logger_raw()->should_log(spdlog::level::trace)) {
			LogObjectHierarchy(actorRoot, "actor-skeleton-after-prototype-build");
		}
		if (a_commitToBullet) {
			LogPrototypeActorBulletObjectsLocked(a_state, "after-prototype-build-commit");
		}
		result.recordable = PrototypeBuildGroupIsRecordableLocked(a_state, buildGroup, a_domain, a_event.bipedObject);
		result.succeeded = result.recordable;
		rollbackGuard.Dismiss();
		if (a_domain == PrototypeBuildDomain::kArmor && hadActorRuntimeBeforeBuild) {
			spdlog::debug(
				"hot armor build committed with localized reset read actor={} buildGroup={} because existing prototype runtime is active",
				static_cast<void*>(a_state.actor),
				buildGroup);
		}
		spdlog::debug(
			"prototype matched bone placement actorRoot={} attachedObject={} underActorRoot={} underAttachedObject={} matchedXMLBones={}",
			static_cast<void*>(actorRoot),
			static_cast<void*>(a_event.object),
			matchedUnderActorRoot,
			matchedUnderAttachedObject,
			matchedBones.size());
		logBuildTiming(result.succeeded ? (a_commitToBullet ? "success" : "success-pending-commit") : "not-recordable", buildGroup);
		return result;
	}

	void Fo4PhysicsWorld::LogPrototypeActorBulletObjectsLocked(const PrototypeActorState& a_state, const std::string_view a_reason) const
	{
		auto* logger = spdlog::default_logger_raw();
		if (!logger || !logger->should_log(spdlog::level::debug)) {
			return;
		}

		spdlog::debug(
			"begin actor bullet physics object dump actor={} firstPerson={} bodies={} meshes={} constraints={} reason={}",
			static_cast<void*>(a_state.actor),
			a_state.firstPerson,
			a_state.bodies.size(),
			a_state.meshes.size(),
			a_state.constraints.size(),
			a_reason);

		for (const auto& body : a_state.bodies) {
			if (!body.bone) {
				continue;
			}

			const auto& rig = body.bone->m_rig;
			const auto transform = rig.getWorldTransform();
			const auto origin = transform.getOrigin();
			const auto rotation = transform.getRotation();
			const auto* node = body.node;
			spdlog::debug(
				"actor bullet rigid body actor={} body={} buildGroup={} bipedObject={} boneName='{}' rigidBody={} motionState={} invMass={:.6f} kinematic={} pos=({:.3f},{:.3f},{:.3f}) rot=({:.6f},{:.6f},{:.6f},{:.6f}) writeTargetNode={} writeTargetWorld={} nodeName='{}'",
				static_cast<void*>(a_state.actor),
				static_cast<const void*>(std::addressof(body)),
				body.buildGroup,
				std::to_underlying(body.bipedObject),
				body.boneName,
				static_cast<const void*>(std::addressof(rig)),
				static_cast<const void*>(rig.getMotionState()),
				rig.getInvMass(),
				rig.isStaticOrKinematicObject(),
				origin.x(),
				origin.y(),
				origin.z(),
				rotation.x(),
				rotation.y(),
				rotation.z(),
				rotation.w(),
				static_cast<const void*>(node),
				node ? static_cast<const void*>(std::addressof(node->world)) : nullptr,
				node ? std::string_view(node->GetName()) : std::string_view{});
		}

		for (const auto& mesh : a_state.meshes) {
			if (!mesh.body) {
				continue;
			}

			const auto transform = mesh.body->getWorldTransform();
			const auto origin = transform.getOrigin();
			const auto rotation = transform.getRotation();
			const auto* geometry = IsProbablyValidNiObject(mesh.geometry) ? mesh.geometry : nullptr;
			spdlog::debug(
				"actor bullet mesh body actor={} meshBody={} buildGroup={} bipedObject={} meshName='{}' geometry={} geometryName='{}' pos=({:.3f},{:.3f},{:.3f}) rot=({:.6f},{:.6f},{:.6f},{:.6f})",
				static_cast<void*>(a_state.actor),
				static_cast<const void*>(mesh.body.get()),
				mesh.buildGroup,
				std::to_underlying(mesh.bipedObject),
				mesh.name,
				static_cast<const void*>(geometry),
				geometry ? std::string_view(geometry->GetName()) : std::string_view{},
				origin.x(),
				origin.y(),
				origin.z(),
				rotation.x(),
				rotation.y(),
				rotation.z(),
				rotation.w());
		}

		for (const auto& constraint : a_state.constraints) {
			if (!constraint.constraint) {
				continue;
			}

			const auto& bodyA = constraint.constraint->getRigidBodyA();
			const auto& bodyB = constraint.constraint->getRigidBodyB();
			const auto transformA = bodyA.getWorldTransform();
			const auto transformB = bodyB.getWorldTransform();
			const auto originA = transformA.getOrigin();
			const auto originB = transformB.getOrigin();
			spdlog::debug(
				"actor bullet constraint actor={} constraint={} buildGroup={} kind={} bodyA='{}' bodyB='{}' enabled={} bodyAkin={} bodyBkin={} bodyApos=({:.3f},{:.3f},{:.3f}) bodyBpos=({:.3f},{:.3f},{:.3f})",
				static_cast<void*>(a_state.actor),
				static_cast<const void*>(constraint.constraint.get()),
				constraint.buildGroup,
				std::to_underlying(constraint.kind),
				constraint.bodyA,
				constraint.bodyB,
				constraint.constraint->isEnabled(),
				bodyA.isStaticOrKinematicObject(),
				bodyB.isStaticOrKinematicObject(),
				originA.x(),
				originA.y(),
				originA.z(),
				originB.x(),
				originB.y(),
				originB.z());
		}

		spdlog::debug(
			"end actor bullet physics object dump actor={} firstPerson={} reason={}",
			static_cast<void*>(a_state.actor),
			a_state.firstPerson,
			a_reason);
	}

	void Fo4PhysicsWorld::ResetPrototypeBuildGroupToCurrentPoseLocked(
		PrototypeActorState& a_state,
		const std::uint64_t a_buildGroup,
		const std::span<PrototypeBody> a_stagedBodies)
	{
		auto isInBuildGroup = [a_buildGroup](const PrototypeBody& a_body) {
			return PrototypeBodyHasBuildGroup(a_body, a_buildGroup);
		};

		auto resetBodyToCurrentPose = [](PrototypeBody& a_body) {
			auto* updateRoot = a_body.node->parent ? static_cast<RE::NiAVObject*>(a_body.node->parent) : static_cast<RE::NiAVObject*>(a_body.node);
			UpdateTransformUpDown(updateRoot, true);
			a_body.bone->readTransform(0.0F);
			a_body.bone->RefreshSkinWorldTransforms();
		};

		for (auto& body : a_state.bodies) {
			if (!isInBuildGroup(body) || !body.bone || !body.node) {
				continue;
			}

			resetBodyToCurrentPose(body);
		}
		for (auto& body : a_stagedBodies) {
			if (!isInBuildGroup(body) || !body.bone || !body.node) {
				continue;
			}

			resetBodyToCurrentPose(body);
		}
	}

	void Fo4PhysicsWorld::ResetPrototypeBuildGroupsToStoredLocalPoseLocked(
		PrototypeActorState& a_state,
		const std::span<const std::uint64_t> a_buildGroups,
		const std::string_view a_reason)
	{
		if (a_buildGroups.empty()) {
			return;
		}

		const auto containsGroup = [&a_buildGroups](const std::uint64_t a_buildGroup) {
			return a_buildGroup != 0 && std::ranges::find(a_buildGroups, a_buildGroup) != a_buildGroups.end();
		};

		std::uint32_t restoredMergedNodes = 0;
		for (auto& mergedNode : a_state.mergedNodes) {
			if (!containsGroup(mergedNode.buildGroup) || !mergedNode.hasLocalToParent) {
				continue;
			}

			auto* node = mergedNode.node ? mergedNode.node->IsNode() : nullptr;
			if (!node) {
				continue;
			}

			node->local = mergedNode.localToParent;
			UpdateTransformUpDown(node, true);
			++restoredMergedNodes;
		}

		for (const auto buildGroup : a_buildGroups) {
			ResetPrototypeBuildGroupToCurrentPoseLocked(a_state, buildGroup);
		}

		spdlog::debug(
			"reset prototype build groups to stored local pose actor={} groups={} restoredMergedNodes={} reason={}",
			static_cast<void*>(a_state.actor),
			a_buildGroups.size(),
			restoredMergedNodes,
			a_reason);
	}

	bool Fo4PhysicsWorld::BuildPrototypeMeshesLocked(
		PrototypeActorState& a_state,
		const PhysicsXmlSummary& a_summary,
		const LifecycleEvent& a_event,
		const DefaultBBP::NameMap& a_meshNameMap,
		const std::uint64_t a_buildGroup,
		const PrototypeBuildDomain a_domain,
		const std::unordered_set<RE::NiAVObject*>& a_trustedActorSkeletonNodes,
		const std::vector<RE::NiAVObject*>& a_mergedSkeletonNodes,
		std::vector<PrototypeBody>& a_stagedBodies,
		std::vector<PrototypeMesh>& a_stagedMeshes)
	{
		if (!dynamicsWorld_ || a_summary.meshDescriptors.empty()) {
			return false;
		}

		auto meshNames = BuildMeshMatchNames(a_summary, a_meshNameMap);

		auto extraction = ExtractSkinnedMeshes(a_event.object, meshNames);
		auto cpuCopyPending = HasPendingCpuCopyExtraction(extraction);
		auto pendingMatchedGeometries = cpuCopyPending ? extraction.stats.matchedGeometries : 0U;
		auto pendingVertexCopies = cpuCopyPending ? extraction.stats.pendingVertexCopies : 0U;
		auto pendingIndexCopies = cpuCopyPending ? extraction.stats.pendingIndexCopies : 0U;
		const char* extractionSource = "attached-object";
		const auto attachedDynamicBones = CountDynamicDecodedSkinBones(extraction, a_summary);
		if (extraction.meshes.empty()) {
			const std::array fallbackRoots{
				std::pair{ "merge-source", a_event.mergeSourceObject },
				std::pair{ "source-object", a_event.sourceObject },
				std::pair{ "source-root", static_cast<RE::NiAVObject*>(a_event.sourceRoot) },
			};
			for (const auto& [sourceName, root] : fallbackRoots) {
				if (!root || root == a_event.object) {
					continue;
				}

				auto fallbackExtraction = ExtractSkinnedMeshes(root, meshNames);
				const auto fallbackCpuCopyPending = HasPendingCpuCopyExtraction(fallbackExtraction);
				if (fallbackCpuCopyPending) {
					cpuCopyPending = true;
					pendingMatchedGeometries += fallbackExtraction.stats.matchedGeometries;
					pendingVertexCopies += fallbackExtraction.stats.pendingVertexCopies;
					pendingIndexCopies += fallbackExtraction.stats.pendingIndexCopies;
				}
				if (fallbackExtraction.meshes.empty()) {
					spdlog::debug(
						"prototype mesh extraction fallback {} root={} produced no meshes for actor={} geometries={} skinned={} matched={} missingCpuVertexData={} missingPositionData={} splitPositionData={} nonFinitePositions={} invalidCpuVertexData={} pendingVertexCopies={} pendingIndexCopies={}",
						sourceName,
						static_cast<void*>(root),
						static_cast<void*>(a_state.actor),
						fallbackExtraction.stats.geometries,
						fallbackExtraction.stats.skinnedGeometries,
						fallbackExtraction.stats.matchedGeometries,
						fallbackExtraction.stats.missingCpuVertexData,
						fallbackExtraction.stats.missingPositionData,
						fallbackExtraction.stats.splitPositionData,
						fallbackExtraction.stats.nonFinitePositions,
						fallbackExtraction.stats.invalidCpuVertexData,
						fallbackExtraction.stats.pendingVertexCopies,
						fallbackExtraction.stats.pendingIndexCopies);
					continue;
				}

				const auto fallbackDynamicBones = CountDynamicDecodedSkinBones(fallbackExtraction, a_summary);
				if (!extraction.meshes.empty() && fallbackDynamicBones <= attachedDynamicBones) {
					spdlog::debug(
						"prototype mesh extraction kept attached-object over {} root={} for actor={} attachedDynamicBones={} fallbackDynamicBones={} decodedMeshes={}",
						sourceName,
						static_cast<void*>(root),
						static_cast<void*>(a_state.actor),
						attachedDynamicBones,
						fallbackDynamicBones,
						fallbackExtraction.stats.decodedMeshes);
					continue;
				}

				extraction = std::move(fallbackExtraction);
				extractionSource = sourceName;
				spdlog::debug(
					"prototype mesh extraction using {} root={} for actor={} decodedMeshes={} attachedDynamicBones={} fallbackDynamicBones={}",
					sourceName,
					static_cast<void*>(root),
					static_cast<void*>(a_state.actor),
					extraction.stats.decodedMeshes,
					attachedDynamicBones,
					fallbackDynamicBones);
				break;
			}
		}
		if (extraction.meshes.empty() && cpuCopyPending) {
			spdlog::debug(
				"prototype mesh extraction delayed for pending CPU copy actor={} object={} matched={} pendingVertexCopies={} pendingIndexCopies={}",
				static_cast<void*>(a_state.actor),
				static_cast<void*>(a_event.object),
				pendingMatchedGeometries,
				pendingVertexCopies,
				pendingIndexCopies);
			return true;
		}
		if (a_domain == PrototypeBuildDomain::kArmor && cpuCopyPending) {
			spdlog::debug(
				"prototype armor mesh extraction delayed for partial pending CPU copy actor={} object={} decodedMeshes={} matched={} pendingVertexCopies={} pendingIndexCopies={}",
				static_cast<void*>(a_state.actor),
				static_cast<void*>(a_event.object),
				extraction.stats.decodedMeshes,
				pendingMatchedGeometries,
				pendingVertexCopies,
				pendingIndexCopies);
			return true;
		}
		std::uint32_t created = 0;
		std::uint32_t skippedMissingBones = 0;
		std::uint32_t skippedMissingBoneData = 0;
		std::uint32_t sanitizedBadBoneMeshes = 0;
		std::uint32_t skippedMissingTriangleIndices = 0;
		std::uint32_t skippedInvalidTriangleIndices = 0;
		std::uint32_t skippedEmpty = 0;
		std::uint32_t skippedNoColliders = 0;
		std::uint32_t skippedExisting = 0;
		std::uint32_t createdFallbackSkinBones = 0;
		std::uint32_t unresolvedCanCollideBones = 0;
		std::uint32_t unresolvedNoCollideBones = 0;
		std::uint32_t unresolvedWeightThresholds = 0;
		auto findPrototypeBody = [&](const Smp::Fo4DecodedSkinBone& a_decodedBone) -> PrototypeBody* {
			auto committedBody = std::ranges::find_if(a_state.bodies, [&a_decodedBone, a_buildGroup](const PrototypeBody& a_body) {
				return a_decodedBone.node && PrototypeBodyHasBuildGroup(a_body, a_buildGroup) && a_body.node == a_decodedBone.node;
			});
			if (committedBody == a_state.bodies.end() && !a_decodedBone.name.empty()) {
				committedBody = std::ranges::find_if(a_state.bodies, [&a_decodedBone, a_buildGroup](const PrototypeBody& a_body) {
					return PrototypeBodyHasBuildGroup(a_body, a_buildGroup) && PhysicsNamesEqual(a_body.boneName, a_decodedBone.name);
				});
			}
			if (committedBody != a_state.bodies.end()) {
				return std::addressof(*committedBody);
			}

			auto stagedBody = std::ranges::find_if(a_stagedBodies, [&a_decodedBone, a_buildGroup](const PrototypeBody& a_body) {
				return a_decodedBone.node && PrototypeBodyHasBuildGroup(a_body, a_buildGroup) && a_body.node == a_decodedBone.node;
			});
			if (stagedBody == a_stagedBodies.end() && !a_decodedBone.name.empty()) {
				stagedBody = std::ranges::find_if(a_stagedBodies, [&a_decodedBone, a_buildGroup](const PrototypeBody& a_body) {
					return PrototypeBodyHasBuildGroup(a_body, a_buildGroup) && PhysicsNamesEqual(a_body.boneName, a_decodedBone.name);
				});
			}
			return stagedBody != a_stagedBodies.end() ? std::addressof(*stagedBody) : nullptr;
		};
		auto isAllowedArmorFallbackSkinBone = [&](RE::NiNode* a_node) {
			if (a_domain != PrototypeBuildDomain::kArmor) {
				return true;
			}
			if (!a_node) {
				return false;
			}
			if (a_trustedActorSkeletonNodes.contains(a_node)) {
				return true;
			}
			return std::ranges::find(a_mergedSkeletonNodes, static_cast<RE::NiAVObject*>(a_node)) != a_mergedSkeletonNodes.end();
		};
		auto* fallbackActorRoot = ResolveSkeletonSearchRoot(a_event);
		const auto fallbackActorSearchExclusions = BuildBipedPartCloneExclusions(a_event);
		const auto fallbackKnownArmorNodes = BuildKnownArmorNodeSet(a_event);
		const auto fallbackLookupMode =
			a_domain == PrototypeBuildDomain::kArmor && !a_trustedActorSkeletonNodes.empty() ?
				ActorSkeletonLookupMode::kAuthoritativeOnly :
				ActorSkeletonLookupMode::kPermissiveNonArmor;
		auto findMergedFallbackSkinNode = [&](const std::string& a_sourceName) -> RE::NiNode* {
			if (a_sourceName.empty()) {
				return nullptr;
			}

			auto renamedName = std::string_view{};
			const auto rename = std::ranges::find_if(a_event.mergeRenameMap, [&a_sourceName](const MergeRename& a_entry) {
				return PhysicsNamesEqual(a_entry.sourceName, a_sourceName);
			});
			if (rename != a_event.mergeRenameMap.end()) {
				renamedName = rename->renamedName;
			}

			for (auto* object : a_mergedSkeletonNodes) {
				auto* node = object ? object->IsNode() : nullptr;
				if (!node) {
					continue;
				}
				const auto nodeName = node->GetName();
				if ((!renamedName.empty() && PhysicsNamesEqual(nodeName, renamedName)) || PhysicsNamesEqual(nodeName, a_sourceName)) {
					return node;
				}
			}
			return nullptr;
		};
		auto findActorFallbackSkinNode = [&](const std::string& a_name) -> RE::NiNode* {
			if (a_name.empty()) {
				return nullptr;
			}

			for (auto* object : a_trustedActorSkeletonNodes) {
				auto* node = object ? object->IsNode() : nullptr;
				if (node && PhysicsNamesEqual(node->GetName(), a_name)) {
					return node;
				}
			}
			auto* node = FindNodeByNameExcludingKnownNodes(
				fallbackActorRoot,
				a_name,
				fallbackActorSearchExclusions,
				fallbackKnownArmorNodes);
			return IsActorSkeletonCandidate(node, a_trustedActorSkeletonNodes, fallbackLookupMode) ? node : nullptr;
		};
		auto resolveFallbackSkinNode = [&](const Smp::Fo4DecodedSkinBone& a_decodedBone) -> RE::NiNode* {
			if (isAllowedArmorFallbackSkinBone(a_decodedBone.node)) {
				return a_decodedBone.node;
			}
			if (auto* mergedNode = findMergedFallbackSkinNode(a_decodedBone.name)) {
				return mergedNode;
			}
			return findActorFallbackSkinNode(a_decodedBone.name);
		};
		auto createFallbackSkinBody = [&](const Smp::Fo4DecodedSkinBone& a_decodedBone, const std::string& a_meshName) -> PrototypeBody* {
			if (a_decodedBone.name.empty()) {
				return nullptr;
			}
			auto* fallbackNode = resolveFallbackSkinNode(a_decodedBone);
			if (!fallbackNode) {
				spdlog::debug(
					"skipping fallback kinematic skin bone for armor mesh '{}' actor={} bone='{}' node={} nodeName='{}' buildGroup={} because no trusted actor skeleton node or plugin-owned merged clone was resolved",
					a_meshName,
					static_cast<void*>(a_event.actor),
					a_decodedBone.name,
					static_cast<void*>(a_decodedBone.node),
					a_decodedBone.node ? std::string_view(a_decodedBone.node->GetName()) : std::string_view{},
					a_buildGroup);
				return nullptr;
			}

			auto shape = std::make_unique<btEmptyShape>();
			btVector3 localInertia(0.0F, 0.0F, 0.0F);
			const auto localToRig = btTransform::getIdentity();
			const auto& initialWorld = fallbackNode->world;
			auto motionState = std::make_unique<btDefaultMotionState>(Smp::Fo4Transform::ToBulletTransform(initialWorld) * localToRig);
			btRigidBody::btRigidBodyConstructionInfo constructionInfo(0.0F, motionState.get(), shape.get(), localInertia);
			auto bone = std::make_unique<Fo4SkinnedMeshBone>(RE::BSFixedString(a_decodedBone.name), fallbackNode, constructionInfo);
			bone->m_localToRig = localToRig;
			bone->m_rigToLocal = localToRig.inverse();
			bone->m_marginMultipler = 1.0F;
			bone->m_gravityFactor = 0.0F;
			bone->m_windFactor = 0.0F;
			bone->m_rig.setGravity(btVector3(0.0F, 0.0F, kGravityAcceleration * bone->m_gravityFactor));
			bone->readTransform(0.0F);
			bone->m_rig.setActivationState(DISABLE_DEACTIVATION);

			PrototypeBody prototypeBody;
			prototypeBody.actor = a_event.actor;
			prototypeBody.node = fallbackNode;
			prototypeBody.buildGroup = a_buildGroup;
			prototypeBody.bipedObject = a_event.bipedObject;
			AddPrototypeBodyBuildGroup(prototypeBody, a_buildGroup, a_domain, a_event.bipedObject);
			prototypeBody.boneName = a_decodedBone.name;
			prototypeBody.shape = std::move(shape);
			prototypeBody.motionState = std::move(motionState);
			prototypeBody.bone = std::move(bone);
			prototypeBody.meshOnlySkinBone = true;
			a_stagedBodies.push_back(std::move(prototypeBody));
			++createdFallbackSkinBones;
			if (auto* logger = spdlog::default_logger_raw(); logger && logger->should_log(spdlog::level::debug)) {
				spdlog::debug(
					"created mesh-only kinematic skin bone for mesh '{}' actor={} bone='{}' node={} nodeName='{}' rawNode={} rawNodeName='{}' buildGroup={} because weighted skin bone was not resolved through XML bodies",
					a_meshName,
					static_cast<void*>(a_event.actor),
					a_decodedBone.name,
					static_cast<void*>(fallbackNode),
					std::string_view(fallbackNode->GetName()),
					static_cast<void*>(a_decodedBone.node),
					a_decodedBone.node ? std::string_view(a_decodedBone.node->GetName()) : std::string_view{},
					a_buildGroup);
			}
			return std::addressof(a_stagedBodies.back());
		};
		struct MatchedDescriptorMesh
		{
			const Smp::Fo4DecodedSkinnedMesh* mesh{ nullptr };
			std::uint32_t vertexOffset{ 0 };
			std::size_t boneOffset{ 0 };
		};

		auto hasWeightedBoneWithoutBindData = [](const Smp::Fo4DecodedSkinnedMesh& a_decodedMesh) {
			std::size_t vertexIndex = 0;
			for (const auto& vertex : a_decodedMesh.vertices) {
				for (int influence = 0; influence < 4; ++influence) {
					const auto boneIndex = static_cast<std::size_t>(vertex.getBoneIdx(influence));
					if (vertex.weight_[influence] > FLT_EPSILON &&
						boneIndex < a_decodedMesh.bones.size() &&
						!a_decodedMesh.bones[boneIndex].hasSkinToBone) {
						const auto& decodedBone = a_decodedMesh.bones[boneIndex];
						spdlog::warn(
							"mesh '{}' bind-pose miss vertex={} influence={} weight={} boneIndex={} boneCount={} boneNode={} boneName='{}' hasBoneData={}",
							a_decodedMesh.name,
							vertexIndex,
							influence,
							vertex.weight_[influence],
							boneIndex,
							a_decodedMesh.bones.size(),
							static_cast<void*>(decodedBone.node),
							decodedBone.name,
							decodedBone.hasBoneData);
						return true;
					}
				}
				++vertexIndex;
			}
			return false;
		};

		for (std::size_t meshDescriptorIndex = 0; meshDescriptorIndex < a_summary.meshDescriptors.size(); ++meshDescriptorIndex) {
			const auto* meshDescriptor = std::addressof(a_summary.meshDescriptors[meshDescriptorIndex]);
			const auto existingMesh = std::ranges::find_if(a_state.meshes, [a_buildGroup, meshDescriptorIndex](const PrototypeMesh& a_mesh) {
				return a_mesh.buildGroup == a_buildGroup && a_mesh.descriptorIndex == meshDescriptorIndex;
			});
			if (existingMesh != a_state.meshes.end()) {
				++skippedExisting;
				continue;
			}
			const auto existingStagedMesh = std::ranges::find_if(a_stagedMeshes, [a_buildGroup, meshDescriptorIndex](const PrototypeMesh& a_mesh) {
				return a_mesh.buildGroup == a_buildGroup && a_mesh.descriptorIndex == meshDescriptorIndex;
			});
			if (existingStagedMesh != a_stagedMeshes.end()) {
				++skippedExisting;
				continue;
			}

			std::vector<MatchedDescriptorMesh> matchedMeshes;
			std::uint32_t vertexOffset = 0;
			std::size_t boneOffset = 0;
			bool missingTriangleIndices = false;
			bool missingBoneData = false;
			for (const auto& decodedMesh : extraction.meshes) {
				if (!MeshNameMatches(meshDescriptor->name, decodedMesh.name, a_meshNameMap)) {
					continue;
				}
				if (decodedMesh.vertices.empty()) {
					++skippedEmpty;
					continue;
				}
				if (decodedMesh.badBoneIndices > 0) {
					++sanitizedBadBoneMeshes;
					spdlog::debug("mesh '{}' discarded {} unusable vertex bone influences during decode", decodedMesh.name, decodedMesh.badBoneIndices);
				}
				if (hasWeightedBoneWithoutBindData(decodedMesh)) {
					missingBoneData = true;
					break;
				}
				if (meshDescriptor->kind == PhysicsMeshShapeKind::kPerTriangle && decodedMesh.indices.size() < 3) {
					missingTriangleIndices = true;
					spdlog::warn("skipping per-triangle mesh '{}' descriptorIndex={} because no usable CPU index buffer was decoded", decodedMesh.name, meshDescriptorIndex);
					break;
				}

				matchedMeshes.push_back({ std::addressof(decodedMesh), vertexOffset, boneOffset });
				vertexOffset += static_cast<std::uint32_t>(decodedMesh.vertices.size());
				boneOffset += decodedMesh.bones.size();
			}

			if (matchedMeshes.empty()) {
				++skippedNoColliders;
				spdlog::debug("skipping mesh descriptor '{}' because no decoded mesh matched", meshDescriptor->name);
				continue;
			}
			if (missingBoneData) {
				++skippedMissingBoneData;
				spdlog::warn("skipping mesh descriptor '{}' because a weighted skin bone is missing bind-pose data", meshDescriptor->name);
				continue;
			}
			if (missingTriangleIndices) {
				++skippedMissingTriangleIndices;
				continue;
			}

			auto meshBody = RE::make_smart<hdt::SkinnedMeshBody>();
			meshBody->name_ = RE::BSFixedString(meshDescriptor->name);
			meshBody->actor_ = a_state.actor;
			meshBody->buildGroup_ = a_buildGroup;
			meshBody->vertices_.reserve(vertexOffset);

			for (const auto& matchedMesh : matchedMeshes) {
				const auto& decodedMesh = *matchedMesh.mesh;
				for (auto vertex : decodedMesh.vertices) {
					for (int influence = 0; influence < 4; ++influence) {
						if (vertex.weight_[influence] > FLT_EPSILON) {
							vertex.setBoneIdx(influence, vertex.getBoneIdx(influence) + static_cast<std::uint32_t>(matchedMesh.boneOffset));
						}
					}
					meshBody->vertices_.push_back(vertex);
				}
			}

			bool missingResolvedSkinBone = false;
			for (const auto& matchedMesh : matchedMeshes) {
				const auto& decodedMesh = *matchedMesh.mesh;
				for (std::size_t boneIndex = 0; boneIndex < decodedMesh.bones.size(); ++boneIndex) {
					const auto& decodedBone = decodedMesh.bones[boneIndex];
					PrototypeBody* matchedBody = findPrototypeBody(decodedBone);
					const auto weightedBone = std::ranges::any_of(decodedMesh.vertices, [boneIndex](const hdt::Vertex& a_vertex) {
						for (int influence = 0; influence < 4; ++influence) {
							if (a_vertex.weight_[influence] > FLT_EPSILON && a_vertex.getBoneIdx(influence) == boneIndex) {
								return true;
							}
						}
						return false;
					});

					if (!matchedBody || !matchedBody->bone) {
						if (weightedBone) {
							matchedBody = createFallbackSkinBody(decodedBone, decodedMesh.name);
							if (!matchedBody || !matchedBody->bone) {
								++skippedMissingBones;
								spdlog::warn(
									"mesh '{}' could not create mesh-only fallback body for weighted skin bone '{}' node={} because no trusted actor skeleton node or plugin-owned merged clone was resolved",
									decodedMesh.name,
									decodedBone.name,
									static_cast<void*>(decodedBone.node));
								missingResolvedSkinBone = true;
								break;
							}
						}
						if (!matchedBody || !matchedBody->bone) {
							meshBody->addBone(
								nullptr,
								decodedBone.hasSkinToBone ? decodedBone.skinToBone : hdt::btQsTransform::getIdentity(),
								decodedBone.hasBoneData ? decodedBone.boundingSphere : hdt::BoundingSphere(btVector3(0.0F, 0.0F, 0.0F), 0.0F));
							continue;
						}
					}

					const auto sphere = decodedBone.hasBoneData ?
						decodedBone.boundingSphere :
						CalculateBoneSphere(decodedMesh, boneIndex).value_or(hdt::BoundingSphere(btVector3(0.0F, 0.0F, 0.0F), 0.0F));
					meshBody->addBone(matchedBody->bone.get(), decodedBone.hasSkinToBone ? decodedBone.skinToBone : hdt::btQsTransform::getIdentity(), sphere);
				}
				if (missingResolvedSkinBone) {
					break;
				}
			}
			if (missingResolvedSkinBone) {
				continue;
			}

			switch (meshDescriptor->shared) {
				case PhysicsMeshSharedScope::kInternal:
					meshBody->shared_ = hdt::SkinnedMeshBody::SharedType::kInternal;
					break;
				case PhysicsMeshSharedScope::kExternal:
					meshBody->shared_ = hdt::SkinnedMeshBody::SharedType::kExternal;
					break;
				case PhysicsMeshSharedScope::kPrivate:
					meshBody->shared_ = hdt::SkinnedMeshBody::SharedType::kPrivate;
					break;
				case PhysicsMeshSharedScope::kPublic:
				default:
					meshBody->shared_ = hdt::SkinnedMeshBody::SharedType::kPublic;
					break;
				}
			for (const auto& tag : meshDescriptor->tags) {
				meshBody->tags_.emplace_back(tag.c_str());
			}
			for (const auto& tag : meshDescriptor->canCollideWithTags) {
				meshBody->canCollideWithTags_.emplace_back(tag.c_str());
			}
			for (const auto& tag : meshDescriptor->noCollideWithTags) {
				meshBody->noCollideWithTags_.emplace_back(tag.c_str());
			}
			meshBody->disableTag_ = meshDescriptor->disableTag;
			meshBody->disablePriority_ = meshDescriptor->disablePriority;
			for (const auto& boneName : meshDescriptor->canCollideWithBones) {
				auto matchedBody = std::ranges::find_if(a_state.bodies, [&boneName, a_buildGroup](const PrototypeBody& a_body) {
					return PrototypeBodyHasBuildGroup(a_body, a_buildGroup) && PhysicsNamesEqual(a_body.boneName, boneName);
				});
				PrototypeBody* resolvedBody = matchedBody != a_state.bodies.end() ? std::addressof(*matchedBody) : nullptr;
				if (!resolvedBody) {
					const auto stagedBody = std::ranges::find_if(a_stagedBodies, [&boneName, a_buildGroup](const PrototypeBody& a_body) {
						return PrototypeBodyHasBuildGroup(a_body, a_buildGroup) && PhysicsNamesEqual(a_body.boneName, boneName);
					});
					if (stagedBody != a_stagedBodies.end()) {
						resolvedBody = std::addressof(*stagedBody);
					}
				}
				if (resolvedBody && resolvedBody->bone) {
					meshBody->canCollideWithBones_.push_back(resolvedBody->bone.get());
				} else {
					++unresolvedCanCollideBones;
					spdlog::debug("mesh '{}' could not resolve can-collide-with-bone '{}'", meshDescriptor->name, boneName);
				}
			}
			for (const auto& boneName : meshDescriptor->noCollideWithBones) {
				auto matchedBody = std::ranges::find_if(a_state.bodies, [&boneName, a_buildGroup](const PrototypeBody& a_body) {
					return PrototypeBodyHasBuildGroup(a_body, a_buildGroup) && PhysicsNamesEqual(a_body.boneName, boneName);
				});
				PrototypeBody* resolvedBody = matchedBody != a_state.bodies.end() ? std::addressof(*matchedBody) : nullptr;
				if (!resolvedBody) {
					const auto stagedBody = std::ranges::find_if(a_stagedBodies, [&boneName, a_buildGroup](const PrototypeBody& a_body) {
						return PrototypeBodyHasBuildGroup(a_body, a_buildGroup) && PhysicsNamesEqual(a_body.boneName, boneName);
					});
					if (stagedBody != a_stagedBodies.end()) {
						resolvedBody = std::addressof(*stagedBody);
					}
				}
				if (resolvedBody && resolvedBody->bone) {
					meshBody->noCollideWithBones_.push_back(resolvedBody->bone.get());
				} else {
					++unresolvedNoCollideBones;
					spdlog::debug("mesh '{}' could not resolve no-collide-with-bone '{}'", meshDescriptor->name, boneName);
				}
			}
			for (const auto& [boneName, threshold] : meshDescriptor->weightThresholds) {
				bool appliedThreshold = false;
				for (const auto& matchedMesh : matchedMeshes) {
					const auto& decodedMesh = *matchedMesh.mesh;
					for (std::size_t boneIndex = 0; boneIndex < decodedMesh.bones.size(); ++boneIndex) {
						const auto skinnedBoneIndex = matchedMesh.boneOffset + boneIndex;
						if (skinnedBoneIndex >= meshBody->skinnedBones_.size()) {
							continue;
						}
						if (PhysicsNamesEqual(decodedMesh.bones[boneIndex].name, boneName)) {
							meshBody->skinnedBones_[skinnedBoneIndex].weightThreshold = threshold;
							appliedThreshold = true;
						}
					}
				}
				if (!appliedThreshold) {
					for (auto& skinnedBone : meshBody->skinnedBones_) {
						if (skinnedBone.ptr && PhysicsNamesEqual(skinnedBone.ptr->m_name, boneName)) {
							skinnedBone.weightThreshold = threshold;
							appliedThreshold = true;
						}
					}
				}
				if (!appliedThreshold) {
					for (const auto& prototypeBody : a_state.bodies) {
						if (!prototypeBody.bone || !PhysicsNamesEqual(prototypeBody.boneName, boneName)) {
							continue;
						}
						for (auto& skinnedBone : meshBody->skinnedBones_) {
							if (skinnedBone.ptr == prototypeBody.bone.get()) {
								skinnedBone.weightThreshold = threshold;
								appliedThreshold = true;
							}
						}
					}
					for (const auto& prototypeBody : a_stagedBodies) {
						if (!prototypeBody.bone || !PhysicsNamesEqual(prototypeBody.boneName, boneName)) {
							continue;
						}
						for (auto& skinnedBone : meshBody->skinnedBones_) {
							if (skinnedBone.ptr == prototypeBody.bone.get()) {
								skinnedBone.weightThreshold = threshold;
								appliedThreshold = true;
							}
						}
					}
				}
				if (!appliedThreshold) {
					++unresolvedWeightThresholds;
					spdlog::debug("mesh '{}' could not resolve weight-threshold bone '{}'", meshDescriptor->name, boneName);
				}
			}

			if (meshDescriptor->kind == PhysicsMeshShapeKind::kPerTriangle) {
				auto* shape = new hdt::PerTriangleShape(meshBody.get());
				if (meshDescriptor->hasMargin) {
					shape->shapeProp_.margin = meshDescriptor->margin;
				}
				if (meshDescriptor->hasPenetration) {
					shape->shapeProp_.penetration = meshDescriptor->penetration;
				}
				for (const auto& matchedMesh : matchedMeshes) {
					const auto& decodedMesh = *matchedMesh.mesh;
					for (std::size_t index = 0; index + 2 < decodedMesh.indices.size(); index += 3) {
						if (decodedMesh.indices[index] >= decodedMesh.vertices.size() ||
							decodedMesh.indices[index + 1] >= decodedMesh.vertices.size() ||
							decodedMesh.indices[index + 2] >= decodedMesh.vertices.size()) {
							++skippedInvalidTriangleIndices;
							continue;
						}
						shape->addTriangle(
							static_cast<int>(decodedMesh.indices[index] + matchedMesh.vertexOffset),
							static_cast<int>(decodedMesh.indices[index + 1] + matchedMesh.vertexOffset),
							static_cast<int>(decodedMesh.indices[index + 2] + matchedMesh.vertexOffset));
					}
				}
			} else {
				auto* shape = new hdt::PerVertexShape(meshBody.get());
				if (meshDescriptor->hasMargin) {
					shape->shapeProp_.margin = meshDescriptor->margin;
				}
				shape->autoGen();
			}

			meshBody->finishBuild();
			if (!meshBody->shape_ || meshBody->shape_->colliders_.empty() || meshBody->vertices_.empty()) {
				++skippedNoColliders;
				continue;
			}

			meshBody->internalUpdate();
			const auto rawVertexStats = CalculateSkinVertexStats(meshBody->vertices_);
			const auto skinnedVertexStats = CalculateVertexPositionStats(meshBody->vertexPositions_);
			const auto skinnedBoneStats = CalculateSkinnedBoneStats(*meshBody);
			const auto* primaryMesh = matchedMeshes.front().mesh;
			const auto skinRootTransform = primaryMesh && primaryMesh->skinRootNode ?
				Smp::Fo4Transform::ToBulletQsTransformNormalizedScale(primaryMesh->skinRootNode->world) :
				hdt::btQsTransform::getIdentity();
			const auto geometryTransform = primaryMesh && primaryMesh->geometry ?
				Smp::Fo4Transform::ToBulletQsTransformNormalizedScale(primaryMesh->geometry->world) :
				hdt::btQsTransform::getIdentity();
			const auto rawCenterThroughSkinRoot = skinRootTransform * rawVertexStats.center;
			const auto rawCenterThroughGeometry = geometryTransform * rawVertexStats.center;
			const auto actorPosition = a_state.actor ? a_state.actor->GetPosition() : RE::NiPoint3{};
			spdlog::debug(
				"mesh coordinate diagnostic actor={} mesh='{}' buildGroup={} domain={} samples(raw={}, skinned={}, bones={}) actorPos=({:.3f},{:.3f},{:.3f}) geometry={} geometryName='{}' geometryWorld=({:.3f},{:.3f},{:.3f}) skinRoot={} skinRootName='{}' skinRootWorld=({:.3f},{:.3f},{:.3f}) rawCenter=({:.3f},{:.3f},{:.3f}) rawAabbCenter=({:.3f},{:.3f},{:.3f}) rawCenterViaGeometry=({:.3f},{:.3f},{:.3f}) rawCenterViaSkinRoot=({:.3f},{:.3f},{:.3f}) vertexCenter=({:.3f},{:.3f},{:.3f}) vertexAabbCenter=({:.3f},{:.3f},{:.3f}) boneCenter=({:.3f},{:.3f},{:.3f}) boneAabbCenter=({:.3f},{:.3f},{:.3f}) objectOrigin=({:.3f},{:.3f},{:.3f})",
				static_cast<void*>(a_state.actor),
				meshDescriptor->name,
				a_buildGroup,
				PrototypeDomainName(a_domain),
				rawVertexStats.samples,
				skinnedVertexStats.samples,
				skinnedBoneStats.samples,
				actorPosition.x,
				actorPosition.y,
				actorPosition.z,
				static_cast<void*>(primaryMesh ? primaryMesh->geometry : nullptr),
				primaryMesh && IsProbablyValidNiObject(primaryMesh->geometry) ? std::string_view(primaryMesh->geometry->GetName()) : std::string_view{},
				geometryTransform.getOrigin().x(),
				geometryTransform.getOrigin().y(),
				geometryTransform.getOrigin().z(),
				static_cast<void*>(primaryMesh ? primaryMesh->skinRootNode : nullptr),
				primaryMesh && primaryMesh->skinRootNode ? std::string_view(primaryMesh->skinRootNode->GetName()) : std::string_view{},
				skinRootTransform.getOrigin().x(),
				skinRootTransform.getOrigin().y(),
				skinRootTransform.getOrigin().z(),
				rawVertexStats.center.x(),
				rawVertexStats.center.y(),
				rawVertexStats.center.z(),
				rawVertexStats.aabbCenter.x(),
				rawVertexStats.aabbCenter.y(),
				rawVertexStats.aabbCenter.z(),
				rawCenterThroughGeometry.x(),
				rawCenterThroughGeometry.y(),
				rawCenterThroughGeometry.z(),
				rawCenterThroughSkinRoot.x(),
				rawCenterThroughSkinRoot.y(),
				rawCenterThroughSkinRoot.z(),
				skinnedVertexStats.center.x(),
				skinnedVertexStats.center.y(),
				skinnedVertexStats.center.z(),
				skinnedVertexStats.aabbCenter.x(),
				skinnedVertexStats.aabbCenter.y(),
				skinnedVertexStats.aabbCenter.z(),
				skinnedBoneStats.center.x(),
				skinnedBoneStats.center.y(),
				skinnedBoneStats.center.z(),
				skinnedBoneStats.aabbCenter.x(),
				skinnedBoneStats.aabbCenter.y(),
				skinnedBoneStats.aabbCenter.z(),
				meshBody->getWorldTransform().getOrigin().x(),
				meshBody->getWorldTransform().getOrigin().y(),
				meshBody->getWorldTransform().getOrigin().z());

			PrototypeMesh prototypeMesh;
			prototypeMesh.name = meshDescriptor->name;
			prototypeMesh.geometry = primaryMesh && IsProbablyValidNiObject(primaryMesh->geometry) ? primaryMesh->geometry : nullptr;
			prototypeMesh.buildGroup = a_buildGroup;
			prototypeMesh.descriptorIndex = meshDescriptorIndex;
			prototypeMesh.bipedObject = a_event.bipedObject;
			prototypeMesh.domain = a_domain;
			prototypeMesh.body = std::move(meshBody);
			a_stagedMeshes.push_back(std::move(prototypeMesh));
			++created;
		}

		if (a_domain != PrototypeBuildDomain::kArmor) {
			spdlog::info(
				"created {} {} prototype skinned mesh bodies for actor={} extractionSource={} from decodedMeshes={} geometries={} skinnedGeometries={} matchedGeometries={} decodedVertices={} decodedTriangles={} skippedExisting={} skippedEmpty={} skippedMissingBones={} skippedMissingBoneData={} sanitizedBadBoneMeshes={} skippedMissingTriangleIndices={} skippedInvalidTriangleIndices={} skippedNoColliders={} fallbackSkinBones={} unresolvedCanCollideBones={} unresolvedNoCollideBones={} unresolvedWeightThresholds={} nullBones={} nonNodeBones={} missingBoneData={} unsupportedGeometryClasses={} missingRendererData={} missingVertexBuffer={} missingIndexBuffer={} missingCpuVertexData={} missingPositionData={} splitPositionData={} faceGenPositionData={} nonFinitePositions={} invalidCpuVertexData={} pendingVertexCopies={} missingCpuIndexData={} invalidCpuIndexData={} pendingIndexCopies={} undersizedVertexBuffers={} undersizedIndexBuffers={} badBoneIndices={}",
				created,
				PrototypeDomainName(a_domain),
				static_cast<void*>(a_state.actor),
				extractionSource,
				extraction.stats.decodedMeshes,
				extraction.stats.geometries,
				extraction.stats.skinnedGeometries,
				extraction.stats.matchedGeometries,
				extraction.stats.decodedVertices,
				extraction.stats.decodedTriangles,
				skippedExisting,
				skippedEmpty,
				skippedMissingBones,
				skippedMissingBoneData,
				sanitizedBadBoneMeshes,
				skippedMissingTriangleIndices,
				skippedInvalidTriangleIndices,
				skippedNoColliders,
				createdFallbackSkinBones,
				unresolvedCanCollideBones,
				unresolvedNoCollideBones,
				unresolvedWeightThresholds,
				extraction.stats.nullBones,
				extraction.stats.nonNodeBones,
				extraction.stats.missingBoneData,
				extraction.stats.unsupportedGeometryClasses,
				extraction.stats.missingRendererData,
				extraction.stats.missingVertexBuffer,
				extraction.stats.missingIndexBuffer,
				extraction.stats.missingCpuVertexData,
				extraction.stats.missingPositionData,
				extraction.stats.splitPositionData,
				extraction.stats.faceGenPositionData,
				extraction.stats.nonFinitePositions,
				extraction.stats.invalidCpuVertexData,
				extraction.stats.pendingVertexCopies,
				extraction.stats.missingCpuIndexData,
				extraction.stats.invalidCpuIndexData,
				extraction.stats.pendingIndexCopies,
				extraction.stats.undersizedVertexBuffers,
				extraction.stats.undersizedIndexBuffers,
				extraction.stats.badBoneIndices);
		}
		return false;
	}

	void Fo4PhysicsWorld::BuildPrototypeConstraintsLocked(
		PrototypeActorState& a_state,
		const PhysicsXmlSummary& a_summary,
		const std::uint64_t a_buildGroup,
		const PrototypeBuildDomain a_domain,
		const std::span<PrototypeBody> a_stagedBodies,
		std::vector<PrototypeConstraint>& a_stagedConstraints)
	{
		std::uint32_t created = 0;
		std::uint32_t skippedMissingBodies = 0;
		std::uint32_t skippedExisting = 0;
		std::uint32_t skippedSelfConstraints = 0;
		std::uint32_t kinematicPairsAllowed = 0;
		std::uint32_t skippedInvalid = 0;
		const auto findBodyForConstraint = [&](const std::string_view a_name) {
			auto body = std::ranges::find_if(a_state.bodies, [a_buildGroup, a_name](const PrototypeBody& a_body) {
				return PrototypeBodyHasBuildGroup(a_body, a_buildGroup) && PhysicsNamesEqual(a_body.boneName, a_name);
			});
			if (body != a_state.bodies.end()) {
				return std::addressof(*body);
			}
			const auto stagedBody = std::ranges::find_if(a_stagedBodies, [a_buildGroup, a_name](const PrototypeBody& a_body) {
				return PrototypeBodyHasBuildGroup(a_body, a_buildGroup) && PhysicsNamesEqual(a_body.boneName, a_name);
			});
			return stagedBody != a_stagedBodies.end() ? std::addressof(*stagedBody) : nullptr;
		};
		for (const auto& descriptor : a_summary.constraintDescriptors) {
			const auto bodyA = findBodyForConstraint(descriptor.bodyA);
			const auto bodyB = findBodyForConstraint(descriptor.bodyB);
			if (!bodyA || !bodyB || !bodyA->bone || !bodyB->bone || bodyA->meshOnlySkinBone || bodyB->meshOnlySkinBone) {
				++skippedMissingBodies;
				spdlog::debug("skipping constraint '{}' because bodies '{}'/'{}' were not both resolved", descriptor.name, descriptor.bodyA, descriptor.bodyB);
				continue;
			}
			if (bodyA == bodyB || bodyA->bone.get() == bodyB->bone.get()) {
				++skippedSelfConstraints;
				spdlog::warn("skipping constraint '{}' between same body '{}'", descriptor.name, descriptor.bodyA);
				continue;
			}
			if (bodyA->bone->m_rig.isStaticOrKinematicObject() && bodyB->bone->m_rig.isStaticOrKinematicObject()) {
				++kinematicPairsAllowed;
				spdlog::debug("allowing FO4 kinematic-to-kinematic constraint '{}' between '{}'/'{}'", descriptor.name, descriptor.bodyA, descriptor.bodyB);
			}
			const auto existing = std::ranges::find_if(a_state.constraints, [&descriptor, a_buildGroup](const PrototypeConstraint& a_constraint) {
					return a_constraint.buildGroup == a_buildGroup && PhysicsNamesEqual(a_constraint.bodyA, descriptor.bodyA) && PhysicsNamesEqual(a_constraint.bodyB, descriptor.bodyB);
			});
			if (existing != a_state.constraints.end()) {
				++skippedExisting;
				continue;
			}
			const auto existingStaged = std::ranges::find_if(a_stagedConstraints, [&descriptor, a_buildGroup](const PrototypeConstraint& a_constraint) {
					return a_constraint.buildGroup == a_buildGroup && PhysicsNamesEqual(a_constraint.bodyA, descriptor.bodyA) && PhysicsNamesEqual(a_constraint.bodyB, descriptor.bodyB);
			});
			if (existingStaged != a_stagedConstraints.end()) {
				++skippedExisting;
				continue;
			}

			auto constraint = CreatePrototypeConstraint(
				descriptor,
				bodyA->bone->m_rig,
				bodyB->bone->m_rig,
				bodyA->bone->m_rigToLocal,
				bodyB->bone->m_rigToLocal,
				bodyA->bone->m_currentTransform,
				bodyB->bone->m_currentTransform);
			if (!constraint) {
				++skippedInvalid;
				continue;
			}

			PrototypeConstraint prototypeConstraint;
			prototypeConstraint.buildGroup = a_buildGroup;
			prototypeConstraint.domain = a_domain;
			prototypeConstraint.bodyA = descriptor.bodyA;
			prototypeConstraint.bodyB = descriptor.bodyB;
			prototypeConstraint.kind = descriptor.kind;
			const auto reversedGeneric = descriptor.kind == PhysicsConstraintKind::kGeneric && descriptor.useLinearReferenceFrameA;
			prototypeConstraint.boneA = reversedGeneric ? bodyB->bone.get() : bodyA->bone.get();
			prototypeConstraint.boneB = reversedGeneric ? bodyA->bone.get() : bodyB->bone.get();
			prototypeConstraint.constraint = std::move(constraint);
			a_stagedConstraints.push_back(std::move(prototypeConstraint));
			++created;
		}

		if (created > 0 || skippedMissingBodies > 0 || skippedExisting > 0 || skippedSelfConstraints > 0 || kinematicPairsAllowed > 0 || skippedInvalid > 0) {
			spdlog::info(
				"created {} {} prototype Bullet constraints for actor={} buildGroup={}; actor constraints={} skippedMissingBodies={} skippedExisting={} skippedSelfConstraints={} kinematicPairsAllowed={} skippedInvalid={}",
				created,
				PrototypeDomainName(a_domain),
				static_cast<void*>(a_state.actor),
				a_buildGroup,
				a_state.constraints.size(),
				skippedMissingBodies,
				skippedExisting,
				skippedSelfConstraints,
				kinematicPairsAllowed,
				skippedInvalid);
		}
	}

	void Fo4PhysicsWorld::ScalePrototypeConstraintsLocked(PrototypeActorState& a_state)
	{
		if (!a_state.runtimes.empty()) {
			for (const auto& runtime : a_state.runtimes) {
				ScalePrototypeConstraintsLocked(a_state, runtime);
			}
			return;
		}

		for (auto& prototypeConstraint : a_state.constraints) {
			if (!prototypeConstraint.constraint || !prototypeConstraint.boneA || !prototypeConstraint.boneB) {
				continue;
			}

			const auto newScaleA = CurrentBoneScale(prototypeConstraint.boneA);
			const auto newScaleB = CurrentBoneScale(prototypeConstraint.boneB);
			if (btFuzzyZero(newScaleA - prototypeConstraint.scaleA) && btFuzzyZero(newScaleB - prototypeConstraint.scaleB)) {
				continue;
			}

			switch (prototypeConstraint.kind) {
			case PhysicsConstraintKind::kConeTwist:
				ScaleConeTwistConstraint(
					*static_cast<btConeTwistConstraint*>(prototypeConstraint.constraint.get()),
					prototypeConstraint.scaleA,
					prototypeConstraint.scaleB,
					newScaleA,
					newScaleB);
				break;
			case PhysicsConstraintKind::kStiffSpring:
				static_cast<PrototypeStiffSpringConstraint*>(prototypeConstraint.constraint.get())
					->ScaleConstraint(prototypeConstraint.scaleA, prototypeConstraint.scaleB, newScaleA, newScaleB);
				break;
			case PhysicsConstraintKind::kGeneric:
			default:
				ScaleGenericConstraint(
					*static_cast<btGeneric6DofSpring2Constraint*>(prototypeConstraint.constraint.get()),
					prototypeConstraint.scaleA,
					prototypeConstraint.scaleB,
					newScaleA,
					newScaleB);
				break;
			}

			prototypeConstraint.scaleA = newScaleA;
			prototypeConstraint.scaleB = newScaleB;
		}
	}

	void Fo4PhysicsWorld::ScalePrototypeConstraintsLocked(PrototypeActorState& a_state, const PrototypeBuildGroupRuntime& a_runtime)
	{
		for (auto* constraint : a_runtime.constraints) {
			if (!constraint) {
				continue;
			}
			auto prototypeConstraint = std::ranges::find_if(a_state.constraints, [constraint](const PrototypeConstraint& a_constraint) {
				return a_constraint.constraint.get() == constraint;
			});
			if (prototypeConstraint == a_state.constraints.end() ||
				!prototypeConstraint->constraint ||
				!prototypeConstraint->boneA ||
				!prototypeConstraint->boneB) {
				continue;
			}

			const auto newScaleA = CurrentBoneScale(prototypeConstraint->boneA);
			const auto newScaleB = CurrentBoneScale(prototypeConstraint->boneB);
			if (btFuzzyZero(newScaleA - prototypeConstraint->scaleA) && btFuzzyZero(newScaleB - prototypeConstraint->scaleB)) {
				continue;
			}

			switch (prototypeConstraint->kind) {
			case PhysicsConstraintKind::kConeTwist:
				ScaleConeTwistConstraint(
					*static_cast<btConeTwistConstraint*>(prototypeConstraint->constraint.get()),
					prototypeConstraint->scaleA,
					prototypeConstraint->scaleB,
					newScaleA,
					newScaleB);
				break;
			case PhysicsConstraintKind::kStiffSpring:
				static_cast<PrototypeStiffSpringConstraint*>(prototypeConstraint->constraint.get())
					->ScaleConstraint(prototypeConstraint->scaleA, prototypeConstraint->scaleB, newScaleA, newScaleB);
				break;
			case PhysicsConstraintKind::kGeneric:
			default:
				ScaleGenericConstraint(
					*static_cast<btGeneric6DofSpring2Constraint*>(prototypeConstraint->constraint.get()),
					prototypeConstraint->scaleA,
					prototypeConstraint->scaleB,
					newScaleA,
					newScaleB);
				break;
			}

			prototypeConstraint->scaleA = newScaleA;
			prototypeConstraint->scaleB = newScaleB;
		}
	}
}
