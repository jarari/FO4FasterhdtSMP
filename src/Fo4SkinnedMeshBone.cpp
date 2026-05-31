#include "Fo4SkinnedMeshBone.h"

#include "Fo4TransformConversion.h"

#include <cmath>

namespace
{
	constexpr float kResetPhysicsTimeStep = 0.0F;
	bool  g_clampRotations = true;
	float g_rotationSpeedLimit = 10.0F;
	bool  g_unclampedResets = true;
	float g_unclampedResetAngle = 130.0F;

	void ResetRigidBody(btRigidBody& a_body, const btTransform& a_transform)
	{
		const btVector3 zero(0.0F, 0.0F, 0.0F);
		a_body.setWorldTransform(a_transform);
		a_body.setInterpolationWorldTransform(a_transform);
		if (auto* motionState = a_body.getMotionState()) {
			motionState->setWorldTransform(a_transform);
		}
		a_body.setLinearVelocity(zero);
		a_body.setAngularVelocity(zero);
		a_body.setInterpolationLinearVelocity(zero);
		a_body.setInterpolationAngularVelocity(zero);
		a_body.updateInertiaTensor();
	}

	float RotationDeltaAngle(const btTransform& a_current, const btTransform& a_destination)
	{
		auto currentRotation = a_current.getRotation();
		auto destinationRotation = a_destination.getRotation();
		if (currentRotation.length2() <= FLT_EPSILON || destinationRotation.length2() <= FLT_EPSILON) {
			return 0.0F;
		}

		currentRotation.normalize();
		destinationRotation.normalize();
		return currentRotation.angleShortestPath(destinationRotation);
	}

}

namespace Smp
{
	Fo4SkinnedMeshBone::Fo4SkinnedMeshBone(const RE::BSFixedString& a_name, RE::NiNode* a_node, btRigidBody::btRigidBodyConstructionInfo& a_constructionInfo) :
		SkinnedMeshBone(a_name, a_constructionInfo),
		node_(a_node)
	{
		if (a_constructionInfo.m_mass > 0.0F) {
			m_rig.setCollisionFlags(0);
		} else {
			m_rig.setCollisionFlags(btCollisionObject::CF_KINEMATIC_OBJECT);
		}

		for (auto* node = a_node; node; node = node->parent) {
			++depth_;
		}
	}

	void Fo4SkinnedMeshBone::ApplyStabilityConfig(
		const bool a_clampRotations,
		const float a_rotationSpeedLimit,
		const bool a_unclampedResets,
		const float a_unclampedResetAngle)
	{
		g_clampRotations = a_clampRotations;
		g_rotationSpeedLimit = std::max(a_rotationSpeedLimit, 0.0F);
		g_unclampedResets = a_unclampedResets;
		g_unclampedResetAngle = std::max(a_unclampedResetAngle, 0.0F);
	}

	void Fo4SkinnedMeshBone::AddSkinWorldTransform(
		RE::BSSkin::Instance* a_skin,
		const std::uint32_t a_index,
		const std::uint64_t a_buildGroup,
		RE::NiAVObject* a_originalBone,
		RE::NiTransform* a_originalWorldTransform,
		RE::NiAVObject* a_originalRootNode)
	{
		if (!a_skin || a_index >= a_skin->worldTransforms.size()) {
			return;
		}

		auto* transform = a_skin->worldTransforms[a_index];
		if (!transform) {
			return;
		}

		const auto found = std::ranges::find_if(skinWorldTransforms_, [a_skin, a_index, a_buildGroup](const SkinWorldTransformSlot& a_slot) {
			return a_slot.skin.get() == a_skin && a_slot.index == a_index && a_slot.buildGroup == a_buildGroup;
		});
		if (found != skinWorldTransforms_.end()) {
			if (!found->originalBone && a_originalBone) {
				found->originalBone = a_originalBone;
			}
			if (!found->originalWorldTransform && a_originalWorldTransform) {
				found->originalWorldTransform = a_originalWorldTransform;
			}
			if (!found->originalRootNode && a_originalRootNode) {
				found->originalRootNode = a_originalRootNode;
			}
			found->cached = transform;
			return;
		}

		skinWorldTransforms_.push_back({
			.skin = a_skin,
			.index = a_index,
			.buildGroup = a_buildGroup,
			.originalBone = a_originalBone,
			.originalWorldTransform = a_originalWorldTransform,
			.originalRootNode = a_originalRootNode,
			.cached = transform,
		});
	}

	void Fo4SkinnedMeshBone::CollectSkinWorldTransformSlots(std::vector<ActiveSkinSlot>& a_slots) const
	{
		for (const auto& slot : skinWorldTransforms_) {
			if (!slot.skin || slot.buildGroup == 0) {
				continue;
			}
			a_slots.push_back({
				.skin = slot.skin.get(),
				.index = slot.index,
				.buildGroup = slot.buildGroup,
			});
		}
	}

	void Fo4SkinnedMeshBone::CollectSkinWorldTransformRestoreSlots(std::vector<SkinSlotRestore>& a_slots) const
	{
		auto* node = node_.get();
		if (!node) {
			return;
		}

		for (const auto& slot : skinWorldTransforms_) {
			if (!slot.skin || slot.buildGroup == 0) {
				continue;
			}
			a_slots.push_back({
				.skin = slot.skin,
				.index = slot.index,
				.buildGroup = slot.buildGroup,
				.reboundBone = node,
				.reboundWorldTransform = std::addressof(node->world),
				.originalBone = slot.originalBone,
				.originalWorldTransform = slot.originalWorldTransform,
				.originalRootNode = slot.originalRootNode,
			});
		}
	}

	void Fo4SkinnedMeshBone::RemoveSkinWorldTransformsForBuildGroup(const std::uint64_t a_buildGroup)
	{
		RemoveSkinWorldTransformsForBuildGroup(a_buildGroup, std::span<const ActiveSkinSlot>{});
	}

	void Fo4SkinnedMeshBone::RemoveSkinWorldTransformsForBuildGroup(const std::uint64_t a_buildGroup, const std::span<const ActiveSkinSlot> a_activeSlots)
	{
		if (a_buildGroup == 0) {
			return;
		}

		std::erase_if(skinWorldTransforms_, [this, a_buildGroup, a_activeSlots](const SkinWorldTransformSlot& a_slot) {
			if (a_slot.buildGroup == a_buildGroup && a_slot.skin) {
				const auto keepRebound =
					std::ranges::any_of(skinWorldTransforms_, [&a_slot, a_buildGroup](const SkinWorldTransformSlot& a_other) {
						return a_other.buildGroup != a_buildGroup &&
							a_other.skin.get() == a_slot.skin.get() &&
							a_other.index == a_slot.index;
					}) ||
					std::ranges::any_of(a_activeSlots, [&a_slot, a_buildGroup](const ActiveSkinSlot& a_other) {
						return a_other.buildGroup != a_buildGroup &&
							a_other.skin == a_slot.skin.get() &&
							a_other.index == a_slot.index;
					});
				if (keepRebound) {
					return true;
				}

				const auto node = node_.get();
				if (node && a_slot.index < a_slot.skin->bones.size()) {
					auto*& bone = a_slot.skin->bones[a_slot.index];
					if (bone == node && a_slot.originalBone) {
						spdlog::debug(
							"restored FO4 skin bone slot for '{}' buildGroup={} index={} skin={} node={} original={}",
							m_name.c_str(),
							a_buildGroup,
							a_slot.index,
							static_cast<void*>(a_slot.skin.get()),
							static_cast<void*>(node),
							static_cast<void*>(a_slot.originalBone.get()));
						bone = a_slot.originalBone.get();
					}
				}
				if (node && a_slot.index < a_slot.skin->worldTransforms.size()) {
					auto*& worldTransform = a_slot.skin->worldTransforms[a_slot.index];
					if (worldTransform == std::addressof(node->world) && a_slot.originalWorldTransform) {
						worldTransform = a_slot.originalWorldTransform;
					}
				}
				const auto keepRootRebound =
					std::ranges::any_of(skinWorldTransforms_, [&a_slot, a_buildGroup](const SkinWorldTransformSlot& a_other) {
						return a_other.buildGroup != a_buildGroup && a_other.skin.get() == a_slot.skin.get();
					}) ||
					std::ranges::any_of(a_activeSlots, [&a_slot, a_buildGroup](const ActiveSkinSlot& a_other) {
						return a_other.buildGroup != a_buildGroup && a_other.skin == a_slot.skin.get();
					});
				if (!keepRootRebound && a_slot.originalRootNode && a_slot.skin->rootNode != a_slot.originalRootNode.get()) {
					spdlog::debug(
						"restored FO4 skin root for '{}' buildGroup={} skin={} root={} originalRoot={}",
						m_name.c_str(),
						a_buildGroup,
						static_cast<void*>(a_slot.skin.get()),
						static_cast<void*>(a_slot.skin->rootNode),
						static_cast<void*>(a_slot.originalRootNode.get()));
					a_slot.skin->rootNode = a_slot.originalRootNode.get();
				}
			}
			return a_slot.buildGroup == a_buildGroup;
		});
	}

	RE::NiTransform* Fo4SkinnedMeshBone::ResolveSkinWorldTransform(SkinWorldTransformSlot& a_slot)
	{
		if (!a_slot.skin || a_slot.index >= a_slot.skin->worldTransforms.size()) {
			return nullptr;
		}

		auto* node = node_.get();
		if (!node) {
			return nullptr;
		}

		if (a_slot.index < a_slot.skin->bones.size() && a_slot.skin->bones[a_slot.index] != node) {
			spdlog::debug(
				"rebound active FO4 skin bone slot for '{}' buildGroup={} index={} skin={} oldNode={} newNode={}",
				m_name.c_str(),
				a_slot.buildGroup,
				a_slot.index,
				static_cast<void*>(a_slot.skin.get()),
				static_cast<void*>(a_slot.skin->bones[a_slot.index]),
				static_cast<void*>(node));
			a_slot.skin->bones[a_slot.index] = node;
		}
		if (a_slot.skin->worldTransforms[a_slot.index] != std::addressof(node->world)) {
			spdlog::debug(
				"rebound active FO4 skin world transform for '{}' buildGroup={} index={} skin={} old={} new={}",
				m_name.c_str(),
				a_slot.buildGroup,
				a_slot.index,
				static_cast<void*>(a_slot.skin.get()),
				static_cast<void*>(a_slot.skin->worldTransforms[a_slot.index]),
				static_cast<void*>(std::addressof(node->world)));
			a_slot.skin->worldTransforms[a_slot.index] = std::addressof(node->world);
		}

		auto* current = a_slot.skin->worldTransforms[a_slot.index];
		if (!current) {
			return nullptr;
		}

		if (a_slot.cached && current != a_slot.cached) {
			spdlog::debug(
				"recached FO4 skin world transform for bone '{}' buildGroup={} index={} old={} new={}",
				m_name.c_str(),
				a_slot.buildGroup,
				a_slot.index,
				static_cast<void*>(a_slot.cached),
				static_cast<void*>(current));
		}
		a_slot.cached = current;
		return current;
	}

	void Fo4SkinnedMeshBone::readTransform(const float a_timeStep)
	{
		if (!node_) {
			return;
		}

		const auto oldScale = m_currentTransform.getScale();
		const auto isStaticOrKinematic = m_rig.isStaticOrKinematicObject();
		m_currentTransform = Smp::Fo4Transform::ToBulletQsTransformNormalizedScale(node_->world);
		const auto newScale = m_currentTransform.getScale();
		const auto current = m_rig.getWorldTransform();

		if (!btFuzzyZero(newScale - oldScale) && newScale > FLT_EPSILON && oldScale > FLT_EPSILON) {
			const auto factor = oldScale / newScale;
			if (!isStaticOrKinematic && m_rig.getInvMass() > 0.0F) {
				const auto factor2 = factor * factor;
				const auto factor3 = factor2 * factor;
				const auto factor5 = factor3 * factor2;
				const auto inertia = m_rig.getInvInertiaDiagLocal();
				m_rig.setMassProps(1.0F / (m_rig.getInvMass() * factor3), btVector3(1.0F, 1.0F, 1.0F));
				m_rig.setInvInertiaDiagLocal(inertia * factor5);
				m_rig.updateInertiaTensor();
			}
			const auto invFactor = 1.0F / factor;
			m_localToRig.getOrigin() *= invFactor;
			m_rigToLocal.getOrigin() *= invFactor;
			if (auto* shape = m_rig.getCollisionShape()) {
				shape->setLocalScaling(btVector3(newScale, newScale, newScale));
			}
		}

		const auto destination = m_currentTransform.asTransform() * m_localToRig;
		if (a_timeStep <= kResetPhysicsTimeStep) {
			ResetRigidBody(m_rig, destination);
		} else if (isStaticOrKinematic) {
			btVector3 linearVelocity;
			btVector3 angularVelocity;
			btTransformUtil::calculateVelocity(current, destination, a_timeStep, linearVelocity, angularVelocity);
			const auto angularSpeed = angularVelocity.length();
			if (g_clampRotations && g_rotationSpeedLimit > 0.0F && angularSpeed > g_rotationSpeedLimit) {
				angularVelocity *= g_rotationSpeedLimit / angularSpeed;
			} else if (!g_clampRotations && g_unclampedResets && g_unclampedResetAngle > 0.0F && RotationDeltaAngle(current, destination) > btRadians(g_unclampedResetAngle)) {
				ResetRigidBody(m_rig, destination);
				return;
			}
			m_rig.setLinearVelocity(linearVelocity);
			m_rig.setAngularVelocity(angularVelocity);
			m_rig.setInterpolationLinearVelocity(linearVelocity);
			m_rig.setInterpolationAngularVelocity(angularVelocity);
		}
	}

	void Fo4SkinnedMeshBone::writeTransform()
	{
		if (!node_) {
			return;
		}

		const auto transform = m_rig.getWorldTransform() * m_rigToLocal;
		m_currentTransform.setBasis(transform.getBasis());
		m_currentTransform.setOrigin(transform.getOrigin());

		const auto world = Smp::Fo4Transform::ToNiTransformNormalizedScale(transform, node_->world.scale);
		node_->world = world;
		if (node_->parent) {
			node_->local = node_->parent->world.Invert() * world;
		} else {
			node_->local = world;
		}

		std::erase_if(skinWorldTransforms_, [this](SkinWorldTransformSlot& a_slot) {
			return ResolveSkinWorldTransform(a_slot) == nullptr;
		});
	}
}
