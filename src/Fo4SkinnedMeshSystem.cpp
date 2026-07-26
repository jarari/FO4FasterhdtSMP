#include "Fo4SkinnedMeshSystem.h"

#include "Fo4NiObjectUtils.h"
#include "Fo4TransformConversion.h"

#include <cfloat>

namespace Smp
{
	Fo4SkinnedMeshSystem::~Fo4SkinnedMeshSystem()
	{
		// Base-class objects depend on collision shapes and motion states stored
		// in the derived FO4 records, so release them before member destruction.
		ClearSystemObjects();
	}

	bool Fo4SkinnedMeshSystem::AddBone(SystemObjectRef<Fo4SkinnedMeshBone>& a_bone)
	{
		auto* bone = a_bone.get();
		if (!bone || ContainsBone(bone)) {
			return false;
		}
		auto owner = a_bone.TakeOwnership();
		if (!owner) {
			return false;
		}
		m_bones.push_back(std::move(owner));
		return true;
	}

	bool Fo4SkinnedMeshSystem::AddMesh(SystemObjectRef<hdt::SkinnedMeshBody>& a_mesh)
	{
		auto* mesh = a_mesh.get();
		if (!mesh || ContainsMesh(mesh)) {
			return false;
		}
		auto owner = a_mesh.TakeOwnership();
		if (!owner) {
			return false;
		}
		m_meshes.push_back(std::move(owner));
		return true;
	}

	bool Fo4SkinnedMeshSystem::AddConstraint(SystemObjectRef<hdt::BoneScaleConstraint>& a_constraint)
	{
		auto* constraint = a_constraint.get();
		if (!constraint || ContainsConstraint(constraint)) {
			return false;
		}
		auto owner = a_constraint.TakeOwnership();
		if (!owner) {
			return false;
		}
		m_constraints.push_back(std::move(owner));
		return true;
	}

	bool Fo4SkinnedMeshSystem::ContainsBone(const Fo4SkinnedMeshBone* a_bone) const
	{
		return std::ranges::any_of(m_bones, [a_bone](const auto& a_entry) {
			return a_entry.get() == a_bone;
		});
	}

	bool Fo4SkinnedMeshSystem::ContainsMesh(const hdt::SkinnedMeshBody* a_mesh) const
	{
		return std::ranges::any_of(m_meshes, [a_mesh](const auto& a_entry) {
			return a_entry.get() == a_mesh;
		});
	}

	bool Fo4SkinnedMeshSystem::ContainsConstraint(const hdt::BoneScaleConstraint* a_constraint) const
	{
		return std::ranges::any_of(m_constraints, [a_constraint](const auto& a_entry) {
			return a_entry.get() == a_constraint;
		});
	}

	RE::BSTSmartPointer<Fo4SkinnedMeshBone> Fo4SkinnedMeshSystem::ReleaseBone(Fo4SkinnedMeshBone* a_bone)
	{
		const auto found = std::ranges::find_if(m_bones, [a_bone](const auto& a_entry) {
			return a_entry.get() == a_bone;
		});
		if (found == m_bones.end()) {
			return {};
		}
		RE::BSTSmartPointer<Fo4SkinnedMeshBone> result(static_cast<Fo4SkinnedMeshBone*>(found->get()));
		m_bones.erase(found);
		return result;
	}

	RE::BSTSmartPointer<hdt::SkinnedMeshBody> Fo4SkinnedMeshSystem::ReleaseMesh(hdt::SkinnedMeshBody* a_mesh)
	{
		const auto found = std::ranges::find_if(m_meshes, [a_mesh](const auto& a_entry) {
			return a_entry.get() == a_mesh;
		});
		if (found == m_meshes.end()) {
			return {};
		}
		auto result = std::move(*found);
		m_meshes.erase(found);
		return result;
	}

	RE::BSTSmartPointer<hdt::BoneScaleConstraint> Fo4SkinnedMeshSystem::ReleaseConstraint(
		hdt::BoneScaleConstraint* a_constraint)
	{
		const auto found = std::ranges::find_if(m_constraints, [a_constraint](const auto& a_entry) {
			return a_entry.get() == a_constraint;
		});
		if (found == m_constraints.end()) {
			return {};
		}
		auto result = std::move(*found);
		m_constraints.erase(found);
		return result;
	}

	void Fo4SkinnedMeshSystem::ClearSystemObjects()
	{
		m_constraintGroups.clear();
		m_constraints.clear();
		m_meshes.clear();
		m_bones.clear();
		m_shapeRefs.clear();
	}

	bool Fo4SkinnedMeshSystem::HasPhysics() const
	{
		return !buildGroups.empty() || !bodies.empty() || !meshes.empty() || !constraints.empty();
	}

	bool Fo4SkinnedMeshSystem::IsActive() const
	{
		return m_world != nullptr;
	}

	bool Fo4SkinnedMeshSystem::IsInactive() const
	{
		return HasPhysics() && m_world == nullptr;
	}

	float Fo4SkinnedMeshSystem::prepareForRead(float a_timeStep)
	{
		readPreparation_ = {
			.timeStep = a_timeStep,
		};

		auto* actorRoot = actor ? actor->Get3D(firstPerson) : nullptr;
		if (!actorRoot && actor && !firstPerson) {
			actorRoot = actor->Get3D();
		}
		auto* skeletonRoot = actorRoot ? actorRoot->IsNode() : nullptr;
		if (!skeletonRoot) {
			return a_timeStep;
		}

		auto* topRoot = static_cast<RE::NiAVObject*>(skeletonRoot);
		while (topRoot && topRoot->parent) {
			topRoot = topRoot->parent;
		}

		if (lastReadRoot && lastReadRoot.get() != topRoot) {
			a_timeStep = 0.0F;
			lastRootRotationInitialized = false;
		}
		if (!readInitialized) {
			a_timeStep = 0.0F;
			readInitialized = true;
			lastRootRotationInitialized = false;
		}

		if (a_timeStep <= 0.0F) {
			NiObject::UpdateWorldData(skeletonRoot, true);
			lastRootRotation = Fo4Transform::ToBulletTransform(skeletonRoot->world).getRotation();
			if (lastRootRotation.length2() <= FLT_EPSILON) {
				lastRootRotation = btQuaternion::getIdentity();
			} else {
				lastRootRotation.normalize();
			}
			lastRootRotationInitialized = true;
			lastReadRoot = topRoot;
			readPreparation_.timeStep = 0.0F;
			return 0.0F;
		}

		auto newRootRotation = Fo4Transform::ToBulletTransform(skeletonRoot->world).getRotation();
		if (newRootRotation.length2() <= FLT_EPSILON) {
			newRootRotation = btQuaternion::getIdentity();
		} else {
			newRootRotation.normalize();
		}
		if (!lastRootRotationInitialized || lastRootRotation.length2() <= FLT_EPSILON) {
			lastRootRotation = newRootRotation;
			lastRootRotationInitialized = true;
		} else if (firstPerson) {
			lastRootRotation = newRootRotation;
		} else {
			btVector3 rotationAxis;
			btScalar rotationAngle = 0.0F;
			btTransformUtil::calculateDiffAxisAngleQuaternion(lastRootRotation, newRootRotation, rotationAxis, rotationAngle);
			if (clampRotations) {
				const auto limit = rotationSpeedLimit * a_timeStep;
				if (rotationAngle < -limit || rotationAngle > limit) {
					rotationAngle = btClamped(rotationAngle, -limit, limit);
					lastRootRotation = btQuaternion(rotationAxis, rotationAngle) * lastRootRotation;
					lastRootRotation.normalize();
					readPreparation_.restoreRoot = skeletonRoot;
					readPreparation_.restoreWorld = skeletonRoot->world;
					skeletonRoot->world.rotate =
						Fo4Transform::ToNiTransform(btTransform(lastRootRotation), skeletonRoot->world.scale).rotate;
					for (auto& child : skeletonRoot->children) {
						NiObject::UpdateWorldData(child.get(), true);
					}
				}
			} else if (unclampedResets) {
				const auto limit = unclampedResetAngle * a_timeStep;
				if (rotationAngle < -limit || rotationAngle > limit) {
					NiObject::UpdateWorldData(skeletonRoot, true);
					lastRootRotation = Fo4Transform::ToBulletTransform(skeletonRoot->world).getRotation();
					if (lastRootRotation.length2() <= FLT_EPSILON) {
						lastRootRotation = btQuaternion::getIdentity();
					} else {
						lastRootRotation.normalize();
					}
					lastReadRoot = topRoot;
					readPreparation_.timeStep = 0.0F;
					return 0.0F;
				}
			}
		}

		lastReadRoot = topRoot;
		readPreparation_.timeStep = a_timeStep;
		return a_timeStep;
	}
}
