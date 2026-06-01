#include "BulletVisualization.h"

#include "hdtSkinnedMesh/hdtSkinnedMeshBody.h"
#include "hdtSkinnedMesh/hdtSkinnedMeshBone.h"
#include "hdtSkinnedMesh/hdtSkinnedMeshShape.h"
#include "RE/A/Actor.h"
#include "RE/B/BSGraphics.h"
#include "RE/M/Main.h"
#include "RE/N/NiCamera.h"
#include "RE/N/NiAVObject.h"
#include "RE/P/PlayerCamera.h"

#include <btBulletDynamicsCommon.h>
#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <BulletCollision/CollisionShapes/btCapsuleShape.h>
#include <BulletCollision/CollisionShapes/btCompoundShape.h>
#include <BulletCollision/CollisionShapes/btConeShape.h>
#include <BulletCollision/CollisionShapes/btCylinderShape.h>
#include <BulletCollision/CollisionShapes/btMultiSphereShape.h>
#include <BulletCollision/CollisionShapes/btPolyhedralConvexShape.h>
#include <BulletCollision/CollisionShapes/btSphereShape.h>
#include <BulletCollision/CollisionShapes/btStaticPlaneShape.h>
#include <imgui.h>
#include <spdlog/spdlog.h>

#include <cmath>
#include <cstdio>

namespace
{
	constexpr ImU32 kKinematicBoneColor = IM_COL32(255, 165, 0, 255);
	constexpr ImU32 kDynamicBoneColor = IM_COL32(135, 206, 235, 255);
	constexpr ImU32 kMeshColor = IM_COL32(120, 255, 140, 210);
	constexpr ImU32 kFallbackColor = IM_COL32(230, 230, 230, 180);
	constexpr ImU32 kTextColor = IM_COL32(255, 255, 255, 255);
	constexpr int kCurveSegments = 24;
	constexpr float kWorldSpaceThreshold = 2048.0F;

	bool LooksWorldSpace(const btVector3& a_position)
	{
		return a_position.length2() >= kWorldSpaceThreshold * kWorldSpaceThreshold;
	}

	struct Projector
	{
		const RE::NiCamera* camera{ nullptr };
		const char* cameraSource{ "none" };
		RE::NiPoint3 posAdjust{};
		RE::NiPoint3 currentPosAdjust{};
		float width{ 0.0F };
		float height{ 0.0F };
		std::uint32_t cameraCacheCount{ 0 };
		mutable std::uint32_t projectAttempts{ 0 };
		mutable std::uint32_t projectSuccesses{ 0 };

		[[nodiscard]] bool Project(const btVector3& a_world, ImVec2& a_screen) const
		{
			++projectAttempts;
			if (width <= 0.0F || height <= 0.0F) {
				return false;
			}

			if (!camera) {
				return false;
			}

			float x = 0.0F;
			float y = 0.0F;
			float z = 0.0F;
			const RE::NiPoint3 world{ a_world.x(), a_world.y(), a_world.z() };
			if (!RE::NiCamera::WorldPtToScreenPt3(camera->worldToCam, camera->port, world, x, y, z, 1e-5F) || z < 0.0F) {
				return false;
			}

			a_screen = ImVec2(x * width, (1.0F - y) * height);
			if (!std::isfinite(a_screen.x) || !std::isfinite(a_screen.y)) {
				return false;
			}
			++projectSuccesses;
			return true;
		}
	};

	bool IsNodeInHierarchy(const RE::NiAVObject* a_object, const RE::NiAVObject* a_root)
	{
		for (auto* current = a_object; current; current = current->parent) {
			if (current == a_root) {
				return true;
			}
		}

		return false;
	}

	const RE::NiAVObject* GetPlayerCameraRoot()
	{
		const auto* playerCamera = RE::PlayerCamera::GetSingleton();
		if (!playerCamera) {
			return nullptr;
		}

		if (playerCamera->currentState && playerCamera->currentState->camera && playerCamera->currentState->camera->cameraRoot) {
			return playerCamera->currentState->camera->cameraRoot.get();
		}

		return playerCamera->cameraRoot.get();
	}

	const RE::BSGraphics::CameraStateData* SelectGameplayCameraState(const RE::BSGraphics::State& a_state, const char*& a_source)
	{
		if (const auto* playerCameraRoot = GetPlayerCameraRoot()) {
			for (const auto& cachedCamera : a_state.cameraDataCache) {
				if (cachedCamera.referenceCamera && IsNodeInHierarchy(cachedCamera.referenceCamera, playerCameraRoot)) {
					a_source = "playerRoot";
					return std::addressof(cachedCamera);
				}
			}
		}

		if (const auto* worldCamera = RE::Main::WorldRootCamera()) {
			for (const auto& cachedCamera : a_state.cameraDataCache) {
				if (cachedCamera.referenceCamera == worldCamera) {
					a_source = "worldRoot";
					return std::addressof(cachedCamera);
				}
			}
		}

		a_source = "fallback";
		return std::addressof(a_state.cameraState);
	}

	Projector MakeProjector()
	{
		auto state = RE::BSGraphics::State::GetSingleton();
		auto* rendererData = RE::BSGraphics::GetRendererData();
		auto width = static_cast<float>(state.screenWidth);
		auto height = static_cast<float>(state.screenHeight);
		if ((width <= 0.0F || height <= 0.0F) && rendererData) {
			width = static_cast<float>(rendererData->renderWindow[0].windowWidth);
			height = static_cast<float>(rendererData->renderWindow[0].windowHeight);
		}

		const char* cameraSource = "none";
		const auto* cameraState = SelectGameplayCameraState(state, cameraSource);
		Projector projector{
			.camera = cameraState ? cameraState->referenceCamera : nullptr,
			.cameraSource = cameraSource,
			.width = width,
			.height = height,
			.cameraCacheCount = static_cast<std::uint32_t>(state.cameraDataCache.size()),
		};
		if (cameraState) {
			projector.posAdjust = cameraState->posAdjust;
			projector.currentPosAdjust = cameraState->currentPosAdjust;
		}

		return projector;
	}

	void DrawLine(ImDrawList& a_drawList, const Projector& a_projector, const btVector3& a_from, const btVector3& a_to, const ImU32 a_color, const float a_thickness = 1.0F)
	{
		ImVec2 from;
		ImVec2 to;
		if (a_projector.Project(a_from, from) && a_projector.Project(a_to, to)) {
			a_drawList.AddLine(from, to, a_color, a_thickness);
		}
	}

	void DrawText(ImDrawList& a_drawList, const Projector& a_projector, const btVector3& a_world, const char* a_text)
	{
		if (!a_text || !a_text[0]) {
			return;
		}

		ImVec2 screen;
		if (a_projector.Project(a_world, screen)) {
			screen.x += 6.0F;
			screen.y -= 6.0F;
			a_drawList.AddText(screen, kTextColor, a_text);
		}
	}

	void DrawPoint(ImDrawList& a_drawList, const Projector& a_projector, const btVector3& a_world, const ImU32 a_color)
	{
		ImVec2 screen;
		if (a_projector.Project(a_world, screen)) {
			a_drawList.AddCircle(screen, 2.0F, a_color, 8, 1.0F);
		}
	}

	void DrawAabb(ImDrawList& a_drawList, const Projector& a_projector, const btVector3& a_min, const btVector3& a_max, const ImU32 a_color)
	{
		const btVector3 corners[8]{
			{ a_min.x(), a_min.y(), a_min.z() }, { a_max.x(), a_min.y(), a_min.z() },
			{ a_max.x(), a_max.y(), a_min.z() }, { a_min.x(), a_max.y(), a_min.z() },
			{ a_min.x(), a_min.y(), a_max.z() }, { a_max.x(), a_min.y(), a_max.z() },
			{ a_max.x(), a_max.y(), a_max.z() }, { a_min.x(), a_max.y(), a_max.z() },
		};
		const int edges[12][2]{
			{ 0, 1 }, { 1, 2 }, { 2, 3 }, { 3, 0 }, { 4, 5 }, { 5, 6 },
			{ 6, 7 }, { 7, 4 }, { 0, 4 }, { 1, 5 }, { 2, 6 }, { 3, 7 },
		};
		for (const auto& edge : edges) {
			DrawLine(a_drawList, a_projector, corners[edge[0]], corners[edge[1]], a_color);
		}
	}

	void DrawFallbackAabb(ImDrawList& a_drawList, const Projector& a_projector, const btCollisionShape& a_shape, const btTransform& a_transform, const ImU32 a_color)
	{
		btVector3 min;
		btVector3 max;
		a_shape.getAabb(a_transform, min, max);
		DrawAabb(a_drawList, a_projector, min, max, a_color);
	}

	void DrawLocalLine(ImDrawList& a_drawList, const Projector& a_projector, const btTransform& a_transform, const btVector3& a_from, const btVector3& a_to, const ImU32 a_color)
	{
		DrawLine(a_drawList, a_projector, a_transform * a_from, a_transform * a_to, a_color);
	}

	void DrawLocalCircle(
		ImDrawList& a_drawList,
		const Projector& a_projector,
		const btTransform& a_transform,
		const btVector3& a_center,
		const int a_axisA,
		const int a_axisB,
		const float a_radiusA,
		const float a_radiusB,
		const ImU32 a_color)
	{
		btVector3 previous = a_center;
		previous[a_axisA] += a_radiusA;
		for (int segment = 1; segment <= kCurveSegments; ++segment) {
			const auto angle = SIMD_2_PI * static_cast<float>(segment) / static_cast<float>(kCurveSegments);
			btVector3 current = a_center;
			current[a_axisA] += std::cos(angle) * a_radiusA;
			current[a_axisB] += std::sin(angle) * a_radiusB;
			DrawLocalLine(a_drawList, a_projector, a_transform, previous, current, a_color);
			previous = current;
		}
	}

	void DrawBox(ImDrawList& a_drawList, const Projector& a_projector, const btBoxShape& a_shape, const btTransform& a_transform, const ImU32 a_color)
	{
		const auto half = a_shape.getHalfExtentsWithMargin();
		const btVector3 corners[8]{
			{ -half.x(), -half.y(), -half.z() }, { half.x(), -half.y(), -half.z() },
			{ half.x(), half.y(), -half.z() }, { -half.x(), half.y(), -half.z() },
			{ -half.x(), -half.y(), half.z() }, { half.x(), -half.y(), half.z() },
			{ half.x(), half.y(), half.z() }, { -half.x(), half.y(), half.z() },
		};
		const int edges[12][2]{
			{ 0, 1 }, { 1, 2 }, { 2, 3 }, { 3, 0 }, { 4, 5 }, { 5, 6 },
			{ 6, 7 }, { 7, 4 }, { 0, 4 }, { 1, 5 }, { 2, 6 }, { 3, 7 },
		};
		for (const auto& edge : edges) {
			DrawLocalLine(a_drawList, a_projector, a_transform, corners[edge[0]], corners[edge[1]], a_color);
		}
	}

	void DrawSphere(ImDrawList& a_drawList, const Projector& a_projector, const btSphereShape& a_shape, const btTransform& a_transform, const ImU32 a_color)
	{
		const auto radius = a_shape.getRadius();
		DrawLocalCircle(a_drawList, a_projector, a_transform, btVector3(0.0F, 0.0F, 0.0F), 0, 1, radius, radius, a_color);
		DrawLocalCircle(a_drawList, a_projector, a_transform, btVector3(0.0F, 0.0F, 0.0F), 0, 2, radius, radius, a_color);
		DrawLocalCircle(a_drawList, a_projector, a_transform, btVector3(0.0F, 0.0F, 0.0F), 1, 2, radius, radius, a_color);
	}

	void DrawCapsule(ImDrawList& a_drawList, const Projector& a_projector, const btCapsuleShape& a_shape, const btTransform& a_transform, const ImU32 a_color)
	{
		const auto radius = a_shape.getRadius();
		const auto halfHeight = a_shape.getHalfHeight();
		const auto up = a_shape.getUpAxis();
		const auto axisA = (up + 1) % 3;
		const auto axisB = (up + 2) % 3;

		btVector3 top(0.0F, 0.0F, 0.0F);
		btVector3 bottom(0.0F, 0.0F, 0.0F);
		top[up] = halfHeight;
		bottom[up] = -halfHeight;
		DrawLocalCircle(a_drawList, a_projector, a_transform, top, axisA, axisB, radius, radius, a_color);
		DrawLocalCircle(a_drawList, a_projector, a_transform, bottom, axisA, axisB, radius, radius, a_color);
		for (int sign : { -1, 1 }) {
			btVector3 from = top;
			btVector3 to = bottom;
			from[axisA] = radius * static_cast<float>(sign);
			to[axisA] = from[axisA];
			DrawLocalLine(a_drawList, a_projector, a_transform, from, to, a_color);
			from = top;
			to = bottom;
			from[axisB] = radius * static_cast<float>(sign);
			to[axisB] = from[axisB];
			DrawLocalLine(a_drawList, a_projector, a_transform, from, to, a_color);
		}
		DrawLocalCircle(a_drawList, a_projector, a_transform, top, up, axisA, radius, radius, a_color);
		DrawLocalCircle(a_drawList, a_projector, a_transform, bottom, up, axisA, radius, radius, a_color);
		DrawLocalCircle(a_drawList, a_projector, a_transform, top, up, axisB, radius, radius, a_color);
		DrawLocalCircle(a_drawList, a_projector, a_transform, bottom, up, axisB, radius, radius, a_color);
	}

	void DrawCylinder(ImDrawList& a_drawList, const Projector& a_projector, const btCylinderShape& a_shape, const btTransform& a_transform, const ImU32 a_color)
	{
		const auto half = a_shape.getHalfExtentsWithMargin();
		const auto up = a_shape.getUpAxis();
		const auto axisA = (up + 1) % 3;
		const auto axisB = (up + 2) % 3;
		btVector3 top(0.0F, 0.0F, 0.0F);
		btVector3 bottom(0.0F, 0.0F, 0.0F);
		top[up] = half[up];
		bottom[up] = -half[up];
		DrawLocalCircle(a_drawList, a_projector, a_transform, top, axisA, axisB, half[axisA], half[axisB], a_color);
		DrawLocalCircle(a_drawList, a_projector, a_transform, bottom, axisA, axisB, half[axisA], half[axisB], a_color);
		for (int segment = 0; segment < 4; ++segment) {
			const auto angle = SIMD_2_PI * static_cast<float>(segment) / 4.0F;
			btVector3 from = top;
			btVector3 to = bottom;
			from[axisA] = std::cos(angle) * half[axisA];
			from[axisB] = std::sin(angle) * half[axisB];
			to[axisA] = from[axisA];
			to[axisB] = from[axisB];
			DrawLocalLine(a_drawList, a_projector, a_transform, from, to, a_color);
		}
	}

	void DrawCone(ImDrawList& a_drawList, const Projector& a_projector, const btConeShape& a_shape, const btTransform& a_transform, const ImU32 a_color)
	{
		const auto radius = a_shape.getRadius();
		const auto height = a_shape.getHeight();
		const auto up = a_shape.getConeUpIndex();
		const auto axisA = (up + 1) % 3;
		const auto axisB = (up + 2) % 3;
		btVector3 apex(0.0F, 0.0F, 0.0F);
		btVector3 base(0.0F, 0.0F, 0.0F);
		apex[up] = height * 0.5F;
		base[up] = -height * 0.5F;
		DrawLocalCircle(a_drawList, a_projector, a_transform, base, axisA, axisB, radius, radius, a_color);
		for (int segment = 0; segment < 4; ++segment) {
			const auto angle = SIMD_2_PI * static_cast<float>(segment) / 4.0F;
			btVector3 rim = base;
			rim[axisA] = std::cos(angle) * radius;
			rim[axisB] = std::sin(angle) * radius;
			DrawLocalLine(a_drawList, a_projector, a_transform, apex, rim, a_color);
		}
	}

	bool DrawPolyhedral(ImDrawList& a_drawList, const Projector& a_projector, const btPolyhedralConvexShape& a_shape, const btTransform& a_transform, const ImU32 a_color)
	{
		const auto edgeCount = a_shape.getNumEdges();
		if (edgeCount <= 0) {
			return false;
		}
		for (int index = 0; index < edgeCount; ++index) {
			btVector3 from;
			btVector3 to;
			a_shape.getEdge(index, from, to);
			DrawLocalLine(a_drawList, a_projector, a_transform, from, to, a_color);
		}
		return true;
	}

	void DrawMultiSphere(ImDrawList& a_drawList, const Projector& a_projector, const btMultiSphereShape& a_shape, const btTransform& a_transform, const ImU32 a_color)
	{
		for (int index = 0; index < a_shape.getSphereCount(); ++index) {
			const btTransform sphereTransform(a_transform.getBasis(), a_transform * a_shape.getSpherePosition(index));
			const auto radius = a_shape.getSphereRadius(index);
			DrawLocalCircle(a_drawList, a_projector, sphereTransform, btVector3(0.0F, 0.0F, 0.0F), 0, 1, radius, radius, a_color);
			DrawLocalCircle(a_drawList, a_projector, sphereTransform, btVector3(0.0F, 0.0F, 0.0F), 0, 2, radius, radius, a_color);
			DrawLocalCircle(a_drawList, a_projector, sphereTransform, btVector3(0.0F, 0.0F, 0.0F), 1, 2, radius, radius, a_color);
		}
	}

	void DrawStaticPlane(ImDrawList& a_drawList, const Projector& a_projector, const btStaticPlaneShape& a_shape, const btTransform& a_transform, const ImU32 a_color)
	{
		const auto localCenter = a_shape.getPlaneNormal() * a_shape.getPlaneConstant();
		const auto center = a_transform * localCenter;
		const auto normal = (a_transform.getBasis() * a_shape.getPlaneNormal()).normalized();
		const auto tangent = normal.cross(btVector3(0.0F, 0.0F, 1.0F)).length2() > SIMD_EPSILON ?
			normal.cross(btVector3(0.0F, 0.0F, 1.0F)).normalized() :
			normal.cross(btVector3(0.0F, 1.0F, 0.0F)).normalized();
		const auto bitangent = normal.cross(tangent).normalized();
		constexpr float size = 96.0F;
		DrawLine(a_drawList, a_projector, center - tangent * size, center + tangent * size, a_color);
		DrawLine(a_drawList, a_projector, center - bitangent * size, center + bitangent * size, a_color);
	}

	void DrawShape(ImDrawList& a_drawList, const Projector& a_projector, const btCollisionShape& a_shape, const btTransform& a_transform, const ImU32 a_color)
	{
		const auto shapeType = a_shape.getShapeType();
		if (shapeType == COMPOUND_SHAPE_PROXYTYPE) {
			const auto* compound = static_cast<const btCompoundShape*>(std::addressof(a_shape));
			for (int index = 0; index < compound->getNumChildShapes(); ++index) {
				if (const auto* child = compound->getChildShape(index)) {
					DrawShape(a_drawList, a_projector, *child, a_transform * compound->getChildTransform(index), a_color);
				}
			}
			return;
		}

		switch (shapeType) {
		case SPHERE_SHAPE_PROXYTYPE:
			DrawSphere(a_drawList, a_projector, static_cast<const btSphereShape&>(a_shape), a_transform, a_color);
			return;
		case CAPSULE_SHAPE_PROXYTYPE:
			DrawCapsule(a_drawList, a_projector, static_cast<const btCapsuleShape&>(a_shape), a_transform, a_color);
			return;
		case BOX_SHAPE_PROXYTYPE:
			DrawBox(a_drawList, a_projector, static_cast<const btBoxShape&>(a_shape), a_transform, a_color);
			return;
		case CYLINDER_SHAPE_PROXYTYPE:
			DrawCylinder(a_drawList, a_projector, static_cast<const btCylinderShape&>(a_shape), a_transform, a_color);
			return;
		case CONE_SHAPE_PROXYTYPE:
			DrawCone(a_drawList, a_projector, static_cast<const btConeShape&>(a_shape), a_transform, a_color);
			return;
		case MULTI_SPHERE_SHAPE_PROXYTYPE:
			DrawMultiSphere(a_drawList, a_projector, static_cast<const btMultiSphereShape&>(a_shape), a_transform, a_color);
			return;
		case STATIC_PLANE_PROXYTYPE:
			DrawStaticPlane(a_drawList, a_projector, static_cast<const btStaticPlaneShape&>(a_shape), a_transform, a_color);
			return;
		default:
			break;
		}

		if (a_shape.isPolyhedral() && DrawPolyhedral(a_drawList, a_projector, static_cast<const btPolyhedralConvexShape&>(a_shape), a_transform, a_color)) {
			return;
		}

		DrawFallbackAabb(a_drawList, a_projector, a_shape, a_transform, a_color);
	}

	struct MeshVisualAlignment
	{
		btTransform transform{ btTransform::getIdentity() };
		btVector3 actorPosition{ 0.0F, 0.0F, 0.0F };
		btVector3 meshCenter{ 0.0F, 0.0F, 0.0F };
		btVector3 boneCenter{ 0.0F, 0.0F, 0.0F };
		btVector3 localBoneCenter{ 0.0F, 0.0F, 0.0F };
		btVector3 worldBoneCenter{ 0.0F, 0.0F, 0.0F };
		btVector3 rigCenter{ 0.0F, 0.0F, 0.0F };
		btVector3 aabbCenter{ 0.0F, 0.0F, 0.0F };
		btVector3 objectOrigin{ 0.0F, 0.0F, 0.0F };
		const char* visualSource{ "none" };
		std::uint32_t meshSamples{ 0 };
		std::uint32_t boneSamples{ 0 };
		std::uint32_t localBoneSamples{ 0 };
		std::uint32_t worldBoneSamples{ 0 };
		std::uint32_t rigSamples{ 0 };
		bool applied{ false };
	};

	MeshVisualAlignment CalculateMeshVisualAlignment(const hdt::SkinnedMeshBody& a_body)
	{
		MeshVisualAlignment result;
		if (a_body.actor_) {
			const auto actorPosition = a_body.actor_->GetPosition();
			result.actorPosition = btVector3(actorPosition.x, actorPosition.y, actorPosition.z);
		}
		result.objectOrigin = a_body.getWorldTransform().getOrigin();
		btVector3 min;
		btVector3 max;
		a_body.bulletShape_.getAabb(a_body.getWorldTransform(), min, max);
		result.aabbCenter = (min + max) * 0.5F;

		for (const auto& position : a_body.vertexPositions_) {
			result.meshCenter += position.pos();
			++result.meshSamples;
		}
		if (result.meshSamples == 0) {
			return result;
		}
		result.meshCenter /= static_cast<float>(result.meshSamples);

		for (const auto& bone : a_body.skinnedBones_) {
			if (!bone.ptr) {
				continue;
			}
			const auto currentOrigin = bone.ptr->m_currentTransform.getOrigin();
			result.boneCenter += currentOrigin;
			if (LooksWorldSpace(currentOrigin)) {
				result.worldBoneCenter += currentOrigin;
				++result.worldBoneSamples;
			} else {
				result.localBoneCenter += currentOrigin;
				++result.localBoneSamples;
			}
			++result.boneSamples;

			const auto rigOrigin = bone.ptr->m_rig.getWorldTransform().getOrigin();
			if (LooksWorldSpace(rigOrigin)) {
				result.rigCenter += rigOrigin;
				++result.rigSamples;
			}
		}
		if (result.boneSamples > 0) {
			result.boneCenter /= static_cast<float>(result.boneSamples);
		}
		if (result.localBoneSamples > 0) {
			result.localBoneCenter /= static_cast<float>(result.localBoneSamples);
		}
		if (result.worldBoneSamples > 0) {
			result.worldBoneCenter /= static_cast<float>(result.worldBoneSamples);
		}
		if (result.rigSamples > 0) {
			result.rigCenter /= static_cast<float>(result.rigSamples);
		}

		const auto meshLooksLocal = !LooksWorldSpace(result.aabbCenter);
		if (meshLooksLocal && result.rigSamples > 0 && result.localBoneSamples > 0) {
			result.transform.setOrigin(result.rigCenter - result.localBoneCenter);
			result.visualSource = "rigLocalBoneDelta";
		} else if (meshLooksLocal && result.worldBoneSamples > 0) {
			result.transform.setOrigin(result.worldBoneCenter - result.meshCenter);
			result.visualSource = "worldBoneCenter";
		} else if (meshLooksLocal && LooksWorldSpace(result.actorPosition)) {
			result.transform.setOrigin(result.actorPosition);
			result.visualSource = "actorPosition";
		}
		result.applied = meshLooksLocal && LooksWorldSpace(result.transform.getOrigin());
		return result;
	}

	void DrawTransformedLine(ImDrawList& a_drawList, const Projector& a_projector, const btVector3& a_from, const btVector3& a_to, const MeshVisualAlignment& a_alignment, const ImU32 a_color)
	{
		if (a_alignment.applied) {
			DrawLine(a_drawList, a_projector, a_alignment.transform * a_from, a_alignment.transform * a_to, a_color);
		} else {
			DrawLine(a_drawList, a_projector, a_from, a_to, a_color);
		}
	}

	void DrawTransformedPoint(ImDrawList& a_drawList, const Projector& a_projector, const btVector3& a_world, const MeshVisualAlignment& a_alignment, const ImU32 a_color)
	{
		DrawPoint(a_drawList, a_projector, a_alignment.applied ? a_alignment.transform * a_world : a_world, a_color);
	}

	void DrawTransformedShapeAabb(ImDrawList& a_drawList, const Projector& a_projector, const hdt::SkinnedMeshBody& a_body, const MeshVisualAlignment& a_alignment, const ImU32 a_color)
	{
		btVector3 min;
		btVector3 max;
		a_body.bulletShape_.getAabb(a_body.getWorldTransform(), min, max);
		if (a_alignment.applied) {
			const btVector3 corners[8]{
				{ min.x(), min.y(), min.z() }, { max.x(), min.y(), min.z() },
				{ max.x(), max.y(), min.z() }, { min.x(), max.y(), min.z() },
				{ min.x(), min.y(), max.z() }, { max.x(), min.y(), max.z() },
				{ max.x(), max.y(), max.z() }, { min.x(), max.y(), max.z() },
			};
			const int edges[12][2]{
				{ 0, 1 }, { 1, 2 }, { 2, 3 }, { 3, 0 }, { 4, 5 }, { 5, 6 },
				{ 6, 7 }, { 7, 4 }, { 0, 4 }, { 1, 5 }, { 2, 6 }, { 3, 7 },
			};
			for (const auto& edge : edges) {
				DrawLine(a_drawList, a_projector, a_alignment.transform * corners[edge[0]], a_alignment.transform * corners[edge[1]], a_color);
			}
		} else {
			DrawAabb(a_drawList, a_projector, min, max, a_color);
		}
	}

	MeshVisualAlignment DrawSkinnedMeshBody(ImDrawList& a_drawList, const Projector& a_projector, const hdt::SkinnedMeshBody& a_body, const ImU32 a_color)
	{
		const auto visualAlignment = CalculateMeshVisualAlignment(a_body);
		if (!a_body.shape_) {
			DrawTransformedShapeAabb(a_drawList, a_projector, a_body, visualAlignment, a_color);
			return visualAlignment;
		}

		if (const auto* triangleShape = a_body.shape_->asPerTriangleShape()) {
			for (const auto& collider : triangleShape->colliders_) {
				const auto i0 = static_cast<std::size_t>(collider.vertices_[0]);
				const auto i1 = static_cast<std::size_t>(collider.vertices_[1]);
				const auto i2 = static_cast<std::size_t>(collider.vertices_[2]);
				if (i0 >= a_body.vertexPositions_.size() || i1 >= a_body.vertexPositions_.size() || i2 >= a_body.vertexPositions_.size()) {
					continue;
				}

				const auto p0 = a_body.vertexPositions_[i0].pos();
				const auto p1 = a_body.vertexPositions_[i1].pos();
				const auto p2 = a_body.vertexPositions_[i2].pos();
				DrawTransformedLine(a_drawList, a_projector, p0, p1, visualAlignment, a_color);
				DrawTransformedLine(a_drawList, a_projector, p1, p2, visualAlignment, a_color);
				DrawTransformedLine(a_drawList, a_projector, p2, p0, visualAlignment, a_color);
			}
			if (visualAlignment.applied) {
				DrawTransformedShapeAabb(a_drawList, a_projector, a_body, visualAlignment, IM_COL32(120, 255, 140, 70));
			}
			return visualAlignment;
		}

		if (const auto* vertexShape = a_body.shape_->asPerVertexShape()) {
			for (const auto& collider : vertexShape->colliders_) {
				const auto vertex = static_cast<std::size_t>(collider.vertex_);
				if (vertex < a_body.vertexPositions_.size()) {
					DrawTransformedPoint(a_drawList, a_projector, a_body.vertexPositions_[vertex].pos(), visualAlignment, a_color);
				}
			}
			DrawTransformedShapeAabb(a_drawList, a_projector, a_body, visualAlignment, IM_COL32(120, 255, 140, 90));
			return visualAlignment;
		}

		DrawTransformedShapeAabb(a_drawList, a_projector, a_body, visualAlignment, a_color);
		return visualAlignment;
	}
}

namespace Smp::BulletVisualization
{
	void DrawWorld(const btDiscreteDynamicsWorld* a_world)
	{
		if (!a_world) {
			return;
		}

		auto* drawList = ImGui::GetForegroundDrawList();
		if (!drawList) {
			return;
		}

		const auto projector = MakeProjector();
		const auto objectCount = a_world->getNumCollisionObjects();
		std::uint32_t rigidBodies = 0;
		std::uint32_t meshBodies = 0;
		std::uint32_t fallbackObjects = 0;
		std::uint32_t meshVisualOffsets = 0;
		std::vector<std::pair<const hdt::SkinnedMeshBody*, MeshVisualAlignment>> meshDebug;
		for (int index = 0; index < objectCount; ++index) {
			const auto* object = a_world->getCollisionObjectArray()[index];
			const auto* shape = object ? object->getCollisionShape() : nullptr;
			if (!object || !shape) {
				continue;
			}

			ImU32 color = kFallbackColor;
			const char* label = nullptr;
			if (const auto* body = btRigidBody::upcast(object); body && body->getUserPointer()) {
				++rigidBodies;
				const auto* bone = static_cast<const hdt::SkinnedMeshBone*>(body->getUserPointer());
				color = body->isStaticOrKinematicObject() ? kKinematicBoneColor : kDynamicBoneColor;
				label = bone->m_name.c_str();
			} else if (shape->getShapeType() == CUSTOM_CONCAVE_SHAPE_TYPE) {
				++meshBodies;
				const auto* mesh = static_cast<const hdt::SkinnedMeshBody*>(object);
				color = kMeshColor;
				label = mesh->name_.c_str();
			} else {
				++fallbackObjects;
			}

			if (shape->getShapeType() == CUSTOM_CONCAVE_SHAPE_TYPE) {
				const auto* mesh = static_cast<const hdt::SkinnedMeshBody*>(object);
				const auto alignment = DrawSkinnedMeshBody(*drawList, projector, *mesh, color);
				if (alignment.applied) {
					++meshVisualOffsets;
				}
				meshDebug.emplace_back(mesh, alignment);
				DrawText(*drawList, projector, alignment.applied ? alignment.transform * alignment.meshCenter : alignment.meshCenter, label);
			} else {
				DrawShape(*drawList, projector, *shape, object->getWorldTransform(), color);
				DrawText(*drawList, projector, object->getWorldTransform().getOrigin(), label);
			}
		}

		char status[256]{};
		std::snprintf(
			status,
			sizeof(status),
			"SMP Bullet vis: objects=%d rigid=%u meshes=%u meshOffsets=%u projected=%u/%u camera=%p %s caches=%u adj=(%.0f %.0f %.0f)/(%.0f %.0f %.0f) size=%.0fx%.0f",
				objectCount,
				rigidBodies,
				meshBodies,
				meshVisualOffsets,
				projector.projectSuccesses,
				projector.projectAttempts,
				static_cast<const void*>(projector.camera),
				projector.cameraSource,
				projector.cameraCacheCount,
				projector.posAdjust.x,
				projector.posAdjust.y,
				projector.posAdjust.z,
				projector.currentPosAdjust.x,
				projector.currentPosAdjust.y,
				projector.currentPosAdjust.z,
				projector.width,
			projector.height);
		drawList->AddText(ImVec2(16.0F, 16.0F), kTextColor, status);

		static std::uint32_t frameCounter = 0;
		if (++frameCounter % 120 == 0) {
			spdlog::debug(
				"bullet visualization frame objects={} rigidBodies={} meshes={} meshVisualOffsets={} fallbackObjects={} projected={}/{} camera={} source={} caches={} posAdjust=({}, {}, {}) currentPosAdjust=({}, {}, {}) size={}x{}",
				objectCount,
				rigidBodies,
				meshBodies,
				meshVisualOffsets,
				fallbackObjects,
				projector.projectSuccesses,
				projector.projectAttempts,
				static_cast<const void*>(projector.camera),
				projector.cameraSource,
				projector.cameraCacheCount,
				projector.posAdjust.x,
				projector.posAdjust.y,
				projector.posAdjust.z,
				projector.currentPosAdjust.x,
				projector.currentPosAdjust.y,
				projector.currentPosAdjust.z,
				projector.width,
				projector.height);
			for (const auto& [mesh, alignment] : meshDebug) {
				if (!mesh) {
					continue;
				}
				spdlog::debug(
					"bullet visualization mesh name='{}' actor={} buildGroup={} samples(mesh={}, bone={}, localBone={}, worldBone={}, rig={}) visualOffsetApplied={} visualSource={} actorPosition=({}, {}, {}) visualOrigin=({}, {}, {}) objectOrigin=({}, {}, {}) aabbCenter=({}, {}, {}) meshCenter=({}, {}, {}) boneCenter=({}, {}, {}) localBoneCenter=({}, {}, {}) worldBoneCenter=({}, {}, {}) rigCenter=({}, {}, {})",
					mesh->name_.c_str(),
					static_cast<void*>(mesh->actor_),
					mesh->buildGroup_,
					alignment.meshSamples,
					alignment.boneSamples,
					alignment.localBoneSamples,
					alignment.worldBoneSamples,
					alignment.rigSamples,
					alignment.applied,
					alignment.visualSource,
					alignment.actorPosition.x(),
					alignment.actorPosition.y(),
					alignment.actorPosition.z(),
					alignment.transform.getOrigin().x(),
					alignment.transform.getOrigin().y(),
					alignment.transform.getOrigin().z(),
					alignment.objectOrigin.x(),
					alignment.objectOrigin.y(),
					alignment.objectOrigin.z(),
					alignment.aabbCenter.x(),
					alignment.aabbCenter.y(),
					alignment.aabbCenter.z(),
					alignment.meshCenter.x(),
					alignment.meshCenter.y(),
					alignment.meshCenter.z(),
					alignment.boneCenter.x(),
					alignment.boneCenter.y(),
					alignment.boneCenter.z(),
					alignment.localBoneCenter.x(),
					alignment.localBoneCenter.y(),
					alignment.localBoneCenter.z(),
					alignment.worldBoneCenter.x(),
					alignment.worldBoneCenter.y(),
					alignment.worldBoneCenter.z(),
					alignment.rigCenter.x(),
					alignment.rigCenter.y(),
					alignment.rigCenter.z());
			}
		}
	}
}
