#pragma once

#include "RE/N/NiTransform.h"
#include "hdtSkinnedMesh/hdtBulletHelper.h"

#include <algorithm>
#include <cfloat>
#include <cmath>

namespace Smp::Fo4Transform
{
	[[nodiscard]] inline float ClampNiScale(const float a_scale)
	{
		return std::max(a_scale, FLT_EPSILON);
	}

	[[nodiscard]] inline float NormalizeNiScale(const float a_scale)
	{
		return std::isfinite(a_scale) && a_scale > FLT_EPSILON ? a_scale : 1.0F;
	}

	[[nodiscard]] inline btTransform ToBulletTransform(const RE::NiTransform& a_transform)
	{
		const btMatrix3x3 basis(
			a_transform.rotate[0].x,
			a_transform.rotate[0].y,
			a_transform.rotate[0].z,
			a_transform.rotate[1].x,
			a_transform.rotate[1].y,
			a_transform.rotate[1].z,
			a_transform.rotate[2].x,
			a_transform.rotate[2].y,
			a_transform.rotate[2].z);

		return btTransform(basis, btVector3(a_transform.translate.x, a_transform.translate.y, a_transform.translate.z));
	}

	[[nodiscard]] inline hdt::btQsTransform ToBulletQsTransform(const RE::NiTransform& a_transform)
	{
		return hdt::btQsTransform(ToBulletTransform(a_transform), ClampNiScale(a_transform.scale));
	}

	[[nodiscard]] inline hdt::btQsTransform ToBulletQsTransformNormalizedScale(const RE::NiTransform& a_transform)
	{
		return hdt::btQsTransform(ToBulletTransform(a_transform), NormalizeNiScale(a_transform.scale));
	}

	[[nodiscard]] inline RE::NiTransform ToNiTransform(const btTransform& a_transform, const float a_scale)
	{
		const auto& basis = a_transform.getBasis();
		RE::NiTransform result;
		result.rotate = RE::NiMatrix3(
			basis[0].x(), basis[0].y(), basis[0].z(), 0.0F,
			basis[1].x(), basis[1].y(), basis[1].z(), 0.0F,
			basis[2].x(), basis[2].y(), basis[2].z(), 0.0F);
		const auto origin = a_transform.getOrigin();
		result.translate = RE::NiPoint3(origin.x(), origin.y(), origin.z());
		result.scale = a_scale;
		return result;
	}

	[[nodiscard]] inline RE::NiTransform ToNiTransformNormalizedScale(const btTransform& a_transform, const float a_scale)
	{
		return ToNiTransform(a_transform, NormalizeNiScale(a_scale));
	}
}
