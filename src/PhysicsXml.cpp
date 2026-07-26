#include "PhysicsXml.h"

#include "ConfigPaths.h"

#include <tinyxml2.h>

#include <cerrno>
#include <charconv>
#include <cfloat>
#include <cstdlib>
#include <cmath>
#include <unordered_map>

namespace
{
	using Smp::ConfigPaths::LowerString;
	using Smp::ConfigPaths::PathExists;
	using Smp::ConfigPaths::Trim;

	bool ParseInt(std::string a_value, int& a_out)
	{
		a_value = Trim(std::move(a_value));
		if (a_value.empty()) {
			return false;
		}

		char* end = nullptr;
		errno = 0;
		const auto value = std::strtol(a_value.c_str(), std::addressof(end), 0);
		if (errno != 0 || end != a_value.c_str() + a_value.size()) {
			return false;
		}

		a_out = static_cast<int>(value);
		return true;
	}

	bool ParseFloat(std::string a_value, float& a_out)
	{
		a_value = Trim(std::move(a_value));
		if (a_value.empty()) {
			return false;
		}

		if (const auto comma = a_value.find(','); comma != std::string::npos) {
			a_value.replace(comma, 1, ".");
		}

		const auto begin = a_value.data();
		const auto end = begin + a_value.size();
		const auto [ptr, error] = std::from_chars(begin, end, a_out);
		if (error != std::errc{} || ptr != end) {
			return false;
		}

		return true;
	}

	int ReadInt(tinyxml2::XMLElement* a_parent, const char* a_name, const int a_default)
	{
		if (const auto element = a_parent ? a_parent->FirstChildElement(a_name) : nullptr) {
			if (const auto text = element->GetText()) {
				if (int value = a_default; ParseInt(text, value)) {
					return value;
				}
			}
		}
		return a_default;
	}

	float ReadFloat(tinyxml2::XMLElement* a_parent, const char* a_name, const float a_default)
	{
		if (const auto element = a_parent ? a_parent->FirstChildElement(a_name) : nullptr) {
			if (const auto text = element->GetText()) {
				if (float value = a_default; ParseFloat(text, value)) {
					return value;
				}
			}
		}
		return a_default;
	}

	float ReadFloatElement(tinyxml2::XMLElement* a_element, const float a_default)
	{
		if (const auto text = a_element ? a_element->GetText() : nullptr) {
			if (float value = a_default; ParseFloat(text, value)) {
				return value;
			}
		}
		return a_default;
	}

	float ReadFloatAttribute(tinyxml2::XMLElement* a_element, const char* a_name, const float a_default)
	{
		if (const auto text = a_element ? a_element->Attribute(a_name) : nullptr) {
			if (float value = a_default; ParseFloat(text, value)) {
				return value;
			}
		}
		return a_default;
	}

	bool TryReadFloatAttribute(tinyxml2::XMLElement* a_element, const char* a_name, float& a_out)
	{
		const auto text = a_element ? a_element->Attribute(a_name) : nullptr;
		if (!text) {
			return false;
		}
		return ParseFloat(text, a_out);
	}

	bool ReadBool(tinyxml2::XMLElement* a_parent, const char* a_name, const bool a_default)
	{
		if (const auto element = a_parent ? a_parent->FirstChildElement(a_name) : nullptr) {
			bool value = a_default;
			if (element->QueryBoolText(std::addressof(value)) == tinyxml2::XML_SUCCESS) {
				return value;
			}

			if (const auto text = element->GetText()) {
				const auto valueText = LowerString(Trim(text));
				if (valueText == "1" || valueText == "true") {
					return true;
				}
				if (valueText == "0" || valueText == "false") {
					return false;
				}
			}
		}
		return a_default;
	}

	bool ReadBoolElement(tinyxml2::XMLElement* a_element, const bool a_default)
	{
		if (!a_element) {
			return a_default;
		}

		bool value = a_default;
		if (a_element->QueryBoolText(std::addressof(value)) == tinyxml2::XML_SUCCESS) {
			return value;
		}

		if (const auto text = a_element->GetText()) {
			const auto valueText = LowerString(Trim(text));
			if (valueText == "1" || valueText == "true") {
				return true;
			}
			if (valueText == "0" || valueText == "false") {
				return false;
			}
		}
		return a_default;
	}

	std::string ReadAttribute(tinyxml2::XMLElement* a_element, const char* a_name, const std::string_view a_default = {})
	{
		if (const auto value = a_element ? a_element->Attribute(a_name) : nullptr) {
			return Trim(value);
		}
		return Trim(std::string(a_default));
	}

	std::string ReadElementText(tinyxml2::XMLElement* a_element)
	{
		if (const auto text = a_element ? a_element->GetText() : nullptr) {
			return Trim(text);
		}
		return {};
	}

	Smp::XmlVector3 ReadVector3(tinyxml2::XMLElement* a_element, const Smp::XmlVector3& a_default)
	{
		auto result = a_default;
		if (!a_element) {
			return result;
		}

		result.x = ReadFloatAttribute(a_element, "x", result.x);
		result.y = ReadFloatAttribute(a_element, "y", result.y);
		result.z = ReadFloatAttribute(a_element, "z", result.z);
		return result;
	}

	Smp::XmlVector3 ReadRequiredVector3(
		tinyxml2::XMLElement* a_element,
		const Smp::XmlVector3& a_default,
		bool& a_valid,
		const std::string_view a_context)
	{
		auto result = a_default;
		if (!a_element) {
			return result;
		}

		float x = 0.0F;
		float y = 0.0F;
		float z = 0.0F;
		if (!TryReadFloatAttribute(a_element, "x", x) ||
			!TryReadFloatAttribute(a_element, "y", y) ||
			!TryReadFloatAttribute(a_element, "z", z)) {
			spdlog::warn("physics XML {} has incomplete vector '{}'; constraint skipped", a_context, a_element->Name());
			a_valid = false;
			return result;
		}

		return { .x = x, .y = y, .z = z };
	}

	Smp::XmlQuaternion ReadQuaternion(tinyxml2::XMLElement* a_element, const Smp::XmlQuaternion& a_default)
	{
		auto result = a_default;
		if (!a_element) {
			return result;
		}

		result.x = ReadFloatAttribute(a_element, "x", result.x);
		result.y = ReadFloatAttribute(a_element, "y", result.y);
		result.z = ReadFloatAttribute(a_element, "z", result.z);
		result.w = ReadFloatAttribute(a_element, "w", result.w);
		const auto lengthSquared = result.x * result.x + result.y * result.y + result.z * result.z + result.w * result.w;
		if (lengthSquared <= FLT_EPSILON) {
			return { .x = 0.0F, .y = 0.0F, .z = 0.0F, .w = 1.0F };
		}

		const auto invLength = 1.0F / std::sqrt(lengthSquared);
		result.x *= invLength;
		result.y *= invLength;
		result.z *= invLength;
		result.w *= invLength;
		return result;
	}

	Smp::XmlQuaternion ReadRequiredQuaternion(
		tinyxml2::XMLElement* a_element,
		const Smp::XmlQuaternion& a_default,
		bool& a_valid,
		const std::string_view a_context)
	{
		if (!a_element) {
			return a_default;
		}

		Smp::XmlQuaternion result;
		if (!TryReadFloatAttribute(a_element, "x", result.x) ||
			!TryReadFloatAttribute(a_element, "y", result.y) ||
			!TryReadFloatAttribute(a_element, "z", result.z) ||
			!TryReadFloatAttribute(a_element, "w", result.w)) {
			spdlog::warn("physics XML {} has incomplete quaternion '{}'; constraint skipped", a_context, a_element->Name());
			a_valid = false;
			return a_default;
		}

		const auto lengthSquared = result.x * result.x + result.y * result.y + result.z * result.z + result.w * result.w;
		if (lengthSquared <= FLT_EPSILON) {
			return { .x = 0.0F, .y = 0.0F, .z = 0.0F, .w = 1.0F };
		}

		const auto invLength = 1.0F / std::sqrt(lengthSquared);
		result.x *= invLength;
		result.y *= invLength;
		result.z *= invLength;
		result.w *= invLength;
		return result;
	}

	Smp::XmlQuaternion ReadAxisAngle(tinyxml2::XMLElement* a_element, const Smp::XmlQuaternion& a_default)
	{
		if (!a_element) {
			return a_default;
		}

		Smp::XmlVector3 axis{ 1.0F, 0.0F, 0.0F };
		axis = ReadVector3(a_element, axis);

		float angle = 0.0F;
		angle = ReadFloatAttribute(a_element, "angle", angle);
		const auto lengthSquared = axis.x * axis.x + axis.y * axis.y + axis.z * axis.z;
		if (lengthSquared <= FLT_EPSILON) {
			return { .x = 0.0F, .y = 0.0F, .z = 0.0F, .w = 1.0F };
		}

		const auto invLength = 1.0F / std::sqrt(lengthSquared);
		const auto halfAngle = angle * 0.5F;
		const auto sinHalf = std::sin(halfAngle);
		return {
			.x = axis.x * invLength * sinHalf,
			.y = axis.y * invLength * sinHalf,
			.z = axis.z * invLength * sinHalf,
			.w = std::cos(halfAngle),
		};
	}

	Smp::XmlQuaternion ReadRequiredAxisAngle(
		tinyxml2::XMLElement* a_element,
		const Smp::XmlQuaternion& a_default,
		bool& a_valid,
		const std::string_view a_context)
	{
		if (!a_element) {
			return a_default;
		}

		Smp::XmlVector3 axis;
		float angle = 0.0F;
		if (!TryReadFloatAttribute(a_element, "x", axis.x) ||
			!TryReadFloatAttribute(a_element, "y", axis.y) ||
			!TryReadFloatAttribute(a_element, "z", axis.z) ||
			!TryReadFloatAttribute(a_element, "angle", angle)) {
			spdlog::warn("physics XML {} has incomplete axis-angle '{}'; constraint skipped", a_context, a_element->Name());
			a_valid = false;
			return a_default;
		}

		const auto lengthSquared = axis.x * axis.x + axis.y * axis.y + axis.z * axis.z;
		if (lengthSquared <= FLT_EPSILON) {
			return { .x = 0.0F, .y = 0.0F, .z = 0.0F, .w = 1.0F };
		}

		const auto invLength = 1.0F / std::sqrt(lengthSquared);
		const auto halfAngle = angle * 0.5F;
		const auto sinHalf = std::sin(halfAngle);
		return {
			.x = axis.x * invLength * sinHalf,
			.y = axis.y * invLength * sinHalf,
			.z = axis.z * invLength * sinHalf,
			.w = std::cos(halfAngle),
		};
	}

	Smp::XmlTransform ReadTransform(tinyxml2::XMLElement* a_element, const Smp::XmlTransform& a_default)
	{
		auto result = a_default;
		if (!a_element) {
			return result;
		}

		for (auto* child = a_element->FirstChildElement(); child; child = child->NextSiblingElement()) {
			const std::string name = child->Name();
			if (name == "origin") {
				result.origin = ReadVector3(child, result.origin);
			} else if (name == "basis") {
				result.rotation = ReadQuaternion(child, result.rotation);
			} else if (name == "basis-axis-angle") {
				result.rotation = ReadAxisAngle(child, result.rotation);
			}
		}
		return result;
	}

	Smp::XmlTransform ReadRequiredTransform(
		tinyxml2::XMLElement* a_element,
		const Smp::XmlTransform& a_default,
		bool& a_valid,
		const std::string_view a_context)
	{
		auto result = a_default;
		if (!a_element) {
			return result;
		}

		for (auto* child = a_element->FirstChildElement(); child; child = child->NextSiblingElement()) {
			const std::string name = child->Name();
			if (name == "origin") {
				result.origin = ReadRequiredVector3(child, result.origin, a_valid, a_context);
			} else if (name == "basis") {
				result.rotation = ReadRequiredQuaternion(child, result.rotation, a_valid, a_context);
			} else if (name == "basis-axis-angle") {
				result.rotation = ReadRequiredAxisAngle(child, result.rotation, a_valid, a_context);
			}
		}
		return result;
	}

	Smp::PhysicsShapeDescriptor ReadShape(
		tinyxml2::XMLElement* a_shape,
		const Smp::PhysicsShapeDescriptor& a_default,
		const std::unordered_map<std::string, Smp::PhysicsShapeDescriptor>* a_namedShapes = nullptr)
	{
		auto result = a_default;
		if (!a_shape) {
			return result;
		}

		const auto type = ReadAttribute(a_shape, "type");
		if (type == "ref") {
			const auto name = ReadAttribute(a_shape, "name");
			if (a_namedShapes) {
				if (const auto found = a_namedShapes->find(name); found != a_namedShapes->end()) {
					return found->second;
				}
			}
			spdlog::warn("physics XML references unknown shape '{}'", name);
			result.valid = false;
			return result;
		} else if (type == "box") {
			result.valid = true;
			result.kind = Smp::PhysicsShapeKind::kBox;
			result.halfExtents = ReadVector3(a_shape->FirstChildElement("halfExtend"), result.halfExtents);
			result.margin = ReadFloat(a_shape, "margin", result.margin);
		} else if (type == "capsule") {
			result.valid = true;
			result.kind = Smp::PhysicsShapeKind::kCapsule;
			result.radius = ReadFloat(a_shape, "radius", result.radius);
			result.height = ReadFloat(a_shape, "height", result.height);
		} else if (type == "cylinder") {
			result.valid = true;
			result.kind = Smp::PhysicsShapeKind::kCylinder;
			result.radius = ReadFloat(a_shape, "radius", result.radius);
			result.height = ReadFloat(a_shape, "height", result.height);
			result.margin = ReadFloat(a_shape, "margin", result.margin);
		} else if (type == "hull") {
			result.valid = true;
			result.kind = Smp::PhysicsShapeKind::kHull;
			result.margin = ReadFloat(a_shape, "margin", result.margin);
			result.hullPoints.clear();
			for (auto* child = a_shape->FirstChildElement("point"); child; child = child->NextSiblingElement("point")) {
				result.hullPoints.push_back(ReadVector3(child, {}));
			}
			result.valid = !result.hullPoints.empty();
		} else if (type == "compound") {
			result.valid = true;
			result.kind = Smp::PhysicsShapeKind::kCompound;
			result.compoundChildren.clear();
			for (auto* child = a_shape->FirstChildElement("child"); child; child = child->NextSiblingElement("child")) {
				Smp::XmlTransform transform;
				if (auto* transformElement = child->FirstChildElement("transform")) {
					transform = ReadTransform(transformElement, transform);
				}
				if (auto* shapeElement = child->FirstChildElement("shape")) {
					auto childShape = ReadShape(shapeElement, {}, a_namedShapes);
					if (childShape.valid) {
						result.compoundChildren.emplace_back(transform, std::move(childShape));
					}
				}
			}
			result.valid = !result.compoundChildren.empty();
		} else if (type == "sphere") {
			result.valid = true;
			result.kind = Smp::PhysicsShapeKind::kSphere;
			result.radius = ReadFloat(a_shape, "radius", result.radius);
		} else {
			spdlog::warn("physics XML has unknown shape type '{}'", type);
			result.valid = false;
		}

		return result;
	}

	Smp::PhysicsMeshShapeDescriptor ReadMeshShape(tinyxml2::XMLElement* a_element, const Smp::PhysicsMeshShapeKind a_kind)
	{
		Smp::PhysicsMeshShapeDescriptor result;
		result.kind = a_kind;
		result.name = ReadAttribute(a_element, "name");
		if (a_element && a_element->FirstChildElement("margin")) {
			result.margin = ReadFloat(a_element, "margin", result.margin);
			result.hasMargin = true;
		}
		if (a_element && a_element->FirstChildElement("penetration")) {
			result.penetration = ReadFloat(a_element, "penetration", result.penetration);
			result.hasPenetration = true;
		}
		if (a_element && a_element->FirstChildElement("prenetration")) {
			result.penetration = ReadFloat(a_element, "prenetration", result.penetration);
			result.hasPenetration = true;
		}

		for (auto* child = a_element ? a_element->FirstChildElement() : nullptr; child; child = child->NextSiblingElement()) {
			const std::string_view name(child->Name());
			if (name == "tag") {
				auto value = ReadElementText(child);
				if (!value.empty()) {
					result.tags.push_back(std::move(value));
				}
			} else if (name == "can-collide-with-tag") {
				auto value = ReadElementText(child);
				if (!value.empty()) {
					result.canCollideWithTags.push_back(std::move(value));
				}
			} else if (name == "no-collide-with-tag") {
				auto value = ReadElementText(child);
				if (!value.empty()) {
					result.noCollideWithTags.push_back(std::move(value));
				}
			} else if (name == "can-collide-with-bone") {
				auto value = ReadElementText(child);
				if (!value.empty()) {
					result.canCollideWithBones.push_back(std::move(value));
				}
			} else if (name == "no-collide-with-bone") {
				auto value = ReadElementText(child);
				if (!value.empty()) {
					result.noCollideWithBones.push_back(std::move(value));
				}
			} else if (name == "weight-threshold") {
				auto bone = ReadAttribute(child, "bone");
				float threshold = 0.0F;
				ParseFloat(ReadElementText(child), threshold);
				if (!bone.empty()) {
					result.weightThresholds.emplace_back(std::move(bone), threshold);
				}
			} else if (name == "disable-tag") {
				result.disableTag = ReadElementText(child);
			} else if (name == "disable-priority") {
				int priority = result.disablePriority;
				if (ParseInt(ReadElementText(child), priority)) {
					result.disablePriority = priority;
				}
			} else if (name == "shared") {
				const auto value = ReadElementText(child);
				if (value == "internal") {
					result.shared = Smp::PhysicsMeshSharedScope::kInternal;
				} else if (value == "external") {
					result.shared = Smp::PhysicsMeshSharedScope::kExternal;
				} else if (value == "private") {
					result.shared = Smp::PhysicsMeshSharedScope::kPrivate;
				} else {
					result.shared = Smp::PhysicsMeshSharedScope::kPublic;
				}
			}
		}
		return result;
	}

	void ApplyBoneChildren(
		tinyxml2::XMLElement* a_boneElement,
		Smp::PhysicsBoneDescriptor& a_bone,
		const std::unordered_map<std::string, Smp::PhysicsShapeDescriptor>& a_namedShapes)
	{
		a_bone.mass = ReadFloat(a_boneElement, "mass", a_bone.mass);
		if (const auto localInertia = a_boneElement ? a_boneElement->FirstChildElement("localInertia") : nullptr) {
			a_bone.localInertia = ReadVector3(localInertia, a_bone.localInertia);
			a_bone.hasLocalInertia = true;
		} else if (const auto inertia = a_boneElement ? a_boneElement->FirstChildElement("inertia") : nullptr) {
			a_bone.localInertia = ReadVector3(inertia, a_bone.localInertia);
			a_bone.hasLocalInertia = true;
		}
		a_bone.linearDamping = ReadFloat(a_boneElement, "linearDamping", a_bone.linearDamping);
		a_bone.angularDamping = ReadFloat(a_boneElement, "angularDamping", a_bone.angularDamping);
		a_bone.friction = ReadFloat(a_boneElement, "friction", a_bone.friction);
		a_bone.rollingFriction = ReadFloat(a_boneElement, "rollingFriction", a_bone.rollingFriction);
		a_bone.restitution = ReadFloat(a_boneElement, "restitution", a_bone.restitution);
		a_bone.gravityFactor = std::clamp(ReadFloat(a_boneElement, "gravity-factor", a_bone.gravityFactor), 0.0F, 1.0F);
		a_bone.windFactor = std::max(ReadFloat(a_boneElement, "wind-factor", a_bone.windFactor), 0.0F);
		a_bone.marginMultiplier = ReadFloat(a_boneElement, "margin-multiplier", a_bone.marginMultiplier);
		a_bone.collisionFilter = ReadInt(a_boneElement, "collision-filter", a_bone.collisionFilter);
		a_bone.centerOfMassTransform = ReadTransform(a_boneElement->FirstChildElement("centerOfMassTransform"), a_bone.centerOfMassTransform);

		bool clearedInheritedCollisionLists = false;
		for (auto* child = a_boneElement ? a_boneElement->FirstChildElement() : nullptr; child; child = child->NextSiblingElement()) {
			const std::string_view name(child->Name());
			if (name == "can-collide-with-bone") {
				if (!clearedInheritedCollisionLists) {
					a_bone.canCollideWithBones.clear();
					a_bone.noCollideWithBones.clear();
					clearedInheritedCollisionLists = true;
				}
				auto value = ReadElementText(child);
				if (!value.empty()) {
					a_bone.canCollideWithBones.push_back(std::move(value));
				}
			} else if (name == "no-collide-with-bone") {
				if (!clearedInheritedCollisionLists) {
					a_bone.canCollideWithBones.clear();
					a_bone.noCollideWithBones.clear();
					clearedInheritedCollisionLists = true;
				}
				auto value = ReadElementText(child);
				if (!value.empty()) {
					a_bone.noCollideWithBones.push_back(std::move(value));
				}
			}
		}

		if (const auto shape = a_boneElement ? a_boneElement->FirstChildElement("shape") : nullptr) {
			a_bone.shape = ReadShape(shape, Smp::PhysicsShapeDescriptor{}, std::addressof(a_namedShapes));
			a_bone.hasShape = a_bone.shape.valid;
		}
	}

	Smp::PhysicsConstraintDescriptor ReadConstraint(
		tinyxml2::XMLElement* a_constraint,
		const Smp::PhysicsConstraintKind a_kind,
		const Smp::PhysicsConstraintDescriptor& a_base = {})
	{
		auto result = a_base;
		result.kind = a_kind;
		result.name = ReadAttribute(a_constraint, "name", result.name);
		result.bodyA = ReadAttribute(a_constraint, "bodyA", result.bodyA);
		result.bodyB = ReadAttribute(a_constraint, "bodyB", result.bodyB);
		result.templateName = ReadAttribute(a_constraint, "template", result.templateName);
		const auto context = result.name.empty() ? std::string_view(a_constraint->Name()) : std::string_view(result.name);

		const auto readFrame = [&](tinyxml2::XMLElement* a_element, const std::string_view a_name) {
			if (a_name == "frameInA") {
				result.frameMode = Smp::PhysicsFrameMode::kFrameInA;
				result.frame = ReadRequiredTransform(a_element, Smp::XmlTransform{}, result.valid, context);
				result.frameInA = result.frame;
				return true;
			}
			if (a_name == "frameInB") {
				result.frameMode = Smp::PhysicsFrameMode::kFrameInB;
				result.frame = ReadRequiredTransform(a_element, Smp::XmlTransform{}, result.valid, context);
				result.frameInB = result.frame;
				return true;
			}
			if (a_name == "frameInLerp") {
				result.frameMode = Smp::PhysicsFrameMode::kFrameInLerp;
				result.frame = Smp::XmlTransform{};
				result.translationLerp = 0.0F;
				result.rotationLerp = 0.0F;
				for (auto* child = a_element->FirstChildElement(); child; child = child->NextSiblingElement()) {
					const std::string_view childName(child->Name());
					if (childName == "translationLerp") {
						result.translationLerp = ReadFloatElement(child, result.translationLerp);
					} else if (childName == "rotationLerp") {
						result.rotationLerp = ReadFloatElement(child, result.rotationLerp);
					}
				}
				return true;
			}

			// Retain the point-to-body frame modes supported by this port. They do not
			// change the behavior of any frame mode accepted by the reference parser.
			if (a_name == "AWithXPointToB" || a_name == "a-with-x-point-to-b") {
				result.frameMode = Smp::PhysicsFrameMode::kAWithXPointToB;
				return true;
			}
			if (a_name == "AWithYPointToB" || a_name == "a-with-y-point-to-b") {
				result.frameMode = Smp::PhysicsFrameMode::kAWithYPointToB;
				return true;
			}
			if (a_name == "AWithZPointToB" || a_name == "a-with-z-point-to-b") {
				result.frameMode = Smp::PhysicsFrameMode::kAWithZPointToB;
				return true;
			}
			return false;
		};

		for (auto* child = a_constraint->FirstChildElement(); child; child = child->NextSiblingElement()) {
			const std::string_view name(child->Name());
			if (a_kind != Smp::PhysicsConstraintKind::kStiffSpring && readFrame(child, name)) {
				continue;
			}

			if (a_kind == Smp::PhysicsConstraintKind::kGeneric) {
				if (name == "enableLinearSprings") {
					result.enableLinearSprings = ReadBoolElement(child, result.enableLinearSprings);
				} else if (name == "enableAngularSprings") {
					result.enableAngularSprings = ReadBoolElement(child, result.enableAngularSprings);
				} else if (name == "linearStiffnessLimited") {
					result.linearStiffnessLimited = ReadBoolElement(child, result.linearStiffnessLimited);
				} else if (name == "angularStiffnessLimited") {
					result.angularStiffnessLimited = ReadBoolElement(child, result.angularStiffnessLimited);
				} else if (name == "springDampingLimited") {
					result.springDampingLimited = ReadBoolElement(child, result.springDampingLimited);
				} else if (name == "linearNonHookeanDamping") {
					result.linearNonHookeanDamping = ReadRequiredVector3(child, result.linearNonHookeanDamping, result.valid, context);
				} else if (name == "angularNonHookeanDamping") {
					result.angularNonHookeanDamping = ReadRequiredVector3(child, result.angularNonHookeanDamping, result.valid, context);
				} else if (name == "linearNonHookeanStiffness") {
					result.linearNonHookeanStiffness = ReadRequiredVector3(child, result.linearNonHookeanStiffness, result.valid, context);
				} else if (name == "angularNonHookeanStiffness") {
					result.angularNonHookeanStiffness = ReadRequiredVector3(child, result.angularNonHookeanStiffness, result.valid, context);
				} else if (name == "linearMotors") {
					result.linearMotors = ReadBoolElement(child, result.linearMotors);
				} else if (name == "angularMotors") {
					result.angularMotors = ReadBoolElement(child, result.angularMotors);
				} else if (name == "linearServoMotors") {
					result.linearServoMotors = ReadBoolElement(child, result.linearServoMotors);
				} else if (name == "angularServoMotors") {
					result.angularServoMotors = ReadBoolElement(child, result.angularServoMotors);
				} else if (name == "linearTargetVelocity") {
					result.linearTargetVelocity = ReadRequiredVector3(child, result.linearTargetVelocity, result.valid, context);
				} else if (name == "angularTargetVelocity") {
					result.angularTargetVelocity = ReadRequiredVector3(child, result.angularTargetVelocity, result.valid, context);
				} else if (name == "linearMaxMotorForce") {
					result.linearMaxMotorForce = ReadRequiredVector3(child, result.linearMaxMotorForce, result.valid, context);
				} else if (name == "angularMaxMotorForce") {
					result.angularMaxMotorForce = ReadRequiredVector3(child, result.angularMaxMotorForce, result.valid, context);
				} else if (name == "stopERP") {
					result.stopErp = ReadFloatElement(child, result.stopErp);
				} else if (name == "stopCFM") {
					result.stopCfm = ReadFloatElement(child, result.stopCfm);
				} else if (name == "motorERP") {
					result.motorErp = ReadFloatElement(child, result.motorErp);
				} else if (name == "motorCFM") {
					result.motorCfm = ReadFloatElement(child, result.motorCfm);
				} else if (name == "useLinearReferenceFrameA") {
					result.useLinearReferenceFrameA = ReadBoolElement(child, result.useLinearReferenceFrameA);
				} else if (name == "linearLowerLimit") {
					result.linearLowerLimit = ReadRequiredVector3(child, result.linearLowerLimit, result.valid, context);
				} else if (name == "linearUpperLimit") {
					result.linearUpperLimit = ReadRequiredVector3(child, result.linearUpperLimit, result.valid, context);
				} else if (name == "angularLowerLimit") {
					result.angularLowerLimit = ReadRequiredVector3(child, result.angularLowerLimit, result.valid, context);
				} else if (name == "angularUpperLimit") {
					result.angularUpperLimit = ReadRequiredVector3(child, result.angularUpperLimit, result.valid, context);
				} else if (name == "linearStiffness") {
					result.linearStiffness = ReadRequiredVector3(child, result.linearStiffness, result.valid, context);
				} else if (name == "angularStiffness") {
					result.angularStiffness = ReadRequiredVector3(child, result.angularStiffness, result.valid, context);
				} else if (name == "linearDamping") {
					result.linearDamping = ReadRequiredVector3(child, result.linearDamping, result.valid, context);
				} else if (name == "angularDamping") {
					result.angularDamping = ReadRequiredVector3(child, result.angularDamping, result.valid, context);
				} else if (name == "linearEquilibrium") {
					result.linearEquilibrium = ReadRequiredVector3(child, result.linearEquilibrium, result.valid, context);
				} else if (name == "angularEquilibrium") {
					result.angularEquilibrium = ReadRequiredVector3(child, result.angularEquilibrium, result.valid, context);
				} else if (name == "linearBounce") {
					result.linearBounce = ReadRequiredVector3(child, result.linearBounce, result.valid, context);
				} else if (name == "angularBounce") {
					result.angularBounce = ReadRequiredVector3(child, result.angularBounce, result.valid, context);
				}
			} else if (a_kind == Smp::PhysicsConstraintKind::kConeTwist) {
				if (name == "swingSpan1" || name == "coneLimit" || name == "limitZ") {
					result.swingSpan1 = std::max(ReadFloatElement(child, result.swingSpan1), 0.0F);
				} else if (name == "swingSpan2" || name == "planeLimit" || name == "limitY") {
					result.swingSpan2 = std::max(ReadFloatElement(child, result.swingSpan2), 0.0F);
				} else if (name == "twistSpan" || name == "twistLimit" || name == "limitX") {
					result.twistSpan = std::max(ReadFloatElement(child, result.twistSpan), 0.0F);
				} else if (name == "limitSoftness") {
					result.limitSoftness = std::clamp(ReadFloatElement(child, result.limitSoftness), 0.0F, 1.0F);
				} else if (name == "biasFactor") {
					result.biasFactor = std::clamp(ReadFloatElement(child, result.biasFactor), 0.0F, 1.0F);
				} else if (name == "relaxationFactor") {
					result.relaxationFactor = std::clamp(ReadFloatElement(child, result.relaxationFactor), 0.0F, 1.0F);
				}
			} else if (a_kind == Smp::PhysicsConstraintKind::kStiffSpring) {
				if (name == "minDistanceFactor") {
					result.minDistanceFactor = std::max(ReadFloatElement(child, result.minDistanceFactor), 0.0F);
				} else if (name == "maxDistanceFactor") {
					result.maxDistanceFactor = std::max(ReadFloatElement(child, result.maxDistanceFactor), 0.0F);
				} else if (name == "stiffness") {
					result.stiffness = std::max(ReadFloatElement(child, result.stiffness), 0.0F);
				} else if (name == "damping") {
					result.damping = std::max(ReadFloatElement(child, result.damping), 0.0F);
				} else if (name == "equilibrium") {
					result.equilibriumFactor = std::clamp(ReadFloatElement(child, result.equilibriumFactor), 0.0F, 1.0F);
				}
			}
		}
		return result;
	}

	void AddUniqueBoneName(Smp::PhysicsXmlSummary& a_summary, const char* a_name)
	{
		if (!a_name || *a_name == '\0') {
			return;
		}

		if (std::ranges::find(a_summary.boneNames, a_name) == a_summary.boneNames.end()) {
			a_summary.boneNames.emplace_back(a_name);
		}
	}

	void AddMeshFilterBoneNames(Smp::PhysicsXmlSummary& a_summary, const Smp::PhysicsMeshShapeDescriptor& a_descriptor)
	{
		for (const auto& boneName : a_descriptor.canCollideWithBones) {
			AddUniqueBoneName(a_summary, boneName.c_str());
		}
		for (const auto& boneName : a_descriptor.noCollideWithBones) {
			AddUniqueBoneName(a_summary, boneName.c_str());
		}
	}

	void CountKnownElement(tinyxml2::XMLElement* a_element, Smp::PhysicsXmlSummary& a_summary)
	{
		for (auto* child = a_element->FirstChildElement(); child; child = child->NextSiblingElement()) {
			const std::string_view name(child->Name());
			if (name == "bone") {
				++a_summary.bones;
				AddUniqueBoneName(a_summary, child->Attribute("name"));
			} else if (name == "bone-default") {
				++a_summary.boneDefaults;
			} else if (name == "per-vertex-shape") {
				++a_summary.perVertexShapes;
			} else if (name == "per-triangle-shape") {
				++a_summary.perTriangleShapes;
			} else if (name == "shape") {
				++a_summary.shapes;
			} else if (name == "constraint-group") {
				++a_summary.constraintGroups;
			} else if (
				name == "generic-constraint" ||
				name == "stiffspring-constraint" ||
				name == "conetwist-constraint") {
				++a_summary.constraints;
			}

			CountKnownElement(child, a_summary);
		}
	}

	std::optional<std::filesystem::file_time_type> GetLastWriteTime(const std::filesystem::path& a_path)
	{
		std::error_code error;
		const auto timestamp = std::filesystem::last_write_time(a_path, error);
		if (error) {
			return std::nullopt;
		}
		return timestamp;
	}
}

namespace Smp
{
	PhysicsXmlLoader* PhysicsXmlLoader::GetSingleton()
	{
		static PhysicsXmlLoader singleton;
		return std::addressof(singleton);
	}

	void PhysicsXmlLoader::ClearCache()
	{
		summaryCache_.clear();
	}

	std::optional<PhysicsXmlSummary> PhysicsXmlLoader::LoadSummary(const std::string& a_path)
	{
		if (a_path.empty()) {
			return std::nullopt;
		}

		const auto resolvedPath = Smp::ConfigPaths::ResolveConfigPath(a_path, true);
		const auto cacheKey = resolvedPath.string();
		const auto currentTimestamp = GetLastWriteTime(resolvedPath);
		if (currentTimestamp) {
			if (const auto found = summaryCache_.find(cacheKey); found != summaryCache_.end()) {
				if (found->second.timestamp == *currentTimestamp) {
					return found->second.summary;
				}
				spdlog::debug("physics XML timestamp changed, reloading {}", cacheKey);
			}
		}

		PhysicsXmlSummary loaded;
		loaded.path = resolvedPath;
		if (!PathExists(loaded.path)) {
			spdlog::warn("physics XML not found: {}", a_path);
			return std::nullopt;
		}

		tinyxml2::XMLDocument document;
		const auto error = document.LoadFile(loaded.path.string().c_str());
		if (error != tinyxml2::XML_SUCCESS) {
			spdlog::error("failed to parse physics XML {}: {}", loaded.path.string(), document.ErrorStr());
			return std::nullopt;
		}

		const auto system = document.FirstChildElement("system");
		loaded.validSystemRoot = system != nullptr;
		if (!system) {
			spdlog::warn("physics XML {} does not contain a <system> root", loaded.path.string());
			return std::nullopt;
		}

		CountKnownElement(system, loaded);

		std::unordered_map<std::string, PhysicsShapeDescriptor> namedShapes;
		std::unordered_map<std::string, PhysicsBoneDescriptor> templates;
		std::unordered_map<std::string, PhysicsConstraintDescriptor> genericConstraintTemplates;
		std::unordered_map<std::string, PhysicsConstraintDescriptor> coneTwistConstraintTemplates;
		std::unordered_map<std::string, PhysicsConstraintDescriptor> stiffSpringConstraintTemplates;

		const auto findExistingBoneTemplate = [](const auto& a_templates, const std::string& a_name) {
			if (const auto found = a_templates.find(a_name); found != a_templates.end()) {
				return found->second;
			}
			if (const auto found = a_templates.find(""); found != a_templates.end()) {
				return found->second;
			}
			return PhysicsBoneDescriptor{};
		};

		const auto findExistingConstraintTemplate = [](const auto& a_templates, const std::string& a_name) {
			if (const auto found = a_templates.find(a_name); found != a_templates.end()) {
				return found->second;
			}
			if (const auto found = a_templates.find(""); found != a_templates.end()) {
				return found->second;
			}
			return PhysicsConstraintDescriptor{};
		};

		const auto processConstraintElement = [&](tinyxml2::XMLElement* child) {
			const std::string_view name(child->Name());
			if (name == "generic-constraint-default") {
				const auto templateName = ReadAttribute(child, "name");
				const auto extends = ReadAttribute(child, "extends");
				const auto base = findExistingConstraintTemplate(genericConstraintTemplates, extends);
				auto descriptor = ReadConstraint(child, PhysicsConstraintKind::kGeneric, base);
				descriptor.name = templateName;
				if (descriptor.valid) {
					genericConstraintTemplates[templateName] = descriptor;
				}
			} else if (name == "conetwist-constraint-default") {
				const auto templateName = ReadAttribute(child, "name");
				const auto extends = ReadAttribute(child, "extends");
				const auto base = findExistingConstraintTemplate(coneTwistConstraintTemplates, extends);
				auto descriptor = ReadConstraint(child, PhysicsConstraintKind::kConeTwist, base);
				descriptor.name = templateName;
				if (descriptor.valid) {
					coneTwistConstraintTemplates[templateName] = descriptor;
				}
			} else if (name == "stiffspring-constraint-default") {
				const auto templateName = ReadAttribute(child, "name");
				const auto extends = ReadAttribute(child, "extends");
				const auto base = findExistingConstraintTemplate(stiffSpringConstraintTemplates, extends);
				auto descriptor = ReadConstraint(child, PhysicsConstraintKind::kStiffSpring, base);
				descriptor.name = templateName;
				if (descriptor.valid) {
					stiffSpringConstraintTemplates[templateName] = descriptor;
				}
			} else if (name == "generic-constraint") {
				const auto templateName = ReadAttribute(child, "template");
				auto descriptor = ReadConstraint(child, PhysicsConstraintKind::kGeneric, findExistingConstraintTemplate(genericConstraintTemplates, templateName));
				if (descriptor.valid) {
					loaded.constraintDescriptors.push_back(std::move(descriptor));
				}
			} else if (name == "conetwist-constraint") {
				const auto templateName = ReadAttribute(child, "template");
				auto descriptor = ReadConstraint(child, PhysicsConstraintKind::kConeTwist, findExistingConstraintTemplate(coneTwistConstraintTemplates, templateName));
				if (descriptor.valid) {
					loaded.constraintDescriptors.push_back(std::move(descriptor));
				}
			} else if (name == "stiffspring-constraint") {
				const auto templateName = ReadAttribute(child, "template");
				auto descriptor = ReadConstraint(child, PhysicsConstraintKind::kStiffSpring, findExistingConstraintTemplate(stiffSpringConstraintTemplates, templateName));
				if (descriptor.valid) {
					loaded.constraintDescriptors.push_back(std::move(descriptor));
				}
			}
		};

		for (auto* child = system->FirstChildElement(); child; child = child->NextSiblingElement()) {
			const std::string_view name(child->Name());
			if (name == "shape") {
				const auto shapeName = ReadAttribute(child, "name");
				if (!shapeName.empty()) {
					namedShapes[shapeName] = ReadShape(child, PhysicsShapeDescriptor{});
				}
			} else if (name == "bone-default") {
				const auto descriptorName = ReadAttribute(child, "name");
				const auto extends = ReadAttribute(child, "extends");
				auto descriptor = findExistingBoneTemplate(templates, extends);
				descriptor.name = descriptorName;
				descriptor.templateName = extends;
				ApplyBoneChildren(child, descriptor, namedShapes);
				templates[descriptor.name] = descriptor;
				if (descriptor.name.empty()) {
					loaded.defaultBoneDescriptor = descriptor;
				}
			} else if (name == "bone") {
				const auto boneName = ReadAttribute(child, "name");
				const auto templateName = ReadAttribute(child, "template");
				auto descriptor = findExistingBoneTemplate(templates, templateName);
				descriptor.name = boneName;
				descriptor.templateName = templateName;
				ApplyBoneChildren(child, descriptor, namedShapes);
				if (!descriptor.name.empty()) {
					loaded.boneDescriptors.push_back(descriptor);
				}
			} else if (name == "per-vertex-shape") {
				auto descriptor = ReadMeshShape(child, PhysicsMeshShapeKind::kPerVertex);
				if (!descriptor.name.empty()) {
					AddMeshFilterBoneNames(loaded, descriptor);
					loaded.meshDescriptors.push_back(std::move(descriptor));
				}
			} else if (name == "per-triangle-shape") {
				auto descriptor = ReadMeshShape(child, PhysicsMeshShapeKind::kPerTriangle);
				if (!descriptor.name.empty()) {
					AddMeshFilterBoneNames(loaded, descriptor);
					loaded.meshDescriptors.push_back(std::move(descriptor));
				}
			} else if (name == "constraint-group") {
				for (auto* groupChild = child->FirstChildElement(); groupChild; groupChild = groupChild->NextSiblingElement()) {
					processConstraintElement(groupChild);
				}
			} else {
				processConstraintElement(child);
			}
		}

		if (!loaded.constraintDescriptors.empty()) {
			for (const auto& descriptor : loaded.constraintDescriptors) {
				AddUniqueBoneName(loaded, descriptor.bodyA.c_str());
				AddUniqueBoneName(loaded, descriptor.bodyB.c_str());
			}
		}
		if (const auto loadedTimestamp = currentTimestamp ? currentTimestamp : GetLastWriteTime(loaded.path)) {
			auto [entry, inserted] = summaryCache_.insert_or_assign(cacheKey, CachedSummary{
				.timestamp = *loadedTimestamp,
				.summary = std::move(loaded),
			});
			return entry->second.summary;
		}

		summaryCache_.erase(cacheKey);
		return loaded;
	}
}
