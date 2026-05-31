#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace Smp
{
	struct XmlVector3
	{
		float x{ 0.0F };
		float y{ 0.0F };
		float z{ 0.0F };
	};

	struct XmlQuaternion
	{
		float x{ 0.0F };
		float y{ 0.0F };
		float z{ 0.0F };
		float w{ 1.0F };
	};

	struct XmlTransform
	{
		XmlVector3 origin;
		XmlQuaternion rotation;
	};

	enum class PhysicsConstraintKind
	{
		kGeneric,
		kConeTwist,
		kStiffSpring
	};

	enum class PhysicsFrameMode
	{
		kFrameInB,
		kFrameInA,
		kFrameInLerp,
		kAWithXPointToB,
		kAWithYPointToB,
		kAWithZPointToB
	};

	struct PhysicsConstraintDescriptor
	{
		bool valid{ true };
		PhysicsConstraintKind kind{ PhysicsConstraintKind::kGeneric };
		std::string name;
		std::string bodyA;
		std::string bodyB;
		std::string templateName;
		PhysicsFrameMode frameMode{ PhysicsFrameMode::kFrameInB };
		XmlTransform frame;
		float translationLerp{ 0.5F };
		float rotationLerp{ 0.5F };
		bool useLinearReferenceFrameA{ false };
		XmlTransform frameInA;
		XmlTransform frameInB;
		XmlVector3 linearLowerLimit{ 1.0F, 1.0F, 1.0F };
		XmlVector3 linearUpperLimit{ -1.0F, -1.0F, -1.0F };
		XmlVector3 angularLowerLimit{ 1.0F, 1.0F, 1.0F };
		XmlVector3 angularUpperLimit{ -1.0F, -1.0F, -1.0F };
		XmlVector3 linearStiffness;
		XmlVector3 angularStiffness;
		XmlVector3 linearDamping;
		XmlVector3 angularDamping;
		XmlVector3 linearNonHookeanDamping;
		XmlVector3 angularNonHookeanDamping;
		XmlVector3 linearNonHookeanStiffness;
		XmlVector3 angularNonHookeanStiffness;
		XmlVector3 linearEquilibrium;
		XmlVector3 angularEquilibrium;
		XmlVector3 linearBounce;
		XmlVector3 angularBounce;
		XmlVector3 linearTargetVelocity;
		XmlVector3 angularTargetVelocity;
		XmlVector3 linearMaxMotorForce;
		XmlVector3 angularMaxMotorForce;
		bool enableLinearSprings{ true };
		bool enableAngularSprings{ true };
		bool linearStiffnessLimited{ true };
		bool angularStiffnessLimited{ true };
		bool springDampingLimited{ true };
		bool linearMotors{ false };
		bool angularMotors{ false };
		bool linearServoMotors{ false };
		bool angularServoMotors{ false };
		float motorErp{ 0.9F };
		float motorCfm{ 0.0F };
		float stopErp{ 0.2F };
		float stopCfm{ 0.0F };
		float swingSpan1{ 0.0F };
		float swingSpan2{ 0.0F };
		float twistSpan{ 0.0F };
		float limitSoftness{ 1.0F };
		float biasFactor{ 0.3F };
		float relaxationFactor{ 1.0F };
		float minDistanceFactor{ 1.0F };
		float maxDistanceFactor{ 1.0F };
		float stiffness{ 0.0F };
		float damping{ 0.0F };
		float equilibriumFactor{ 0.5F };
	};

	enum class PhysicsShapeKind
	{
		kSphere,
		kBox,
		kCapsule,
		kCylinder,
		kHull,
		kCompound
	};

	struct PhysicsShapeDescriptor
	{
		bool valid{ true };
		PhysicsShapeKind kind{ PhysicsShapeKind::kSphere };
		float radius{ 0.0F };
		float height{ 0.0F };
		float margin{ 0.0F };
		XmlVector3 halfExtents{ 0.0F, 0.0F, 0.0F };
		std::vector<XmlVector3> hullPoints;
		std::vector<std::pair<XmlTransform, PhysicsShapeDescriptor>> compoundChildren;
	};

	struct PhysicsBoneDescriptor
	{
		std::string name;
		std::string templateName;
		float mass{ 0.0F };
		float linearDamping{ 0.0F };
		float angularDamping{ 0.0F };
		float friction{ 0.5F };
		float rollingFriction{ 0.0F };
		float restitution{ 0.0F };
		float gravityFactor{ 1.0F };
		float windFactor{ 1.0F };
		float marginMultiplier{ 1.0F };
		int collisionFilter{ 0 };
		XmlVector3 localInertia;
		bool hasLocalInertia{ false };
		XmlTransform centerOfMassTransform;
		PhysicsShapeDescriptor shape;
		bool hasShape{ false };
		std::vector<std::string> canCollideWithBones;
		std::vector<std::string> noCollideWithBones;
	};

	enum class PhysicsMeshShapeKind
	{
		kPerVertex,
		kPerTriangle
	};

	enum class PhysicsMeshSharedScope
	{
		kPublic,
		kInternal,
		kExternal,
		kPrivate
	};

	struct PhysicsMeshShapeDescriptor
	{
		PhysicsMeshShapeKind kind{ PhysicsMeshShapeKind::kPerVertex };
		PhysicsMeshSharedScope shared{ PhysicsMeshSharedScope::kPublic };
		std::string name;
		std::string disableTag;
		float margin{ 0.0F };
		float penetration{ 1.0F };
		bool hasMargin{ false };
		bool hasPenetration{ false };
		int disablePriority{ 0 };
		std::vector<std::string> tags;
		std::vector<std::string> canCollideWithTags;
		std::vector<std::string> noCollideWithTags;
		std::vector<std::string> canCollideWithBones;
		std::vector<std::string> noCollideWithBones;
		std::vector<std::pair<std::string, float>> weightThresholds;
	};

	struct PhysicsXmlSummary
	{
		std::filesystem::path path;
		std::uint32_t bones{ 0 };
		std::uint32_t boneDefaults{ 0 };
		std::uint32_t perVertexShapes{ 0 };
		std::uint32_t perTriangleShapes{ 0 };
		std::uint32_t shapes{ 0 };
		std::uint32_t constraintGroups{ 0 };
		std::uint32_t constraints{ 0 };
		std::vector<std::string> boneNames;
		std::optional<PhysicsBoneDescriptor> defaultBoneDescriptor;
		std::vector<PhysicsBoneDescriptor> boneDescriptors;
		std::vector<PhysicsMeshShapeDescriptor> meshDescriptors;
		std::vector<PhysicsConstraintDescriptor> constraintDescriptors;
		bool validSystemRoot{ false };
	};

	class PhysicsXmlLoader
	{
	public:
		static PhysicsXmlLoader* GetSingleton();

		void ClearCache();
		bool LoadPrototype(const std::string& a_path);
		std::optional<PhysicsXmlSummary> LoadSummary(const std::string& a_path);
		const PhysicsXmlSummary& GetPrototypeSummary() const { return prototype_; }
		bool HasPrototype() const { return hasPrototype_; }

	private:
		struct CachedSummary
		{
			std::filesystem::file_time_type timestamp{};
			PhysicsXmlSummary summary;
		};

		std::unordered_map<std::string, CachedSummary> summaryCache_;
		PhysicsXmlSummary prototype_;
		bool hasPrototype_{ false };
	};
}
