#pragma once

#include <RE/B/BSTEvent.h>

template <typename T>
class btAlignedObjectArray;
class btCollisionObject;

namespace hdt
{
	// Plugins should register for messages from FO4FasterHdtSMP through F4SE
	// during F4SE's PostLoad event. Once startup is complete, this plugin
	// broadcasts MSG_STARTUP with a PluginInterface* in Message::data.
	struct PreStepEvent
	{
		// Sent immediately before Bullet advances. Listeners may apply forces and
		// torques; collision objects must otherwise be treated as read-only.
		const btAlignedObjectArray<btCollisionObject*>& objects;
		float timeStep{ 0.0F };
	};

	struct PostStepEvent
	{
		// Sent immediately after Bullet advances. Collision objects are read-only.
		const btAlignedObjectArray<btCollisionObject*>& objects;
		float timeStep{ 0.0F };
	};

	using IPreStepListener = RE::BSTEventSink<PreStepEvent>;
	using IPostStepListener = RE::BSTEventSink<PostStepEvent>;

	class PluginInterface
	{
	public:
		enum MessageType : unsigned long
		{
			MSG_STARTUP,
		};

		struct Version
		{
			int major;
			int minor;
			int patch;
		};

		struct VersionInfo
		{
			Version interfaceVersion;
			Version bulletVersion;
		};

		static constexpr Version INTERFACE_VERSION{ 2, 0, 0 };
		static constexpr Version BULLET_VERSION{ 3, 25, 0 };

		virtual ~PluginInterface() = default;

		[[nodiscard]] virtual const VersionInfo& getVersionInfo() const = 0;

		virtual void addListener(IPreStepListener*) = 0;
		virtual void removeListener(IPreStepListener*) = 0;

		virtual void addListener(IPostStepListener*) = 0;
		virtual void removeListener(IPostStepListener*) = 0;
	};
}
