#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace RE
{
	class Actor;
}

namespace Smp::Papyrus
{
	bool Register();
	bool RegisterSerialization();
	void QueueResetPhysics(RE::Actor* a_actor, bool a_full);

	bool RegisterPhysicsFileOverride(
		std::uint32_t a_actorFormID,
		std::string a_oldPhysicsFilePath,
		std::string a_newPhysicsFilePath);
	[[nodiscard]] std::optional<std::string> ResolvePhysicsFileOverride(
		std::uint32_t a_actorFormID,
		std::string_view a_physicsFilePath);
	[[nodiscard]] std::string QueryPhysicsFileOverrides();
}
