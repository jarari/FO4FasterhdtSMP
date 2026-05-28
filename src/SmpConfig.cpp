#include "SmpConfig.h"

#include "ConfigPaths.h"

#include <tinyxml2.h>

#include <algorithm>
#include <cerrno>
#include <charconv>
#include <cstdlib>
#include <sstream>

namespace
{
	using Smp::ConfigPaths::LowerString;
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

	std::string ReadText(tinyxml2::XMLElement* a_parent, const char* a_name)
	{
		if (const auto element = a_parent ? a_parent->FirstChildElement(a_name) : nullptr) {
			if (const auto text = element->GetText()) {
				return Trim(text);
			}
		}
		return {};
	}

	std::vector<std::string> SplitCommaList(const std::string& a_value)
	{
		std::vector<std::string> result;
		std::stringstream stream(a_value);
		std::string item;
		while (std::getline(stream, item, ',')) {
			item = Trim(std::move(item));

			if (!item.empty()) {
				result.push_back(item);
			}
		}
		return result;
	}
}

namespace Smp
{
	Config* Config::GetSingleton()
	{
		static Config singleton;
		return std::addressof(singleton);
	}

	bool Config::Load()
	{
		settings_ = RuntimeSettings{};

		const auto configPath = Smp::ConfigPaths::ResolveExistingConfigPath("configs.xml", true);
		if (!configPath) {
			spdlog::info("config not found at Data/F4SE/Plugins/FO4FasterHdtSMP/configs.xml or Data/SKSE/Plugins/hdtSkinnedMeshConfigs/configs.xml; using defaults");
			return false;
		}

		tinyxml2::XMLDocument document;
		const auto error = document.LoadFile(configPath->string().c_str());
		if (error != tinyxml2::XML_SUCCESS) {
			spdlog::error("failed to parse {}: {}", configPath->string(), document.ErrorStr());
			return false;
		}

		const auto configs = document.FirstChildElement("configs");
		if (!configs) {
			spdlog::warn("{} does not contain a <configs> root; using defaults", configPath->string());
			return false;
		}

		if (const auto solver = configs->FirstChildElement("solver")) {
			settings_.solver.numIterations = std::clamp(ReadInt(solver, "numIterations", settings_.solver.numIterations), 4, 128);
			settings_.solver.erp = std::clamp(ReadFloat(solver, "erp", settings_.solver.erp), 0.01F, 1.0F);
			settings_.solver.minFps = std::clamp(ReadInt(solver, "min-fps", settings_.solver.minFps), 1, 300);
			settings_.solver.maxSubSteps = std::clamp(ReadInt(solver, "maxSubSteps", settings_.solver.maxSubSteps), 1, 60);
		}

		if (const auto smp = configs->FirstChildElement("smp")) {
			settings_.smp.logLevel = std::clamp(ReadInt(smp, "logLevel", settings_.smp.logLevel), 0, 6);
			settings_.smp.clampRotations = ReadBool(smp, "clampRotations", settings_.smp.clampRotations);
			settings_.smp.rotationSpeedLimit = ReadFloat(smp, "rotationSpeedLimit", settings_.smp.rotationSpeedLimit);
			settings_.smp.unclampedResets = ReadBool(smp, "unclampedResets", settings_.smp.unclampedResets);
			settings_.smp.unclampedResetAngle = ReadFloat(smp, "unclampedResetAngle", settings_.smp.unclampedResetAngle);
			settings_.smp.budgetMs = std::clamp(ReadFloat(smp, "budgetMs", settings_.smp.budgetMs), 0.1F, 30.0F);
			settings_.smp.useRealTime = ReadBool(smp, "useRealTime", settings_.smp.useRealTime);
			settings_.smp.sampleSize = std::max(ReadInt(smp, "sampleSize", settings_.smp.sampleSize), 1);
			settings_.smp.disableFirstPersonViewPhysics = ReadBool(smp, "disable1stPersonViewPhysics", settings_.smp.disableFirstPersonViewPhysics);
			settings_.smp.enableNpcPhysics = ReadBool(smp, "enableNpcPhysics", settings_.smp.enableNpcPhysics);
			settings_.smp.autoAdjustMaxActors = ReadBool(smp, "autoAdjustMaxSkeletons", settings_.smp.autoAdjustMaxActors);
			settings_.smp.autoAdjustMaxActors = ReadBool(smp, "autoAdjustMaxActors", settings_.smp.autoAdjustMaxActors);
			const auto maxActiveActors = ReadInt(smp, "maximumActiveSkeletons", settings_.smp.maxActiveActors);
			settings_.smp.maxActiveActors = std::max(ReadInt(smp, "maxActiveActors", maxActiveActors), 1);
			settings_.smp.maxActorDistance = std::clamp(ReadFloat(smp, "maxActorDistance", settings_.smp.maxActorDistance), 0.0F, 100000.0F);
			settings_.smp.enablePrototypeDiagnostics = ReadBool(smp, "enablePrototypeDiagnostics", settings_.smp.enablePrototypeDiagnostics);
			settings_.smp.prototypePhysicsXml = ReadText(smp, "prototypePhysicsXml");
			settings_.smp.backupNodeByName = SplitCommaList(ReadText(smp, "backupNodeByName"));
		}

		if (const auto wind = configs->FirstChildElement("wind")) {
			settings_.wind.enabled = ReadBool(wind, "enabled", settings_.wind.enabled);
			settings_.wind.useWeather = ReadBool(wind, "useWeather", settings_.wind.useWeather);
			settings_.wind.windStrength = std::clamp(ReadFloat(wind, "windStrength", settings_.wind.windStrength), 0.0F, 1000.0F);
			settings_.wind.directionX = ReadFloat(wind, "directionX", settings_.wind.directionX);
			settings_.wind.directionY = ReadFloat(wind, "directionY", settings_.wind.directionY);
			settings_.wind.directionZ = ReadFloat(wind, "directionZ", settings_.wind.directionZ);
			settings_.wind.distanceForNoWind = std::clamp(ReadFloat(wind, "distanceForNoWind", settings_.wind.distanceForNoWind), 0.0F, 10000.0F);
			settings_.wind.distanceForMaxWind = std::clamp(ReadFloat(wind, "distanceForMaxWind", settings_.wind.distanceForMaxWind), 0.0F, 10000.0F);
		}

		const auto level = settings_.smp.logLevel >= 6 ? spdlog::level::off : static_cast<spdlog::level::level_enum>(5 - settings_.smp.logLevel);
		spdlog::set_level(level);
		spdlog::flush_on(level);
		return true;
	}

	void Config::Log() const
	{
		spdlog::debug("config: solver.numIterations = {}", settings_.solver.numIterations);
		spdlog::debug("config: solver.erp = {}", settings_.solver.erp);
		spdlog::debug("config: solver.min-fps = {}", settings_.solver.minFps);
		spdlog::debug("config: solver.maxSubSteps = {}", settings_.solver.maxSubSteps);
		spdlog::debug("config: smp.logLevel = {}", settings_.smp.logLevel);
		spdlog::debug("config: smp.budgetMs = {}", settings_.smp.budgetMs);
		spdlog::debug("config: smp.useRealTime = {}", settings_.smp.useRealTime);
		spdlog::debug("config: smp.sampleSize = {}", settings_.smp.sampleSize);
		spdlog::debug("config: smp.disable1stPersonViewPhysics = {}", settings_.smp.disableFirstPersonViewPhysics);
		spdlog::debug("config: smp.enableNpcPhysics = {}", settings_.smp.enableNpcPhysics);
		spdlog::debug("config: smp.autoAdjustMaxActors = {}", settings_.smp.autoAdjustMaxActors);
		spdlog::debug("config: smp.maxActiveActors = {}", settings_.smp.maxActiveActors);
		spdlog::debug("config: smp.maxActorDistance = {}", settings_.smp.maxActorDistance);
		spdlog::debug("config: smp.enablePrototypeDiagnostics = {}", settings_.smp.enablePrototypeDiagnostics);
		spdlog::debug("config: smp.prototypePhysicsXml = {}", settings_.smp.prototypePhysicsXml);
		for (const auto& item : settings_.smp.backupNodeByName) {
			spdlog::debug("config: smp.backupNodeByName += {}", item);
		}
		spdlog::debug("config: wind.enabled = {}", settings_.wind.enabled);
		spdlog::debug("config: wind.useWeather = {}", settings_.wind.useWeather);
		spdlog::debug("config: wind.windStrength = {}", settings_.wind.windStrength);
		spdlog::debug("config: wind.direction = ({}, {}, {})", settings_.wind.directionX, settings_.wind.directionY, settings_.wind.directionZ);
		spdlog::debug("config: wind.distanceForNoWind = {}", settings_.wind.distanceForNoWind);
		spdlog::debug("config: wind.distanceForMaxWind = {}", settings_.wind.distanceForMaxWind);
	}
}
