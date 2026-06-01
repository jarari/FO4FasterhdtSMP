#pragma once

#include <string>
#include <vector>

namespace Smp
{
	struct SolverSettings
	{
		int numIterations{ 10 };
		float erp{ 0.2F };
		int minFps{ 60 };
		int maxSubSteps{ 4 };
	};

	struct SmpSettings
	{
		int logLevel{ 3 };
		bool clampRotations{ true };
		float rotationSpeedLimit{ 10.0F };
		bool unclampedResets{ true };
		float unclampedResetAngle{ 130.0F };
		float budgetMs{ 3.5F };
		bool useRealTime{ false };
		int sampleSize{ 5 };
		bool disableFirstPersonViewPhysics{ false };
		bool enableNpcPhysics{ true };
		bool autoAdjustMaxActors{ false };
		int maxActiveActors{ 4 };
		float maxActorDistance{ 3000.0F };
		bool enablePrototypeDiagnostics{ false };
		bool enableBulletVisualization{ false };
		bool disableSMPHairWhenWigEquipped{ true };
		std::string prototypePhysicsXml;
		std::vector<std::string> backupNodeByName;
	};

	struct WindSettings
	{
		bool enabled{ false };
		bool useWeather{ true };
		float windStrength{ 2.0F };
		float directionX{ 1.0F };
		float directionY{ 0.0F };
		float directionZ{ 0.0F };
		float distanceForNoWind{ 50.0F };
		float distanceForMaxWind{ 3000.0F };
		float weatherShortCooldownSeconds{ 0.5F };
		float weatherLongCooldownSeconds{ 5.0F };
		int smoothingSamples{ 8 };
		bool randomizePerBoneWind{ true };
	};

	struct RuntimeSettings
	{
		SolverSettings solver;
		SmpSettings smp;
		WindSettings wind;
	};

	class Config
	{
	public:
		static Config* GetSingleton();

		bool Load();
		void Log() const;
		const RuntimeSettings& GetSettings() const { return settings_; }

	private:
		RuntimeSettings settings_;
	};
}
