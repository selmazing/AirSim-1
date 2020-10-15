#ifndef msr_airlib_vehicles_FixedWingParamsFactory_hpp
#define msr_airlib_vehicles_FixedWingParamsFactory_hpp

#include "vehicles/fixedwing/firmwares/mavlink/MavLinkFixedWingApi.hpp"
#include "vehicles/fixedwing/firmwares/ardupilot/ArduPlaneParams.hpp"
#include "vehicles/fixedwing/firmwares/mavlink/Px4FixedWingParams.hpp"
#include "vehicles/fixedwing/firmwares/plane_flight/PlaneFlightXParams.hpp"

namespace msr
{	
	namespace airlib{
		class FixedWingParamsFactory
		{
		public:
			static std::unique_ptr<FixedWingParams> createConfig(const AirSimSettings::VehicleSetting* vehicle_setting,
				std::shared_ptr<const SensorFactory> sensor_factory)
			{
				std::unique_ptr<FixedWingParams> config;

				if (vehicle_setting->vehicle_type == AirSimSettings::kVehicleTypePX4Plane)
				{
					config.reset(new Px4FixedWingParams(*static_cast<const AirSimSettings::MavLinkVehicleSetting*>(vehicle_setting), sensor_factory));
				}
				else if (vehicle_setting->vehicle_type == AirSimSettings::kVehicleTypeArduPlane)
				{
					config.reset(new ArduPlaneParams(*static_cast<const AirSimSettings::MavLinkVehicleSetting*>(vehicle_setting), sensor_factory));
				}
				else if (vehicle_setting->vehicle_type == "" || vehicle_setting->vehicle_type == AirSimSettings::kVehicleTypePlaneFlight)
				{
					config.reset(new PlaneFlightXParams(vehicle_setting, sensor_factory));
				}
				else
					throw std::runtime_error(Utils::stringf(
						"Cannot create vehicle config because vehicle name '%s' is not recognized",
						vehicle_setting->vehicle_name.c_str()));

				config->initialize(vehicle_setting);

				return config;
			}
		};
	}
}
#endif
