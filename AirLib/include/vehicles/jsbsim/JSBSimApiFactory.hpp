#ifndef msr_airlib_vehicles_JSBSimFactory_hpp
#define msr_airlib_vehicles_JSBSimFactory_hpp

#include "api/JSBSimApiBase.hpp"
#include "common/AirSimSettings.hpp"
#include "sensors/SensorFactory.hpp"
#include "vehicles/jsbsim/firmwares/JSBSimPlaneApi.hpp"

namespace msr { namespace airlib {

class JSBSimApiFactory {
public:
	static std::unique_ptr<JSBSimApiBase> createApi(const AirSimSettings::VehicleSetting* vehicle_setting,
													std::shared_ptr<SensorFactory> sensor_factory,
													const Kinematics::State& state, const Environment& environment)
	{
		if (vehicle_setting->vehicle_type == AirSimSettings::kVehicleTypeJSBSimPlane)
		{
			return std::unique_ptr<JSBSimApiBase>(new JSBSimPlaneApi(vehicle_setting, sensor_factory,
				state, environment));
		}
		else
			throw std::runtime_error(Utils::stringf(
				"Cannot create vehicle config because vehicle name '%s' is not recognized",
				vehicle_setting->vehicle_name.c_str()));
	}
};
}
}

#endif
