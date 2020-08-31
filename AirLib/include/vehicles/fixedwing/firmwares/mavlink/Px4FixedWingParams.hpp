#ifndef msr_airlib_vehicles_Px4FixedWing_hpp
#define msr_airlib_vehicles_Px4FixedWing_hpp

#include "vehicles/fixedwing/firmwares/mavlink/MavLinkFixedWingApi.hpp"
#include "common/AirSimSettings.hpp"
#include "vehicles/fixedwing/FixedWingParams.hpp"

namespace msr
{
	namespace airlib
	{
		class Px4FixedWingParams : public FixedWingParams
		{
		public:
			Px4FixedWingParams(const AirSimSettings::MavLinkVehicleSetting& vehicle_setting, std::shared_ptr<const SensorFactory> sensor_factory)
				: sensor_factory_(sensor_factory)
			{
				connection_info_ = getConnectionInfo(vehicle_setting);
			}

			virtual ~Px4FixedWingParams() = default;

			virtual std::unique_ptr<FixedWingApiBase> createFixedWingApi() override
			{
				unique_ptr<FixedWingApiBase> api(new MavLinkFixedWingApi());
				auto api_ptr = static_cast<MavLinkFixedWingApi*>(api.get());
				api_ptr->initialize(connection_info_, &getSensors(), true);

				return api;
			}

			virtual void setupParams() override
			{
				auto& params = getParams();

				if (connection_info_.model == "Cherokee")
				{
					setupFrameCherokee(params);
				}
				else if (connection_info_.model == "SkywalkerX8")
				{
					setupFrameSkywalkerX8(params);
				}
				else if (connection_info_.model == "DA62")
				{
					setupFrameDA62(params);
				}
				else if (connection_info_.model == "Glider")
				{
					setupFrameGlider(params);
				}
				else
					setupFrameGenericAircraft(params);
			};

		protected:
			virtual const SensorFactory* getSensorFactory() const override
			{
				return sensor_factory_.get();
			}

		private:
			static const AirSimSettings::MavLinkConnectionInfo& getConnectionInfo(const AirSimSettings::MavLinkVehicleSetting& vehicle_setting)
			{
				return vehicle_setting.connection_info;
			}

		private:
			AirSimSettings::MavLinkConnectionInfo connection_info_;
			std::shared_ptr<const SensorFactory> sensor_factory_;
		};
	}
}


#endif
