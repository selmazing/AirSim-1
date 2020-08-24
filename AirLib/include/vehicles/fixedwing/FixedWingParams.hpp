#ifndef msr_airlib_FixedWingParameters_hpp
#define msr_airlib_FixedWingParameters_hpp

#include "common/Common.hpp"
#include "AircraftParams.hpp"
#include "api/FixedWingApiBase.hpp"
#include "sensors/SensorCollection.hpp"
#include "sensors/SensorFactory.hpp"
#include "vehicles/multirotor/firmwares/simple_flight/firmware/Params.hpp"

namespace msr
{
	namespace airlib
	{
		class FixedWingParams
			// SI units used throughout
		{
		public:
			struct Params
			{
				/*required parameters*/
				real_T mass; // operational aircraft mass [kg]
				Matrix3x3r inertia; // operational inertia matrix
				LinearAeroDerivatives derivatives; // initialize aerodynamic derivatives
				Dimensions dimensions; // aircraft reference dimensions

				/*parameters set with defaults*/
				real_T restitution = 0.55f; // needed for FixedWingPawnSimApi.cpp API creation
				real_T friction = 0.5f; // needed for FixedWingPawnSimApi.cpp API creation
				unsigned int control_count = 3; // number of control variables (elevator, aileron, rudder)
			};

		protected:
			virtual void setupParams() = 0;
			virtual const SensorFactory* getSensorFactory() const = 0;

		public: //interface
			virtual std::unique_ptr<FixedWingApiBase> createFixedWingApi() = 0;

			virtual ~FixedWingParams() = default;
			virtual void initialize(const AirSimSettings::VehicleSetting* vehicle_setting)
			{
				sensor_storage_.clear();
				sensors_.clear();

				setupParams();

				addSensorsFromSettings(vehicle_setting);
			}

			const Params& getParams() const
			{
				return params_;
			}
			Params& getParams()
			{
				return params_;
			}
			SensorCollection& getSensors()
			{
				return sensors_;
			}
			const SensorCollection& getSensors() const
			{
				return sensors_;
			}

			/* simple_flight::Params& getParams() // Currently points to Multirotor simple flight, don't know if this is needed
			{
				return params_;
			} */
			

			void addSensorsFromSettings(const AirSimSettings::VehicleSetting* vehicle_setting)
			{
				// use sensors from vehicle settings; if empty list, use default sensors.
				// note that the vehicle settings completely override the default sensor "list";
				// there is no piecemeal add/remove/update per sensor.
				const std::map<std::string, std::unique_ptr<AirSimSettings::SensorSetting>>& sensor_settings
					= vehicle_setting->sensors.size() > 0 ? vehicle_setting->sensors : AirSimSettings::AirSimSettings::singleton().sensor_defaults;

				getSensorFactory()->createSensorsFromSettings(sensor_settings, sensors_, sensor_storage_);
			}

		protected:
			void setupFrameGenericAircraft(Params& params)
			{
				// Basic Parameters //
				params.mass = 10.0f; //mass in [kg]
				params.inertia(0, 0) = 100.0f; // Ixx [Kgm^-2]
				params.inertia(1, 1) = 10.0f; // Iyy [Kgm^-2]
				params.inertia(2, 2) = 5.0f; //Izz [Kgm^-2]
				params.inertia(0, 2) = params.inertia(2,0) = 2.0f; // Ixz [Kgm^-2]
				params.inertia(0, 1) = params.inertia(1, 0) = 0; // Ixy [Kgm^-2], symmetric
				params.inertia(2, 1) = params.inertia(1, 2) = 0; // Iyz [Kgm^-2], symmetric

				// Reference Areas //
				params.dimensions.main_plane_area = 10; // S [m^2]
				params.dimensions.horizontal_tail_plane_area = 2; // St [m^2]
				params.dimensions.vertical_tail_plane_area = 3.5; // Svt [m^2]

				// Body Terms //
				params.derivatives.zero_lift_coefficient = 0.5f;
				params.derivatives.alpha_lift_coefficient = 0.05f;
				params.derivatives.pitch_lift_coefficient = 0.0f;
				
				params.derivatives.zero_drag_coefficient = 0.01f;
				params.derivatives.alpha_drag_coefficient = 0.005f;
				params.derivatives.pitch_drag_coefficient = 0.0f;

				params.derivatives.sidevelocity_sideforce_coefficient = 0.01f;

				params.derivatives.zero_roll_coefficient = 0.0f;
				params.derivatives.rollrate_roll_coefficient = 0.01f;

				params.derivatives.zero_pitch_coefficient = 0.01f;
				params.derivatives.alpha_pitch_coefficient = 0.05f;
				params.derivatives.pitchrate_pitch_coefficient = 0.01f;

				params.derivatives.zero_yaw_coefficient = 0.0f;
				params.derivatives.yawrate_yaw_coefficient = 0.01f;

				// Control Terms //
				params.derivatives.elev_lift_coefficient = -0.005f;
				params.derivatives.elev_drag_coefficient = 0.01f;
				params.derivatives.elevator_pitch_coefficient = 0.05f;

				params.derivatives.aileron_roll_coefficient = 0.1f;
				
				params.derivatives.rudder_yaw_coefficient = 0.1f;
				params.derivatives.rudder_sideforce_coefficient = 0.01f;
				
			}

			void setupFrameCherokee(Params& params)
			{
				params.mass = 10.0f; //mass in [kg]
				params.inertia(0, 0) = 100.0f; // Ixx [Kgm^-2]
				params.inertia(1, 1) = 10.0f; // Iyy [Kgm^-2]
				params.inertia(2, 2) = 5.0f; //Izz [Kgm^-2]
				params.inertia(0, 2) = params.inertia(2, 0) = 2.0f; // Ixz [Kgm^-2]
				params.inertia(0, 1) = params.inertia(1, 0) = 0; // Ixy [Kgm^-2], symmetric
				params.inertia(2, 1) = params.inertia(1, 2) = 0; // Iyz [Kgm^-2], symmetric

				// Reference Areas //
				params.dimensions.main_plane_area = 10; // S [m^2]
				params.dimensions.horizontal_tail_plane_area = 2; // St [m^2]
				params.dimensions.vertical_tail_plane_area = 3.5; // Svt [m^2]

				// Body Terms //
				params.derivatives.zero_lift_coefficient = 0.5f;
				params.derivatives.alpha_lift_coefficient = 0.05f;
				params.derivatives.pitch_lift_coefficient = 0.0f;

				params.derivatives.zero_drag_coefficient = 0.01f;
				params.derivatives.alpha_drag_coefficient = 0.005f;
				params.derivatives.pitch_drag_coefficient = 0.0f;

				params.derivatives.sidevelocity_sideforce_coefficient = 0.01f;

				params.derivatives.zero_roll_coefficient = 0.0f;
				params.derivatives.rollrate_roll_coefficient = 0.01f;

				params.derivatives.zero_pitch_coefficient = 0.01f;
				params.derivatives.alpha_pitch_coefficient = 0.05f;
				params.derivatives.pitchrate_pitch_coefficient = 0.01f;

				params.derivatives.zero_yaw_coefficient = 0.0f;
				params.derivatives.yawrate_yaw_coefficient = 0.01f;

				// Control Terms //
				params.derivatives.elev_lift_coefficient = -0.005f;
				params.derivatives.elev_drag_coefficient = 0.01f;
				params.derivatives.elevator_pitch_coefficient = 0.05f;

				params.derivatives.aileron_roll_coefficient = 0.1f;

				params.derivatives.rudder_yaw_coefficient = 0.1f;
				params.derivatives.rudder_sideforce_coefficient = 0.01f;
			}

			void setupFrameDA62(Params& params)
			{
				params.mass = 10.0f; //mass in [kg]
				params.inertia(0, 0) = 100.0f; // Ixx [Kgm^-2]
				params.inertia(1, 1) = 10.0f; // Iyy [Kgm^-2]
				params.inertia(2, 2) = 5.0f; //Izz [Kgm^-2]
				params.inertia(0, 2) = params.inertia(2, 0) = 2.0f; // Ixz [Kgm^-2]
				params.inertia(0, 1) = params.inertia(1, 0) = 0; // Ixy [Kgm^-2], symmetric
				params.inertia(2, 1) = params.inertia(1, 2) = 0; // Iyz [Kgm^-2], symmetric

				// Reference Areas //
				params.dimensions.main_plane_area = 10; // S [m^2]
				params.dimensions.horizontal_tail_plane_area = 2; // St [m^2]
				params.dimensions.vertical_tail_plane_area = 3.5; // Svt [m^2]

				// Body Terms //
				params.derivatives.zero_lift_coefficient = 0.5f;
				params.derivatives.alpha_lift_coefficient = 0.05f;
				params.derivatives.pitch_lift_coefficient = 0.0f;

				params.derivatives.zero_drag_coefficient = 0.01f;
				params.derivatives.alpha_drag_coefficient = 0.005f;
				params.derivatives.pitch_drag_coefficient = 0.0f;

				params.derivatives.sidevelocity_sideforce_coefficient = 0.01f;

				params.derivatives.zero_roll_coefficient = 0.0f;
				params.derivatives.rollrate_roll_coefficient = 0.01f;

				params.derivatives.zero_pitch_coefficient = 0.01f;
				params.derivatives.alpha_pitch_coefficient = 0.05f;
				params.derivatives.pitchrate_pitch_coefficient = 0.01f;

				params.derivatives.zero_yaw_coefficient = 0.0f;
				params.derivatives.yawrate_yaw_coefficient = 0.01f;

				// Control Terms //
				params.derivatives.elev_lift_coefficient = -0.005f;
				params.derivatives.elev_drag_coefficient = 0.01f;
				params.derivatives.elevator_pitch_coefficient = 0.05f;

				params.derivatives.aileron_roll_coefficient = 0.1f;

				params.derivatives.rudder_yaw_coefficient = 0.1f;
				params.derivatives.rudder_sideforce_coefficient = 0.01f;
			}

			void setupFrameGlider(Params& params)
			{
				params.mass = 10.0f; //mass in [kg]
				params.inertia(0, 0) = 100.0f; // Ixx [Kgm^-2]
				params.inertia(1, 1) = 10.0f; // Iyy [Kgm^-2]
				params.inertia(2, 2) = 5.0f; //Izz [Kgm^-2]
				params.inertia(0, 2) = params.inertia(2, 0) = 2.0f; // Ixz [Kgm^-2]
				params.inertia(0, 1) = params.inertia(1, 0) = 0; // Ixy [Kgm^-2], symmetric
				params.inertia(2, 1) = params.inertia(1, 2) = 0; // Iyz [Kgm^-2], symmetric

				// Reference Areas //
				params.dimensions.main_plane_area = 10; // S [m^2]
				params.dimensions.horizontal_tail_plane_area = 2; // St [m^2]
				params.dimensions.vertical_tail_plane_area = 3.5; // Svt [m^2]

				// Body Terms //
				params.derivatives.zero_lift_coefficient = 0.5f;
				params.derivatives.alpha_lift_coefficient = 0.05f;
				params.derivatives.pitch_lift_coefficient = 0.0f;

				params.derivatives.zero_drag_coefficient = 0.01f;
				params.derivatives.alpha_drag_coefficient = 0.005f;
				params.derivatives.pitch_drag_coefficient = 0.0f;

				params.derivatives.sidevelocity_sideforce_coefficient = 0.01f;

				params.derivatives.zero_roll_coefficient = 0.0f;
				params.derivatives.rollrate_roll_coefficient = 0.01f;

				params.derivatives.zero_pitch_coefficient = 0.01f;
				params.derivatives.alpha_pitch_coefficient = 0.05f;
				params.derivatives.pitchrate_pitch_coefficient = 0.01f;

				params.derivatives.zero_yaw_coefficient = 0.0f;
				params.derivatives.yawrate_yaw_coefficient = 0.01f;

				// Control Terms //
				params.derivatives.elev_lift_coefficient = -0.005f;
				params.derivatives.elev_drag_coefficient = 0.01f;
				params.derivatives.elevator_pitch_coefficient = 0.05f;

				params.derivatives.aileron_roll_coefficient = 0.1f;

				params.derivatives.rudder_yaw_coefficient = 0.1f;
				params.derivatives.rudder_sideforce_coefficient = 0.01f;
			}
		
		private:
			Params params_;
			SensorCollection sensors_; //maintains sensor type indexed collection of sensors
			vector<unique_ptr<SensorBase>> sensor_storage_; // RAII for created sensors
		
		};
	}
}

#endif