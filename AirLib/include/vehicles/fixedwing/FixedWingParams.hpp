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
			struct AirplanePose
			{
				Vector3r position;
				Vector3r normal;

				AirplanePose()
				{}
				AirplanePose(const Vector3r& position_val, const Vector3r& normal_val)
					: position(position_val), normal(normal_val)
				{}
			};
			
			struct Params
			{
				/*required parameters*/
				real_T mass; // operational aircraft mass [kg]
				Matrix3x3r inertia; // operational inertia matrix
				LinearAeroDerivatives derivatives; // initialize aerodynamic derivatives
				PropulsionDerivatives prop_derivatives; // propulsion derivatives
				Dimensions dimensions; // aircraft reference dimensions
				AirplanePose airplane_pose; // full aircraft pose

				/*parameters set with defaults*/
				real_T restitution = 0.1f; // needed for FixedWingPawnSimApi.cpp API creation
				real_T friction = 0.05f; // needed for FixedWingPawnSimApi.cpp API creation
				uint control_count = 4; // number of control variables (elevator, aileron, rudder, TLA)
				uint airplane_count = 1; // number of aircraft (Should not normally be more than 1)
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
			// Based on Arduplane SITL aircraft SIM_Plane.h//
			void setupFrameGenericAircraft(Params& params)
			{
				// Basic Parameters //
				params.mass = 2.0f; //mass in [kg]
				params.inertia(0, 0) = 0.045f; // Ixx [Kgm^-2]
				params.inertia(1, 1) = 0.0005f; // Iyy [Kgm^-2]
				params.inertia(2, 2) = 0.005f; //Izz [Kgm^-2]
				params.inertia(0, 2) = params.inertia(2, 0) = 0.00001f; // Ixz [Kgm^-2]
				params.inertia(0, 1) = params.inertia(1, 0) = 0.00001f; // Ixy [Kgm^-2], symmetric
				params.inertia(2, 1) = params.inertia(1, 2) = 0.00001f; // Iyz [Kgm^-2], symmetric

				// Reference Areas //
				params.dimensions.main_plane_area = 0.45f; // S [m^2]
				params.dimensions.main_plane_span = 1.88f; // b [m]
				params.dimensions.main_plane_chord = 0.24f; // c [m], defined @ 1/4 span as Mean Aero Chord (MAC), use boeing/american for area
				params.dimensions.horizontal_tail_plane_area = 0.0f; // St [m^2]
				params.dimensions.vertical_tail_plane_area = 0.0f; // Svt [m^2]

				// Body Terms //
				params.derivatives.zero_lift_coefficient = 1.244f;
				params.derivatives.alpha_lift_coefficient = 15.33f;
				params.derivatives.pitch_lift_coefficient = 0.0f; // no value for this !!!
				
				params.derivatives.zero_drag_coefficient = 0.253f; // seems too large
				params.derivatives.alpha_drag_coefficient = 0.773f; // again 10 times too large
				params.derivatives.alpha_drag_coefficient_2 = 4.769f; // might be ok?
				params.derivatives.pitch_drag_coefficient = 0.0f;

				params.derivatives.zero_sideforce_coefficient = 0.0f;
				params.derivatives.beta_sideforce_coefficient = -2.178f;
				params.derivatives.rollrate_sideforce_coefficient = 0.0f;
				params.derivatives.yawrate_sideforce_coefficient = 0.0f;
				params.derivatives.sidevelocity_sideforce_coefficient = 0.0f;
				
				params.derivatives.zero_roll_coefficient = 0.0f;
				params.derivatives.beta_roll_coefficient = -0.267f;
				params.derivatives.rollrate_roll_coefficient = -2.222f;
				params.derivatives.yawrate_roll_coefficient = 0.311f;

				params.derivatives.zero_pitch_coefficient = 0.1f;
				params.derivatives.alpha_pitch_coefficient = 1.556f;
				params.derivatives.pitchrate_pitch_coefficient = -44.4f;

				params.derivatives.zero_yaw_coefficient = 0.0f;
				params.derivatives.beta_yaw_coefficient = 0.556f;
				params.derivatives.rollrate_yaw_coefficient = 0.0489;
				params.derivatives.yawrate_yaw_coefficient = -2.222f;

				// Control Terms //
				params.derivatives.elev_lift_coefficient = 0.0f;
				params.derivatives.elev_drag_coefficient = 0.00f;
				params.derivatives.elevator_pitch_coefficient = 2.222f;

				params.derivatives.aileron_sideforce_coefficient = 0.0f;
				params.derivatives.aileron_roll_coefficient = 0.555f;
				params.derivatives.aileron_yaw_coefficient = 0.0f;
				
				params.derivatives.rudder_yaw_coefficient = 0.222f;
				params.derivatives.rudder_sideforce_coefficient = 0.445f;
				params.derivatives.rudder_roll_coefficient = 0.082f;

				// Propulsion Terms //
				params.prop_derivatives.thrust_tla_coefficient = 1.0f; // set arbitrarily
				
			}

			void setupFrameSkywalkerX8(Params& params)
			{
				params.mass = 3.3640f; //mass in [kg]
				params.inertia(0, 0) = 1.2290f; // Ixx [Kgm^-2]
				params.inertia(1, 1) = 0.1702f; // Iyy [Kgm^-2]
				params.inertia(2, 2) = 0.8808f; //Izz [Kgm^-2]
				params.inertia(0, 2) = params.inertia(2, 0) = -0.9343f; // Ixz [Kgm^-2]
				params.inertia(0, 1) = params.inertia(1, 0) = 0; // Ixy [Kgm^-2], symmetric
				params.inertia(2, 1) = params.inertia(1, 2) = 0; // Iyz [Kgm^-2], symmetric

				// Reference Areas //
				params.dimensions.main_plane_area = 0.7500f; // S [m^2]
				params.dimensions.main_plane_span = 2.1000f; // b [m]
				params.dimensions.main_plane_chord = 0.3571f; // c [m], defined @ 1/4 span as Mean Aero Chord (MAC), use boeing/american for area
				params.dimensions.horizontal_tail_plane_area = 0.0f; // St [m^2]
				params.dimensions.vertical_tail_plane_area = 0.0f; // Svt [m^2]

				// Body Terms //
				params.derivatives.zero_lift_coefficient = 0.0867f;
				params.derivatives.alpha_lift_coefficient = 4.0203f;
				params.derivatives.pitch_lift_coefficient = 3.8700f;

				params.derivatives.zero_drag_coefficient = 0.0197f;
				params.derivatives.alpha_drag_coefficient = 0.0791f;
				params.derivatives.alpha_drag_coefficient_2 = 1.0555f;
				params.derivatives.beta_drag_coefficient = -0.0058;
				params.derivatives.beta_drag_coefficient_2 = 0.1478;
				params.derivatives.pitch_drag_coefficient = 0.0f;
				
				params.derivatives.zero_sideforce_coefficient = 0.0f;
				params.derivatives.beta_sideforce_coefficient = -0.2239f;
				params.derivatives.rollrate_sideforce_coefficient = -0.1374f;
				params.derivatives.yawrate_sideforce_coefficient = 0.0839f;
				params.derivatives.sidevelocity_sideforce_coefficient = 0.0f;

				params.derivatives.zero_roll_coefficient = 0.0f;
				params.derivatives.beta_roll_coefficient = -0.0849f;
				params.derivatives.rollrate_roll_coefficient = -0.4042f;
				params.derivatives.yawrate_roll_coefficient = 0.0555f;

				params.derivatives.zero_pitch_coefficient = 0.0227f;
				params.derivatives.alpha_pitch_coefficient = -0.4629f;
				params.derivatives.pitchrate_pitch_coefficient = -1.3012f;

				params.derivatives.zero_yaw_coefficient = 0.0f;
				params.derivatives.beta_yaw_coefficient = 0.0283;
				params.derivatives.rollrate_yaw_coefficient = 0.0044f;
				params.derivatives.yawrate_yaw_coefficient = -0.0720f;

				// Control Terms //
				params.derivatives.elev_lift_coefficient = 0.2781f;
				params.derivatives.elev_drag_coefficient = 0.0633f; // the control variable is squared for this parameter in the model can this be returned to a simpler relation 
				params.derivatives.elevator_pitch_coefficient = -0.2292f;

				params.derivatives.aileron_sideforce_coefficient = 0.0433;
				params.derivatives.aileron_roll_coefficient = 0.1202f;
				params.derivatives.aileron_yaw_coefficient = -0.0034f;

				params.derivatives.rudder_yaw_coefficient = 0.0f;
				params.derivatives.rudder_sideforce_coefficient = 0.0f;
				params.derivatives.rudder_roll_coefficient = 0.0f;

				// Propulsion Terms //
				params.prop_derivatives.thrust_tla_coefficient = 40.0f; // set arbitrarily
				params.prop_derivatives.propeller_area = 0.1018f;
				params.prop_derivatives.propeller_thrust_coefficient = 1.0f;
				params.prop_derivatives.k_motor_coefficient = 40.0f;
				params.prop_derivatives.propeller_torque_coefficient = 0.0f;
				params.prop_derivatives.motor_torque_coefficient = 0.0f;
			}

			void setupFrameCherokee(Params& params)
			{
				// Basic Parameters //
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