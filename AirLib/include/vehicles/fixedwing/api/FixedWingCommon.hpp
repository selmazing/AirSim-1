#ifndef air_FixedWingCommon_hpp
#define air_FixedWingCommon_hpp

#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "physics/Kinematics.hpp"

namespace msr
{
	namespace airlib
	{
		//properties of vehicle
		struct FixedWingApiParams
		{
			FixedWingApiParams() {};

			//just copied from multirotor maybe this is a good way to get the fixed wing aircraft to learn?
			float vel_to_breaking_dist = 0.5f;   //ideally this should be 2X for very high speed but for testing we are keeping it 0.5
			float min_breaking_dist = 1; //min breaking distance
			float max_breaking_dist = 3; //min breaking distance
			float breaking_vel = 1.0f;
			float min_vel_for_breaking = 3;


			/*what is the differential positional accuracy of cur_loc?
			this is not same as GPS accuracy because translational errors
			usually cancel out. Typically this would be 0.2m or less*/
			float distance_accuracy = 0.1f;

			//what is the minimum clearance from obstacles?
			float obs_clearance = 2;

			/*what is the +/-window we should check on obstacle map?
			for example 2 means check from ticks -2 to 2*/
			int obs_window = 0;
		};

		struct FixedWingState
		{
			CollisionInfo collision;
			Kinematics::State kinematics_estimated;
			GeoPoint gps_location;
			uint64_t timestamp;
			LandedState landed_state;
			RCData rc_data;
			bool ready; // indicates the aircraft is ready to receive commands
			std::string ready_message; // displays error message if aircraft not reachable over network
			bool can_arm; // indicates aircraft is ready to be armed

			FixedWingState()
			{}
			FixedWingState(
				const CollisionInfo& collision_val,
				const Kinematics::State& kinematics_estimated_val,
				const GeoPoint& gps_location_val,
				uint64_t timestamp_val,
				LandedState landed_state_val,
				const RCData& rc_data_val,
				bool ready_val,
				const std::string& message,
				bool can_arm_val) :
			collision(collision_val),
			kinematics_estimated(kinematics_estimated_val),
			gps_location(gps_location_val),
			timestamp(timestamp_val),
			landed_state(landed_state_val),
			rc_data(rc_data_val),
			ready(ready_val),
			ready_message(message),
			can_arm(can_arm_val)
			{
			}

			//shortcuts
			const Vector3r& getPosition() const
			{
				return kinematics_estimated.pose.position;
			};
			const Quaternionr& getOrientation() const
			{
				return kinematics_estimated.pose.orientation;
			};
		};

	}
}

#endif
