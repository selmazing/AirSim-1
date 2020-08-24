#ifndef msr_airlib_fixedwingphysicsbody_hpp
#define msr_airlib_fixedwingphysicsbody_hpp

#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "api/VehicleApiBase.hpp"
#include "api/VehicleSimApiBase.hpp"
#include <vector>
#include "physics/PhysicsBody.hpp"
#include "FixedWingParams.hpp"
#include "ControlSurface.hpp"
#include "FixedWingParams.hpp"


namespace msr
{
	namespace airlib
	{
		class FixedWingPhysicsBody : public PhysicsBody
		{
		public:
			FixedWingPhysicsBody(FixedWingParams* params, VehicleApiBase* vehicle_api, Kinematics* kinematics, Environment* environment)
				: params_(params), vehicle_api_(vehicle_api)
			{
				initialize(kinematics, environment);
			}
			
			//*** Start: UpdatableState implementation ***//
			virtual void resetImplementation() override
			{
				//reset rotors, kinematics and environment
				PhysicsBody::resetImplementation();

				//reset sensors last after their ground truth has been reset
				resetSensors();
			}

			virtual void update() override
			{
				//update forces on vertices that we will use next
				PhysicsBody::update();

				//Note that controller gets updated after kinematics gets updated in updateKinematics
				//otherwise sensors will have values from previous cycle causing lags which will appear
				//as crazy jerks whenever commands like velocity is issued
			}

			virtual void reportState(StateReporter& reporter) override
			{
				//call base
				PhysicsBody::reportState(reporter);

				reportSensors(*params_, reporter);
				for (uint control_index = 0; control_index < controls_.size(); ++control_index)
				{
					reporter.startHeading("", 1);
					reporter.writeValue("Control", control_index);
					reporter.endHeading(false, 1);
					controls_.at(control_index).reportState(reporter);
				}
			}
			//*** End: UpdatableState implementation ***//

			//Physics engine calls this method to set next kinematics
			virtual void updateKinematics(const Kinematics::State& kinematics) override
			{
				PhysicsBody::updateKinematics(kinematics);

				updateSensors(*params_, getKinematics(), getEnvironment());

				//update controller which will update actuator control signal
				vehicle_api_->update();

				//transfer new input values from controller to control_surface
				for(uint control_index = 0; control_index < controls_.size(); ++control_index)
				{
					controls_.at(control_index).setControlSignal(vehicle_api_->getActuation(control_index));
				}
			}

			//sensor getter
			const SensorCollection& getSensors() const
			{
				return params_->getSensors();
			}

			//physics body interface

			ControlSurface::Output getControlSurfaceOutput(uint control_index) const
			{
				return controls_.at(control_index).getOutput();
			}

			virtual real_T getRestitution() const override
			{
				return params_->getParams().restitution;
			}

			virtual real_T getFriction() const override
			{
				return params_ ->getParams().friction;
			}
			
			virtual ~FixedWingPhysicsBody() = default;

		protected:

			virtual void setAoA(const Kinematics::State& kinematics)
			{
				aoa_->aero_axis = VectorMath::rotateVector(kinematics.twist.angular, kinematics.pose.orientation, true);
				aoa_->alpha = aoa_->aero_axis(0);
				aoa_->beta = aoa_->aero_axis(2);
			}

			void updateEnvironmentalFactors(const Kinematics::State& kinematics)
			{
				air_density_ratio_ = environment_->getState().air_density / air_density_sea_level_; // Sigma ratio
				forward_velocity_ = kinematics.twist.linear(0); // Indicated Airspeed
				dyn_pressure_ = 0.5 * environment_->getState().air_density * forward_velocity_ * forward_velocity_;
			}

			virtual void setWrench(Wrench& wrench)
			{
				wrench.force(0) = -1 * aero_force_->drag;
				wrench.force(1) = aero_force_->side_force;
				wrench.force(2) = -1 * aero_force_->lift;
				wrench.torque(0) = aero_force_->roll_mom;
				wrench.torque(1) = aero_force_->pitch_mom;
				wrench.torque(2) = aero_force_->yaw_mom;
			}

		private: //methods
			void initialize(Kinematics* kinematics, Environment* environment)
			{
				PhysicsBody::initialize(params_->getParams().mass, params_->getParams().inertia, kinematics, environment);
				

				initSensors(*params_, getKinematics(), getEnvironment());
			}

			virtual void createControls()
			{
				aileron_deflection_ = controls_.at(0).getOutput().control_deflection;
				elevator_deflection_ = controls_.at(1).getOutput().control_deflection;
				rudder_deflection_ = controls_.at(2).getOutput().control_deflection;
			}

			virtual void createAeroForces(const LinearAeroDerivatives& derivatives, const Dimensions& dimensions, const Kinematics::State& kinematics, ControlSurface& output)
			{	
				aero_force_->lift = dyn_pressure_ * dimensions.main_plane_area * (derivatives.zero_lift_coefficient + derivatives.alpha_lift_coefficient * aoa_->alpha + derivatives.pitch_lift_coefficient * kinematics.twist.angular(0) + derivatives.elev_lift_coefficient * elevator_deflection_);
				aero_force_->drag = dyn_pressure_ * dimensions.main_plane_area * (derivatives.zero_drag_coefficient + derivatives.alpha_drag_coefficient * aoa_->alpha + derivatives.pitch_drag_coefficient * kinematics.twist.linear(0) + derivatives.elev_drag_coefficient * elevator_deflection_);
				aero_force_->side_force = dyn_pressure_ * dimensions.vertical_tail_plane_area * (derivatives.sidevelocity_sideforce_coefficient * kinematics.twist.linear(1) + derivatives.rudder_sideforce_coefficient * rudder_deflection_);
				aero_force_->pitch_mom = dyn_pressure_ * dimensions.main_plane_area * (derivatives.zero_pitch_coefficient + derivatives.alpha_pitch_coefficient * aoa_->alpha + derivatives.pitchrate_pitch_coefficient * kinematics.twist.angular(1) + derivatives.elevator_pitch_coefficient * elevator_deflection_);
				aero_force_->roll_mom = dyn_pressure_ * dimensions.main_plane_area * (derivatives.zero_roll_coefficient + derivatives.rollrate_roll_coefficient * kinematics.twist.angular(0) + derivatives.aileron_roll_coefficient * aileron_deflection_);
				aero_force_->yaw_mom = dyn_pressure_ * dimensions.vertical_tail_plane_area * (derivatives.zero_yaw_coefficient + kinematics.twist.angular(2) + derivatives.rudder_yaw_coefficient * rudder_deflection_);
			}
			
			void reportSensors(FixedWingParams& params, StateReporter& reporter)
			{
				params.getSensors().reportState(reporter);
			}

			void updateSensors(FixedWingParams& params, const Kinematics::State& state, const Environment& environment)
			{
				unused(state);
				unused(environment);
				params.getSensors().update();
			}

			void initSensors(FixedWingParams& params, const Kinematics::State& state, const Environment& environment)
			{
				params.getSensors().initialize(&state, &environment);
			}

			void resetSensors()
			{
				params_->getSensors().reset();
			}

			private: //fields
			FixedWingParams* params_;
			vector<ControlSurface> controls_;
			real_T aileron_deflection_, elevator_deflection_, rudder_deflection_;
			std::unique_ptr<Environment> environment_;
			VehicleApiBase* vehicle_api_;
			real_T air_density_sea_level_, air_density_ratio_, dyn_pressure_, forward_velocity_;
			AoA* aoa_;
			AeroFM* aero_force_;
		};
	}
}

#endif
