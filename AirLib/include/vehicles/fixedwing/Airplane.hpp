#pragma once
#ifndef msr_airlib_airplane_hpp
#define msr_airlib_airplane_hpp

#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "physics/Environment.hpp"
#include "physics/Kinematics.hpp"
#include "physics/PhysicsBodyVertex.hpp"
#include "AircraftParams.hpp"
#include "ControlSurface.hpp"

namespace msr
{
	namespace airlib
	{
		class Airplane : PhysicsBodyVertex
		{
		public:
			struct Output
			{
				AeroFM aero_force_;
			};
			// types, declare variables in struct? --> currently no output struct

		public: // methods
			Airplane()
			{
				// default constructor
			}

			Airplane(const Vector3r& position, const Vector3r& normal, const LinearAeroDerivatives& derivative)
			{
				initialize(position, normal, derivative);
				air_density_sea_level_ = EarthUtils::getAirDensity(0.0f);
				
			}
			void initialize(const Vector3r& position, const Vector3r& normal, const LinearAeroDerivatives& derivative)
			{
				PhysicsBodyVertex::initialize(position, normal); // call base initializer
				
			}

			/* Start: update state implementation*/
			virtual void resetImplementation() override
			{
				PhysicsBodyVertex::resetImplementation();
				updateEnvironmentalFactors();
				//setOutput(output_, control_signal_filter_);
			}

			virtual void update() override
			{
				updateEnvironmentalFactors();
				PhysicsBodyVertex::update();
				//setOutput(output_, control_signal_filter_);
			}

			/*virtual void reportState(StateReporter& reporter) override
			{
				reporter.writeValue("Elevator-Ctrl-in",  setWrench());
			}*/
			/* End: update state implementation*/

			ControlSurface::Output getControlSurfaceOutput(uint control_index) const
			{
				return controls_.at(control_index).getOutput();
			}

			Output getOutput() const
			{
				return output_;
			}

		private: // methods

			virtual void setAoA()
			{
				aoa_->aero_axis = VectorMath::rotateVector(kinematics_->getState().twist.angular, kinematics_->getState().pose.orientation, true);
				aoa_->alpha = aoa_->aero_axis(0);
				aoa_->beta = aoa_->aero_axis(2);
			}

			void updateEnvironmentalFactors()
			{
				air_density_ratio_ = environment_->getState().air_density / air_density_sea_level_; // Sigma ratio
				forward_velocity_ = kinematics_->getState().twist.linear(0); // Indicated Airspeed
				dyn_pressure_ = 0.5 * environment_->getState().air_density * forward_velocity_ * forward_velocity_;
			}

			virtual void setWrench(Wrench& wrench) override
			{
				wrench.force(0) = -1 * output_.aero_force_.drag;
				wrench.force(1) = output_.aero_force_.side_force;
				wrench.force(2) = -1 * output_.aero_force_.lift;
				wrench.torque(0) = output_.aero_force_.roll_mom;
				wrench.torque(1) = output_.aero_force_.pitch_mom;
				wrench.torque(2) = output_.aero_force_.yaw_mom;
			}

			virtual void createControls()
			{
				aileron_deflection_ = controls_.at(0).getOutput().control_deflection;
				elevator_deflection_ = controls_.at(1).getOutput().control_deflection;
				rudder_deflection_ = controls_.at(2).getOutput().control_deflection;
			}

			virtual void createAeroForces(const LinearAeroDerivatives& derivatives, const Dimensions& dimensions, const Kinematics::State& kinematics, ControlSurface& output)
			{
				output_.aero_force_.lift = dyn_pressure_ * dimensions.main_plane_area * (derivatives.zero_lift_coefficient + derivatives.alpha_lift_coefficient * aoa_->alpha + derivatives.pitch_lift_coefficient * kinematics.twist.angular(0) + derivatives.elev_lift_coefficient * elevator_deflection_);
				output_.aero_force_.drag = dyn_pressure_ * dimensions.main_plane_area * (derivatives.zero_drag_coefficient + derivatives.alpha_drag_coefficient * aoa_->alpha + derivatives.pitch_drag_coefficient * kinematics.twist.linear(0) + derivatives.elev_drag_coefficient * elevator_deflection_);
				output_.aero_force_.side_force = dyn_pressure_ * dimensions.vertical_tail_plane_area * (derivatives.sidevelocity_sideforce_coefficient * kinematics.twist.linear(1) + derivatives.rudder_sideforce_coefficient * rudder_deflection_);
				output_.aero_force_.pitch_mom = dyn_pressure_ * dimensions.main_plane_area * (derivatives.zero_pitch_coefficient + derivatives.alpha_pitch_coefficient * aoa_->alpha + derivatives.pitchrate_pitch_coefficient * kinematics.twist.angular(1) + derivatives.elevator_pitch_coefficient * elevator_deflection_);
				output_.aero_force_.roll_mom = dyn_pressure_ * dimensions.main_plane_area * (derivatives.zero_roll_coefficient + derivatives.rollrate_roll_coefficient * kinematics.twist.angular(0) + derivatives.aileron_roll_coefficient * aileron_deflection_);
				output_.aero_force_.yaw_mom = dyn_pressure_ * dimensions.vertical_tail_plane_area * (derivatives.zero_yaw_coefficient + kinematics.twist.angular(2) + derivatives.rudder_yaw_coefficient * rudder_deflection_);
			}

		private: // fields
			const Environment* environment_ = nullptr;
			const Kinematics* kinematics_ = nullptr;
			real_T air_density_sea_level_, air_density_ratio_, dyn_pressure_, forward_velocity_;
			AoA* aoa_;
			Output output_;
			// AeroFM* aero_force_;
			real_T aileron_deflection_, elevator_deflection_, rudder_deflection_;
			vector<ControlSurface> controls_;
		};
		
	}
}
#endif