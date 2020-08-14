#ifndef dynamic_forces_hpp
#define dynamic_forces_hpp

#include "common/Common.hpp"
#include <limits>
#include "physics/Environment.hpp"
#include "physics/Kinematics.hpp"
#include "common/FirstOrderFilter.hpp"
#include "physics/PhysicsBodyVertex.hpp"
#include "DynamicParams.hpp"

namespace msr
{
	namespace airlib
	{
		/* Aircraft gets control deflection signal as input which changes the balance of forces on the aircraft*/
		class DynamicForces : public PhysicsBodyVertex
		{
		public: //types
			struct Output
			{
				real_T lift;
				real_T drag;
				real_T thrust;
				real_T weight;
				real_T side_force;
				real_T pitch_mom;
				real_T roll_mom;
				real_T yaw_mom;
				real_T elevator_signal_filtered; 
				real_T elevator_signal_input;
				real_T elevator_speed;
				real_T elevator_deflection;
				real_T aileron_signal_filtered;
				real_T aileron_signal_input;
				real_T aileron_speed;
				real_T aileron_deflection;
				real_T rudder_signal_filtered;
				real_T rudder_signal_input;
				real_T rudder_speed;
				real_T rudder_deflection;
				real_T alpha;
				real_T psi; // aero roll?
				real_T beta;
				Vector3r aero_axis;
			};
		public: //methods
			
			DynamicForces()
			{
				// Default constructor
			}

			DynamicForces(const Vector3r& position, const Vector3r& normal, const LinearAeroDerivatives& derivatives, const Environment* environment, uint id = -1 )
			{
				initialize(position, normal, derivatives, environment, id);
			}
			void initialize(const Vector3r& position, const Vector3r& normal, const LinearAeroDerivatives& derivatives, const Environment* environment, uint id = -1)
			{
				id_ = id;
				derivatives_ = derivatives;
				environment_ = environment;
				air_density_sea_level_ = EarthUtils::getAirDensity(0.0f);

				PhysicsBodyVertex::initialize(position, normal); // call base initializer
			}

			// set elevator signal from -1 to 1 for full deflection at limit
			void setElevatorSignal(real_T control_signal)
			{
				elevator_signal_filter_.setInput(Utils::clip(control_signal, -1.0f, 1.0f));
			}

			// set elevator signal from -1 to 1 for full deflection at limit
			void setAileronSignal(real_T control_signal)
			{
				aileron_signal_filter_.setInput(Utils::clip(control_signal, -1.0f, 1.0f));
			}

			// set elevator signal from -1 to 1 for full deflection at limit
			void setRudderSignal(real_T control_signal)
			{
				rudder_signal_filter_.setInput(Utils::clip(control_signal, -1.0f, 1.0f));
			}

			Output getOutput() const
			{
				return output_;
			}

			/* Start: update state implementation*/
			virtual void resetImplementation() override
			{
				PhysicsBodyVertex::resetImplementation();
				updateEnvironmentalFactors();
				elevator_signal_filter_.reset();
				aileron_signal_filter_.reset();
				rudder_signal_filter_.reset();
				setOutput(output_, derivatives_, elevator_signal_filter_, aileron_signal_filter_, rudder_signal_filter_, dimensions_);
			}

			virtual void update() override
			{
				updateEnvironmentalFactors();
				PhysicsBodyVertex::update();
				setOutput(output_, derivatives_, elevator_signal_filter_, aileron_signal_filter_, rudder_signal_filter_,  dimensions_);
				elevator_signal_filter_.update();
				aileron_signal_filter_.update();
				rudder_signal_filter_.update();
			}

			virtual void reportState(StateReporter& reporter) override
			{
				reporter.writeValue("Elevator-Ctrl-in", output_.elevator_signal_input);
				reporter.writeValue("Elevator-Ctrl-fl", output_.elevator_signal_filtered);
				reporter.writeValue("Elevator-Control-Deflection", output_.elevator_deflection);
				reporter.writeValue("Elevator-Control-Speed", output_.elevator_speed);
				reporter.writeValue("Aileron-Ctrl-in", output_.aileron_signal_input);
				reporter.writeValue("Aileron-Ctrl-fl", output_.aileron_signal_filtered);
				reporter.writeValue("Aileron-Control-Deflection", output_.aileron_deflection);
				reporter.writeValue("Aileron-Control-Speed", output_.aileron_speed);
				reporter.writeValue("Rudder-Ctrl-in", output_.rudder_signal_input);
				reporter.writeValue("Rudder-Ctrl-fl", output_.rudder_signal_filtered);
				reporter.writeValue("Rudder-Control-Deflection", output_.rudder_deflection);
				reporter.writeValue("Rudder-Control-Speed", output_.rudder_speed);
				reporter.writeValue("Lift", output_.lift);
				reporter.writeValue("Drag", output_.drag);
				reporter.writeValue("Thrust", output_.thrust);
				reporter.writeValue("Weight", output_.weight); // Might be confusing, will be 0 here I think, in FixedWingPhysicsBody.hpp more important
				reporter.writeValue("Side-Force", output_.side_force);
				reporter.writeValue("Pitch-Moment", output_.pitch_mom);
				reporter.writeValue("Roll-Moment", output_.roll_mom);
				reporter.writeValue("Yaw-Moment", output_.yaw_mom);
			}
			/* End: update state implementation*/
		protected:
			virtual void setWrench(Wrench& wrench) override
			{
				wrench.force(0) = -1 * output_.drag;
				wrench.force(1) = output_.side_force;
				wrench.force(2) = -1 * output_.lift;
				wrench.torque(0) = output_.roll_mom;
				wrench.torque(1) = output_.pitch_mom;
				wrench.torque(2) = output_.yaw_mom;
			}
			
		private: // methods

			//set angle of attack
			//TODO: Understand why pose is not working do I need to set/get pose?
			virtual void setAoA(Output& output)
			{
				output.aero_axis = VectorMath::rotateVector(kinematics_->getState().twist.angular, kinematics_->getState().pose.orientation, true);
				output.alpha = output_.aero_axis(0);
				output.beta = output_.aero_axis(2);
			}


			//calculates all the forces
			virtual void setOutput(Output& output, const LinearAeroDerivatives& derivatives, const FirstOrderFilter<real_T>& elevator_signal_filter, const FirstOrderFilter<real_T>& aileron_signal_filter, const FirstOrderFilter<real_T>& rudder_signal_filter, const Dimensions& dimensions )
			{
				output.elevator_signal_input = elevator_signal_filter_.getInput();
				output.elevator_signal_filtered = elevator_signal_filter_.getOutput();
				output.aileron_signal_input = aileron_signal_filter_.getInput();
				output.aileron_signal_filtered = aileron_signal_filter_.getOutput();
				output.rudder_signal_input = rudder_signal_filter_.getInput();
				output.rudder_signal_filtered = rudder_signal_filter_.getOutput();

				output.lift = dyn_pressure_ * dimensions.main_plane_area * (derivatives.zero_lift_coefficient + derivatives.alpha_lift_coefficient * output_.alpha + derivatives.pitch_lift_coefficient * kinematics_->getState().twist.angular(0)+ derivatives.elev_lift_coefficient * output_.elevator_deflection);
				output.drag = dyn_pressure_ * dimensions.main_plane_area * (derivatives.zero_drag_coefficient + derivatives.alpha_drag_coefficient * output_.alpha + derivatives.pitch_drag_coefficient * kinematics_->getState().twist.linear(0) + derivatives.elev_drag_coefficient * output_.elevator_deflection);
				output.side_force = dyn_pressure_ * dimensions.vertical_tail_plane_area * (derivatives.sidevelocity_sideforce_coefficient * kinematics_->getState().twist.linear(1) + derivatives.rudder_sideforce_coefficient * output_.rudder_deflection);
				output.pitch_mom = dyn_pressure_ * dimensions.main_plane_area * (derivatives.zero_pitch_coefficient + derivatives.alpha_pitch_coefficient * output_.alpha + derivatives.pitchrate_pitch_coefficient * kinematics_->getState().twist.angular(1) + derivatives.elevator_pitch_coefficient * output_.elevator_deflection);
				output.roll_mom = dyn_pressure_ * dimensions.main_plane_area * (derivatives.zero_roll_coefficient + derivatives.rollrate_roll_coefficient * kinematics_->getState().twist.angular(0) + derivatives.aileron_roll_coefficient * output_.aileron_deflection);
				output.yaw_mom = dyn_pressure_ * dimensions.vertical_tail_plane_area * (derivatives.zero_yaw_coefficient + kinematics_->getState().twist.angular(2) + derivatives.rudder_yaw_coefficient * output.rudder_deflection);
			}

			void updateEnvironmentalFactors()
			{
				air_density_ratio_ = environment_->getState().air_density / air_density_sea_level_; // Sigma ratio
				forward_velocity_ = kinematics_->getState().twist.linear(0); // Indicated Airspeed
				dyn_pressure_ = 0.5 * environment_->getState().air_density * forward_velocity_ * forward_velocity_;
			}

		private: //fields
			uint id_; // for debug messages
			LinearAeroDerivatives derivatives_;
			Dimensions dimensions_;
			FirstOrderFilter<real_T> elevator_signal_filter_, aileron_signal_filter_, rudder_signal_filter_;
			const Environment* environment_ = nullptr;
			const Kinematics* kinematics_ = nullptr;
			real_T air_density_sea_level_, air_density_ratio_, dyn_pressure_, forward_velocity_;
			Output output_;
			
			
		};
	}
}

#endif

