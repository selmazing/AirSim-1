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

#include "Engine/Engine.h"

namespace msr
{
	namespace airlib
	{
		class Airplane : public PhysicsBodyVertex
		{
		public:
			struct Output
			{
				AeroFM aero_force_;
				real_T thrust;
			};
			// types, declare variables in struct? --> currently no output struct

		public: // methods
			Airplane()
			{
				// default constructor
			}

			Airplane(const Vector3r& position, const Vector3r& normal, const LinearAeroDerivatives& aero_derivatives, const PropulsionDerivatives& prop_derivatives, const Dimensions dimensions, const Environment* environment, const Kinematics* kinematics)
			{
				initialize(position, normal, aero_derivatives, prop_derivatives, dimensions, environment, kinematics);
			}
			void initialize(const Vector3r& position, const Vector3r& normal, const LinearAeroDerivatives& aero_derivatives, const PropulsionDerivatives& prop_derivatives, const Dimensions dimensions, const Environment* environment, const Kinematics* kinematics)
			{
				air_density_sea_level_ = EarthUtils::getAirDensity(0.0f);
				environment_ = environment;
				kinematics_ = kinematics;
				aero_derivatives_ = aero_derivatives;
				prop_derivatives_ = prop_derivatives;
				dimensions_ = dimensions;
				createControls(4); // currently hardcoded should really get params when initialized in FixedWingPhysicsBody
				FString DEBUG_MSG = FString::Printf(TEXT("Its Running!"));
				GEngine->AddOnScreenDebugMessage(1, 3000.0f, FColor::Green, DEBUG_MSG);
				PhysicsBodyVertex::initialize(position, normal); // call base initializer
			}

			/* Start: update state implementation*/
			virtual void resetImplementation() override
			{
				PhysicsBodyVertex::resetImplementation();
				for (auto& control : controls_)
				{
					control.resetImplementation();
				}
				updateEnvironmentalFactors();
				createPropulsionForces(prop_derivatives_, output_);
				updateAeroForces(aero_derivatives_, dimensions_, kinematics_, output_);
			}

			virtual void update() override
			{
				updateEnvironmentalFactors();
				for(auto& control: controls_)
				{
					control.update();
				}
				createPropulsionForces(prop_derivatives_, output_);
				updateAeroForces(aero_derivatives_, dimensions_, kinematics_, output_);
				// should call getWrench
				PhysicsBodyVertex::update();
	
			}

			ControlSurface::Output getControlSurfaceOutput(uint control_index) const{
	
				return controls_.at(control_index).getOutput();
			}

			Output getOutput() const
			{
				return output_;
			}

		protected:

			virtual void setWrench(Wrench& wrench) override
			{
				const Vector3r aero_x = Vector3r(1, 0, 0);
				const Vector3r aero_y = Vector3r(0, 1, 0);
				const Vector3r aero_z = Vector3r(0, 0, 1);
				wrench.force = (aero_x * (-1 * output_.aero_force_.drag)) + (aero_x * output_.thrust);
				wrench.force += aero_y * output_.aero_force_.side_force;
				wrench.force += aero_z * (-1 * output_.aero_force_.lift);
				wrench.torque += aero_x * output_.aero_force_.roll_mom;
				wrench.torque += aero_y * output_.aero_force_.pitch_mom;
				wrench.torque = aero_z * output_.aero_force_.yaw_mom;
				FString DEBUG_MSG = FString::Printf(TEXT("Forces are being Called!"));
				GEngine->AddOnScreenDebugMessage(2, 3000.0f, FColor::Blue, DEBUG_MSG);
			}

		private: // methods

			void setAoA()
			{
				aoa_->aero_axis = VectorMath::rotateVector(kinematics_->getState().twist.angular, kinematics_->getState().pose.orientation, true);
				aoa_->alpha = aoa_->aero_axis(0);
				aoa_->beta = aoa_->aero_axis(2);
			}

			real_T setAirspeed()
			{
				real_T airspeed = sqrt(pow(kinematics_->getState().twist.linear(0), 2) * pow(kinematics_->getState().twist.linear(1), 2) * pow(kinematics_->getState().twist.linear(2), 2));
				return airspeed;
			}

			void updateEnvironmentalFactors()
			{
				air_density_ratio_ = environment_->getState().air_density / air_density_sea_level_; // Sigma ratio
			}

			void createPropulsionForces(const PropulsionDerivatives& derivatives, Output& output)
			{

				tla_deflection_ = controls_.at(2).getOutput().control_deflection + 1.0f;
				output.thrust = derivatives.thrust_tla_coefficient * tla_deflection_;
			}

			void createControls(const uint control_count)
			{
				for (uint i = 0; i < control_count; ++i)
				{
					controls_.push_back(ControlSurface());
				}
			}

			void updateAeroForces(const LinearAeroDerivatives& derivatives, const Dimensions& dimensions, const Kinematics* kinematics, Output& output)
			{
				aileron_deflection_ = controls_.at(0).getOutput().control_deflection;
				elevator_deflection_ = controls_.at(1).getOutput().control_deflection;
				rudder_deflection_ = controls_.at(3).getOutput().control_deflection;
				
				const real_T airspeed = setAirspeed();
				dyn_pressure_ = 0.5 * environment_->getState().air_density * pow(airspeed, 2);
				const real_T angular_pressure = 0.25 * environment_->getState().air_density * airspeed * dimensions.main_plane_area * dimensions.main_plane_chord; // expanded out 1/2*rho*V^2 * S * (c / 2 * V) to prevent division by zero

				
				output.aero_force_.lift = dyn_pressure_ * dimensions.main_plane_area * (
					derivatives.zero_lift_coefficient +
					(derivatives.alpha_lift_coefficient * aoa_->alpha) +
					(derivatives.elev_lift_coefficient * elevator_deflection_)) +
					(derivatives.pitch_lift_coefficient * angular_pressure * kinematics_->getState().twist.angular(1));
				
				output.aero_force_.drag = dyn_pressure_ * dimensions.main_plane_area * (
					derivatives.zero_drag_coefficient +
					(derivatives.alpha_drag_coefficient * aoa_->alpha) +
					(derivatives.alpha_drag_coefficient_2 * (aoa_->alpha * aoa_->alpha)) +
					(derivatives.beta_drag_coefficient * aoa_->beta) +
					(derivatives.beta_drag_coefficient_2 * (aoa_->beta * aoa_->beta)) +
					(derivatives.elev_drag_coefficient * elevator_deflection_)) +
					(derivatives.pitch_drag_coefficient * angular_pressure * kinematics_->getState().twist.angular(1));
				
				output.aero_force_.side_force = dyn_pressure_ * dimensions.main_plane_area * (
					derivatives.zero_sideforce_coefficient +
					(derivatives.beta_sideforce_coefficient * aoa_->beta) +
					(derivatives.sidevelocity_sideforce_coefficient * kinematics_->getState().twist.linear(1)) +
					(derivatives.rudder_sideforce_coefficient * rudder_deflection_)) +
					(derivatives.rollrate_sideforce_coefficient * angular_pressure * kinematics_->getState().twist.angular(0)) +
					(derivatives.yawrate_sideforce_coefficient * angular_pressure * kinematics_->getState().twist.angular(2));
				
				output.aero_force_.pitch_mom = dyn_pressure_ * dimensions.main_plane_area * dimensions.main_plane_chord * (
					derivatives.zero_pitch_coefficient +
					(derivatives.alpha_pitch_coefficient * aoa_->alpha) +
					(derivatives.elevator_pitch_coefficient * elevator_deflection_)) +
					(derivatives.pitchrate_pitch_coefficient * angular_pressure * kinematics_->getState().twist.angular(1));
				
				output.aero_force_.roll_mom = dyn_pressure_ * dimensions.main_plane_area * dimensions.main_plane_span * (
					derivatives.zero_roll_coefficient +
					(derivatives.beta_roll_coefficient * aoa_->beta) +
					(derivatives.aileron_roll_coefficient * aileron_deflection_)) +
					(derivatives.rollrate_roll_coefficient * angular_pressure * kinematics_->getState().twist.angular(0)) +
					(derivatives.yawrate_roll_coefficient * angular_pressure * kinematics_->getState().twist.angular(2));

				
				output.aero_force_.yaw_mom = dyn_pressure_ * dimensions.main_plane_area * dimensions.main_plane_span * (
					derivatives.zero_yaw_coefficient +
					(derivatives.beta_yaw_coefficient * aoa_->beta) +
					(derivatives.aileron_yaw_coefficient * aileron_deflection_) +
					(derivatives.rudder_yaw_coefficient * rudder_deflection_)) +
					(derivatives.rollrate_yaw_coefficient * angular_pressure * kinematics_->getState().twist.angular(0)) +
					(derivatives.yawrate_yaw_coefficient * angular_pressure * kinematics_->getState().twist.angular(2));

				if(isnan(output.aero_force_.lift))
				{
					FString DEBUG_MSG = FString::Printf(TEXT("Forces are being Called!"));
					GEngine->AddOnScreenDebugMessage(2, 3000.0f, FColor::Blue, DEBUG_MSG);
				}
			}


		private: // fields
			const Environment* environment_ = nullptr;
			const Kinematics* kinematics_ = nullptr;
			LinearAeroDerivatives aero_derivatives_;
			PropulsionDerivatives prop_derivatives_;
			Dimensions dimensions_;
			real_T air_density_sea_level_, air_density_ratio_, dyn_pressure_;
			AoA* aoa_;
			Output output_;
			// AeroFM* aero_force_;
			real_T aileron_deflection_, elevator_deflection_, rudder_deflection_, tla_deflection_;
		public:
			vector<ControlSurface> controls_;
		};
		
	}
}
#endif