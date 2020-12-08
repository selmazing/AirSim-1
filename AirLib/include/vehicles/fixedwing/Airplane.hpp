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
				real_T torque;
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
				updatePropulsionForces(prop_derivatives_, output_);
				updateAeroForces(aero_derivatives_, dimensions_, output_);
			}

			virtual void update() override
			{
				updateEnvironmentalFactors();
				for(auto& control: controls_)
				{
					control.update();
				}
				updateAoA();
				updatePropulsionForces(prop_derivatives_, output_);
				updateAeroForces(aero_derivatives_, dimensions_, output_);
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

			// something not right here with the += potentially?
			virtual void setWrench(Wrench& wrench) override
			{
				const Vector3r aero_x = Vector3r(1, 0, 0);
				const Vector3r aero_y = Vector3r(0, 1, 0);
				const Vector3r aero_z = Vector3r(0, 0, 1);
				const Vector3r aero_body_force = WindToBodyForce(aoa_.alpha, aoa_.beta, output_.aero_force_.drag, output_.aero_force_.side_force, output_.aero_force_.lift);

				wrench.force = (aero_x * aero_body_force(0)) + (aero_x * output_.thrust) + (aero_y * aero_body_force(1)) + (aero_z * aero_body_force(2));
				wrench.torque = (aero_x * output_.aero_force_.roll_mom) + (aero_x * output_.torque) + (aero_y * output_.aero_force_.pitch_mom) + (aero_z * output_.aero_force_.yaw_mom);

				Utils::log(Utils::stringf("AForce [%f, %f, %f] ATorque [%f, %f, %f]", output_.thrust-output_.aero_force_.drag,  output_.aero_force_.side_force, -output_.aero_force_.lift, output_.aero_force_.roll_mom,
					output_.aero_force_.pitch_mom, output_.aero_force_.yaw_mom, Utils::kLogLevelInfo));

				// debugWrenchFile(wrench);

				
			}

		private: // methods

			void updateAoA()
			{
				// Quaternionr quaternion = kinematics_->getState().pose.orientation;
				// Vector3r aircraft_euler = toEuler(quaternion);
				Vector3r linear_velocity = kinematics_->getState().twist.linear;
				// aoa_.aero_axis = angleBetweenVectors(aircraft_euler, linear_velocity); 
				// aoa_.aero_axis = VectorMath::rotateVector(kinematics_->getState().twist.angular, kinematics_->getState().pose.orientation, true);
				// aoa_.alpha = aoa_.aero_axis(0);
				// aoa_.alpha = 0.15;
				// aoa_.beta = aoa_.aero_axis(1);
				// aoa_.beta = aoa_.aero_axis(1);

				/* if(isnan(aoa_.alpha) || isnan(aoa_.beta))
				{
					aoa_.alpha = 0;
					aoa_.beta = 0;
				} */
				
				/* aoa_.alpha = 0.0f;
				aoa_.beta = 0.0f; */

				/* Utils::log(Utils::stringf("Angular variables: p: %f, q: %f, r: %f, alpha: %f, beta: %f, psi (aero_roll?): %f",
				kinematics_->getState().twist.angular(0), kinematics_->getState().twist.angular(1), kinematics_->getState().twist.angular(2), aoa_.alpha, aoa_.beta, Utils::kLogLevelInfo)); */

				// x8 paper style, not sure if this doesn't allow for alpha to increase with pitch angle?
				aoa_.alpha = atan2(linear_velocity(2), linear_velocity(0));
				aoa_.beta = asin(linear_velocity(1) / updateAirspeed());
				Utils::log(Utils::stringf("AeroAngles [%f, %f, %f]", aoa_.alpha, aoa_.beta, 0, Utils::kLogLevelInfo));

			}

			real_T updateAirspeed()
			{
				real_T airspeed = sqrt(pow(kinematics_->getState().twist.linear(0), 2) + pow(kinematics_->getState().twist.linear(1), 2) + pow(kinematics_->getState().twist.linear(2), 2));
				if (airspeed == 0)
				{
					airspeed = 0.00000001;
				}
				return airspeed;
			}

			void updateEnvironmentalFactors()
			{
				air_density_ratio_ = environment_->getState().air_density / air_density_sea_level_; // Sigma ratio
			}

			void updatePropulsionForces(const PropulsionDerivatives& derivatives, Output& output)
			{
				tla_deflection_ = controls_.at(2).getOutput().control_deflection + 1.0f;
				const real_T airspeed = updateAirspeed();
				const real_T discharge_velocity = airspeed + tla_deflection_ * (derivatives.k_motor_coefficient - airspeed);
				output.thrust = 0.5 * environment_->getState().air_density * derivatives.propeller_area * discharge_velocity * (discharge_velocity - airspeed);
				output.torque = -derivatives.motor_torque_coefficient * (derivatives.propeller_torque_coefficient * tla_deflection_);
				Utils::log(Utils::stringf("Va = %f, Vd = %f, thrust = %f, torque = %f", airspeed, discharge_velocity, output.thrust, output.torque, Utils::kLogLevelInfo));
				
				// output.thrust = derivatives.thrust_tla_coefficient * tla_deflection_; old linear thrust method
				// Utils::log(Utils::stringf("Thrust: %f = TLA: %f * Cttla: %f", output.thrust, tla_deflection_, derivatives.thrust_tla_coefficient, Utils::kLogLevelInfo));
			}

			void createControls(const uint control_count)
			{
				for (uint i = 0; i < control_count; ++i)
				{
					controls_.push_back(ControlSurface());
				}
			}

			void updateAeroForces(const LinearAeroDerivatives& derivatives, const Dimensions& dimensions, Output& output)
			{
				aileron_deflection_ = (30 * (std::_Pi / 180)) * controls_.at(0).getOutput().control_deflection;
				elevator_deflection_ = (25 * (std::_Pi / 180)) * controls_.at(1).getOutput().control_deflection;
				rudder_deflection_ = (20 * (std::_Pi / 180)) * controls_.at(3).getOutput().control_deflection;
				
				real_T airspeed = updateAirspeed();
				// Utils::log(Utils::stringf("Airspeed: %f", airspeed, Utils::kLogLevelInfo));
				dyn_pressure_ = 0.5 * environment_->getState().air_density * pow(airspeed, 2);
				// Utils::log(Utils::stringf("Dynamic Pressure: %f", dyn_pressure_, Utils::kLogLevelInfo));
				Utils::log(Utils::stringf("air density: %f", environment_->getState().air_density, Utils::kLogLevelInfo));
				const real_T angular_pressure = 0.25 * environment_->getState().air_density * airspeed * dimensions.main_plane_area; // expanded out 1/2*rho*V^2 * S * (c / 2 * V) to prevent division by zero

				output.aero_force_.lift = 0;
				output.aero_force_.drag = 0;
				output.aero_force_.side_force = 0;
				output.aero_force_.pitch_mom = 0;
				output.aero_force_.roll_mom = 0;
				output.aero_force_.yaw_mom = 0;
				
				
				output.aero_force_.lift = dyn_pressure_ * dimensions.main_plane_area * (
					derivatives.zero_lift_coefficient +
					(derivatives.alpha_lift_coefficient * aoa_.alpha) +
					(derivatives.elev_lift_coefficient * elevator_deflection_)) +
					(derivatives.pitch_lift_coefficient * angular_pressure * dimensions.main_plane_chord * kinematics_->getState().twist.angular(1));
				
				output.aero_force_.drag = dyn_pressure_ * dimensions.main_plane_area * (
					derivatives.zero_drag_coefficient +
					(derivatives.alpha_drag_coefficient * aoa_.alpha) +
					(derivatives.alpha_drag_coefficient_2 * (aoa_.alpha * aoa_.alpha)) +
					(derivatives.beta_drag_coefficient * aoa_.beta) +
					(derivatives.beta_drag_coefficient_2 * (aoa_.beta * aoa_.beta)) +
					(derivatives.elev_drag_coefficient * (elevator_deflection_ * elevator_deflection_))) +
					(derivatives.pitch_drag_coefficient * angular_pressure * dimensions.main_plane_chord * kinematics_->getState().twist.angular(1));
				
				output.aero_force_.side_force = dyn_pressure_ * dimensions.main_plane_area * (
					derivatives.zero_sideforce_coefficient +
					(derivatives.beta_sideforce_coefficient * aoa_.beta) +
					(derivatives.sidevelocity_sideforce_coefficient * kinematics_->getState().twist.linear(1)) +
					(derivatives.rudder_sideforce_coefficient * rudder_deflection_) +
					(derivatives.aileron_sideforce_coefficient * aileron_deflection_)) +
					(derivatives.rollrate_sideforce_coefficient * angular_pressure * dimensions.main_plane_span * kinematics_->getState().twist.angular(0)) +
					(derivatives.yawrate_sideforce_coefficient * angular_pressure * dimensions.main_plane_span * kinematics_->getState().twist.angular(2));
				
				output.aero_force_.pitch_mom = dyn_pressure_ * dimensions.main_plane_area * dimensions.main_plane_chord * (
					derivatives.zero_pitch_coefficient +
					(derivatives.alpha_pitch_coefficient * aoa_.alpha) +
					(derivatives.elevator_pitch_coefficient * elevator_deflection_)) +
					(derivatives.pitchrate_pitch_coefficient * angular_pressure * dimensions.main_plane_chord * dimensions.main_plane_chord * kinematics_->getState().twist.angular(1));
				
				output.aero_force_.roll_mom = dyn_pressure_ * dimensions.main_plane_area * dimensions.main_plane_span * (
					derivatives.zero_roll_coefficient +
					(derivatives.beta_roll_coefficient * aoa_.beta) +
					(derivatives.aileron_roll_coefficient * aileron_deflection_)) +
					(derivatives.rollrate_roll_coefficient * angular_pressure * dimensions.main_plane_span * dimensions.main_plane_span * kinematics_->getState().twist.angular(0)) +
					(derivatives.yawrate_roll_coefficient * angular_pressure * dimensions.main_plane_span * dimensions.main_plane_span * kinematics_->getState().twist.angular(2));

				
				output.aero_force_.yaw_mom = dyn_pressure_ * dimensions.main_plane_area * dimensions.main_plane_span * (
					derivatives.zero_yaw_coefficient +
					(derivatives.beta_yaw_coefficient * aoa_.beta) +
					(derivatives.aileron_yaw_coefficient * aileron_deflection_) +
					(derivatives.rudder_yaw_coefficient * rudder_deflection_)) +
					(derivatives.rollrate_yaw_coefficient * angular_pressure * dimensions.main_plane_span * dimensions.main_plane_span * kinematics_->getState().twist.angular(0)) +
					(derivatives.yawrate_yaw_coefficient * angular_pressure * dimensions.main_plane_span * dimensions.main_plane_span * kinematics_->getState().twist.angular(2));
				
				debugKinematicsMessages();
				debugAeroMessages(derivatives, dimensions, output, angular_pressure);
				
				if(isnan(output.aero_force_.lift))
				{
					Utils::log(Utils::stringf("Lift is not a number, something has gone wrong!"));
				}
			}


			void debugAeroMessages(const LinearAeroDerivatives& derivatives, const Dimensions& dimensions, Output& output, real_T angular_pressure) const
			{
				Utils::log(Utils::stringf("Lift: %f = q: %f * S: %f * (Cl0: %f + Clalpha: %f * alpha: %f + Clelev: %f * elev: %f) + (Clq: %f * Q_ang: %f * q: %f) ", output.aero_force_.lift, dyn_pressure_, dimensions.main_plane_area,
					derivatives.zero_lift_coefficient, derivatives.alpha_lift_coefficient, aoa_.alpha, derivatives.elev_lift_coefficient, elevator_deflection_,
					derivatives.pitch_lift_coefficient, angular_pressure, kinematics_->getState().twist.angular(1), Utils::kLogLevelInfo));

				Utils::log(Utils::stringf("Drag: %f = q: %f * S: %f * (Cd0: %f + Cdalpha: %f * alpha: %f + Cdalpha2: %f * alpha^2: %f + Cdbeta: %f * beta: %f + Cdbeta2: %f * beta2: %f + Cdelev: %f * elev: %f) + (Cdq: %f * Q_ang: %f * q: %f) ", output.aero_force_.drag, dyn_pressure_, dimensions.main_plane_area,
					derivatives.zero_drag_coefficient, derivatives.alpha_drag_coefficient, aoa_.alpha, derivatives.alpha_drag_coefficient_2, aoa_.alpha * aoa_.alpha,
					derivatives.beta_drag_coefficient, aoa_.beta, derivatives.beta_drag_coefficient_2, (aoa_.beta * aoa_.beta),
					derivatives.elev_drag_coefficient, elevator_deflection_,
					derivatives.pitch_drag_coefficient, angular_pressure, kinematics_->getState().twist.angular(1), Utils::kLogLevelInfo));

				Utils::log(Utils::stringf("SideForce: %f = q: %f * S: %f * (Cy0: %f + Cybeta: %f * beta %f + Cyv: %f * v: %f + Cyrudd: %f * rudd: %f) + (CYp: %f * Q_ang: %f * p: %f) + (CYr: %f * Q_ang: %f * r: %f) ", output.aero_force_.side_force, dyn_pressure_, dimensions.main_plane_area,
					derivatives.zero_sideforce_coefficient, derivatives.beta_sideforce_coefficient, aoa_.beta, derivatives.sidevelocity_sideforce_coefficient, kinematics_->getState().twist.linear(1), derivatives.rudder_sideforce_coefficient, rudder_deflection_,
					derivatives.rollrate_sideforce_coefficient, angular_pressure, kinematics_->getState().twist.angular(0),
					derivatives.yawrate_sideforce_coefficient, angular_pressure, kinematics_->getState().twist.angular(2), Utils::kLogLevelInfo));

				Utils::log(Utils::stringf("Pitching Moment: %f = q: %f * S: %f * b: %f (Cm0: %f + Cmalpha: %f * alpha: %f + Cmelev: %f * elev: %f) + (Cmq: %f * Q_ang: %f * q: %f) ", output.aero_force_.pitch_mom, dyn_pressure_, dimensions.main_plane_area, dimensions.main_plane_span,
					derivatives.zero_pitch_coefficient, derivatives.alpha_pitch_coefficient, aoa_.alpha, derivatives.elevator_pitch_coefficient, elevator_deflection_,
					derivatives.pitchrate_pitch_coefficient, angular_pressure, kinematics_->getState().twist.angular(1), Utils::kLogLevelInfo));

				Utils::log(Utils::stringf("Rolling Moment: %f = q: %f * S: %f * c: %f (Cl0: %f + Clbeta: %f * beta: %f + Clail: %f * ail: %f) + (Clp: %f * Q_ang: %f * p: %f) + (Clr: %f * Q_ang: %f * r: %f) ", output.aero_force_.roll_mom, dyn_pressure_, dimensions.main_plane_area, dimensions.main_plane_chord,
					derivatives.zero_roll_coefficient, derivatives.beta_roll_coefficient, aoa_.beta, derivatives.aileron_roll_coefficient, aileron_deflection_,
					derivatives.rollrate_roll_coefficient, angular_pressure, kinematics_->getState().twist.angular(0),
					derivatives.yawrate_roll_coefficient, angular_pressure, kinematics_->getState().twist.angular(2), Utils::kLogLevelInfo));

				Utils::log(Utils::stringf("Yawing Moment: %f = q: %f * S: %f * c: %f (Cn0: %f + Cnbeta: %f * beta: %f + Clail: %f * ail: %f) + (Cnp: %f * Q_ang: %f * p: %f) + (Cnr: %f * Q_ang: %f * r: %f) ", output.aero_force_.yaw_mom, dyn_pressure_, dimensions.main_plane_area, dimensions.main_plane_chord,
					derivatives.zero_yaw_coefficient, derivatives.beta_yaw_coefficient, aoa_.beta, derivatives.aileron_yaw_coefficient, aileron_deflection_,
					derivatives.rollrate_yaw_coefficient, angular_pressure, kinematics_->getState().twist.angular(0),
					derivatives.yawrate_yaw_coefficient, angular_pressure, kinematics_->getState().twist.angular(2), Utils::kLogLevelInfo));

				// debugAeroFile();
			}

			void debugKinematicsMessages() const
			{
				Quaternionr quaternion = kinematics_->getState().pose.orientation;
				Vector3r aircraft_euler = toEuler(quaternion);
				Vector3r position = kinematics_->getState().pose.position;
				Vector3r linear_velocity = kinematics_->getState().twist.linear;
				Vector3r angular_velocity = kinematics_->getState().twist.angular;
				Vector3r linear_acceleration = kinematics_->getState().accelerations.linear;
				Vector3r angular_acceleration = kinematics_->getState().accelerations.angular;
				Vector3r wind_axis = VectorMath::rotateVector(linear_velocity, quaternion, true);
				Vector3r manual_wind_axis = angleBetweenVectors(aircraft_euler, linear_velocity);

				Utils::log(Utils::stringf("y = [%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f]",
					position(0), position(1), position(2),
					aircraft_euler(0), aircraft_euler(1), aircraft_euler(2),
					linear_velocity(0), linear_velocity(1), linear_velocity(2),
					angular_velocity(0), angular_velocity(1), angular_velocity(2)));
				Utils::log(Utils::stringf("u = [%f, %f, %f, %f]", elevator_deflection_, aileron_deflection_, rudder_deflection_, tla_deflection_));
				
				/* Utils::log(Utils::stringf("Pose Values as quaternion: q = %f + %f i + %f j + %f k", quaternion.coeffs().w(), quaternion.coeffs().x(), quaternion.coeffs().y(), quaternion.coeffs().z(), Utils::kLogLevelInfo));
				Utils::log(Utils::stringf("Position: Xe = %f, Ye = %f, Ze = %f", position(0), position(1), position(2), Utils::kLogLevelInfo));
				// Utils::log(Utils::stringf("Wind axis: Xwang = %f, Ywang = %f, Zwang = %f", wind_axis(0), wind_axis(1), wind_axis(2), Utils::kLogLevelInfo));
				Utils::log(Utils::stringf("Manual Wind axis: Xmwang = %f, Ymwang = %f, Zmwang = %f", manual_wind_axis(0), manual_wind_axis(1), manual_wind_axis(2), Utils::kLogLevelInfo));
				Utils::log(Utils::stringf("Linear velocity: Ub = %f, Vb = %f, Wb = %f", linear_velocity(0), linear_velocity(1), linear_velocity(2), Utils::kLogLevelInfo));
				Utils::log(Utils::stringf("Angular velocity: pb = %f, qb = %f, qr = %f", angular_velocity(0), angular_velocity(1), angular_velocity(2), Utils::kLogLevelInfo));
				Utils::log(Utils::stringf("Linear acceleration: axb = %f, ayb = %f, azb = %f", linear_acceleration(0), linear_acceleration(1), linear_acceleration(2), Utils::kLogLevelInfo));
				Utils::log(Utils::stringf("Angular acceleration: pdotb = %f, qdotb = %f, rdotb = %f", angular_acceleration(0), angular_acceleration(1), angular_acceleration(2), Utils::kLogLevelInfo)); */
				
			}

			void debugWrenchFile(Wrench& wrench) const
			{
				std::ofstream wrenchFile("C:/Users/quessy/Documents/AirSim/force-file.txt", std::ios_base::app | std::ios_base::out);
				wrenchFile << wrench.force(0)
					<< "," << wrench.force(1)
					<< "," << wrench.force(2)
					<< "," << wrench.torque(0)
					<< "," << wrench.torque(1)
					<< "," << wrench.torque(2)
					<< "\n";
			}

			void debugAeroFile() const
			{
				std::ofstream aeroFile("C:/Users/quessy/Documents/AirSim/aero-file.txt", std::ios_base::app | std::ios_base::out);
				aeroFile << aoa_.alpha
					<< "," << aoa_.beta
					<< "," << kinematics_->getState().twist.linear(0)
					<< "," << kinematics_->getState().twist.linear(1)
					<< "," << kinematics_->getState().twist.linear(2)
					<< "," << kinematics_->getState().twist.angular(0)
					<< "," << kinematics_->getState().twist.angular(1)
					<< "," << kinematics_->getState().twist.angular(2)
					<< "\n";
			}

			Vector3r WindToBodyForce(real_T alpha, real_T beta, real_T drag, real_T side_force, real_T lift) const
			{
				Vector3r body_force;
				body_force(0) = (-drag * cos(alpha) * cos(beta)) + (side_force * sin(beta) * cos(alpha)) + (lift * sin(alpha));
				body_force(1) = (drag * sin(beta)) + (side_force * cos(beta));
				body_force(2) = (-drag * sin(alpha) * cos(beta)) + (side_force * sin(alpha) * sin(beta)) - (lift * cos(alpha));

				Utils::log(Utils::stringf("Rzyx breakdown: drag = %f, side_force = %f, lift = %f, cos(%f) = %f, sin(%f) = %f, cos(%f) = %f, sin(%f) = %f", drag, side_force, lift, alpha, cos(alpha), alpha, sin(alpha), beta, cos(beta), beta, sin(beta)));

				return body_force;
			}

			void kinematicsWrite() const
			{
				float time = Utils::getTimeSinceEpochSecs();
				Vector3r linear_acceleration = kinematics_->getState().accelerations.linear;
			}

			Vector3r angleBetweenVectors(Vector3r aVector, Vector3r bVector) const
			{
				Vector3r direction_vector;
				Vector3r dot;
				// dot product a.b
				dot(0) = aVector(0) * bVector(1);
				dot(1) = aVector(1) * bVector(1);
				dot(2) = aVector(2) * bVector(2);

				// normalize result
				const real_T size = pow(pow(aVector(0), 2) + pow(aVector(1), 2) + pow(aVector(2), 2), 0.5) + pow(pow(bVector(0), 2) + pow(bVector(1), 2) + pow(bVector(2), 2), 0.5);
				dot = dot / size;

				direction_vector(0) = std::acos(dot(0));
				direction_vector(1) = std::acos(dot(1));
				direction_vector(2) = std::acos(dot(2));
				return direction_vector;
			}
			
			Vector3r toEuler(Quaternionr quaternion) const
			{
				Vector3r euler;
				// Converts a quaternion to an euler angle, shamelessly using method from wikipedia https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
				// roll (x-axis rotation)
				double sinr_cosp = 2 * (quaternion.coeffs().w() * quaternion.coeffs().x() + quaternion.coeffs().y() * quaternion.coeffs().z());
				double cosr_cosp = 1 - 2 * (quaternion.coeffs().x() * quaternion.coeffs().x() + quaternion.coeffs().y() * quaternion.coeffs().y());
				euler(0) = std::atan2(sinr_cosp, cosr_cosp);

				// pitch (y-axis rotation)
				double sinp = 2 * (quaternion.coeffs().w() * quaternion.coeffs().y() - quaternion.coeffs().z() * quaternion.coeffs().x());
				if (std::abs(sinp) >= 1)
					euler(1) = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
				else
					euler(1) = std::asin(sinp);

				// yaw (z-axis rotation)
				double siny_cosp = 2 * (quaternion.coeffs().w() * quaternion.coeffs().z() + quaternion.coeffs().x() * quaternion.coeffs().y());
				double cosy_cosp = 1 - 2 * (quaternion.coeffs().y() * quaternion.coeffs().y() + quaternion.coeffs().z() * quaternion.coeffs().z());
				euler(2) = std::atan2(siny_cosp, cosy_cosp);

				return euler;
			}

		private: // fields
			const Environment* environment_ = nullptr;
			const Kinematics* kinematics_ = nullptr;
			LinearAeroDerivatives aero_derivatives_;
			PropulsionDerivatives prop_derivatives_;
			Dimensions dimensions_;
			real_T air_density_sea_level_, air_density_ratio_, dyn_pressure_;
			AoA aoa_;
			Output output_;
			// AeroFM* aero_force_;
			real_T aileron_deflection_, elevator_deflection_, rudder_deflection_, tla_deflection_;
		public:
			vector<ControlSurface> controls_;
		};
		
	}
}
#endif