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
#include "Airplane.hpp"


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
				Utils::log("Hello Physics!");
			}
			
			//*** Start: UpdatableState implementation ***//
			virtual void resetImplementation() override
			{
				//reset controls, kinematics and environment
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
				for (uint control_index = 0; control_index < airplane_.controls_.size(); ++control_index)
				{
					reporter.startHeading("", 1);
					reporter.writeValue("Control", control_index);
					reporter.endHeading(false, 1);
					airplane_.controls_.at(control_index).reportState(reporter);
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
				for(uint control_index = 0; control_index < airplane_.controls_.size(); ++control_index)
				{
					airplane_.controls_.at(control_index).setControlSignal(vehicle_api_->getActuation(control_index));

					// Utils::log(Utils::stringf("Received Control [%i]: %f", control_index, airplane_.controls_.at(control_index).getOutput().control_deflection, Utils::kLogLevelInfo));
				}
			}

			//sensor getter
			const SensorCollection& getSensors() const
			{
				return params_->getSensors();
			}

			//physics body interface
			uint controlCount() const
			{
				return params_->getParams().control_count;
			}

			virtual uint wrenchVertexCount() const override
			{
				return params_->getParams().airplane_count;
			}
			virtual PhysicsBodyVertex& getWrenchVertex(uint index)  override
			{
				// Utils::log("getWrenchVertex called");
				return airplane_;
			}
			virtual const PhysicsBodyVertex& getWrenchVertex(uint index) const override
			{
				return airplane_;
			}

			ControlSurface::Output getControlSurfaceOutput(uint control_index) const
			{
				return airplane_.controls_.at(control_index).getOutput();
			}

			virtual real_T getRestitution() const override
			{
				return params_->getParams().restitution;
			}

			virtual real_T getFriction() const override
			{
				return params_ ->getParams().friction;
			}

			Airplane::Output getAircraftOutput() const
			{
				return airplane_.getOutput();
			}

			virtual ~FixedWingPhysicsBody() = default;

		private: //methods
			void initialize(Kinematics* kinematics, Environment* environment)
			{
				PhysicsBody::initialize(params_->getParams().mass, params_->getParams().inertia, kinematics, environment);

				createAirplane(*params_, airplane_, environment, kinematics);
				
				initSensors(*params_, getKinematics(), getEnvironment());
			}

			void createAirplane(const FixedWingParams& params, Airplane& airplane, const Environment* environment, const Kinematics* kinematics)
			{
				const FixedWingParams::AirplanePose& airplane_pose = params.getParams().airplane_pose;
				airplane = Airplane(airplane_pose.position, airplane_pose.normal, params.getParams().derivatives, params.getParams().prop_derivatives, params.getParams().dimensions, environment, kinematics);
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
			Airplane airplane_;
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
