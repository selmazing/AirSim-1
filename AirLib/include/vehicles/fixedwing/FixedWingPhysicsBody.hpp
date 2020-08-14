#ifndef msr_airlib_fixedwingphysicsbody_hpp
#define msr_airlib_fixedwingphysicsbody_hpp

#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "api/VehicleApiBase.hpp"
#include "api/VehicleSimApiBase.hpp"
#include <vector>
#include "physics/PhysicsBody.hpp"
#include "FixedWingParams.hpp"
#include "DynamicForces.hpp"


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

				//report controls usage (x3), throttle not included
					reporter.startHeading("", 1);
					reporter.writeValue("Elevator", 0);
					reporter.writeValue("Aileron", 1);
					reporter.writeValue("Rudder", 2);
					reporter.endHeading(false, 1);
					output_.reportState(reporter);
			}
			//*** End: UpdatableState implementation ***//

			//Physics engine calls this method to set next kinematics
			virtual void updateKinematics(const Kinematics::State& kinematics) override
			{
				PhysicsBody::updateKinematics(kinematics);

				updateSensors(*params_, getKinematics(), getEnvironment());

				//update controller which will update actuator control signal
				vehicle_api_->update();

				/*setting actuator index to name
				elevator_.index = 0;
				aileron_.index = 1;
				rudder_.index = 2;
				Needs its own class don't think its worth it?*/

				//transfer new input values from controller to control_surface
				output_.setElevatorSignal(vehicle_api_->getActuation(0));
				output_.setAileronSignal(vehicle_api_->getActuation(1));
				output_.setRudderSignal(vehicle_api_->getActuation(2));
			}

			//sensor getter
			const SensorCollection& getSensors() const
			{
				return params_->getSensors();
			}

			//physics body interface

			//TODO: Add the physics getters, might need to use the wrench object --> convert aero-forces to translation?
			DynamicForces::Output getDynamicForcesOutput() const
			{
				return output_.getOutput();
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

		private: //methods
			void initialize(Kinematics* kinematics, Environment* environment)
			{
				PhysicsBody::initialize(params_->getParams().mass, params_->getParams().inertia, kinematics, environment);

				//createControls(*params_, controls_, environment);

				initSensors(*params_, getKinematics(), getEnvironment());
			}

			/*static void createAircraft(const FixedWingParams& params, vector<DynamicForces>& aircraft, const Environment* environment)
			{
				aircraft.clear();
				const FixedWingParmas::f
			} */

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

			//let us be the owner of the controls object
			//vector<DynamicForces> elevator_, aileron_, rudder_; Don't think this is needed anymore
			DynamicForces output_;
			std::unique_ptr<Environment> environment_;
			VehicleApiBase* vehicle_api_;
		};
	}
}

#endif
