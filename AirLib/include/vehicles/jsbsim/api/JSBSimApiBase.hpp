#ifndef air_JSBSimControlServer_hpp
#define air_JSBSimControlServer_hpp

#include "common/Common.hpp"
#include "physics/Kinematics.hpp"
#include "physics/Environment.hpp"
#include "api/VehicleApiBase.hpp"
#include "sensors/SensorBase.hpp"
#include "sensors/SensorCollection.hpp"
#include "sensors/SensorFactory.hpp"

#include <memory>

using namespace msr::airlib;
/*
 * Class to interface with the running simulator the processes attached to main should all be found here, could even do
 * the reward function stuff here. 
 */
namespace msr { namespace airlib {
class JSBSimApiBase : public VehicleApiBase {
public:
	struct JSBSimControls
	{
		// based on ardupilot definition [a, e, t, r]
		double aileron;
		double elevator;
		double throttle;
		double rudder;

		JSBSimControls() // default constructor
		{}
		
		JSBSimControls(double aileron_val, double elevator_val, double throttle_val, double rudder_val) :
			aileron(aileron_val), elevator(elevator_val), throttle(throttle_val), rudder(rudder_val)
		{}
		
	};
	
	struct JSBSimState
	{
		CollisionInfo collision;
		Kinematics::State kinematics_estimated;
		uint64_t timestamp;

		JSBSimState()
		{}

		JSBSimState(CollisionInfo& collision_val, Kinematics::State& kinematics_estimate_val,
			uint64_t timestamp_val) : collision(collision_val),
			kinematics_estimated(kinematics_estimate_val), timestamp(timestamp_val)
		{}

		// useful shortcuts
		const Vector3r& getPosition() const
		{
			return kinematics_estimated.pose.position;
		}

		const Quaternionr& getOrientation() const
		{
			return kinematics_estimated.pose.orientation;
		}
	};

public:

	JSBSimApiBase(const AirSimSettings::VehicleSetting* vehicle_setting, 
		std::shared_ptr<SensorFactory> sensor_factory,
		const Kinematics::State& state, const Environment& environment)
	{
		initialize(vehicle_setting, sensor_factory, state, environment);
	}

	virtual void update() override
	{
		VehicleApiBase::update();
		getSensors().update();
	}
	
	void initialize(const AirSimSettings::VehicleSetting* vehicle_setting,
		std::shared_ptr<SensorFactory> sensor_factory,
		const Kinematics::State& state, const Environment& environment)
	{
		sensor_factory_ = sensor_factory;
		sensor_storage_.clear();
		sensors_.clear();
		addSensorsFromSettings(vehicle_setting);
		getSensors().initialize(&state, &environment);
	}

	void addSensorsFromSettings(const AirSimSettings::VehicleSetting* vehicle_setting)
	{
		// use sensors from vehicle settings; if empty list, use default sensors.
		// note that the vehicle settings completely override the default sensor "list";
		// there is no piecemeal add/remove/update per sensor.
		const std::map<std::string, std::unique_ptr<AirSimSettings::SensorSetting>>& sensor_settings
			= vehicle_setting->sensors.size()
		> 0 ? vehicle_setting->sensors : AirSimSettings::AirSimSettings::singleton().sensor_defaults;
		sensor_factory_->createSensorsFromSettings(sensor_settings, sensors_, sensor_storage_);
	}

	void reportState(StateReporter& reporter) override
	{
		getSensors().reportState(reporter);
	}

	JSBSimState getJSBSimState() {
		JSBSimState state;
		state.collision = collision_info_;
		state.kinematics_estimated = kinematics_;
		state.timestamp = clock()->nowNanos();
		return state;
	}

	virtual void getCollisionState(CollisionInfo collision_info)
	{
		collision_info_ = collision_info;
	}

	virtual void getKinematicState(Kinematics::State kinematics)
	{
		kinematics_ = kinematics;
	}

	// sensor helpers
	virtual const SensorCollection& getSensors() const override
	{
		return sensors_;
	}

	SensorCollection& getSensors()
	{
		return sensors_;
	}
	
	virtual void setJSBSimControls(const JSBSimControls& controls) = 0;
	virtual void updateJSBSimState(const JSBSimState& state) = 0;
	virtual const JSBSimState& getJSBSimState() const = 0;
	virtual const JSBSimControls& getJSBSimControls() const = 0;
	
	virtual ~JSBSimApiBase() = default;

	std::shared_ptr<const SensorFactory> sensor_factory_;
	SensorCollection sensors_;  // maintains sensor type indexed collection of sensors
	vector<unique_ptr<SensorBase>> sensor_storage_; // RAII for created sensors

	
protected: // must be implemented methods
	virtual void resetImplementation() override
	{
		// reset sensors last after their ground truth has been reset
		getSensors().reset();
		// add a resetImplementation for JSBSimPhysics Engine
	}
	
private:
	CollisionInfo collision_info_;
	Kinematics::State kinematics_;
	
};

}
}
#endif

