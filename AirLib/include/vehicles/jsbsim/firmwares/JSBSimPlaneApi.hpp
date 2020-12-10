#ifndef msr_airlib_JSBSimPlaneApi_hpp
#define msr_airlib_JSBSimPlaneApi_hpp

#include "vehicles/jsbsim/api/JSBSimApiBase.hpp"
#include "vehicles/JSBSim/JSBSimPawnSimApi.h"
#include "JSBSimPawnSimApi.h"

namespace msr { namespace airlib {

class JSBSimPlaneApi : public JSBSimApiBase
{
public:
	JSBSimPlaneApi(const AirSimSettings::VehicleSetting* vehicle_setting, std::shared_ptr<SensorFactory> sensor_factory,
		const Kinematics::State& state, const Environment& environment) :
	JSBSimApiBase(vehicle_setting, sensor_factory, state, environment)
	{/*insert constructor code*/}

	~JSBSimPlaneApi()
	{/*insert destructor code*/}

protected:
	virtual void resetImplementation() override
	{
		JSBSimApiBase::resetImplementation();
		// reset state
	}

public:
	virtual void update() override
	{
		JSBSimApiBase::update();
		last_jsbsim_state_ = getJSBSimState();
	}

	virtual const SensorCollection& getSensors() const override
	{
		return JSBSimApiBase::getSensors();
	}

	// VehicleApiBase Implementation
	virtual void enableApiControl(bool is_enabled) override
	{
		if (api_control_enabled_ != is_enabled)
		{
			last_controls_ = JSBSimControls();
			api_control_enabled_ = is_enabled;
		}
	}

	virtual bool isApiControlEnabled() const override
	{
		return api_control_enabled_;
	}

	virtual bool armDisarm(bool arm) override
	{
		unused(arm);
		return true;
	}

public:
	virtual void setJSBSimControls(const JSBSimControls& controls) override
	{
		last_controls_ = controls;
	}

	virtual void updateJSBSimState(const JSBSimState& jsbsim_state) override
	{
		last_jsbsim_state_ = jsbsim_state;
	}

	virtual JSBSimState& getJSBSimState() const
	{
		// return last_jsbsim_state_;
	}

	virtual const JSBSimControls& getJSBSimControls() const override
	{
		return last_controls_;
	}

	virtual GeoPoint getHomeGeoPoint() const override
	{
		
	};

private:
	bool api_control_enabled_ = false;
	JSBSimControls last_controls_;
	JSBSimState last_jsbsim_state_;
	
};	
}
}
#endif