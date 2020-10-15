// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_vehicles_PlaneFlightXParams_hpp
#define msr_airlib_vehicles_PlaneFlightXParams_hpp

#include "vehicles/fixedwing/firmwares/plane_flight/PlaneFlightApi.hpp"
#include "vehicles/fixedwing/FixedWingParams.hpp"
#include "common/AirSimSettings.hpp"
#include "sensors/SensorFactory.hpp"


namespace msr { namespace airlib {

class PlaneFlightXParams : public FixedWingParams {
public:
    PlaneFlightXParams(const AirSimSettings::VehicleSetting* vehicle_setting, std::shared_ptr<const SensorFactory> sensor_factory)
        : vehicle_setting_(vehicle_setting), sensor_factory_(sensor_factory)
    {
    }

    virtual ~PlaneFlightXParams() = default;

    virtual std::unique_ptr<FixedWingApiBase> createFixedWingApi() override
    {
        return std::unique_ptr<FixedWingApiBase>(new PlaneFlightApi(this, vehicle_setting_));
    }

protected:
    virtual void setupParams() override
    {
        auto& params = getParams();

        // Use connection_info_.model for the model name, see Px4MultiRotorParams for example

        // Only Generic for now
        setupFrameSkywalkerX8(params);
    }

    virtual const SensorFactory* getSensorFactory() const override
    {
        return sensor_factory_.get();
    }

private:
    const AirSimSettings::VehicleSetting* vehicle_setting_; //store as pointer because of derived classes
    std::shared_ptr<const SensorFactory> sensor_factory_;
};

}} //namespace
#endif
