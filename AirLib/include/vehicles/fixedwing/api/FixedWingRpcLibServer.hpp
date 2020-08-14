// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_FixedWingRpcLibServer_hpp
#define air_FixedWingRpcLibServer_hpp

#include "common/Common.hpp"
#include <functional>
#include "api/RpcLibServerBase.hpp"
#include "vehicles/fixedwing/api/FixedWingApiBase.hpp"


namespace msr { namespace airlib {

class FixedWingRpcLibServer : public RpcLibServerBase {
public:
    FixedWingRpcLibServer(ApiProvider* api_provider, string server_address, uint16_t port = RpcLibPort);
    virtual ~FixedWingRpcLibServer();

protected:
    virtual FixedWingApiBase* getVehicleApi(const std::string& vehicle_name) override
    {
        return static_cast<FixedWingApiBase*>(RpcLibServerBase::getVehicleApi(vehicle_name));
    }

};

}} //namespace
#endif
