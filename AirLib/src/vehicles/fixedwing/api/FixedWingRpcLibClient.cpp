// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

//in header only mode, control library is not available
#ifndef AIRLIB_HEADER_ONLY
//RPC code requires C++14. If build system like Unreal doesn't support it then use compiled binaries
#ifndef AIRLIB_NO_RPC
//if using Unreal Build system then include precompiled header file first

#include "vehicles/fixedwing/api/FixedWingRpcLibClient.hpp"

#include "common/Common.hpp"
#include <thread>
STRICT_MODE_OFF

#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK

#ifdef nil
#undef nil
#endif // nil

#include "common/common_utils/WindowsApisCommonPre.hpp"
#undef FLOAT
#undef check
#include "rpc/client.h"
//TODO: HACK: UE4 defines macro with stupid names like "check" that conflicts with msgpack library
#ifndef check
#define check(expr) (static_cast<void>((expr)))
#endif
#include "common/common_utils/WindowsApisCommonPost.hpp"

#include "vehicles/fixedwing/api/FixedWingRpcLibAdapators.hpp"

STRICT_MODE_ON
#ifdef _MSC_VER
__pragma(warning( disable : 4239))
#endif


namespace msr { namespace airlib {


typedef msr::airlib_rpclib::FixedWingRpcLibAdapators FixedWingRpcLibAdapators;

struct FixedWingRpcLibClient::impl {
public:
    std::future<RPCLIB_MSGPACK::object_handle> last_future;
};


FixedWingRpcLibClient::FixedWingRpcLibClient(const string&  ip_address, uint16_t port, float timeout_sec)
    : RpcLibClientBase(ip_address, port, timeout_sec)
{
    pimpl_.reset(new impl());
}

FixedWingRpcLibClient::~FixedWingRpcLibClient()
{}

FixedWingRpcLibClient* FixedWingRpcLibClient::takeoffAsync(float timeout_sec, const std::string& vehicle_name)
{
    pimpl_->last_future = static_cast<rpc::client*>(getClient())->async_call("takeoff", timeout_sec, vehicle_name);
    return this;
}
FixedWingRpcLibClient* FixedWingRpcLibClient::landAsync(float timeout_sec, const std::string& vehicle_name)
{
    pimpl_->last_future = static_cast<rpc::client*>(getClient())->async_call("land", timeout_sec, vehicle_name);
    return this;
}
FixedWingRpcLibClient* FixedWingRpcLibClient::goHomeAsync(float timeout_sec, const std::string& vehicle_name)
{
    pimpl_->last_future = static_cast<rpc::client*>(getClient())->async_call("goHome", timeout_sec, vehicle_name);
    return this;
}

FixedWingRpcLibClient* FixedWingRpcLibClient::moveByControlsAsync(float elevator, float aileron, float rudder, float tla, float duration, const std::string& vehicle_name)
{
    pimpl_->last_future = static_cast<rpc::client*>(getClient())->async_call("moveByControlsAsync", elevator, aileron, rudder, tla, duration, vehicle_name);
    return this;
}

FixedWingRpcLibClient* FixedWingRpcLibClient::moveByAttitudeAsync(float roll, float pitch, float yaw, float tla, float duration, const std::string& vehicle_name)
{
    pimpl_->last_future = static_cast<rpc::client*>(getClient())->async_call("moveByAttitudeAsync", roll, pitch, yaw, tla, duration, vehicle_name);
    return this;
}

FixedWingRpcLibClient* FixedWingRpcLibClient::moveByAngleRatesAsync(float roll_rate, float pitch_rate, float yaw_rate, float tla, float duration, const std::string& vehicle_name)
{
    pimpl_->last_future = static_cast<rpc::client*>(getClient())->async_call("moveByAngleRatesAsync", roll_rate, pitch_rate, yaw_rate, tla, duration, vehicle_name);
    return this;
}

FixedWingRpcLibClient* FixedWingRpcLibClient::moveByVelocityAsync(float vx, float vy, float vz, float duration, const std::string& vehicle_name)
{
    pimpl_->last_future = static_cast<rpc::client*>(getClient())->async_call("moveByVelocityAsync", vx, vy, vz, duration, vehicle_name);
    return this;
}

FixedWingRpcLibClient* FixedWingRpcLibClient::moveOnPathAsync(const vector<Vector3r>& path, float velocity, float timeout_sec, float lookahead, float adaptive_lookahead, const std::string& vehicle_name)
{
    vector<FixedWingRpcLibAdapators::Vector3r> conv_path;
    FixedWingRpcLibAdapators::from(path, conv_path);
    pimpl_->last_future = static_cast<rpc::client*>(getClient())->async_call("moveOnPathAsync", conv_path, velocity, timeout_sec, lookahead, adaptive_lookahead, vehicle_name);
    return this;
}

FixedWingRpcLibClient* FixedWingRpcLibClient::moveToPositionAsync(float x, float y, float z, float velocity, float timeout_sec, float lookahead, float adaptive_lookahead, const std::string& vehicle_name)
{
    pimpl_->last_future = static_cast<rpc::client*>(getClient())->async_call("moveToPositionAsync", x, y, z, velocity, timeout_sec, lookahead, adaptive_lookahead, vehicle_name);
    return this;
}

FixedWingRpcLibClient* FixedWingRpcLibClient::moveToAltAsync(float z, float velocity, float timeout_sec, float lookahead, float adaptive_lookahead, const std::string& vehicle_name)
{
    pimpl_->last_future = static_cast<rpc::client*>(getClient())->async_call("moveToAltAsync", z, velocity, timeout_sec, lookahead, adaptive_lookahead, vehicle_name);
    return this;
}
FixedWingRpcLibClient* FixedWingRpcLibClient::moveByManualAsync(float vx_max, float vy_max, float z_min, float duration, const std::string& vehicle_name)
{
    pimpl_->last_future = static_cast<rpc::client*>(getClient())->async_call("moveToAltAsync", vx_max, vy_max, z_min, duration, vehicle_name);
    return this;
}

// FixedWingRpcLibClient* FixedWingRpcLibClient::rotateToYawAsync(float yaw, float timeout_sec, float margin, const std::string& vehicle_name)
// {
//     pimpl_->last_future = static_cast<rpc::client*>(getClient())->async_call("rotateToYaw", yaw, timeout_sec, margin, vehicle_name);
//     return this;
// }

// FixedWingRpcLibClient* FixedWingRpcLibClient::rotateByYawRateAsync(float yaw_rate, float duration, const std::string& vehicle_name)
// {
//     pimpl_->last_future = static_cast<rpc::client*>(getClient())->async_call("rotateByYawRate", yaw_rate, duration, vehicle_name);
//     return this;
// }

// FixedWingRpcLibClient* FixedWingRpcLibClient::hoverAsync(const std::string& vehicle_name)
// {
//     pimpl_->last_future = static_cast<rpc::client*>(getClient())->async_call("hover", vehicle_name);
//     return this;
// }

void FixedWingRpcLibClient::setAngleLevelControllerGains(const vector<float>& kp, const vector<float>& ki, const vector<float>& kd, const std::string& vehicle_name)
{
    static_cast<rpc::client*>(getClient())->call("setAngleLevelControllerGains", kp, ki, kd, vehicle_name);
}

void FixedWingRpcLibClient::setAngleRateControllerGains(const vector<float>& kp, const vector<float>& ki, const vector<float>& kd, const std::string& vehicle_name)
{
    static_cast<rpc::client*>(getClient())->call("setAngleRateControllerGains", kp, ki, kd, vehicle_name);
}

void FixedWingRpcLibClient::setVelocityControllerGains(const vector<float>& kp, const vector<float>& ki, const vector<float>& kd, const std::string& vehicle_name)
{
    static_cast<rpc::client*>(getClient())->call("setVelocityControllerGains", kp, ki, kd, vehicle_name);
}

void FixedWingRpcLibClient::setPositionControllerGains(const vector<float>& kp, const vector<float>& ki, const vector<float>& kd, const std::string& vehicle_name)
{
    static_cast<rpc::client*>(getClient())->call("setPositionControllerGains", kp, ki, kd, vehicle_name);
}

bool FixedWingRpcLibClient::setSafety(SafetyEval::SafetyViolationType enable_reasons, float obs_clearance, SafetyEval::ObsAvoidanceStrategy obs_startegy,
    float obs_avoidance_vel, const Vector3r& origin, float xy_length, float max_z, float min_z, const std::string& vehicle_name)
{
    return static_cast<rpc::client*>(getClient())->call("setSafety", static_cast<uint>(enable_reasons), obs_clearance, obs_startegy,
        obs_avoidance_vel, FixedWingRpcLibAdapators::Vector3r(origin), xy_length, max_z, min_z, vehicle_name).as<bool>();
}

//status getters
FixedWingState FixedWingRpcLibClient::getFixedWingState(const std::string& vehicle_name)
{
    return static_cast<rpc::client*>(getClient())->call("getFixedWingState", vehicle_name).
        as<FixedWingRpcLibAdapators::FixedWingState>().to();
}

void FixedWingRpcLibClient::moveByRC(const RCData& rc_data, const std::string& vehicle_name)
{
    static_cast<rpc::client*>(getClient())->call("moveByRC", FixedWingRpcLibAdapators::RCData(rc_data), vehicle_name);
}

//return value of last task. It should be true if task completed without
//cancellation or timeout
FixedWingRpcLibClient* FixedWingRpcLibClient::waitOnLastTask(bool* task_result, float timeout_sec)
{
    bool result;
    if (std::isnan(timeout_sec) || timeout_sec == Utils::max<float>())
        result = pimpl_->last_future.get().as<bool>();
    else {
        auto future_status = pimpl_->last_future.wait_for(std::chrono::duration<double>(timeout_sec));
        if (future_status == std::future_status::ready)
            result = pimpl_->last_future.get().as<bool>();
        else
            result = false;
    }

    if (task_result)
        *task_result = result;

    return this;
}

}} //namespace

#endif
#endif
