// Copyright (c) Microsoft Corporation. All rights reserved.FixedWingRpcLibClient
// Licensed under the MIT License.

#ifndef air_FixedWingRpcLibClient_hpp
#define air_FixedWingRpcLibClient_hpp

#include "common/Common.hpp"
#include <functional>
#include "common/CommonStructs.hpp"
#include "common/ImageCaptureBase.hpp"
#include "vehicles/fixedwing/api/FixedWingApiBase.hpp"
#include "api/RpcLibClientBase.hpp"
#include "vehicles/fixedwing/api/FixedWingCommon.hpp"


namespace msr { namespace airlib {

class FixedWingRpcLibClient : public RpcLibClientBase {
public:
    FixedWingRpcLibClient(const string& ip_address = "localhost", uint16_t port = RpcLibPort, float timeout_sec = 60);

    FixedWingRpcLibClient* takeoffAsync(float timeout_sec = 20, const std::string& vehicle_name = "");
    FixedWingRpcLibClient* landAsync(float timeout_sec = 60, const std::string& vehicle_name = "");
    FixedWingRpcLibClient* goHomeAsync(float timeout_sec = Utils::max<float>(), const std::string& vehicle_name = "");

    FixedWingRpcLibClient* moveByControlsAsync(float elevator, float aileron, float rudder, float tla, float duration, const std::string& vehicle_name = "");
    FixedWingRpcLibClient* moveByAttitudeAsync(float roll, float pitch, float yaw, float tla, float duration, const std::string& vehicle_name = "");
    FixedWingRpcLibClient* moveByAngleRatesAsync(float roll_rate, float pitch_rate, float yaw_rate, float tla, float duration, const std::string& vehicle_name = "");
    FixedWingRpcLibClient* moveByVelocityAsync(float vx, float vy, float vz, float duration, const std::string& vehicle_name = "");
    FixedWingRpcLibClient* moveOnPathAsync(const vector<Vector3r>& path, float velocity, float timeout_sec, float lookahead, float adaptive_lookahead, const std::string& vehicle_name = "");
    FixedWingRpcLibClient* moveToPositionAsync(float x, float y, float z, float velocity, float timeout_sec, float lookahead, float adaptive_lookahead, const std::string& vehicle_name = "");
    FixedWingRpcLibClient* moveToAltAsync(float z, float velocity, float timeout_sec, float lookahead, float adaptive_lookahead, const std::string& vehicle_name = "");
    FixedWingRpcLibClient* moveByManualAsync(float vx_max, float vy_max, float z_min, float duration, const std::string& vehicle_name = "");

    void setAngleLevelControllerGains(const vector<float>& kp, const vector<float>& ki, const vector<float>& kd, const std::string& vehicle_name="");
    void setAngleRateControllerGains(const vector<float>& kp, const vector<float>& ki, const vector<float>& kd, const std::string& vehicle_name="");
    void setVelocityControllerGains(const vector<float>& kp, const vector<float>& ki, const vector<float>& kd, const std::string& vehicle_name="");
    void setPositionControllerGains(const vector<float>& kp, const vector<float>& ki, const vector<float>& kd, const std::string& vehicle_name="");
    void moveByRC(const RCData& rc_data, const std::string& vehicle_name = "");

    FixedWingState getFixedWingState(const std::string& vehicle_name = "");

    bool setSafety(SafetyEval::SafetyViolationType enable_reasons, float obs_clearance, SafetyEval::ObsAvoidanceStrategy obs_startegy,
        float obs_avoidance_vel, const Vector3r& origin, float xy_length, float max_z, float min_z, const std::string& vehicle_name = "");

    virtual FixedWingRpcLibClient* waitOnLastTask(bool* task_result = nullptr, float timeout_sec = Utils::nan<float>()) override;

    virtual ~FixedWingRpcLibClient();    //required for pimpl

private:
    struct impl;
    std::unique_ptr<impl> pimpl_;
};

}} //namespace
#endif
