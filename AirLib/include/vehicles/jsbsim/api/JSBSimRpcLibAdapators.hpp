#ifndef air_JSBSimRpcLibAdaptors_hpp
#define air_JSBSimRpcLibAdaptors_hpp

#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "api/RpcLibAdapatorsBase.hpp"
#include "vehicles/JSBSim/api/JSBSimApiBase.hpp"

#include "common/common_utils/WindowsApisCommonPre.hpp"
#include "rpc/msgpack.hpp"
#include "common/common_utils/WindowsApisCommonPost.hpp"

namespace msr { namespace airlib_rpclib {

class JSBSimRpcLibAdaptors : public RpcLibAdapatorsBase
{
public:
	struct JSBSimControls
	{
		double aileron;
		double elevator;
		double throttle;
		double rudder;

		MSGPACK_DEFINE_MAP(aileron, elevator, throttle, rudder);

		JSBSimControls()
		{}

		JSBSimControls(const msr::airlib::JSBSimApiBase::JSBSimControls& s)
		{
			aileron = s.aileron;
			elevator = s.elevator;
			throttle = s.throttle;
			rudder = s.rudder;
		}

		msr::airlib::JSBSimApiBase::JSBSimControls to() const
		{
			return msr::airlib::JSBSimApiBase::JSBSimControls(aileron, elevator, throttle, rudder);
		}
	};

	struct JSBSimState {
		/* Declare State variables in struct */
		CollisionInfo collision;
		KinematicsState kinematics_estimated;
		uint64_t timestamp;
		std::vector<std::string> controller_messages;
		// add MSGPACK_DEFINE_MAP

		MSGPACK_DEFINE_MAP(collision, kinematics_estimated, timestamp);
		
		JSBSimState()
		{}

		// add a constructor with an input from JSBSimApiBase
		JSBSimState(const msr::airlib::JSBSimApiBase::JSBSimState& s)
		{
			collision = s.collision;
			kinematics_estimated = s.kinematics_estimated;
			timestamp = s.timestamp;
		}

		msr::airlib::JSBSimApiBase::JSBSimState to() const
		{
			return msr::airlib::JSBSimApiBase::JSBSimState(collision.to(),
				kinematics_estimated.to(), timestamp);
		}
		
	};
};	
}
}
#endif