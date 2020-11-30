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

	struct JSBSimState {
		/* Declare State variables in struct */

		// add MSGPACK_DEFINE_MAP
		
		JSBSimState()
		{}

		// add a constructor with an input from JSBSimApiBase or JSBSimApiCommon (common may be better way to go)


	};
};	
}
}
#endif