#ifndef air_JSBSimRpcLibClient_hpp
#define air_JSBSimRpcLibClient_hpp

#include "common/Common.hpp"
#include "vehicles/JSBSim/api/JSBSimApiBase.hpp"
#include "api/RpcLibClientBase.hpp"


namespace msr { namespace airlib {

class JSBSimRpcLibClient : RpcLibClientBase {
public:
	JSBSimRpcLibClient(const string& ip_address = "localhost", uint16_t port = RpcLibPort, float timeout_sec = 60);

	/* Add a method for
	 * - set JSBSim Controls
	 * - get JSBSim State
	 * - get JSBSim Controls
	 * look @ CarRpcLibClient for guidance
	 */
	virtual ~JSBSimRpcLibClient(); // required for pimpl	
};
}
}
#endif