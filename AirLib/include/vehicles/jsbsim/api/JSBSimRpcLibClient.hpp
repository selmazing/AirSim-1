#ifndef air_JSBSimRpcLibClient_hpp
#define air_JSBSimRpcLibClient_hpp

#include "common/Common.hpp"
#include "vehicles/jsbsim/api/JSBSimApiBase.hpp"
#include "api/RpcLibClientBase.hpp"

// This is really behaving as a header file
namespace msr { namespace airlib {
/*Deriving from RpcLibClientBase provides access to many camera and sensor APIs
 *The key methods to interact with the API should be provided here and these are inherited
 *from JSBSimApiBase
 */
class JSBSimRpcLibClient : RpcLibClientBase {
public:
	JSBSimRpcLibClient(const string& ip_address = "localhost", uint16_t port = RpcLibPort, float timeout_sec = 60);

	/* Add a method for
	 * - set JSBSim Controls
	 * - get JSBSim State
	 * - get JSBSim Controls
	 * look @ MultiRotorRpcLibClient for guidance
	 * - this is interfaced with the python API if settings can be applied the vehicle can be controlled remotely
	 */

	void setJSBSimControls(const JSBSimApiBase::JSBSimControls& controls, const std::string& vehicle_name = "");
	JSBSimApiBase::JSBSimState getJSBSimState(const std::string& vehicle_name = ""); // doesnt do anything?
	JSBSimApiBase::JSBSimControls getJSBSimControls(const std::string& vehicle_name = "");
	virtual ~JSBSimRpcLibClient(); // required for pimpl
private:
	struct impl;
	std::unique_ptr<impl> pimpl_; // hangover from multi-rotor not sure if this is necessary as unused in body
};
}
}
#endif