#ifndef air_JSBSimRpcLibServer_hpp
#define air_JSBSimRpcLibServer_hpp

#ifndef AIRLIB_NO_RPC

#include "common/Common.hpp"
#include <functional>
#include "api/RpcLibServerBase.hpp"
#include "vehicles/jsbsim/api/JSBSimApiBase.hpp"

namespace msr { namespace airlib {

class JSBSimRpcLibServer : public RpcLibServerBase
{
public:
	JSBSimRpcLibServer(ApiProvider* api_provider, string server_address, uint16_t port = RpcLibPort);
	virtual ~JSBSimRpcLibServer();

protected:
	virtual JSBSimApiBase* getVehicleApi(const std::string& vehicle_name) override
	{
		return static_cast<JSBSimApiBase*>(RpcLibServerBase::getVehicleApi(vehicle_name));
	}	
};
#endif
}
}
#endif

