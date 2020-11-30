#ifndef msr_airlib_jsbsimphysicsbody_hpp
#define msr_airlib_jsbsimphysicsbody_hpp

#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "physics/JSBSimPhysicsEngine.hpp"
#include <vector>

namespace msr { namespace airlib {

class JSBSimPhysicsBody : JSBSimPhysicsEngine {
public:

	// Constructor
	JSBSimPhysicsBody()
	{
		initialize();
	}

	//*** Start: UpdateState implementation ***//
	virtual void resetImplementation() override
	{
		//reset kinematics and environment
		/* call JSBSim reset code*/
	}

	virtual void update() override
	{
		// update from JSBSim
		JSBSimPhysicsEngine::update();
	}

	virtual void reportState(StateReporter& reporter) override
	{
		JSBSimPhysicsEngine::reportState(reporter);
		// add in any things here you want to report
		
	}
	//** End: UpdatableState implementation **//

	

};


}
}
#endif