

#include "SimModeWorldJSBSim.h"
#include "UObject/ConstructorHelpers.h"
#include "Logging/MessageLog.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"

#include "AirBlueprintLib.h"
#include "vehicles/jsbsim/api/JSBSimApiBase.hpp"
#include "JSBSimPawnSimApi.h"
#include "common/ClockFactory.hpp"
#include <memory>
#include "vehicles/jsbsim/api/JSBSimRpcLibServer.hpp"
// JSBSimRpcLibServer.hpp

void ASimModeWorldJSBSim::BeginPlay()
{
	Super::BeginPlay();

	// Let base class setup the physics world
	initializeForPlay();
}

void ASimModeWorldJSBSim::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	// Stop physics thead before we dismantle the simulator
	stopAsyncUpdator();
	Super::EndPlay(EndPlayReason);
}

void ASimModeWorldJSBSim::setupClockSpeed()
{
	typedef msr::airlib::ClockFactory ClockFactory;

	float clock_speed = getSettings().clock_speed;

	// Setup clock in ClockFactory
	std::string clock_type = getSettings().clock_type;


	/* this clock is scalable for the purpose of the physical update frame-rate being different from the
	 * graphic update rate
	 */
	if (clock_type == "ScalableClock")
	{
		// Scalable clock returns interval same as wall clock but multiplied by a scale factor
		ClockFactory::get(std::make_shared<msr::airlib::ScalableClock>(clock_speed == 1 ? 1 : 1 / clock_speed));
	}
	else if (clock_type == "SteppableClock")
	{
		if (clock_speed >= 1)
		{
			ClockFactory::get(std::make_shared<msr::airlib::SteppableClock>(
				static_cast<msr::airlib::TTimeDelta>(getPhysicsLoopPeriod() * 1E-9)));

			setPhysicsLoopPeriod(getPhysicsLoopPeriod() / static_cast<long long>(clock_speed));
		}
		else
		{
			ClockFactory::get(std::make_shared<msr::airlib::SteppableClock>(
				static_cast<msr::airlib::TTimeDelta>(getPhysicsLoopPeriod() * 1E-9 * clock_speed)));

		}
	}
	else
		throw std::invalid_argument(common_utils::Utils::stringf("clock_type %s is not recognized", clock_type.c_str()));
}

//-------------------------------- overrides -----------------------------------------------//

std::unique_ptr<msr::airlib::ApiServerBase> ASimModeWorldJSBSim::createApiServer() const
{
#ifdef AIRLIB_NO_RPC
	return ASimModeBase::createApiServer();
#else
	return std::unique_ptr<msr::airlib::ApiServerBase>(new msr::airlib::JSBSimRpcLibServer(
		getApiProvider(), getSettings().api_server_address, getSettings().api_port));
#endif
}

void ASimModeWorldJSBSim::getExistingVehiclePawns(TArray<AActor*>& pawns) const
{
	UAirBlueprintLib::FindAllActor<TVehiclePawn>(this, pawns);
}

bool ASimModeWorldJSBSim::isVehicleTypeSupported(const std::string& vehicle_type) const
{
	/* Implement code to return a vehicle type form AirSimSettings, will probably inherit from FGExec by calling a script*/
	return (vehicle_type == AirSimSettings::kVehicleTypePlaneFlight);
}

std::string ASimModeWorldJSBSim::getVehiclePawnPathName(const AirSimSettings::VehicleSetting& vehicle_setting) const
{
	/* get the path to the blueprint used for the aircraft in UE4, can probably use the same as was used in FixedWing initially*/
	std::string pawn_path = vehicle_setting.pawn_path; // can set a seperate pawn path for a different aircraft
	if (pawn_path == "")
		pawn_path = "DefaultFixedWing";
	
	return pawn_path;
}

PawnEvents* ASimModeWorldJSBSim::getVehiclePawnEvents(APawn* pawn) const
{
	return static_cast<TVehiclePawn*>(pawn)->getPawnEvents();
}

const common_utils::UniqueValueMap<std::string, APIPCamera*> ASimModeWorldJSBSim::getVehiclePawnCameras(APawn* pawn) const
{
	return (static_cast<const TVehiclePawn*>(pawn))->getCameras();
}

void ASimModeWorldJSBSim::initializeVehiclePawn(APawn* pawn)
{
	static_cast<TVehiclePawn*>(pawn)->initializeForBeginPlay();
}

// calls the initialise method from from the JSBSimPawnSimApi
std::unique_ptr<PawnSimApi> ASimModeWorldJSBSim::createVehicleSimApi(
	const PawnSimApi::Params& pawn_sim_api_params) const
{
	auto vehicle_sim_api = std::unique_ptr<PawnSimApi>(new JSBSimPawnSimApi(pawn_sim_api_params));
	vehicle_sim_api->initialize();
	return vehicle_sim_api;
}
msr::airlib::VehicleApiBase* ASimModeWorldJSBSim::getVehicleApi(const PawnSimApi::Params& pawn_sim_api_params, const PawnSimApi* sim_api) const
{
	const auto jsbsim_sim_api = static_cast<const JSBSimPawnSimApi*>(sim_api);
	return jsbsim_sim_api->getVehicleApi();
}



