#include "FixedWingPawn.h"
#include "Components/StaticMeshComponent.h"
#include "AirBlueprintLib.h"
#include "common/CommonStructs.hpp"
#include "common/Common.hpp"

AFixedWingPawn::AFixedWingPawn()
{
    pawn_events_.getElevatorSignal().connect_member(this, &AFixedWingPawn::setElevatorDeflection);
    pawn_events_.getAileronSignal().connect_member(this, &AFixedWingPawn::setAileronDeflection);
    pawn_events_.getRudderSignal().connect_member(this, &AFixedWingPawn::setRudderDeflection);
}

void AFixedWingPawn::BeginPlay()
{
    Super::BeginPlay();

    elevator_position_ = UAirBlueprintLib::GetActorComponent<URotatingMovementComponent>(this, TEXT("Elevator-Position"));
    aileron_position_ = UAirBlueprintLib::GetActorComponent<URotatingMovementComponent>(this, TEXT("Aileron-Position"));
    rudder_position_ = UAirBlueprintLib::GetActorComponent<URotatingMovementComponent>(this, TEXT("Rudder-Position"));
}

void AFixedWingPawn::initializeForBeginPlay()
{
    //get references of existing camera
    camera_front_right_ = Cast<APIPCamera>(
        (UAirBlueprintLib::GetActorComponent<UChildActorComponent>(this, TEXT("FrontRightCamera")))->GetChildActor());
    camera_front_left_ = Cast<APIPCamera>(
        (UAirBlueprintLib::GetActorComponent<UChildActorComponent>(this, TEXT("FrontLeftCamera")))->GetChildActor());
    camera_front_center_ = Cast<APIPCamera>(
        (UAirBlueprintLib::GetActorComponent<UChildActorComponent>(this, TEXT("FrontCenterCamera")))->GetChildActor());
    camera_back_center_ = Cast<APIPCamera>(
        (UAirBlueprintLib::GetActorComponent<UChildActorComponent>(this, TEXT("BackCenterCamera")))->GetChildActor());
    camera_bottom_center_ = Cast<APIPCamera>(
        (UAirBlueprintLib::GetActorComponent<UChildActorComponent>(this, TEXT("BottomCenterCamera")))->GetChildActor());
}

void AFixedWingPawn::Tick(float DeltaSeconds)
{
    Super::Tick(DeltaSeconds);
    pawn_events_.getPawnTickSignal().emit(DeltaSeconds);
}


void AFixedWingPawn::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    camera_front_right_ = nullptr;
    camera_front_left_ = nullptr;
    camera_front_center_ = nullptr;
    camera_back_center_ = nullptr;
    camera_bottom_center_ = nullptr;

    Super::EndPlay(EndPlayReason);
}

const common_utils::UniqueValueMap<std::string, APIPCamera*> AFixedWingPawn::getCameras() const
{
    common_utils::UniqueValueMap<std::string, APIPCamera*> cameras;
    cameras.insert_or_assign("front_center", camera_front_center_);
    cameras.insert_or_assign("front_right", camera_front_right_);
    cameras.insert_or_assign("front_left", camera_front_left_);
    cameras.insert_or_assign("bottom_center", camera_bottom_center_);
    cameras.insert_or_assign("back_center", camera_back_center_);

    cameras.insert_or_assign("0", camera_front_center_);
    cameras.insert_or_assign("1", camera_front_right_);
    cameras.insert_or_assign("2", camera_front_left_);
    cameras.insert_or_assign("3", camera_bottom_center_);
    cameras.insert_or_assign("4", camera_back_center_);

    cameras.insert_or_assign("", camera_front_center_);
    cameras.insert_or_assign("fpv", camera_front_center_);

    return cameras;
}

void AFixedWingPawn::NotifyHit(class UPrimitiveComponent* MyComp, class AActor* Other, class UPrimitiveComponent* OtherComp, bool bSelfMoved, FVector HitLocation,
    FVector HitNormal, FVector NormalImpulse, const FHitResult& Hit)
{
    pawn_events_.getCollisionSignal().emit(MyComp, Other, OtherComp, bSelfMoved, HitLocation,
        HitNormal, NormalImpulse, Hit);
}

//TODO: Need to change the funtion decleration so that it is just a const not a templated vector
void AFixedWingPawn::setElevatorDeflection(const std::vector<FixedWingPawnEvents::FixedWingElevatorInfo>& elevator_info)
{
    auto comp = elevator_position_;
	if (comp != nullptr)
	{
		// not sure how to do set the RotationRate.Yaw = elevator_info.at(0).elevator_speed; if mod(angle) < mod(command) else RotationRate.Yaw = 0
		if(elevator_info.at(0).elevator_deflection == elevator_info.at(0).elevator_command)
		{
            comp->RotationRate.Yaw = 0;
		}
        else
        {
            comp->RotationRate.Yaw = elevator_info.at(0).elevator_speed; // should this be pitch, can probably tilt local control surface
        }

	}
}

void AFixedWingPawn::setAileronDeflection(const std::vector<FixedWingPawnEvents::FixedWingAileronInfo>& aileron_info)
{
    auto comp = aileron_position_;
    if (comp != nullptr)
    {
    	if(aileron_info.at(0).aileron_deflection == aileron_info.at(0).aileron_command)
    	{
            comp->RotationRate.Yaw = 0;
    	}
        else
        {
            comp->RotationRate.Yaw = aileron_info.at(0).aileron_speed;// should this be pitch? and do we need 2 of these components this may already be done to some extent?

        }
    }
}

void AFixedWingPawn::setRudderDeflection(const std::vector<FixedWingPawnEvents::FixedWingRudderInfo>& rudder_info)
{
    auto comp = rudder_position_;
    if (comp != nullptr)
    {
    	if(rudder_info.at(0).rudder_command == rudder_info.at(0).rudder_deflection)
    	{
            comp->RotationRate.Yaw = 0;
    	}
        else
        {
            comp->RotationRate.Yaw = rudder_info.at(0).rudder_speed;
        }
    }
}
