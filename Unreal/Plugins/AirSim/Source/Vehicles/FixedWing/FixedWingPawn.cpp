#include "FixedWingPawn.h"
#include "Components/StaticMeshComponent.h"
#include "AirBlueprintLib.h"
#include "common/CommonStructs.hpp"
#include "common/Common.hpp"

AFixedWingPawn::AFixedWingPawn()
{
    pawn_events_.getControlSignal().connect_member(this, &AFixedWingPawn::setControlDeflection);
}

void AFixedWingPawn::BeginPlay()
{
    Super::BeginPlay();

    for (auto i = 0; i < control_count; ++i)
    {
        control_position_[i] = UAirBlueprintLib::GetActorComponent<URotatingMovementComponent>(this, TEXT("Control-Position"));
    }
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

void AFixedWingPawn::setControlDeflection(const std::vector<FixedWingPawnEvents::FixedWingControlInfo>& control_info)
{
	for (auto control_index = 0; control_index < control_info.size(); ++control_index)
	{
        auto comp = control_position_[control_index];
		if (comp != nullptr)
		{
			if(control_info.at(control_index).control_deflection == control_info.at(control_index).control_command)
			{
                comp->RotationRate.Yaw = 0;
			}
            else 
            {
                comp->RotationRate.Yaw = control_info.at(control_index).control_speed;
            }
		}   
	}
}

