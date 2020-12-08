#pragma once

#include "GameFramework/RotatingMovementComponent.h"
#include "PIPCamera.h"
#include "common/common_utils/UniqueValueMap.hpp"
#include "JSBSimPawnEvents.h"

#include "JSBSimPawn.generated.h"

UCLASS()
class AIRSIM_API AJSBSimPawn : public APawn
{
	GENERATED_BODY()
	
public:

	AJSBSimPawn();
	virtual void BeginPlay() override;
	virtual void Tick(float DeltaSeconds) override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void NotifyHit(class UPrimitiveComponent* MyComp,
		class AActor* Other,
		class UPrimitiveComponent* OtherComp,
		bool bSelfMoved,
		FVector HitLocation,
		FVector HitNormal,
		FVector NormalImpulse,
		const FHitResult& Hit) override;

	//interface with UnrealEngine
	void initializeForBeginPlay();
	const common_utils::UniqueValueMap<std::string, APIPCamera*> getCameras() const;
	AJSBSimPawnEvents* getPawnEvents()
	{
		return &pawn_events_;
	}

private:
	//Unreal Camera Components
	UPROPERTY() APIPCamera* camera_front_left_;
	UPROPERTY() APIPCamera* camera_front_right_;
	UPROPERTY() APIPCamera* camera_front_center_;
	UPROPERTY() APIPCamera* camera_back_center_;
	UPROPERTY() APIPCamera* camera_bottom_center_;

	AJSBSimPawnEvents pawn_events_;
};

