// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "Camera/CameraComponent.h"
#include "Components/TextRenderComponent.h"
#include "MotionControllerComponent.h"
#include "UObject/ConstructorHelpers.h"
#include "XRMotionControllerBase.h"
#include "Components/StaticMeshComponent.h"
#include "Materials/Material.h"
#include "Kismet/KismetMathLibrary.h"
#include "Kismet/KismetSystemLibrary.h"
#include "NavMesh/RecastNavMesh.h"
#include "Misc/FileHelper.h"
#include "Kismet/GameplayStatics.h"
#include "MotionObject.h"
#include "MotionObjectActor.h"
#include "MotionControllerComponent.h"
//#include "OculusFunctionLibrary.h"
#include "MotionObject.h"
#include "MyPawn.generated.h"

UCLASS()
class UNREALSIM_API AMyPawn : public APawn
{
	GENERATED_BODY()

public:
	// Sets default values for this pawn's properties
	AMyPawn();

	// Called every frame
	virtual void Tick(float DeltaTime) override;

	// Called to bind functionality to input
	virtual void SetupPlayerInputComponent(class UInputComponent* PlayerInputComponent) override;

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = __hide)
		USceneComponent* VRTrackingCenter;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = __hide)
		UCameraComponent* Head;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = __hide)
		UTextRenderComponent* outputText;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = __hide)
		UMotionControllerComponent* LeftController;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = __hide)
		UMotionControllerComponent* RightController;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = __hide)
		UStaticMeshComponent* raygun;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = __hide)
		UStaticMeshComponent* LeftCone;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = __hide)
		UStaticMeshComponent* RightMesh;

	//UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = __hide)
	//	USkeletalMesh* LeftHand;

	void MoveForward(float AxisValue);
	void MoveRight(float AxisValue);
	void Turn(float AxisValue);
	void TurnUp(float AxisValue);
	TSharedPtr<Eigen::Matrix4d> ConvertToFMatrix(const FVector& Translation, const FQuat& Rotation);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = speeds)
		float translationSpeed = 10.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = speeds)
		float rotationalSpeed = 1.0f;

	UFUNCTION(BlueprintCallable)
		void ShootGunPressed();
	UFUNCTION(BlueprintCallable)
		void ShootGunReleased();
	UFUNCTION(BlueprintCallable)
		void GoToThere();

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = VR)
		ARecastNavMesh* navmesh;

	UFUNCTION(BlueprintCallable)
		void GripPressed();

	UFUNCTION(BlueprintCallable)
		void GripReleased();

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
		FName objectName;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
		FVector Location;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
		FRotator Angles;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
		FQuat AnglesQuat;

	UFUNCTION(BlueprintCallable)
		void letGo();
	AMotionObjectActor* thingIGrabbed;
	//MotionEngine::ArticulatedMotionObject* trycast2;
	MotionEngine::ArticulatedElement* thingIGrabbed2;

	//MotionEngine::MotionObject* thingIGrabbed;

	FTimerHandle ExampleTimerHandle;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "VR")
		UMotionControllerComponent* LeftMotionController;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "VR")
		UMotionControllerComponent* RightMotionController;

	FVector AAcceleration;
	FVector AVelocity;
	FVector LAcceleration;
	FVector LVelocity;
	float time_in;
	//ETrackedDeviceType headsettype;
	/*UFUNCTION()
		void MyCallbackFunction();*/
	TArray<int32> JointItr;
	TArray<double> JointVal;
	TArray<int32> JointDelta;
	bool flag;

private:
	FTimerHandle TimerHandle_MyCallback;
	FVector PreviousPosition;
	Eigen::Vector3d PreviousPositionEigen;
	float TimeSinceLastUpdate;
};
