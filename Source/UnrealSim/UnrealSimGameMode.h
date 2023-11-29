// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameMode.h"
#include "MotionObject.h"
#include "MotionObjectActor.h"
#include "ArticulatedMotionObject.h"
#include "TreeRecursiveSolver.h"
#include "NavMesh/RecastNavMesh.h"
#include "SceneManager.h"
#include "Camera.h"
#include "kinlib_kinematics.h"
#include "Kismet/GameplayStatics.h"
#include "UnrealSimGameMode.generated.h"

/**
 *
 */
UCLASS()
class UNREALSIM_API AUnrealSimGameMode : public AGameMode
{
	GENERATED_BODY()

		virtual void BeginPlay() override;

	virtual void Tick(float DeltaTime) override;

	TArray<AMotionObjectActor*> SceneActors;

	MotionEngine::SceneManager SimulationSceneManager;

	FTimerHandle ExampleTimerHandle;
	FTimerHandle CameraTimerHandle;

	void ExampleTimerCallback(void);
	void SaveImagesTimerCallback(void);

	TArray<int32> JointItr;
	TArray<double> JointVal;
	TArray<int32> JointDelta;

	void SpawnMotionObject(TSharedPtr<MotionEngine::MotionObject> MObject);

	ACamera* RGBDSensor;

	int ImageNumber;

	kinlib::KinematicsSolver RobotKinSolver;
	TArray<FString> ManipulatorJoints;
	TMap<FString, double> ManipJointValues;
	Eigen::VectorXd ManipJointValuesVector;

	FTimerHandle MobileRobotTimerHandle;
	void MobileRobotTimerCallback(void);

	TArray<Eigen::Matrix4d> MobileBaseMotion;
	int32 GoalItr;
	int32 InterpolationItr;
	Eigen::Matrix4d MobileBaseStartPose;

	TArray<Eigen::Matrix4d> TaskTrajectory;
	int32 EEGoalPoseItr;
	std::vector<Eigen::VectorXd> MotionPlan;
	int32 MotionPlanItr;
	bool PlanMotion;

	TMap<FString, double> GripperPositions;
	double GripperPosition;

	int32 DelayItr;
	int32 ImageDelayItr;

	Eigen::Matrix4d DiffG;
	Eigen::Matrix4d RefG;

	bool ResetJointPos;

	// UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = VR)
	APlayerController* playerController;
};
