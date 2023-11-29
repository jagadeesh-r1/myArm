// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "MotionObjectProperties.h"
#include "Eigen/Dense"
#include "MotionObject.h"
#include "TreeRecursiveSolver.h"
#include "MotionObjectActor.generated.h"

UCLASS()
class UNREALSIM_API AMotionObjectActor : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AMotionObjectActor();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	TSharedPtr<MotionEngine::Visual> MObjectVisual;
	
	FVector ActorLocation;
	FQuat ActorOrientation;

	TSharedPtr<Eigen::Matrix4d> ActorG;

	void InitializeActorProperties(TSharedPtr<MotionEngine::Visual> VisualProperties, const TSharedPtr< Eigen::Matrix4d > &ActorPose, FString &Name);

	UPROPERTY(VisibleAnywhere)
	UStaticMeshComponent* VisualMesh;

	FString LinkName;

	TSharedPtr<MotionEngine::MotionObject> SceneObjectPtr;
	TSharedPtr<MotionEngine::ArticulatedMotionObject> SceneObjectPtr2;
	//TSharedPtr<MotionEngine::ArticulatedElement> SceneObjectPtr3;


	TSharedPtr<MotionEngine::TreeRecursiveSolver> ObjectTreeSolverPtr;
};
