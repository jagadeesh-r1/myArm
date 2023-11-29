// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Camera/CameraActor.h"
#include "Engine/World.h"
#include "Engine/SceneCapture2D.h"
#include "Components/SceneCaptureComponent2D.h"
#include "GameFramework/Actor.h"
#include "Camera.generated.h"

UCLASS()
class UNREALSIM_API ACamera : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ACamera();
	UPROPERTY(EditAnywhere)
	int generated_images_width;

	UPROPERTY(EditAnywhere)
	int generated_images_height;
    
	UPROPERTY(EditAnywhere)
	FString screenshots_save_directory;

	UPROPERTY(EditAnywhere)
	FString screenshots_folder;

	// UPROPERTY(EditAnywhere)
	// EROXRGBImageFormats format_rgb;

	UPROPERTY(EditAnywhere, AdvancedDisplay)
	float delay_change_materials;

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;
	
	UPROPERTY(EditAnywhere, BlueprintReadOnly, meta = (ClampMin = "5.0", ClampMax = "170.0", UIMin = "5.0", UIMax = "170.0"))
	float FOV;

	UPROPERTY(Category = GroundTruth, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
	USceneCaptureComponent2D* RGBCamera;

	UPROPERTY(Category = GroundTruth, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
	USceneCaptureComponent2D* DepthCamera;

	//UPROPERTY(Category = GroundTruth, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
	//USceneComponent* Stereo_RGB;

	//UPROPERTY(Category = GroundTruth, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
	//USceneComponent* Stereo_D;

	UFUNCTION(CallInEditor, BlueprintCallable)
	void SaveRGBImage(FString Filename = "");

	UFUNCTION(CallInEditor, BlueprintCallable)
	void SaveDepthImage(FString Filename = "");

	// void SaveAnyImage(FString ImageType, FString Filename);


	float TimeCamChange;

	float TimeSaveImage;

	int ImageNumber;
};
