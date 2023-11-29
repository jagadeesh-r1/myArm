// Fill out your copyright notice in the Description page of Project Settings.


#include "Camera.h"
#include "Kismet/GameplayStatics.h"


#include "ROXTaskUtils.h"
#include "Misc/FileHelper.h"
#include "Engine/TextureRenderTarget2D.h"
#include "UObject/ConstructorHelpers.h"
#include "UObject/UObjectIterator.h"
#include "IImageWrapper.h"
#include "IImageWrapperModule.h"
#include "ImageUtils.h"
#include "Modules/ModuleManager.h"
#include "ROSIntegration/Classes/RI/Topic.h"
#include "ROSIntegration/Classes/ROSIntegrationGameInstance.h"
#include "ROSIntegration/Public/std_msgs/String.h"
#include "ROSIntegration/Public/std_msgs/UInt8MultiArray.h"
#include "ROSIntegration/Public/sensor_msgs/Image.h"
#include "ROSIntegration/Public/sensor_msgs/PointCloud2.h"
//#include <boost/make_shared.hpp>
#include "Misc/Base64.h"



// Sets default values
ACamera::ACamera()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	screenshots_save_directory = FPaths::ProjectUserDir();
	screenshots_folder = "AcquiredData";
	generated_images_width = 1280;
	generated_images_height = 720;
	FOV = 90.0f;

	// Stereo_RGB = CreateDefaultSubobject<USceneComponent>("Stereo_RGB");
	// Stereo_RGB->SetupAttachment(GetRootComponent());
	// Stereo_RGB->SetRelativeLocation(FVector(0.0f, 0.0f, 0.0f));

	RGBCamera = CreateDefaultSubobject<USceneCaptureComponent2D>(FName(TEXT("SceneCapture_RGB")));
	// RGBCamera->SetupAttachment(Stereo_RGB);
	//RGBCamera->SetRelativeLocation(FVector(0.0f, 0.0f, 0.0f));
	RGBCamera->SetupAttachment(RootComponent);

	RGBCamera->TextureTarget = NewObject<UTextureRenderTarget2D>();
	RGBCamera->TextureTarget->InitAutoFormat(generated_images_width, generated_images_height);
	RGBCamera->PostProcessSettings.bOverride_AutoExposureMinBrightness = true;
	RGBCamera->PostProcessSettings.bOverride_AutoExposureMaxBrightness = true;
	RGBCamera->PostProcessSettings.AutoExposureMinBrightness = 1.0f;
	RGBCamera->PostProcessSettings.AutoExposureMaxBrightness = 1.0f;
	RGBCamera->FOVAngle = FOV;

	RGBCamera->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
	RGBCamera->TextureTarget->RenderTargetFormat = ETextureRenderTargetFormat::RTF_RGBA8;
	RGBCamera->TextureTarget->TargetGamma = 2.2;
	RGBCamera->PostProcessBlendWeight = 0;



	// Stereo_D = CreateDefaultSubobject<USceneComponent>("Stereo_D");
	// Stereo_D->SetupAttachment(GetRootComponent());
	// Stereo_D->SetRelativeLocation(FVector(0.0f, 0.0f, 0.0f));

	DepthCamera = CreateDefaultSubobject<USceneCaptureComponent2D>(FName(TEXT("SceneCapture_D")));
	// DepthCamera->SetupAttachment(Stereo_D);
	//DepthCamera->SetRelativeLocation(FVector(0.0f, 0.0f, 0.0f));
	//DepthCamera->SetupAttachment(RootComponent);
	DepthCamera->SetupAttachment(RGBCamera);

	DepthCamera->TextureTarget = NewObject<UTextureRenderTarget2D>();
	DepthCamera->TextureTarget->InitAutoFormat(generated_images_width, generated_images_height);
	DepthCamera->PostProcessSettings.bOverride_AutoExposureMinBrightness = true;
	DepthCamera->PostProcessSettings.bOverride_AutoExposureMaxBrightness = true;
	DepthCamera->PostProcessSettings.AutoExposureMinBrightness = 1.0f;
	DepthCamera->PostProcessSettings.AutoExposureMaxBrightness = 1.0f;
	DepthCamera->FOVAngle = FOV;

	DepthCamera->CaptureSource = ESceneCaptureSource::SCS_SceneDepth;
	DepthCamera->TextureTarget->RenderTargetFormat = ETextureRenderTargetFormat::RTF_RGBA16f;
	DepthCamera->TextureTarget->TargetGamma = 0;

}

// Called when the game starts or when spawned
void ACamera::BeginPlay()
{
	Super::BeginPlay();


}

// Called every frame
void ACamera::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

int counter = 0;

void ACamera::SaveRGBImage(FString Filename)
{
	//FString CameraName = GetActorLabel();

	//FString FullFilename = screenshots_save_directory + screenshots_folder + "/" + GetWorld()->GetMapName() + "/" + CameraName + "/" + "rgb" + "/" + Filename;

	int32 Width = RGBCamera->TextureTarget->SizeX;
	int32 Height = RGBCamera->TextureTarget->SizeY;

	FTextureRenderTargetResource* RenderTargetResource;
	RenderTargetResource = RGBCamera->TextureTarget->GameThread_GetRenderTargetResource();

	TArray<FColor> ImageData;
	ImageData.AddUninitialized(Width * Height);

	TArray<FLinearColor> ImageDataLC;
	ImageDataLC.AddUninitialized(Width * Height);
	RenderTargetResource->ReadLinearColorPixels(ImageDataLC);

	TArray<uint8> test2;
	test2.Init(0, generated_images_width * generated_images_height * 3);

	for (int i = 0; i < ImageData.Num(); ++i)
	{
		ImageData[i] = ImageDataLC[i].ToFColor(false);
		ImageData[i].A = 255;
		
	}
	int k = 0;
	for (int j = 0; j < test2.Num(); j+=3)
	{
		test2[j] = ImageData[k].B;
		test2[j + 1] = ImageData[k].G;
		test2[j + 2] = ImageData[k].R;
		k++;
	}

	UTopic* ExampleTopic = NewObject<UTopic>(UTopic::StaticClass());
	UROSIntegrationGameInstance* rosinst = Cast<UROSIntegrationGameInstance>(GetGameInstance());
	ExampleTopic->Init(rosinst->ROSIntegrationCore, TEXT("camera/image"), TEXT("sensor_msgs/Image"));

	// (Optional) Advertise the topic
	ExampleTopic->Advertise();
	ROSMessages::sensor_msgs::Image OutputImage;
	OutputImage.data = test2.GetData();
	OutputImage.header.seq = 1;

	OutputImage.height = generated_images_height;
	OutputImage.width = generated_images_width;
	OutputImage.encoding = "bgr8";
	OutputImage.is_bigendian = false;
	OutputImage.step = 3 * generated_images_width;
	TSharedPtr<ROSMessages::sensor_msgs::Image> CompressedImage(new ROSMessages::sensor_msgs::Image(OutputImage));
	ExampleTopic->Publish(CompressedImage);

}


void ACamera::SaveDepthImage(FString Filename)
{
	//FString CameraName = GetActorLabel();

	//FString FullFilename = screenshots_save_directory + screenshots_folder + "/" + GetWorld()->GetMapName() + "/" + CameraName + "/" + "depth" + "/" + Filename;

	int32 Width = DepthCamera->TextureTarget->SizeX;
	int32 Height = DepthCamera->TextureTarget->SizeY;

	FTextureRenderTargetResource* RenderTargetResource;
	RenderTargetResource = DepthCamera->TextureTarget->GameThread_GetRenderTargetResource();

	//Logic for Depth
	TArray<FFloat16Color> ImageData_Depth;
	ImageData_Depth.AddUninitialized(Width * Height);
	RenderTargetResource->ReadFloat16Pixels(ImageData_Depth);

	TArray<uint8> test2;
	test2.Init(0, 2* generated_images_width * generated_images_height);
	int j = 0;

	if (ImageData_Depth.Num() != 0 && ImageData_Depth.Num() == Width * Height)
	{
		TArray<uint16> Grayscaleuint16Data;
		for (auto px : ImageData_Depth)
		{
			// Max value float16: 65504.0 -> It is cm, so it can represent up to 655.04m
			// Max value uint16: 65535 (65536 different values) -> It is going to be mm, so it can represent up to 65.535m - 6553.5cm
			float pixelCm = px.R.GetFloat();
			if (pixelCm > 655.4f)
			// if (pixelCm > 6553.4f || pixelCm < 0.3f)
			{
				Grayscaleuint16Data.Add(0);
				float pixelMm = pixelCm * 100.0f;
				test2[j] = ((uint16)floorf(pixelMm))>>8;
				test2[++j] = ((uint16)floorf(pixelMm)) & 0xff;
				j++;
			}
			else
			{
				float pixelMm = pixelCm * 100.0f;
				Grayscaleuint16Data.Add((uint16)floorf(pixelMm));
				test2[j] = ((uint16)floorf(pixelMm)) >> 8;
				test2[++j] = ((uint16)floorf(pixelMm)) & 0xff;
				j++;
			}
		}

		UTopic* ExampleTopic = NewObject<UTopic>(UTopic::StaticClass());
		UROSIntegrationGameInstance* rosinst = Cast<UROSIntegrationGameInstance>(GetGameInstance());
		ExampleTopic->Init(rosinst->ROSIntegrationCore, TEXT("camera/image2"), TEXT("sensor_msgs/Image"));

		ROSMessages::sensor_msgs::Image OutputImage;
		OutputImage.data = test2.GetData();
		OutputImage.header.seq = 1;
		OutputImage.height = generated_images_height;
		OutputImage.width = generated_images_width;
		OutputImage.encoding = "16UC1";
		OutputImage.is_bigendian = false;
		OutputImage.step = 2 * generated_images_width;

		TSharedPtr<ROSMessages::sensor_msgs::Image> CompressedImage(new ROSMessages::sensor_msgs::Image(OutputImage));
		ExampleTopic->Publish(CompressedImage);
	}
}

