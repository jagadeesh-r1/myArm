// Fill out your copyright notice in the Description page of Project Settings.


#include "MotionObjectActor.h"
#include "MotionEngineStatics.h"
#include "ROSIntegration/Classes/RI/Topic.h"
#include "ROSIntegration/Classes/ROSIntegrationGameInstance.h"
#include "ROSIntegration/Public/std_msgs/String.h"

// Sets default values
AMotionObjectActor::AMotionObjectActor()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = false;

	VisualMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("VisualMesh"));
	SetRootComponent(VisualMesh);
}

// Called when the game starts or when spawned
void AMotionObjectActor::BeginPlay()
{
	Super::BeginPlay();

	ActorG = MakeShareable(new Eigen::Matrix4d());
	*ActorG = Eigen::Matrix4d::Identity();



	////publish ****vinay*****
	//UTopic* ExampleTopic = NewObject<UTopic>(UTopic::StaticClass());
	//UROSIntegrationGameInstance* rosinst = Cast<UROSIntegrationGameInstance>(GetGameInstance());
	//ExampleTopic->Init(rosinst->ROSIntegrationCore, TEXT("/chatter"), TEXT("std_msgs/String"));

	//// (Optional) Advertise the topic
	//ExampleTopic->Advertise();

	//// Publish a string to the topic
	//TSharedPtr<ROSMessages::std_msgs::String> StringMessage(new ROSMessages::std_msgs::String("This is *** example of playing with ros Vinay Nandamuri project"));
	//ExampleTopic->Publish(StringMessage);
	////*****************************//





	////subcribe ****vinay*****
	//UTopic* ExampleTopic2 = NewObject<UTopic>(UTopic::StaticClass());
	//UROSIntegrationGameInstance* rosinst2 = Cast<UROSIntegrationGameInstance>(GetGameInstance());
	//ExampleTopic2->Init(rosinst2->ROSIntegrationCore, TEXT("/example_topic"), TEXT("std_msgs/String"));



	//std::function<void(TSharedPtr<FROSBaseMsg>)> SubscribeCallback = [](TSharedPtr<FROSBaseMsg> msg) -> void
	//{
	//	auto Concrete = StaticCastSharedPtr<ROSMessages::std_msgs::String>(msg);
	//	if (Concrete.IsValid())
	//	{
	//		UE_LOG(LogTemp, Log, TEXT("Incoming string was: %s"), (*(Concrete->_Data)));
	//	}
	//	return;
	//};

	//// Subscribe to the topic
	//ExampleTopic2->Subscribe(SubscribeCallback);
	////*************************//



}

// Called every frame
void AMotionObjectActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

void AMotionObjectActor::InitializeActorProperties(TSharedPtr<MotionEngine::Visual> VisualProperties, const TSharedPtr<Eigen::Matrix4d> &ActorPose, FString &Name)
{
	MObjectVisual = VisualProperties;

	LinkName = Name;

	if(MObjectVisual)
	{
		if(MObjectVisual->VisualGeometry)
		{
			ActorG = ActorPose;

			if(MObjectVisual->VisualGeometry->Type == MotionEngine::Geometry::MESH)
			{
				TSharedPtr<MotionEngine::GeometryMesh> VisualMeshGeometry(StaticCastSharedPtr<MotionEngine::GeometryMesh>(MObjectVisual->VisualGeometry));

				if(VisualMeshGeometry)
				{
					FString MeshPath = VisualMeshGeometry->MeshDirectory + TEXT('/') + VisualMeshGeometry->MeshFileName + TEXT('.') + VisualMeshGeometry->MeshFileName;
					UE_LOG(LogTemp, Display, TEXT("Loading MotionObjectActor VisualMesh from %s"), *MeshPath)
					UStaticMesh* ThisLinkMesh = (UStaticMesh*)StaticLoadObject(UStaticMesh::StaticClass(), NULL, *FString(MeshPath));
					VisualMesh->SetStaticMesh(ThisLinkMesh);
					if (VisualMeshGeometry->MeshFileName == "glass" || VisualMeshGeometry->MeshFileName == "bowl" || VisualMeshGeometry->MeshFileName == "sphere" || VisualMeshGeometry->MeshFileName == "spoon") {
						UE_LOG(LogTemp, Display, TEXT("Loading MotionObjectActor VisualMesh from ***************************** %s"), *MeshPath)
						//VisualMesh->SetMobility(EComponentMobility::Movable);
						////VisualMesh->SetSimulatePhysics(true);
						//VisualMesh->SetMassOverrideInKg(NAME_None, 0.4f);
						//VisualMesh->SetGenerateOverlapEvents(true);
						//VisualMesh->SetCollisionProfileName("PhysicsActor");
						//VisualMesh->SetNotifyRigidBodyCollision(true);
						//VisualMesh->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
						if (VisualMeshGeometry->MeshFileName == "sphere") {
							VisualMesh->SetSimulatePhysics(true);
							VisualMesh->SetEnableGravity(true);
							VisualMesh->SetMassOverrideInKg(NAME_None, 0.2f);
						}
					}
					//VisualMesh->SetSimulatePhysics(true);
					//VisualMesh->SetEnableGravity(true);
					VisualMesh->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
					VisualMesh->SetMobility(EComponentMobility::Movable);
					VisualMesh->SetNotifyRigidBodyCollision(true);
					VisualMesh->SetCollisionProfileName("PhysicsActor");
				}
			}
		}
	}
}