// Fill out your copyright notice in the Description page of Project Settings.


#include "MyPawn.h"
#include <DrawDebugHelpers.h>
#include "UObject/NameTypes.h"
#include "Misc/DateTime.h"
#include "MotionEngineStatics.h"
#include "MotionObject.h"
#include "MotionObjectActor.h"
#include "UnrealSimGameMode.h"
#include "ROSIntegration/Classes/RI/Topic.h"
#include "ROSIntegration/Classes/ROSIntegrationGameInstance.h"
#include "ROSIntegration/Public/std_msgs/String.h"
#include "URDFParser.h"
#include "Link.h"
#include "Joint.h"
#include "Tree.h"
#include "Chain.h"
#include "SimpleMotionObject.h"
#include "ArticulatedMotionObject.h"
#include "MotionEngineStatics.h"
#include "iostream"

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <math.h>
#include "SimpleMotionObject.h"
#include "SDFParser.h"
#include "URDFParser.h"
//#include "../../../../../../../../Program Files/Epic Games/UE_4.27/Engine/Plugins/Runtime/Oculus/OculusVR/Source/OculusHMD/Public/OculusFunctionLibrary.h"

// Sets default values
AMyPawn::AMyPawn()
{
	flag = true;
 	// Set this pawn to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	VRTrackingCenter = CreateDefaultSubobject<USceneComponent>(TEXT("VRTrackingCenter"));
	Head = CreateDefaultSubobject<UCameraComponent>(TEXT("Head"));
	//outputText = CreateDefaultSubobject<UTextRenderComponent>(TEXT("outputText"));

	LeftController = CreateDefaultSubobject<UMotionControllerComponent>(TEXT("LeftContoller"));
	raygun = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("raygun"));
	LeftCone = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("LeftCone"));

	RightController = CreateDefaultSubobject<UMotionControllerComponent>(TEXT("RightController"));
	RightMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("RightMesh"));

	this->RootComponent = VRTrackingCenter;

	Head->SetupAttachment(VRTrackingCenter);
	//outputText->SetupAttachment(Head);
	LeftController->SetupAttachment(VRTrackingCenter);
	raygun->SetupAttachment(LeftController);
	LeftCone->SetupAttachment(raygun);
	RightController->SetupAttachment(VRTrackingCenter);
	RightMesh->SetupAttachment(RightController);

	//LeftHand = CreateDefaultSubobject<USkeletalMesh>(TEXT("LeftHand"));
	//LeftHand->attach(LeftController);


	LeftMotionController = CreateDefaultSubobject<UMotionControllerComponent>(TEXT("LeftMotionController"));
	LeftMotionController->SetupAttachment(GetRootComponent());

	RightMotionController = CreateDefaultSubobject<UMotionControllerComponent>(TEXT("RightMotionController"));
	RightMotionController->SetupAttachment(GetRootComponent());





	//static ConstructorHelpers::FObjectFinder<UStaticMesh> loadedObj(TEXT("StaticMesh'/Game/__rayguns/raygun.raygun'"));
	//this->raygun->SetStaticMesh(loadedObj.Object);

	//static ConstructorHelpers::FObjectFinder<USkeletalMeshComponent> hand(TEXT("SkeletalMesh'/Game/HandStuff/HandModel/QK_CustomHand.QK_CustomHand'"));
	//this->LeftHand->SetSkeletalMesh(hand.Object);
	//static ConstructorHelpers::FObjectFinder<UMaterial> handmat(TEXT("Material'/Game/HandStuff/HandModel/lambert2.lambert2'"));
	//this->LeftHand->SetMaterial(0, handmat.Object);
	//LeftHand->SetRelativeRotation(FRotator(0, 180, 0));
	//LeftHand->SetRelativeLocation(FVector(0, 0, 0));
	//LeftHand->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
	

	static ConstructorHelpers::FObjectFinder<UStaticMesh> cone(TEXT("StaticMesh'/Engine/BasicShapes/Cone.Cone'"));
	this->LeftCone->SetStaticMesh(cone.Object);
	this->RightMesh->SetStaticMesh(cone.Object);

	static ConstructorHelpers::FObjectFinder<UMaterial> graymat(TEXT("/Engine/BasicShapes/BasicShapeMaterial"));

	LeftCone->SetMaterial(0, graymat.Object);
	RightMesh->SetMaterial(0, graymat.Object);

	RightController->MotionSource = FXRMotionControllerBase::RightHandSourceId;
	LeftController->MotionSource = FXRMotionControllerBase::LeftHandSourceId;

	//raygun->SetRelativeLocation(FVector(15, 0, 0));
	//LeftCone->SetRelativeLocation(FVector(26, 0, 6));
	LeftCone->SetRelativeRotation(FRotator(-90, 180, 180));
	LeftCone->SetRelativeScale3D(FVector(0.1f, 0.1f, 0.1f));

	RightMesh->SetRelativeRotation(FRotator(-90, 180, 180));
	RightMesh->SetRelativeScale3D(FVector(0.1f, 0.1f, 0.1f));

	AutoPossessPlayer = EAutoReceiveInput::Player0;

	/*static ConstructorHelpers::FObjectFinder<UMaterial> unlitText(TEXT("Material'/Engine/EngineMaterials/DefaultTextMaterialTranslucent.DefaultTextMaterialTranslucent'"));
	outputText->SetMaterial(0, unlitText.Object);
	outputText->SetTextRenderColor(FColor::Red);
	outputText->HorizontalAlignment = EHorizTextAligment::EHTA_Center;
	outputText->VerticalAlignment = EVerticalTextAligment::EVRTA_TextCenter;
	outputText->SetRelativeRotation(FRotator(0, 180.0f, 0));
	outputText->SetRelativeLocation(FVector(150, 0, 0));*/

	raygun->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	LeftCone->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
	RightMesh->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);

	LeftCone->SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Overlap);
	RightMesh->SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Overlap);

	//navmesh = dynamic_cast<ARecastNavMesh*>(UGameplayStatics::GetActorOfClass(GetWorld(), ARecastNavMesh::StaticClass()));


}

// Called when the game starts or when spawned
void AMyPawn::BeginPlay()
{	

	Super::BeginPlay();
	PreviousPosition = RightMesh->GetComponentLocation();
	PreviousPositionEigen = Eigen::Vector3d(PreviousPosition.X/100, -PreviousPosition.Y/100, PreviousPosition.Z/100);
}



// Called every frame
void AMyPawn::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	TimeSinceLastUpdate += DeltaTime;
	FVector CurrentPosition = RightMesh->GetComponentLocation();
	Eigen::Vector3d CurrentPositionEigen(CurrentPosition.X/100, -CurrentPosition.Y/100, CurrentPosition.Z/100);
	//Eigen::Vector3d Velocity_eigen = (CurrentPositionEigen - PreviousPositionEigen) / (DeltaTime * 100);

	FVector Velocity = (CurrentPosition - PreviousPosition) / DeltaTime;
	Eigen::Vector3d Velocity_eigen = MotionEngine::MotionEngineStatics::UnrealToROS(Velocity);
	

	PreviousPosition = CurrentPosition;
	PreviousPositionEigen = CurrentPositionEigen;
	//GLog->Log(Velocity.ToString() + "---------- current velocity");
	


	//GEngine->AddOnScreenDebugMessage(-1, 0.1f, FColor::Red, FString::Printf(TEXT("Controller Velocity: %f"), normm));


	if (thingIGrabbed) {


		FDateTime CurrentTime = FDateTime::Now();
		FString TimestampString = CurrentTime.ToString();

		objectName = thingIGrabbed->GetFName();
		FString objectName1 = FString::Printf(TEXT("%s"), *objectName.ToString());
		Location = thingIGrabbed->GetActorLocation();
		Angles = thingIGrabbed->GetActorRotation();
		AnglesQuat = thingIGrabbed->GetActorQuat();


		float LocationX = Location.X;
		float LocationY = Location.Y;
		float LocationZ = Location.Z;

		float QuatX = AnglesQuat.X;
		float QuatY = AnglesQuat.Y;
		float QuatZ = AnglesQuat.Z;
		float QuatW = AnglesQuat.W;

		FString LocationStringX = FString::SanitizeFloat(LocationX);
		FString LocationStringY = FString::SanitizeFloat(LocationY);
		FString LocationStringZ = FString::SanitizeFloat(LocationZ);

		FString LocationString = LocationStringX + "," + LocationStringY + "," + LocationStringZ;


		FString QuatStringX = FString::SanitizeFloat(QuatX);
		FString QuatStringY = FString::SanitizeFloat(QuatY);
		FString QuatStringZ = FString::SanitizeFloat(QuatZ);
		FString QuatStringW = FString::SanitizeFloat(QuatW);


		FString AnglesString = QuatStringX + "," + QuatStringY + "," + QuatStringZ + "," + QuatStringW;

		//GLog->Log(LocationString + "----------" + AnglesString);
		FString DataString = TimestampString + "," + LocationString + "," + AnglesString + '\n';

		FString FileName = FPaths::ProjectContentDir() + objectName1 + "_location_data.csv";
		FFileHelper::SaveStringToFile(DataString, *FileName, FFileHelper::EEncodingOptions::AutoDetect, &IFileManager::Get(), EFileWrite::FILEWRITE_Append);


		TSharedPtr<MotionEngine::MotionObject> MObject(StaticCastSharedPtr<MotionEngine::MotionObject>(thingIGrabbed->SceneObjectPtr));
		TSharedPtr<MotionEngine::ArticulatedMotionObject> ArticulatedMObject(StaticCastSharedPtr<MotionEngine::ArticulatedMotionObject>(thingIGrabbed->SceneObjectPtr));

		TSharedPtr<MotionEngine::TreeRecursiveSolver> TreeRSolver(StaticCastSharedPtr<MotionEngine::TreeRecursiveSolver>(thingIGrabbed->ObjectTreeSolverPtr));

		//TSharedPtr<MotionEngine::ArticulatedElement> ArticulatedElementObject(StaticCastSharedPtr<MotionEngine::ArticulatedElement>(thingIGrabbed2->ParentElement));

		if (ArticulatedMObject) {
			GLog->Log(thingIGrabbed->LinkName + " Thing I  GRABBED");
			GLog->Log(ArticulatedMObject->ObjectName + "LINK GRABBED");
			GLog->Log(ArticulatedMObject->ObjectTree->RootElement->Name + " Root Name");

			TSharedPtr<MotionEngine::TreeRecursiveSolver> Solver = MakeShareable(new MotionEngine::TreeRecursiveSolver());
			Solver->InitializeSolver(ArticulatedMObject->ObjectTree);

			//for (const TPair<FString, TSharedPtr<MotionEngine::JointState>>& pair: ArticulatedMObject->ObjectTree->TreeJointStateMap) {
			//	GLog->Log(pair.Key + " pair Name");
			//}
			int direction = 1;
		/*	for (const TPair<FString, TSharedPtr<MotionEngine::LinkState>>& pair : ArticulatedMObject->ObjectTree->TreeLinkStateMap) {
				GLog->Log(pair.Key + " link Name");
			}*/

			if (ArticulatedMObject->ObjectTree->TreeLinkStateMap[thingIGrabbed->LinkName]->ParentElement->ElementType == MotionEngine::ArticulatedElement::JOINT) {
				GLog->Log(ArticulatedMObject->ObjectTree->TreeLinkStateMap[thingIGrabbed->LinkName]->ParentElement->Name + " Parent name of  GRABBED");


				TSharedPtr<MotionEngine::Joint> HandleJoint(StaticCastSharedPtr<MotionEngine::Joint>(ArticulatedMObject->ObjectTree->TreeLinkStateMap[thingIGrabbed->LinkName]->ParentElement));
				

				if (HandleJoint->Type == MotionEngine::Joint::FIXED) {
					GLog->Log(HandleJoint->ParentLinkName);
					GLog->Log(ArticulatedMObject->ObjectTree->TreeLinkStateMap[HandleJoint->ParentLinkName]->ParentElement->Name + " Parent name of  GRABBED");

					if (ArticulatedMObject->JointMap[ArticulatedMObject->ObjectTree->TreeLinkStateMap[HandleJoint->ParentLinkName]->ParentElement->Name]->Type == MotionEngine::Joint::PRISMATIC) {
						TSharedPtr<MotionEngine::JointState> Currentjointstate(StaticCastSharedPtr<MotionEngine::JointState>(ArticulatedMObject->JointMap[ArticulatedMObject->ObjectTree->TreeLinkStateMap[HandleJoint->ParentLinkName]->ParentElement->Name]));
						double InitialJointPos = *Currentjointstate->JointPosition.Get();

						Eigen::Matrix4d JointG = *Currentjointstate->JointG.Get();
						double jointX = JointG[12];
						double jointY = JointG[13];
						double jointZ = JointG[14];

						double MinJointVal = ArticulatedMObject->JointMap[ArticulatedMObject->ObjectTree->TreeLinkStateMap[HandleJoint->ParentLinkName]->ParentElement->Name]->Limits.LowerLimit;
						double MaxJointVal = ArticulatedMObject->JointMap[ArticulatedMObject->ObjectTree->TreeLinkStateMap[HandleJoint->ParentLinkName]->ParentElement->Name]->Limits.UpperLimit;



						Eigen::Vector3d joint_axis_vector_eigen(JointG[8], JointG[9], JointG[10]);

						float deltax_eigen = joint_axis_vector_eigen.dot(Velocity_eigen);
						GLog->Log("deltavalue before : " + FString::SanitizeFloat(deltax_eigen));


						int scale_factor = 65;
						double newposition = InitialJointPos + deltax_eigen / scale_factor * direction;
						//double newposition = InitialJointPos + (((MaxJointVal - MinJointVal) / 5) * deltax);

						//GLog->Log("InitialPos: " + FString::SanitizeFloat(InitialJointPos));
						//GLog->Log("newposition: "+ FString::SanitizeFloat(newposition));


						if (newposition <= MaxJointVal && newposition >= MinJointVal) {

							GLog->Log("In between--------" + ArticulatedMObject->JointMap[ArticulatedMObject->ObjectTree->TreeLinkStateMap[thingIGrabbed->LinkName]->ParentElement->Name]->Name);

							TMap<FString, double> XXXXXX;
							GLog->Log(ArticulatedMObject->ObjectTree->TreeLinkStateMap[HandleJoint->ParentLinkName]->ParentElement->Name + "++++++++++++++++++++++++++++");
							XXXXXX.Add(ArticulatedMObject->ObjectTree->TreeLinkStateMap[HandleJoint->ParentLinkName]->ParentElement->Name, newposition);

							Solver->SolvePositionFK(XXXXXX);
						}
					}
					if (ArticulatedMObject->JointMap[ArticulatedMObject->ObjectTree->TreeLinkStateMap[HandleJoint->ParentLinkName]->ParentElement->Name]->Type == MotionEngine::Joint::REVOLUTE) {
						TSharedPtr<MotionEngine::JointState> Currentjointstate(StaticCastSharedPtr<MotionEngine::JointState>(ArticulatedMObject->JointMap[ArticulatedMObject->ObjectTree->TreeLinkStateMap[HandleJoint->ParentLinkName]->ParentElement->Name]));
						double InitialJointPos = *Currentjointstate->JointPosition.Get();

						Eigen::Matrix4d JointG = *Currentjointstate->JointG.Get();
						double jointX = JointG[12];
						double jointY = JointG[13];
						double jointZ = JointG[14];

						double MinJointVal = ArticulatedMObject->JointMap[ArticulatedMObject->ObjectTree->TreeLinkStateMap[HandleJoint->ParentLinkName]->ParentElement->Name]->Limits.LowerLimit;
						double MaxJointVal = ArticulatedMObject->JointMap[ArticulatedMObject->ObjectTree->TreeLinkStateMap[HandleJoint->ParentLinkName]->ParentElement->Name]->Limits.UpperLimit;


						Eigen::Vector3d v1_eigen(jointX, jointY, jointZ);

						Eigen::Vector3d r_vector_eigen = (CurrentPositionEigen - v1_eigen);	 

						Eigen::Vector3d joint_axis_vector_eigen(JointG[8], JointG[9], JointG[10]);

						Eigen::Vector3d tangent_eigen = joint_axis_vector_eigen.cross(r_vector_eigen);
						tangent_eigen.normalize();
						float deltax_eigen = tangent_eigen.dot(Velocity_eigen);
						GLog->Log("abs: " + FString::SanitizeFloat(abs(deltax_eigen)));
						GLog->Log("max joint value: " + FString::SanitizeFloat(MaxJointVal));
						GLog->Log("before modifying: " + FString::SanitizeFloat(deltax_eigen));

						int scale_factor = 10;

						if (abs(deltax_eigen) > MaxJointVal/5) {

							deltax_eigen = deltax_eigen/5;
							if (abs(deltax_eigen) > MaxJointVal / 5) {
								deltax_eigen = deltax_eigen / 2;
							}
						}

						double newposition = InitialJointPos + deltax_eigen / scale_factor * direction;
						//double newposition = InitialJointPos + (((MaxJointVal - MinJointVal) / 5) * deltax);

						GLog->Log("after modifying: " + FString::SanitizeFloat(deltax_eigen));
						GLog->Log("newposition: "+ FString::SanitizeFloat(newposition));


						if (newposition <= MaxJointVal && newposition >= MinJointVal) {

							GLog->Log("In between--------" + ArticulatedMObject->JointMap[ArticulatedMObject->ObjectTree->TreeLinkStateMap[thingIGrabbed->LinkName]->ParentElement->Name]->Name);

							TMap<FString, double> XXXXXX;
							GLog->Log(ArticulatedMObject->ObjectTree->TreeLinkStateMap[HandleJoint->ParentLinkName]->ParentElement->Name + "++++++++++++++++++++++++++++");
							XXXXXX.Add(ArticulatedMObject->ObjectTree->TreeLinkStateMap[HandleJoint->ParentLinkName]->ParentElement->Name, newposition);

							Solver->SolvePositionFK(XXXXXX);
						}
					}
				}

			}


		}

		//if (ArticulatedMObject->ObjectTree->RootElement->ElementType == MotionEngine::ArticulatedElement::LINK) {
		//	GLog->Log("*********************************************************");
		//	//GLog->Log(ArticulatedMObject->ObjectTree->RootElement->Name + "LINK GRABBED");
		//	//GLog->Log(ArticulatedMObject->ObjectTree->TreeLinkStateMap[thingIGrabbed->LinkName]->Name + " LINK GRABBED");
		//	
		//	if (ArticulatedMObject->ObjectTree->TreeLinkStateMap[thingIGrabbed->LinkName]->ParentElement->ElementType == MotionEngine::ArticulatedElement::JOINT) {
		//		GLog->Log(ArticulatedMObject->ObjectTree->TreeLinkStateMap[thingIGrabbed->LinkName]->ParentElement->Name + "Parent joint element name");


		//		double MinJointVal = ArticulatedMObject->JointMap[ArticulatedMObject->ObjectTree->TreeLinkStateMap[thingIGrabbed->LinkName]->ParentElement->Name]->Limits.LowerLimit;
		//		double MaxJointVal = ArticulatedMObject->JointMap[ArticulatedMObject->ObjectTree->TreeLinkStateMap[thingIGrabbed->LinkName]->ParentElement->Name]->Limits.UpperLimit;

		//		TSharedPtr<MotionEngine::JointState> Currentjointstate(StaticCastSharedPtr<MotionEngine::JointState>(ArticulatedMObject->JointMap[ArticulatedMObject->ObjectTree->TreeLinkStateMap[thingIGrabbed->LinkName]->ParentElement->Name]));
		//		double InitialJointPos = *Currentjointstate->JointPosition.Get();
		//		
		//		Eigen::Matrix4d JointG = *Currentjointstate->JointG.Get();
		//		double jointX = JointG[12];
		//		double jointY = JointG[13];
		//		double jointZ = JointG[14];

		//		Eigen::Vector3d v1_eigen(jointX, jointY, jointZ);

		//		Eigen::Vector3d r_vector_eigen = (CurrentPositionEigen - v1_eigen);	 

		//		Eigen::Vector3d joint_axis_vector_eigen(JointG[8], JointG[9], JointG[10]);

		//		Eigen::Vector3d tangent_eigen = joint_axis_vector_eigen.cross(r_vector_eigen);
		//		tangent_eigen.normalize();
		//		float deltax_eigen = tangent_eigen.dot(Velocity_eigen);


		//		if (flag) {
		//			GLog->Log("*********************************************************");
		//			GLog->Log("hsfdsfdsd");

		//			std::cout << JointG << std::endl;
		//			flag = false;
		//		}

		//		//double newposition = InitialJointPos + deltaTheta * 10;
		//		double newposition = InitialJointPos;
		//		newposition = InitialJointPos + deltax_eigen / 500;
		//		if (newposition <= MaxJointVal && newposition>= MinJointVal) {
		//			//newposition = InitialJointPos + deltax/10;

		//			InitialJointPos = newposition;
		//			this->outputText->SetText("In between range");
		//			TMap<FString, double> XXXXXX;
		//			XXXXXX.Add(ArticulatedMObject->JointMap[ArticulatedMObject->ObjectTree->TreeLinkStateMap[thingIGrabbed->LinkName]->ParentElement->Name]->Name, newposition);
		//			thingIGrabbed->ObjectTreeSolverPtr->SolvePositionFK(XXXXXX);
		//		}
		//		
		//	}
		//	/*for (int i = 0; i < ArticulatedMObject->ObjectTree->TreeLinkStateMap[thingIGrabbed->LinkName]->ChildElements.Num(); i++) {
		//		GLog->Log(ArticulatedMObject->ObjectTree->TreeLinkStateMap[thingIGrabbed->LinkName]->ChildElements[i]->Name + " - Child Name");
		//	}*/

		//	//GLog->Log(ArticulatedMObject->ObjectTree->RootElement->ParentElement->Name);
		//	GLog->Log("*********************************************************");
		//}
		//

	}
}



// Called to bind functionality to input
void AMyPawn::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);

	PlayerInputComponent->BindAxis("MoveForward", this, &AMyPawn::MoveForward);
	PlayerInputComponent->BindAxis("MoveRight", this, &AMyPawn::MoveRight);
	PlayerInputComponent->BindAxis("Turn", this, &AMyPawn::Turn);
	PlayerInputComponent->BindAxis("LookUp", this, &AMyPawn::TurnUp);

	PlayerInputComponent->BindAction("GoToThere", EInputEvent::IE_Pressed, this, &AMyPawn::GoToThere);
	PlayerInputComponent->BindAction("ShootGun", EInputEvent::IE_Pressed, this, &AMyPawn::ShootGunPressed);
	PlayerInputComponent->BindAction("ShootGun", IE_Released, this, &AMyPawn::ShootGunReleased);

	PlayerInputComponent->BindAction("Grip", IE_Pressed, this, &AMyPawn::GripPressed);
	PlayerInputComponent->BindAction("Grip", IE_Released, this, &AMyPawn::GripReleased);

}

void AMyPawn::MoveForward(float AxisValue) {
	this->AddActorWorldOffset(this->GetActorForwardVector() * AxisValue * translationSpeed);
}

void AMyPawn::MoveRight(float AxisValue) {
	this->AddActorWorldOffset(this->GetActorRightVector() * AxisValue * translationSpeed);
}

void AMyPawn::Turn(float AxisValue) {
	this->AddActorWorldRotation(FRotator(0, AxisValue * rotationalSpeed, 0));
}

void AMyPawn::TurnUp(float AxisValue) {
	this->AddActorLocalRotation(FRotator(-AxisValue * rotationalSpeed, 0, 0));
}



TSharedPtr<Eigen::Matrix4d> AMyPawn::ConvertToFMatrix(const FVector& Translation, const FQuat& Rotation)
{
	TSharedPtr<Eigen::Matrix4d> Matrix = MakeShareable(new Eigen::Matrix4d());

	// Set the translation component
	Matrix->block<3, 1>(0, 3) = Eigen::Vector3d(Translation.X, -Translation.Y, Translation.Z);

	// Set the rotation component
	Eigen::Quaterniond EigenRotation(Rotation.W, -Rotation.X, Rotation.Y, -Rotation.Z);
	Matrix->block<3, 3>(0, 0) = EigenRotation.matrix();

	return Matrix;
}





void AMyPawn::ShootGunPressed() {

}

void AMyPawn::ShootGunReleased() {
	//this->outputText->SetText("left trigger released");

}

void AMyPawn::GoToThere() {

}



void AMyPawn::GripPressed() {
	//this->outputText->SetText("grip");
	TArray<AActor*> outoverlap;
	RightMesh->GetOverlappingActors(outoverlap);
	GLog->Log("grip button pressed");
	if (outoverlap.Num() > 0) {
		//this->outputText->SetText(outoverlap[0]->GetName());
		GLog->Log(outoverlap[0]->GetName());
		AMotionObjectActor* trycast = dynamic_cast<AMotionObjectActor*>(outoverlap[0]);
		MotionEngine::ArticulatedElement* trycast2 = dynamic_cast<MotionEngine::ArticulatedElement*>(outoverlap[0]);

		if (trycast2) {
			thingIGrabbed2 = trycast2;
		}

		if (trycast) {
			//this->outputText->SetText("succeed");

			//thingIGrabbed = trycast;
			thingIGrabbed = trycast;
		}
		else {
			//this->outputText->SetText("failed");
		}
	}

}

void AMyPawn::GripReleased() {
	//this->outputText->SetText("grip");
	letGo();
}


void AMyPawn::letGo() {
	if (thingIGrabbed) {
		//this->outputText->SetText("grip");

		thingIGrabbed = nullptr;
	}
}