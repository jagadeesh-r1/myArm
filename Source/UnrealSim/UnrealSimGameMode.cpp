// Copyright Epic Games, Inc. All Rights Reserved.


#include "UnrealSimGameMode.h"
#include "Engine/StaticMeshActor.h"
#include "MotionEngineStatics.h"
#include "MotionObject.h"
#include "SimpleMotionObject.h"
#include "ArticulatedMotionObject.h"
#include "URDFParser.h"
#include "Link.h"
#include "Joint.h"
#include "Tree.h"
#include "Chain.h"
#include "MyPawn.h"
#include "ROSIntegration/Classes/RI/Topic.h"
#include "ROSIntegration/Classes/ROSIntegrationGameInstance.h"
#include "ROSIntegration/Public/std_msgs/String.h"


#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <math.h>
#include <sdf/sdf.hh>

#include "DualQuat.h"
#include <iostream>
#include <fstream>


Eigen::MatrixXd readCSV(std::string file, int rows, int cols)
{
    std::ifstream in(file);

    std::string line;

    int row = 0;
    int col = 0;

    Eigen::MatrixXd res = Eigen::MatrixXd(rows, cols);

    if (in.is_open()) {

        while (std::getline(in, line)) {

            char* ptr = (char*)line.c_str();
            int len = line.length();

            col = 0;

            char* start = ptr;
            for (int i = 0; i < len; i++) {

                if (ptr[i] == ',') {
                    res(row, col++) = atof(start);
                    start = ptr + i + 1;
                }
            }
            res(row, col) = atof(start);

            row++;
        }

        in.close();
    }
    return res;
}

void AUnrealSimGameMode::BeginPlay()
{
    UE_LOG(LogTemp, Display, TEXT("GameMode Started"));

    PrimaryActorTick.bStartWithTickEnabled = true;
    PrimaryActorTick.bCanEverTick = true;

    SimulationSceneManager.InitializeScene();

    for(int itr = 0; itr < SimulationSceneManager.SceneObjects.Num(); itr++)
    {
        SpawnMotionObject(SimulationSceneManager.SceneObjects[itr]);
    }

    JointItr.Init(0, SimulationSceneManager.SceneObjects.Num());
    JointVal.Init(0.0, SimulationSceneManager.SceneObjects.Num());
    JointDelta.Init(1, SimulationSceneManager.SceneObjects.Num());

    //Uncomment This
    //GetWorldTimerManager().SetTimer(ExampleTimerHandle, this, &AUnrealSimGameMode::ExampleTimerCallback, 0.01f, true, 2.0f);

    //GetWorldTimerManager().SetTimer(CameraTimerHandle, this, &AUnrealSimGameMode::SaveImagesTimerCallback, 0.2f, true, 2.0f);



    // sdf::Root root;
    // std::string sdf_filepath = "C:/Users/irsl_/Documents/Unreal Projects/UnrealSim/Source/UnrealSim/world_complete.sdf";

    // sdf::Errors errors = root.Load(sdf_filepath);

    // FString LogText;

    // if(errors.empty())
    // {
    //     UE_LOG(LogTemp, Display, TEXT("SDF file meets specification requirements"));
    //     UE_LOG(LogTemp, Display, TEXT("Number of models : %d"), root.ModelCount());
    //     UE_LOG(LogTemp, Display, TEXT("Number of worlds : %d"), root.WorldCount());

    //     const sdf::World* sdf_world = root.WorldByIndex(0);
    
    //     if(sdf_world)
    //     {
    //         //LogText = FString(UTF8_TO_TCHAR(sdf_world->Name().c_str()));
    //         //LogText = sdf_world->Name().c_str();
    //         //UE_LOG(LogTemp, Display, TEXT("Name of world : %s"), *LogText);
            
    //         UE_LOG(LogTemp, Display, TEXT("World light count : %d"), sdf_world->LightCount());
            
    //         for(int i = 0; i < sdf_world->LightCount(); i++)
    //         {
    //             const sdf::Light* sdf_light = sdf_world->LightByIndex(i);
                
    //             if(sdf_light)
    //             {
    //                 std::string temp = sdf_light->Name(); 
    //                 LogText = FString(temp.c_str());
    //                 //LogText = sdf_light->Name();
    //                 //UE_LOG(LogTemp, Display, TEXT("Light %d name : %s"), sdf_world->LightCount(), *LogText);
    //                 //UE_LOG(LogTemp, Display, TEXT("Light %d name : %s"), i, (sdf_light->Name()).c_str());
    //             }
    //         }
    //     }
    // } 
    // else
    // {
    //      UE_LOG(LogTemp, Error, TEXT("SDF ERROR"));
    // }

    ResetJointPos = true;
}

void AUnrealSimGameMode::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    // Update actor positions if their state has changed
    for(int32 actor_itr = 0; actor_itr < SceneActors.Num(); actor_itr++)
    {
        Eigen::Matrix4d VisualActorG = *SceneActors[actor_itr]->ActorG * SceneActors[actor_itr]->MObjectVisual->VisualOriginG;
        FVector ActorLoc = MotionEngine::MotionEngineStatics::ROSToUnreal(VisualActorG.block<3,1>(0,3));

        Eigen::Quaterniond OrientationQuat(VisualActorG.block<3,3>(0,0));
        FQuat ActorRot = MotionEngine::MotionEngineStatics::ROSToUnreal(OrientationQuat);

        SceneActors[actor_itr]->SetActorLocationAndRotation(ActorLoc, ActorRot);
       /* if (SceneActors[actor_itr]->GetName() == "ArticulatedMotionObject_Chairbase") {
            GLog->Log("*******************************************************************************************");
            GLog->Log(SceneActors[actor_itr]->GetName());
            GLog->Log(SceneActors[actor_itr]->GetActorLocation().ToString());
            GLog->Log("*******************************************************************************************");
        }*/
    }

}

void AUnrealSimGameMode::ExampleTimerCallback()
{
    for(int32 itr = 0; itr < SimulationSceneManager.SceneObjects.Num(); itr++)
    {
        if(SimulationSceneManager.SceneObjects[itr]->ObjectType == MotionEngine::MotionObject::MotionObjectType::ARTICULATED_MOTION_OBJECT)
        {
            TSharedPtr<MotionEngine::ArticulatedMotionObject> ArticulatedMObject(StaticCastSharedPtr<MotionEngine::ArticulatedMotionObject>(SimulationSceneManager.SceneObjects[itr]));

                if ((ArticulatedMObject->Joints[JointItr[itr]]->Type == MotionEngine::Joint::REVOLUTE) || (ArticulatedMObject->Joints[JointItr[itr]]->Type == MotionEngine::Joint::PRISMATIC) || (ArticulatedMObject->Joints[JointItr[itr]]->Type == MotionEngine::Joint::CONTINUOUS))
                {
                    TMap<FString, double> JointPosition;
                    double JointPos;
                    FString JointName;

                    bool OutOfBounds = false;

                    double MinJointVal = ArticulatedMObject->Joints[JointItr[itr]]->Limits.LowerLimit;
                    double MaxJointVal = ArticulatedMObject->Joints[JointItr[itr]]->Limits.UpperLimit;

                    JointName = ArticulatedMObject->Joints[JointItr[itr]]->Name;

                    UE_LOG(LogTemp, Display, TEXT("joint %s with index %d and joint pos %f"), *JointName, JointItr[itr], JointVal[itr]);

                    JointVal[itr] = JointVal[itr] + (((MaxJointVal - MinJointVal) / 500) * JointDelta[itr]);
                    JointPos = JointVal[itr];

                    if (JointVal[itr] >= MaxJointVal)
                    {
                        UE_LOG(LogTemp, Display, TEXT("Reached maximum for joint %s with index %d and joint pos %f"), *JointName, JointItr[itr], JointVal[itr]);

                        JointVal[itr] = MaxJointVal;
                        JointPos = JointVal[itr];
                        JointDelta[itr] = -1;
                        OutOfBounds = true;
                    }
                    else if (JointVal[itr] <= MinJointVal)
                    {
                        UE_LOG(LogTemp, Display, TEXT("Reached minimum for joint %s with index %d and joint pos %f"), *JointName, JointItr[itr], JointVal[itr]);
                        JointVal[itr] = 0;
                        JointPos = JointVal[itr];
                        JointDelta[itr] = 1;
                        JointItr[itr]++;

                        if (ArticulatedMObject->ObjectName == TEXT("j2n7s300"))
                        {
                            switch (JointItr[itr])
                            {
                            case 3:
                            case 5:
                                UE_LOG(LogTemp, Display, TEXT("Setting pos for joint after %s with index %d"), *JointName, JointItr[itr]);
                                JointVal[itr] = M_PI;
                                JointPos = JointVal[itr];
                                break;
                            default:
                                JointVal[itr] = 0;
                                break;
                            }
                        }

                        ResetJointPos = true;

                        TMap<FString, double> InitialJointPositions;

                        InitialJointPositions.Add(FString("j2n7s300_joint_1"), 0.0);
                        InitialJointPositions.Add(FString("j2n7s300_joint_2"), M_PI);
                        InitialJointPositions.Add(FString("j2n7s300_joint_3"), 0.0);
                        InitialJointPositions.Add(FString("j2n7s300_joint_4"), M_PI);
                        InitialJointPositions.Add(FString("j2n7s300_joint_5"), 0.0);
                        InitialJointPositions.Add(FString("j2n7s300_joint_6"), 0.0);
                        InitialJointPositions.Add(FString("j2n7s300_joint_7"), 0.0);
                        InitialJointPositions.Add(FString("j2n7s300_joint_finger_1"), 0.0);
                        InitialJointPositions.Add(FString("j2n7s300_joint_finger_tip_1"), 0.0);
                        InitialJointPositions.Add(FString("j2n7s300_joint_finger_2"), 0.0);
                        InitialJointPositions.Add(FString("j2n7s300_joint_finger_tip_2"), 0.0);
                        InitialJointPositions.Add(FString("j2n7s300_joint_finger_3"), 0.0);
                        InitialJointPositions.Add(FString("j2n7s300_joint_finger_tip_3"), 0.0);


                        //SimulationSceneManager.ArticulatedObjectTreeSolvers[TEXT("j2n7s300")]->SolvePositionFK(InitialJointPositions);

                        UE_LOG(LogTemp, Display, TEXT("Moving to initial configuration"));

                        OutOfBounds = true;
                    }

                    if (!OutOfBounds)
                    {
                        JointPosition.Add(JointName, JointPos);
                        SimulationSceneManager.ArticulatedObjectTreeSolvers[ArticulatedMObject->ObjectName]->SolvePositionFK(JointPosition);
                    }
                }
                else
                {
                    JointItr[itr]++;
                }

                if (JointItr[itr] >= ArticulatedMObject->Joints.Num())
                {
                    JointItr[itr] = 0;
                }
        }
    }
}



void AUnrealSimGameMode::SaveImagesTimerCallback()
{
    //UE_LOG(LogTemp, Display, TEXT("Saving Image"));

    RGBDSensor->SaveRGBImage(FString::SanitizeFloat(ImageNumber));
    RGBDSensor->SaveDepthImage(FString::SanitizeFloat(ImageNumber));
    ImageNumber += 1;
}


void AUnrealSimGameMode::SpawnMotionObject(TSharedPtr<MotionEngine::MotionObject> MObject)
{
    if(MObject)
    {
        if(MObject->ObjectType == MotionEngine::MotionObject::MotionObjectType::SIMPLE_MOTION_OBJECT)
        {
            TSharedPtr<MotionEngine::SimpleMotionObject> SimpleMObject(StaticCastSharedPtr<MotionEngine::SimpleMotionObject>(MObject));

            if(SimpleMObject->ObjectLink)
            {
                if(SimpleMObject->ObjectLink->LinkVisual)
                {
                    if(SimpleMObject->ObjectLink->LinkVisual->VisualGeometry)
                    {
                        if(SimpleMObject->ObjectLink->LinkVisual->VisualGeometry->Type == MotionEngine::Geometry::MESH)
                        {
                            FString CurrentLinkName = SimpleMObject->ObjectName;
                            FVector ActorSpawnLoc;
                            FRotator ActorSpawnRot;
                            FQuat ActorSpawnRotQuat;
                            FActorSpawnParameters ActorSpawnParams;

                            ActorSpawnParams.Name = FName(TEXT("SimpleMotionObject_") + CurrentLinkName);

                            Eigen::Matrix4d CurrentLinkG = *SimpleMObject->ObjectLinkState->LinkG;
                            CurrentLinkG = CurrentLinkG * SimpleMObject->ObjectLink->LinkVisual->VisualOriginG;
                            Eigen::Quaterniond CurrentLinkRot(CurrentLinkG.block<3,3>(0,0));

                            ActorSpawnLoc = MotionEngine::MotionEngineStatics::ROSToUnreal(CurrentLinkG.block<3,1>(0,3));
                            ActorSpawnRotQuat = MotionEngine::MotionEngineStatics::ROSToUnreal(CurrentLinkRot);

                            ActorSpawnRot = ActorSpawnRotQuat.Rotator();

                            AMotionObjectActor* SpawnedActor = GetWorld()->SpawnActor<AMotionObjectActor>(ActorSpawnLoc, ActorSpawnRot, ActorSpawnParams);
                            SpawnedActor->InitializeActorProperties(SimpleMObject->ObjectLink->LinkVisual, SimpleMObject->ObjectLinkState->LinkG, CurrentLinkName);
                            SpawnedActor->SceneObjectPtr = MObject;
                            SceneActors.Push(SpawnedActor);
                        }
                    }
                }
            }
        }
        else if(MObject->ObjectType == MotionEngine::MotionObject::MotionObjectType::ARTICULATED_MOTION_OBJECT)
        {
            TSharedPtr<MotionEngine::ArticulatedMotionObject> ArticulatedMObject(StaticCastSharedPtr<MotionEngine::ArticulatedMotionObject>(MObject));

            // Spawn actors for links which have visual information
            for(int32 i = 0; i < ArticulatedMObject->Links.Num(); i++)
            {
                if(ArticulatedMObject->Links[i]->LinkVisual)
                {
                    if(ArticulatedMObject->Links[i]->LinkVisual->VisualGeometry)
                    {
                        if(ArticulatedMObject->Links[i]->LinkVisual->VisualGeometry->Type == MotionEngine::Geometry::MESH)
                        {
                            FString CurrentLinkName = ArticulatedMObject->Links[i]->Name;
                            FVector ActorSpawnLoc;
                            FRotator ActorSpawnRot;
                            FQuat ActorSpawnRotQuat;
                            FActorSpawnParameters ActorSpawnParams;

                            ActorSpawnParams.Name = FName(TEXT("ArticulatedMotionObject_") + ArticulatedMObject->ObjectName + ArticulatedMObject->Links[i]->Name);

                            Eigen::Matrix4d CurrentLinkG = *ArticulatedMObject->ObjectTree->TreeLinkStateMap[CurrentLinkName]->LinkG;
                            CurrentLinkG = CurrentLinkG * ArticulatedMObject->Links[i]->LinkVisual->VisualOriginG;
                            Eigen::Quaterniond CurrentLinkRot(CurrentLinkG.block<3,3>(0,0));

                            ActorSpawnLoc = MotionEngine::MotionEngineStatics::ROSToUnreal(CurrentLinkG.block<3,1>(0,3));
                            ActorSpawnRotQuat = MotionEngine::MotionEngineStatics::ROSToUnreal(CurrentLinkRot);

                            ActorSpawnRot = ActorSpawnRotQuat.Rotator();

                            AMotionObjectActor* SpawnedActor = GetWorld()->SpawnActor<AMotionObjectActor>(ActorSpawnLoc, ActorSpawnRot, ActorSpawnParams);
                            SpawnedActor->InitializeActorProperties(ArticulatedMObject->Links[i]->LinkVisual, ArticulatedMObject->ObjectTree->TreeLinkStateMap[CurrentLinkName]->LinkG, CurrentLinkName);
                            SpawnedActor->SceneObjectPtr = MObject;
                            SpawnedActor->SceneObjectPtr2 = ArticulatedMObject;
                            SpawnedActor->ObjectTreeSolverPtr = SimulationSceneManager.ArticulatedObjectTreeSolvers[ArticulatedMObject->ObjectName];
                            SceneActors.Push(SpawnedActor);
                        }
                    }
                }
            }
        }
    }
}


void AUnrealSimGameMode::MobileRobotTimerCallback()
{
    TSharedPtr<MotionEngine::ArticulatedMotionObject> MobileRobotMObject(StaticCastSharedPtr<MotionEngine::ArticulatedMotionObject>(SimulationSceneManager.SceneObjectsMap[TEXT("mobile_manipulator")]));
    TSharedPtr<MotionEngine::ArticulatedMotionObject> PedestalMObject(StaticCastSharedPtr<MotionEngine::ArticulatedMotionObject>(SimulationSceneManager.SceneObjectsMap[TEXT("pedestal")]));

    Eigen::Matrix4d MobileBaseG = *MobileRobotMObject->ObjectTree->TreeLinkStateMap[TEXT("panda_link0")]->LinkG;

    if (GoalItr < MobileBaseMotion.Num())
    {
        FString RootElementName = MobileRobotMObject->ObjectTree->RootElement->Name;

        eigen_ext::DualQuat StartPose(MobileBaseMotion[GoalItr - 1]);
        //eigen_ext::DualQuat CurrentPose(*MobileRobotMObject->ObjectTree->TreeLinkStateMap[RootElementName]->LinkG);
        eigen_ext::DualQuat GoalPose(MobileBaseMotion[GoalItr]);
        double Tau = 0.01 * InterpolationItr;
        //double Tau = 0.1 * InterpolationItr;

        if (InterpolationItr <= 100)
            //if(InterpolationItr < 10)
        {
            InterpolationItr++;
        }
        else
        {
            InterpolationItr = 0;
            GoalItr++;
        }

        //eigen_ext::DualQuat InterpolatedGoalPose = eigen_ext::DualQuat::dualQuatInterpolation(CurrentPose, GoalPose, Tau);
        eigen_ext::DualQuat InterpolatedGoalPose = eigen_ext::DualQuat::dualQuatInterpolation(StartPose, GoalPose, Tau);

        Eigen::Matrix4d NewPose = InterpolatedGoalPose.getTransform();
        TMap<FString, double> JointPosition;
        SimulationSceneManager.ArticulatedObjectTreeSolvers[MobileRobotMObject->ObjectName]->SolvePositionFK(JointPosition, NewPose);
    }
    else
    {
        if (PlanMotion)
        {
            if (EEGoalPoseItr < TaskTrajectory.Num())
            {
                MotionPlan.clear();
                MotionPlanItr = 0;

                Eigen::Matrix4d InitialG;
                RobotKinSolver.getFK(ManipJointValuesVector, InitialG);

                kinlib::ErrorCodes PlanResult = RobotKinSolver.getMotionPlan(ManipJointValuesVector, InitialG, TaskTrajectory[EEGoalPoseItr], MotionPlan);

                switch (PlanResult)
                {
                case kinlib::ErrorCodes::OPERATION_SUCCESS:
                    UE_LOG(LogTemp, Display, TEXT("Motion Plan : Success"), MotionPlan.size());
                    break;
                case kinlib::ErrorCodes::JOINT_LIMIT_ERROR:
                    UE_LOG(LogTemp, Error, TEXT("Motion Plan : Joint Limit Error"), MotionPlan.size());
                    break;
                default:
                    UE_LOG(LogTemp, Error, TEXT("Motion Plan : Failed"), MotionPlan.size());
                }

                UE_LOG(LogTemp, Display, TEXT("Motion Plan Length : %d"), MotionPlan.size());

                //EEGoalPoseItr++;

                PlanMotion = false;
            }
        }
        else
        {
            if (MotionPlanItr < MotionPlan.size())
            {
                ManipJointValuesVector = MotionPlan[MotionPlanItr];

                for (int32 i = 0; i < 7; i++)
                {
                    ManipJointValues[ManipulatorJoints[i]] = ManipJointValuesVector(i);
                }

                SimulationSceneManager.ArticulatedObjectTreeSolvers[MobileRobotMObject->ObjectName]->SolvePositionFK(ManipJointValues);

                MotionPlanItr++;
            }
            else
            {
                if (EEGoalPoseItr != 1)
                {
                    EEGoalPoseItr++;
                    PlanMotion = true;
                }
                else
                {
                    if (EEGoalPoseItr == 1)
                    {
                        if (GripperPosition >= 0.0025)
                        {
                            GripperPosition = GripperPosition - 0.001;
                            DelayItr = 0;
                        }
                        else
                        {
                            if (DelayItr > 15)
                            {
                                EEGoalPoseItr++;
                                PlanMotion = true;

                                Eigen::Matrix4d Drawer4Pose = *PedestalMObject->ObjectTree->TreeLinkStateMap[TEXT("drawer_4_handle")]->LinkG;
                                Eigen::Matrix4d PandaFingerPose = *MobileRobotMObject->ObjectTree->TreeLinkStateMap[TEXT("panda_leftfinger")]->LinkG;
                                RefG = Drawer4Pose;
                                DiffG = Eigen::Matrix4d::Identity();
                                DiffG.block<3, 1>(0, 3) = PandaFingerPose.block<3, 1>(0, 3) - Drawer4Pose.block<3, 1>(0, 3);

                                UE_LOG(LogTemp, Display, TEXT("Diff : %f\t%f\t%f"), DiffG(0, 3), DiffG(1, 3), DiffG(2, 3));

                                UE_LOG(LogTemp, Display, TEXT("PandaFingerPos : %f\t%f\t%f"), PandaFingerPose(0, 3), PandaFingerPose(1, 3), PandaFingerPose(2, 3));
                                UE_LOG(LogTemp, Display, TEXT("Drawer4Pos     : %f\t%f\t%f"), Drawer4Pose(0, 3), Drawer4Pose(1, 3), Drawer4Pose(2, 3));
                            }

                            DelayItr = DelayItr + 1;
                        }
                        GripperPositions[TEXT("panda_finger_joint1")] = GripperPosition;
                        GripperPositions[TEXT("panda_finger_joint2")] = GripperPosition;
                        SimulationSceneManager.ArticulatedObjectTreeSolvers[MobileRobotMObject->ObjectName]->SolvePositionFK(GripperPositions);
                    }
                }
            }
        }

        if (EEGoalPoseItr > 1)
        {
            if (PedestalMObject)
            {
                Eigen::Matrix4d Drawer4Pose = *PedestalMObject->ObjectTree->TreeLinkStateMap[TEXT("drawer_4_handle")]->LinkG;
                Eigen::Matrix4d PandaFingerPose = *MobileRobotMObject->ObjectTree->TreeLinkStateMap[TEXT("panda_leftfinger")]->LinkG;

                Eigen::Matrix4d TempG = Eigen::Matrix4d::Identity();
                TempG.block<3, 1>(0, 3) = PandaFingerPose.block<3, 1>(0, 3) - DiffG.block<3, 1>(0, 3);

                //UE_LOG(LogTemp, Display, TEXT("TempG : %f\t%f\t%f"), DiffG(0,3), DiffG(1,3), DiffG(2,3));

                double Drawer4SliderValue = abs(RefG(1, 3) - TempG(1, 3));
                //double Drawer4SliderValue = PandaFingerPose(1,3) - 0.0817;
                //UE_LOG(LogTemp, Display, TEXT("Drawer4SliderValue : %f"), Drawer4SliderValue);

                TMap<FString, double> DrawerPositions;
                DrawerPositions.Add(FString("joint_4"), Drawer4SliderValue);
                SimulationSceneManager.ArticulatedObjectTreeSolvers[PedestalMObject->ObjectName]->SolvePositionFK(DrawerPositions);
            }
        }
    }

    /*
    if(ImageDelayItr > 5)
    {
        RGBDSensor->SaveRGBImage(FString::SanitizeFloat(ImageNumber));
        ImageNumber += 1;
        ImageDelayItr = 0;
    }
    */

    //RGBDSensor->SaveRGBImage(FString::SanitizeFloat(ImageNumber));
    //ImageNumber += 1;

}