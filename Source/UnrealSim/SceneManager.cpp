#include "SceneManager.h"
#include "MotionObject.h"
#include "SimpleMotionObject.h"
#include "ArticulatedMotionObject.h"
#include "Link.h"
#include "Joint.h"
#include "Tree.h"
#include "URDFParser.h"
#include "SDFParser.h"


#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <math.h>

DEFINE_LOG_CATEGORY(MotionEngineSceneManagerLog);

namespace MotionEngine
{

SceneManager::SceneManager()
{
    bIsInitialized = false;
}

SceneManager::~SceneManager()
{

}


bool SceneManager::InitializeScene(void)
{
    FString ContentDir = FPaths::ProjectContentDir();
    FString ContentDirAbsolutePath = IFileManager::Get().ConvertToAbsolutePathForExternalAppForRead(*ContentDir);
    FString XMLFilePath = ContentDirAbsolutePath + TEXT("robot.xml");
    //LoadRobotsFromXML(XMLFilePath);
    /*
    FString RobotURDFFilePath = ContentDirAbsolutePath + TEXT("RobotDescription/urdf/robot_description.urdf");
    FString RobotResourcesDir = TEXT("/Game/RobotDescription");
    // The base position of the robot
    Eigen::Matrix4d RobotBase = Eigen::Matrix4d::Identity();
    RobotBase.block<3,3>(0,0) = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()) * RobotBase.block<3,3>(0,0);
    RobotBase(0,3) = 2.5;
    RobotBase(1, 3) = 0;
    RobotBase(2,3) = 0.925;

    FString HuskyURDFFilePath = ContentDirAbsolutePath + TEXT("RobotDescription/urdf/mobile_manipulator.urdf");
    FString HuskyResourcesDir = TEXT("/Game/RobotDescription");
    // The base position of the robot
    Eigen::Matrix4d HuskyBase = Eigen::Matrix4d::Identity();
    HuskyBase.block<3,3>(0,0) = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()) * HuskyBase.block<3,3>(0,0);
    HuskyBase(0,3) = -2.5;
    HuskyBase(2,3) = 0.13228;


    
    FString KinovaURDFFilePath = ContentDirAbsolutePath + TEXT("RobotDescription/kinova/urdf/j2n7s300_standalone.urdf");
    FString KinovaResourcesDir = TEXT("/Game/RobotDescription");
    // The base position of the robot
    Eigen::Matrix4d KinovaBase = Eigen::Matrix4d::Identity();
    KinovaBase.block<3, 3>(0, 0) = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()) * KinovaBase.block<3, 3>(0, 0);
    KinovaBase(0, 3) = 0;
    KinovaBase(1, 3) = 2.5;
    KinovaBase(2, 3) = 0.65;
    


    FString UR10URDFFilePath = ContentDirAbsolutePath + TEXT("RobotDescription/urdf/ur10_robot.urdf");
    FString UR10ResourcesDir = TEXT("/Game/RobotDescription");
    // The base position of the robot
    Eigen::Matrix4d UR10Base = Eigen::Matrix4d::Identity();
    UR10Base.block<3, 3>(0, 0) = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()) * UR10Base.block<3, 3>(0, 0);
    UR10Base(0, 3) = 0;
    UR10Base(1, 3) = -2.5;
    UR10Base(2, 3) = 0.85;
    */
    

    UE_LOG(LogTemp, Display, TEXT("Initializing Scene"));

    //FString PedestalFilePath = ContentDirAbsolutePath + TEXT("Objects/Pedestal/urdf/pedestal.urdf");
    //FString PedestalResourcesDir = TEXT("/Game/Objects/Pedestal");
    //// The base position of the pedestal
    //Eigen::Matrix4d PedestalBase = Eigen::Matrix4d::Identity();
    //PedestalBase.block<3,3>(0,0) = Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitZ()) * PedestalBase.block<3,3>(0,0);
    //PedestalBase(0,3) = 0.75;
    //PedestalBase(1,3) = 2;
    //PedestalBase(2,3) = 0;

    //FString CheezItBoxPath = ContentDirAbsolutePath + TEXT("Objects/CheezIt_Box/urdf/cheezit_box.urdf");
    //FString CheezItBoxResourcesDir = TEXT("/Game/Objects/CheezIt_Box");
    //// The base position of the pedestal
    //Eigen::Matrix4d CheezItBoxBase = Eigen::Matrix4d::Identity();
    ////CheezItBoxBase.block<3,3>(0,0) = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()) * CheezItBoxBase.block<3,3>(0,0);
    //CheezItBoxBase(0,3) = 0.75;
    //CheezItBoxBase(1,3) = 1;
    //CheezItBoxBase(2,3) = 0.5;

    FString SpoonPath = ContentDirAbsolutePath + TEXT("Objects/Spoon/urdf/Soup_Spoon.urdf");
    FString SpoonResourcesDir = TEXT("/Game/Objects/Spoon");
    Eigen::Matrix4d SpoonBase = Eigen::Matrix4d::Identity();

    FString SceneSDFFilePath = ContentDirAbsolutePath + TEXT("SceneDescription/sdf/scene2.sdf");
    FString SceneResourcesDir = TEXT("/Game/SceneDescription");
    // The base position of the pedestal
    // Eigen::Matrix4d SceneTestingBase = Eigen::Matrix4d::Identity();
    //CheezItBoxBase.block<3,3>(0,0) = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()) * CheezItBoxBase.block<3,3>(0,0);
    // SceneTestingBase(0,3) = 1.75;
    // SceneTestingBase(1,3) = 1;
    // SceneTestingBase(2,3) = 0.5;

    UE_LOG(LogTemp, Display, TEXT("Loading SDF"));


    if (!AddMObjectsToScene(SceneSDFFilePath, SceneResourcesDir))
    {
        UE_LOG(LogTemp, Error, TEXT("Cannot load scene SDF"));
        return false;
    } 
    /*

   if(!AddRobotToScene(RobotURDFFilePath, RobotResourcesDir, RobotBase))
    {
        return false;
    }

    if(!AddRobotToScene(HuskyURDFFilePath, HuskyResourcesDir, HuskyBase))
    {
        UE_LOG(LogTemp, Error, TEXT("Cannot load Husky"));
        return false;
    }
    else
    {
        if(ArticulatedObjectTreeSolvers.Contains(TEXT("mobile_manipulator")))
        {
            UE_LOG(LogTemp, Display, TEXT("Solving FK for initial configuration"));
            TMap<FString, double> InitialJointPositions;

            InitialJointPositions.Add(FString("panda_joint1"), 0.0);
            InitialJointPositions.Add(FString("panda_joint2"), -M_PI_4);
            InitialJointPositions.Add(FString("panda_joint3"), 0.0);
            InitialJointPositions.Add(FString("panda_joint4"), -3 * M_PI_4);
            InitialJointPositions.Add(FString("panda_joint5"), 0.0);
            InitialJointPositions.Add(FString("panda_joint6"), M_PI_2);
            InitialJointPositions.Add(FString("panda_joint7"), M_PI_4);

            ArticulatedObjectTreeSolvers[TEXT("mobile_manipulator")]->SolvePositionFK(InitialJointPositions);
        }
    } 
    
    if (!AddRobotToScene(UR10URDFFilePath, UR10ResourcesDir, UR10Base))
    {
        return false;
    } 
    else
    {
        if (ArticulatedObjectTreeSolvers.Contains(TEXT("ur10")))
        {
            UE_LOG(LogTemp, Display, TEXT("Solving FK for initial configuration"));
            TMap<FString, double> InitialJointPositions;

            InitialJointPositions.Add(FString("shoulder_pan_joint"), 0.0);
            InitialJointPositions.Add(FString("shoulder_lift_joint"), 0.0);
            InitialJointPositions.Add(FString("elbow_joint"), 0.0);
            InitialJointPositions.Add(FString("wrist_1_joint"), 0.0);
            InitialJointPositions.Add(FString("wrist_2_joint"), 0.0);
            InitialJointPositions.Add(FString("wrist_3_joint"), 0.0);

            ArticulatedObjectTreeSolvers[TEXT("ur10")]->SolvePositionFK(InitialJointPositions);
        }
    }

    if (!AddRobotToScene(KinovaURDFFilePath, KinovaResourcesDir, KinovaBase))
    {
        return false;
    }
    else
    {
        if (ArticulatedObjectTreeSolvers.Contains(TEXT("j2n7s300")))
        {
            UE_LOG(LogTemp, Display, TEXT("Solving FK for initial configuration"));
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
            

            ArticulatedObjectTreeSolvers[TEXT("j2n7s300")]->SolvePositionFK(InitialJointPositions);
        }
    }
    */

    // if(!AddArticulatedObjectToScene(PedestalFilePath, PedestalResourcesDir, PedestalBase))
    // {
    //     return false;
    // }

    // if(!AddSimpleObjectToScene(CheezItBoxPath, CheezItBoxResourcesDir, CheezItBoxBase))
    // {
    //     return false;
    // }
    
    if(SceneObjectsMap.Contains(TEXT("mobile_manipulator")))
    {
        TSharedPtr<MotionEngine::ArticulatedMotionObject> MobileRobotMObject(StaticCastSharedPtr<MotionEngine::ArticulatedMotionObject>(SceneObjectsMap[TEXT("mobile_manipulator")]));
        if(MobileRobotMObject->ObjectTree->TreeLinkStateMap.Contains(TEXT("panda_hand")))
        {
            SpoonBase = *MobileRobotMObject->ObjectTree->TreeLinkStateMap["panda_hand"]->LinkG;
            SpoonBase(2,3) = SpoonBase(2,3) - 0.09;

            Eigen::AngleAxisd RotZ(M_PI_2, Eigen::Vector3d::UnitZ());
            SpoonBase.block<3,3>(0,0) = SpoonBase.block<3,3>(0,0) * RotZ;
        }
    }
    else
    {
        UE_LOG(LogTemp, Error, TEXT("mobile_manipulator not found"));
    }

    // if(!AddSimpleObjectToScene(SpoonPath, SpoonResourcesDir, SpoonBase))
    // {
    //     return false;
    // }

    bIsInitialized = true;
    return true;
}

bool SceneManager::AddRobotToScene(FString URDFPath, FString ResourceDirectory, Eigen::Matrix4d RobotBaseG)
{   
    TSharedPtr<MotionObject> MObject;
    if(MotionEngine::URDFParser::ParseURDF(URDFPath, ResourceDirectory, MObject))
    {
        TSharedPtr<ArticulatedMotionObject> ArticulatedMObject(StaticCastSharedPtr<ArticulatedMotionObject>(MObject));

        if(!ArticulatedMObject->ObjectTree->InitializeTree(TEXT("base"), RobotBaseG))
        {
            return false;
        }

        TSharedPtr<TreeRecursiveSolver> ArticulatedMObjectSolver = MakeShareable(new TreeRecursiveSolver());
        ArticulatedMObjectSolver->InitializeSolver(ArticulatedMObject->ObjectTree);

        ArticulatedObjectTreeSolvers.Add(ArticulatedMObject->ObjectName, ArticulatedMObjectSolver);
    }
    else
    {
        return false;
    }

    SceneObjects.Push(MObject);
    SceneObjectsMap.Add(MObject->ObjectName, MObject);
    return true;
}

bool SceneManager::AddArticulatedObjectToScene(FString URDFPath, FString ResourceDirectory, Eigen::Matrix4d ObjectBaseG)
{
    TSharedPtr<MotionObject> MObject;
    if(MotionEngine::URDFParser::ParseURDF(URDFPath, ResourceDirectory, MObject))
    {
        TSharedPtr<ArticulatedMotionObject> ArticulatedMObject(StaticCastSharedPtr<ArticulatedMotionObject>(MObject));

        if(!ArticulatedMObject->ObjectTree->InitializeTree(TEXT("base"), ObjectBaseG))
        {
            return false;
        }

        TSharedPtr<TreeRecursiveSolver> ArticulatedMObjectSolver = MakeShareable(new TreeRecursiveSolver());
        ArticulatedMObjectSolver->InitializeSolver(ArticulatedMObject->ObjectTree);

        ArticulatedObjectTreeSolvers.Add(ArticulatedMObject->ObjectName, ArticulatedMObjectSolver);
    }
    else
    {
        return false;
    }

    SceneObjects.Push(MObject);
    SceneObjectsMap.Add(MObject->ObjectName, MObject);
    return true;
}



bool SceneManager::LoadRobotsFromXML(FString XMLFilePath)
{
    FString FilePath = XMLFilePath;
    FString ContentDir = FPaths::ProjectContentDir();
    FString ContentDirAbsolutePath = IFileManager::Get().ConvertToAbsolutePathForExternalAppForRead(*ContentDir);

    FXmlFile XmlFile;
    if (!XmlFile.LoadFile(FilePath, EConstructMethod::Type::ConstructFromFile))
    {
        UE_LOG(LogTemp, Error, TEXT("Failed to load XML file: %s"), *FilePath);
        return false;
    }

    FXmlNode* RootNode = XmlFile.GetRootNode();
    if (!RootNode)
    {
        UE_LOG(LogTemp, Error, TEXT("Failed to get root node of XML file: %s"), *FilePath);
        return false;
    }

    for (FXmlNode* RobotNode : RootNode->GetChildrenNodes())
    {
        FString UrdfFilePath = ContentDirAbsolutePath + RobotNode->FindChildNode(TEXT("urdf_file_path"))->GetContent();
        FString ResourcesDir = RobotNode->FindChildNode(TEXT("resources_dir"))->GetContent();
        FString name = RobotNode->FindChildNode(TEXT("name"))->GetContent();
        const TCHAR* LogText1;
        LogText1 = *UrdfFilePath;
        UE_LOG(LogTemp, Display, TEXT("This is the data: %s"), LogText1);

        FXmlNode* BasePoseNode = RobotNode->FindChildNode(TEXT("base_pose"));
        double Tx = FCString::Atof(*BasePoseNode->FindChildNode(TEXT("translation"))->GetAttribute(TEXT("x")));
        double Ty = FCString::Atof(*BasePoseNode->FindChildNode(TEXT("translation"))->GetAttribute(TEXT("y")));
        double Tz = FCString::Atof(*BasePoseNode->FindChildNode(TEXT("translation"))->GetAttribute(TEXT("z")));
        double Angle = FCString::Atof(*BasePoseNode->FindChildNode(TEXT("rotation"))->GetAttribute(TEXT("angle")));
        double Rx = FCString::Atof(*BasePoseNode->FindChildNode(TEXT("rotation"))->GetAttribute(TEXT("axis_x")));
        double Ry = FCString::Atof(*BasePoseNode->FindChildNode(TEXT("rotation"))->GetAttribute(TEXT("axis_y")));
        double Rz = FCString::Atof(*BasePoseNode->FindChildNode(TEXT("rotation"))->GetAttribute(TEXT("axis_z")));

        Eigen::Matrix4d BasePose = Eigen::Matrix4d::Identity();
        BasePose.block<3, 3>(0, 0) = Eigen::AngleAxisd(Angle, Eigen::Vector3d(Rx, Ry, Rz).normalized()) * BasePose.block<3, 3>(0, 0);
        BasePose(0, 3) = Tx;
        BasePose(1, 3) = Ty;
        BasePose(2, 3) = Tz;
        TMap<FString, double> JointPositions;
        const FXmlNode* InitialJointPositionsNode = RobotNode->FindChildNode("initial_joint_positions");
        if (InitialJointPositionsNode != nullptr) {
            for (const FXmlNode* JointNode : InitialJointPositionsNode->GetChildrenNodes()) {
                FString JointName = JointNode->GetAttribute("name");
                double JointPosition = FCString::Atof(*JointNode->GetContent());
                JointPositions.Add(JointName, JointPosition);
            }
        }
        
        if (!AddRobotToScene(UrdfFilePath, ResourcesDir, BasePose))
        {
            UE_LOG(LogTemp, Display, TEXT("This is the data inside if: %s"), LogText1);
            return false;
        }
        ArticulatedObjectTreeSolvers[name]->SolvePositionFK(JointPositions);

        // Use the parsed data as needed...
    }
    return true;
}


bool SceneManager::AddSimpleObjectToScene(FString URDFPath, FString ResourceDirectory, Eigen::Matrix4d ObjectBaseG)
{
    TSharedPtr<MotionObject> MObject;
    if(MotionEngine::URDFParser::ParseURDF(URDFPath, ResourceDirectory, MObject))
    {
        TSharedPtr<SimpleMotionObject> SimpleMObject(StaticCastSharedPtr<SimpleMotionObject>(MObject));

        if(SimpleMObject->ObjectLinkState)
        {
            if(SimpleMObject->ObjectLinkState->LinkG)
            {
                *SimpleMObject->ObjectLinkState->LinkG = ObjectBaseG;
            }
        }
    }
    else
    {
        return false;
    }

    SceneObjects.Push(MObject);
    SceneObjectsMap.Add(MObject->ObjectName, MObject);
    return true;
}

bool SceneManager::AddMObjectsToScene(FString SDFPath, FString ResourceDirectory)
{
    TArray< TSharedPtr<MotionObject> > MObjects;
    if(MotionEngine::SDFParser::ParseSDF(SDFPath, ResourceDirectory, MObjects))
    {
        for(auto MObject: MObjects){
            TSharedPtr<ArticulatedMotionObject> ArticulatedMObject(StaticCastSharedPtr<ArticulatedMotionObject>(MObject));

            if(!ArticulatedMObject->ObjectTree->InitializeTree(TEXT("base"), MObject->ObjectInitialPose))
            {
                UE_LOG(MotionEngineSceneManagerLog, Error, TEXT("ArticulatedMObject %s does not have a base link"), *ArticulatedMObject->ObjectName);
                return false;
            }
            TSharedPtr<TreeRecursiveSolver> ArticulatedMObjectSolver = MakeShareable(new TreeRecursiveSolver());
            ArticulatedMObjectSolver->InitializeSolver(ArticulatedMObject->ObjectTree);

            ArticulatedObjectTreeSolvers.Add(ArticulatedMObject->ObjectName, ArticulatedMObjectSolver); 
            
            SceneObjects.Push(MObject);
            SceneObjectsMap.Add(MObject->ObjectName, MObject);
        }
        
    }
    else
    {
        return false;
    }
    return true;
}

}