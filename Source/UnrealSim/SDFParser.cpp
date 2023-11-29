#include "SDFParser.h"
#include "MotionObject.h"
#include "MotionObjectProperties.h"
#include "SimpleMotionObject.h"
#include "ArticulatedMotionObject.h"
#include "Link.h"
#include "Joint.h"
#include "Tree.h"

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <math.h>

DEFINE_LOG_CATEGORY(SDFParserLog);

namespace MotionEngine
{

SDFParser::SDFParser()
{

}

SDFParser::~SDFParser()
{

}

Eigen::Vector3d SDFParser::ParseOriginAttributes(FString AttributeVal)
{
    Eigen::Vector3d Result = Eigen::Vector3d::Zero();

    int32 StrIdx = 0;
    FString Val;

    for(int itr = 0; itr < 3; itr++)
    {
        AttributeVal.FindChar(TEXT(' '), StrIdx);
        Val = AttributeVal.Left(StrIdx);
        Result[itr] = FCString::Atod(*Val);
        AttributeVal = AttributeVal.RightChop(StrIdx+1);
    }

    Result[2] = FCString::Atod(*AttributeVal);

    return Result;
}

bool SDFParser::ParseOriginAttributes(FString AttributeVal, Eigen::Vector3d &ElementG)
{
    int32 StrIdx = 0;
    FString Val;

    //"X=10.000 Y=10.000 Z=10.000"
    for(int itr = 0; itr < 3; itr++)
    {
        AttributeVal.FindChar(TEXT(' '), StrIdx);
        Val = AttributeVal.Left(StrIdx).RightChop(2);
        if(itr==2) Val = AttributeVal.RightChop(2);
        ElementG[itr] = FCString::Atod(*Val);
        AttributeVal = AttributeVal.RightChop(StrIdx+1);
    }

    //VisualOriginXYZ[2] = FCString::Atod(*AttributeVal);

    return true;
}

bool SDFParser::ParsePose(ignition::math::Quaterniond quat, ignition::math::Vector3d vec, Eigen::Vector3d &VisualOriginRPY, Eigen::Vector3d &VisualOriginXYZ, Eigen::Matrix4d &VisualOriginG)
{
    Eigen::Quaterniond VisualOriginQuat(quat.W(),quat.X(),quat.Y(),quat.Z());
    FString OriginXYZ = FVector(vec.X(),vec.Y(),vec.Z()).ToString();

    UE_LOG(SDFParserLog, Display, TEXT("Debuggin a new link with its OriginXYZ %s"), *OriginXYZ);

    ParseOriginAttributes(OriginXYZ, VisualOriginXYZ);

    VisualOriginG.block<3,1>(0,3) = VisualOriginXYZ;
    VisualOriginG.block<3,3>(0,0) = VisualOriginQuat.normalized().toRotationMatrix();
    UE_LOG(SDFParserLog, Display, TEXT("Debuggin a new link with its OriginG  : X = %f\tY = %f\tZ = %f"),  
            VisualOriginG(0,3), VisualOriginG(1,3), VisualOriginG(2,3));

    return true;
}

bool SDFParser::ParseGeometry(sdf::ElementPtr geometryElement, FString ResourceDirectory, TSharedPtr<Geometry> &NewGeometry)
{
    NewGeometry.Reset();

    if(geometryElement->HasElement("mesh"))
    {
        TSharedPtr<GeometryMesh> NewLinkGeoMesh(new GeometryMesh());

        NewLinkGeoMesh->Type = Geometry::MESH;

        std::string filename = geometryElement->GetElement("mesh")->Get<std::string>("uri");
        std::string abs_filepath = sdf::findFile(filename);

        FString MeshFilePath = UTF8_TO_TCHAR(geometryElement->GetElement("mesh")->Get<std::string>("uri").c_str());

        if(MeshFilePath.IsEmpty())
        {
            UE_LOG(SDFParserLog, Warning, TEXT("Mesh filename not provided with geometry information"));
        }
        else
        {
            int32 StrIdx;
            FString MeshName;

            UE_LOG(SDFParserLog, Display, TEXT("MeshFilePath        : %s"), *MeshFilePath);
            
            // Remove ROS Package Name
            StrIdx = MeshFilePath.Find(TEXT("/meshes/"), ESearchCase::IgnoreCase, ESearchDir::FromStart, 0);
            MeshName = MeshFilePath.RightChop(StrIdx+1);

            UE_LOG(SDFParserLog, Display, TEXT("MeshName            : %s"), *MeshName);

            // Remove file extension
            MeshName = MeshName.LeftChop(4);

            // Find name of mesh file
            MeshName.FindLastChar(TEXT('/'), StrIdx);
            NewLinkGeoMesh->MeshDirectory = ResourceDirectory + TEXT("/") + MeshName.Left(StrIdx);
            MeshName = MeshName.RightChop(StrIdx+1);
            NewLinkGeoMesh->MeshFileName = MeshName;

            UE_LOG(SDFParserLog, Display, TEXT("Link mesh directory : %s"), *NewLinkGeoMesh->MeshDirectory);
            UE_LOG(SDFParserLog, Display, TEXT("Link mesh filename  : %s"), *NewLinkGeoMesh->MeshFileName);
        }

        NewGeometry = NewLinkGeoMesh;
    }
    
    return true;
}

bool SDFParser::ParseVisual(sdf::ElementPtr visualElement, FString ResourceDirectory, TSharedPtr<Visual> &NewVisual)
{
    NewVisual.Reset();
    NewVisual = MakeShareable(new Visual());

    ignition::math::Pose3d poseElement = visualElement->Get<ignition::math::Pose3d>("pose");
    ParsePose(poseElement.Rot(), poseElement.Pos(), NewVisual->VisualOriginRPY, NewVisual->VisualOriginXYZ, NewVisual->VisualOriginG);
    sdf::ElementPtr geometryElement = visualElement->GetElement("geometry");
    if(geometryElement)
    {
        ParseGeometry(geometryElement, ResourceDirectory, NewVisual->VisualGeometry);
    }
    
    
    return true;
}

bool SDFParser::ParseLink(sdf::ElementPtr linkElement, FString ResourceDirectory, TSharedPtr<Link> &NewLink)
{
    NewLink.Reset();
    NewLink = MakeShareable<Link>(new Link());

    NewLink->Name = UTF8_TO_TCHAR(linkElement->Get<std::string>("name").c_str());

    UE_LOG(SDFParserLog, Display, TEXT("Parsing \"%s\" link information"), *NewLink->Name);

    sdf::ElementPtr visualElement = linkElement->GetElement("visual");

    if(visualElement)
    {
        ParseVisual(visualElement, ResourceDirectory, NewLink->LinkVisual);
    }
    // TODO : Add link inertial and collisioin information

    return true;
}

bool SDFParser::ParseJoint(sdf::ElementPtr jointElement, TSharedPtr<Joint> &NewJoint)
{
    NewJoint.Reset();
    NewJoint = MakeShareable<Joint>(new Joint());

    NewJoint->Name = UTF8_TO_TCHAR(jointElement->Get<std::string>("name").c_str());

    UE_LOG(SDFParserLog, Display, TEXT("Parsing \"%s\" joint informatioin"), *NewJoint->Name);

    FString JointType = UTF8_TO_TCHAR(jointElement->Get<std::string>("type").c_str());

    UE_LOG(SDFParserLog, Display, TEXT("Joint Type : %s"), *JointType);

    if(JointType == TEXT("revolute"))
    {
        NewJoint->Type = Joint::REVOLUTE;
    }
    else if(JointType == TEXT("prismatic"))
    {
        NewJoint->Type = Joint::PRISMATIC;
    }
    else if(JointType == TEXT("continuous"))
    {
        NewJoint->Type = Joint::CONTINUOUS;
    }
    else if(JointType == TEXT("fixed"))
    {
        NewJoint->Type = Joint::FIXED;
    }
    else
    {
        UE_LOG(SDFParserLog, Error, TEXT("SDF contains invalid/unknown joint type"));
        NewJoint->Type = Joint::UNKNOWN;
    }

    if((NewJoint->Type == Joint::REVOLUTE) || (NewJoint->Type == Joint::PRISMATIC) || (NewJoint->Type == Joint::CONTINUOUS))
    {
        sdf::JointAxis axisElement;
        axisElement.SetXyz(jointElement->GetElement("axis")->Get<ignition::math::Vector3d>("xyz"));
        if(true)
        {
            FString AxisXYZ = FVector(axisElement.Xyz().X(), axisElement.Xyz().Y(), axisElement.Xyz().Z()).ToString();
            UE_LOG(SDFParserLog, Display, TEXT("Raw Axis XYZ : %s"), *AxisXYZ);
            ParseOriginAttributes(AxisXYZ, NewJoint->JointAxis);
            UE_LOG(SDFParserLog, Display, TEXT("Axis X : %f\tY : %f\tZ : %f"), NewJoint->JointAxis[0], NewJoint->JointAxis[1], NewJoint->JointAxis[2]);
        }
        else
        {
            UE_LOG(SDFParserLog, Warning, TEXT("Joint axis not defined for joint \"%s\""), *NewJoint->Name);
            UE_LOG(SDFParserLog, Warning, TEXT("Joint axis defaulting to \"[1;0;0]\""))

            NewJoint->JointAxis = Eigen::Vector3d::Zero();
            NewJoint->JointAxis[0] = 1;
        }

        if(NewJoint->Type != Joint::CONTINUOUS)
        {
            sdf::ElementPtr limitElement = jointElement->GetElement("limit");
            if(limitElement)
            {
                FString LimitValue;
                LimitValue = UTF8_TO_TCHAR(limitElement->Get<std::string>("lower").c_str());
                NewJoint->Limits.LowerLimit = FCString::Atod(*LimitValue);
                LimitValue = UTF8_TO_TCHAR(limitElement->Get<std::string>("upper").c_str());
                NewJoint->Limits.UpperLimit = FCString::Atod(*LimitValue);

                UE_LOG(SDFParserLog, Display, TEXT("Joint Limits : Upper = %f\tLower : %f"), NewJoint->Limits.UpperLimit, NewJoint->Limits.LowerLimit);
            }
            else
            {
                UE_LOG(SDFParserLog, Warning, TEXT("Limits not defined for joint \"%s\""), *NewJoint->Name);
                UE_LOG(SDFParserLog, Warning, TEXT("Defaulting upper and lower joint limits to \"0\""));
            }
        }
        else
        {
            NewJoint->Limits.LowerLimit = 0;
            NewJoint->Limits.UpperLimit = 2 * M_PI;
        }
    }
    
    ignition::math::Pose3d poseElement = jointElement->Get<ignition::math::Pose3d>("pose");
    ParsePose(poseElement.Rot(), poseElement.Pos(), NewJoint->OriginRPY, NewJoint->OriginXYZ, NewJoint->OriginG);

    sdf::ElementPtr parentElement = jointElement->GetElement("parent");
    if(parentElement)
    {
        FString testName = UTF8_TO_TCHAR(jointElement->Get<std::string>("parent").c_str());
        NewJoint->ParentLinkName = testName;
        //NewJoint->ParentLinkName = UTF8_TO_TCHAR(parentElement->Get<std::string>("link").c_str());
        UE_LOG(SDFParserLog, Display, TEXT("Joint \"%s\" parent link : \"%s\""), *NewJoint->Name, *NewJoint->ParentLinkName);
    }
    else
    {
        UE_LOG(SDFParserLog, Warning, TEXT("Joint \"%s\" not provided with parent link information"));
    }

    sdf::ElementPtr childElement = jointElement->GetElement("child");
    if(childElement)
    {
        NewJoint->ChildLinkName = UTF8_TO_TCHAR(jointElement->Get<std::string>("child").c_str());
        UE_LOG(SDFParserLog, Display, TEXT("Joint \"%s\" child link  : \"%s\""), *NewJoint->Name, *NewJoint->ChildLinkName);
    }
    else
    {
        UE_LOG(SDFParserLog, Warning, TEXT("Joint \"%s\" not provided with child link information"));
    }

    return true;
}


bool SDFParser::ParseModel(sdf::ElementPtr modelElement, FString ResourceDirectory, TSharedPtr<MotionObject> &MObject)
{
    TSharedPtr<ArticulatedMotionObject> ArticulatedObject(new ArticulatedMotionObject());
    MObject.Reset();
    MObject = ArticulatedObject;
    std::string modelName = modelElement->Get<std::string>("name");
    ArticulatedObject->ObjectName = UTF8_TO_TCHAR(modelName.c_str());

    UE_LOG(SDFParserLog, Display, TEXT("Parsing model %s"), *ArticulatedObject->ObjectName);

    if (modelElement->GetElement("pose"))
    {
        ignition::math::Pose3d poseElement = modelElement->Get<ignition::math::Pose3d>("pose");

        ignition::math::Quaterniond quat = poseElement.Rot();
        ignition::math::Vector3d vec = poseElement.Pos();

        Eigen::Matrix4d ObjectInitialPose;
        Eigen::Vector3d ModelOriginXYZ;

        Eigen::Quaterniond IncludeOriginQuat(quat.W(), quat.X(), quat.Y(), quat.Z());
        FString OriginXYZ = FVector(vec.X(), vec.Y(), vec.Z()).ToString();

        ParseOriginAttributes(OriginXYZ, ModelOriginXYZ);

        ObjectInitialPose.block<3, 1>(0, 3) = ModelOriginXYZ;
        ObjectInitialPose.block<3, 3>(0, 0) = IncludeOriginQuat.toRotationMatrix();

        MObject->ObjectInitialPose = ObjectInitialPose;
    }

    for(sdf::ElementPtr linkElement = modelElement->GetElement("link"); linkElement != sdf::ElementPtr(nullptr);
        linkElement = linkElement->GetNextElement("link"))
    {
        TSharedPtr<Link> NewLink(new Link());
        ParseLink(linkElement, ResourceDirectory, NewLink);
        UE_LOG(SDFParserLog, Display, TEXT("Adding a new link named \"%s\" and its originG  : X = %f\tY = %f\tZ = %f"),  
            *NewLink->Name, NewLink->LinkVisual->VisualOriginG(0,3), NewLink->LinkVisual->VisualOriginG(1,3), NewLink->LinkVisual->VisualOriginG(2,3));
        
        ArticulatedObject->AddLink(NewLink.Get());
    }

    for(sdf::ElementPtr jointElement = modelElement->GetElement("joint"); jointElement != sdf::ElementPtr(nullptr);
        jointElement = jointElement->GetNextElement("joint"))
    {
        TSharedPtr<Joint> NewJoint(new Joint());
        ParseJoint(jointElement, NewJoint);
        ArticulatedObject->AddJoint(NewJoint.Get());
    }
    return true;
}

// parses the include tag and gets the pose of the objects with reference to the global frame
void SDFParser::ParseInclude(sdf::ElementPtr includeElement, FString ResourceDirectory, TSharedPtr<MotionObject>& MObject)
{
    ignition::math::Pose3d includePoseElement = includeElement->Get<ignition::math::Pose3d>("pose");

    ignition::math::Quaterniond quat = includePoseElement.Rot();
    ignition::math::Vector3d vec = includePoseElement.Pos();

    Eigen::Matrix4d ObjectInitialPose;
    Eigen::Vector3d IncludeOriginXYZ;

    Eigen::Quaterniond IncludeOriginQuat(quat.W(), quat.X(), quat.Y(), quat.Z());
    FString OriginXYZ = FVector(vec.X(), vec.Y(), vec.Z()).ToString();

    ParseOriginAttributes(OriginXYZ, IncludeOriginXYZ);

    ObjectInitialPose.block<3, 1>(0, 3) = IncludeOriginXYZ;
    ObjectInitialPose.block<3, 3>(0, 0) = IncludeOriginQuat.toRotationMatrix();

    MObject->ObjectInitialPose = ObjectInitialPose;
}


bool SDFParser::ParseSDF(FString SDFFilePath, FString ResourceDirectory, TArray< TSharedPtr<MotionObject> > &MObjects)
{
    const TCHAR* LogText;

    LogText = *SDFFilePath;
    UE_LOG(SDFParserLog, Display, TEXT("Loading SDF file from path : %s"), LogText);

    FString ContentDir = FPaths::ProjectContentDir();
    FString ContentDirAbsolutePath = IFileManager::Get().ConvertToAbsolutePathForExternalAppForRead(*ContentDir);

    FString SceneResourcesDir = ContentDirAbsolutePath + TEXT("SceneDescription/models");
    std::string resource_dir = TCHAR_TO_UTF8(*SceneResourcesDir);
    sdf::addURIPath("model://", resource_dir);

    sdf::Root root;
    std::string sdf_filepath = TCHAR_TO_UTF8(*SDFFilePath);

    sdf::Errors errors = root.Load(sdf_filepath);

    if(errors.empty())
    {
        UE_LOG(LogTemp, Display, TEXT("SDF file meets specification requirements"));
        UE_LOG(LogTemp, Display, TEXT("Number of models : %d"), root.ModelCount());
        UE_LOG(LogTemp, Display, TEXT("Number of worlds : %d"), root.WorldCount());

        sdf::SDFPtr sdfElement(new sdf::SDF());
        sdf::init(sdfElement);
        if (!sdf::readFile(sdf_filepath, sdfElement))
        {
            UE_LOG(LogTemp, Warning, TEXT("%s is not a valid SDF file!"), *SDFFilePath);
            return false;
        }

        // start parsing model
        const sdf::ElementPtr rootElement = sdfElement->Root();

        //std::string rootElementDesc = rootElement->GetDescription();
        //FString descFString = UTF8_TO_TCHAR(rootElementDesc.c_str());
        //UE_LOG(LogTemp, Warning, TEXT("rootElementDesc :\n %s"), *descFString);
        
        /*
        if (!rootElement->HasElement("world"))
        {
            UE_LOG(LogTemp, Warning, TEXT("%s is not a valid model SDF file!"), *SDFFilePath);
            return false;
        }
        */

        
        if (!rootElement->HasElement("include") || !rootElement->HasElement("model")) {
            UE_LOG(LogTemp, Warning, TEXT("%s is not a valid model SDF file!"), *SDFFilePath);
            return false;
        }
        

        //sdf::ElementPtr worldElement = rootElement->GetElement("world");

        for (sdf::ElementPtr modelElement = rootElement->GetElement("model"); modelElement != sdf::ElementPtr(nullptr); modelElement = modelElement->GetNextElement("model"))
        {
            TSharedPtr<MotionObject> MObject(new MotionObject());
            ParseModel(modelElement, ResourceDirectory, MObject);

            MObjects.Add(MObject);
        }

        /*
        for (sdf::ElementPtr includeElement = rootElement->GetElement("include"); includeElement && modelElement;
            includeElement = includeElement->GetNextElement("include"))
        {
            TSharedPtr<MotionObject> MObject(new MotionObject());
            ParseModel(modelElement, ResourceDirectory, MObject);
            if (modelElement->GetElement("pose"))
            {
                ignition::math::Pose3d includePoseElement = modelElement->Get<ignition::math::Pose3d>("pose");

            }

            if(includeElement->GetElement("pose"))
                ParseInclude(includeElement, ResourceDirectory, MObject);
            

            modelElement = modelElement->GetNextElement("model");
        }
        */
    } 
    else
    {
        UE_LOG(LogTemp, Error, TEXT("SDF ERROR"));
        FString errorMessage(errors[0].Message().c_str());
        UE_LOG(LogTemp, Error, TEXT("%s"), *errorMessage);
        return false;
    }
    return true;
}

Eigen::Quaterniond SDFParser::RPYToQuaternion(Eigen::Vector3d RPY)
{
    Eigen::AngleAxisd RollRot(RPY[0], Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd PitchRot(RPY[1], Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd YawRot(RPY[2], Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond QuatRot = YawRot * PitchRot * RollRot;

    return QuatRot;
}

}
