#include "URDFParser.h"
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

DEFINE_LOG_CATEGORY(URDFParserLog);

namespace MotionEngine
{

URDFParser::URDFParser()
{

}

URDFParser::~URDFParser()
{

}

Eigen::Vector3d URDFParser::ParseOriginAttributes(FString AttributeVal)
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

bool URDFParser::ParseOrigin(FXmlNode* OriginXmlNode, Eigen::Vector3d &VisualOriginRPY, Eigen::Vector3d &VisualOriginXYZ, Eigen::Matrix4d &VisualOriginG)
{
    FString OriginRPY = OriginXmlNode->GetAttribute(TEXT("rpy"));
    FString OriginXYZ = OriginXmlNode->GetAttribute(TEXT("xyz"));

    UE_LOG(URDFParserLog, Display, TEXT("Debuggin in urdf parser and ... with its OriginRPY %s and OriginXYZ %s"),  
            *OriginRPY, *OriginXYZ);

    VisualOriginRPY = ParseOriginAttributes(OriginRPY);
    VisualOriginXYZ = ParseOriginAttributes(OriginXYZ);

    Eigen::Quaterniond VisualOriginQuat = RPYToQuaternion(VisualOriginRPY);
    VisualOriginG.block<3,3>(0,0) = VisualOriginQuat.normalized().toRotationMatrix();
    VisualOriginG.block<3,1>(0,3) = VisualOriginXYZ;

    return true;
}

bool URDFParser::ParseGeometry(FXmlNode* GeometryXmlNode, FString ResourceDirectory, TSharedPtr<Geometry> &NewGeometry)
{
    NewGeometry.Reset();

    TArray<FXmlNode*> GeometryChildrenNodes = GeometryXmlNode->GetChildrenNodes();

    if(GeometryChildrenNodes.Num() == 0)
    {
        UE_LOG(URDFParserLog, Warning, TEXT("Geometry information undefined"));
    }
    else if(GeometryChildrenNodes.Num() > 1)
    {
        UE_LOG(URDFParserLog, Warning, TEXT("Conflicting geometry information provided for link"));
    }
    else
    {
        // TODO : Primitive shape support for geometry

        if(GeometryChildrenNodes[0]->GetTag() == TEXT("mesh"))
        {
            UE_LOG(URDFParserLog, Display, TEXT("Link geometry is defined by a mesh"));

            TSharedPtr<GeometryMesh> NewLinkGeoMesh(new GeometryMesh());

            NewLinkGeoMesh->Type = Geometry::MESH;

            FString MeshFilePath = GeometryChildrenNodes[0]->GetAttribute(TEXT("filename"));
            if(MeshFilePath.IsEmpty())
            {
                UE_LOG(URDFParserLog, Warning, TEXT("Mesh filename not provided with geometry information"));
            }
            else
            {
                int32 StrIdx;
                FString MeshName;

                UE_LOG(URDFParserLog, Display, TEXT("MeshFilePath        : %s"), *MeshFilePath);
                
                // Remove ROS Package Name
                StrIdx = MeshFilePath.Find(TEXT("/meshes/"), ESearchCase::IgnoreCase, ESearchDir::FromStart, 0);
                MeshName = MeshFilePath.RightChop(StrIdx+1);

                UE_LOG(URDFParserLog, Display, TEXT("MeshName            : %s"), *MeshName);

                // Remove file extension
                MeshName = MeshName.LeftChop(4);

                // Find name of mesh file
                MeshName.FindLastChar(TEXT('/'), StrIdx);
                NewLinkGeoMesh->MeshDirectory = ResourceDirectory + TEXT("/") + MeshName.Left(StrIdx);
                MeshName = MeshName.RightChop(StrIdx+1);
                NewLinkGeoMesh->MeshFileName = MeshName;

                UE_LOG(URDFParserLog, Display, TEXT("Link mesh directory : %s"), *NewLinkGeoMesh->MeshDirectory);
                UE_LOG(URDFParserLog, Display, TEXT("Link mesh filename  : %s"), *NewLinkGeoMesh->MeshFileName);
            }

            NewGeometry = NewLinkGeoMesh;
        }
    }
    return true;
}

bool URDFParser::ParseVisual(FXmlNode* VisualXmlNode, FString ResourceDirectory, TSharedPtr<Visual> &NewVisual)
{
    NewVisual.Reset();
    NewVisual = MakeShareable(new Visual());

    FXmlNode* OriginNode = VisualXmlNode->FindChildNode(TEXT("origin"));
    if(OriginNode)
    {
        ParseOrigin(OriginNode, NewVisual->VisualOriginRPY, NewVisual->VisualOriginXYZ, NewVisual->VisualOriginG);
    }

    FXmlNode* GeometryNode = VisualXmlNode->FindChildNode(TEXT("geometry"));
    if(GeometryNode)
    {
        ParseGeometry(GeometryNode, ResourceDirectory, NewVisual->VisualGeometry);
    }
    return true;
}

bool URDFParser::ParseLink(FXmlNode* LinkXmlNode, FString ResourceDirectory, TSharedPtr<Link> &NewLink)
{
    NewLink.Reset();
    NewLink = MakeShareable<Link>(new Link());

    NewLink->Name = LinkXmlNode->GetAttribute(TEXT("name"));

    UE_LOG(URDFParserLog, Display, TEXT("Parsing \"%s\" link information"), *NewLink->Name);

    FXmlNode* VisualNode = LinkXmlNode->FindChildNode(TEXT("visual"));
    if(VisualNode)
    {
        ParseVisual(VisualNode, ResourceDirectory, NewLink->LinkVisual);
    }

    // TODO : Add link inertial and collisioin information

    return true;
}

bool URDFParser::ParseJoint(FXmlNode* JointXmlNode, TSharedPtr<Joint> &NewJoint)
{
    NewJoint.Reset();
    NewJoint = MakeShareable<Joint>(new Joint());

    NewJoint->Name = JointXmlNode->GetAttribute(TEXT("name"));

    UE_LOG(URDFParserLog, Display, TEXT("Parsing \"%s\" joint informatioin"), *NewJoint->Name);

    FString JointType = JointXmlNode->GetAttribute(TEXT("type"));

    UE_LOG(URDFParserLog, Display, TEXT("Joint Type : %s"), *JointType);

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
        UE_LOG(URDFParserLog, Error, TEXT("URDF contains invalid/unknown joint type"));
        NewJoint->Type = Joint::UNKNOWN;
    }

    if((NewJoint->Type == Joint::REVOLUTE) || (NewJoint->Type == Joint::PRISMATIC) || (NewJoint->Type == Joint::CONTINUOUS))
    {
        FXmlNode* AxisNode = JointXmlNode->FindChildNode(TEXT("axis"));
        if(AxisNode)
        {
            FString AxisXYZ = AxisNode->GetAttribute(TEXT("xyz"));

            NewJoint->JointAxis = ParseOriginAttributes(AxisXYZ);

            UE_LOG(URDFParserLog, Display, TEXT("Raw Axis XYZ : %s"), *AxisXYZ);
            UE_LOG(URDFParserLog, Display, TEXT("Axis X : %f\tY : %f\tZ : %f"), NewJoint->JointAxis[0], NewJoint->JointAxis[1], NewJoint->JointAxis[2]);
        }
        else
        {
            UE_LOG(URDFParserLog, Warning, TEXT("Joint axis not defined for joint \"%s\""), *NewJoint->Name);
            UE_LOG(URDFParserLog, Warning, TEXT("Joint axis defaulting to \"[1;0;0]\""))

            NewJoint->JointAxis = Eigen::Vector3d::Zero();
            NewJoint->JointAxis[0] = 1;
        }

        if(NewJoint->Type != Joint::CONTINUOUS)
        {
            FXmlNode* LimitNode = JointXmlNode->FindChildNode(TEXT("limit"));
            if(LimitNode)
            {
                FString LimitValue;

                LimitValue = LimitNode->GetAttribute(TEXT("lower"));
                NewJoint->Limits.LowerLimit = FCString::Atod(*LimitValue);
                LimitValue = LimitNode->GetAttribute(TEXT("upper"));
                NewJoint->Limits.UpperLimit = FCString::Atod(*LimitValue);

                UE_LOG(URDFParserLog, Display, TEXT("Joint Limits : Upper = %f\tLower : %f"), NewJoint->Limits.UpperLimit, NewJoint->Limits.LowerLimit);
            }
            else
            {
                UE_LOG(URDFParserLog, Warning, TEXT("Limits not defined for joint \"%s\""), *NewJoint->Name);
                UE_LOG(URDFParserLog, Warning, TEXT("Defaulting upper and lower joint limits to \"0\""));
            }
        }
        else
        {
            NewJoint->Limits.LowerLimit = -2 * M_PI;
            NewJoint->Limits.UpperLimit = 2 * M_PI;
        }
    }

    FXmlNode* OriginNode = JointXmlNode->FindChildNode(TEXT("origin"));
    if(OriginNode)
    {
        ParseOrigin(OriginNode, NewJoint->OriginRPY, NewJoint->OriginXYZ, NewJoint->OriginG);
    }

    FXmlNode* ParentNode = JointXmlNode->FindChildNode(TEXT("parent"));
    if(ParentNode)
    {
        NewJoint->ParentLinkName = ParentNode->GetAttribute(TEXT("link"));
        UE_LOG(URDFParserLog, Display, TEXT("Joint \"%s\" parent link : \"%s\""), *NewJoint->Name, *NewJoint->ParentLinkName);
    }
    else
    {
        UE_LOG(URDFParserLog, Warning, TEXT("Joint \"%s\" not provided with parent link information"));
    }

    FXmlNode* ChildNode = JointXmlNode->FindChildNode(TEXT("child"));
    if(ChildNode)
    {
        NewJoint->ChildLinkName = ChildNode->GetAttribute(TEXT("link"));
        UE_LOG(URDFParserLog, Display, TEXT("Joint \"%s\" child link  : \"%s\""), *NewJoint->Name, *NewJoint->ChildLinkName);
    }
    else
    {
        UE_LOG(URDFParserLog, Warning, TEXT("Joint \"%s\" not provided with child link information"));
    }

    return true;
}

bool URDFParser::ParseURDF(FString URDFFilePath, FString ResourceDirectory, TSharedPtr<MotionObject> &MObject)
{
    const TCHAR* LogText;

    MObject.Reset();

    LogText = *URDFFilePath;
    UE_LOG(URDFParserLog, Display, TEXT("Loading URDF file from path : %s"), LogText);

    TSharedPtr<FXmlFile> URDFXmlFile;

    URDFXmlFile.Reset();
    URDFXmlFile = MakeShareable(new FXmlFile(URDFFilePath));

    FXmlNode* URDFRootNode = URDFXmlFile->GetRootNode();

    if(!URDFRootNode)
    {
        UE_LOG(URDFParserLog, Error, TEXT("Error loading URDF file from given path"));
        return false;
    }

    if(URDFRootNode->GetTag() == FString(TEXT("robot")))
    {
        // TODO : Processing WRT robots (Eg. Presence of powered drive at joints)
        UE_LOG(URDFParserLog, Display, TEXT("URDF defines a robot"));

        TSharedPtr<ArticulatedMotionObject> ArticulatedObject(new ArticulatedMotionObject());
        MObject.Reset();
        MObject = ArticulatedObject;

        ArticulatedObject->ObjectName = URDFRootNode->GetAttribute(TEXT("name"));
        if(ArticulatedObject->ObjectName.IsEmpty())
        {
            UE_LOG(URDFParserLog, Warning, TEXT("Name of robot is undefined"));
        }
        else
        {
            LogText = *ArticulatedObject->ObjectName;
            UE_LOG(URDFParserLog, Display, TEXT("Loading description of \"%s\" from URDF"), LogText);
        }

        // Get all Xml Nodes (Links, Joints, Sensors, ...) which define the object
        TArray<FXmlNode*> URDFRootChildNodes = URDFRootNode->GetChildrenNodes();

        for(int32 ChildItr = 0; ChildItr < URDFRootChildNodes.Num(); ChildItr++)
        {
            // Process the Xml Node based on its type

            if(URDFRootChildNodes[ChildItr]->GetTag() == TEXT("link"))
            {
                TSharedPtr<Link> NewLink(new Link());
                ParseLink(URDFRootChildNodes[ChildItr], ResourceDirectory, NewLink);

                ArticulatedObject->AddLink(NewLink.Get());
            }
            else if(URDFRootChildNodes[ChildItr]->GetTag() == TEXT("joint"))
            {
                TSharedPtr<Joint> NewJoint(new Joint());
                ParseJoint(URDFRootChildNodes[ChildItr], NewJoint);

                ArticulatedObject->AddJoint(NewJoint.Get());
            }
            else
            {
                LogText = *URDFRootChildNodes[ChildItr]->GetTag();
                UE_LOG(URDFParserLog, Warning, TEXT("URDF contains unknown element of type \"%s\""), LogText);
            }
        }
    }
    else if(URDFRootNode->GetTag() == FString(TEXT("object")))
    {
        FString ObjectName = URDFRootNode->GetAttribute(TEXT("name"));
        FString ObjectType = URDFRootNode->GetAttribute(TEXT("type"));

        if(ObjectType == TEXT("articulated"))
        {
            UE_LOG(URDFParserLog, Display, TEXT("URDF defines an articulated object"));

            TSharedPtr<ArticulatedMotionObject> ArticulatedObject(new ArticulatedMotionObject());
            MObject.Reset();
            MObject = ArticulatedObject;

            ArticulatedObject->ObjectName = URDFRootNode->GetAttribute(TEXT("name"));
            if(ArticulatedObject->ObjectName.IsEmpty())
            {
                UE_LOG(URDFParserLog, Warning, TEXT("Name of object is undefined"));
            }
            else
            {
                LogText = *ArticulatedObject->ObjectName;
                UE_LOG(URDFParserLog, Display, TEXT("Loading description of \"%s\" from URDF"), LogText);
            }

            // Get all Xml Nodes (Links, Joints, Sensors, ...) which define the object
            TArray<FXmlNode*> URDFRootChildNodes = URDFRootNode->GetChildrenNodes();

            for(int32 ChildItr = 0; ChildItr < URDFRootChildNodes.Num(); ChildItr++)
            {
                // Process the Xml Node based on its type

                if(URDFRootChildNodes[ChildItr]->GetTag() == TEXT("link"))
                {
                    TSharedPtr<Link> NewLink(new Link());
                    ParseLink(URDFRootChildNodes[ChildItr], ResourceDirectory, NewLink);

                    ArticulatedObject->AddLink(NewLink.Get());
                }
                else if(URDFRootChildNodes[ChildItr]->GetTag() == TEXT("joint"))
                {
                    TSharedPtr<Joint> NewJoint(new Joint());
                    ParseJoint(URDFRootChildNodes[ChildItr], NewJoint);

                    ArticulatedObject->AddJoint(NewJoint.Get());
                }
                else
                {
                    LogText = *URDFRootChildNodes[ChildItr]->GetTag();
                    UE_LOG(URDFParserLog, Warning, TEXT("URDF contains unknown element of type \"%s\""), LogText);
                }
            }
        }
        else if(ObjectType == TEXT("simple"))
        {
            UE_LOG(URDFParserLog, Display, TEXT("URDF defines a simple object"));

            TSharedPtr<SimpleMotionObject> SimpleObject(new SimpleMotionObject());
            MObject.Reset();
            MObject = SimpleObject;

            SimpleObject->ObjectName = ObjectName;
            if(SimpleObject->ObjectName.IsEmpty())
            {
                UE_LOG(URDFParserLog, Warning, TEXT("Name of object is undefined"));
            }
            else
            {
                UE_LOG(URDFParserLog, Display, TEXT("Loading description of \"%s\" from URDF"), *ObjectName);
            }
            
            TArray<FXmlNode*> URDFRootChildNodes = URDFRootNode->GetChildrenNodes();

            if(URDFRootChildNodes.Num() != 1)
            {
                UE_LOG(URDFParserLog, Error, TEXT("Unknown URDF format specification for simple object"));
                return false;
            }

            if(URDFRootChildNodes[0]->GetTag() == TEXT("link"))
            {
                TSharedPtr<Link> NewLink(new Link());
                ParseLink(URDFRootChildNodes[0], ResourceDirectory, NewLink);

                SimpleObject->InitializeLink(NewLink.Get());
            }
        }
        else
        {
            UE_LOG(URDFParserLog, Error, TEXT("URDF contains unknown object of type \"%s\""), LogText);
            return false;
        }
    }
    else
    {
        UE_LOG(URDFParserLog, Error, TEXT("URDF format unknown"));
        return false;
    }

    return true;
}

Eigen::Quaterniond URDFParser::RPYToQuaternion(Eigen::Vector3d RPY)
{
    Eigen::AngleAxisd RollRot(RPY[0], Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd PitchRot(RPY[1], Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd YawRot(RPY[2], Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond QuatRot = YawRot * PitchRot * RollRot;

    return QuatRot;
}

}
