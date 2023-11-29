#pragma once

#include "CoreMinimal.h"
#include "XmlFile.h"
#include "Eigen/Dense"
// #include <sdf/sdf.hh>

DECLARE_LOG_CATEGORY_EXTERN(URDFParserLog, All, All);

namespace MotionEngine
{

class MotionObject;
class Geometry;
class Visual;
class Link;
class Joint;

class URDFParser
{
    public:
        URDFParser();
        ~URDFParser();

        static Eigen::Vector3d ParseOriginAttributes(FString AttributeVal);

        static bool ParseURDF(FString URDFFilePath, FString ResourceDirectory, TSharedPtr<MotionObject> &MObject);

        static bool ParseOrigin(FXmlNode* OriginXmlNode, Eigen::Vector3d &VisualOriginRPY, Eigen::Vector3d &VisualOriginXYZ, Eigen::Matrix4d &VisualOriginG);
        static bool ParseGeometry(FXmlNode* GeometryXmlNode, FString ResourceDirectory, TSharedPtr<Geometry> &NewGeometry);
        static bool ParseVisual(FXmlNode* VisualXmlNode, FString ResourceDirectory, TSharedPtr<Visual> &NewVisual);
        static bool ParseLink(FXmlNode* LinkXmlNode, FString ResourceDirectory, TSharedPtr<Link> &NewLink);
        static bool ParseJoint(FXmlNode* JointXmlNode, TSharedPtr<Joint> &NewJoint);

        static Eigen::Quaterniond RPYToQuaternion(Eigen::Vector3d RPY);
};

}