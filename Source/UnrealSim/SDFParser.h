#pragma once

#include "CoreMinimal.h"
#include "XmlFile.h"
#include "Eigen/Dense"
#include <math.h>
#include <sdf/sdf.hh>

DECLARE_LOG_CATEGORY_EXTERN(SDFParserLog, All, All);

namespace MotionEngine
{

class MotionObject;
class Geometry;
class Visual;
class Link;
class Joint;

class SDFParser
{
    public:
        SDFParser();
        ~SDFParser();

        static Eigen::Vector3d ParseOriginAttributes(FString AttributeVal);
        static bool ParseOriginAttributes(FString AttributeVal, Eigen::Vector3d &ElementG);

        static bool ParseSDF(FString URDFFilePath, FString ResourceDirectory, TArray< TSharedPtr<MotionObject> > &MObjectst);

        static bool ParsePose(ignition::math::Quaterniond quat, ignition::math::Vector3d vec, Eigen::Vector3d &VisualOriginRPY, Eigen::Vector3d &VisualOriginXYZ, Eigen::Matrix4d &VisualOriginG);
        static bool ParseGeometry(sdf::ElementPtr geometryElement, FString ResourceDirectory, TSharedPtr<Geometry> &NewGeometry);
        static bool ParseVisual(sdf::ElementPtr visualElement, FString ResourceDirectory, TSharedPtr<Visual> &NewVisual);
        static bool ParseLink(sdf::ElementPtr linkElement, FString ResourceDirectory, TSharedPtr<Link> &NewLink);
        static bool ParseJoint(sdf::ElementPtr jointElement, TSharedPtr<Joint> &NewJoint);
        static bool ParseModel(sdf::ElementPtr modelElement, FString ResourceDirectory, TSharedPtr<MotionObject> &MObject);
        static void ParseInclude(sdf::ElementPtr includeElement, FString ResourceDirectory, TSharedPtr<MotionObject>& MObject);

        static Eigen::Quaterniond RPYToQuaternion(Eigen::Vector3d RPY);
};

}