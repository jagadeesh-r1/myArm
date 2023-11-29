#include "ArticulatedElement.h"

namespace MotionEngine
{

ArticulatedElement::ArticulatedElement()
{
    ElementType = ArticulatedElementType::UNKNOWN;

    Name.Reset(20);

    ParentElement.Reset();
    ChildElements.Reset();

    OriginXYZ = Eigen::Vector3d::Zero();
    OriginRPY = Eigen::Vector3d::Zero();
    OriginG = Eigen::Matrix4d::Identity();

    ElementG = Eigen::Matrix4d::Identity();
}

ArticulatedElement::~ArticulatedElement()
{

}

ArticulatedElement::ArticulatedElement(const ArticulatedElement* ElementSpecs)
{
    if(ElementSpecs)
    {
        ElementType = ElementSpecs->ElementType;

        Name = ElementSpecs->Name;

        ParentElement.Reset();
        ChildElements.Reset();

        OriginXYZ = ElementSpecs->OriginXYZ;
        OriginRPY = ElementSpecs->OriginRPY;
        OriginG = ElementSpecs->OriginG;

        ElementG = ElementSpecs->ElementG;
    }
    else
    {
        ElementType = ArticulatedElementType::UNKNOWN;

        Name.Reset(20);

        ParentElement.Reset();
        ChildElements.Reset();

        OriginXYZ = Eigen::Vector3d::Zero();
        OriginRPY = Eigen::Vector3d::Zero();
        OriginG = Eigen::Matrix4d::Identity();

        ElementG = Eigen::Matrix4d::Identity();
    }
}

}