#include "Link.h"

namespace MotionEngine
{

Link::Link()
{
    LinkVisual.Reset();
    LinkCollision.Reset();
    LinkInertial.Reset();

    OriginXYZ = Eigen::Vector3d::Zero();
    OriginRPY = Eigen::Vector3d::Zero();
    OriginG = Eigen::Matrix4d::Identity();

    ElementType = ArticulatedElement::LINK;
}

Link::~Link()
{

}

Link::Link(const Link* LinkSpecs) :
    ArticulatedElement(LinkSpecs)
{
    LinkVisual.Reset();
    if(LinkSpecs->LinkVisual)
    {
        LinkVisual = MakeShareable(new Visual(LinkSpecs->LinkVisual.Get()));
    }
    
    LinkCollision.Reset();
    if(LinkSpecs->LinkCollision)
    {
        LinkCollision = MakeShareable(new Collision(LinkSpecs->LinkCollision.Get()));
    }

    LinkInertial.Reset();
    if(LinkSpecs->LinkInertial)
    {
        LinkInertial = MakeShareable(new Inertial(LinkSpecs->LinkInertial.Get()));
    }
}

LinkState::LinkState()
{
    LinkG = MakeShareable(new Eigen::Matrix4d());
    *LinkG = Eigen::Matrix4d::Identity();
}

LinkState::~LinkState()
{

}

LinkState::LinkState(const Link* LinkSpecs) :
    Link(LinkSpecs)
{
    LinkG = MakeShareable(new Eigen::Matrix4d());
    *LinkG = Eigen::Matrix4d::Identity();
}

}