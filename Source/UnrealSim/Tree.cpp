#include "Tree.h"
#include "Chain.h"
#include "Link.h"
#include "Joint.h"

DEFINE_LOG_CATEGORY(MotionEngineTreeLog)

namespace MotionEngine
{

Tree::Tree()
{

}

Tree::~Tree()
{

}

void Tree::AddLink(const Link* NewLink)
{
    TSharedPtr<LinkState> NewLinkState(new LinkState(NewLink));

    TreeLinkStates.Push(NewLinkState);
    TreeLinkStateMap.Add(NewLinkState->Name, NewLinkState);
}

void Tree::AddJoint(const Joint* NewJoint)
{
    TSharedPtr<JointState> NewJointState(new JointState(NewJoint));

    TreeJointStates.Push(NewJointState);
    TreeJointStateMap.Add(NewJointState->Name, NewJointState);
}

bool Tree::InitializeTree(const FString &RootElementName, const Eigen::Matrix4d &RootBaseG)
{
    UE_LOG(MotionEngineTreeLog, Display, TEXT("Initializing tree with link \"%s\" as the root element"), *RootElementName);

    RootElement = PropagateTree(RootElementName, RootBaseG);

    if(RootElement)
    {
        return true;
    }
    else
    {
        return false;
    }
}

TSharedPtr< ArticulatedElement > Tree::PropagateTree(const FString &ElementName, const Eigen::Matrix4d &ElementBaseG)
{
    TSharedPtr<ArticulatedElement> CurrentElement;
    TSharedPtr<LinkState> CurrentLink;

    if(!TreeLinkStateMap.Contains(ElementName))
    {
        UE_LOG(MotionEngineTreeLog, Error, TEXT("Tree does not contain link \"%s\""), *ElementName);
        return CurrentLink;
    }

    UE_LOG(MotionEngineTreeLog, Display, TEXT("Traversing Articulated object from link \"%s\""), *ElementName);

    CurrentLink = TreeLinkStateMap[ElementName];
    CurrentElement = CurrentLink;

    CurrentLink->ElementG = ElementBaseG;

    *CurrentLink->LinkG = CurrentLink->ElementG;

    Eigen::Quaterniond CurrentLinkQuat(CurrentLink->ElementG.block<3,3>(0,0));
    UE_LOG(MotionEngineTreeLog, Display, TEXT("Link Base Orientation  : X = %f\tY = %f\tZ = %f\tW = %f"), CurrentLinkQuat.x(), CurrentLinkQuat.y(), CurrentLinkQuat.z(), CurrentLinkQuat.w());
    UE_LOG(MotionEngineTreeLog, Display, TEXT("Link Base Position     : X = %f\tY = %f\tZ = %f"), CurrentLink->ElementG(0,3), CurrentLink->ElementG(1,3), CurrentLink->ElementG(2,3));

    for(int32 itr = 0; itr < TreeJointStates.Num(); itr++)
    {
        if(TreeJointStates[itr]->ParentLinkName == ElementName)
        {
            UE_LOG(MotionEngineTreeLog, Display, TEXT("Traversing Articulated object from joint \"%s\""), *TreeJointStates[itr]->Name);

            TSharedPtr<JointState> ChildJoint = TreeJointStates[itr];

            ChildJoint->ParentElement = CurrentLink;

            ChildJoint->ElementG = CurrentLink->ElementG * ChildJoint->OriginG;

            *ChildJoint->JointG = ChildJoint->ElementG;

            Eigen::Quaterniond ChildJointQuat(ChildJoint->ElementG.block<3,3>(0,0));
            UE_LOG(MotionEngineTreeLog, Display, TEXT("Joint Base Orientation : X = %f\tY = %f\tZ = %f\tW = %f"), ChildJointQuat.x(), ChildJointQuat.y(), ChildJointQuat.z(), CurrentLinkQuat.w());
            UE_LOG(MotionEngineTreeLog, Display, TEXT("Joint Base Position    : X = %f\tY = %f\tZ = %f"), ChildJoint->ElementG(0,3), ChildJoint->ElementG(1,3), ChildJoint->ElementG(2,3));

            TSharedPtr<ArticulatedElement> NextLink = PropagateTree(ChildJoint->ChildLinkName, ChildJoint->ElementG);

            NextLink->ParentElement = ChildJoint;

            ChildJoint->ChildElements.Push(NextLink);

            UE_LOG(MotionEngineTreeLog, Display, TEXT("Reached joint \"%s\" in the tree"), *ChildJoint->Name);

            CurrentLink->ChildElements.Push(ChildJoint);
        }
    }

    return CurrentElement;
}

bool Tree::GetChain(const FString &RootElementName, const FString &TipElementName, TSharedPtr<Chain> &SerialChain)
{
    if(!TreeLinkStateMap.Contains(RootElementName))
    {
        UE_LOG(MotionEngineTreeLog, Error, TEXT("Invalid RootElementName. Tree does not contain link \"%s\""), *RootElementName);
        return false;
    }

    if(!TreeLinkStateMap.Contains(TipElementName))
    {
        UE_LOG(MotionEngineTreeLog, Error, TEXT("Invalid TipElementName. Tree does not contain link \"%s\""), *TipElementName);
        return false;
    }

    SerialChain.Reset();
    SerialChain = MakeShareable<Chain>(new Chain());

    TSharedPtr<LinkState> CurrentLink = TreeLinkStateMap[TipElementName];

    SerialChain->AddLink(CurrentLink.Get());
    UE_LOG(MotionEngineTreeLog, Display, TEXT("Added link \"%s\" to chain"), *CurrentLink->Name);

    while(CurrentLink->ParentElement)
    {
        TSharedPtr<JointState> ParentJoint = TreeJointStateMap[CurrentLink->ParentElement->Name];

        SerialChain->AddJoint(ParentJoint.Get());
        UE_LOG(MotionEngineTreeLog, Display, TEXT("Added joint \"%s\" to chain"), *ParentJoint->Name);

        SerialChain->AddLink(TreeLinkStateMap[ParentJoint->ParentLinkName].Get());
        UE_LOG(MotionEngineTreeLog, Display, TEXT("Added link \"%s\" to chain"), *ParentJoint->ParentLinkName);

        CurrentLink.Reset();
        CurrentLink = TreeLinkStateMap[ParentJoint->ParentLinkName];

        if(CurrentLink->Name == RootElementName)
        {
            UE_LOG(MotionEngineTreeLog, Display, TEXT("Reached RootElement of chain"));
            bool InitResult = SerialChain->InitializeChain(RootElementName, TipElementName);
            if(!InitResult)
            {
                UE_LOG(MotionEngineTreeLog, Error, TEXT("Chain initialization failed"));
            }
            return InitResult;
        }
    }

    UE_LOG(MotionEngineTreeLog, Error, TEXT("Chain construction failed"));
    return false;
}

}