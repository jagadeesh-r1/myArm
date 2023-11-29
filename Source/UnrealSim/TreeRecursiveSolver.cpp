#include "TreeRecursiveSolver.h"
#include "Tree.h"
#include "ArticulatedElement.h"
#include "ArticulatedMotionObject.h"
#include "Joint.h"
#include "Link.h"

DEFINE_LOG_CATEGORY(TreeRecursiveSolverLog);

namespace MotionEngine
{

void PrintMatrix(Eigen::Matrix4d Mat)
{
    for(int i = 0; i < 4; i++)
    {
        UE_LOG(TreeRecursiveSolverLog, Display, TEXT("%+.3f\t%+.3f\t%+.3f\t%+.3f"), Mat(i,0), Mat(i,1), Mat(i,2), Mat(i,3));
    }
}

TreeRecursiveSolver::TreeRecursiveSolver()
{

}

TreeRecursiveSolver::~TreeRecursiveSolver()
{

}

bool TreeRecursiveSolver::InitializeSolver(TSharedPtr<Tree> MObjectTree)
{
    /*
    if(!MObject)
    {
        UE_LOG(TreeRecursiveSolverLog, Error, TEXT("Passed a NULL MotionObject pointer"));
        return false;
    }

    if(MObject->ObjectType != MotionObject::ARTICULATED_MOTION_OBJECT)
    {
        UE_LOG(TreeRecursiveSolverLog, Error, TEXT("Invalid object type passed to TreeRecursiveSolver"));
        return false;
    }

    ArticulatedMObject = StaticCastSharedPtr<ArticulatedMotionObject>(MObject);

    if(!ArticulatedMObject->ObjectTree)
    {
        UE_LOG(TreeRecursiveSolverLog, Error, TEXT("ArticulatedMotionObject::ObjectTree not initialized"));
        return false;
    }

    ObjectTree = ArticulatedMObject->ObjectTree;
    */

    ObjectTree.Reset();
    ObjectTree = MObjectTree;

    UE_LOG(TreeRecursiveSolverLog, Display, TEXT("TreeRecursiveSolver successfully initialized"));
    return true;
}

void TreeRecursiveSolver::SolvePositionFK(const TMap<FString, double> &JointPositions)
{
    Eigen::Matrix4d BaseG = *(ObjectTree->TreeLinkStateMap[ObjectTree->RootElement->Name]->LinkG);
    ComputeElementState(ObjectTree->RootElement, BaseG, JointPositions);
}

void TreeRecursiveSolver::SolvePositionFK(const TMap<FString, double>& JointPositions, const Eigen::Matrix4d& RootG)
{
    *(ObjectTree->TreeLinkStateMap[ObjectTree->RootElement->Name]->LinkG) = RootG;
    ComputeElementState(ObjectTree->RootElement, RootG, JointPositions);
}

void TreeRecursiveSolver::ComputeElementState(TSharedPtr<ArticulatedElement> CurrentElement, const Eigen::Matrix4d &ParentElementG, const TMap<FString, double> &JointPositions)
{
    Eigen::Matrix4d CurrentElementG = Eigen::Matrix4d::Identity();

    if(CurrentElement->ElementType == ArticulatedElement::JOINT)
    {
        TSharedPtr<JointState> CurrentJointState = ObjectTree->TreeJointStateMap[CurrentElement->Name];

        Eigen::Matrix4d JointTF = Eigen::Matrix4d::Identity();

        if(JointPositions.Contains(CurrentElement->Name))
        {
            *CurrentJointState->JointPosition = JointPositions[CurrentElement->Name];
        }

        switch(CurrentJointState->Type)
        {
            case Joint::REVOLUTE:
            case Joint::CONTINUOUS:
                JointTF.block<3,3>(0,0) = Eigen::AngleAxisd(*CurrentJointState->JointPosition, CurrentJointState->JointAxis).toRotationMatrix();
                break;
            case Joint::PRISMATIC:
                JointTF.block<3,1>(0,3) = *CurrentJointState->JointPosition * CurrentJointState->JointAxis;
                break;
            default:
                break;
        }

        *CurrentJointState->JointG = ParentElementG * CurrentJointState->OriginG * JointTF;
        CurrentElementG = *CurrentJointState->JointG;
    }
    else if(CurrentElement->ElementType == ArticulatedElement::LINK)
    {
        *(ObjectTree->TreeLinkStateMap[CurrentElement->Name]->LinkG) = ParentElementG;
        CurrentElementG = ParentElementG;
    }
    else
    {

    }

    for(int32 ChildItr = 0; ChildItr < CurrentElement->ChildElements.Num(); ChildItr++)
    {
        if(CurrentElement->ChildElements[ChildItr])
        {
            ComputeElementState(CurrentElement->ChildElements[ChildItr], CurrentElementG, JointPositions);
        }
    }
}


}