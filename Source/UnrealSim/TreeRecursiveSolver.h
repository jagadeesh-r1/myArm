#pragma once

#include "CoreMinimal.h"
#include "MotionEngineSolver.h"
#include "Eigen/Dense"

DECLARE_LOG_CATEGORY_EXTERN(TreeRecursiveSolverLog, All, All);

namespace MotionEngine
{

class ArticulatedElement;
class ArticulatedMotionObject;
class Tree;

class TreeRecursiveSolver : public MotionEngineSolver<Tree>
{
    public:
        TreeRecursiveSolver();
        virtual ~TreeRecursiveSolver();

        virtual bool InitializeSolver(TSharedPtr<Tree> MObjectTree) override;

        void SolvePositionFK(const TMap<FString, double> &JointPositions);
        void SolvePositionFK(const TMap<FString, double>& JointPositions, const Eigen::Matrix4d& RootG);

        void ComputeElementState(TSharedPtr<ArticulatedElement> CurrentElement, const Eigen::Matrix4d &ParentElementG, const TMap<FString, double> &JointPositions);

        TSharedPtr<ArticulatedMotionObject> ArticulatedMObject;
        TSharedPtr<Tree> ObjectTree;
};

}