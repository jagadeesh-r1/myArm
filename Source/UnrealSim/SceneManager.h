#pragma once

#include "CoreMinimal.h"
#include "Eigen/Dense"
#include "TreeRecursiveSolver.h"

DECLARE_LOG_CATEGORY_EXTERN(MotionEngineSceneManagerLog, All, All);

namespace MotionEngine
{

class MotionObject;

class SceneManager
{
    public:
        SceneManager();
        ~SceneManager();

        bool InitializeScene(void); 
        bool LoadRobotsFromXML(FString XMLFilePath);

        bool AddRobotToScene(FString URDFPath, FString ResourceDirectory, Eigen::Matrix4d RobotBaseG);
        bool AddArticulatedObjectToScene(FString URDFPath, FString ResourceDirectory, Eigen::Matrix4d ObjectBaseG);
        bool AddSimpleObjectToScene(FString URDFPath, FString ResourceDirectory, Eigen::Matrix4d ObjectBaseG);
        bool AddMObjectsToScene(FString SDFPath, FString ResourceDirectory);

        TArray<TSharedPtr<MotionObject>> SceneObjects;
        TMap< FString, TSharedPtr<MotionObject> > SceneObjectsMap;

        TMap< FString, TSharedPtr<TreeRecursiveSolver> > ArticulatedObjectTreeSolvers;

    private:
        bool bIsInitialized;
};

}