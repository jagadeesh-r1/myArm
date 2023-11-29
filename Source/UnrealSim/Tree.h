#pragma once

#include "CoreMinimal.h"
#include "Eigen/Dense"

DECLARE_LOG_CATEGORY_EXTERN(MotionEngineTreeLog, All, All);

namespace MotionEngine
{

class ArticulatedElement;
class Link;
class LinkState;
class Joint;
class JointState;
class Chain;

class Tree
{
    public:
        Tree();
        ~Tree();

        TArray< TSharedPtr< LinkState > > TreeLinkStates;
        TArray< TSharedPtr< JointState > > TreeJointStates;

        TMap< FString, TSharedPtr< LinkState > > TreeLinkStateMap;
        TMap< FString, TSharedPtr< JointState > > TreeJointStateMap;

        TSharedPtr< ArticulatedElement > RootElement;

        void AddLink(const Link* NewLink);
        void AddJoint(const Joint* NewJoint);

        bool InitializeTree(const FString &RootElementName, const Eigen::Matrix4d &RootBaseG);
        
        TSharedPtr< ArticulatedElement > PropagateTree(const FString &ElementName, const Eigen::Matrix4d &ElementBaseG);

        bool GetChain(const FString &RootElementName, const FString &TipElementName, TSharedPtr< Chain > &SerialChain);
};

};