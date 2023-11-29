#pragma once

#include "CoreMinimal.h"
#include "Eigen/Dense"

DECLARE_LOG_CATEGORY_EXTERN(MotionEngineChainLog, All, All);

namespace MotionEngine
{

class ArticulatedElement;
class Link;
class LinkState;
class Joint;
class JointState;

class Chain
{
    public:
        Chain();
        ~Chain();

        FString ChainRootElementName;
        FString ChainTipElementName;

        TArray< TSharedPtr< LinkState > > ChainLinkStates;
        TArray< TSharedPtr< JointState > > ChainJointStates;

        TMap< FString, TSharedPtr< LinkState > > ChainLinkStateMap;
        TMap< FString, TSharedPtr< JointState > > ChainJointStateMap;

        TSharedPtr< LinkState > RootElement;

        void AddLink(const Link* NewLink);
        void AddJoint(const Joint* NewJoint);

        bool InitializeChain(const FString &RootElementName, const FString &TipElementName);

};

} // namespace MotionEngine
