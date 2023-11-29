#include "Chain.h"
#include "Link.h"
#include "Joint.h"

DEFINE_LOG_CATEGORY(MotionEngineChainLog)

namespace MotionEngine
{
    
Chain::Chain()
{

}

Chain::~Chain()
{

}

void Chain::AddLink(const Link* NewLink)
{
    TSharedPtr<LinkState> NewLinkState(new LinkState(NewLink));

    ChainLinkStates.Push(NewLinkState);
    ChainLinkStateMap.Add(NewLinkState->Name, NewLinkState);
}

void Chain::AddJoint(const Joint* NewJoint)
{
    TSharedPtr<JointState> NewJointState(new JointState(NewJoint));

    ChainJointStates.Push(NewJointState);
    ChainJointStateMap.Add(NewJointState->Name, NewJointState);
}

bool Chain::InitializeChain(const FString &RootElementName, const FString &TipElementName)
{
    TSharedPtr<ArticulatedElement> CurrentElement;
    TSharedPtr<LinkState> CurrentLinkState;
    TSharedPtr<JointState> CurrentJointState;

    int32 LinkItr = 0;

    if(!ChainLinkStateMap.Contains(RootElementName))
    {
        UE_LOG(MotionEngineChainLog, Error, TEXT("Invalid RootElementName. Chain does not contain link \"%s\""), *RootElementName);
    }

    if(!ChainLinkStateMap.Contains(TipElementName))
    {
        UE_LOG(MotionEngineChainLog, Error, TEXT("Invalid TipElementName. Chain does not contain link \"%s\""), *TipElementName);
    }

    ChainRootElementName = RootElementName;
    ChainTipElementName = TipElementName;

    CurrentLinkState = ChainLinkStateMap[TipElementName];

    while(LinkItr < ChainLinkStates.Num())
    {
        UE_LOG(MotionEngineChainLog, Display, TEXT("CurrentLink Name : \"%s\""), *CurrentLinkState->Name);

        for(int32 JointItr = 0; JointItr < ChainJointStates.Num(); JointItr++)
        {
            if(ChainJointStates[JointItr]->ChildLinkName == CurrentLinkState->Name)
            {
                CurrentLinkState->ParentElement = ChainJointStates[JointItr];
                UE_LOG(MotionEngineChainLog, Display, TEXT("CurrentLink Parent Name : \"%s\""), *ChainJointStates[JointItr]->Name);

                ChainJointStates[JointItr]->ChildElements.Push(CurrentLinkState);
                ChainJointStates[JointItr]->ParentElement = ChainLinkStateMap[ChainJointStates[JointItr]->ParentLinkName];
                UE_LOG(MotionEngineChainLog, Display, TEXT("CurrentJoint Parent Name : \"%s\""), *ChainJointStates[JointItr]->ParentElement->Name);

                CurrentLinkState.Reset();
                CurrentLinkState = ChainLinkStateMap[ChainJointStates[JointItr]->ParentLinkName];

                if(CurrentLinkState->Name == RootElementName)
                {
                    UE_LOG(MotionEngineChainLog, Display, TEXT("Chain reached RootElement"));
                    return true;
                }

                break;
            }
        }

        LinkItr = LinkItr + 1;
    }

    return false;
}

} // namespace MotionEngine