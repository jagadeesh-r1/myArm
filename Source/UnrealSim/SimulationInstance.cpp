// Fill out your copyright notice in the Description page of Project Settings.


#include "SimulationInstance.h"
#include "Engine/StaticMeshActor.h"

void USimulationInstance::OnStart()
{
    UE_LOG(LogTemp, Display, TEXT("Simulation Instance Started"));

    /*
    FVector ActorSpawnLoc(0,0,20);
    FRotator ActorSpawnRot(0,0,0);
    FActorSpawnParameters ActorSpawnParams;

    ActorSpawnParams.Name = TEXT("GameInstance_Spawned_Shape_Actor");

    AStaticMeshActor* ShapeActor = GetWorld()->SpawnActor<AStaticMeshActor>(ActorSpawnLoc, ActorSpawnRot, ActorSpawnParams);
    UStaticMesh* ShapeActorMesh = (UStaticMesh*)StaticLoadObject(UStaticMesh::StaticClass(), NULL, TEXT("/Game/StarterContent/Shapes/Shape_Cylinder.Shape_Cylinder"));
    ShapeActor->GetStaticMeshComponent()->SetStaticMesh(ShapeActorMesh);
    */
}


