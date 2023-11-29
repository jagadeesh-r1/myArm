// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Engine/GameInstance.h"
#include "SimulationInstance.generated.h"

/**
 * 
 */
UCLASS()
class UNREALSIM_API USimulationInstance : public UGameInstance
{
	GENERATED_BODY()
	
	virtual void OnStart() override;
};
