// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Engine/GameInstance.h"
#include "C:\Program Files\Epic Games\UE_4.27\Engine\Source\Runtime\Online\WebSockets\Public\IWebSocket.h"
#include "WebSocketGameInstance.generated.h"

/**
 * 
 */
UCLASS()
class UNREALSIM_API UWebSocketGameInstance : public UGameInstance
{
	GENERATED_BODY()

public:
	virtual void Init() override;
	virtual void Shutdown() override;
	TSharedPtr<IWebSocket> WebSocket;
	
};