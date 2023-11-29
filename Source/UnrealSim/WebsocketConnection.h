// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "C:\Program Files\Epic Games\UE_4.27\Engine\Source\Runtime\Online\WebSockets\Public\IWebSocket.h"

/**
 *
 */

class UNREALSIM_API WebsocketConnection 
{
public:
	WebsocketConnection();
	~WebsocketConnection();


	TSharedPtr<IWebSocket> WebSocket;

	void Connect();
};
