// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "IWebSocket.h"

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
