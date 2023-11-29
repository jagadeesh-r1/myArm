// Fill out your copyright notice in the Description page of Project Settings.


#include "WebsocketConnection.h"
#include "C:\Program Files\Epic Games\UE_4.27\Engine\Source\Runtime\Online\WebSockets\Public\WebSocketsModule.h"

WebsocketConnection::WebsocketConnection()
{
	//WebsocketConnection::Connect();
}

WebsocketConnection::~WebsocketConnection()
{
	UE_LOG(LogTemp, Display, TEXT("Websocket connection destructor called"));
	if (WebSocket && WebSocket->IsConnected()) {
		WebSocket->Close();
	}
}

int count = 0;

void WebsocketConnection::Connect() {

	if (!FModuleManager::Get().IsModuleLoaded("WebSockets"))
	{
		FModuleManager::Get().LoadModule("WebSockets");
	}

	WebSocket = FWebSocketsModule::Get().CreateWebSocket("ws://0.0.0.0:9090");

	WebSocket->OnConnected().AddLambda([]()
		{
			GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Green, "Successfully connected");
		});

	WebSocket->OnConnectionError().AddLambda([](const FString& Error)
		{
			GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Red, Error);
		});

	WebSocket->OnClosed().AddLambda([](int32 StatusCode, const FString& Reason, bool bWasClean)
		{
			GEngine->AddOnScreenDebugMessage(-1, 15.0f, bWasClean ? FColor::Green : FColor::Red, "Connection closed " + Reason);
		});

	WebSocket->OnMessage().AddLambda([](const FString& MessageString)
		{
			GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Cyan, "Received message: " + MessageString);
		});

	WebSocket->OnMessageSent().AddLambda([](const FString& MessageString)
		{
			GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Yellow, "Sent message: " + MessageString);
		});

	WebSocket->Connect();

	if (WebSocket->IsConnected()) {
		UE_LOG(LogTemp, Display, TEXT("Connected to ROS brodge : %d"), count);
		count++;
		FString msg = "test msg from unreal";
		WebSocket->Send(msg);
	}
}
