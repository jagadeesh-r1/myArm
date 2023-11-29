// Copyright Epic Games, Inc. All Rights Reserved.

using System;
using System.IO;
using UnrealBuildTool;

public class UnrealSim : ModuleRules
{
	public UnrealSim(ReadOnlyTargetRules Target) : base(Target)
	{
		bUseRTTI = true;
		bEnableExceptions = true;
		bEnableUndefinedIdentifierWarnings = false;
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
	
		PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore", "XmlParser", "SDF", "WebSockets" , "ROSIntegration", "HeadMountedDisplay", "NavigationSystem", "OculusHMD" });
		
		PublicIncludePaths.Add(Path.Combine(ModuleDirectory, "../Eigen"));

		PrivateDependencyModuleNames.AddRange(new string[] { "SDF" });
		PrivateDependencyModuleNames.AddRange(new string[] { });
		//PrivateDependencyModuleNames.AddRange(new string[] { "ROSIntegration" });
		//PrivateDependencyModuleNames.Add(Path.Combine(ProjectDirectory, "Source/SDF"));

		// Uncomment if you are using Slate UI
		// PrivateDependencyModuleNames.AddRange(new string[] { "Slate", "SlateCore" });

		// Uncomment if you are using online features
		// PrivateDependencyModuleNames.Add("OnlineSubsystem");

		// To include OnlineSubsystemSteam, add it to the plugins section in your uproject file with the Enabled attribute set to true
	}
}
