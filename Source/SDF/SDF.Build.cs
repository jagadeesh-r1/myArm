using System;
using System.IO;
using UnrealBuildTool;

public class SDF : ModuleRules
{
	public SDF(ReadOnlyTargetRules Target) : base(Target)
	{
		// bUseRTTI = true;
		// bEnableExceptions = true;
		Type = ModuleType.External;
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
		
		// bEnableUndefinedIdentifierWarnings = false;

		PublicDependencyModuleNames.AddRange(new string[] {"Core", "CoreUObject"});
		PublicIncludePaths.Add(Path.Combine(ModuleDirectory, "Public"));

		if (Target.Platform == UnrealTargetPlatform.Win64)
		{
			PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "Lib", "Win64", "ignition-math4.lib"));
			PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "Lib", "Win64", "sdformat7.lib"));
			//RuntimeDependencies.Add(Path.Combine(ModuleDirectory, "Lib", "Win64", "ignition-math4.lib"));
			//RuntimeDependencies.Add(Path.Combine(ModuleDirectory, "Lib", "Win64", "sdformat7.lib"));
			RuntimeDependencies.Add(Path.Combine(ModuleDirectory, "Lib", "Win64", "sdformat7.dll"));
			RuntimeDependencies.Add(Path.Combine(ModuleDirectory, "Lib", "Win64", "ignition-math4.dll"));
			//PublicDelayLoadDlls.Add("sdformat7.dll");
		}
		else if(Target.Platform == UnrealTargetPlatform.Mac)
		{
			//for later
		}
		else if(Target.Platform == UnrealTargetPlatform.Linux)
		{
			//for later
		}

		

		// Uncomment if you are using Slate UI
		// PrivateDependencyModuleNames.AddRange(new string[] { "Slate", "SlateCore" });
		
		// Uncomment if you are using online features
		// PrivateDependencyModuleNames.Add("OnlineSubsystem");

		// To include OnlineSubsystemSteam, add it to the plugins section in your uproject file with the Enabled attribute set to true
	}
}