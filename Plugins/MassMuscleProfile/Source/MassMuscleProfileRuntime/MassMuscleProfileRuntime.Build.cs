// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class MassMuscleProfileRuntime : ModuleRules
{
    public MassMuscleProfileRuntime(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

        PublicDependencyModuleNames.AddRange(
            new string[]
            {
                "Core",
                "CoreUObject",
                "Engine",
                "AnimationCore",
                "ControlRig",
                "RigVM"
            }
        );
    }
}
