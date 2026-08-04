// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class MassMuscleProfile : ModuleRules
{
	public MassMuscleProfile(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicIncludePaths.AddRange(
			new string[] {
				// ... add public include paths required here ...
			}
			);


		PrivateIncludePaths.AddRange(
			new string[] {
				// ... add other private include paths required here ...
			}
			);


		PublicDependencyModuleNames.AddRange(
			new string[]
			{
				"Core",
				"MassMuscleProfileRuntime"
			}
		);

		PrivateDependencyModuleNames.AddRange(
			new string[]
			{
				"CoreUObject",
				"Engine",
				"BlueprintGraph",

				"Slate",
				"SlateCore",

				"ApplicationCore",

				"EditorFramework",
				"UnrealEd",
				"AdvancedPreviewScene",
				"ToolMenus",
				"LevelEditor",

				"PropertyEditor",

				"EditorStyle",

				"InputCore",
    			"ContentBrowser",
				"EditorWidgets",
				"AdvancedPreviewScene",
				"AssetTools",
				"ControlRigDeveloper"
			}
		);


		DynamicallyLoadedModuleNames.AddRange(
			new string[]
			{
				// ... add any modules that your module loads dynamically here ...
			}
			);
	}
}
