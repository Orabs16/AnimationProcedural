// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class AgentSolver : ModuleRules
{
	public AgentSolver(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

		// CreatureBatchSolver's SIMD path uses raw AVX2 intrinsics directly
		// (FCreatureBatchState::SIMDWidth is hard-coded to 8 float lanes for
		// exactly this reason). This raises the minspec for this module —
		// acceptable since it's a training-time module with no shipping path.
		MinCpuArchX64 = MinimumCpuArchitectureX64.AVX2;

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
				// ... add other public dependencies that you statically link with here ...
			}
			);


		PrivateDependencyModuleNames.AddRange(
			new string[]
			{
				"CoreUObject",
				"Engine",
				"Slate",
				"SlateCore",

				// MutoRLTrainingDriver.h declares UPROPERTY/UCLASS types against
				// these, so — unlike MassMuscleProfile below — they can't be
				// gated to Editor-only targets (UHT rejects UPROPERTY wrapped in
				// #if WITH_EDITOR; only the plugin's own Runtime-type modules can
				// be referenced unconditionally like this). The plugin's own
				// LearningAgents.uplugin declares all four of these as
				// Type:"Runtime", so this is a legitimate always-on dependency —
				// only MutoRLTrainingDriver's actual BeginPlay/Tick *logic* is
				// WITH_EDITOR-gated (it needs MutoTopology.h, which is not).
				"Learning",
				"LearningTraining",
				"LearningAgents",
				"LearningAgentsTraining",
				// ... add private dependencies that you statically link with here ...
			}
			);

		if (Target.Type == TargetRules.TargetType.Editor)
		{
			// MutoTopology.h (WITH_EDITOR-only) reads the authored mass/muscle
			// profile data (UMassMuscleProfileAssetMuscle/Mass) to build a real
			// FCreatureTopology. MassMuscleProfile is an Editor-type module, so
			// this dependency — and everything that uses it — must stay out of
			// non-editor targets.
			PrivateDependencyModuleNames.Add("MassMuscleProfile");

			// UIControls/SAgentSolverControlPanel.h + AgentSolver.cpp's Window-menu
			// registration are WITH_EDITOR-gated (see those files) for the same
			// reason as above — all of these are Editor-type modules and must
			// stay out of non-editor targets too. AdvancedPreviewScene/InputCore
			// back the embedded SEditorViewport (FAgentSolverViewportClient),
			// PropertyEditor backs the category-filtered IDetailsView tabs —
			// same set MassMuscleProfile's own viewport+details panels need.
			PrivateDependencyModuleNames.AddRange(
				new string[]
				{
					"UnrealEd",
					"ToolMenus",
					"EditorFramework",
					"AdvancedPreviewScene",
					"PropertyEditor",
					"InputCore",
					// FLevelEditorModule::GetFirstActiveViewport(), used to point
					// GEditor->RequestPlaySession's DestinationSlateViewport at the
					// level editor's own active viewport instead of a new window.
					"LevelEditor",
					// IAssetTools::RegisterAssetTypeActions, for
					// FAgentSolverPresetAssetActions -- same module
					// MassMuscleProfile's own asset actions need
					// (MassMuscleProfile.Build.cs).
					"AssetTools",
				}
				);
		}


		DynamicallyLoadedModuleNames.AddRange(
			new string[]
			{
				// ... add any modules that your module loads dynamically here ...
			}
			);
	}
}
