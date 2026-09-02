// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class AgentSolverEditor : ModuleRules
{
	public AgentSolverEditor(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(
			new string[]
			{
				"Core",
				"AgentSolver",
			}
			);

		PrivateDependencyModuleNames.AddRange(
			new string[]
			{
				"CoreUObject",
				"Engine",
				"Slate",
				"SlateCore",
				"InputCore",
				"UnrealEd",
				"EditorFramework",
				"ToolMenus",
				"AssetTools",
				"PropertyEditor",
				// The Learning Program node graph itself -- SGraphEditor,
				// UEdGraphSchema, the standard node-drag/pin-link widgetry.
				// Not referenced anywhere else in this plugin (see
				// AgentSolver.Build.cs), which is the whole reason this got
				// its own module rather than growing that one's existing
				// WITH_EDITOR block.
				"GraphEditor",
				// SGraphNode_LearningProgram includes AgentSolver/MutoRLTrainingDriver.h
				// for its live PIE polling (AMutoRLTrainingDriver::GetLearningProgramLiveState).
				// That header -- a PUBLIC AgentSolver header -- itself #includes
				// both the LearningAgents plugin's headers (below) and
				// PhysicsSolver/CreatureGroundContact.h, which needs
				// UMassMuscleProfileAsset.h (MassMuscleProfile). AgentSolver.Build.cs
				// only lists all of these as PRIVATE dependencies, and a private
				// dependency's include paths don't propagate to a downstream
				// module that merely links against AgentSolver -- this was never
				// a problem before because nothing outside AgentSolver ever
				// included this header. So this module needs its own copy of the
				// same set. Unlike AgentSolver.Build.cs's MassMuscleProfile entry,
				// none of this is behind a Target.Type check: AgentSolverEditor is
				// unconditionally an Editor-type module.
				"MassMuscleProfile",
				"Learning",
				"LearningTraining",
				"LearningAgents",
				"LearningAgentsTraining",
			}
			);
	}
}
