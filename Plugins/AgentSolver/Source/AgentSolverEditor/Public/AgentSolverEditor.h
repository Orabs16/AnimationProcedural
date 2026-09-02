#pragma once

// Registers the double-click-to-open node editor for ULearningProgram
// assets -- same StartupModule pattern AgentSolver.cpp uses for
// FAgentSolverPresetAssetActions/UAgentSolverPresetFactory, just in this
// Editor-type module instead (see AgentSolverEditor.Build.cs's comment for
// why: GraphEditor/BlueprintGraph/Kismet are the first heavy toolkit
// dependencies this plugin needs).

#include "Modules/ModuleManager.h"

class FLearningProgramAssetTypeActions;
struct FGraphPanelNodeFactory;

class FAgentSolverEditorModule : public IModuleInterface
{
public:
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;

private:
	/** Registered in StartupModule so double-clicking a ULearningProgram asset opens FLearningProgramAssetEditorToolkit instead of the generic "no editor for this asset" fallback. */
	TSharedPtr<FLearningProgramAssetTypeActions> LearningProgramAssetTypeActions;

	/** Registered in StartupModule so the graph editor uses SGraphNode_LearningProgram (PIE active-node highlight + per-transition progress bars) instead of the generic default SGraphNode look. */
	TSharedPtr<FGraphPanelNodeFactory> LearningProgramNodeFactory;
};
