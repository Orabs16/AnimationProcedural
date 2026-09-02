// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "Modules/ModuleManager.h"
#include "UObject/WeakObjectPtr.h"

class SDockTab;
class FSpawnTabArgs;
class UAgentSolverPreset;
class SAgentSolverControlPanel;
class SAgentSolverAIDebugPanel;

#if WITH_EDITOR
class FAgentSolverPresetAssetActions;
#endif

class FAgentSolverModule : public IModuleInterface
{
public:

	/** IModuleInterface implementation */
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;

#if WITH_EDITOR
	/** Same accessor pattern as FMassMuscleProfileModule::Get() -- lets FAgentSolverPresetAssetActions reach this module from a static context. */
	static FAgentSolverModule& Get();

	/**
	 * Opens (or focuses, if already open) the Agent Solver control panel tab
	 * with the given preset loaded -- called from
	 * FAgentSolverPresetAssetActions::OpenAssetEditor (double-click on a
	 * UAgentSolverPreset asset). Mirrors FMassMuscleProfileModule::
	 * OpenToolForAsset's PendingAssetToOpen/ActivePanel split: if the tab
	 * doesn't exist yet, SpawnControlPanelTab below consumes
	 * PendingPresetToOpen; if it already exists, TryInvokeTab only focuses
	 * it (SpawnControlPanelTab does not run again), so this pushes the
	 * preset into the already-live panel directly instead.
	 */
	void OpenToolForPreset(UAgentSolverPreset* Preset);
#endif

private:
#if WITH_EDITOR
	/** Spawns the SAgentSolverControlPanel into a nomad dock tab -- see AgentSolver.cpp's RegisterMenus for how it reaches the Window menu, and SAgentSolverControlPanel.h for what it does. */
	TSharedRef<SDockTab> SpawnControlPanelTab(const FSpawnTabArgs& SpawnTabArgs);

	/** Spawns the SAgentSolverAIDebugPanel into its own nomad dock tab -- separate from the control panel so it can sit side-by-side with it, same reasoning as MassMuscleProfile's own multi-tab tools. See SAgentSolverAIDebugPanel.h. */
	TSharedRef<SDockTab> SpawnAIDebugPanelTab(const FSpawnTabArgs& SpawnTabArgs);

	/** Adds the "Agent Solver" and "Agent Solver AI Debug" entries to Window > (WindowLayout section), same ToolMenus pattern MassMuscleProfile uses for its own Window-menu entry. */
	void RegisterMenus();

	/** Set by OpenToolForPreset, consumed by SpawnControlPanelTab. */
	TWeakObjectPtr<UAgentSolverPreset> PendingPresetToOpen;

	/** The live panel widget, if the tab is currently spawned -- lets OpenToolForPreset push a preset into an already-open tab (TryInvokeTab only focuses it in that case, it doesn't re-run SpawnControlPanelTab). */
	TWeakPtr<SAgentSolverControlPanel> ActivePanel;

	/** Registered in StartupModule so double-clicking a UAgentSolverPreset asset opens this tool instead of the generic "no editor for this asset" fallback -- see AgentSolverPresetAssetActions.h. */
	TSharedPtr<FAgentSolverPresetAssetActions> PresetAssetActions;
#endif
};
