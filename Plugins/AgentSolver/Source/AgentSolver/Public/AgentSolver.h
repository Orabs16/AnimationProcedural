// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "Modules/ModuleManager.h"

class SDockTab;
class FSpawnTabArgs;

class FAgentSolverModule : public IModuleInterface
{
public:

	/** IModuleInterface implementation */
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;

private:
#if WITH_EDITOR
	/** Spawns the SAgentSolverControlPanel into a nomad dock tab -- see AgentSolver.cpp's RegisterMenus for how it reaches the Window menu, and SAgentSolverControlPanel.h for what it does. */
	TSharedRef<SDockTab> SpawnControlPanelTab(const FSpawnTabArgs& SpawnTabArgs);

	/** Adds the "Agent Solver" entry to Window > (WindowLayout section), same ToolMenus pattern MassMuscleProfile uses for its own Window-menu entry. */
	void RegisterMenus();
#endif
};
