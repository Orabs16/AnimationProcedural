// Copyright Epic Games, Inc. All Rights Reserved.

#include "AgentSolver.h"

#define LOCTEXT_NAMESPACE "FAgentSolverModule"

#if WITH_EDITOR
#include "UIControls/SAgentSolverControlPanel.h"
#include "Widgets/Docking/SDockTab.h"
#include "Framework/Docking/TabManager.h"
#include "Framework/Application/SlateApplication.h"
#include "ToolMenus.h"

static const FName AgentSolverControlPanelTabName(TEXT("AgentSolverControlPanel"));
#endif

void FAgentSolverModule::StartupModule()
{
#if WITH_EDITOR
	FGlobalTabmanager::Get()
		->RegisterNomadTabSpawner(AgentSolverControlPanelTabName, FOnSpawnTab::CreateRaw(this, &FAgentSolverModule::SpawnControlPanelTab))
		.SetDisplayName(LOCTEXT("AgentSolverControlPanelTitle", "Agent Solver"))
		.SetMenuType(ETabSpawnerMenuType::Hidden);

	UToolMenus::RegisterStartupCallback(
		FSimpleMulticastDelegate::FDelegate::CreateRaw(this, &FAgentSolverModule::RegisterMenus));
#endif
}

void FAgentSolverModule::ShutdownModule()
{
#if WITH_EDITOR
	FGlobalTabmanager::Get()->UnregisterNomadTabSpawner(AgentSolverControlPanelTabName);
#endif
}

#if WITH_EDITOR
TSharedRef<SDockTab> FAgentSolverModule::SpawnControlPanelTab(const FSpawnTabArgs& SpawnTabArgs)
{
	return SNew(SDockTab)
		.TabRole(ETabRole::NomadTab)
		[
			SNew(SAgentSolverControlPanel)
		];
}

void FAgentSolverModule::RegisterMenus()
{
	FToolMenuOwnerScoped OwnerScoped(this);

	UToolMenu* Menu = UToolMenus::Get()->ExtendMenu("LevelEditor.MainMenu.Window");
	FToolMenuSection& Section = Menu->FindOrAddSection("WindowLayout");

	Section.AddMenuEntry(NAME_None, LOCTEXT("AgentSolverMenuEntry", "Agent Solver"),
		LOCTEXT("AgentSolverMenuEntryTooltip", "Open the Agent Solver training control panel"), FSlateIcon(),
		FToolUIActionChoice(FExecuteAction::CreateLambda(
			[]() { FGlobalTabmanager::Get()->TryInvokeTab(AgentSolverControlPanelTabName); })));
}
#endif

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FAgentSolverModule, AgentSolver)
