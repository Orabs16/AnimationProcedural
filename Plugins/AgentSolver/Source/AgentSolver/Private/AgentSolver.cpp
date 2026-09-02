// Copyright Epic Games, Inc. All Rights Reserved.

#include "AgentSolver.h"

#define LOCTEXT_NAMESPACE "FAgentSolverModule"

#if WITH_EDITOR
#include "UIControls/SAgentSolverControlPanel.h"
#include "UIControls/SAgentSolverAIDebugPanel.h"
#include "UIControls/AgentSolverPresetAssetActions.h"
#include "AgentSolver/AgentSolverPreset.h"
#include "Widgets/Docking/SDockTab.h"
#include "Framework/Docking/TabManager.h"
#include "Framework/Application/SlateApplication.h"
#include "ToolMenus.h"
#include "AssetToolsModule.h"
#include "IAssetTools.h"

static const FName AgentSolverControlPanelTabName(TEXT("AgentSolverControlPanel"));
static const FName AgentSolverAIDebugPanelTabName(TEXT("AgentSolverAIDebugPanel"));
#endif

void FAgentSolverModule::StartupModule()
{
#if WITH_EDITOR
	FGlobalTabmanager::Get()
		->RegisterNomadTabSpawner(AgentSolverControlPanelTabName, FOnSpawnTab::CreateRaw(this, &FAgentSolverModule::SpawnControlPanelTab))
		.SetDisplayName(LOCTEXT("AgentSolverControlPanelTitle", "Agent Solver"))
		.SetMenuType(ETabSpawnerMenuType::Hidden);

	FGlobalTabmanager::Get()
		->RegisterNomadTabSpawner(AgentSolverAIDebugPanelTabName, FOnSpawnTab::CreateRaw(this, &FAgentSolverModule::SpawnAIDebugPanelTab))
		.SetDisplayName(LOCTEXT("AgentSolverAIDebugPanelTitle", "Agent Solver AI Debug"))
		.SetMenuType(ETabSpawnerMenuType::Hidden);

	UToolMenus::RegisterStartupCallback(
		FSimpleMulticastDelegate::FDelegate::CreateRaw(this, &FAgentSolverModule::RegisterMenus));

	// Double-clicking a UAgentSolverPreset asset opens this tool with it
	// loaded instead of the generic "no editor for this asset type"
	// fallback -- see AgentSolverPresetAssetActions.h.
	IAssetTools& AssetTools = FModuleManager::LoadModuleChecked<FAssetToolsModule>("AssetTools").Get();
	PresetAssetActions = MakeShared<FAgentSolverPresetAssetActions>();
	AssetTools.RegisterAssetTypeActions(PresetAssetActions.ToSharedRef());
#endif
}

void FAgentSolverModule::ShutdownModule()
{
#if WITH_EDITOR
	FGlobalTabmanager::Get()->UnregisterNomadTabSpawner(AgentSolverControlPanelTabName);
	FGlobalTabmanager::Get()->UnregisterNomadTabSpawner(AgentSolverAIDebugPanelTabName);

	if (FModuleManager::Get().IsModuleLoaded("AssetTools"))
	{
		IAssetTools& AssetTools = FModuleManager::GetModuleChecked<FAssetToolsModule>("AssetTools").Get();
		AssetTools.UnregisterAssetTypeActions(PresetAssetActions.ToSharedRef());
	}
#endif
}

#if WITH_EDITOR
FAgentSolverModule& FAgentSolverModule::Get()
{
	return FModuleManager::LoadModuleChecked<FAgentSolverModule>("AgentSolver");
}

TSharedRef<SDockTab> FAgentSolverModule::SpawnControlPanelTab(const FSpawnTabArgs& SpawnTabArgs)
{
	TSharedRef<SAgentSolverControlPanel> Panel = SNew(SAgentSolverControlPanel);
	ActivePanel = Panel;

	// Preset the panel loads on open: whatever was double-clicked to get
	// here (see OpenToolForPreset), or -- manually opening the tool with
	// nothing pending -- the first UAgentSolverPreset the asset registry
	// finds. Same "pending asset, else grab first in registry" split
	// FMassMuscleProfileModule::SpawnPluginTab uses.
	if (PendingPresetToOpen.IsValid())
	{
		Panel->LoadPreset(PendingPresetToOpen.Get());
		PendingPresetToOpen = nullptr;
	}
	else
	{
		Panel->LoadFirstAvailablePreset();
	}

	return SNew(SDockTab)
		.TabRole(ETabRole::NomadTab)
		[
			Panel
		];
}

TSharedRef<SDockTab> FAgentSolverModule::SpawnAIDebugPanelTab(const FSpawnTabArgs& SpawnTabArgs)
{
	return SNew(SDockTab)
		.TabRole(ETabRole::NomadTab)
		[
			SNew(SAgentSolverAIDebugPanel)
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

	Section.AddMenuEntry(NAME_None, LOCTEXT("AgentSolverAIDebugMenuEntry", "Agent Solver AI Debug"),
		LOCTEXT("AgentSolverAIDebugMenuEntryTooltip", "Open the real-time AI input/output debug window"), FSlateIcon(),
		FToolUIActionChoice(FExecuteAction::CreateLambda(
			[]() { FGlobalTabmanager::Get()->TryInvokeTab(AgentSolverAIDebugPanelTabName); })));
}

void FAgentSolverModule::OpenToolForPreset(UAgentSolverPreset* Preset)
{
	PendingPresetToOpen = Preset;

	// If the tab doesn't exist yet, this calls SpawnControlPanelTab, which
	// consumes PendingPresetToOpen. If it already exists, this just focuses
	// it -- SpawnControlPanelTab does NOT run again, so the already-live
	// panel needs to be told about the preset directly instead.
	FGlobalTabmanager::Get()->TryInvokeTab(AgentSolverControlPanelTabName);

	if (ActivePanel.IsValid() && PendingPresetToOpen.IsValid())
	{
		ActivePanel.Pin()->LoadPreset(PendingPresetToOpen.Get());
		PendingPresetToOpen = nullptr;
	}
}
#endif

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FAgentSolverModule, AgentSolver)
