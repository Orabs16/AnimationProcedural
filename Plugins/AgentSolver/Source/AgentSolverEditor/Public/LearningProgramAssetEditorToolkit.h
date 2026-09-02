#pragma once

// The real FAssetEditorToolkit ULearningProgram opens into -- unlike the
// rest of this plugin's UI (SAgentSolverControlPanel, a plain nomad-tab
// SCompoundWidget), this needs a genuine SGraphEditor + per-node details
// panel, which is what FAssetEditorToolkit exists for. See
// FLearningProgramAssetTypeActions::OpenAssetEditor for the entry point.

#include "CoreMinimal.h"
#include "Toolkits/AssetEditorToolkit.h"
#include "UObject/GCObject.h"
#include "GraphEditor.h"

class ULearningProgram;
class ULearningProgramGraphNode;
class IDetailsView;

class FLearningProgramAssetEditorToolkit : public FAssetEditorToolkit, public FGCObject
{
public:
	/** Builds the tab layout and opens the toolkit on Program -- called once, from FLearningProgramAssetTypeActions::OpenAssetEditor. */
	void InitLearningProgramEditor(const EToolkitMode::Type Mode, const TSharedPtr<IToolkitHost>& InitToolkitHost, ULearningProgram* Program);

	//~ Begin FAssetEditorToolkit interface
	virtual void RegisterTabSpawners(const TSharedRef<class FTabManager>& InTabManager) override;
	virtual void UnregisterTabSpawners(const TSharedRef<class FTabManager>& InTabManager) override;
	virtual FName GetToolkitFName() const override;
	virtual FText GetBaseToolkitName() const override;
	virtual FString GetWorldCentricTabPrefix() const override;
	virtual FLinearColor GetWorldCentricTabColorScale() const override;
	//~ End FAssetEditorToolkit interface

	//~ Begin FGCObject interface
	virtual void AddReferencedObjects(FReferenceCollector& Collector) override;
	virtual FString GetReferencerName() const override { return TEXT("FLearningProgramAssetEditorToolkit"); }
	//~ End FGCObject interface

private:
	TSharedRef<SDockTab> SpawnGraphTab(const FSpawnTabArgs& Args);
	TSharedRef<SDockTab> SpawnDetailsTab(const FSpawnTabArgs& Args);

	/** Pushes the selected graph node's RuntimeNode (or nullptr, if none/multiple selected) into DetailsView. */
	void OnGraphSelectionChanged(const FGraphPanelSelectionSet& NewSelection);

	/** Refreshes the selected node's output pin labels after a Details-panel edit (e.g. a Transition's Condition/ThresholdValue/bEnabled) -- edits to existing struct fields don't go through AddTransitionPin/RemoveTransitionPin, so pin labels would otherwise go stale. */
	void OnNodeDetailsChanged(const FPropertyChangedEvent& Event);

	TObjectPtr<ULearningProgram> LearningProgram;
	TSharedPtr<SGraphEditor> GraphEditorWidget;
	TSharedPtr<IDetailsView> DetailsView;

	static const FName GraphTabId;
	static const FName DetailsTabId;
};
