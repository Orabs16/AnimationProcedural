#include "LearningProgramAssetEditorToolkit.h"
#include "AgentSolver/LearningProgram.h"
#include "LearningProgramGraphNode.h"
#include "LearningProgramEntryGraphNode.h"
#include "LearningProgramEdGraph.h"
#include "LearningProgramEdGraphSchema.h"
#include "GraphEditor.h"
#include "PropertyEditorModule.h"
#include "IDetailsView.h"
#include "Widgets/Docking/SDockTab.h"
#include "Framework/Docking/TabManager.h"

#define LOCTEXT_NAMESPACE "LearningProgramAssetEditorToolkit"

const FName FLearningProgramAssetEditorToolkit::GraphTabId(TEXT("LearningProgramEditor_Graph"));
const FName FLearningProgramAssetEditorToolkit::DetailsTabId(TEXT("LearningProgramEditor_Details"));

namespace
{
	/**
	 * Repairs an asset that has no Start node -- one saved before
	 * ULearningProgramEntryGraphNode existed, or (CanUserDeleteNode()==false
	 * notwithstanding) one that somehow lost it. StartTraining() can only
	 * ever begin from ULearningProgram::EntryNodeId, and the Start node's
	 * connection is the only thing that sets it (see that class's own
	 * comment), so an asset with no Start node can never be trained from
	 * until one exists. Also creates EdGraph itself if that is somehow
	 * missing too, for full defensiveness.
	 */
	void EnsureEntryNode(ULearningProgram* Program)
	{
		if (!Program->EdGraph)
		{
			Program->Modify();
			Program->EdGraph = NewObject<ULearningProgramEdGraph>(Program, NAME_None, RF_Transactional);
			Program->EdGraph->Schema = ULearningProgramEdGraphSchema::StaticClass();
		}

		for (UEdGraphNode* Node : Program->EdGraph->Nodes)
		{
			if (Node->IsA<ULearningProgramEntryGraphNode>())
			{
				return;
			}
		}

		Program->EdGraph->Modify();
		ULearningProgramEntryGraphNode* EntryNode = NewObject<ULearningProgramEntryGraphNode>(Program->EdGraph, NAME_None, RF_Transactional);
		EntryNode->NodePosX = -300;
		EntryNode->NodePosY = 0;
		EntryNode->CreateNewGuid();
		Program->EdGraph->AddNode(EntryNode, /*bFromUI=*/false, /*bSelectNewNode=*/false);
		EntryNode->PostPlacedNewNode();
		EntryNode->AllocateDefaultPins();

		// If EntryNodeId already names a real stage, wire the new Start node
		// to it so the graph's visual state matches what StartTraining()
		// already does -- otherwise it's left unconnected, same as any
		// other freshly added stage.
		if (const ULearningProgramNode* ExistingEntry = Program->GetEntryNode())
		{
			for (UEdGraphNode* Node : Program->EdGraph->Nodes)
			{
				ULearningProgramGraphNode* StageNode = Cast<ULearningProgramGraphNode>(Node);
				if (StageNode && StageNode->RuntimeNode == ExistingEntry)
				{
					Program->EdGraph->GetSchema()->TryCreateConnection(EntryNode->GetOutputPin(), StageNode->GetInputPin());
					break;
				}
			}
		}
	}
}

void FLearningProgramAssetEditorToolkit::InitLearningProgramEditor(const EToolkitMode::Type Mode, const TSharedPtr<IToolkitHost>& InitToolkitHost, ULearningProgram* Program)
{
	LearningProgram = Program;
	EnsureEntryNode(LearningProgram);

	FPropertyEditorModule& PropertyEditorModule = FModuleManager::LoadModuleChecked<FPropertyEditorModule>("PropertyEditor");
	FDetailsViewArgs DetailsViewArgs;
	DetailsViewArgs.bAllowSearch = false;
	DetailsViewArgs.NameAreaSettings = FDetailsViewArgs::HideNameArea;
	DetailsView = PropertyEditorModule.CreateDetailView(DetailsViewArgs);
	DetailsView->OnFinishedChangingProperties().AddSP(this, &FLearningProgramAssetEditorToolkit::OnNodeDetailsChanged);

	SGraphEditor::FGraphEditorEvents GraphEvents;
	GraphEvents.OnSelectionChanged = SGraphEditor::FOnSelectionChanged::CreateSP(this, &FLearningProgramAssetEditorToolkit::OnGraphSelectionChanged);

	GraphEditorWidget = SNew(SGraphEditor)
		.GraphToEdit(LearningProgram->EdGraph)
		.GraphEvents(GraphEvents);

	const TSharedRef<FTabManager::FLayout> Layout = FTabManager::NewLayout("LearningProgramEditor_Layout_v1")
		->AddArea(
			FTabManager::NewPrimaryArea()->SetOrientation(Orient_Horizontal)
			->Split(
				FTabManager::NewStack()
				->SetSizeCoefficient(0.75f)
				->AddTab(GraphTabId, ETabState::OpenedTab)
			)
			->Split(
				FTabManager::NewStack()
				->SetSizeCoefficient(0.25f)
				->AddTab(DetailsTabId, ETabState::OpenedTab)
			)
		);

	const bool bCreateDefaultStandaloneMenu = true;
	const bool bCreateDefaultToolbar = true;
	InitAssetEditor(Mode, InitToolkitHost, TEXT("LearningProgramEditorApp"), Layout, bCreateDefaultStandaloneMenu, bCreateDefaultToolbar, Program);
}

void FLearningProgramAssetEditorToolkit::RegisterTabSpawners(const TSharedRef<FTabManager>& InTabManager)
{
	FAssetEditorToolkit::RegisterTabSpawners(InTabManager);

	InTabManager->RegisterTabSpawner(GraphTabId, FOnSpawnTab::CreateSP(this, &FLearningProgramAssetEditorToolkit::SpawnGraphTab))
		.SetDisplayName(LOCTEXT("GraphTab", "Graph"));

	InTabManager->RegisterTabSpawner(DetailsTabId, FOnSpawnTab::CreateSP(this, &FLearningProgramAssetEditorToolkit::SpawnDetailsTab))
		.SetDisplayName(LOCTEXT("DetailsTab", "Details"));
}

void FLearningProgramAssetEditorToolkit::UnregisterTabSpawners(const TSharedRef<FTabManager>& InTabManager)
{
	FAssetEditorToolkit::UnregisterTabSpawners(InTabManager);
	InTabManager->UnregisterTabSpawner(GraphTabId);
	InTabManager->UnregisterTabSpawner(DetailsTabId);
}

TSharedRef<SDockTab> FLearningProgramAssetEditorToolkit::SpawnGraphTab(const FSpawnTabArgs& Args)
{
	return SNew(SDockTab)
		.Label(LOCTEXT("GraphTabLabel", "Graph"))
		[
			GraphEditorWidget.ToSharedRef()
		];
}

TSharedRef<SDockTab> FLearningProgramAssetEditorToolkit::SpawnDetailsTab(const FSpawnTabArgs& Args)
{
	return SNew(SDockTab)
		.Label(LOCTEXT("DetailsTabLabel", "Details"))
		[
			DetailsView.ToSharedRef()
		];
}

void FLearningProgramAssetEditorToolkit::OnGraphSelectionChanged(const FGraphPanelSelectionSet& NewSelection)
{
	UObject* SelectedRuntimeNode = nullptr;
	if (NewSelection.Num() == 1)
	{
		if (ULearningProgramGraphNode* GraphNode = Cast<ULearningProgramGraphNode>(*NewSelection.CreateConstIterator()))
		{
			SelectedRuntimeNode = GraphNode->RuntimeNode;
		}
	}
	DetailsView->SetObject(SelectedRuntimeNode);
}

void FLearningProgramAssetEditorToolkit::OnNodeDetailsChanged(const FPropertyChangedEvent& Event)
{
	if (!GraphEditorWidget.IsValid())
	{
		return;
	}
	for (UObject* Selected : GraphEditorWidget->GetSelectedNodes())
	{
		if (ULearningProgramGraphNode* GraphNode = Cast<ULearningProgramGraphNode>(Selected))
		{
			GraphNode->RefreshTransitionPinNames();
		}
	}
}

FName FLearningProgramAssetEditorToolkit::GetToolkitFName() const
{
	return FName("LearningProgramEditor");
}

FText FLearningProgramAssetEditorToolkit::GetBaseToolkitName() const
{
	return LOCTEXT("ToolkitName", "Learning Program Editor");
}

FString FLearningProgramAssetEditorToolkit::GetWorldCentricTabPrefix() const
{
	return TEXT("LearningProgram ");
}

FLinearColor FLearningProgramAssetEditorToolkit::GetWorldCentricTabColorScale() const
{
	return FLinearColor(0.8f, 0.6f, 0.2f);
}

void FLearningProgramAssetEditorToolkit::AddReferencedObjects(FReferenceCollector& Collector)
{
	Collector.AddReferencedObject(LearningProgram);
}

#undef LOCTEXT_NAMESPACE
