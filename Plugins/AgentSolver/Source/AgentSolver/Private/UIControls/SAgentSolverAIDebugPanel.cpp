#include "UIControls/SAgentSolverAIDebugPanel.h"

#include "UIControls/SAIValueGaugeCard.h"
#include "UIControls/AgentSolverUIUtils.h"
#include "AgentSolver/MutoRLVisualizer.h"
#include "Widgets/Layout/SWrapBox.h"
#include "Widgets/Layout/SScrollBox.h"
#include "Widgets/Layout/SSplitter.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/SBoxPanel.h"
#include "Styling/CoreStyle.h"

#define LOCTEXT_NAMESPACE "SAgentSolverAIDebugPanel"

void SAgentSolverAIDebugPanel::Construct(const FArguments& InArgs)
{
	ChildSlot
	[
		SNew(SVerticalBox)
		+ SVerticalBox::Slot()
		.AutoHeight()
		.Padding(6.0f, 4.0f)
		[
			SNew(STextBlock)
			.Text(this, &SAgentSolverAIDebugPanel::GetStatusText)
		]
		+ SVerticalBox::Slot()
		.FillHeight(1.0f)
		[
			SNew(SSplitter)
			.Orientation(Orient_Horizontal)
			+ SSplitter::Slot()
			.Value(0.5f)
			[
				BuildSideWidget(InputSide, LOCTEXT("Inputs", "Inputs (Observations)"))
			]
			+ SSplitter::Slot()
			.Value(0.5f)
			[
				BuildSideWidget(OutputSide, LOCTEXT("Outputs", "Outputs (Actions)"))
			]
		]
	];
}

TSharedRef<SWidget> SAgentSolverAIDebugPanel::BuildSideWidget(FDebugSide& Side, const FText& Title)
{
	return SNew(SVerticalBox)
		+ SVerticalBox::Slot()
		.AutoHeight()
		.Padding(6.0f, 4.0f)
		[
			SNew(STextBlock)
			.Text(Title)
			.Font(FCoreStyle::GetDefaultFontStyle("Bold", 10))
		]
		+ SVerticalBox::Slot()
		.FillHeight(1.0f)
		[
			SNew(SScrollBox)
			+ SScrollBox::Slot()
			[
				SAssignNew(Side.Container, SWrapBox)
				.UseAllottedSize(true)
				.InnerSlotPadding(FVector2D(4.0f, 4.0f))
			]
		];
}

void SAgentSolverAIDebugPanel::UpdateSide(FDebugSide& Side, const TArray<FAIValueDebugData>& Data)
{
	// Fast path: identical name/order to last call (true almost every tick
	// once the rig's topology is built) -- just push values into the
	// existing cards, no Slate widget tree changes at all.
	bool bLayoutUnchanged = Side.OrderedNames.Num() == Data.Num();
	if (bLayoutUnchanged)
	{
		for (int32 Index = 0; Index < Data.Num(); ++Index)
		{
			if (Side.OrderedNames[Index] != Data[Index].VariableName)
			{
				bLayoutUnchanged = false;
				break;
			}
		}
	}

	if (bLayoutUnchanged)
	{
		for (const FAIValueDebugData& Item : Data)
		{
			if (TSharedPtr<SAIValueGaugeCard>* Card = Side.CardsByName.Find(Item.VariableName))
			{
				(*Card)->SetValue(Item.NormalizedValue);
			}
		}
		return;
	}

	// Slow path: the observation/action layout itself changed. Rebuild the
	// container's slot list, but reuse any card whose name survived rather
	// than destroying and recreating it.
	Side.Container->ClearChildren();

	TMap<FName, TSharedPtr<SAIValueGaugeCard>> NewCardsByName;
	NewCardsByName.Reserve(Data.Num());
	Side.OrderedNames.Reset(Data.Num());

	for (const FAIValueDebugData& Item : Data)
	{
		TSharedPtr<SAIValueGaugeCard> Card;
		if (TSharedPtr<SAIValueGaugeCard>* Existing = Side.CardsByName.Find(Item.VariableName))
		{
			Card = *Existing;
		}
		else
		{
			Card = SNew(SAIValueGaugeCard, Item.VariableName);
		}
		Card->SetValue(Item.NormalizedValue);

		Side.Container->AddSlot()
		[
			Card.ToSharedRef()
		];

		NewCardsByName.Add(Item.VariableName, Card);
		Side.OrderedNames.Add(Item.VariableName);
	}

	Side.CardsByName = MoveTemp(NewCardsByName);
}

void SAgentSolverAIDebugPanel::UpdateAIDebugData(const TArray<FAIValueDebugData>& InputData, const TArray<FAIValueDebugData>& OutputData)
{
	UpdateSide(InputSide, InputData);
	UpdateSide(OutputSide, OutputData);
}

void SAgentSolverAIDebugPanel::Tick(const FGeometry& AllottedGeometry, const double InCurrentTime, const float InDeltaTime)
{
	SCompoundWidget::Tick(AllottedGeometry, InCurrentTime, InDeltaTime);

	AMutoRLVisualizerActor* Visualizer = AgentSolverUI::FindRLVisualizer();
	LastKnownVisualizer = Visualizer;
	if (!Visualizer)
	{
		return;
	}

	// Guards against reading mid-rebuild: ObservationNames/ActionNames are
	// only rewritten inside StartTraining(), while LastObservation/
	// LastNormalizedActions are rewritten every Tick() -- a one-frame
	// mismatch is possible right as StartTraining() runs, and would
	// otherwise pair a stale name with the wrong value.
	if (Visualizer->ObservationNames.Num() != Visualizer->LastObservation.Num()
		|| Visualizer->ActionNames.Num() != Visualizer->LastNormalizedActions.Num())
	{
		return;
	}

	TArray<FAIValueDebugData> Inputs;
	Inputs.Reserve(Visualizer->LastObservation.Num());
	for (int32 Index = 0; Index < Visualizer->LastObservation.Num(); ++Index)
	{
		Inputs.Emplace(Visualizer->ObservationNames[Index], Visualizer->LastObservation[Index], EAIValueDebugCategory::Input);
	}

	TArray<FAIValueDebugData> Outputs;
	Outputs.Reserve(Visualizer->LastNormalizedActions.Num());
	for (int32 Index = 0; Index < Visualizer->LastNormalizedActions.Num(); ++Index)
	{
		Outputs.Emplace(Visualizer->ActionNames[Index], Visualizer->LastNormalizedActions[Index], EAIValueDebugCategory::Output);
	}

	UpdateAIDebugData(Inputs, Outputs);
}

FText SAgentSolverAIDebugPanel::GetStatusText() const
{
	return LastKnownVisualizer.IsValid()
		? FText::Format(LOCTEXT("LiveSource", "Live source: {0}"), FText::FromString(LastKnownVisualizer->GetName()))
		: LOCTEXT("NoSource", "No AMutoRLVisualizerActor found in the current world -- place one, assign SourceTrainingDriver, and press Play.");
}

#undef LOCTEXT_NAMESPACE
