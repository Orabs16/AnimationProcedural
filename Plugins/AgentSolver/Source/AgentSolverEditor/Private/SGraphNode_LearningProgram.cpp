#include "SGraphNode_LearningProgram.h"
#include "LearningProgramGraphNode.h"
#include "AgentSolver/LearningProgram.h"
#include "AgentSolver/MutoRLTrainingDriver.h"
#include "UIControls/AgentSolverUIUtils.h"
#include "EdGraph/EdGraph.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Notifications/SProgressBar.h"
#include "Widgets/Text/STextBlock.h"

namespace
{
	/** How often to re-poll the driver -- a Learning Program graph has a handful of nodes at most, but no reason to take a lock and iterate every single frame. */
	constexpr float PollIntervalSeconds = 0.25f;

	/** Dimmed opacity for every stage except the currently active one. Full (1.0) whenever there is no live training to compare against, so the graph looks normal outside PIE. */
	constexpr float InactiveOpacity = 0.35f;
}

void SGraphNode_LearningProgram::Construct(const FArguments& InArgs, ULearningProgramGraphNode* InNode)
{
	GraphNode = InNode;
	UpdateGraphNode();
}

void SGraphNode_LearningProgram::CreateBelowWidgetControls(TSharedPtr<SVerticalBox> MainBox)
{
	TransitionProgressBars.Reset();

	TSharedRef<SVerticalBox> Box = SNew(SVerticalBox);

	const ULearningProgramGraphNode* MyNode = Cast<ULearningProgramGraphNode>(GraphNode);
	if (MyNode && MyNode->RuntimeNode)
	{
		for (const FLearningProgramTransition& Transition : MyNode->RuntimeNode->Transitions)
		{
			TSharedPtr<SProgressBar> Bar;
			Box->AddSlot()
				.AutoHeight()
				.Padding(4.0f, 1.0f)
				[
					SNew(SVerticalBox)
					+ SVerticalBox::Slot()
					.AutoHeight()
					[
						SNew(STextBlock)
						.Text(ULearningProgramGraphNode::DescribeTransition(Transition))
						.Font(FCoreStyle::GetDefaultFontStyle("Regular", 7))
					]
					+ SVerticalBox::Slot()
					.AutoHeight()
					.Padding(0.0f, 1.0f)
					[
						SAssignNew(Bar, SProgressBar)
						.Percent(0.0f)
					]
				];
			TransitionProgressBars.Add(Bar);
		}
	}

	ProgressBarsContainer = Box;
	ProgressBarsContainer->SetVisibility(EVisibility::Collapsed);

	MainBox->AddSlot()
		.AutoHeight()
		.Padding(0.0f, 2.0f, 0.0f, 0.0f)
		[
			ProgressBarsContainer.ToSharedRef()
		];
}

void SGraphNode_LearningProgram::Tick(const FGeometry& AllottedGeometry, const double InCurrentTime, const float InDeltaTime)
{
	SGraphNode::Tick(AllottedGeometry, InCurrentTime, InDeltaTime);

	PollAccumulatedSeconds += InDeltaTime;
	if (PollAccumulatedSeconds < PollIntervalSeconds)
	{
		return;
	}
	PollAccumulatedSeconds = 0.0f;

	ULearningProgramGraphNode* MyNode = Cast<ULearningProgramGraphNode>(GraphNode);
	if (!MyNode || !MyNode->RuntimeNode)
	{
		return;
	}

	const ULearningProgram* Program = MyNode->GetGraph() ? Cast<ULearningProgram>(MyNode->GetGraph()->GetOuter()) : nullptr;
	AMutoRLTrainingDriver* Driver = AgentSolverUI::FindTrainingDriver();

	bool bActive = false;
	FLearningProgramLiveState LiveState;
	if (Driver && Program && Driver->ActiveLearningProgram == Program)
	{
		LiveState = Driver->GetLearningProgramLiveState();
		bActive = LiveState.bValid && LiveState.CurrentNodeId == MyNode->RuntimeNode->NodeId;
	}

	// Only actually dim once there IS a live driver to compare against --
	// bDriverPresent false (no PIE, or PIE with a different/no Learning
	// Program active) means every node stays full brightness, same as
	// before this feature existed.
	const bool bDriverPresent = Driver && Program && Driver->ActiveLearningProgram == Program && LiveState.bValid;
	SetRenderOpacity(bDriverPresent && !bActive ? InactiveOpacity : 1.0f);

	if (ProgressBarsContainer.IsValid())
	{
		ProgressBarsContainer->SetVisibility(bActive ? EVisibility::Visible : EVisibility::Collapsed);
	}

	if (bActive)
	{
		const TArray<FLearningProgramTransition>& Transitions = MyNode->RuntimeNode->Transitions;
		for (int32 Index = 0; Index < TransitionProgressBars.Num(); ++Index)
		{
			if (!TransitionProgressBars[Index].IsValid() || !Transitions.IsValidIndex(Index))
			{
				continue;
			}

			const FLearningProgramTransition& Transition = Transitions[Index];
			float Fraction = 0.0f;
			if (Transition.bEnabled && Transition.ThresholdValue > 0.0f)
			{
				const float Current = (Transition.Condition == ELearningProgramConditionType::AverageRewardTarget)
					? LiveState.AverageReward
					: (float)LiveState.StepsSinceNodeEntry;
				Fraction = FMath::Clamp(Current / Transition.ThresholdValue, 0.0f, 1.0f);
			}
			TransitionProgressBars[Index]->SetPercent(Fraction);
		}
	}
}
