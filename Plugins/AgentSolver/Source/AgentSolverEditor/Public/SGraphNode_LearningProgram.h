#pragma once

// Custom visual for ULearningProgramGraphNode -- during PIE, dims every
// stage except the one AMutoRLTrainingDriver::ActiveLearningProgram is
// currently on (SetRenderOpacity, same universal SWidget mechanism used
// elsewhere for fade effects) and shows a progress bar per enabled
// transition underneath the active node (CreateBelowWidgetControls -- the
// engine's own documented "add widgets below the node and pins" hook, the
// same one Behavior Tree's debugger view builds on for its own live
// PIE-state visuals). Registered via FGraphPanelNodeFactory in
// FAgentSolverEditorModule::StartupModule.
//
// Polls the driver itself, throttled, rather than going through a shared
// toolkit-level poller -- a Learning Program graph has at most a handful of
// stages, so every node independently taking a lock-guarded snapshot every
// ~0.25s is negligible cost, and it keeps this widget self-contained.

#include "CoreMinimal.h"
#include "SGraphNode.h"

class ULearningProgramGraphNode;
class SProgressBar;
class SVerticalBox;

class SGraphNode_LearningProgram : public SGraphNode
{
public:
	SLATE_BEGIN_ARGS(SGraphNode_LearningProgram) {}
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs, ULearningProgramGraphNode* InNode);

	//~ Begin SGraphNode interface
	virtual void CreateBelowWidgetControls(TSharedPtr<SVerticalBox> MainBox) override;
	virtual void Tick(const FGeometry& AllottedGeometry, const double InCurrentTime, const float InDeltaTime) override;
	//~ End SGraphNode interface

private:
	/** One row (label + bar) per RuntimeNode->Transitions entry, built once in CreateBelowWidgetControls and rebuilt whenever the node's pin/transition count changes (UpdateGraphNode reruns -- see AddTransitionPin/RemoveTransitionPin's NotifyGraphChanged calls). Index-parallel with RuntimeNode->Transitions. */
	TArray<TSharedPtr<SProgressBar>> TransitionProgressBars;

	/** The whole "below the node" area -- collapsed entirely except on the currently active node, so inactive stages don't show a wall of meaningless zeroed bars. */
	TSharedPtr<SVerticalBox> ProgressBarsContainer;

	float PollAccumulatedSeconds = 0.0f;
};
