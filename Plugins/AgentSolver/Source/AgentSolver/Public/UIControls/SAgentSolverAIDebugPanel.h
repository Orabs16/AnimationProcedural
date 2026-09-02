#pragma once

// Real-time AI input/output debug window: horizontal split, Inputs
// (observations) on the left / Outputs (actions) on the right, each side a
// scrollable grid of SAIValueGaugeCard bipolar gauges. See
// AIValueDebugData.h for the per-gauge data record and
// UIControls/SAIValueGaugeCard.h for the gauge widget itself.
//
// Live data source: AMutoRLVisualizerActor (see AgentSolver/MutoRLVisualizer.h)
// -- the single-env, real-time policy PLAYBACK actor, not the headless
// multi-env training driver (that one runs NumEnvs agents on a background
// thread with no single "the" observation/action to show). Tick() below
// pulls AMutoRLVisualizerActor::LastObservation/LastNormalizedActions +
// ObservationNames/ActionNames (captured once per its own Tick(), see that
// class) via AgentSolverUI::FindRLVisualizer(), same PIE-vs-editor-world
// lookup rule every other AgentSolver editor panel uses.
//
// Card reuse (see UpdateAIDebugData/UpdateSide): the common per-tick case is
// the SAME set of variable names in the SAME order as last tick (topology
// doesn't change frame to frame), so cards are matched by name and only
// SetValue() is called -- no Slate widget creation, no layout churn. Cards
// are only created/destroyed when the observation/action layout itself
// changes (e.g. a fresh StartTraining rebuilds the topology), and even then
// any surviving name's card is reused rather than recreated.

#include "CoreMinimal.h"
#include "Widgets/SCompoundWidget.h"
#include "UObject/WeakObjectPtr.h"
#include "UIControls/AIValueDebugData.h"

class SWrapBox;
class SScrollBox;
class SAIValueGaugeCard;
class AMutoRLVisualizerActor;

class SAgentSolverAIDebugPanel : public SCompoundWidget
{
public:
	SLATE_BEGIN_ARGS(SAgentSolverAIDebugPanel) {}
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs);

	/**
	 * Updates the Input/Output gauge grids from the given data. Efficient,
	 * per the class comment: reuses existing cards by VariableName, and only
	 * touches Slate's widget tree (add/remove/reorder) when the incoming set
	 * of names actually differs from last call -- otherwise this is just a
	 * loop of SAIValueGaugeCard::SetValue() calls.
	 */
	void UpdateAIDebugData(const TArray<FAIValueDebugData>& InputData, const TArray<FAIValueDebugData>& OutputData);

	//~ Begin SWidget Interface
	virtual void Tick(const FGeometry& AllottedGeometry, const double InCurrentTime, const float InDeltaTime) override;
	//~ End SWidget Interface

private:
	/** One side (Inputs or Outputs) of the split -- its own scroll container and name-keyed card cache. */
	struct FDebugSide
	{
		TSharedPtr<SWrapBox> Container;
		TMap<FName, TSharedPtr<SAIValueGaugeCard>> CardsByName;
		/** VariableName order as of the last UpdateSide call -- compared against the incoming data's order to detect the common "nothing changed" fast path. */
		TArray<FName> OrderedNames;
	};

	TSharedRef<SWidget> BuildSideWidget(FDebugSide& Side, const FText& Title);
	void UpdateSide(FDebugSide& Side, const TArray<FAIValueDebugData>& Data);

	FText GetStatusText() const;

	FDebugSide InputSide;
	FDebugSide OutputSide;

	/** Set each Tick() -- drives GetStatusText's "no live source" message. */
	TWeakObjectPtr<AMutoRLVisualizerActor> LastKnownVisualizer;
};
