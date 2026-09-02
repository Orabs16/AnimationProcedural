#pragma once

// Minimal Slate line-graph widget for the AgentSolver control panel's
// reward/throughput readouts (see SAgentSolverControlPanel.h) -- a plain
// SLeafWidget, not backed by any external charting library.
//
// History retention: once the buffer reaches MaxSamples, it is NOT evicted
// oldest-first -- it is compressed by averaging every adjacent pair, halving
// the sample count, and new samples keep landing in the freed half. This
// means the graph always covers the full history back to the widget's first
// AddSample call, just at exponentially coarser resolution the further back
// you look (the most recent stretch since the last compression is always at
// full per-sample resolution). A fixed-size sliding window would instead
// silently drop everything older than MaxSamples samples.
//
// Y-axis: eased toward the visible samples' min/max rather than snapped to
// it every paint, so a single outlier (e.g. the transient overshoot right
// after a PPO policy update) doesn't rescale -- and thus flatten -- the rest
// of the graph; it just clips at the top/bottom edge until the eased range
// catches up. Drawn as one polyline via FSlateDrawElement::MakeLines. A
// faint baseline is always drawn too, so the widget isn't blank before
// AddSample has ever been called.

#include "CoreMinimal.h"
#include "Widgets/SLeafWidget.h"

class SAgentSolverLineGraph : public SLeafWidget
{
public:
	SLATE_BEGIN_ARGS(SAgentSolverLineGraph)
		: _LineColor(FLinearColor::White)
		, _MaxSamples(120)
		{}
		SLATE_ARGUMENT(FLinearColor, LineColor)
		/** Buffer capacity before adjacent samples start getting compressed (averaged) in pairs. */
		SLATE_ARGUMENT(int32, MaxSamples)
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs);

	/** Pushes a new sample; compresses the whole buffer 2:1 once MaxSamples is exceeded. */
	void AddSample(float Value);

	//~ Begin SWidget Interface
	virtual int32 OnPaint(const FPaintArgs& Args, const FGeometry& AllottedGeometry, const FSlateRect& MyCullingRect, FSlateWindowElementList& OutDrawElements, int32 LayerId, const FWidgetStyle& InWidgetStyle, bool bParentEnabled) const override;
	virtual FVector2D ComputeDesiredSize(float) const override;
	//~ End SWidget Interface

private:
	TArray<float> Samples;
	int32 MaxSamples = 120;
	FLinearColor LineColor = FLinearColor::White;

	// Eased Y-axis range -- see the file-level comment above. Updated once
	// per AddSample (i.e. once per real data point, independent of how often
	// OnPaint happens to run) rather than in OnPaint, so the easing rate is
	// tied to the data's own cadence.
	float SmoothedMinValue = 0.0f;
	float SmoothedMaxValue = 0.0f;
	bool bHasSmoothedRange = false;
};
