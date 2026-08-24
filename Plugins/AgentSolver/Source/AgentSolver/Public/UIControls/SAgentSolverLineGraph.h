#pragma once

// Minimal Slate line-graph widget for the AgentSolver control panel's
// reward/throughput readouts (see SAgentSolverControlPanel.h) -- a plain
// SLeafWidget, not backed by any external charting library. Ring-buffered
// samples (oldest evicted once MaxSamples is exceeded), normalized to the
// visible samples' own min/max range each paint (so the line always uses the
// full height regardless of the value scale) and drawn as one polyline via
// FSlateDrawElement::MakeLines. A faint baseline is always drawn too, so the
// widget isn't blank before AddSample has ever been called.

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
		/** Ring-buffer capacity -- oldest sample evicted once exceeded. */
		SLATE_ARGUMENT(int32, MaxSamples)
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs);

	/** Pushes a new sample, evicting the oldest once MaxSamples is exceeded. */
	void AddSample(float Value);

	//~ Begin SWidget Interface
	virtual int32 OnPaint(const FPaintArgs& Args, const FGeometry& AllottedGeometry, const FSlateRect& MyCullingRect, FSlateWindowElementList& OutDrawElements, int32 LayerId, const FWidgetStyle& InWidgetStyle, bool bParentEnabled) const override;
	virtual FVector2D ComputeDesiredSize(float) const override;
	//~ End SWidget Interface

private:
	TArray<float> Samples;
	int32 MaxSamples = 120;
	FLinearColor LineColor = FLinearColor::White;
};
