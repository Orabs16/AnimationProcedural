#pragma once

// One AI-debug "card" for SAgentSolverAIDebugPanel: a variable-name label
// over a bipolar [-1,1] gauge bar over a numeric readout. Two widgets in
// this one file/pair, same grouping SAgentSolverLineGraph uses for its own
// single-purpose paint widget:
//  - SAIBipolarGaugeBar: a plain SLeafWidget (no external charting library,
//    same approach as SAgentSolverLineGraph) that paints a centered bar --
//    neutral at 0, fills LEFT (red/orange) for negative, RIGHT (green) for
//    positive.
//  - SAIValueGaugeCard: the labeled card wrapping it.
//
// SetValue() on either widget updates the existing widget in place (a float
// field + an explicit repaint/text invalidation) -- it never re-creates
// child widgets, which is what lets SAgentSolverAIDebugPanel::
// UpdateAIDebugData refresh potentially hundreds of these every tick without
// Slate churn.

#include "CoreMinimal.h"
#include "Widgets/SLeafWidget.h"
#include "Widgets/SCompoundWidget.h"

class STextBlock;

class SAIBipolarGaugeBar : public SLeafWidget
{
public:
	SLATE_BEGIN_ARGS(SAIBipolarGaugeBar)
		: _Value(0.0f)
		{}
		SLATE_ARGUMENT(float, Value)
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs);

	/** Updates the fill in place and requests a repaint -- no layout/child changes. */
	void SetValue(float InValue);

	//~ Begin SWidget Interface
	virtual int32 OnPaint(const FPaintArgs& Args, const FGeometry& AllottedGeometry, const FSlateRect& MyCullingRect, FSlateWindowElementList& OutDrawElements, int32 LayerId, const FWidgetStyle& InWidgetStyle, bool bParentEnabled) const override;
	virtual FVector2D ComputeDesiredSize(float) const override;
	//~ End SWidget Interface

private:
	float Value = 0.0f;
};

class SAIValueGaugeCard : public SCompoundWidget
{
public:
	SLATE_BEGIN_ARGS(SAIValueGaugeCard) {}
	SLATE_END_ARGS()

	/** InVariableName is fixed for this card's lifetime -- a renamed variable gets a new card (see UpdateSide's name-keyed reuse), not a re-labeled one. */
	void Construct(const FArguments& InArgs, FName InVariableName);

	/** Updates the gauge bar + numeric readout in place. */
	void SetValue(float NormalizedValue);

	FName GetVariableName() const { return VariableName; }

private:
	FName VariableName;
	TSharedPtr<SAIBipolarGaugeBar> GaugeBar;
	TSharedPtr<STextBlock> ValueText;
};
