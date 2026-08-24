#include "UIControls/SAgentSolverLineGraph.h"

#include "Rendering/DrawElements.h"

void SAgentSolverLineGraph::Construct(const FArguments& InArgs)
{
	LineColor = InArgs._LineColor;
	MaxSamples = FMath::Max(2, InArgs._MaxSamples);
	Samples.Reserve(MaxSamples);
}

void SAgentSolverLineGraph::AddSample(float Value)
{
	Samples.Add(Value);
	if (Samples.Num() > MaxSamples)
	{
		// Ring-buffer eviction -- MaxSamples is small (~120-150) and this only
		// runs a few times a second at most, so the O(n) shift from RemoveAt(0)
		// is not worth a circular-index scheme here.
		Samples.RemoveAt(0);
	}
}

FVector2D SAgentSolverLineGraph::ComputeDesiredSize(float) const
{
	return FVector2D(200.0f, 60.0f);
}

int32 SAgentSolverLineGraph::OnPaint(const FPaintArgs& Args, const FGeometry& AllottedGeometry, const FSlateRect& MyCullingRect, FSlateWindowElementList& OutDrawElements, int32 LayerId, const FWidgetStyle& InWidgetStyle, bool bParentEnabled) const
{
	const FVector2D LocalSize = AllottedGeometry.GetLocalSize();
	const float MidY = LocalSize.Y * 0.5f;

	// Always drawn, even with zero samples -- otherwise the widget is
	// indistinguishable from "not rendering at all" before the first sample.
	TArray<FVector2D> BaselinePoints;
	BaselinePoints.Add(FVector2D(0.0f, MidY));
	BaselinePoints.Add(FVector2D(LocalSize.X, MidY));
	FSlateDrawElement::MakeLines(
		OutDrawElements, LayerId, AllottedGeometry.ToPaintGeometry(), BaselinePoints,
		ESlateDrawEffect::None, FLinearColor(1.0f, 1.0f, 1.0f, 0.15f), true, 1.0f);

	if (Samples.Num() >= 2)
	{
		// Normalized to THIS paint's own visible min/max -- the point is a
		// readable trend line, not an absolute-scale plot, and the value
		// range (e.g. AverageReward vs StepsPerSecond) varies wildly between
		// what feeds this widget.
		float MinValue = Samples[0];
		float MaxValue = Samples[0];
		for (const float Value : Samples)
		{
			MinValue = FMath::Min(MinValue, Value);
			MaxValue = FMath::Max(MaxValue, Value);
		}
		const float Range = FMath::Max(MaxValue - MinValue, KINDA_SMALL_NUMBER);

		TArray<FVector2D> Points;
		Points.Reserve(Samples.Num());
		for (int32 Index = 0; Index < Samples.Num(); ++Index)
		{
			const float X = (LocalSize.X * Index) / (float)(Samples.Num() - 1);
			const float NormalizedY = (Samples[Index] - MinValue) / Range;
			// Y grows downward in local space -- flip so a HIGHER value draws HIGHER on screen.
			const float Y = LocalSize.Y - (NormalizedY * LocalSize.Y);
			Points.Add(FVector2D(X, Y));
		}
		FSlateDrawElement::MakeLines(
			OutDrawElements, LayerId + 1, AllottedGeometry.ToPaintGeometry(), Points,
			ESlateDrawEffect::None, LineColor, true, 2.0f);
	}

	return LayerId + 1;
}
