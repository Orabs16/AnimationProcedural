#include "UIControls/SAgentSolverLineGraph.h"

#include "Rendering/DrawElements.h"

void SAgentSolverLineGraph::Construct(const FArguments& InArgs)
{
	LineColor = InArgs._LineColor;
	MaxSamples = FMath::Max(2, InArgs._MaxSamples);
	Samples.Reserve(MaxSamples + 1);
}

void SAgentSolverLineGraph::AddSample(float Value)
{
	Samples.Add(Value);

	if (Samples.Num() > MaxSamples)
	{
		// 2:1 compression instead of RemoveAt(0) eviction -- see the header's
		// file-level comment. AddSample only ever adds one sample at a time,
		// so this only ever fires with Samples.Num() == MaxSamples + 1.
		const int32 NewNum = Samples.Num() / 2;
		for (int32 Index = 0; Index < NewNum; ++Index)
		{
			Samples[Index] = (Samples[2 * Index] + Samples[2 * Index + 1]) * 0.5f;
		}
		if (Samples.Num() % 2 == 1)
		{
			// Odd count: carry the final unpaired sample over uncompressed
			// rather than dropping it -- it simply isn't averaged with
			// anything yet, and will be the first half of a pair next time.
			Samples[NewNum] = Samples[Samples.Num() - 1];
			Samples.SetNum(NewNum + 1);
		}
		else
		{
			Samples.SetNum(NewNum);
		}
	}

	float RawMin = Samples[0];
	float RawMax = Samples[0];
	for (const float Sample : Samples)
	{
		RawMin = FMath::Min(RawMin, Sample);
		RawMax = FMath::Max(RawMax, Sample);
	}

	if (!bHasSmoothedRange)
	{
		SmoothedMinValue = RawMin;
		SmoothedMaxValue = RawMax;
		bHasSmoothedRange = true;
	}
	else
	{
		// Slow-moving on purpose -- ~10 samples to substantially catch up to
		// a genuine, sustained shift in range, so one outlier sample (e.g.
		// the overshoot right after a PPO policy update) only nudges the
		// axis instead of snapping it. See the header comment.
		constexpr float SmoothingAlpha = 0.1f;
		SmoothedMinValue = FMath::Lerp(SmoothedMinValue, RawMin, SmoothingAlpha);
		SmoothedMaxValue = FMath::Lerp(SmoothedMaxValue, RawMax, SmoothingAlpha);
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
		// Normalized to the EASED min/max (SmoothedMinValue/SmoothedMaxValue,
		// updated once per AddSample -- see header comment), not this paint's
		// instantaneous range. A sample outside the eased range is clamped
		// to [0,1] below, so it draws as a flat clip at the top/bottom edge
		// instead of rescaling -- and flattening -- the rest of the line.
		const float Range = FMath::Max(SmoothedMaxValue - SmoothedMinValue, KINDA_SMALL_NUMBER);

		TArray<FVector2D> Points;
		Points.Reserve(Samples.Num());
		for (int32 Index = 0; Index < Samples.Num(); ++Index)
		{
			const float X = (LocalSize.X * Index) / (float)(Samples.Num() - 1);
			const float NormalizedY = FMath::Clamp((Samples[Index] - SmoothedMinValue) / Range, 0.0f, 1.0f);
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
