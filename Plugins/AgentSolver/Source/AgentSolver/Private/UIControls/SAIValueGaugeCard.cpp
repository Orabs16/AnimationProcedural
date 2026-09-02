#include "UIControls/SAIValueGaugeCard.h"

#include "Rendering/DrawElements.h"
#include "Styling/CoreStyle.h"
#include "Brushes/SlateColorBrush.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/SBoxPanel.h"

namespace
{
	/** Plain white 1x1 brush, tinted per-draw-call -- same trick as SAgentSolverLineGraph's use of MakeLines with an explicit color, just for filled boxes instead of polylines. One shared static instance; the brush itself carries no state. */
	const FSlateBrush* GetFillBrush()
	{
		static FSlateColorBrush Brush(FLinearColor::White);
		return &Brush;
	}
}

// ================================ SAIBipolarGaugeBar ================================

void SAIBipolarGaugeBar::Construct(const FArguments& InArgs)
{
	Value = FMath::Clamp(InArgs._Value, -1.0f, 1.0f);
}

void SAIBipolarGaugeBar::SetValue(float InValue)
{
	const float Clamped = FMath::Clamp(InValue, -1.0f, 1.0f);
	if (Clamped != Value)
	{
		Value = Clamped;
		Invalidate(EInvalidateWidgetReason::Paint);
	}
}

FVector2D SAIBipolarGaugeBar::ComputeDesiredSize(float) const
{
	return FVector2D(100.0f, 14.0f);
}

int32 SAIBipolarGaugeBar::OnPaint(const FPaintArgs& Args, const FGeometry& AllottedGeometry, const FSlateRect& MyCullingRect, FSlateWindowElementList& OutDrawElements, int32 LayerId, const FWidgetStyle& InWidgetStyle, bool bParentEnabled) const
{
	const FVector2D LocalSize = AllottedGeometry.GetLocalSize();
	const FSlateBrush* FillBrush = GetFillBrush();

	// Background track.
	FSlateDrawElement::MakeBox(
		OutDrawElements, LayerId, AllottedGeometry.ToPaintGeometry(),
		FillBrush, ESlateDrawEffect::None, FLinearColor(0.03f, 0.03f, 0.03f, 1.0f));

	const float MidX = LocalSize.X * 0.5f;
	const float HalfWidth = MidX;
	const float FillWidth = FMath::Abs(Value) * HalfWidth;

	if (FillWidth > KINDA_SMALL_NUMBER)
	{
		const float FillX = (Value >= 0.0f) ? MidX : (MidX - FillWidth);
		// Positive: green/blue toward the right. Negative: red/orange toward the left.
		const FLinearColor FillColor = (Value >= 0.0f)
			? FLinearColor(0.20f, 0.70f, 0.90f, 1.0f)
			: FLinearColor(0.90f, 0.35f, 0.20f, 1.0f);

		FSlateDrawElement::MakeBox(
			OutDrawElements, LayerId + 1,
			AllottedGeometry.ToPaintGeometry(FVector2D(FillWidth, LocalSize.Y), FSlateLayoutTransform(FVector2D(FillX, 0.0f))),
			FillBrush, ESlateDrawEffect::None, FillColor);
	}

	// Neutral (0.0) center marker, always drawn so the bar reads correctly even at exactly 0.
	TArray<FVector2D> CenterLine;
	CenterLine.Add(FVector2D(MidX, 0.0f));
	CenterLine.Add(FVector2D(MidX, LocalSize.Y));
	FSlateDrawElement::MakeLines(
		OutDrawElements, LayerId + 2, AllottedGeometry.ToPaintGeometry(), CenterLine,
		ESlateDrawEffect::None, FLinearColor(1.0f, 1.0f, 1.0f, 0.5f), true, 1.0f);

	return LayerId + 2;
}

// ================================ SAIValueGaugeCard ================================

void SAIValueGaugeCard::Construct(const FArguments& InArgs, FName InVariableName)
{
	VariableName = InVariableName;

	ChildSlot
	[
		SNew(SBox)
		.WidthOverride(112.0f)
		[
			SNew(SBorder)
			.BorderImage(FCoreStyle::Get().GetBrush("ToolPanel.GroupBorder"))
			.Padding(FMargin(4.0f))
			[
				SNew(SVerticalBox)
				+ SVerticalBox::Slot()
				.AutoHeight()
				.Padding(0.0f, 0.0f, 0.0f, 2.0f)
				[
					SNew(STextBlock)
					.Text(FText::FromName(VariableName))
					.Justification(ETextJustify::Center)
					.AutoWrapText(true)
					.Font(FCoreStyle::Get().GetFontStyle("SmallFont"))
				]
				+ SVerticalBox::Slot()
				.AutoHeight()
				.Padding(0.0f, 0.0f, 0.0f, 2.0f)
				[
					SAssignNew(GaugeBar, SAIBipolarGaugeBar)
				]
				+ SVerticalBox::Slot()
				.AutoHeight()
				[
					SAssignNew(ValueText, STextBlock)
					.Text(FText::FromString(TEXT("0.00")))
					.Justification(ETextJustify::Center)
					.Font(FCoreStyle::Get().GetFontStyle("SmallFont"))
				]
			]
		]
	];
}

void SAIValueGaugeCard::SetValue(float NormalizedValue)
{
	const float Clamped = FMath::Clamp(NormalizedValue, -1.0f, 1.0f);
	if (GaugeBar)
	{
		GaugeBar->SetValue(Clamped);
	}
	if (ValueText)
	{
		ValueText->SetText(FText::FromString(FString::Printf(TEXT("%.2f"), Clamped)));
	}
}
