#include "SMassMuscleItemRow.h"
#include "Brushes/SlateColorBrush.h"
#include "Layout/Margin.h"
#include "Widgets/DeclarativeSyntaxSupport.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Images/SImage.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/SOverlay.h"
#include "Widgets/Text/STextBlock.h"
#include "Styling/AppStyle.h"
#include "FMassMuscleStyle.h"

void SMassMuscleItemRow::Construct(const FArguments& InArgs)
{
    for (int32 i = 0; i < InArgs._Slots.Num(); ++i)
    {
        ContentSlots.AddSlot(
            MoveTemp(const_cast<FContentSlot::FSlotArguments&>(InArgs._Slots[i])));
    }
    TSharedRef<SHorizontalBox> ContentBox = SNew(SHorizontalBox);

    for (int32 i = 0; i < ContentSlots.Num(); ++i)
    {
        ContentBox->AddSlot()
        .FillWidth(1.f)
        .Padding(FMargin(0,0,(i+1) < ContentSlots.Num()? 1 : 0,0))
        [
            SNew(SBorder)
            .VAlign(VAlign_Center)
            .BorderImage_Lambda([this]()
            {
                return IsHovered() ? FMassMuscleStyle::Get().GetBrush("MassMuscle.LightBackGround") : FMassMuscleStyle::Get().GetBrush("MassMuscle.BackGround");
            })
            [
                ContentSlots[i].GetWidget()
            ]
        ];
    }


    ChildSlot
    [
        SNew(SBorder)
        .BorderBackgroundColor(FMassMuscleStyle::UnrealDarkBackGround)
        .Padding(0)
        [
            ContentBox
        ]
        
    ];
}