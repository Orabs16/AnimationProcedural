#include "SMassMuscleExpandableSection.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Images/SImage.h"
#include "Widgets/Text/STextBlock.h"
#include "Styling/AppStyle.h"
#include "FMassMuscleStyle.h"

void SMassMuscleExpandableSection::Construct(const FArguments& InArgs)
{
    bIsExpanded = InArgs._InitiallyExpanded;

    
    for (int32 i = 0; i < InArgs._Slots.Num(); ++i)
    {
        ContentSlots.AddSlot(
            MoveTemp(const_cast<FContentSlot::FSlotArguments&>(InArgs._Slots[i])));
    }
    TSharedRef<SVerticalBox> ContentBox = SNew(SVerticalBox);

    for (int32 i = 0; i < ContentSlots.Num(); ++i)
    {
        ContentBox->AddSlot()
        .AutoHeight()
        .Padding(FMargin(0,1,0,(i+1) < ContentSlots.Num()? 0 : 1))
        [
            ContentSlots[i].GetWidget()
        ];
    }

    ChildSlot
    [
        SNew(SVerticalBox)

        // ── Header ────────────────────────────────────────────────────────
        + SVerticalBox::Slot()
        .AutoHeight()
        [
            SNew(SBorder)
            .BorderImage(FAppStyle::GetBrush("DetailsView.CategoryTop"))
            .Padding(FMargin(6.f, 2.f))
            [
                SNew(SButton)
                .ButtonStyle(FAppStyle::Get(), "NoBorder")
                .OnClicked(this, &SMassMuscleExpandableSection::OnHeaderClicked)
                [
                    SNew(SHorizontalBox)

                    // arrow icon
                    + SHorizontalBox::Slot()
                    .AutoWidth()
                    .VAlign(VAlign_Center)
                    .Padding(FMargin(2.f, 0.f, 6.f, 0.f))
                    [
                        SNew(SImage)
                        .Image(this, &SMassMuscleExpandableSection::GetArrowBrush)
                        .ColorAndOpacity(FSlateColor::UseSubduedForeground())
                    ]

                    // title
                    + SHorizontalBox::Slot()
                    .VAlign(VAlign_Center)
                    .FillWidth(1.f)
                    [
                        SNew(STextBlock)
                        .Text(InArgs._Title)
                        .Font(FAppStyle::GetFontStyle(
                            "DetailsView.CategoryFontStyle"))
                    ]
                ]
            ]
        ]

        // ── Content ───────────────────────────────────────────────────────
        + SVerticalBox::Slot()
        .AutoHeight()
        [
            SNew(SBorder)
            .Visibility(this, &SMassMuscleExpandableSection::GetContentVisibility)
            .BorderImage(FMassMuscleStyle::Get().GetBrush("MassMuscle.DarkBackGround"))
            .Padding(FMargin(.2f, .2f))
            [
                ContentBox
            ]
        ]
    ];
}

FReply SMassMuscleExpandableSection::OnHeaderClicked()
{
    bIsExpanded = !bIsExpanded;
    return FReply::Handled();
}

EVisibility SMassMuscleExpandableSection::GetContentVisibility() const
{
    return bIsExpanded ? EVisibility::Visible : EVisibility::Collapsed;
}

const FSlateBrush* SMassMuscleExpandableSection::GetArrowBrush() const
{
    return FAppStyle::GetBrush(
        bIsExpanded ? "TreeArrow_Expanded" : "TreeArrow_Collapsed");
}