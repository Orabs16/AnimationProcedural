#include "SMassMuscleMainWidget.h"

#include "Brushes/SlateColorBrush.h"
#include "Engine/SkeletalMesh.h"
#include "Layout/Visibility.h"
#include "Math/Color.h"
#include "PropertyCustomizationHelpers.h"
#include "PropertyEditorModule.h"
#include "SMassMuscleHierarchy.h"
#include "SMassMuscleViewport.h"
#include "Styling/SlateColor.h"
#include "ThumbnailRendering/ThumbnailManager.h"
#include "Widgets/DeclarativeSyntaxSupport.h"
#include "Widgets/Input/SSearchBox.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Layout/SSplitter.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/SOverlay.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Views/STreeView.h"
#include "Templates/SharedPointer.h"
#include "Widgets/Layout/SWidgetSwitcher.h"
#include "Widgets/Input/SCheckBox.h"
#include "Widgets/Input/SButton.h"
#include "Styling/AppStyle.h"
#include "Styling/CoreStyle.h"
#include "FMassMuscleStyle.h"

#include "FMassMuscleViewportClient.h"
#include "FMassMuscleEditorModel.h"
#include "SMassMuscleViewport.h"
#include "SMassMuscleButtonPannel.h"
#include "SMassMuscleHierarchy.h"
#include "SMassMuscleDetailsPannel.h"
#include "SMassMuscleSettingsPannel.h"

void SMassMuscleMainWidget::Construct(const FArguments& InArgs)
{
	Model = InArgs._Model;
    Settings = MakeShared<FMassMuscleEditorSettings>();

	SAssignNew(ViewportWidget, SMassMuscleViewport).Model(Model).Settings(Settings);
	SAssignNew(HierarchyWidget, SMassMuscleHierarchy).Model(Model).Settings(Settings);
	SAssignNew(ButtonPanelWidget, SMassMuscleButtonPanel).Model(Model).Settings(Settings).HierarchyWidget(HierarchyWidget);
	SAssignNew(DetailsPanelWidget, SMassMuscleDetailsPannel).Model(Model).Hierarchy(HierarchyWidget);
	SAssignNew(SettingsPanelWidget, SMassMuscleSettingsPannel).Model(Model);

	// clang-format off
	ChildSlot
	[
        SNew(SVerticalBox)
            +SVerticalBox::Slot()
            .AutoHeight()
            [
                SNew(SBox)
                .HeightOverride(40)
                [
                    ButtonPanelWidget.ToSharedRef()
                ]
            ]
            +SVerticalBox::Slot().FillHeight(0.8)
            [
                SNew(SBorder)
                .Padding(0, 10)
                .BorderImage(FMassMuscleStyle::Get().GetBrush("MassMuscle.DarkBackGround"))
                [
                    SNew(SSplitter)
	                + SSplitter::Slot().Value(0.2f)
                    [
                        SNew(SVerticalBox) 
	                        + SVerticalBox::Slot().FillHeight(0.6f).Padding(4.f)[SNew(SBorder)[HierarchyWidget.ToSharedRef()]]
                    ]
	                + SSplitter::Slot().Value(0.6f)[ViewportWidget.ToSharedRef()]
	                + SSplitter::Slot().Value(0.2f)
                    [
                        SNew(SBorder)
                        .BorderImage(FMassMuscleStyle::Get().GetBrush("MassMuscle.DarkBackGround"))
                        .Padding(2)
                        [
                            SNew(SVerticalBox)


                            // ── Tab header strip ─────────────────────────────────────────────
                            + SVerticalBox::Slot()
                            .AutoHeight()
                            [
                                SNew(SHorizontalBox)
                                // Details tab
                                + SHorizontalBox::Slot()
                                .FillWidth(1.f)
                                [
                                    SNew(SButton)
                                    //.ButtonColorAndOpacity(FLinearColor(FColor(21,21,21,0)))
                                    .ButtonStyle(FAppStyle::Get(), "NoBorder")
                                    .OnClicked(this, &SMassMuscleMainWidget::OnTabClicked, 0)
                                    .ContentPadding(0.f)
                                    [
                                        SNew(SBorder)
                                        .BorderImage_Lambda([this]()
                                        {
                                            return ActiveTabIndex == 0
                                                ? FMassMuscleStyle::Get().GetBrush("MassMuscle.Tab.Active")
                                                : FMassMuscleStyle::Get().GetBrush("MassMuscle.Tab.Inactive");
                                        })
                                        //.ColorAndOpacity(FLinearColor(FColor(21,21,21,255)))
                                        .Padding(FMargin(0.f, 0.f, 0.f, 2.f))
                                        [
                                            SNew(SBox)
                                            .HAlign(HAlign_Center)
                                            .VAlign(VAlign_Center)
                                            .HeightOverride(28.f)
                                            [
                                                SNew(STextBlock)
                                                .Text(FText::FromString("Details"))
                                                //.ColorAndOpacity(FSlateColor(FMassMuscleStyle::UnrealText))
                                            ]
                                        ]
                                    ]
                                ]
                                // Settings tab
                                + SHorizontalBox::Slot()
                                .FillWidth(1.f)
                                [
                                    SNew(SButton)
                                    .ButtonStyle(FAppStyle::Get(), "NoBorder")
                                    .OnClicked(this, &SMassMuscleMainWidget::OnTabClicked, 1)
                                    .ContentPadding(0.f)
                                    [
                                        SNew(SBorder)
                                        .BorderImage_Lambda([this]()
                                        {
                                            return ActiveTabIndex == 1
                                                ? FMassMuscleStyle::Get().GetBrush("MassMuscle.Tab.Active")
                                                : FMassMuscleStyle::Get().GetBrush("MassMuscle.Tab.Inactive");
                                        })
                                        .Padding(FMargin(0.f, 0.f, 0.f, 2.f))
                                        [
                                            SNew(SBox)
                                            .HAlign(HAlign_Center)
                                            .VAlign(VAlign_Center)
                                            .HeightOverride(28.f)
                                            [
                                                SNew(STextBlock)
                                                .Text(FText::FromString("Settings"))
                                                .ColorAndOpacity(FSlateColor(FMassMuscleStyle::UnrealText))
                                            ]
                                        ]
                                    ]
                                ]
                            
                            ]
                            // ── Tab content ──────────────────────────────────────────────────
                            + SVerticalBox::Slot()
                            .FillHeight(1.f)
                            [
                                SAssignNew(RightPanelSwitcher, SWidgetSwitcher)
                            
                                + SWidgetSwitcher::Slot()   // index 0 — Details
                                [
                                    DetailsPanelWidget.ToSharedRef()
                                ]
                            
                                + SWidgetSwitcher::Slot()   // index 1 — Settings
                                [
                                    SettingsPanelWidget.ToSharedRef()
                                ]
                            ]
                        ]
                    ]
                ]
            ]
    ];      
    //clang-format on
}       

FReply SMassMuscleMainWidget::OnTabClicked(int32 TabIndex)
{               
    ActiveTabIndex = TabIndex;
    RightPanelSwitcher->SetActiveWidgetIndex(TabIndex);
    return FReply::Handled();
}