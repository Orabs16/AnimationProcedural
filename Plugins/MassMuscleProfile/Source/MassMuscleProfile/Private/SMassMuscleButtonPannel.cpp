#include "SMassMuscleButtonPannel.h"
#include "FMassMuscleEditorModel.h"
#include "FMassMuscleEditorSettings.h"

#include "FMassMuscleData.h"
#include "Templates/SharedPointer.h"
#include "Types/SlateEnums.h"
#include "UObject/NameTypes.h"
#include "Widgets/DeclarativeSyntaxSupport.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/SBoxPanel.h"
#include "FMassMuscleStyle.h"
#include "Widgets/SOverlay.h"
#include "Widgets/Images/SImage.h"
#include <functional>

void SMassMuscleButtonPanel::Construct(const FArguments& InArgs)
{
    Settings = InArgs._Settings;
    Model = InArgs._Model;
    HierarchyWidget = InArgs._HierarchyWidget;
    ChildSlot
    .VAlign(VAlign_Center)
    [
        SNew(SHorizontalBox)
        +SHorizontalBox::Slot()
        .AutoWidth()
        .Padding(30,0,10,0)
        [
            SNew(SButton)
            .ContentPadding(FMargin(4))
            .ToolTipText(NSLOCTEXT("MassMuscleProfile", "SaveTooltip", "Save"))
            .OnClicked(this, &SMassMuscleButtonPanel::OnSaveClicked)
            [
                SNew(SImage)
                .Image(FAppStyle::GetBrush("Icons.Save"))
                .ColorAndOpacity(FSlateColor(FLinearColor(FColor(201, 201, 201, 255))))
            ]
        ]
        + SHorizontalBox::Slot()
        .AutoWidth()
        .Padding(10,0,10,0)
        [
            SNew(SBorder)
            .VAlign(VAlign_Center)
            .HAlign(HAlign_Center)
            .BorderImage(FMassMuscleStyle::Get().GetBrush("DarkRoundBrush"))
            .Padding(1)
            [
                SNew(SHorizontalBox)
                +SHorizontalBox::Slot()
                .VAlign(VAlign_Fill)
                .HAlign(HAlign_Fill)
                .AutoWidth()
                [
                    SAssignNew(extend,SButton)
                    .ButtonStyle(FAppStyle::Get(), "NoBorder")
                    .OnClicked(this, &SMassMuscleButtonPanel::OnExtend)
                    .ContentPadding(0)
                    [
                        SNew(SBorder)
                        .BorderImage_Lambda([this]() 
                        {
                            return extend->IsHovered()? 
                            FMassMuscleStyle::Get().GetBrush("LeftLightRoundBrush") :
                            !IsHierarchyCollapsed?
                                FMassMuscleStyle::Get().GetBrush("LeftDarkRoundBrush") :
                                FMassMuscleStyle::Get().GetBrush("LeftRoundBrush");
                        })
                        [
                            SNew(SBox)
                            .HAlign(HAlign_Center)
                            .VAlign(VAlign_Center)
                            .Padding(5)
                            [
                                SNew(STextBlock)
                                .Text(FText::FromString("Extend"))
                                .ColorAndOpacity(FMassMuscleStyle::UnrealText)

                            ]
                        ]
                    ]
                ]
                +SHorizontalBox::Slot()
                .VAlign(VAlign_Fill)
                .HAlign(HAlign_Fill)
                .AutoWidth()
                [
                    SAssignNew(collapse,SButton)
                    .ButtonStyle(FAppStyle::Get(), "NoBorder")
                    .OnClicked(this, &SMassMuscleButtonPanel::OnCollapse)
                    .ContentPadding(0)
                    [
                        SNew(SBorder)
                        .BorderImage_Lambda([this]() 
                        {
                            return collapse->IsHovered()? 
                            FMassMuscleStyle::Get().GetBrush("RightLightRoundBrush") :
                            IsHierarchyCollapsed?
                                FMassMuscleStyle::Get().GetBrush("RightDarkRoundBrush") :
                                FMassMuscleStyle::Get().GetBrush("RightRoundBrush");
                        })
                        [
                            SNew(SBox)
                            .HAlign(HAlign_Center)
                            .VAlign(VAlign_Center)
                            .Padding(5)
                            [
                                SNew(STextBlock)
                                .Text(FText::FromString("Collapse"))
                                .ColorAndOpacity(FMassMuscleStyle::UnrealText)

                            ]
                        ]
                    ]
                ]
            ]
        ]
        + SHorizontalBox::Slot()
        .AutoWidth()
        .VAlign(VAlign_Center)
        .Padding(10,0,10,0)
        [
            SNew(SButton)
            .HAlign(HAlign_Center)
            .Text(FText::FromString("XRay Skeleton"))
            .OnClicked(this, &SMassMuscleButtonPanel::OnXRaySkeleton)
        ]
        
        + SHorizontalBox::Slot()
        .AutoWidth()
        .VAlign(VAlign_Center)
        .Padding(10,0,10,0)
        [
            SNew(SButton)
            .HAlign(HAlign_Center)
            .Text(FText::FromString("XRay Muscles"))
            .OnClicked(this, &SMassMuscleButtonPanel::OnXRayMuscles)
        ]
        
        + SHorizontalBox::Slot()
        .AutoWidth()
        .VAlign(VAlign_Center)
        .Padding(10,0,10,0)
        [
            SNew(SButton)
            .HAlign(HAlign_Center)
            .Text(FText::FromString("New Muscle"))
            .OnClicked(this, &SMassMuscleButtonPanel::OnNewMuscle)
        ]
        + SHorizontalBox::Slot()
        .AutoWidth()
        .VAlign(VAlign_Center)
        .Padding(10,0,10,0)
        [
            SNew(SButton)
            .HAlign(HAlign_Center)
            .Text(FText::FromString("Delete Muscles"))
            .OnClicked(this, &SMassMuscleButtonPanel::OnDeleteMuscles)
        ]
        + SHorizontalBox::Slot()
        .AutoWidth()
        .VAlign(VAlign_Center)
        .Padding(10,0,10,0)
        [
            SNew(SButton)
            .HAlign(HAlign_Center)
            .Text(FText::FromString("Mirror Muscles"))
            .OnClicked(this, &SMassMuscleButtonPanel::OnMirrorMuscles)
        ]
    ];
}

FReply SMassMuscleButtonPanel::OnXRaySkeleton()
{
    // Handle button click logic here
    Settings->XRaySkeleton = !Settings->XRaySkeleton;
    return FReply::Handled();
}
FReply SMassMuscleButtonPanel::OnXRayMuscles()
{
    Settings->XRayMuscles = !Settings->XRayMuscles;
    return FReply::Handled();
}

FReply SMassMuscleButtonPanel::OnExtend()
{
    IsHierarchyCollapsed = true;
    Settings->SetHierarchy(true);
    return FReply::Handled();
}

FReply SMassMuscleButtonPanel::OnCollapse()
{
    IsHierarchyCollapsed = false;
    Settings->SetHierarchy(false);
    return FReply::Handled(); 
}

FReply SMassMuscleButtonPanel::OnNewMuscle()
{
    if(!Model) return FReply::Handled();
    Model->AddMuscle();
    return FReply::Handled();
}

FReply SMassMuscleButtonPanel::OnDeleteMuscles()
{
    if(!Model) return FReply::Handled();
    if(!HierarchyWidget) return FReply::Handled();
    TArray<TSharedPtr<FMassMuscleTreeItem>> SelectedItems = HierarchyWidget->GetSelectedItems();
    if(SelectedItems.Num() == 0) return FReply::Handled();
    Model->DeleteMuscles(SelectedItems, std::bind(&SMassMuscleHierarchy::OnDeleteMuscle, HierarchyWidget.Get(), std::placeholders::_1));
    return FReply::Handled();
}
FReply SMassMuscleButtonPanel::OnMirrorMuscles()
{
    if(!Model) return FReply::Handled();
    if(!HierarchyWidget) return FReply::Handled();
    TArray<TSharedPtr<FMassMuscleTreeItem>> SelectedItems = HierarchyWidget->GetSelectedItems();
    if(SelectedItems.Num() == 0) return FReply::Handled();
    Model->MirrorMuscles(SelectedItems, std::bind(&SMassMuscleHierarchy::OnAddMuscle, HierarchyWidget.Get(), std::placeholders::_1));
    return FReply::Handled();
}

FReply SMassMuscleButtonPanel::OnSaveClicked()
{
    if(!Model) return FReply::Handled();
    Model->SaveProfile();
    return FReply::Handled();
}