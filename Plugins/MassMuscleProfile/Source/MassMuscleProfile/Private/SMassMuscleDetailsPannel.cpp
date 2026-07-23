#include "SMassMuscleDetailsPannel.h"
#include "CoreGlobals.h"
#include "FMassMuscleTreeItem.h"
#include "Logging/LogMacros.h"
#include "Logging/LogVerbosity.h"
#include "Math/MathFwd.h"
#include "Misc/CoreMiscDefines.h"
#include "Misc/Optional.h"
#include "SlateFwd.h"
#include "Templates/SharedPointer.h"
#include "UMassMuscleProfileAsset.h"
#include "Widgets/DeclarativeSyntaxSupport.h"
#include "Widgets/Input/SComboBox.h"
#include "Widgets/Input/SNumericEntryBox.h"
#include "Widgets/Input/SVectorInputBox.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Text/STextBlock.h"
#include "SMassMuscleExpandableSection.h"
#include "SMassMuscleItemRow.h"
#include "FMassMuscleEditorModel.h"
#include "Widgets/Layout/SWidgetSwitcher.h"
#include "FMassMuscleData.h"
#include "Widgets/Input/SEditableTextBox.h"


FVector MyVector = FVector::ForwardVector;
void SMassMuscleDetailsPannel::Construct(const FArguments& InArgs)
{
    Model = InArgs._Model;
    Hierarchy = InArgs._Hierarchy;
	Model->OnSelectionChanged.AddSP(this, &SMassMuscleDetailsPannel::OnSelectionChanged);
    SAssignNew(MuscleComboBox, SComboBox<TSharedPtr<FString>>)
    .OptionsSource(&MuscleChildOptions)
    .OnGenerateWidget_Lambda([](TSharedPtr<FString> InOption)
    {
        return SNew(STextBlock).Text(FText::FromString(*InOption));
    })
    .OnSelectionChanged(this, &SMassMuscleDetailsPannel::OnChildBoneChanged)
    .InitiallySelectedItem(MuscleChildCurrentSelection)
    [
        SNew(STextBlock)
        .Text_Lambda([this]()
        {
            return FText::FromString(*MuscleChildCurrentSelection);
        })
    ];

    ChildSlot[
        SAssignNew(PanelSwitcher, SWidgetSwitcher)
        + SWidgetSwitcher::Slot() 
        [
            SNew(SMassMuscleExpandableSection)
            .Title(FText::FromString("Bone Info"))
            .InitiallyExpanded(true)
            +SMassMuscleExpandableSection::Slot()
            [
                SNew(SMassMuscleItemRow)
                +SMassMuscleItemRow::Slot()
                [
                     SNew(STextBlock).Text(FText::FromString("Bone name"))
                ]
                +SMassMuscleItemRow::Slot()
                [
                     SNew(STextBlock).Text_Lambda([this] () {return FText::FromName(SelectedBone? SelectedBone->BoneName : FName());})
                ]
            ]
            +SMassMuscleExpandableSection::Slot()
            [
                SNew(SMassMuscleItemRow)
                +SMassMuscleItemRow::Slot()
                [
                     SNew(STextBlock).Text(FText::FromString("Bone mass"))
                ]
                +SMassMuscleItemRow::Slot()
                [
                     SNew(SNumericEntryBox<float>)
                    .Value(this, &SMassMuscleDetailsPannel::GetBoneMass)
                    .OnValueChanged(this, &SMassMuscleDetailsPannel::OnMassChanged)
                    .AllowSpin(true) 
                    .MinValue(0.0f)
                    .MaxValue(TOptional<float>())
                    .MaxSliderValue(TOptional<float>())
                ]
            ]
        ]
        + SWidgetSwitcher::Slot() 
        [
            SNew(SMassMuscleExpandableSection)
            .Title(FText::FromString("Muscle Info"))
            .InitiallyExpanded(true)
            +SMassMuscleExpandableSection::Slot()
            [
                SNew(SMassMuscleItemRow)
                +SMassMuscleItemRow::Slot()
                [
                     SNew(STextBlock).Text(FText::FromString("Muscle Name"))
                ]
                +SMassMuscleItemRow::Slot()
                [
                    SNew(SEditableTextBox)
                    .Text(this, &SMassMuscleDetailsPannel::GetMuscleName)
                    .OnTextCommitted(this, &SMassMuscleDetailsPannel::CommitNewName)
                    .ClearKeyboardFocusOnCommit(false)
                    .IsReadOnly(false)
                ]
            ]
            +SMassMuscleExpandableSection::Slot()
            [
                SNew(SMassMuscleItemRow)
                +SMassMuscleItemRow::Slot()
                [
                     SNew(STextBlock).Text(FText::FromString("Attached to bone"))
                ]
                +SMassMuscleItemRow::Slot()
                [
                    SNew(STextBlock).Text_Lambda([this] () {return FText::FromName(SelectedMuscle->BoneName);})
                ]
            ]
            +SMassMuscleExpandableSection::Slot()
            [
                SNew(SMassMuscleItemRow)
                +SMassMuscleItemRow::Slot()
                [
                     SNew(STextBlock).Text(FText::FromString("Linked to bone"))
                ]
                +SMassMuscleItemRow::Slot()
                [
                    MuscleComboBox.ToSharedRef()
                ]
            ]
            +SMassMuscleExpandableSection::Slot()
            [
                SNew(SMassMuscleItemRow)
                +SMassMuscleItemRow::Slot()
                [
                     SNew(STextBlock).Text(FText::FromString("Strength"))
                ]
                +SMassMuscleItemRow::Slot()
                [
                    SNew(SNumericEntryBox<float>)
                    .Value(this, &SMassMuscleDetailsPannel::GetMuscleStrength)
                    .OnValueChanged(this, &SMassMuscleDetailsPannel::OnStrengthChanged)
                    .AllowSpin(true) 
                    .MinValue(0.0f)
                    .MaxValue(TOptional<float>())        // explicitly no max
                    .MaxSliderValue(TOptional<float>())  // explicitly no slider max
                ]
            ]
            +SMassMuscleExpandableSection::Slot()
            [
                SNew(SMassMuscleItemRow)
                +SMassMuscleItemRow::Slot()
                [
                     SNew(STextBlock).Text(FText::FromString("Min range"))
                ]
                +SMassMuscleItemRow::Slot()
                [
                    SNew(SNumericEntryBox<float>)
                    .Value(this, &SMassMuscleDetailsPannel::GetMinRange)
                    .OnValueChanged(this, &SMassMuscleDetailsPannel::OnMinRangeChanged)
                    .AllowSpin(true) 
                    .Delta(0.1f)          
                    .LinearDeltaSensitivity(1)
                    .MinValue(TOptional<float>())        // explicitly no min
                    .MaxValue(TOptional<float>())        // explicitly no max
                    .MinSliderValue(TOptional<float>())  // explicitly no slider min
                    .MaxSliderValue(TOptional<float>())  // explicitly no slider max
                ]
            ]
            +SMassMuscleExpandableSection::Slot()
            [
                SNew(SMassMuscleItemRow)
                +SMassMuscleItemRow::Slot()
                [
                     SNew(STextBlock).Text(FText::FromString("Max range"))
                ]
                +SMassMuscleItemRow::Slot()
                [
                    SNew(SNumericEntryBox<float>)
                    .Value(this, &SMassMuscleDetailsPannel::GetMaxRange)
                    .OnValueChanged(this, &SMassMuscleDetailsPannel::OnMaxRangeChanged)
                    .AllowSpin(true) 
                    .Delta(0.1f)          
                    .LinearDeltaSensitivity(1)
                    .MinValue(TOptional<float>())        // explicitly no min
                    .MaxValue(TOptional<float>())        // explicitly no max
                    .MinSliderValue(TOptional<float>())  // explicitly no slider min
                    .MaxSliderValue(TOptional<float>())  // explicitly no slider max
                ]
            ]
            +SMassMuscleExpandableSection::Slot()
            [
                SNew(SMassMuscleItemRow)
                +SMassMuscleItemRow::Slot()
                [
                     SNew(STextBlock).Text(FText::FromString("Orientation"))
                ]
                +SMassMuscleItemRow::Slot()
                [
                    SNew(SComboBox<TSharedPtr<FString>>)
                    .OptionsSource(&RotationOptions).OnGenerateWidget_Lambda([](TSharedPtr<FString> InOption)
                    {
                        return SNew(STextBlock).Text(FText::FromString(*InOption)); 
                    })
                    .OnSelectionChanged_Lambda([this](TSharedPtr<FString> NewSelection, ESelectInfo::Type SelectInfo)
                    {
                        OnOrientationChanged(NewSelection, SelectInfo);
                    })
                    .InitiallySelectedItem(MuscleRotationCurrentSelection)
                    [
                        SNew(STextBlock)
                        .Text_Lambda([this]()
                        {
                            return FText::FromString(*MuscleRotationCurrentSelection);
                        })
                    ]
                ]
            ]
        ]
        
    ];
}

void SMassMuscleDetailsPannel::OnSelectionChanged(FName NewSelectionName, EItemType Type)
{
    PanelSwitcher->SetActiveWidgetIndex(int32(Type));
    if(Type == EItemType::Muscle) GetMuscleInfo(NewSelectionName);
    if(Type == EItemType::Bone) GetBoneInfo(NewSelectionName);
    //else GetBoneInfo
}

void SMassMuscleDetailsPannel::GetMuscleInfo(FName MuscleName)
{
    auto RotationTypeToString = [](ERotationType rotation){
        switch(rotation){
            case ERotationType::Yaw:
                return "Yaw";
            case ERotationType::Roll:
                return "Roll";
            case ERotationType::Pitch:
                return "Pitch";
            default:
                return "Pitch";
        }
    };
    UMassMuscleProfileAssetMuscle* profile =  Model->GetMuscleProfile();
    int32 index = profile->FindMuscleByName(MuscleName);
    if(index == INDEX_NONE) return;
    SelectedMuscle = &profile->Muscles[index];
    TArray<FName> ChildBones;
    profile->GetChildBonesNames(SelectedMuscle->BoneName, ChildBones);
    MuscleChildOptions.Empty();
    for(FName ChildBone : ChildBones)
    {
        MuscleChildOptions.Add(MakeShared<FString>(ChildBone.ToString()));
    }
    if(ChildBones.Num() == 0)
    {
        MuscleChildOptions.Add(MakeShared<FString>(TEXT("None")));
    }
    MuscleChildCurrentSelection = SelectedMuscle->ChildBoneName != NAME_None ? MakeShared<FString>(SelectedMuscle->ChildBoneName.ToString()) : MakeShared<FString>(TEXT("None"));
    MuscleRotationCurrentSelection = MakeShared<FString>(RotationTypeToString(SelectedMuscle->Orientation));
    UE_LOG(LogTemp, Warning, TEXT("Rotation changed to : %hs"), RotationTypeToString(SelectedMuscle->Orientation))
}

void SMassMuscleDetailsPannel::GetBoneInfo(FName boneName)
{
    UMassMuscleProfileAssetMass* profile = Model->GetMassProfile();
    int32 index = profile->FindBoneByName(boneName);
    if(index == INDEX_NONE) return;
    SelectedBone = &profile->Mass[index];
}


void SMassMuscleDetailsPannel::MarkDirtyMuscle() const
{
    UMassMuscleProfileAssetMuscle* profile = Model->GetMuscleProfile();
    if(!profile)
    {
        UE_LOG(LogTemp, Warning, TEXT("Couldn't find the attached profile"));
        return;
    } 
    profile->NotifyMuscleChange();
}
void SMassMuscleDetailsPannel::MarkDirtyMass() const
{
    UMassMuscleProfileAssetMass* profile = Model->GetMassProfile();
    if(!profile)
    {
        UE_LOG(LogTemp, Warning, TEXT("Couldn't find the attached profile"));
        return;
    } 
    profile->NotifyMassChange();
}

