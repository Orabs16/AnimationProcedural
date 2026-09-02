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
#include "Widgets/Input/SCheckBox.h"
#include "Widgets/Input/SNumericEntryBox.h"
#include "Widgets/Input/SVectorInputBox.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Text/STextBlock.h"
#include "SMassMuscleExpandableSection.h"
#include "SMassMuscleItemRow.h"
#include "FMassMuscleEditorModel.h"
#include "IDetailsView.h"
#include "MassMuscleCurveEditorProxy.h"
#include "Modules/ModuleManager.h"
#include "PropertyEditorModule.h"
#include "Widgets/Layout/SWidgetSwitcher.h"
#include "FMassMuscleData.h"
#include "Widgets/Input/SEditableTextBox.h"

void SMassMuscleDetailsPannel::Construct(const FArguments& InArgs)
{
    Model = InArgs._Model;
    Hierarchy = InArgs._Hierarchy;
	Model->OnSelectionChanged.AddSP(this, &SMassMuscleDetailsPannel::OnSelectionChanged);
	Model->OnMuscleDataChanged.AddSP(this, &SMassMuscleDetailsPannel::OnMuscleDataChanged);

    CurveProxy.Reset(NewObject<UMassMuscleCurveEditorProxy>(GetTransientPackage()));

    FDetailsViewArgs DetailsViewArgs;
    DetailsViewArgs.bAllowSearch = false;
    DetailsViewArgs.bHideSelectionTip = true;
    DetailsViewArgs.bLockable = false;
    DetailsViewArgs.bShowOptions = false;
    DetailsViewArgs.NameAreaSettings = FDetailsViewArgs::HideNameArea;

    FPropertyEditorModule& PropertyEditorModule = FModuleManager::LoadModuleChecked<FPropertyEditorModule>("PropertyEditor");
    CurveDetailsView = PropertyEditorModule.CreateDetailView(DetailsViewArgs);
    CurveDetailsView->SetObject(CurveProxy.Get());
    CurveDetailsView->OnFinishedChangingProperties().AddSP(this, &SMassMuscleDetailsPannel::OnCurveDetailsChanged);

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
            +SMassMuscleExpandableSection::Slot()
            [
                SNew(SMassMuscleItemRow)
                +SMassMuscleItemRow::Slot()
                [
                     SNew(STextBlock).Text(FText::FromString("Capsule radius"))
                ]
                +SMassMuscleItemRow::Slot()
                [
                     SNew(SNumericEntryBox<float>)
                    .Value(this, &SMassMuscleDetailsPannel::GetBoneRadius)
                    .OnValueChanged(this, &SMassMuscleDetailsPannel::OnRadiusChanged)
                    .AllowSpin(true)
                    .MinValue(0.0f)
                    .MaxValue(TOptional<float>())
                    .MaxSliderValue(TOptional<float>())
                ]
            ]
            +SMassMuscleExpandableSection::Slot()
            [
                SNew(SMassMuscleItemRow)
                +SMassMuscleItemRow::Slot()
                [
                     SNew(STextBlock).Text(FText::FromString("Capsule half height"))
                ]
                +SMassMuscleItemRow::Slot()
                [
                     SNew(SNumericEntryBox<float>)
                    .Value(this, &SMassMuscleDetailsPannel::GetCapsuleHalfHeight)
                    .OnValueChanged(this, &SMassMuscleDetailsPannel::OnCapsuleHalfHeightChanged)
                    .AllowSpin(true)
                    .MinValue(0.0f)
                    .MaxValue(TOptional<float>())
                    .MaxSliderValue(TOptional<float>())
                ]
            ]
            +SMassMuscleExpandableSection::Slot()
            [
                SNew(SMassMuscleItemRow)
                +SMassMuscleItemRow::Slot()
                [
                     SNew(STextBlock).Text(FText::FromString("Can Touch Ground"))
                ]
                +SMassMuscleItemRow::Slot()
                [
                    SNew(SCheckBox)
                    .IsChecked(this, &SMassMuscleDetailsPannel::GetCanTouchGround)
                    .OnCheckStateChanged(this, &SMassMuscleDetailsPannel::OnCanTouchGroundChanged)
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
                    SNew(STextBlock).Text_Lambda([this] () {return FText::FromName(SelectedMuscle ? SelectedMuscle->BoneName : FName());})
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
                     SNew(STextBlock).Text(FText::FromString("Extension strength"))
                ]
                +SMassMuscleItemRow::Slot()
                [
                    SNew(SNumericEntryBox<float>)
                    .Value(this, &SMassMuscleDetailsPannel::GetExtensionStrength)
                    .OnValueChanged(this, &SMassMuscleDetailsPannel::OnExtensionStrengthChanged)
                    .AllowSpin(true)
                    .MinValue(TOptional<float>())
                    .MaxValue(TOptional<float>())
                    .MinSliderValue(TOptional<float>())
                    .MaxSliderValue(TOptional<float>())
                ]
            ]
            +SMassMuscleExpandableSection::Slot()
            [
                SNew(SMassMuscleItemRow)
                +SMassMuscleItemRow::Slot()
                [
                     SNew(STextBlock).Text(FText::FromString("Flexion strength"))
                ]
                +SMassMuscleItemRow::Slot()
                [
                    SNew(SNumericEntryBox<float>)
                    .Value(this, &SMassMuscleDetailsPannel::GetFlexionStrength)
                    .OnValueChanged(this, &SMassMuscleDetailsPannel::OnFlexionStrengthChanged)
                    .AllowSpin(true)
                    .MinValue(TOptional<float>())
                    .MaxValue(TOptional<float>())
                    .MinSliderValue(TOptional<float>())
                    .MaxSliderValue(TOptional<float>())
                ]
            ]
            +SMassMuscleExpandableSection::Slot()
            [
                SNew(SMassMuscleItemRow)
                +SMassMuscleItemRow::Slot()
                [
                     SNew(STextBlock).Text(FText::FromString("Activation threshold"))
                ]
                +SMassMuscleItemRow::Slot()
                [
                    SNew(SNumericEntryBox<float>)
                    .Value(this, &SMassMuscleDetailsPannel::GetMuscleActivationThreshold)
                    .OnValueChanged(this, &SMassMuscleDetailsPannel::OnMuscleActivationThresholdChanged)
                    .AllowSpin(true)
                    .Delta(0.01f)
                    .LinearDeltaSensitivity(1)
                    .MinValue(0.0f)
                    .MaxValue(1.0f)
                    .MinSliderValue(0.0f)
                    .MaxSliderValue(1.0f)
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
            +SMassMuscleExpandableSection::Slot()
            [
                SNew(SBorder)
                .Padding(4.0f)
                [
                    CurveDetailsView.ToSharedRef()
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

void SMassMuscleDetailsPannel::OnMuscleDataChanged()
{
    // Delete/Mirror can reallocate or shift the owning TArray, which would
    // leave SelectedMuscle/SelectedBone dangling since we only otherwise
    // refresh them on an explicit selection change. Re-resolve (or clear,
    // if the selected item no longer exists) against the current selection.
    if (Model->GetSelectedType() == EItemType::Muscle)
    {
        GetMuscleInfo(Model->GetSelected());
    }
    else if (Model->GetSelectedType() == EItemType::Bone)
    {
        GetBoneInfo(Model->GetSelected());
    }
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
    const int32 index = profile ? profile->FindMuscleByName(MuscleName) : INDEX_NONE;
    if(index == INDEX_NONE)
    {
        SelectedMuscle = nullptr;
        return;
    }
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
    SyncCurveProxyFromSelectedMuscle();
}

void SMassMuscleDetailsPannel::GetBoneInfo(FName boneName)
{
    UMassMuscleProfileAssetMass* profile = Model->GetMassProfile();
    const int32 index = profile ? profile->FindBoneByName(boneName) : INDEX_NONE;
    if(index == INDEX_NONE)
    {
        SelectedBone = nullptr;
        return;
    }
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

void SMassMuscleDetailsPannel::SyncCurveProxyFromSelectedMuscle()
{
    if (!CurveProxy.IsValid() || !CurveDetailsView.IsValid() || !SelectedMuscle)
    {
        return;
    }

    bUpdatingCurveProxy = true;
    CurveProxy->ExtensionStrengthCurve = SelectedMuscle->ExtensionStrengthCurve;
    CurveProxy->FlexionStrengthCurve = SelectedMuscle->FlexionStrengthCurve;
    CurveDetailsView->SetObject(CurveProxy.Get());
    bUpdatingCurveProxy = false;
}

void SMassMuscleDetailsPannel::ApplyCurveProxyToSelectedMuscle() const
{
    if (!CurveProxy.IsValid() || !SelectedMuscle)
    {
        return;
    }

    SelectedMuscle->ExtensionStrengthCurve = CurveProxy->ExtensionStrengthCurve;
    SelectedMuscle->FlexionStrengthCurve = CurveProxy->FlexionStrengthCurve;
}

void SMassMuscleDetailsPannel::OnCurveDetailsChanged(const FPropertyChangedEvent& PropertyChangedEvent)
{
    if (bUpdatingCurveProxy)
    {
        return;
    }

    ApplyCurveProxyToSelectedMuscle();
    MarkDirtyMuscle();
}

