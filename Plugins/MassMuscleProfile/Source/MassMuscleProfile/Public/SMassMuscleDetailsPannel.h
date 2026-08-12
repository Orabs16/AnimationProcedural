#pragma once

#include "Widgets/SCompoundWidget.h"
#include "Widgets/DeclarativeSyntaxSupport.h"
#include "FMassMuscleEditorModel.h"
#include "Widgets/Layout/SWidgetSwitcher.h"
#include "SMassMuscleHierarchy.h"
#include "Widgets/Input/SComboBox.h"
#include "Styling/SlateTypes.h"
#include "UObject/StrongObjectPtr.h"
#include "FMassMuscleData.h"

class IDetailsView;
class UMassMuscleCurveEditorProxy;
struct FPropertyChangedEvent;

class SMassMuscleDetailsPannel : public SCompoundWidget
{
public:
    SLATE_BEGIN_ARGS(SMassMuscleDetailsPannel) {}
        SLATE_ARGUMENT(TSharedPtr<FMassMuscleEditorModel>, Model)
        SLATE_ARGUMENT(TSharedPtr<class SMassMuscleHierarchy>, Hierarchy)
    SLATE_END_ARGS()

	void Construct(const FArguments& InArgs);
	
private:
    void OnSelectionChanged(FName NewSelectionName, EItemType Type);
    void OnMuscleDataChanged();
    TSharedPtr<FMassMuscleEditorModel> Model;
    TSharedPtr<SWidgetSwitcher> PanelSwitcher;
    void GetMuscleInfo(FName muscleName);
    void GetBoneInfo(FName boneName);
    FMassMuscleDataMuscle* SelectedMuscle = nullptr;
    FMassMuscleDataMass* SelectedBone = nullptr;
    TSharedPtr<IDetailsView> CurveDetailsView;
    TStrongObjectPtr<UMassMuscleCurveEditorProxy> CurveProxy;
    bool bUpdatingCurveProxy = false;

    void MarkDirtyMass() const;
    void MarkDirtyMuscle() const;
    void SyncCurveProxyFromSelectedMuscle();
    void ApplyCurveProxyToSelectedMuscle() const;
    void OnCurveDetailsChanged(const FPropertyChangedEvent& PropertyChangedEvent);

    TSharedPtr<SMassMuscleHierarchy> Hierarchy;

    TOptional<float> GetBoneMass() const{
        if(!SelectedBone) return 0;
        return SelectedBone->Mass;
    };
    void OnMassChanged(float value) const{
        if(SelectedBone){
            SelectedBone->Mass = value;
            SMassMuscleDetailsPannel::MarkDirtyMass();
        }
    }

    TOptional<float> GetBoneRadius() const{
        if(!SelectedBone) return 0;
        return SelectedBone->Radius;
    };
    void OnRadiusChanged(float value) const{
        if(SelectedBone){
            SelectedBone->Radius = value;
            SMassMuscleDetailsPannel::MarkDirtyMass();
        }
    }

    TOptional<float> GetCapsuleHalfHeight() const{
        if(!SelectedBone) return 0;
        return SelectedBone->CapsuleHalfHeight;
    };
    void OnCapsuleHalfHeightChanged(float value) const{
        if(SelectedBone){
            SelectedBone->CapsuleHalfHeight = value;
            SMassMuscleDetailsPannel::MarkDirtyMass();
        }
    }

    TOptional<float> GetExtensionStrength() const{return SelectedMuscle ? SelectedMuscle->ExtensionStrength : 0;}
    void OnExtensionStrengthChanged(float value) const {if(SelectedMuscle){ SelectedMuscle->ExtensionStrength = value; SMassMuscleDetailsPannel::MarkDirtyMuscle(); }}
    TOptional<float> GetFlexionStrength() const{return SelectedMuscle ? SelectedMuscle->FlexionStrength : 0;}
    void OnFlexionStrengthChanged(float value) const {if(SelectedMuscle){ SelectedMuscle->FlexionStrength = value; SMassMuscleDetailsPannel::MarkDirtyMuscle(); }}

    ECheckBoxState GetCanTouchGround() const
    {
        return (SelectedBone && SelectedBone->CanTouchGround)
            ? ECheckBoxState::Checked
            : ECheckBoxState::Unchecked;
    }

    void OnCanTouchGroundChanged(ECheckBoxState NewState) const
    {
        if (SelectedBone)
        {
            SelectedBone->CanTouchGround = (NewState == ECheckBoxState::Checked);
            SMassMuscleDetailsPannel::MarkDirtyMass();
        }
    }

    FText GetMuscleName() const{return SelectedMuscle ? FText::FromName(SelectedMuscle->Name) : FText::FromString(TEXT(""));}
    void CommitNewName(const FText& value, ETextCommit::Type CommitType) const{
        if(value.ToString() == TEXT("")) return;
        if(!Model) return;
        if(!SelectedMuscle) return;
        if(!Model->GetMuscleProfile()) return;
        FName oldName = FName(SelectedMuscle->Name.ToString());
        Model->GetMuscleProfile()->RenameMuscle(*SelectedMuscle, FName(value.ToString()));
        Hierarchy->OnItemNameChanged(oldName, FName(SelectedMuscle->Name.ToString()));
    }
    TOptional<float> GetMaxRange() const{return SelectedMuscle? SelectedMuscle->MaxRange : 0;}
    void OnMaxRangeChanged(float value) const {if(SelectedMuscle){ SelectedMuscle->MaxRange = value; SMassMuscleDetailsPannel::MarkDirtyMuscle(); }}
    TOptional<float> GetMinRange() const{return SelectedMuscle? SelectedMuscle->MinRange : 0;}
    void OnMinRangeChanged(float value) const {if(SelectedMuscle){ SelectedMuscle->MinRange = value; SMassMuscleDetailsPannel::MarkDirtyMuscle(); }}

    TSharedPtr<class SComboBox<TSharedPtr<FString>>> MuscleComboBox;
    TArray<TSharedPtr<FString>> MuscleChildOptions = {MakeShared<FString>(TEXT("None"))};
    TSharedPtr<FString> MuscleChildCurrentSelection = MakeShared<FString>(TEXT("None"));

    void OnChildBoneChanged(TSharedPtr<FString> NewSelection, ESelectInfo::Type SelectInfo)
    {
        if(!NewSelection.IsValid()) return;
        if(!SelectedMuscle) return;
        if(!Model) return;
        if(!Model->GetMuscleProfile()) return;
        SelectedMuscle->ChildBoneName = FName(*NewSelection);
        MuscleChildCurrentSelection = NewSelection;
        SMassMuscleDetailsPannel::MarkDirtyMuscle();
    }

    TArray<TSharedPtr<FString>> RotationOptions = {MakeShared<FString>(TEXT("Yaw")), MakeShared<FString>(TEXT("Pitch")), MakeShared<FString>(TEXT("Roll"))};
    TSharedPtr<FString> MuscleRotationCurrentSelection = MakeShared<FString>(TEXT("None"));
    void OnOrientationChanged(TSharedPtr<FString> NewSelection, ESelectInfo::Type SelectInfo)
    {
        if(!NewSelection.IsValid()) return;
        if(!SelectedMuscle) return;
        if(!Model) return;
        if(!Model->GetMuscleProfile()) return;
        if(*NewSelection == TEXT("Yaw")) SelectedMuscle->Orientation = ERotationType::Yaw;
        else if(*NewSelection == TEXT("Pitch")) SelectedMuscle->Orientation = ERotationType::Pitch;
        else if(*NewSelection == TEXT("Roll")) SelectedMuscle->Orientation = ERotationType::Roll;
        MuscleRotationCurrentSelection = NewSelection;
        SMassMuscleDetailsPannel::MarkDirtyMuscle();
    }
};