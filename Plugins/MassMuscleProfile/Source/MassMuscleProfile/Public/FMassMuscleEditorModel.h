#pragma once

#include "CoreMinimal.h"
#include "Engine/SkeletalMesh.h"
#include "FMassMuscleData.h"
#include "HAL/Platform.h"
#include "UMassMuscleProfileAsset.h"
#include "FMassMuscleTreeItem.h"
#include "FileHelpers.h"

DECLARE_MULTICAST_DELEGATE_OneParam(FOnSkeletalMeshChanged, USkeletalMesh*);
DECLARE_MULTICAST_DELEGATE_TwoParams(FOnSelectionChanged, FName, EItemType);
DECLARE_MULTICAST_DELEGATE_OneParam(FOnProfileChanged, UMassMuscleProfileAssetMuscle*);
DECLARE_MULTICAST_DELEGATE_OneParam(FOnAddMuscle, FMassMuscleDataMuscle);
DECLARE_MULTICAST_DELEGATE(FOnMuscleDataChanged);
class FMassMuscleEditorModel
{
public:

    USkeletalMesh*  GetSkeletalMesh()    const { return SkeletalMesh; }
    FName           GetSelected()    const { return Selected; }
    UMassMuscleProfileAssetMuscle* GetMuscleProfile() const { return MuscleProfile; }
    UMassMuscleProfileAssetMass* GetMassProfile() const { return MassProfile; }
    FVector MyVector;  
    EItemType GetSelectedType() const {return SelectedType; }

    void SetSkeletalMesh(USkeletalMesh* InMesh)
    {
        if (SkeletalMesh == InMesh) return;
        SkeletalMesh = InMesh;
        OnSkeletalMeshChanged.Broadcast(SkeletalMesh);  // ← notify everyone
    }
    void SetSelected(FName InItem, EItemType InSelectedType)
    {
        if (Selected == InItem) return;
        Selected = InItem;
        if (!SkeletalMesh || Selected.IsNone())
        {
            SelectedBoneIndex = INDEX_NONE;
            return;
        }
        SelectedType = InSelectedType;
        if(InSelectedType == EItemType::Bone)
        {
            SelectedBoneIndex = SkeletalMesh->GetRefSkeleton().FindBoneIndex(Selected);
        }
        if(InSelectedType == EItemType::Muscle)
        {
            SelectedBoneIndex = INDEX_NONE;
        }
        OnSelectionChanged.Broadcast(Selected, InSelectedType);
    }
    void SetMuscleProfile(UMassMuscleProfileAssetMuscle* InProfile)
    {
        if (MuscleProfile == InProfile) return;
        MuscleProfile = InProfile;
    }
    void SetMassProfile(UMassMuscleProfileAssetMass* InProfile)
    {
        if (MassProfile == InProfile) return;
        MassProfile = InProfile;
    }

    int32 GetSelectedBoneIndex() const
    {
        return SelectedBoneIndex;
    }
    FOnSkeletalMeshChanged      OnSkeletalMeshChanged;
    FOnSelectionChanged         OnSelectionChanged;
    FOnProfileChanged           OnMuscleProfileChanged;

    void AddMuscle()
    {
        if(!MuscleProfile) return;
        if(Selected.IsNone()) return;
        if(SelectedType != EItemType::Bone) return;
        FMassMuscleDataMuscle InMuscle;
        if(MuscleProfile->AddMuscle(Selected, InMuscle))
        {
            OnMuscleAdd.Broadcast(InMuscle);
        }
    }

    void SaveProfile()
    {
        if(!MuscleProfile) return;
        UEditorLoadingAndSavingUtils::SavePackages({MuscleProfile->GetPackage()}, false);
        if(!MassProfile) return;
        UEditorLoadingAndSavingUtils::SavePackages({MassProfile->GetPackage()}, false);
    }
    void DeleteMuscles(const TArray<TSharedPtr<FMassMuscleTreeItem>>& SelectedItems, TFunction<void(const FMassMuscleDataMuscle&)> OnDeleteMuscleDelegate)
    {
        if(!MuscleProfile) return;
        MuscleProfile->DeleteMuscles(SelectedItems, OnDeleteMuscleDelegate);
        OnMuscleDataChanged.Broadcast();
    }

    void MirrorMuscles(const TArray<TSharedPtr<FMassMuscleTreeItem>>& SelectedItems, TFunction<void(const FMassMuscleDataMuscle&)> OnAddMuscleDelegate)
    {
        if(!MuscleProfile) return;
        MuscleProfile->MirrorMuscles(SelectedItems, OnAddMuscleDelegate);
        OnMuscleDataChanged.Broadcast();
    }

    FOnAddMuscle             OnMuscleAdd;
    FOnMuscleDataChanged     OnMuscleDataChanged;
private:
    TObjectPtr<USkeletalMesh> SkeletalMesh = nullptr;
    FName                     Selected = TEXT("");
    int32                     SelectedBoneIndex = INDEX_NONE;
    TObjectPtr<UMassMuscleProfileAssetMuscle> MuscleProfile = nullptr;
    TObjectPtr<UMassMuscleProfileAssetMass> MassProfile = nullptr;
    EItemType                 SelectedType = EItemType::Bone;
};