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
        const TArray<TSharedPtr<FMassMuscleTreeItem>> ValidSelectedItems = SelectedItems.FilterByPredicate([](const TSharedPtr<FMassMuscleTreeItem>& Item)
        {
            return Item.IsValid() && Item->ItemType == EItemType::Muscle;
        });

        for (const TSharedPtr<FMassMuscleTreeItem>& SelectedItem : ValidSelectedItems)
        {
            const int32 MuscleIndex = MuscleProfile->FindMuscleByName(SelectedItem->Name);
            if (MuscleIndex == INDEX_NONE)
            {
                continue;
            }

            const FMassMuscleDataMuscle MuscleToDelete = MuscleProfile->Muscles[MuscleIndex];
            OnDeleteMuscleDelegate(MuscleToDelete);
            MuscleProfile->RemoveMuscle(MuscleToDelete.Name);
        }

        MuscleProfile->MarkPackageDirty();
        OnMuscleDataChanged.Broadcast();
    }

    void MirrorMuscles(const TArray<TSharedPtr<FMassMuscleTreeItem>>& SelectedItems, TFunction<void(const FMassMuscleDataMuscle&)> OnAddMuscleDelegate)
    {
        if(!MuscleProfile) return;
        if(!MuscleProfile->SkeletalMesh) return;

        for (const TSharedPtr<FMassMuscleTreeItem>& SelectedItem : SelectedItems)
        {
            if (!SelectedItem.IsValid() || SelectedItem->ItemType != EItemType::Muscle)
            {
                continue;
            }

            const int32 MuscleIndex = MuscleProfile->FindMuscleByName(SelectedItem->Name);
            if (MuscleIndex == INDEX_NONE)
            {
                continue;
            }

            FMassMuscleDataMuscle& Muscle = MuscleProfile->Muscles[MuscleIndex];
            if (Muscle.Name == NAME_None || Muscle.BoneName == NAME_None)
            {
                continue;
            }

            const FName MirrorMuscleName = GetMirrorBoneName(Muscle.Name);
            const int32 MirrorIndex = MuscleProfile->FindMuscleByName(MirrorMuscleName);
            if (MirrorIndex != INDEX_NONE)
            {
                MuscleProfile->Muscles[MirrorIndex].MaxRange = Muscle.MaxRange;
                MuscleProfile->Muscles[MirrorIndex].MinRange = Muscle.MinRange;
                MuscleProfile->Muscles[MirrorIndex].ExtensionStrength = Muscle.ExtensionStrength;
                MuscleProfile->Muscles[MirrorIndex].FlexionStrength = Muscle.FlexionStrength;
                MuscleProfile->Muscles[MirrorIndex].Orientation = Muscle.Orientation;
                continue;
            }

            const FName MirrorBone = GetMirrorBoneName(Muscle.BoneName);
            const int32 MirrorBoneIndex = MuscleProfile->SkeletalMesh->GetRefSkeleton().FindBoneIndex(MirrorBone);
            if (MirrorBoneIndex == INDEX_NONE)
            {
                continue;
            }

            FMassMuscleDataMuscle NewMuscle = Muscle;
            NewMuscle.Name = MirrorMuscleName;
            NewMuscle.BoneName = MirrorBone;

            TArray<int32> ChildBoneIndices;
            MuscleProfile->SkeletalMesh->GetRefSkeleton().GetDirectChildBones(MirrorBoneIndex, ChildBoneIndices);
            if (ChildBoneIndices.Num() == 0)
            {
                continue;
            }

            NewMuscle.ChildBoneName = MuscleProfile->SkeletalMesh->GetRefSkeleton().GetBoneName(ChildBoneIndices[0]);
            MuscleProfile->Muscles.Add(NewMuscle);
            OnAddMuscleDelegate(NewMuscle);
        }

        MuscleProfile->MarkPackageDirty();
        OnMuscleDataChanged.Broadcast();
    }

    // Copies capsule geometry (Radius, CapsuleHalfHeight) from each selected
    // bone onto its L/R mirror bone — every bone already has a Mass entry
    // (populated 1:1 from the skeleton by InitializeFromSkeletalMesh), so
    // unlike MirrorMuscles there's no "create if missing" case to handle.
    void MirrorCapsules(const TArray<TSharedPtr<FMassMuscleTreeItem>>& SelectedItems)
    {
        if(!MassProfile) return;

        for (const TSharedPtr<FMassMuscleTreeItem>& SelectedItem : SelectedItems)
        {
            if (!SelectedItem.IsValid() || SelectedItem->ItemType != EItemType::Bone)
            {
                continue;
            }

            const int32 BoneMassIndex = MassProfile->FindBoneByName(SelectedItem->Name);
            if (BoneMassIndex == INDEX_NONE)
            {
                continue;
            }

            const FName MirrorBoneName = GetMirrorBoneName(SelectedItem->Name);
            const int32 MirrorMassIndex = MassProfile->FindBoneByName(MirrorBoneName);
            if (MirrorMassIndex == INDEX_NONE)
            {
                continue;
            }

            MassProfile->Mass[MirrorMassIndex].Radius = MassProfile->Mass[BoneMassIndex].Radius;
            MassProfile->Mass[MirrorMassIndex].CapsuleHalfHeight = MassProfile->Mass[BoneMassIndex].CapsuleHalfHeight;
        }

        MassProfile->MarkPackageDirty();
    }

    FOnAddMuscle             OnMuscleAdd;
    FOnMuscleDataChanged     OnMuscleDataChanged;
private:
    // Swaps a name's _L/_R suffix (case-insensitive), leaving it unchanged if
    // it has neither — shared by MirrorMuscles and MirrorCapsules.
    static FName GetMirrorBoneName(FName Name)
    {
        if (Name == NAME_None)
        {
            return Name;
        }
        const FString NameString = Name.ToString();
        const FString Suffix = NameString.Right(2).ToUpper();
        if (Suffix == TEXT("_R"))
        {
            return FName(NameString.LeftChop(2) + TEXT("_L"));
        }
        if (Suffix == TEXT("_L"))
        {
            return FName(NameString.LeftChop(2) + TEXT("_R"));
        }
        return Name;
    }

    TObjectPtr<USkeletalMesh> SkeletalMesh = nullptr;
    FName                     Selected = TEXT("");
    int32                     SelectedBoneIndex = INDEX_NONE;
    TObjectPtr<UMassMuscleProfileAssetMuscle> MuscleProfile = nullptr;
    TObjectPtr<UMassMuscleProfileAssetMass> MassProfile = nullptr;
    EItemType                 SelectedType = EItemType::Bone;
};