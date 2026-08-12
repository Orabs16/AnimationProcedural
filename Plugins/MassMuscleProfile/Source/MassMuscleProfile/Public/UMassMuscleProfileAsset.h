#pragma once
#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "FMassMuscleData.h"
#include "Engine/SkeletalMesh.h"
#include "UMassMuscleProfileAsset.generated.h"

UCLASS(BlueprintType)
class MASSMUSCLEPROFILE_API UMassMuscleProfileAssetMuscle : public UDataAsset
{
    GENERATED_BODY()

public:

    virtual void PostLoad() override;

    
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SkeletalMesh")
    USkeletalMesh* SkeletalMesh;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Muscles")
    TArray<FMassMuscleDataMuscle> Muscles;

    bool EnsureMuscleCurvesInitialized();

    

    // ── Helpers ───────────────────────────────────────────────────────────────

    // Returns INDEX_NONE if not found
    int32 FindMuscleByName(FName InName) const
    {
        return Muscles.IndexOfByPredicate(
            [&](const FMassMuscleDataMuscle& M){ return M.Name == InName; });
    }

    bool AddMuscle(FName BoneAttachedTo, FMassMuscleDataMuscle& OutNewMuscle)
    {
        
        auto NewMuscleName = [BoneAttachedTo, this]() -> FName
        { 
            FString baseNameSuffix = BoneAttachedTo.ToString().Right(2);
            FString newName;
            if(baseNameSuffix.ToUpper() == "_L" || baseNameSuffix.ToUpper() == "_R")
            {
                newName = BoneAttachedTo.ToString().LeftChop(2);
                newName += "_muscle";
                newName = newName + baseNameSuffix;
            }
            else
            {
                newName = BoneAttachedTo.ToString() + "_muscle";
            }
            int count = 1;
            while(this->FindMuscleByName(FName(newName)) != INDEX_NONE)
            {
                if(baseNameSuffix.ToUpper() == "_L" || baseNameSuffix.ToUpper() == "_R")
                {
                    newName = BoneAttachedTo.ToString().LeftChop(2);
                    newName += "_muscle";
                    newName += FString::Printf(TEXT("_%d"), count);
                    newName = newName + baseNameSuffix;
                }
                else
                {
                    newName = BoneAttachedTo.ToString() + "_muscle";
                    newName += FString::Printf(TEXT("_%d"), count);
                }
                count++;
            }
            return FName(newName);

        };
        if(BoneAttachedTo == NAME_None) return false;
        if(!SkeletalMesh) return false;
        FReferenceSkeleton RefSkeleton = SkeletalMesh->GetRefSkeleton();
        int32 BoneIndex = RefSkeleton.FindBoneIndex(BoneAttachedTo);
        if(BoneIndex == INDEX_NONE) return false;
        FName NewName = NewMuscleName();
        FName NewBoneName = BoneAttachedTo;
        TArray<FName> NewChildBoneNames;
        GetChildBonesNames(NewBoneName, NewChildBoneNames);
        if(NewChildBoneNames.Num() == 0) return false;
        FName NewChildBoneName = NewChildBoneNames[0];

        OutNewMuscle = FMassMuscleDataMuscle();
        OutNewMuscle.Name = NewName;
        OutNewMuscle.BoneName = NewBoneName;
        OutNewMuscle.ChildBoneName = NewChildBoneName;
        OutNewMuscle.Orientation = ERotationType::Yaw;
        OutNewMuscle.MinRange = -45.f;
        OutNewMuscle.MaxRange = 45.f;
        EnsureDefaultStrengthCurvesInitialized(OutNewMuscle);
        Muscles.Add(OutNewMuscle);
        MarkPackageDirty();   // tells UE the asset has unsaved changes
        return true;
    }

    void RemoveMuscle(FName InName)
    {
        Muscles.RemoveAll(
            [&](const FMassMuscleDataMuscle& M){ return M.Name == InName; });
        MarkPackageDirty();
    }

    void NotifyMuscleChange()
    {
        MarkPackageDirty();
    }
    void RenameMuscle(const FMassMuscleDataMuscle& Muscle, FName NewName)
    {
        if(!this) return;
        if(!SkeletalMesh) return;
        if(FindMuscleByName(NewName) != INDEX_NONE) return;
        int32 MuscleIndex = FindMuscleByName(Muscle.Name);
        if(MuscleIndex == INDEX_NONE) return;
        Muscles[MuscleIndex].Name = NewName;
        MarkPackageDirty();
        return;
    }

    void GetChildBonesNames(FName BoneName, TArray<FName>& OutChildBones) const
    {
        TArray<int32> ChildBoneIndices;
        GetChildBones(BoneName, ChildBoneIndices);
        if(!SkeletalMesh) return;
        for(int32 ChildBoneIndex : ChildBoneIndices)
        {
            FName ChildBoneName = SkeletalMesh->GetRefSkeleton().GetBoneName(ChildBoneIndex);
            OutChildBones.Add(ChildBoneName);
        }
    }
    void GetChildBones(FName BoneName, TArray<int32>& OutChildBones) const
    {
        if(!SkeletalMesh) return;
        int32 BoneIndex = SkeletalMesh->GetRefSkeleton().FindBoneIndex(BoneName);
        if(BoneIndex == INDEX_NONE) return;
        SkeletalMesh->GetRefSkeleton().GetDirectChildBones(BoneIndex, OutChildBones);
    }
};

UCLASS(BlueprintType)
class MASSMUSCLEPROFILE_API UMassMuscleProfileAssetMass : public UDataAsset
{
    GENERATED_BODY()

public:

    
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SkeletalMesh")
    USkeletalMesh* SkeletalMesh;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mass")
    TArray<FMassMuscleDataMass> Mass;


    UFUNCTION(BlueprintCallable, Category="MassMuscleProfile")
    void InitializeFromSkeletalMesh(USkeletalMesh* InSkeletalMesh);

    void NotifyMassChange()
    {
        MarkPackageDirty();
    }
    int32 FindBoneByName(FName InName) const
    {
        return Mass.IndexOfByPredicate(
            [&](const FMassMuscleDataMass& M){ return M.BoneName == InName; });
    }
};