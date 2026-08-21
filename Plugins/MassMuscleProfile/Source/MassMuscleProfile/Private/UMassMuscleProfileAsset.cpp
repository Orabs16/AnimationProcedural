#include "UMassMuscleProfileAsset.h"
#include "FMassMuscleData.h"
#include "Engine/SkeletalMesh.h"

void UMassMuscleProfileAssetMuscle::PostLoad()
{
    Super::PostLoad();
    EnsureMuscleCurvesInitialized();
}

bool UMassMuscleProfileAssetMuscle::EnsureMuscleCurvesInitialized()
{
    bool bUpdatedAnyMuscle = false;

    for (FMassMuscleDataMuscle& Muscle : Muscles)
    {
        bUpdatedAnyMuscle |= EnsureDefaultStrengthCurvesInitialized(Muscle);
    }

    if (bUpdatedAnyMuscle)
    {
        MarkPackageDirty();
    }

    return bUpdatedAnyMuscle;
}

void UMassMuscleProfileAssetMass::PostLoad()
{
    Super::PostLoad();
    // Repair on load rather than on save: an asset saved before BoneIndex was
    // read by anything carries whatever was there at authoring time, and every
    // consumer now keys off it. Deliberately does NOT MarkPackageDirty -- a
    // repair that only restores the derived field costs nothing to redo next
    // load, and dirtying every asset on open is worse than recomputing.
    SyncBoneIndices();
}

bool UMassMuscleProfileAssetMass::SyncBoneIndices()
{
    if (!SkeletalMesh)
    {
        return false;
    }

    const FReferenceSkeleton& RefSkeleton = SkeletalMesh->GetRefSkeleton();
    bool bChanged = false;
    for (FMassMuscleDataMass& Entry : Mass)
    {
        const int32 Resolved = RefSkeleton.FindBoneIndex(Entry.BoneName);
        if (Entry.BoneIndex != Resolved)
        {
            Entry.BoneIndex = Resolved;
            bChanged = true;
        }
    }
    return bChanged;
}

void UMassMuscleProfileAssetMass::InitializeFromSkeletalMesh(USkeletalMesh* InSkeletalMesh)
{
    if (!InSkeletalMesh)
    {
        return;
    }


    SkeletalMesh = InSkeletalMesh;
    Mass.Reset();

    const FReferenceSkeleton& RefSkeleton = InSkeletalMesh->GetRefSkeleton();
    const int32 NumBones = RefSkeleton.GetNum();

    Mass.Reserve(NumBones);
    for (int32 i = 0; i < NumBones; ++i)
    {
        FMassMuscleDataMass NewEntry;
        NewEntry.BoneName = RefSkeleton.GetBoneName(i);
        NewEntry.BoneIndex = i; // one entry per bone, in bone order -- FindBoneIndex(GetBoneName(i)) is just i
        Mass.Add(NewEntry);
    }
}

