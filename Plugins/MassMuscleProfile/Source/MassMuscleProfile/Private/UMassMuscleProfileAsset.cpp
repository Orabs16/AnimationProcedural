#include "UMassMuscleProfileAsset.h"
#include "FMassMuscleData.h"
#include "Engine/SkeletalMesh.h"
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
        NewEntry.BoneIndex = RefSkeleton.FindBoneIndex(NewEntry.BoneName);
        Mass.Add(NewEntry);
    }
}

