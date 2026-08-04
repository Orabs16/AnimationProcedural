#include "UMassMuscleRuntimeProfileAsset.h"
#include "UMassMuscleRuntimeState.h"
#include "Engine/SkeletalMesh.h"

UMassMuscleRuntimeState* UMassMuscleRuntimeProfileAsset::CreateInitializedRuntimeState(UObject* Outer) const
{
    if (SkeletalMesh == nullptr)
    {
        UE_LOG(LogTemp, Warning, TEXT("CreateInitializedRuntimeState failed: SkeletalMesh is null on profile '%s'."), *GetName());
        return nullptr;
    }

    return UMassMuscleRuntimeState::CreateInitializedRuntimeState(Outer, SkeletalMesh, Muscles, Masses);
}

bool UMassMuscleRuntimeProfileAsset::ReinitializeRuntimeState(UMassMuscleRuntimeState* RuntimeState) const
{
    if (RuntimeState == nullptr)
    {
        UE_LOG(LogTemp, Warning, TEXT("ReinitializeRuntimeState failed: RuntimeState is null for profile '%s'."), *GetName());
        return false;
    }

    if (SkeletalMesh == nullptr)
    {
        UE_LOG(LogTemp, Warning, TEXT("ReinitializeRuntimeState failed: SkeletalMesh is null on profile '%s'."), *GetName());
        return false;
    }

    RuntimeState->Initialize(SkeletalMesh, Muscles, Masses);
    return true;
}

bool UMassMuscleRuntimeProfileAsset::ValidateOneArmChain(FName RootBone, FName TipBone, FString& OutMessage) const
{
    OutMessage.Reset();

    if (SkeletalMesh == nullptr)
    {
        OutMessage = TEXT("Profile has no SkeletalMesh.");
        return false;
    }

    if (RootBone == NAME_None || TipBone == NAME_None)
    {
        OutMessage = TEXT("RootBone and TipBone must be valid names.");
        return false;
    }

    const FReferenceSkeleton& RefSkeleton = SkeletalMesh->GetRefSkeleton();
    const int32 RootBoneIndex = RefSkeleton.FindBoneIndex(RootBone);
    const int32 TipBoneIndex = RefSkeleton.FindBoneIndex(TipBone);

    if (RootBoneIndex == INDEX_NONE)
    {
        OutMessage = FString::Printf(TEXT("RootBone '%s' was not found in '%s'."), *RootBone.ToString(), *SkeletalMesh->GetName());
        return false;
    }

    if (TipBoneIndex == INDEX_NONE)
    {
        OutMessage = FString::Printf(TEXT("TipBone '%s' was not found in '%s'."), *TipBone.ToString(), *SkeletalMesh->GetName());
        return false;
    }

    int32 CurrentBoneIndex = TipBoneIndex;
    int32 ChainLength = 1;
    bool bRootFound = (TipBoneIndex == RootBoneIndex);

    while (!bRootFound && CurrentBoneIndex != INDEX_NONE)
    {
        CurrentBoneIndex = RefSkeleton.GetParentIndex(CurrentBoneIndex);
        if (CurrentBoneIndex != INDEX_NONE)
        {
            ++ChainLength;
            bRootFound = (CurrentBoneIndex == RootBoneIndex);
        }
    }

    if (!bRootFound)
    {
        OutMessage = FString::Printf(
            TEXT("Invalid chain: '%s' is not an ancestor of '%s'."),
            *RootBone.ToString(),
            *TipBone.ToString());
        return false;
    }

    if (ChainLength < 2)
    {
        OutMessage = FString::Printf(
            TEXT("Invalid chain: '%s' and '%s' resolve to a single bone. Choose at least one joint in between."),
            *RootBone.ToString(),
            *TipBone.ToString());
        return false;
    }

    OutMessage = FString::Printf(
        TEXT("Valid one-arm chain (%d bones): %s -> %s."),
        ChainLength,
        *RootBone.ToString(),
        *TipBone.ToString());
    return true;
}
