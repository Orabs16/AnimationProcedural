#include "UMassMuscleRuntimeBlueprintLibrary.h"
#include "UMassMuscleRuntimeProfileAsset.h"
#include "UMassMuscleRuntimeState.h"

UMassMuscleRuntimeState* UMassMuscleRuntimeBlueprintLibrary::CreateRuntimeStateFromProfile(
    UObject* Outer,
    const UMassMuscleRuntimeProfileAsset* Profile)
{
    if (Profile == nullptr)
    {
        UE_LOG(LogTemp, Warning, TEXT("CreateRuntimeStateFromProfile failed: Profile is null."));
        return nullptr;
    }

    return Profile->CreateInitializedRuntimeState(Outer);
}

bool UMassMuscleRuntimeBlueprintLibrary::ReinitializeRuntimeStateFromProfile(
    UMassMuscleRuntimeState* RuntimeState,
    const UMassMuscleRuntimeProfileAsset* Profile)
{
    if (Profile == nullptr)
    {
        UE_LOG(LogTemp, Warning, TEXT("ReinitializeRuntimeStateFromProfile failed: Profile is null."));
        return false;
    }

    return Profile->ReinitializeRuntimeState(RuntimeState);
}

bool UMassMuscleRuntimeBlueprintLibrary::ValidateOneArmSetup(
    const UMassMuscleRuntimeProfileAsset* Profile,
    FName RootBone,
    FName TipBone,
    FString& OutMessage)
{
    if (Profile == nullptr)
    {
        OutMessage = TEXT("Profile is null.");
        return false;
    }

    return Profile->ValidateOneArmChain(RootBone, TipBone, OutMessage);
}
