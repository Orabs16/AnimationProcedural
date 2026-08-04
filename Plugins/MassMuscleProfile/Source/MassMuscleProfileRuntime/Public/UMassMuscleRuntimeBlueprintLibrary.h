#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "UMassMuscleRuntimeBlueprintLibrary.generated.h"

class UMassMuscleRuntimeProfileAsset;
class UMassMuscleRuntimeState;

UCLASS()
class MASSMUSCLEPROFILERUNTIME_API UMassMuscleRuntimeBlueprintLibrary : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()

public:
    UFUNCTION(BlueprintCallable, Category = "MassMuscleProfile|Runtime", meta = (DefaultToSelf = "Outer"))
    static UMassMuscleRuntimeState* CreateRuntimeStateFromProfile(UObject* Outer, const UMassMuscleRuntimeProfileAsset* Profile);

    UFUNCTION(BlueprintCallable, Category = "MassMuscleProfile|Runtime")
    static bool ReinitializeRuntimeStateFromProfile(UMassMuscleRuntimeState* RuntimeState, const UMassMuscleRuntimeProfileAsset* Profile);

    UFUNCTION(BlueprintCallable, Category = "MassMuscleProfile|Runtime")
    static bool ValidateOneArmSetup(const UMassMuscleRuntimeProfileAsset* Profile, FName RootBone, FName TipBone, FString& OutMessage);
};
