#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "FMassMuscleData.h"
#include "UMassMuscleRuntimeProfileAsset.generated.h"

class UMassMuscleRuntimeState;
class USkeletalMesh;

UCLASS(BlueprintType)
class MASSMUSCLEPROFILERUNTIME_API UMassMuscleRuntimeProfileAsset : public UDataAsset
{
    GENERATED_BODY()

public:
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "MassMuscleProfile|Runtime")
    TObjectPtr<USkeletalMesh> SkeletalMesh = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "MassMuscleProfile|Runtime")
    TArray<FMassMuscleDataMuscle> Muscles;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "MassMuscleProfile|Runtime")
    TArray<FMassMuscleDataMass> Masses;

    UFUNCTION(BlueprintCallable, Category = "MassMuscleProfile|Runtime")
    UMassMuscleRuntimeState* CreateInitializedRuntimeState(UObject* Outer) const;

    UFUNCTION(BlueprintCallable, Category = "MassMuscleProfile|Runtime")
    bool ReinitializeRuntimeState(UMassMuscleRuntimeState* RuntimeState) const;

    UFUNCTION(BlueprintCallable, Category = "MassMuscleProfile|Runtime")
    bool ValidateOneArmChain(FName RootBone, FName TipBone, FString& OutMessage) const;
};
