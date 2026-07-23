#pragma once
#include "CoreMinimal.h"
#include "FMassMuscleData.h"
#include "UMassMuscleProfileAsset.h"
#include "UMassMuscleRuntimeState.generated.h"

UCLASS(BlueprintType)
class UMassMuscleRuntimeState : public UObject
{
    GENERATED_BODY()
public:
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TArray<FMuscleConstraintData> Muscles;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TArray<FBoneKinematicState> BoneStates;

    UPROPERTY()
    TMap<int32, FMuscleIndexArray> BoneIndexToMuscleIndices;

    void InitializeFromAsset(const UMassMuscleProfileAssetMuscle* Asset);
};