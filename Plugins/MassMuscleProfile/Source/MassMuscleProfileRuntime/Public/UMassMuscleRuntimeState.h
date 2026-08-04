#pragma once
#include "CoreMinimal.h"
#include "FMassMuscleData.h"
#include "UMassMuscleRuntimeState.generated.h"

UCLASS(BlueprintType)
class MASSMUSCLEPROFILERUNTIME_API UMassMuscleRuntimeState : public UObject
{
    GENERATED_BODY()
public:
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TArray<FMuscleConstraintData> Muscles;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TArray<FBoneKinematicState> BoneStates;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TArray<float> BoneMasses;

    UPROPERTY()
    TMap<int32, FMuscleIndexArray> BoneIndexToMuscleIndices;

    UFUNCTION(BlueprintCallable, Category = "MassMuscleProfile|Runtime State")
    void Initialize(USkeletalMesh* SkeletalMesh, const TArray<FMassMuscleDataMuscle>& MuscleData, const TArray<FMassMuscleDataMass>& MassData);

    UFUNCTION(BlueprintCallable, Category = "MassMuscleProfile|Runtime State", meta = (DefaultToSelf = "Outer"))
    static UMassMuscleRuntimeState* CreateInitializedRuntimeState(UObject* Outer, USkeletalMesh* SkeletalMesh, const TArray<FMassMuscleDataMuscle>& MuscleData, const TArray<FMassMuscleDataMass>& MassData);

    UFUNCTION(BlueprintCallable, BlueprintPure = false, Category = "MassMuscleProfile|Runtime State")
    FString DebugDumpRuntimeState(bool bPrintToLog = true) const;

    UFUNCTION(BlueprintCallable, BlueprintPure = false, Category = "MassMuscleProfile|Runtime State")
    FString DebugDumpRuntimeStateForBones(const TArray<int32>& BoneIndices, bool bPrintToLog = true) const;
};