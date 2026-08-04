#pragma once

#include "CoreMinimal.h"
#include "Units/Highlevel/RigUnit_HighlevelBase.h"
#include "FRigUnit_MuscleDebugBoneRotations.generated.h"

USTRUCT(meta = (DisplayName = "Muscle Debug Bone Rotations V2", Category = "MassMuscle", Keywords = "Muscle,Debug,Rotation,Bone,V2"))
struct MASSMUSCLEPROFILERUNTIME_API FRigUnit_MuscleDebugBoneRotationsV2 : public FRigUnit_HighlevelBaseMutable
{
    GENERATED_BODY()

    RIGVM_METHOD()
    virtual void Execute() override;

    UPROPERTY(meta = (Input))
    TArray<FRigElementKey> Bones;

    UPROPERTY(meta = (Input))
    bool bPrintToLog = false;

    UPROPERTY(meta = (Output))
    int32 DebuggedBoneCount = 0;
};
