#pragma once

#include "Units/Highlevel/RigUnit_HighlevelBase.h"
#include "Rigs/RigHierarchyCache.h"
#include "UMassMuscleRuntimeState.h"
#include "FRigUnit_MuscleCCDIK.generated.h"

UENUM()
enum class EMuscleCCDIKEffectorSpace : uint8
{
    Global,
    RootParentLocal,
    RootLocal
};

USTRUCT(meta=(DisplayName="Muscle CCDIK Custom", Category="MassMuscle", Keywords="Muscle,CCDIK,IK,Custom"))
struct FRigUnit_MuscleCCDIK : public FRigUnit_HighlevelBaseMutable
{
    GENERATED_BODY()

    RIGVM_METHOD()
    virtual void Execute() override;

    UPROPERTY(meta = (Input))
    FRigElementKey RootBone;

    UPROPERTY(meta = (Input))
    FRigElementKey TipBone;

    UPROPERTY(meta = (Input))
    FVector EffectorTarget = FVector::ZeroVector;

    UPROPERTY(meta = (Input))
    EMuscleCCDIKEffectorSpace EffectorSpace = EMuscleCCDIKEffectorSpace::Global;

    UPROPERTY(meta = (Input))
    UMassMuscleRuntimeState* MuscleData = nullptr;

    UPROPERTY(meta = (Input))
    int32 MaxIterations = 10;

    UPROPERTY(meta = (Input))
    float Precision = 0.1f;

    UPROPERTY(meta = (Input))
    bool bPreserveTipLocalRotation = true;

    UPROPERTY(meta = (Input))
    bool bPreserveTipInitialLocalRotation = true;

    UPROPERTY(meta = (Output, DisplayName = "Reached Effector"))
    bool bReachedEffector = false;

    UPROPERTY(meta = (Output))
    TArray<int32> AffectedBoneIndices;

    // Resolved once, cached across frames - the standard Control Rig pattern
    // for avoiding a hierarchy walk every tick (see FCachedRigElement)
    UPROPERTY()
    TArray<FCachedRigElement> CachedChain;
};