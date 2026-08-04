#pragma once
#include "CoreMinimal.h"
#include "Curves/CurveFloat.h"
#include "FMassMuscleData.generated.h"

UENUM(BlueprintType)
enum class ERotationType : uint8
{
    Yaw,
    Roll,
    Pitch,
};

USTRUCT(BlueprintType)
struct FMassMuscleDataMuscle
{
    GENERATED_BODY()
    FMassMuscleDataMuscle(
        FName InName = NAME_None,
        FName InBoneName = NAME_None,
        FName InChildBoneName = NAME_None,
        ERotationType InOrientation = ERotationType::Yaw,
        //FString InOrientation = "Yaw",
        float InMinRange = 0.f,
        float InMaxRange = 90.f,
        float InStrength = 1.f)
        : Name(InName)
        , BoneName(InBoneName)
        , ChildBoneName(InChildBoneName)
        , Orientation(InOrientation)
        , MinRange(InMinRange)
        , MaxRange(InMaxRange)
        , Strength(InStrength)
    {}

    // Identifies the muscle in your tool
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Muscle")
    FName Name = NAME_None;

    // Which bone this muscle is attached to
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Muscle")
    FName BoneName = NAME_None;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Muscle")
    FName ChildBoneName = NAME_None;

     //Direction the muscle pulls in bone-local space
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Muscle")
    ERotationType Orientation = ERotationType::Yaw;
    //UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Muscle")
    //FString Orientation = "Yaw";

    // Angle range the muscle is active within (degrees)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Muscle")
    float MinRange = 0.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Muscle")
    float MaxRange = 90.f;

    // How strongly this muscle pulls [0..1]
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Muscle", meta = (ClampMin = "0.0"))
    float Strength = 1.f;
};



USTRUCT(BlueprintType)
struct FMassMuscleDataMass
{
    GENERATED_BODY()
    FMassMuscleDataMass(
        FName InBoneName = NAME_None,
        float InMass = 1.f)
        : BoneName(InBoneName)
        , Mass(InMass)
    {}

    
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mass")
    FName BoneName = NAME_None;

    
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mass")
    int32 BoneIndex = INDEX_NONE;

    
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mass", meta = (ClampMin = "0.0"))
    float Mass = 1.f;
};

USTRUCT(BlueprintType)
struct FMuscleConstraintData
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    FName MuscleName;

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    FName BoneName;

    // Resolved once at init, avoids FName lookups in the hot solve loop
    UPROPERTY(Transient)
    int32 BoneIndex = INDEX_NONE;

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    FVector RotationAxis = FVector::ZeroVector;

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    float MinAngle = 0.f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    float MaxAngle = 0.f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    float BaseStrength = 1.f;

    // Reserved for Phase 4 — don't wire these up yet, just leave the fields
    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    FRuntimeFloatCurve StrengthByAngleCurve;

    UPROPERTY(EditAnywhere, BlueprintReadOnly)
    FRuntimeFloatCurve ExplosivityCurve;
};

USTRUCT(BlueprintType)
struct FBoneKinematicState
{
    GENERATED_BODY()

    UPROPERTY(Transient) int32 BoneIndex = INDEX_NONE;
    UPROPERTY(Transient) FTransform CurrentTransform;
    UPROPERTY(Transient) FQuat PreviousRotation = FQuat::Identity;
    UPROPERTY(Transient) FVector AngularVelocity = FVector::ZeroVector;
};

// Map value type can't be a bare TArray<int32> as a UPROPERTY, wrap it
USTRUCT()
struct FMuscleIndexArray
{
    GENERATED_BODY()
    UPROPERTY() TArray<int32> Indices;
};