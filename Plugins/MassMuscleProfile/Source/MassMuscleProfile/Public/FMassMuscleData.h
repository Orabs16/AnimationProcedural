#pragma once

#include "CoreMinimal.h"
#include "Curves/CurveFloat.h"
#include "FMassMuscleData.generated.h"

UENUM(BlueprintType)
enum class ERotationType : uint8
{
    Yaw,
    Pitch,
    Roll
};

USTRUCT(BlueprintType)
struct FMassMuscleDataMuscle
{
    GENERATED_BODY()

public:


    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Muscle")
    FName Name = NAME_None;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Muscle")
    FName BoneName = NAME_None;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Muscle")
    FName ChildBoneName = NAME_None;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Muscle")
    ERotationType Orientation = ERotationType::Yaw;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Muscle")
    float MinRange = 0.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Muscle")
    float MaxRange = 0.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Muscle")
    float ExtensionStrength = 1.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Muscle")
    float FlexionStrength = 1.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Muscle")
    FRuntimeFloatCurve ExtensionStrengthCurve;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Muscle")
    FRuntimeFloatCurve FlexionStrengthCurve;

    // Minimum |normalized action| (policy command, [-1,1]) required for this
    // muscle to produce any torque at all -- below this, the muscle is
    // treated as not activated and contributes zero torque, same idea as a
    // biological motor-unit recruitment threshold. Applied in
    // CreatureRLEnvironment::ApplyActions, the single choke point where a
    // policy's normalized action becomes Batch.JointTorque.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Muscle")
    float MuscleActivationThreshold = 0.2f;
};

USTRUCT(BlueprintType)
struct FMassMuscleDataMass
{
    GENERATED_BODY()

public:

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mass")
    FName BoneName = NAME_None;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mass")
    int32 BoneIndex = INDEX_NONE;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mass")
    bool CanTouchGround = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mass")
    float Mass = 0.0f;

    // Collision geometry for this bone, authored manually like a lightweight
    // physics asset (no real PhysicsAsset/Chaos data is readable from
    // AgentSolver — see AgentSolver.Build.cs's AVX2 note): a CAPSULE running
    // from this bone's own joint toward its child/tip, uniform Radius at
    // both ends. Consumed by CreatureGroundContact.h, which evaluates BOTH
    // capsule end-caps against the ground (not just the tip) — replaces the
    // old "compute a correction that forces every limb to align" heuristic
    // with real per-bone geometry. Placeholder default (15.0) — tune per
    // bone to roughly match Muto's actual mesh thickness.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mass")
    float Radius = 15.0f;

    // Pulls the capsule's START cap back from the tip, along the bone->tip
    // axis, by 2x this distance (i.e. this is a true half-height: the start
    // cap sits CapsuleHalfHeight*2 away from the fixed end cap, which always
    // sits exactly at the tip). Default 0 degenerates to the original
    // single-sphere-at-the-tip behavior; increasing it grows the capsule
    // back up the limb — e.g. ~half the bone's own rest length makes it span
    // the whole segment (bone origin to tip).
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mass")
    float CapsuleHalfHeight = 0.0f;
};

bool EnsureDefaultStrengthCurvesInitialized(FMassMuscleDataMuscle& Muscle);