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

private:
    static void InitializeCurveIfMissing(FRuntimeFloatCurve& Curve)
    {
        const bool bHasExternalCurve = Curve.ExternalCurve != nullptr;
        const FRichCurve* RichCurve = Curve.GetRichCurveConst();
        const bool bHasInlineKeys = RichCurve && RichCurve->GetNumKeys() > 0;

        if (bHasExternalCurve || bHasInlineKeys)
        {
            return;
        }

        Curve.EditorCurveData.Reset();
        Curve.EditorCurveData.AddKey(0.0f, 1.0f);
        Curve.EditorCurveData.AddKey(1.0f, 1.0f);
    }

public:

    bool EnsureDefaultStrengthCurvesInitialized()
    {
        const FRichCurve* ExtensionBefore = ExtensionStrength.GetRichCurveConst();
        const bool bExtensionHadKeys = ExtensionStrength.ExternalCurve != nullptr || (ExtensionBefore && ExtensionBefore->GetNumKeys() > 0);

        const FRichCurve* FlexionBefore = FlexionStrength.GetRichCurveConst();
        const bool bFlexionHadKeys = FlexionStrength.ExternalCurve != nullptr || (FlexionBefore && FlexionBefore->GetNumKeys() > 0);

        InitializeCurveIfMissing(ExtensionStrength);
        InitializeCurveIfMissing(FlexionStrength);

        return !bExtensionHadKeys || !bFlexionHadKeys;
    }


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
    float Strength = 0.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Muscle")
    FRuntimeFloatCurve ExtensionStrength;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Muscle")
    FRuntimeFloatCurve FlexionStrength;

    FMassMuscleDataMuscle()
    {
        EnsureDefaultStrengthCurvesInitialized();
    }

    FMassMuscleDataMuscle(
        FName InName,
        FName InBoneName,
        FName InChildBoneName,
        ERotationType InOrientation,
        float InMinRange,
        float InMaxRange)
        : Name(InName)
        , BoneName(InBoneName)
        , ChildBoneName(InChildBoneName)
        , Orientation(InOrientation)
        , MinRange(InMinRange)
        , MaxRange(InMaxRange)
        , Strength(1.0f)
    {
        EnsureDefaultStrengthCurvesInitialized();
    }

    FMassMuscleDataMuscle(
        FName InName,
        FName InBoneName,
        FName InChildBoneName,
        ERotationType InOrientation,
        float InMinRange,
        float InMaxRange,
        float InStrength)
        : Name(InName)
        , BoneName(InBoneName)
        , ChildBoneName(InChildBoneName)
        , Orientation(InOrientation)
        , MinRange(InMinRange)
        , MaxRange(InMaxRange)
        , Strength(InStrength)
    {
        EnsureDefaultStrengthCurvesInitialized();
    }
};

USTRUCT(BlueprintType)
struct FMassMuscleDataMass
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mass")
    FName BoneName = NAME_None;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mass")
    int32 BoneIndex = INDEX_NONE;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mass")
    float Mass = 0.0f;
};