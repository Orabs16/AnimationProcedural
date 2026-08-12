#include "FMassMuscleData.h"

namespace
{
    void InitializeMassMuscleCurveIfMissing(FRuntimeFloatCurve& Curve)
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
}

bool EnsureDefaultStrengthCurvesInitialized(FMassMuscleDataMuscle& Muscle)
{
    const FRichCurve* ExtensionBefore = Muscle.ExtensionStrengthCurve.GetRichCurveConst();
    const bool bExtensionHadKeys = Muscle.ExtensionStrengthCurve.ExternalCurve != nullptr || (ExtensionBefore && ExtensionBefore->GetNumKeys() > 0);

    const FRichCurve* FlexionBefore = Muscle.FlexionStrengthCurve.GetRichCurveConst();
    const bool bFlexionHadKeys = Muscle.FlexionStrengthCurve.ExternalCurve != nullptr || (FlexionBefore && FlexionBefore->GetNumKeys() > 0);

    InitializeMassMuscleCurveIfMissing(Muscle.ExtensionStrengthCurve);
    InitializeMassMuscleCurveIfMissing(Muscle.FlexionStrengthCurve);

    return !bExtensionHadKeys || !bFlexionHadKeys;
}