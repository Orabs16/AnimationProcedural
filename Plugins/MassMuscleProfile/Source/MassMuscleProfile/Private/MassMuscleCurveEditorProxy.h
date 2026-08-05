#pragma once

#include "CoreMinimal.h"
#include "UObject/Object.h"
#include "FMassMuscleData.h"
#include "MassMuscleCurveEditorProxy.generated.h"

UCLASS()
class UMassMuscleCurveEditorProxy : public UObject
{
    GENERATED_BODY()

public:
    UPROPERTY(EditAnywhere, Category = "Muscle")
    FRuntimeFloatCurve ExtensionStrength;

    UPROPERTY(EditAnywhere, Category = "Muscle")
    FRuntimeFloatCurve FlexionStrength;
};