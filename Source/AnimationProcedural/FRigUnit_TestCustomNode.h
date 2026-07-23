#pragma once

#include "Units/RigUnit.h"
#include "FRigUnit_TestCustomNode.generated.h"

USTRUCT(meta = (DisplayName = "My Custom Rig Node", Category = "Custom"))
struct ANIMATIONPROCEDURAL_API FRigUnit_TestCustomNode : public FRigUnitMutable
{
    GENERATED_BODY()

    RIGVM_METHOD()
        virtual void Execute() override;

    FRigUnit_TestCustomNode()
    {
        Value = 0.f;
        Ratio = 1.f;
        Output = 0.f;
    }

    // Input pin
    UPROPERTY(meta = (Input))
    int Value;
    UPROPERTY(meta = (Input))
    int Ratio;

    // Output pin
    UPROPERTY(meta = (Output))
    int Output;

};
