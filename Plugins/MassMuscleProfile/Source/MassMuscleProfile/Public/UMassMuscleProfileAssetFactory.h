#pragma once
#include "CoreMinimal.h"
#include "Factories/Factory.h"
#include "Templates/SharedPointer.h"
#include "UMassMuscleProfileAsset.h"

#include "UMassMuscleProfileAssetFactory.generated.h"

UCLASS()
class UMassMuscleProfileAssetFactoryMuscle : public UFactory
{
    GENERATED_BODY()

public:

    UMassMuscleProfileAssetFactoryMuscle()
    {
        SupportedClass = UMassMuscleProfileAssetMuscle::StaticClass();
        bCreateNew     = true;   // appears in the right-click create menu
        bEditAfterNew  = true;   // opens the asset editor right after creation
    }

    virtual UObject* FactoryCreateNew(
        UClass*          Class,
        UObject*         InParent,
        FName            Name,
        EObjectFlags     Flags,
        UObject*         Context,
        FFeedbackContext* Warn) override
    {
        return NewObject<UMassMuscleProfileAssetMuscle>(InParent, Class, Name, Flags);
    }

    virtual FText GetDisplayName() const override
    {
        return FText::FromString("Muscle Profile");
    }
};

UCLASS()
class UMassMuscleProfileAssetFactoryMass : public UFactory
{
    GENERATED_BODY()

public:

    UMassMuscleProfileAssetFactoryMass()
    {
        SupportedClass = UMassMuscleProfileAssetMass::StaticClass();
        bCreateNew     = true;   // appears in the right-click create menu
        bEditAfterNew  = true;   // opens the asset editor right after creation
    }

    virtual UObject* FactoryCreateNew(
        UClass*          Class,
        UObject*         InParent,
        FName            Name,
        EObjectFlags     Flags,
        UObject*         Context,
        FFeedbackContext* Warn) override
    {
        return NewObject<UMassMuscleProfileAssetMass>(InParent, Class, Name, Flags);
    }

    virtual FText GetDisplayName() const override
    {
        return FText::FromString("Mass Profile");
    }
};