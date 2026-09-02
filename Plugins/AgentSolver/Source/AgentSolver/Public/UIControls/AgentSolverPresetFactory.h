#pragma once

// Editor-only UFactory for UAgentSolverPreset -- same reasoning as
// LearningAgentsNeuralNetworkFactory.h: without a registered UFactory,
// neither the Content Browser's Add menu nor a "create new" (+) button on an
// object-reference property row (e.g. UAgentSolverViewportSettings::
// ActivePreset) can create this asset type at all.
//
// Whole file is WITH_EDITOR-gated -- UFactory lives in UnrealEd, an
// Editor-only module (see AgentSolver.Build.cs), so this can't compile at
// all outside Editor targets.

#include "CoreMinimal.h"

#if WITH_EDITOR

#include "Factories/Factory.h"

#include "AgentSolverPresetFactory.generated.h"

UCLASS()
class UAgentSolverPresetFactory : public UFactory
{
	GENERATED_BODY()

public:
	UAgentSolverPresetFactory();

	virtual UObject* FactoryCreateNew(UClass* Class, UObject* InParent, FName Name, EObjectFlags Flags, UObject* Context, FFeedbackContext* Warn) override;
};

#endif // WITH_EDITOR
