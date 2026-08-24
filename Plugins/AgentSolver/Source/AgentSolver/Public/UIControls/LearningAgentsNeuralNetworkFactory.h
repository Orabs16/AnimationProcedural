#pragma once

// Editor-only UFactory for ULearningAgentsNeuralNetwork -- the Learning
// Agents plugin ships no factory/asset-type-actions of its own (confirmed by
// reading its module list: Learning/LearningAgents/LearningAgentsTraining/
// LearningTraining, no editor-only module among them), so there was
// previously NO way to create this asset type at all -- not via Content
// Browser's Add/Import menu, and not via the "create new" (+) button on an
// object-reference property row in the Details panel (that button needs an
// actual UFactory instance to call into; with none registered, the button
// simply doesn't render for this class, which is what SAgentSolverControlPanel
// users were hitting on the Learning category's 6 Load*/Save*NetworkAsset
// fields). Registering this factory fixes BOTH: Content Browser gets it too
// (any UFactory with bCreateNew=true and a SupportedClass gets picked up
// there automatically), matching how every other AgentSolver.Build.cs
// Editor-only asset-adjacent piece already works.
//
// Whole file is WITH_EDITOR-gated, same reasoning as FAgentSolverViewportClient.h
// -- UFactory itself lives in UnrealEd, an Editor-only module (see
// AgentSolver.Build.cs), so this can't compile at all outside Editor targets.

#include "CoreMinimal.h"

#if WITH_EDITOR

#include "Factories/Factory.h"

#include "LearningAgentsNeuralNetworkFactory.generated.h"

UCLASS()
class ULearningAgentsNeuralNetworkFactory : public UFactory
{
	GENERATED_BODY()

public:
	ULearningAgentsNeuralNetworkFactory();

	virtual UObject* FactoryCreateNew(UClass* Class, UObject* InParent, FName Name, EObjectFlags Flags, UObject* Context, FFeedbackContext* Warn) override;
};

#endif // WITH_EDITOR
