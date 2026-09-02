#pragma once

// Editor-only UFactory for ULearningProgram -- same reasoning/shape as
// UAgentSolverPresetFactory in the runtime module (see that header's
// comment): without this, neither the Content Browser's Add menu nor a
// "create new" button on an object-reference property row can create this
// asset type.

#include "CoreMinimal.h"
#include "Factories/Factory.h"
#include "LearningProgramFactory.generated.h"

UCLASS()
class ULearningProgramFactory : public UFactory
{
	GENERATED_BODY()

public:
	ULearningProgramFactory();

	virtual UObject* FactoryCreateNew(UClass* Class, UObject* InParent, FName Name, EObjectFlags Flags, UObject* Context, FFeedbackContext* Warn) override;
};
