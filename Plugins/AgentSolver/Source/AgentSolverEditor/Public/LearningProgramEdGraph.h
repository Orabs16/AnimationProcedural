#pragma once

// The concrete UEdGraph subclass ULearningProgram::EdGraph points to (as the
// base UEdGraph* -- see that property's comment). No behavior of its own;
// exists so ULearningProgramFactory can assign ULearningProgramEdGraphSchema
// as this graph's Schema class.

#include "CoreMinimal.h"
#include "EdGraph/EdGraph.h"
#include "LearningProgramEdGraph.generated.h"

UCLASS()
class ULearningProgramEdGraph : public UEdGraph
{
	GENERATED_BODY()
};
