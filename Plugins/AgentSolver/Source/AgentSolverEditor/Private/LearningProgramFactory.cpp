#include "LearningProgramFactory.h"
#include "AgentSolver/LearningProgram.h"
#include "LearningProgramEdGraph.h"
#include "LearningProgramEdGraphSchema.h"
#include "LearningProgramEntryGraphNode.h"
#include "LearningProgramGraphNode.h"
#include "EdGraph/EdGraphPin.h"

ULearningProgramFactory::ULearningProgramFactory()
{
	SupportedClass = ULearningProgram::StaticClass();
	bCreateNew = true;
	bEditAfterNew = true;
}

UObject* ULearningProgramFactory::FactoryCreateNew(UClass* Class, UObject* InParent, FName Name, EObjectFlags Flags, UObject* Context, FFeedbackContext* Warn)
{
	ULearningProgram* NewProgram = NewObject<ULearningProgram>(InParent, Class, Name, Flags);

	NewProgram->EdGraph = NewObject<ULearningProgramEdGraph>(NewProgram, NAME_None, RF_Transactional);
	NewProgram->EdGraph->Schema = ULearningProgramEdGraphSchema::StaticClass();

	// A program with no stages at all can't train anything -- seed the Start
	// pseudo-node plus a single stage, connected together, so a
	// freshly-created asset is immediately usable (StartTraining will apply
	// that one stage's Params) instead of opening to an empty canvas with no
	// entry point.
	ULearningProgramEntryGraphNode* EntryNode = NewObject<ULearningProgramEntryGraphNode>(NewProgram->EdGraph, NAME_None, RF_Transactional);
	EntryNode->NodePosX = -300;
	EntryNode->NodePosY = 0;
	EntryNode->CreateNewGuid();
	NewProgram->EdGraph->AddNode(EntryNode, /*bFromUI=*/false, /*bSelectNewNode=*/false);
	EntryNode->PostPlacedNewNode();
	EntryNode->AllocateDefaultPins();

	ULearningProgramGraphNode* StageNode = ULearningProgramEdGraphSchema::SpawnStageNode(NewProgram->EdGraph, FVector2D(0.0f, 0.0f), /*bSelectNewNode=*/false);

	NewProgram->EdGraph->GetSchema()->TryCreateConnection(EntryNode->GetOutputPin(), StageNode->GetInputPin());

	return NewProgram;
}
