#include "LearningProgramEntryGraphNode.h"
#include "LearningProgramGraphNode.h"
#include "AgentSolver/LearningProgram.h"
#include "EdGraph/EdGraph.h"
#include "EdGraph/EdGraphPin.h"

static const FName EntryOutputPinName(TEXT("Out"));

void ULearningProgramEntryGraphNode::AllocateDefaultPins()
{
	CreatePin(EGPD_Output, LearningProgramPinCategory_Transition, EntryOutputPinName);
}

FText ULearningProgramEntryGraphNode::GetNodeTitle(ENodeTitleType::Type TitleType) const
{
	return NSLOCTEXT("LearningProgramEntryGraphNode", "Title", "Start");
}

FLinearColor ULearningProgramEntryGraphNode::GetNodeTitleColor() const
{
	return FLinearColor(0.2f, 0.7f, 0.2f);
}

FText ULearningProgramEntryGraphNode::GetTooltipText() const
{
	return NSLOCTEXT("LearningProgramEntryGraphNode", "Tooltip", "Training begins at whichever stage this is connected to.");
}

UEdGraphPin* ULearningProgramEntryGraphNode::GetOutputPin() const
{
	return Pins.Num() > 0 ? Pins[0] : nullptr;
}

void ULearningProgramEntryGraphNode::PinConnectionListChanged(UEdGraphPin* Pin)
{
	Super::PinConnectionListChanged(Pin);

	ULearningProgram* Program = Cast<ULearningProgram>(GetGraph()->GetOuter());
	if (!Program || !Pin)
	{
		return;
	}

	Program->Modify();

	if (Pin->LinkedTo.Num() == 0)
	{
		Program->EntryNodeId.Invalidate();
		return;
	}

	// The schema's single-outgoing-link rule (CONNECT_RESPONSE_BREAK_OTHERS_*)
	// applies here too, so LinkedTo[0] is always the whole story.
	if (const ULearningProgramGraphNode* TargetNode = Cast<ULearningProgramGraphNode>(Pin->LinkedTo[0]->GetOwningNode()))
	{
		Program->EntryNodeId = TargetNode->RuntimeNode ? TargetNode->RuntimeNode->NodeId : FGuid();
	}
}
