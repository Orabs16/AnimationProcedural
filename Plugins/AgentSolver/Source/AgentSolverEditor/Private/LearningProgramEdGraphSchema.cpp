#include "LearningProgramEdGraphSchema.h"
#include "LearningProgramGraphNode.h"
#include "AgentSolver/LearningProgram.h"
#include "EdGraph/EdGraph.h"
#include "EdGraph/EdGraphPin.h"
#include "ToolMenus.h"

#define LOCTEXT_NAMESPACE "LearningProgramEdGraphSchema"

UEdGraphNode* FLearningProgramSchemaAction_NewStage::PerformAction(UEdGraph* ParentGraph, UEdGraphPin* FromPin, const FVector2D Location, bool bSelectNewNode)
{
	ULearningProgramGraphNode* NewNode = ULearningProgramEdGraphSchema::SpawnStageNode(ParentGraph, Location, bSelectNewNode);
	if (FromPin && NewNode)
	{
		const UEdGraphSchema* Schema = ParentGraph->GetSchema();
		UEdGraphPin* TargetPin = (FromPin->Direction == EGPD_Output) ? NewNode->GetInputPin() : nullptr;
		if (TargetPin)
		{
			Schema->TryCreateConnection(FromPin, TargetPin);
		}
	}
	return NewNode;
}

ULearningProgramGraphNode* ULearningProgramEdGraphSchema::SpawnStageNode(UEdGraph* Graph, const FVector2D& Location, bool bSelectNewNode)
{
	ULearningProgram* Program = CastChecked<ULearningProgram>(Graph->GetOuter());
	Program->Modify();

	ULearningProgramNode* NewRuntimeNode = NewObject<ULearningProgramNode>(Program, NAME_None, RF_Transactional);
	NewRuntimeNode->DisplayName = TEXT("New Stage");
	NewRuntimeNode->EditorPosition = Location;
	// Seed one default transition so a freshly created node already has an
	// output pin to drag a connection from -- discovering the "Add
	// Transition" right-click action is not the only way to get started.
	NewRuntimeNode->Transitions.Add(FLearningProgramTransition());
	Program->Nodes.Add(NewRuntimeNode);
	// Deliberately does NOT touch Program->EntryNodeId -- entry is decided
	// solely by what the graph's Start node (ULearningProgramEntryGraphNode)
	// is connected to, not by "first node created". See that class's comment.

	Graph->Modify();
	ULearningProgramGraphNode* GraphNode = NewObject<ULearningProgramGraphNode>(Graph, NAME_None, RF_Transactional);
	GraphNode->RuntimeNode = NewRuntimeNode;
	GraphNode->NodePosX = (int32)Location.X;
	GraphNode->NodePosY = (int32)Location.Y;
	GraphNode->CreateNewGuid();
	Graph->AddNode(GraphNode, /*bFromUI=*/true, bSelectNewNode);
	GraphNode->PostPlacedNewNode();
	GraphNode->AllocateDefaultPins();

	return GraphNode;
}

void ULearningProgramEdGraphSchema::GetGraphContextActions(FGraphContextMenuBuilder& ContextMenuBuilder) const
{
	TSharedPtr<FLearningProgramSchemaAction_NewStage> Action = MakeShared<FLearningProgramSchemaAction_NewStage>(
		FText::GetEmpty(), LOCTEXT("AddStage", "Add Stage"), LOCTEXT("AddStageTooltip", "Adds a new Learning Program stage."), 0);
	ContextMenuBuilder.AddAction(Action);
}

void ULearningProgramEdGraphSchema::GetContextMenuActions(UToolMenu* Menu, UGraphNodeContextMenuContext* Context) const
{
	if (Context && Context->Node)
	{
		if (ULearningProgramGraphNode* GraphNode = const_cast<ULearningProgramGraphNode*>(Cast<ULearningProgramGraphNode>(Context->Node)))
		{
			FToolMenuSection& Section = Menu->AddSection(TEXT("LearningProgramNodeActions"), LOCTEXT("NodeActionsHeader", "Learning Program"));

			Section.AddMenuEntry(
				TEXT("AddTransition"),
				LOCTEXT("AddTransition", "Add Transition"),
				LOCTEXT("AddTransitionTooltip", "Adds a new outgoing transition (and pin) to this stage."),
				FSlateIcon(),
				FUIAction(FExecuteAction::CreateUObject(GraphNode, &ULearningProgramGraphNode::AddTransitionPin)));

			if (Context->Pin && Context->Pin->Direction == EGPD_Output)
			{
				UEdGraphPin* PinPtr = const_cast<UEdGraphPin*>(Context->Pin);
				Section.AddMenuEntry(
					TEXT("RemoveTransition"),
					LOCTEXT("RemoveTransition", "Remove Transition"),
					LOCTEXT("RemoveTransitionTooltip", "Removes this transition and its pin."),
					FSlateIcon(),
					FUIAction(FExecuteAction::CreateUObject(GraphNode, &ULearningProgramGraphNode::RemoveTransitionPin, PinPtr)));
			}
		}
	}

	Super::GetContextMenuActions(Menu, Context);
}

const FPinConnectionResponse ULearningProgramEdGraphSchema::CanCreateConnection(const UEdGraphPin* PinA, const UEdGraphPin* PinB) const
{
	if (!PinA || !PinB)
	{
		return FPinConnectionResponse(CONNECT_RESPONSE_DISALLOW, LOCTEXT("InvalidPin", "Invalid pin").ToString());
	}
	if (PinA->GetOwningNode() == PinB->GetOwningNode())
	{
		return FPinConnectionResponse(CONNECT_RESPONSE_DISALLOW, LOCTEXT("SelfLoop", "A stage cannot transition to itself").ToString());
	}
	if (PinA->Direction == PinB->Direction)
	{
		return FPinConnectionResponse(CONNECT_RESPONSE_DISALLOW, LOCTEXT("DirectionMismatch", "Directions are not compatible").ToString());
	}
	if (PinA->PinType.PinCategory != PinB->PinType.PinCategory)
	{
		return FPinConnectionResponse(CONNECT_RESPONSE_DISALLOW, LOCTEXT("CategoryMismatch", "Pin types are not compatible").ToString());
	}

	// A transition goes to exactly one target -- making a new connection
	// from an already-linked output pin replaces its old target rather than
	// adding a second, via CONNECT_RESPONSE_BREAK_OTHERS_*.
	const bool bAIsOutput = PinA->Direction == EGPD_Output;
	return FPinConnectionResponse(bAIsOutput ? CONNECT_RESPONSE_BREAK_OTHERS_A : CONNECT_RESPONSE_BREAK_OTHERS_B,
		LOCTEXT("ReplaceTransition", "Replace existing transition target").ToString());
}

FLinearColor ULearningProgramEdGraphSchema::GetPinTypeColor(const FEdGraphPinType& PinType) const
{
	return FLinearColor::White;
}

#undef LOCTEXT_NAMESPACE
