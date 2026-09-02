#pragma once

// Connection/creation rules for the Learning Program graph: exactly one node
// type (ULearningProgramGraphNode), one input pin per node, N labeled output
// pins per node (one per transition, see ULearningProgramGraphNode), each
// output pin carries at most one outgoing link (a transition goes to exactly
// one target), and an input pin can receive links from any number of
// transitions.

#include "CoreMinimal.h"
#include "EdGraph/EdGraphSchema.h"
#include "LearningProgramEdGraphSchema.generated.h"

/**
 * Right-click-on-empty-canvas action that spawns a new ULearningProgramGraphNode
 * (and its backing ULearningProgramNode, appended to the owning
 * ULearningProgram::Nodes).
 *
 * Plain struct, NOT a USTRUCT -- FEdGraphSchemaAction predates/sits outside
 * UHT reflection and uses its own lightweight GetTypeId() RTTI instead (same
 * pattern every engine FEdGraphSchemaAction_* subclass follows).
 */
struct FLearningProgramSchemaAction_NewStage : public FEdGraphSchemaAction
{
	FLearningProgramSchemaAction_NewStage() : FEdGraphSchemaAction() {}
	FLearningProgramSchemaAction_NewStage(FText InNodeCategory, FText InMenuDesc, FText InToolTip, int32 InGrouping)
		: FEdGraphSchemaAction(MoveTemp(InNodeCategory), MoveTemp(InMenuDesc), MoveTemp(InToolTip), InGrouping)
	{
	}

	static FName StaticGetTypeId() { static FName Type(TEXT("FLearningProgramSchemaAction_NewStage")); return Type; }
	virtual FName GetTypeId() const override { return StaticGetTypeId(); }

	virtual UEdGraphNode* PerformAction(UEdGraph* ParentGraph, UEdGraphPin* FromPin, const FVector2D Location, bool bSelectNewNode = true) override;
};

UCLASS()
class ULearningProgramEdGraphSchema : public UEdGraphSchema
{
	GENERATED_BODY()

public:
	//~ Begin UEdGraphSchema interface
	virtual void GetGraphContextActions(FGraphContextMenuBuilder& ContextMenuBuilder) const override;
	virtual void GetContextMenuActions(class UToolMenu* Menu, class UGraphNodeContextMenuContext* Context) const override;
	/** Enforces: opposite-direction pins only, matching category, no self-loop, and (via CONNECT_RESPONSE_BREAK_OTHERS_*) at most one outgoing link per output pin -- a transition goes to exactly one target, so making a new connection from an already-linked output pin replaces its old target instead of adding a second. */
	virtual const FPinConnectionResponse CanCreateConnection(const UEdGraphPin* PinA, const UEdGraphPin* PinB) const override;
	virtual FLinearColor GetPinTypeColor(const FEdGraphPinType& PinType) const override;
	//~ End UEdGraphSchema interface

	/** Creates one new ULearningProgramNode (appended to the owning ULearningProgram) and its wrapping ULearningProgramGraphNode at Location. Shared by FLearningProgramSchemaAction_NewStage::PerformAction and ULearningProgramFactory (for the initial entry node a brand new asset gets). */
	static class ULearningProgramGraphNode* SpawnStageNode(UEdGraph* Graph, const FVector2D& Location, bool bSelectNewNode = true);
};
