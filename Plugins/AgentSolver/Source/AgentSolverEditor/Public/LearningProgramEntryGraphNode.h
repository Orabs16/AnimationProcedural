#pragma once

// The single, undeletable "Start" pseudo-node every Learning Program graph
// gets (see ULearningProgramFactory). OUTPUT-ONLY -- it has no "In" pin,
// because nothing ever transitions INTO the start of training; it exists
// purely to say which ULearningProgramGraphNode training begins at, by
// wiring its one "Out" pin to that node's "In" pin. Whichever stage it is
// currently connected to becomes ULearningProgram::EntryNodeId, kept live in
// sync by PinConnectionListChanged -- there is no separate "Set As Entry"
// action on regular stage nodes; dragging the connection IS how you set it.

#include "CoreMinimal.h"
#include "EdGraph/EdGraphNode.h"
#include "LearningProgramEntryGraphNode.generated.h"

UCLASS()
class ULearningProgramEntryGraphNode : public UEdGraphNode
{
	GENERATED_BODY()

public:
	//~ Begin UEdGraphNode interface
	virtual void AllocateDefaultPins() override;
	virtual FText GetNodeTitle(ENodeTitleType::Type TitleType) const override;
	virtual FLinearColor GetNodeTitleColor() const override;
	virtual FText GetTooltipText() const override;
	virtual void PinConnectionListChanged(UEdGraphPin* Pin) override;
	virtual bool CanUserDeleteNode() const override { return false; }
	virtual bool CanDuplicateNode() const override { return false; }
	//~ End UEdGraphNode interface

	UEdGraphPin* GetOutputPin() const;
};
