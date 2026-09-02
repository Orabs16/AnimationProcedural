#pragma once

// The editor-only visual wrapper around one ULearningProgramNode -- a thin
// UEdGraphNode whose pins mirror RuntimeNode->Transitions 1:1 (one "In" pin,
// one "Out" pin per transition). There is no separate compile step: pin
// links ARE the source of truth for FLearningProgramTransition::TargetNodeId,
// kept in sync live by PinConnectionListChanged as the user draws/breaks
// connections in the graph -- see that function's comment.

#include "CoreMinimal.h"
#include "EdGraph/EdGraphNode.h"
#include "LearningProgramGraphNode.generated.h"

class ULearningProgramNode;

/** Shared pin category for both the "In" pin and every "Transition_N" output pin -- CanCreateConnection just needs the two sides to match, there is no real type system here. */
extern const FName LearningProgramPinCategory_Transition;

UCLASS()
class ULearningProgramGraphNode : public UEdGraphNode
{
	GENERATED_BODY()

public:
	/** The runtime stage this node visualizes -- created and owned by the ULearningProgram asset (see ULearningProgramEdGraphSchema::SpawnStageNode), not by this graph node. */
	UPROPERTY()
	TObjectPtr<ULearningProgramNode> RuntimeNode;

	//~ Begin UEdGraphNode interface
	virtual void AllocateDefaultPins() override;
	virtual FText GetNodeTitle(ENodeTitleType::Type TitleType) const override;
	virtual FLinearColor GetNodeTitleColor() const override;
	virtual FText GetTooltipText() const override;
	virtual void PinConnectionListChanged(UEdGraphPin* Pin) override;
	virtual void PrepareForCopying() override;
	//~ End UEdGraphNode interface

	/** The single "In" pin every node has. */
	UEdGraphPin* GetInputPin() const;

	/** The "Out" pin for RuntimeNode->Transitions[TransitionIndex], or nullptr if out of range. */
	UEdGraphPin* GetOutputPin(int32 TransitionIndex) const;

	/** Appends a new default-constructed transition to RuntimeNode and a matching output pin -- the graph editor's "Add Transition" node action, and SpawnStageNode's default seed for a brand new node. */
	void AddTransitionPin();

	/** Removes the transition (and its pin) OutputPin corresponds to -- the graph editor's "Remove Transition" pin action. Renumbers the remaining output pins' names to stay contiguous with Transitions' new indices. */
	void RemoveTransitionPin(UEdGraphPin* OutputPin);

	/** Re-derives every output pin's friendly name from its current FLearningProgramTransition (bEnabled/Condition/ThresholdValue) -- called by the toolkit whenever the Details panel edits this node's Transitions, so pin labels don't go stale after editing values that aren't the pin count. */
	void RefreshTransitionPinNames();

	/** Short human-readable summary of one transition (e.g. "After 1000 Steps", "Avg Reward >= 50 (Disabled)"), used as the matching output pin's friendly name. */
	static FText DescribeTransition(const struct FLearningProgramTransition& Transition);
};
