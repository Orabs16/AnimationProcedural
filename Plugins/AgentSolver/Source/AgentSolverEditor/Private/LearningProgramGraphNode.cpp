#include "LearningProgramGraphNode.h"
#include "AgentSolver/LearningProgram.h"
#include "EdGraph/EdGraph.h"
#include "EdGraph/EdGraphPin.h"
#include "Misc/DefaultValueHelper.h"

const FName LearningProgramPinCategory_Transition(TEXT("Transition"));

static const FName InputPinName(TEXT("In"));

static bool ParseTransitionIndex(const FName& PinName, int32& OutIndex)
{
	FString Name = PinName.ToString();
	if (!Name.StartsWith(TEXT("Transition_")))
	{
		return false;
	}
	return FDefaultValueHelper::ParseInt(Name.RightChop(11), OutIndex);
}

static FName MakeTransitionPinName(int32 Index)
{
	return FName(*FString::Printf(TEXT("Transition_%d"), Index));
}

FText ULearningProgramGraphNode::DescribeTransition(const FLearningProgramTransition& Transition)
{
	FString Description;
	switch (Transition.Condition)
	{
	case ELearningProgramConditionType::AverageRewardTarget:
		Description = FString::Printf(TEXT("Avg Reward >= %.2f"), Transition.ThresholdValue);
		break;
	case ELearningProgramConditionType::StepsSinceNodeEntry:
	default:
		Description = FString::Printf(TEXT("After %.0f Steps"), Transition.ThresholdValue);
		break;
	}
	if (!Transition.bEnabled)
	{
		Description += TEXT(" (Disabled)");
	}
	return FText::FromString(Description);
}

void ULearningProgramGraphNode::AllocateDefaultPins()
{
	CreatePin(EGPD_Input, LearningProgramPinCategory_Transition, InputPinName);

	if (RuntimeNode)
	{
		for (int32 Index = 0; Index < RuntimeNode->Transitions.Num(); ++Index)
		{
			UEdGraphPin* Pin = CreatePin(EGPD_Output, LearningProgramPinCategory_Transition, MakeTransitionPinName(Index));
			Pin->PinFriendlyName = DescribeTransition(RuntimeNode->Transitions[Index]);
		}
	}
}

FText ULearningProgramGraphNode::GetNodeTitle(ENodeTitleType::Type TitleType) const
{
	const FString Title = (RuntimeNode && !RuntimeNode->DisplayName.IsEmpty()) ? RuntimeNode->DisplayName : TEXT("Stage");
	return FText::FromString(Title);
}

FLinearColor ULearningProgramGraphNode::GetNodeTitleColor() const
{
	return FLinearColor(0.35f, 0.5f, 0.75f);
}

FText ULearningProgramGraphNode::GetTooltipText() const
{
	if (!RuntimeNode)
	{
		return FText::FromString(TEXT("Invalid Learning Program node."));
	}
	return FText::FromString(FString::Printf(TEXT("%s\n%d transition(s)."), *RuntimeNode->DisplayName, RuntimeNode->Transitions.Num()));
}

void ULearningProgramGraphNode::PrepareForCopying()
{
	// Copy/paste across Learning Program nodes is not supported -- each
	// RuntimeNode is a real, individually-owned subobject of the ULearningProgram
	// asset (not a template to be duplicated wholesale), and there is no
	// compile step to reconcile a pasted node's identity against the asset's
	// Nodes array. Base implementation is a no-op already; overridden here
	// only so this decision is explicit rather than accidental.
	Super::PrepareForCopying();
}

void ULearningProgramGraphNode::PinConnectionListChanged(UEdGraphPin* Pin)
{
	Super::PinConnectionListChanged(Pin);

	if (!RuntimeNode || !Pin || Pin->Direction != EGPD_Output)
	{
		return;
	}

	int32 TransitionIndex = INDEX_NONE;
	if (!ParseTransitionIndex(Pin->PinName, TransitionIndex) || !RuntimeNode->Transitions.IsValidIndex(TransitionIndex))
	{
		return;
	}

	RuntimeNode->Modify();
	FLearningProgramTransition& Transition = RuntimeNode->Transitions[TransitionIndex];

	if (Pin->LinkedTo.Num() == 0)
	{
		Transition.TargetNodeId.Invalidate();
		return;
	}

	// Output pins only ever carry ONE outgoing connection (see
	// ULearningProgramEdGraphSchema::CreateConnection) -- LinkedTo[0] is
	// always the whole story.
	if (const ULearningProgramGraphNode* TargetGraphNode = Cast<ULearningProgramGraphNode>(Pin->LinkedTo[0]->GetOwningNode()))
	{
		Transition.TargetNodeId = TargetGraphNode->RuntimeNode ? TargetGraphNode->RuntimeNode->NodeId : FGuid();
	}
}

UEdGraphPin* ULearningProgramGraphNode::GetInputPin() const
{
	for (UEdGraphPin* Pin : Pins)
	{
		if (Pin->Direction == EGPD_Input)
		{
			return Pin;
		}
	}
	return nullptr;
}

UEdGraphPin* ULearningProgramGraphNode::GetOutputPin(int32 TransitionIndex) const
{
	const FName WantedName = MakeTransitionPinName(TransitionIndex);
	for (UEdGraphPin* Pin : Pins)
	{
		if (Pin->Direction == EGPD_Output && Pin->PinName == WantedName)
		{
			return Pin;
		}
	}
	return nullptr;
}

void ULearningProgramGraphNode::AddTransitionPin()
{
	if (!RuntimeNode)
	{
		return;
	}

	Modify();
	RuntimeNode->Modify();

	const int32 NewIndex = RuntimeNode->Transitions.Add(FLearningProgramTransition());
	UEdGraphPin* NewPin = CreatePin(EGPD_Output, LearningProgramPinCategory_Transition, MakeTransitionPinName(NewIndex));
	NewPin->PinFriendlyName = DescribeTransition(RuntimeNode->Transitions[NewIndex]);

	GetGraph()->NotifyGraphChanged();
}

void ULearningProgramGraphNode::RemoveTransitionPin(UEdGraphPin* OutputPin)
{
	if (!RuntimeNode || !OutputPin || OutputPin->Direction != EGPD_Output)
	{
		return;
	}

	int32 RemovedIndex = INDEX_NONE;
	if (!ParseTransitionIndex(OutputPin->PinName, RemovedIndex) || !RuntimeNode->Transitions.IsValidIndex(RemovedIndex))
	{
		return;
	}

	Modify();
	RuntimeNode->Modify();

	RuntimeNode->Transitions.RemoveAt(RemovedIndex);
	OutputPin->BreakAllPinLinks();
	Pins.Remove(OutputPin);
	OutputPin->MarkAsGarbage();

	// Close the gap: every output pin after the removed one shifts down by
	// one index, both in name (so GetOutputPin/PinConnectionListChanged's
	// index parsing keeps matching Transitions' new layout) and in its
	// friendly-name summary (Transitions has already shifted under it).
	const int32 OldCount = RuntimeNode->Transitions.Num() + 1; // +1: array already shrunk by RemoveAt above
	for (int32 OldIndex = RemovedIndex + 1; OldIndex < OldCount; ++OldIndex)
	{
		if (UEdGraphPin* ShiftedPin = GetOutputPin(OldIndex))
		{
			const int32 NewIndex = OldIndex - 1;
			ShiftedPin->PinName = MakeTransitionPinName(NewIndex);
			ShiftedPin->PinFriendlyName = DescribeTransition(RuntimeNode->Transitions[NewIndex]);
		}
	}

	GetGraph()->NotifyGraphChanged();
}

void ULearningProgramGraphNode::RefreshTransitionPinNames()
{
	if (!RuntimeNode)
	{
		return;
	}
	for (int32 Index = 0; Index < RuntimeNode->Transitions.Num(); ++Index)
	{
		if (UEdGraphPin* Pin = GetOutputPin(Index))
		{
			Pin->PinFriendlyName = DescribeTransition(RuntimeNode->Transitions[Index]);
		}
	}
	if (UEdGraph* Graph = GetGraph())
	{
		Graph->NotifyGraphChanged();
	}
}
