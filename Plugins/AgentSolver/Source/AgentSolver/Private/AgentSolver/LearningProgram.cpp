#include "AgentSolver/LearningProgram.h"

ULearningProgramNode::ULearningProgramNode()
{
	NodeId = FGuid::NewGuid();
	Params = CreateDefaultSubobject<UAgentSolverPreset>(TEXT("Params"));
}

ULearningProgramNode* ULearningProgram::FindNode(const FGuid& NodeId) const
{
	for (const TObjectPtr<ULearningProgramNode>& Node : Nodes)
	{
		if (Node && Node->NodeId == NodeId)
		{
			return Node;
		}
	}
	return nullptr;
}

ULearningProgramNode* ULearningProgram::GetEntryNode() const
{
	return FindNode(EntryNodeId);
}
