#include "AgentSolverEditor.h"
#include "LearningProgramAssetTypeActions.h"
#include "LearningProgramGraphNode.h"
#include "SGraphNode_LearningProgram.h"
#include "AssetToolsModule.h"
#include "IAssetTools.h"
#include "EdGraphUtilities.h"

namespace
{
	class FLearningProgramGraphPanelNodeFactory : public FGraphPanelNodeFactory
	{
		virtual TSharedPtr<SGraphNode> CreateNode(UEdGraphNode* Node) const override
		{
			if (ULearningProgramGraphNode* LearningProgramNode = Cast<ULearningProgramGraphNode>(Node))
			{
				return SNew(SGraphNode_LearningProgram, LearningProgramNode);
			}
			return nullptr;
		}
	};
}

void FAgentSolverEditorModule::StartupModule()
{
	// Double-clicking a ULearningProgram asset opens the node editor instead
	// of the generic "no editor for this asset type" fallback -- see
	// LearningProgramAssetTypeActions.h. Same registration pattern
	// AgentSolver.cpp uses for FAgentSolverPresetAssetActions.
	IAssetTools& AssetTools = FModuleManager::LoadModuleChecked<FAssetToolsModule>("AssetTools").Get();
	LearningProgramAssetTypeActions = MakeShared<FLearningProgramAssetTypeActions>();
	AssetTools.RegisterAssetTypeActions(LearningProgramAssetTypeActions.ToSharedRef());

	LearningProgramNodeFactory = MakeShared<FLearningProgramGraphPanelNodeFactory>();
	FEdGraphUtilities::RegisterVisualNodeFactory(LearningProgramNodeFactory);
}

void FAgentSolverEditorModule::ShutdownModule()
{
	if (FModuleManager::Get().IsModuleLoaded("AssetTools"))
	{
		IAssetTools& AssetTools = FModuleManager::GetModuleChecked<FAssetToolsModule>("AssetTools").Get();
		AssetTools.UnregisterAssetTypeActions(LearningProgramAssetTypeActions.ToSharedRef());
	}

	FEdGraphUtilities::UnregisterVisualNodeFactory(LearningProgramNodeFactory);
}

IMPLEMENT_MODULE(FAgentSolverEditorModule, AgentSolverEditor)
