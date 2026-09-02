#pragma once

// Double-click (or right-click -> Edit) on a ULearningProgram asset opens
// FLearningProgramAssetEditorToolkit's node graph, instead of the generic
// "no editor registered for this asset type" fallback. Same pattern as
// FAgentSolverPresetAssetActions in the runtime module, just constructing a
// real FAssetEditorToolkit instead of pushing into a nomad-tab widget.

#include "CoreMinimal.h"
#include "AssetTypeActions_Base.h"
#include "AgentSolver/LearningProgram.h"

class FLearningProgramAssetTypeActions : public FAssetTypeActions_Base
{
public:
	virtual FText GetName() const override
	{
		return NSLOCTEXT("AgentSolverEditor", "LearningProgramAssetActionsName", "Learning Program");
	}

	virtual FColor GetTypeColor() const override
	{
		return FColor(220, 160, 60);
	}

	virtual UClass* GetSupportedClass() const override
	{
		return ULearningProgram::StaticClass();
	}

	virtual uint32 GetCategories() override
	{
		return EAssetTypeCategories::Gameplay;
	}

	virtual void OpenAssetEditor(const TArray<UObject*>& InObjects, TSharedPtr<IToolkitHost> EditWithinLevelEditor = TSharedPtr<IToolkitHost>()) override;
};
