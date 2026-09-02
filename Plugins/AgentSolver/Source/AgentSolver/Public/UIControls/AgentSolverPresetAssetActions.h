#pragma once

// Double-click (or right-click -> Edit) on a UAgentSolverPreset asset opens
// the Agent Solver tool with it loaded, instead of the generic "no editor
// registered for this asset type" fallback. Exact same pattern
// MassMuscleProfile uses for its own two asset types (see
// FMassMuscleProfileAssetActionsMuscle/Mass in
// FMassMuscleProfileAssetActions.h) -- OpenAssetEditor calls into the
// owning module instead of spawning a generic asset editor.

#include "CoreMinimal.h"

#if WITH_EDITOR

#include "AssetTypeActions_Base.h"
#include "AgentSolver.h"
#include "AgentSolver/AgentSolverPreset.h"

class FAgentSolverPresetAssetActions : public FAssetTypeActions_Base
{
public:
	virtual FText GetName() const override
	{
		return NSLOCTEXT("AgentSolver", "PresetAssetActionsName", "Agent Solver Preset");
	}

	virtual FColor GetTypeColor() const override
	{
		return FColor(80, 180, 220);
	}

	virtual UClass* GetSupportedClass() const override
	{
		return UAgentSolverPreset::StaticClass();
	}

	virtual uint32 GetCategories() override
	{
		return EAssetTypeCategories::Gameplay;
	}

	virtual void OpenAssetEditor(const TArray<UObject*>& InObjects, TSharedPtr<IToolkitHost> EditWithinLevelEditor = TSharedPtr<IToolkitHost>()) override
	{
		for (UObject* Obj : InObjects)
		{
			if (UAgentSolverPreset* Preset = Cast<UAgentSolverPreset>(Obj))
			{
				FAgentSolverModule::Get().OpenToolForPreset(Preset);
			}
		}
	}
};

#endif // WITH_EDITOR
