#include "UIControls/AgentSolverPresetFactory.h"

#if WITH_EDITOR

#include "AgentSolver/AgentSolverPreset.h"

UAgentSolverPresetFactory::UAgentSolverPresetFactory()
{
	SupportedClass = UAgentSolverPreset::StaticClass();
	bCreateNew = true;
	bEditAfterNew = true;
}

UObject* UAgentSolverPresetFactory::FactoryCreateNew(UClass* Class, UObject* InParent, FName Name, EObjectFlags Flags, UObject* Context, FFeedbackContext* Warn)
{
	return NewObject<UAgentSolverPreset>(InParent, Class, Name, Flags);
}

#endif // WITH_EDITOR
