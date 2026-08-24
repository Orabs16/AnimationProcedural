#include "UIControls/AgentSolverUIUtils.h"

#if WITH_EDITOR

#include "AgentSolver/MutoRLTrainingDriver.h"
#include "AgentSolver/MutoRLVisualizer.h"
#include "PhysicsSolver/MutoRagdollVisualizer.h"
#include "Editor.h"
#include "EngineUtils.h"

namespace
{
	UWorld* FindRelevantWorld()
	{
		if (!GEditor)
		{
			return nullptr;
		}
		return GEditor->PlayWorld ? GEditor->PlayWorld.Get() : GEditor->GetEditorWorldContext().World();
	}
}

AMutoRLTrainingDriver* AgentSolverUI::FindTrainingDriver()
{
	UWorld* World = FindRelevantWorld();
	if (!World)
	{
		return nullptr;
	}

	for (TActorIterator<AMutoRLTrainingDriver> It(World); It; ++It)
	{
		// Exclude the two known C++ subclasses specifically -- NOT an exact
		// GetClass()==StaticClass() match, which would also (wrongly) reject
		// any Blueprint subclass of AMutoRLTrainingDriver placed in the level
		// (a very normal way to place this actor) and silently find nothing
		// at all, with everything downstream (toolbar actions) going quiet
		// with no visible error.
		if (!It->IsA<AMutoRagdollVisualizerActor>() && !It->IsA<AMutoRLVisualizerActor>())
		{
			return *It;
		}
	}
	return nullptr;
}

AMutoRLTrainingDriver* AgentSolverUI::FindViewportSourceActor(EAgentSolverViewportSource Source)
{
	switch (Source)
	{
	case EAgentSolverViewportSource::Ragdoll:
		return FindRagdollVisualizer();
	case EAgentSolverViewportSource::Visualizer:
	default:
		return FindRLVisualizer();
	}
}

AMutoRagdollVisualizerActor* AgentSolverUI::FindRagdollVisualizer()
{
	UWorld* World = FindRelevantWorld();
	if (!World)
	{
		return nullptr;
	}

	for (TActorIterator<AMutoRagdollVisualizerActor> It(World); It; ++It)
	{
		return *It;
	}
	return nullptr;
}

AMutoRLVisualizerActor* AgentSolverUI::FindRLVisualizer()
{
	UWorld* World = FindRelevantWorld();
	if (!World)
	{
		return nullptr;
	}

	for (TActorIterator<AMutoRLVisualizerActor> It(World); It; ++It)
	{
		return *It;
	}
	return nullptr;
}

bool AgentSolverUI::IsPIERunning()
{
	return GEditor && GEditor->PlayWorld != nullptr;
}

#else

AMutoRLTrainingDriver* AgentSolverUI::FindTrainingDriver() { return nullptr; }
AMutoRLTrainingDriver* AgentSolverUI::FindViewportSourceActor(EAgentSolverViewportSource) { return nullptr; }
AMutoRagdollVisualizerActor* AgentSolverUI::FindRagdollVisualizer() { return nullptr; }
AMutoRLVisualizerActor* AgentSolverUI::FindRLVisualizer() { return nullptr; }
bool AgentSolverUI::IsPIERunning() { return false; }

#endif // WITH_EDITOR
