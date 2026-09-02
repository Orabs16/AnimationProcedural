#pragma once

// Shared helper for every AgentSolver editor UI piece (control panel,
// embedded viewport client) that needs to find the AMutoRLTrainingDriver/
// Ragdoll/Visualizer actor relevant right now -- factored out so the
// PIE-vs-editor-world lookup rule lives in exactly one place instead of
// being copy-pasted per widget.

#include "UIControls/AgentSolverViewportSettings.h"

class AMutoRLTrainingDriver;
class AMutoRagdollVisualizerActor;
class AMutoRLVisualizerActor;

namespace AgentSolverUI
{
	/**
	 * The one AMutoRLTrainingDriver in the relevant world: PIE's world while
	 * playing (that's the instance StartTraining actually ran on -- see
	 * AMutoRLTrainingDriver's class comment, "the one actor to place in a
	 * level"), otherwise the editor world's placed instance. Re-resolve on
	 * every call rather than caching the result, since PIE starting/stopping
	 * swaps which actor instance is "live" out from under any UI that stays
	 * open across that transition. Returns nullptr outside WITH_EDITOR, or if
	 * GEditor/the world/the actor don't exist (yet).
	 *
	 * EXACT class match only -- AMutoRagdollVisualizerActor and
	 * AMutoRLVisualizerActor both subclass AMutoRLTrainingDriver (for the
	 * shared rig/Batch/Solver plumbing, see their own class comments), so a
	 * plain TActorIterator<AMutoRLTrainingDriver> would also match them,
	 * silently pointing the toolbar's Start/Stop actions at whichever one
	 * happens to iterate first. Use FindRagdollVisualizer/FindRLVisualizer
	 * below to look for those specifically.
	 */
	// AGENTSOLVER_API on every function here -- these are free functions in a
	// namespace, not class members, so unlike UAgentSolverPreset etc. they
	// need an explicit export macro to be linkable from another module's DLL
	// at all. Missing entirely until the AgentSolverEditor module's live PIE
	// polling (SGraphNode_LearningProgram) became the first caller from
	// outside AgentSolver itself -- every previous caller (the control
	// panel, the embedded viewport client) links into the SAME DLL, where an
	// unexported symbol still resolves fine, so this was never caught.

	AGENTSOLVER_API AMutoRLTrainingDriver* FindTrainingDriver();

	/** Same world-selection rule as FindTrainingDriver, but for the ragdoll playback actor (AMutoRagdollVisualizerActor) specifically -- see EAgentSolverViewportSource. */
	AGENTSOLVER_API AMutoRagdollVisualizerActor* FindRagdollVisualizer();

	/** Same world-selection rule as FindTrainingDriver, but for the policy-inference playback actor (AMutoRLVisualizerActor) specifically -- see EAgentSolverViewportSource. */
	AGENTSOLVER_API AMutoRLVisualizerActor* FindRLVisualizer();

	/**
	 * Dispatches to FindRagdollVisualizer/FindRLVisualizer based on Source --
	 * the one place the embedded viewport client and the control panel's
	 * Agent/Physics parameter tabs both resolve "which actor is this
	 * EAgentSolverViewportSource pointing at right now" from, so the two
	 * can never disagree about it.
	 */
	AGENTSOLVER_API AMutoRLTrainingDriver* FindViewportSourceActor(EAgentSolverViewportSource Source);

	/**
	 * True only when a PIE session is actually running (GEditor->PlayWorld is
	 * valid). Gates AMutoRLTrainingDriver::StartTraining() specifically --
	 * unlike the read-only Find* functions above, which intentionally fall
	 * back to the editor world so the panel can show/edit an actor's
	 * properties before pressing Play, StartTraining() is NOT safe to run
	 * against the editor-world instance: it registers a real
	 * ULearningAgentsManager component, spawns NumEnvs UObject agents,
	 * launches the Python training subprocess and starts a background
	 * thread, all against a world that PIE never intended and that never
	 * gets torn down the way a PIE world does on Stop.
	 */
	AGENTSOLVER_API bool IsPIERunning();
}
