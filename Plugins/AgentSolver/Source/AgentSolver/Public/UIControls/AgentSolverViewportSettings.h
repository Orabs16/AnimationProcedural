#pragma once

// Backing object for the AgentSolver control panel's "Viewport Parameters"
// tab (see SAgentSolverControlPanel.h) -- a plain UObject purely so
// IDetailsView::SetObject has something to edit. Not saved as an asset or
// persisted anywhere; one instance lives for as long as the Slate panel does.

#include "CoreMinimal.h"
#include "UObject/Object.h"
#include "UObject/SoftObjectPtr.h"

#include "AgentSolverViewportSettings.generated.h"

class UWorld;

/** Which actor's live pose the embedded viewport reads every frame -- see AgentSolverUI::FindViewportSourceActor, which this maps directly onto. */
UENUM(BlueprintType)
enum class EAgentSolverViewportSource : uint8
{
	/** AMutoRagdollVisualizerActor's passive-physics playback -- no policy, isolates the solver from the policy/reward (see that class's comment). */
	Ragdoll,
	/** AMutoRLVisualizerActor's policy-inference playback -- what the trained policy is doing, sharing weights from the live training driver. */
	Visualizer,
};

UCLASS()
class AGENTSOLVER_API UAgentSolverViewportSettings : public UObject
{
	GENERATED_BODY()

public:
	/** Which row of the source actor's Batch (env index) the embedded viewport poses its preview mesh from. Clamped to [0, NumEnvs) every frame -- irrelevant for Ragdoll/Visualizer, which only ever have 1 env, so this just clamps to 0 there. */
	UPROPERTY(EditAnywhere, Category = "Viewport", meta = (ClampMin = "0"))
	int32 EnvIndexToVisualize = 0;

	/** Show the preview scene's floor plane/grid -- purely cosmetic, matches the ground plane the batched solver's contact model treats as Z=0. */
	UPROPERTY(EditAnywhere, Category = "Viewport")
	bool bShowFloor = true;

	/** Which actor's pose the viewport currently displays. See EAgentSolverViewportSource. */
	UPROPERTY(EditAnywhere, Category = "Viewport")
	EAgentSolverViewportSource ViewportSource = EAgentSolverViewportSource::Ragdoll;

	/**
	 * Level whose static geometry (StaticMeshComponents only -- floors, walls,
	 * environment dressing) is copied into the embedded preview scene, purely
	 * for visual context around the rig. The level itself is never opened or
	 * streamed in; it's loaded as an inactive UWorld package
	 * (TSoftObjectPtr::LoadSynchronous) and its PersistentLevel's actors are
	 * scanned once, not simulated. Cleared = an empty preview scene (just the
	 * rig, no environment).
	 */
	UPROPERTY(EditAnywhere, Category = "Viewport")
	TSoftObjectPtr<UWorld> EnvironmentLevel;
};
