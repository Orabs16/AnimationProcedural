#pragma once

// Real-time, single-instance PLAYBACK of the currently trained policy —
// no training, no reward/completion gather, no experience buffer. Just:
// gather observations from its own single-env simulation, run inference
// deterministically (the policy's mean action, no exploration noise), apply
// the resulting actions, step physics at REAL wall-clock Dt (not the
// training driver's own, possibly-accelerated FixedDt), and pose a
// UPoseableMeshComponent from the result every frame.
//
// Owns its OWN, PRIVATE copy of the network weights — like a deployed game
// loading a policy checkpoint, not like a debugger attached to a live
// process. On a slow cadence (NetworkRefreshInterval) it copies the target
// AMutoRLTrainingDriver's current weights into its own networks via
// ULearningAgentsNeuralNetwork::LoadNetworkFromAsset (a fast in-memory
// struct copy, not a training/inference call). Every per-tick RunInference()
// call therefore only ever touches this actor's OWN networks, which nothing
// else ever writes to — no lock needed there.
// The refresh itself still has to read from the SAME live network objects
// the background training thread periodically overwrites (see
// AMutoRLTrainingDriver's GetNetworkAccessLock()), so it's guarded by that
// shared lock — but via TryLock, not a blocking Lock: if training happens to
// be mid-sync right when a refresh is due, the refresh is just skipped for
// this interval (this actor keeps animating its last-known-good snapshot)
// rather than ever stalling the game thread. This is what actually fixes
// the editor freezing again once this actor was added: the old design
// shared the SAME live network objects with the training thread and took
// the SAME lock, every single tick, for however long RunTraining() happened
// to be blocked — which reintroduced exactly the freeze the background
// thread was meant to solve, just one level removed.
//
// Usage: place one of these in the level alongside the AMutoRLTrainingDriver
// that's actually training, assign the same three rig assets, point
// SourceTrainingDriver at it, and press Play (or start it after training's
// already begun — it just needs SourceTrainingDriver's Policy to exist).

#include "CoreMinimal.h"
#include "AgentSolver/MutoRLTrainingDriver.h"
#include "MutoRLVisualizer.generated.h"

class UPoseableMeshComponent;

/**
 * Subclasses AMutoRLTrainingDriver purely for code reuse (rig-asset/reward-
 * config UPROPERTYs, Batch/Solver/ContactPoints simulation-state members,
 * GetEnvIndexForAgent, ComputeDefaultStandingHeight) — it always runs
 * exactly 1 env and overrides StartTraining()/Tick() entirely with a much
 * lighter inference-only setup (no Critic/TrainingEnvironment/Trainer/
 * Communicator). The inherited Training-category properties
 * (TrainerSettings/TrainingSettings/communicator settings, etc.) are unused
 * here and can be ignored in the Details panel.
 */
UCLASS()
class AGENTSOLVER_API AMutoRLVisualizerActor : public AMutoRLTrainingDriver
{
	GENERATED_BODY()

public:
	AMutoRLVisualizerActor();

	virtual void StartTraining() override;
	virtual void Tick(float DeltaTime) override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	/** The actor actually training — this visualizer periodically copies its Policy's network weights into its own, private networks (see class comment). */
	UPROPERTY(EditAnywhere, Category = "Muto RL Visualizer")
	TObjectPtr<AMutoRLTrainingDriver> SourceTrainingDriver;

	UPROPERTY(VisibleAnywhere, Category = "Muto RL Visualizer")
	TObjectPtr<UPoseableMeshComponent> MeshComponent;

	/** How often (seconds) to refresh this actor's private network snapshot from SourceTrainingDriver's current weights. Small enough to look "live", large enough that the refresh (and its TryLock — see class comment) is a rare event, not a per-tick one. */
	UPROPERTY(EditAnywhere, Category = "Muto RL Visualizer", meta = (ClampMin = "0.1"))
	float NetworkRefreshInterval = 2.0f;

	// ----- Live AI debug feed (see UIControls/SAgentSolverAIDebugPanel.h) -----
	// Not UPROPERTY (plain runtime data, rebuilt every Tick()/StartTraining(),
	// never meant to serialize) -- same treatment as Batch/Config above.
	// Single-writer (this actor's own Tick(), game thread only) / read from
	// the debug panel's own Slate Tick(), also game thread, so no lock is
	// needed, unlike AverageReward's cross-thread EMAs on the base class.

	/** This tick's full observation vector, exactly as fed to the policy — see CreatureRLEnvironment::ComputeObservations. Captured once per Tick(), just before RunInference(), so it reflects the SAME Batch/ContactStates snapshot the policy actually saw. */
	TArray<float> LastObservation;

	/** This tick's normalized [-1,1] action per DOF, i.e. Batch.JointTorque right after PerformAgentAction_Implementation's CreatureRLEnvironment::ApplyActions divided back by Config.MaxTorquePerDOF (the inverse of ApplyActions' own scale-up) — captured right after RunInference(), before StepPhysicsSubstepped() advances the sim. */
	TArray<float> LastNormalizedActions;

	/** Human-readable name for each LastObservation entry, same layout CreatureRLEnvironment::ComputeObservations documents (TorsoUp/TorsoLinVel/TorsoAngVel/HeightDelta, then JointPos/JointVel per DOF, then Touching/NormalForce per contact point). Rebuilt once in StartTraining() alongside BodyDebugNames — see BuildDebugNames(). */
	TArray<FName> ObservationNames;

	/** Human-readable name for each LastNormalizedActions entry (one per DOF). See ObservationNames. */
	TArray<FName> ActionNames;

private:
	/** Fills ObservationNames/ActionNames from Topo/BodyDebugNames/ContactPoints — called once from StartTraining() after the topology and contact points exist. */
	void BuildDebugNames();


	/** Poses MeshComponent's bones from the current single-env Batch state (body 0 -> "Pelvis", every other body -> its own bone name via BodyDebugNames). */
	void UpdateMeshPose();

	/**
	 * Copies SourceTrainingDriver's current network weights into this
	 * actor's own, private network objects (see class comment). Always
	 * non-blocking (TryLock): if the training thread currently holds
	 * SourceTrainingDriver's network lock, the refresh is skipped for this
	 * call rather than waiting for it.
	 *
	 * Used to ALWAYS be a genuine blocking Lock() for StartTraining()'s
	 * one-time initial sync (the reasoning being "this runs once, at
	 * startup, not per-tick, so blocking briefly is fine") -- that
	 * assumption is false: AMutoRLTrainingDriver::RunOneTrainingStep() holds
	 * this exact lock for the full duration of Trainer->RunTraining(),
	 * measured at up to ~375s on a real run (see
	 * AMutoRLTrainingDriver::AMutoRLTrainingDriver()'s
	 * SharedMemoryCommunicatorSettings.Timeout comment, 900s chosen to clear
	 * the slowest observed iteration) -- and the background training thread
	 * starts grabbing it the moment StartTraining() runs, so a visualizer
	 * started around the same time has a real chance of landing mid-sync
	 * and blocking the GAME thread (i.e. freezing the whole editor) for
	 * that same multi-minute span. Confirmed as the cause of a reported
	 * "editor completely freezes when starting Imitation training" bug --
	 * not actually Imitation-specific (the lock is unconditional), just
	 * more likely to land mid-sync there (longer-surviving episodes delay
	 * the replay buffer filling, so the first RunTraining() call for a
	 * batch of Imitation episodes tends to run longer). Non-blocking here
	 * means the mesh may show its default/random-weights pose for up to
	 * NetworkRefreshInterval seconds after starting, instead of trained
	 * weights immediately -- a trivial cost next to freezing the editor.
	 */
	void RefreshNetworkSnapshot();

	float NetworkRefreshTimer = 0.0f;

	/** Gates Tick()'s "[AS-TRACE]" agent+physics-tick heartbeat to roughly once a second instead of every frame -- see FAgentSolverViewportClient's matching mesh-show heartbeat, same "[AS-TRACE]" prefix so both sides of the pipeline can be read as one ordered trace. */
	int32 TraceHeartbeatCounter = 0;

	/** So the "Policy is null, Tick() no-oping entirely" warning below logs once when it starts, not every frame -- mirrors FAgentSolverViewportClient::LastPoseFailureReason's dedup pattern. */
	bool bLoggedNullPolicyWarning = false;
};
