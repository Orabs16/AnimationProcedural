#pragma once

// Wires CreatureRLEnvironment.h's plain-C++ environment glue into the
// Learning Agents plugin, fully headlessly — no per-environment Actors, no
// level placement of anything but this ONE driver actor. Every "agent" the
// Learning Agents Manager tracks is a bare UObject standing in for one row
// (env index) of FCreatureBatchState's SoA arrays; there is nothing in the
// world to look at while training runs (see project memory/roadmap for the
// "visual scene vs. headless" design fork this resolves).
//
// The UCLASS/UPROPERTY declarations below are NOT WITH_EDITOR-gated (UHT
// rejects UPROPERTY wrapped in #if WITH_EDITOR) — but the actual logic in
// MutoRLTrainingDriver.cpp's BeginPlay/StartTraining IS, since it needs
// MutoTopology.h's Editor-type MassMuscleProfile dependency, and training
// only ever runs in-editor (PIE) for this project anyway. The LearningAgents
// modules this header includes are all Type:"Runtime" in their own
// .uplugin, so unlike MassMuscleProfile they're linked unconditionally (see
// AgentSolver.Build.cs) and safe to include here without a guard.
//
// Usage: place ONE of these actors in an (otherwise empty) level, assign
// Muto's SkeletalMesh/MassAsset/MuscleAsset in the Details panel, and press
// Play — BeginPlay wires up the Manager/Interactor/Policy/Critic/
// TrainingEnvironment/PPOTrainer and (if bAutoStartOnBeginPlay) launches the
// local Python training subprocess and starts a background thread that
// drives it continuously, decoupled from the editor's frame rate (see
// StartTraining()'s comment — ULearningAgentsPPOTrainer::RunTraining()
// blocks the calling thread whenever the replay buffer fills, so it must
// not run on the game thread or the whole editor stalls with it).

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "CreatureBatchState.h"
#include "CreatureBatchSolver.h"
#include "CreatureGroundContact.h"
#include "CreatureRLEnvironment.h"
#include "Async/Future.h"
#include <atomic>

#include "LearningAgentsInteractor.h"
#include "LearningAgentsTrainingEnvironment.h"
#include "LearningAgentsPolicy.h"
#include "LearningAgentsCritic.h"
#include "LearningAgentsPPOTrainer.h"
#include "LearningAgentsCommunicator.h"

#include "MutoRLTrainingDriver.generated.h"

class ULearningAgentsManager;
class UMassMuscleProfileAssetMass;
class UMassMuscleProfileAssetMuscle;
class USkeletalMesh;
class AMutoRLTrainingDriver;

/**
 * Observation/action schema and per-step gather/perform, delegating to
 * CreatureRLEnvironment.h. Pulls its shared simulation state from the owning
 * AMutoRLTrainingDriver via GetTypedOuter (Interactor's Outer is the
 * Manager component, whose Outer is the Driver actor — see MakeInteractor's
 * NewObject(InManager, ...) construction).
 */
UCLASS()
class AGENTSOLVER_API UMutoRLInteractor : public ULearningAgentsInteractor
{
	GENERATED_BODY()

public:
	virtual void SpecifyAgentObservation_Implementation(FLearningAgentsObservationSchemaElement& OutObservationSchemaElement, ULearningAgentsObservationSchema* InObservationSchema) override;
	virtual void GatherAgentObservation_Implementation(FLearningAgentsObservationObjectElement& OutObservationObjectElement, ULearningAgentsObservationObject* InObservationObject, const int32 AgentId) override;
	virtual void SpecifyAgentAction_Implementation(FLearningAgentsActionSchemaElement& OutActionSchemaElement, ULearningAgentsActionSchema* InActionSchema) override;
	virtual void PerformAgentAction_Implementation(const ULearningAgentsActionObject* InActionObject, const FLearningAgentsActionObjectElement& InActionObjectElement, const int32 AgentId) override;
};

/** Reward/completion/reset, delegating to CreatureRLEnvironment.h. Same Outer-chain pattern as UMutoRLInteractor. */
UCLASS()
class AGENTSOLVER_API UMutoRLTrainingEnvironment : public ULearningAgentsTrainingEnvironment
{
	GENERATED_BODY()

public:
	virtual void GatherAgentReward_Implementation(float& OutReward, const int32 AgentId) override;
	virtual void GatherAgentCompletion_Implementation(ELearningAgentsCompletion& OutCompletion, const int32 AgentId) override;
	virtual void ResetAgentEpisode_Implementation(const int32 AgentId) override;
};

/**
 * The one actor to place in a level to run headless standing/balance
 * training for Muto. Owns the batched physics simulation (FCreatureBatchState
 * + FCreatureABASolver, exactly as used by AgentSolver's automation tests —
 * no Chaos, no per-env Actors) and drives the Learning Agents training loop
 * on a dedicated background thread (see StartTraining()) rather than Tick(),
 * since ULearningAgentsPPOTrainer::RunTraining() blocks its calling thread
 * whenever the replay buffer fills — on the game thread, that stalls the
 * whole editor once per PPO sync.
 */
UCLASS()
class AGENTSOLVER_API AMutoRLTrainingDriver : public AActor
{
	GENERATED_BODY()

public:
	AMutoRLTrainingDriver();

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	/**
	 * Constructs the Manager/Interactor/Policy/Critic/TrainingEnvironment/
	 * PPOTrainer, launches the local Python training subprocess, runs one
	 * RunTraining()+physics-step iteration synchronously (see the .cpp — this
	 * one has to happen on the game thread), then hands off to a background
	 * thread for every iteration after that. Called automatically from
	 * BeginPlay if bAutoStartOnBeginPlay; otherwise call manually once ready
	 * (e.g. after tweaking settings in the Details panel without restarting
	 * PIE). Virtual: AMutoRLVisualizerActor overrides this with a much
	 * lighter inference-only setup (no Critic/TrainingEnvironment/Trainer/
	 * Communicator), and does not use the background thread at all.
	 */
	UFUNCTION(BlueprintCallable, Category = "Muto RL")
	virtual void StartTraining();

	/** Env index (row in Batch) for a given Learning Agents AgentId, or INDEX_NONE. */
	int32 GetEnvIndexForAgent(int32 AgentId) const;

	/** The trained policy, for AMutoRLVisualizerActor to share network weights with (read-only inference, no reinitialization). */
	ULearningAgentsPolicy* GetPolicy() const { return Policy; }

	/**
	 * Writes the Policy's current Encoder/Policy/Decoder network weights to
	 * NetworkSnapshotDirectory (3 raw ULearningAgentsNeuralNetwork snapshot
	 * files — see LoadNetworkFromSnapshot/SaveNetworkToSnapshot on that
	 * class). This is the only persistence this actor has: MakePolicy is
	 * always given nullptr network assets (see StartTraining), i.e. every
	 * training run's weights otherwise live ONLY in that transient in-memory
	 * Policy object and are lost the moment PIE stops — there is nothing
	 * else in this codebase that saves them. Safe to call any time once
	 * training has started; locks NetworkAccessLock the same way the
	 * training thread and AMutoRLVisualizerActor's periodic refresh do.
	 */
	UFUNCTION(BlueprintCallable, CallInEditor, Category = "Muto RL")
	void SaveTrainedNetworks();

	/**
	 * Loads a snapshot previously written by SaveTrainedNetworks into the
	 * CURRENTLY LIVE Policy, hot-swapping its weights mid-training. To
	 * resume a previous run from the very start instead (so the first
	 * inference/training step already uses the loaded weights, not a brief
	 * moment of freshly-initialized ones), set bLoadSnapshotOnStart=true
	 * instead of calling this manually.
	 */
	UFUNCTION(BlueprintCallable, CallInEditor, Category = "Muto RL")
	void LoadTrainedNetworks();

	/**
	 * Guards concurrent access to the Policy's live network weight objects:
	 * held by the background training thread while it calls RunTraining()
	 * (which reads AND overwrites those weights via PPO updates), and by
	 * AMutoRLVisualizerActor::Tick() while it calls RunInference() on the
	 * same shared Policy from the game thread.
	 */
	FCriticalSection& GetNetworkAccessLock() { return NetworkAccessLock; }

	// ----- Rig assets -----
	UPROPERTY(EditAnywhere, Category = "Muto RL|Rig")
	TObjectPtr<USkeletalMesh> SkeletalMesh;

	UPROPERTY(EditAnywhere, Category = "Muto RL|Rig")
	TObjectPtr<UMassMuscleProfileAssetMass> MassAsset;

	UPROPERTY(EditAnywhere, Category = "Muto RL|Rig")
	TObjectPtr<UMassMuscleProfileAssetMuscle> MuscleAsset;

	// ----- Simulation -----
	UPROPERTY(EditAnywhere, Category = "Muto RL|Simulation", meta = (ClampMin = "1"))
	int32 NumEnvs = 256;

	/** Simulated time one agent decision/RunOneTrainingStep() covers — the control-frequency knob (default 60Hz). NOT the physics integration step; see PhysicsSubstepDt for that. */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Simulation")
	float FixedDt = 1.0f / 60.0f;

	/**
	 * Actual physics integration step — StepPhysicsSubstepped() subdivides
	 * whatever total Dt it's given into substeps of (at most) this size,
	 * re-applying ground contact each substep, so the torque command from
	 * one agent decision (or one visualizer frame) holds constant across
	 * several finer integration steps. Needed because the ground-contact
	 * spring-damper (ContactSpringK/ContactDamperK) is only numerically
	 * stable for semi-implicit Euler at Dt <= ~1/240 for Muto's placeholder
	 * ~1-unit body masses (see CreatureGroundContactTest.cpp's MutoWiring
	 * test, which validated these exact constants at this exact Dt) — a
	 * 60Hz decision rate (FixedDt) or real frame time (the visualizer) is
	 * both too coarse for the contact dynamics on their own, which showed up
	 * as persistent jitter/instability once the standing pose was corrected
	 * enough to actually start making contact.
	 */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Simulation", meta = (ClampMin = "0.0001"))
	float PhysicsSubstepDt = 1.0f / 240.0f;

	/**
	 * Was hardcoded (-980 on Z, FCreatureABASolver::Step's own default
	 * parameter — matching UE's standard cm-scale gravity) with no way to
	 * change it from outside the solver. Exposed here so gravity can be
	 * tuned per training run — e.g. a curriculum that starts low (easier to
	 * balance while the policy is still learning basic coordination) and
	 * raises it toward -980 as training progresses.
	 */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Simulation")
	FVector Gravity = FVector(0.0f, 0.0f, -980.0f);

	/**
	 * Caps how fast any single joint (revolute or ball) can rotate, in
	 * degrees/second — applied every substep, after ClampJointLimits'
	 * position clamp. <= 0 disables it (no speed limit, the original
	 * behavior). Existed only implicitly before (whatever the torque/
	 * inertia ratio happened to produce) — this is a direct "slow down the
	 * muscles" knob, independent of MaxTorquePerDOF (which limits force,
	 * not speed) or the strength curves (which scale force by angle, not
	 * speed either).
	 */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Simulation")
	float MaxJointSpeedDegPerSec = 0.0f;

	UPROPERTY(EditAnywhere, Category = "Muto RL|Simulation")
	bool bAutoStartOnBeginPlay = true;

	// ----- Reward / episode config (see CreatureRLEnvironment::FEnvConfig) -----
	/** <= 0 means auto-derive from the topology's rest-pose leg length (see ComputeDefaultStandingHeight). */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Reward")
	float TargetTorsoHeightOverride = -1.0f;

	/** See CreatureRLEnvironment::FEnvConfig::MaxTorquePerDOF's comment — kept small deliberately given placeholder body masses. */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Reward")
	float MaxTorquePerDOF = 5.0f;

	UPROPERTY(EditAnywhere, Category = "Muto RL|Reward")
	float MinUprightDot = 0.5f;

	UPROPERTY(EditAnywhere, Category = "Muto RL|Reward")
	float MinHeightFraction = 0.5f;

	UPROPERTY(EditAnywhere, Category = "Muto RL|Reward")
	float AliveBonus = 1.0f;

	UPROPERTY(EditAnywhere, Category = "Muto RL|Reward")
	float UprightWeight = 1.0f;

	UPROPERTY(EditAnywhere, Category = "Muto RL|Reward")
	float BalanceWeight = 0.5f;

	/** See CreatureRLEnvironment::FEnvConfig::TorquePenaltyWeight's comment — TorquePenalty is now normalized to [0,1], this weight is directly comparable to AliveBonus/UprightWeight/BalanceWeight. */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Reward")
	float TorquePenaltyWeight = 0.1f;

	// ----- Reset domain randomization -----
	UPROPERTY(EditAnywhere, Category = "Muto RL|Reset")
	float PosNoiseStdDev = 5.0f;

	UPROPERTY(EditAnywhere, Category = "Muto RL|Reset")
	float AngleNoiseRad = 0.1f;

	UPROPERTY(EditAnywhere, Category = "Muto RL|Reset")
	int32 ResetRandomSeed = 1234;

	// ----- Ground contact tuning (see CreatureGroundContact::FContactParams; defaults match
	// the values CreatureGroundContactTest.cpp's MutoWiring test found stable for Muto's
	// placeholder ~1-unit body masses) -----
	/**
	 * Halved from the originally-validated 1000 (2026-08-12) after the
	 * ground contact model grew a capsule option (see
	 * FMassMuscleDataMass::CapsuleHalfHeight) — a capsule pressed flat
	 * against the ground can have BOTH end caps penetrating simultaneously,
	 * roughly doubling the total spring force a single limb can exert
	 * compared to the single-point-contact case this constant was
	 * originally tuned against. Halving keeps the WORST-CASE (both caps
	 * touching) combined stiffness equal to what was already validated
	 * stable, at the cost of a softer single-point contact (e.g. only the
	 * tip touching) sagging a bit more than before until re-tuned. Pending
	 * empirical re-confirmation, not independently re-validated the way
	 * the original 1000/40 were.
	 */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Contact")
	float ContactSpringK = 500.0f;

	/** See ContactSpringK's comment — halved for the same reason, same caveat. */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Contact")
	float ContactDamperK = 20.0f;

	/**
	 * Bumped 50->150 (2026-08-12): the friction model here is purely
	 * viscous (force proportional to tangential velocity, see
	 * ApplyGroundContactForces) — it damps sliding but never actually
	 * locks a contact point in place the way real static friction/stiction
	 * does, so persistent tangential forces (e.g. from an off-center
	 * contact torque) can cause slow, continuous drift even while
	 * "touching." A stronger FrictionK makes that decay to a standstill
	 * much faster, which is the practical mitigation here — it does not
	 * add true stiction. If drift is still visible after this, the real
	 * fix is a position-anchored (stiction-capable) friction model, not a
	 * further FrictionK increase.
	 */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Contact")
	float ContactFrictionK = 150.0f;

	UPROPERTY(EditAnywhere, Category = "Muto RL|Contact")
	float ContactFrictionCoefficient = 0.8f;

	// ----- Learning Agents settings (exposed directly, all default-constructed) -----
	UPROPERTY(EditAnywhere, Category = "Muto RL|Training")
	FLearningAgentsPPOTrainerSettings TrainerSettings;

	UPROPERTY(EditAnywhere, Category = "Muto RL|Training")
	FLearningAgentsPPOTrainingSettings TrainingSettings;

	UPROPERTY(EditAnywhere, Category = "Muto RL|Training")
	FLearningAgentsTrainingGameSettings GameSettings;

	UPROPERTY(EditAnywhere, Category = "Muto RL|Training")
	FLearningAgentsPolicySettings PolicySettings;

	UPROPERTY(EditAnywhere, Category = "Muto RL|Training")
	FLearningAgentsCriticSettings CriticSettings;

	UPROPERTY(EditAnywhere, Category = "Muto RL|Training")
	FLearningAgentsTrainerProcessSettings TrainerProcessSettings;

	/**
	 * Timeout bumped from the engine default (10s) to 60s: the very first
	 * handshake has to wait for the Python subprocess to import torch and
	 * friends, which can take a while on a cold start, and a too-short
	 * timeout here can look identical to a hard failure.
	 */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Training", meta = (EditCondition = "!bUseSocketCommunicator"))
	FLearningAgentsSharedMemoryCommunicatorSettings SharedMemoryCommunicatorSettings;

	/**
	 * The default (shared-memory) communicator uses Windows named shared
	 * memory, which can fail with a handshake/timeout error on some
	 * machine configs (session/privilege mismatches between the editor and
	 * the spawned Python subprocess, AV interference, etc. — see project
	 * memory for a real occurrence of this). If StartTraining logs
	 * "Communication timeout" / a Python-side FileNotFoundError opening a
	 * shared memory segment, try flipping this on: sockets avoid Windows
	 * shared memory entirely (loopback TCP instead), at the cost of a bit
	 * more communication overhead.
	 */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Training")
	bool bUseSocketCommunicator = false;

	UPROPERTY(EditAnywhere, Category = "Muto RL|Training", meta = (EditCondition = "bUseSocketCommunicator"))
	FLearningAgentsSocketCommunicatorSettings SocketCommunicatorSettings;

	/** Where SaveTrainedNetworks/LoadTrainedNetworks/bLoadSnapshotOnStart/bAutoSaveOnEndPlay read and write network snapshot files. Empty = ProjectSavedDir/MutoRL/Snapshots. */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Training")
	FDirectoryPath NetworkSnapshotDirectory;

	/** If true and a snapshot exists in NetworkSnapshotDirectory, StartTraining loads it into the freshly created Policy before the first training step — resumes a previous run instead of starting from random weights. */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Training")
	bool bLoadSnapshotOnStart = false;

	/** If true, EndPlay saves the current networks to NetworkSnapshotDirectory before tearing down — so simply stopping PIE never silently discards trained progress (see SaveTrainedNetworks's comment: this is otherwise the ONLY thing that persists trained weights at all). */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Training")
	bool bAutoSaveOnEndPlay = true;

	/**
	 * Periodically calls SaveTrainedNetworks() from the training thread while
	 * running (real wall-clock seconds, not sim time) — independent of
	 * bAutoSaveOnEndPlay/EndPlay, since a hard engine assertion (e.g. a NaN
	 * that reaches Learning Agents' own array-finite check) terminates the
	 * process without running EndPlay at all, so that safety net alone
	 * cannot protect a long unattended run against losing everything to a
	 * crash. <= 0 disables periodic autosave.
	 */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Training")
	float AutoSaveIntervalSeconds = 300.0f;

	// ----- Simulation state (plain C++, not UObjects — the batched solver, exactly as used
	// by AgentSolver's automation tests) -----
	FCreatureBatchState Batch;
	FCreatureABASolver Solver;
	TArray<CreatureGroundContact::FContactPointDef> ContactPoints;
	TArray<CreatureGroundContact::FContactPointState> ContactStates;
	CreatureRLEnvironment::FEnvConfig Config;
	FVector StandingTorsoPos = FVector::ZeroVector;
	FQuat StandingTorsoRot = FQuat::Identity;
	FRandomStream ResetStream;
	TArray<FName> BodyDebugNames; // body index -> bone name (index 0 is the synthetic "Torso" label, not a real bone)

protected:
	UPROPERTY()
	TObjectPtr<ULearningAgentsManager> Manager;

	UPROPERTY()
	TObjectPtr<ULearningAgentsInteractor> Interactor;

	UPROPERTY()
	TObjectPtr<ULearningAgentsPolicy> Policy;

	UPROPERTY()
	TObjectPtr<ULearningAgentsCritic> Critic;

	UPROPERTY()
	TObjectPtr<ULearningAgentsTrainingEnvironment> TrainingEnvironment;

	UPROPERTY()
	TObjectPtr<ULearningAgentsPPOTrainer> Trainer;

	UPROPERTY()
	TArray<TObjectPtr<UObject>> AgentObjects;

	TArray<int32> AgentIdToEnvIndex;

	/**
	 * Rest-pose standing height estimate: walks the parent chain from one
	 * contact point's body up to the torso, composing each body's REAL
	 * bind-pose rotation (BodyRestRotInParent) and joint offset — the same
	 * relation the solver's own forward kinematics reduces to at
	 * JointPos==0 — to find how far below StandingTorsoRotIn (in world Z)
	 * that contact point sits at rest. Override via TargetTorsoHeightOverride
	 * if it still looks wrong for the real rig.
	 */
	static float ComputeDefaultStandingHeight(const FCreatureTopology& Topo, const TArray<CreatureGroundContact::FContactPointDef>& InContactPoints, const FQuat& StandingTorsoRotIn);

	/** See GetNetworkAccessLock(). */
	FCriticalSection NetworkAccessLock;

	/** NetworkSnapshotDirectory.Path, or ProjectSavedDir/MutoRL/Snapshots if that's empty. */
	FString GetSnapshotDirectory() const;

	/** FPlatformTime::Seconds() timestamp of the last periodic autosave (see AutoSaveIntervalSeconds); set in StartTraining(). */
	double LastAutoSaveTime = 0.0;

	/**
	 * Clears external forces, applies ground contact, and steps Batch/Solver
	 * repeatedly at (at most) PhysicsSubstepDt each, covering TotalDt overall
	 * — see PhysicsSubstepDt's comment for why this exists. TotalDt is
	 * clamped to a sane max per call (avoids a substep storm after an editor
	 * hitch, e.g. in the visualizer's real-wall-clock DeltaTime). Shared by
	 * RunOneTrainingStep() (TotalDt = FixedDt) and AMutoRLVisualizerActor::
	 * Tick() (TotalDt = frame DeltaTime) — operates on whichever Batch/
	 * Solver/ContactPoints/Config the calling instance owns.
	 */
	void StepPhysicsSubstepped(float TotalDt);

	/**
	 * One RunTraining()+physics-step iteration. Returns false if training
	 * failed or has completed (caller should stop calling it) — shared by
	 * StartTraining()'s one synchronous call and RunTrainingThreadLoop().
	 */
	bool RunOneTrainingStep();

	/** Calls RunOneTrainingStep() in a loop until it returns false or a stop is requested; runs entirely off the game thread (see StartTraining()). */
	void RunTrainingThreadLoop();

	std::atomic<bool> bStopTrainingThreadRequested{false};
	TFuture<void> TrainingThreadFuture;
};
