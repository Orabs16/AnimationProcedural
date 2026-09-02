#pragma once

// Wires CreatureRLEnvironment.h's plain-C++ environment glue into the
// Learning Agents plugin, fully headlessly -- no per-environment Actors, no
// level placement of anything but this ONE driver actor. Every "agent" the
// Learning Agents Manager tracks is a bare UObject standing in for one row
// (env index) of FCreatureBatchState's SoA arrays; there is nothing in the
// world to look at while training runs (see project memory/roadmap for the
// "visual scene vs. headless" design fork this resolves).
//
// The UCLASS/UPROPERTY declarations below are NOT WITH_EDITOR-gated (UHT
// rejects UPROPERTY wrapped in #if WITH_EDITOR) -- but the actual logic in
// MutoRLTrainingDriver.cpp's BeginPlay/StartTraining IS, since it needs
// MutoTopology.h's Editor-type MassMuscleProfile dependency, and training
// only ever runs in-editor (PIE) for this project anyway. The LearningAgents
// modules this header includes are all Type:"Runtime" in their own
// .uplugin, so unlike MassMuscleProfile they're linked unconditionally (see
// AgentSolver.Build.cs) and safe to include here without a guard.
//
// Usage: place ONE of these actors in an (otherwise empty) level, assign
// Muto's SkeletalMesh/MassAsset/MuscleAsset in the Details panel, and press
// Play -- BeginPlay wires up the Manager/Interactor/Policy/Critic/
// TrainingEnvironment/PPOTrainer and (if bAutoStartOnBeginPlay) launches the
// local Python training subprocess and starts a background thread that
// drives it continuously, decoupled from the editor's frame rate (see
// StartTraining()'s comment -- ULearningAgentsPPOTrainer::RunTraining()
// blocks the calling thread whenever the replay buffer fills, so it must
// not run on the game thread or the whole editor stalls with it).

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "PhysicsSolver/CreatureBatchState.h"
#include "PhysicsSolver/CreatureBatchSolver.h"
#include "PhysicsSolver/CreatureGroundContact.h"
#include "AgentSolver/CreatureRLEnvironment.h"
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
class UAnimSequence;
class AMutoRLTrainingDriver;
class ULearningProgram;
class ULearningProgramNode;

/**
 * Editor-facing mirror of CreatureRLEnvironment::EObjectiveMode -- the plain
 * enum lives there so the header-only environment/reward code stays free of
 * UObject machinery, and this UENUM exists purely so the Details panel shows a
 * dropdown instead of a raw integer. Kept in lockstep by a static_assert in
 * MutoRLTrainingDriver.cpp; do not renumber either one independently.
 */
UENUM()
enum class EMutoObjectiveMode : uint8
{
	/** The original objective: stay upright, keep the centre of pressure under the torso, hold the target torso height. */
	Standing UMETA(DisplayName = "Standing / Balance"),

	/** Track a reference pose or animation (see CreatureImitation.h). */
	Imitation UMETA(DisplayName = "Imitation"),
};

/**
 * Observation/action schema and per-step gather/perform, delegating to
 * CreatureRLEnvironment.h. Pulls its shared simulation state from the owning
 * AMutoRLTrainingDriver via GetTypedOuter (Interactor's Outer is the
 * Manager component, whose Outer is the Driver actor -- see MakeInteractor's
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
 * One of these per env, purely as Manager->AddAgents' required per-agent
 * UObject identity handle -- carries no data or behavior of its own. Used to
 * be a bare NewObject<UObject>(this) (256 of them, one per env); that turned
 * out to be the exact, reproducible cause of "Class which was marked
 * abstract was trying to be loaded ... None Object" firing every single
 * StartTraining() call (LogUObjectGlobals, UObjectGlobals.cpp:3444's
 * ensureMsgf) -- decoding that ensure's own printf arguments gives
 * InName="None", InClass->GetName()="Object", an exact match for a bare,
 * unnamed UObject construction, and the ~256-line repeat count of the
 * underlying warning (one per AddAgents loop iteration) matched exactly.
 * Epic's own guidance is that NewObject<UObject>() directly (the literal
 * base class, no subclass) is not a supported pattern -- a trivial concrete
 * UCLASS sidesteps whatever editor-only check that trips on the bare base.
 */
UCLASS()
class UMutoRLAgentHandle : public UObject
{
	GENERATED_BODY()
};

/**
 * Live snapshot of a Learning Program's progress, for UI polling -- the node
 * graph editor's active-node highlight and per-transition progress bars (see
 * AMutoRLTrainingDriver::GetLearningProgramLiveState). Plain struct, not a
 * USTRUCT: nothing here needs Blueprint/serialization, just a cheap
 * lock-guarded copy out to the game thread.
 */
struct FLearningProgramLiveState
{
	/** False if ActiveLearningProgram is unset or StartTraining hasn't run yet (CurrentNodeId/the two progress fields below are meaningless when this is false). */
	bool bValid = false;
	FGuid CurrentNodeId;
	/** AMutoRLTrainingDriver::GetAverageReward() at snapshot time -- compare against an AverageRewardTarget transition's ThresholdValue. */
	float AverageReward = 0.0f;
	/** GetTrainingStepCount() - NodeEntryTrainingStepCount at snapshot time -- compare against a StepsSinceNodeEntry transition's ThresholdValue. */
	int32 StepsSinceNodeEntry = 0;
};

/**
 * The one actor to place in a level to run headless standing/balance
 * training for Muto. Owns the batched physics simulation (FCreatureBatchState
 * + FCreatureABASolver, exactly as used by AgentSolver's automation tests --
 * no Chaos, no per-env Actors) and drives the Learning Agents training loop
 * on a dedicated background thread (see StartTraining()) rather than Tick(),
 * since ULearningAgentsPPOTrainer::RunTraining() blocks its calling thread
 * whenever the replay buffer fills -- on the game thread, that stalls the
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
	 * RunTraining()+physics-step iteration synchronously (see the .cpp -- this
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

	/**
	 * Stops the background training thread and the Python trainer process
	 * (same sequence EndPlay runs on PIE stop -- see StopTrainingInternal),
	 * without destroying this actor. Lets a control panel/UI offer a "Stop"
	 * button independent of stopping PIE. Safe to call if training was never
	 * started or is already stopped (each step no-ops when there's nothing
	 * to do). Resets bStartTrainingCalled (see StopTrainingInternal) so a
	 * subsequent StartTraining() call in the SAME PIE session genuinely
	 * restarts training instead of hitting the re-entrancy guard.
	 */
	UFUNCTION(BlueprintCallable, Category = "Muto RL")
	void StopTraining();

	/** Env index (row in Batch) for a given Learning Agents AgentId, or INDEX_NONE. */
	int32 GetEnvIndexForAgent(int32 AgentId) const;

	/** The trained policy, for AMutoRLVisualizerActor to share network weights with (read-only inference, no reinitialization). */
	ULearningAgentsPolicy* GetPolicy() const { return Policy; }

	/** True once StartTraining has set up a Trainer that is still actively training (see ULearningAgentsPPOTrainer::IsTraining). For UI status readouts. */
	bool IsTrainingActive() const { return Trainer && Trainer->IsTraining(); }

	/**
	 * Writes the Policy's current Encoder/Policy/Decoder network weights to
	 * NetworkSnapshotDirectory (3 raw ULearningAgentsNeuralNetwork snapshot
	 * files -- see LoadNetworkFromSnapshot/SaveNetworkToSnapshot on that
	 * class). MakePolicy is always given nullptr network assets (see
	 * StartTraining), i.e. every training run's weights otherwise live ONLY
	 * in that transient in-memory Policy object and are lost the moment PIE
	 * stops -- this (and the UAsset-based SaveTrainedNetworksToAssets below)
	 * are the only things in this codebase that save them. Safe to call any
	 * time once training has started; locks NetworkAccessLock the same way
	 * the training thread and AMutoRLVisualizerActor's periodic refresh do.
	 */
	UFUNCTION(BlueprintCallable, Category = "Muto RL")
	void SaveTrainedNetworks();

	/**
	 * Loads a snapshot previously written by SaveTrainedNetworks into the
	 * CURRENTLY LIVE Policy, hot-swapping its weights mid-training. To
	 * resume a previous run from the very start instead (so the first
	 * inference/training step already uses the loaded weights, not a brief
	 * moment of freshly-initialized ones), set bLoadSnapshotOnStart=true
	 * instead of calling this manually.
	 */
	UFUNCTION(BlueprintCallable, Category = "Muto RL")
	void LoadTrainedNetworks();

	/**
	 * Copies the Policy's current Encoder/Policy/Decoder network weights into
	 * SaveEncoderNetworkAsset/SavePolicyNetworkAsset/SaveDecoderNetworkAsset
	 * (see ULearningAgentsNeuralNetwork::SaveNetworkToAsset) instead of the raw
	 * snapshot files SaveTrainedNetworks writes. Unlike those files, these are
	 * normal UDataAsset UAssets: assign them in the Details panel (create with
	 * Content Browser -> Miscellaneous -> Learning Agents Neural Network) to
	 * get versionable, content-browser-visible weight assets instead of loose
	 * files under Saved/. Logs and does nothing for any of the three slots
	 * left unassigned. Safe to call any time once training has started; takes
	 * NetworkAccessLock like SaveTrainedNetworks.
	 */
	UFUNCTION(BlueprintCallable, Category = "Muto RL")
	void SaveTrainedNetworksToAssets();

	/**
	 * Loads LoadEncoderNetworkAsset/LoadPolicyNetworkAsset/LoadDecoderNetworkAsset
	 * into the CURRENTLY LIVE Policy, hot-swapping its weights mid-training
	 * (see ULearningAgentsNeuralNetwork::LoadNetworkFromAsset). Logs and does
	 * nothing for any of the three slots left unassigned, so partially-filled
	 * asset references are safe to use. Deliberately a SEPARATE trio from the
	 * Save* properties above -- so you can seed from one checkpoint and save
	 * results to another without the two operations fighting over one asset.
	 */
	UFUNCTION(BlueprintCallable, Category = "Muto RL")
	void LoadTrainedNetworksFromAssets();

	/**
	 * Guards concurrent access to the Policy's live network weight objects:
	 * held by the background training thread while it calls RunTraining()
	 * (which reads AND overwrites those weights via PPO updates), and by
	 * AMutoRLVisualizerActor::Tick() while it calls RunInference() on the
	 * same shared Policy from the game thread.
	 */
	FCriticalSection& GetNetworkAccessLock() { return NetworkAccessLock; }

	/**
	 * Guards Batch/Solver/ContactPoints/ContactStates against the background
	 * training thread's StepPhysicsSubstepped call inside RunOneTrainingStep
	 * -- until the UI control-panel viewport started reading Batch for live
	 * display, nothing but that one loop ever touched them (see
	 * RunOneTrainingStep's comment: "no lock needed here"), so this is the
	 * first cross-thread reader and must take the SAME lock
	 * RunOneTrainingStep already holds for its own unrelated reason (see
	 * TrainingStepLock's comment), not a new one -- a second, independent
	 * lock would not actually prevent a torn read of Batch mid-step.
	 */
	FCriticalSection& GetTrainingStepLock() { return TrainingStepLock; }

	/**
	 * Thread-safe snapshot of the active Learning Program node/progress, for
	 * the node graph editor to poll (active-node highlight, per-transition
	 * progress bars) roughly every quarter second.
	 *
	 * Takes LearningProgramStateLock -- a SEPARATE, dedicated critical
	 * section, deliberately NOT TrainingStepLock/GetTrainingStepLock(),
	 * even though CurrentLearningProgramNodeId/NodeEntryTrainingStepCount are
	 * themselves written from inside RunOneTrainingStep (which already holds
	 * TrainingStepLock the whole time). TrainingStepLock is held for the
	 * ENTIRE step, including Trainer->RunTraining() -- which this class's own
	 * SharedMemoryCommunicatorSettings comment documents as measured at
	 * ~375s for one real PPO iteration, with a 900s timeout. A polling Tick()
	 * on the game thread taking that lock every 0.25s would therefore
	 * intermittently block the ENTIRE EDITOR UI for as long as whatever
	 * training step happened to be in flight -- exactly the "opening the
	 * tool freezes the editor" bug this project already hit once before with
	 * unthrottled/lock-heavy polling, and the reason GetTrainingStepLock()
	 * exists as a distinct, narrowly-scoped lock from this one: taking it
	 * here would silently reintroduce that bug. LearningProgramStateLock
	 * instead guards ONLY the two fields above, held for a trivial
	 * assignment/copy on both the write side (ApplyLearningProgramNode, which
	 * only runs at StartTraining and at each transition -- rare) and this
	 * read side, so it is never held for longer than a few instructions on
	 * either thread.
	 */
	FLearningProgramLiveState GetLearningProgramLiveState();

	/** Running exponential moving average of GatherAgentReward's per-agent, per-step reward, across every agent. Written only by the background training thread (UMutoRLTrainingEnvironment::GatherAgentReward_Implementation); safe to read from the game thread for UI status without a lock (single-writer, and each store is one atomic float write). Not used by training itself -- diagnostic only. */
	float GetAverageReward() const { return AverageReward.load(std::memory_order_relaxed); }

	// ----- Reward-component EMAs, same single-writer/lock-free-read pattern as
	// AverageReward above, for the Reward Settings pane's visualization graphs
	// (see SAgentSolverControlPanel.h) -- diagnostic only, not used by training. -----
	float GetLastTorsoHeightBonus() const { return LastTorsoHeightBonus.load(std::memory_order_relaxed); }
	float GetLastEnergyConsumptionMalus() const { return LastEnergyConsumptionMalus.load(std::memory_order_relaxed); }
	float GetLastMusclesUseMalus() const { return LastMusclesUseMalus.load(std::memory_order_relaxed); }
	float GetLastPoseReward() const { return LastPoseReward.load(std::memory_order_relaxed); }
	float GetLastVelocityReward() const { return LastVelocityReward.load(std::memory_order_relaxed); }
	float GetLastEndEffectorReward() const { return LastEndEffectorReward.load(std::memory_order_relaxed); }
	float GetLastRootReward() const { return LastRootReward.load(std::memory_order_relaxed); }

	/** True once StartTraining has successfully baked a reference pose/motion. The control panel uses this to show the imitation graphs only when they mean something. */
	bool HasReferenceMotion() const { return ReferenceMotionBaked.IsValid(); }

	/** Total RunOneTrainingStep() calls that have completed since StartTraining -- Learning Agents doesn't expose a PPO iteration counter publicly, so this stands in for "how far has training gotten" in the UI. */
	int32 GetTrainingStepCount() const { return TrainingStepCount.load(std::memory_order_relaxed); }

	/** Total episodes completed (summed across every agent) since StartTraining -- incremented once per UMutoRLTrainingEnvironment::ResetAgentEpisode_Implementation call. */
	int32 GetEpisodeCount() const { return EpisodeCount.load(std::memory_order_relaxed); }

	// ----- Rig assets -----
	UPROPERTY(EditAnywhere, Category = "Muto RL|Rig")
	TObjectPtr<USkeletalMesh> SkeletalMesh;

	UPROPERTY(EditAnywhere, Category = "Muto RL|Rig")
	TObjectPtr<UMassMuscleProfileAssetMass> MassAsset;

	UPROPERTY(EditAnywhere, Category = "Muto RL|Rig")
	TObjectPtr<UMassMuscleProfileAssetMuscle> MuscleAsset;

	/**
	 * Uniform multiplier on every muscle's delivered torque, applied on top of
	 * MuscleAsset's own per-DOF ExtensionStrength/FlexionStrength curves (see
	 * FCreatureABASolver::ComputeMuscleMultipliers's comment) -- a single
	 * "how strong is this whole rig" knob, independent of MaxTorquePerDOF
	 * (which caps what the POLICY can command, not what the muscle can
	 * physically deliver once commanded) and independent of each DOF's own
	 * authored strength (which sets its strength RELATIVE to other DOFs, not
	 * the rig's overall scale). 1.0 reproduces the authored strengths exactly;
	 * e.g. 0.5 halves every muscle's actual pulling force without touching
	 * the muscle profile asset or retuning MaxTorquePerDOF/reward
	 * normalization.
	 */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Rig|Muscle Strength")
	float GlobalMuscleStrengthScale = 1.0f;

	/**
	 * Curriculum knob for FMassMuscleDataMuscle::MuscleActivationThreshold
	 * (see CreatureRLEnvironment::FEnvConfig::MuscleActivationThresholdMultiplier's
	 * comment for the mechanism): the EFFECTIVE per-DOF activation threshold
	 * is the muscle asset's authored value times this multiplier, not the
	 * authored value directly. 1.0 (default) reproduces the authored
	 * thresholds exactly; 0.0 disables the dead zone entirely. Editable
	 * while training is running (refreshed every step from
	 * UMutoRLTrainingEnvironment::GatherAgentReward_Implementation, same as
	 * MaxTorquePerDOF/the reward weights below) -- the intended use is to
	 * start a run at 0 so early exploration isn't gated off before the
	 * policy has learned anything, then raise it toward 1.0 once the policy
	 * already knows how to move.
	 */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Rig|Muscle Strength")
	float MuscleActivationThresholdMultiplier = 1.0f;

	// ----- Simulation -----
	UPROPERTY(EditAnywhere, Category = "Muto RL|Simulation", meta = (ClampMin = "1"))
	int32 NumEnvs = 256;

	/** Simulated time one agent decision/RunOneTrainingStep() covers -- the control-frequency knob (default 60Hz). NOT the physics integration step; see PhysicsSubstepDt for that. */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Simulation")
	float FixedDt = 1.0f / 60.0f;

	/**
	 * Actual physics integration step -- StepPhysicsSubstepped() subdivides
	 * whatever total Dt it's given into substeps of (at most) this size,
	 * re-applying ground contact each substep, so the torque command from
	 * one agent decision (or one visualizer frame) holds constant across
	 * several finer integration steps. Needed because the ground-contact
	 * spring-damper (ContactSpringK/ContactDamperK) needs a far finer step
	 * than the 60Hz decision rate (FixedDt) or a real frame time (the
	 * visualizer) to stay stable, which showed up as persistent jitter once
	 * the standing pose was corrected enough to actually start making contact.
	 *
	 * The earlier claim here that 1/240 was validated as stable was WRONG --
	 * CreatureGroundContactTest's MutoWiring test only runs a brief no-NaN
	 * smoke check, which is not a stability test.
	 *
	 * MEASURED 2026-08-12 -- 1/240 was NOT stable and had to be raised to
	 * 1/960. A throwaway diagnostic ran the real rig with zero torque, zero
	 * reset noise and the shipped contact constants (500/20), i.e. nothing
	 * adversarial at all, and the solver DIVERGED within 2 simulated seconds
	 * (torso Z reached -2.4e17). The identical run at 1/960 is stable. The
	 * same diagnostic showed a contact-free fall is perfectly well behaved, so
	 * this is specifically the contact forces being integrated too coarsely,
	 * not the ABA solver or the joint limits.
	 *
	 * This is a 4x increase in physics cost per training step, which is real,
	 * but the alternative is a simulation that can blow up on its own -- the
	 * most likely explanation for limbs that "fall through the ground and drag
	 * the model with them".
	 *
	 * Do NOT assume a finer substep buys arbitrary contact stiffness: the same
	 * sweep found SpringK=3000 unstable at 1/3840 and SpringK=11000 unstable
	 * even at 1/15360, which is not how a linear stability limit behaves and
	 * is still unexplained. Raising SpringK therefore needs its own
	 * investigation, not just more substeps.
	 */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Simulation", meta = (ClampMin = "0.0001"))
	// 1/960 was adopted to mask two ABA bugs that are now fixed (SOLVER_DEBUG_LOG.md
	// entries 005 and 012) -- the contact-free rig is stable at 60 Hz single
	// substep and converges properly with dt again. 240 Hz is chosen for FIDELITY,
	// not stability: it is a normal rate for this class of solver and gives
	// smoother contact and joint behaviour than 60. Raise it if the motion needs
	// it; do not raise it to buy stability, since that was the symptom that
	// concealed the original defects.
	float PhysicsSubstepDt = 1.0f / 240.0f;

	/**
	 * Was hardcoded (-980 on Z, FCreatureABASolver::Step's own default
	 * parameter -- matching UE's standard cm-scale gravity) with no way to
	 * change it from outside the solver. Exposed here so gravity can be
	 * tuned per training run -- e.g. a curriculum that starts low (easier to
	 * balance while the policy is still learning basic coordination) and
	 * raises it toward -980 as training progresses.
	 *
	 * Nested under "Muto RL|Simulation|Gravity" specifically (not the bare
	 * "Muto RL|Simulation" every other Simulation property uses) so it can
	 * ALSO be listed in SAgentSolverControlPanel's RewardSettingsCategoryPrefixes
	 * without pulling in the rest of Simulation (e.g. bAutoStartOnBeginPlay)
	 * too -- still caught by the Physics tab's plain "Muto RL|Simulation"
	 * prefix as well (StartsWith), so it keeps showing there too. One real
	 * UPROPERTY, editable from both IDetailsViews -- not a separate mirrored
	 * copy that could drift out of sync.
	 */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Simulation|Gravity")
	FVector Gravity = FVector(0.0f, 0.0f, -980.0f);


	UPROPERTY(EditAnywhere, Category = "Muto RL|Simulation")
	bool bAutoStartOnBeginPlay = true;

	// ----- Reward / episode config (see CreatureRLEnvironment::FEnvConfig) -----
	/** <= 0 means auto-derive from the topology's rest-pose leg length (see ComputeDefaultStandingHeight). */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Reward")
	float TargetTorsoHeightOverride = -1.0f;

	/**
	 * See CreatureRLEnvironment::FEnvConfig::MaxTorquePerDOF's comment --
	 * kept small deliberately given placeholder body masses.
	 *
	 * Nested under "Muto RL|Reward|Torque" (2026-08-25), not the bare
	 * "Muto RL|Reward" every other field below (except the Core
	 * Weights/Termination groups) uses, so it ALSO shows in the Reward
	 * Settings pane next to Torso Height/Muscle Penalty -- still caught by
	 * the Agent tab's plain "Muto RL|Reward" prefix too (StartsWith), same
	 * one-property-two-views trick Gravity already uses (see its own
	 * comment). This one and TorquePenaltyWeight sit together since
	 * MaxTorquePerDOF is what TorquePenalty is normalized against.
	 */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Reward|Torque")
	float MaxTorquePerDOF = 5.0e7f;

	/**
	 * "Fallen over" termination threshold, not itself a reward term (see
	 * CreatureRLEnvironment::IsTerminated) -- grouped under "Muto RL|Reward|
	 * Termination" (2026-08-25) anyway since it directly shapes how much
	 * reward an episode can accumulate before ending, same reasoning as
	 * MinHeightFraction below. Same dual-tab trick as MaxTorquePerDOF above.
	 */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Reward|Termination")
	float MinUprightDot = 0.5f;

	/** See MinUprightDot's comment -- the other half of the same termination check. */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Reward|Termination")
	float MinHeightFraction = 0.5f;

	/**
	 * Nested under "Muto RL|Reward|Core Weights" (2026-08-25) -- same
	 * dual-tab trick as MaxTorquePerDOF's comment above.
	 */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Reward|Core Weights")
	float AliveBonus = 1.0f;

	UPROPERTY(EditAnywhere, Category = "Muto RL|Reward|Core Weights")
	float UprightWeight = 1.0f;

	UPROPERTY(EditAnywhere, Category = "Muto RL|Reward|Core Weights")
	float BalanceWeight = 0.5f;

	/** See CreatureRLEnvironment::FEnvConfig::TorquePenaltyWeight's comment -- TorquePenalty is now normalized to [0,1], this weight is directly comparable to AliveBonus/UprightWeight/BalanceWeight. Grouped with MaxTorquePerDOF above -- see its comment for the category-nesting reasoning. */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Reward|Torque")
	float TorquePenaltyWeight = 0.1f;

	// ----- Reward Settings pane (see SAgentSolverControlPanel.h) -- deliberately
	// a SEPARATE "Muto RL|Tuning" category, not "Muto RL|Reward" above, so this
	// doesn't ALSO show in the Agent tab (whose category-prefix allowlist would
	// otherwise catch it too, showing every field twice). See
	// CreatureRLEnvironment::ComputeTorsoHeightBonus/ComputeEnergyConsumptionMalus/
	// ComputeMusclesUseMalus for the exact formulas these feed. -----
	/** Torso height (Batch.GetBodyPos(0,Env).Z) the exponential-falloff height bonus rewards being close to. */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Tuning|Torso Height")
	float RewardHeightTarget = 100.0f;

	/** Scales the torso-height bonus's contribution to the total reward (added, not subtracted). */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Tuning|Torso Height")
	float RewardHeightMultiplier = 1.0f;

	/**
	 * Scales the energy-consumption malus (sum, across DOFs, of |commanded
	 * torque| as a fraction of MaxTorquePerDOF -- normalized 2026-08-26,
	 * see CreatureRLEnvironment::ComputeEnergyConsumptionMalus's comment for
	 * why the old flat /10000 divisor was wrong). DisplayName drops the
	 * "Reward" prefix the C++/serialized name still carries (kept as-is so
	 * already-tuned values and saved presets don't reset to default on a
	 * rename) -- "Reward Energy Consumption Multiplier" read as scaling a
	 * bonus, when this is SUBTRACTED from the total, which is exactly
	 * backwards (2026-08-25 UI feedback).
	 */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Tuning|Muscle Penalty", meta = (DisplayName = "Energy Consumption Malus Multiplier"))
	float RewardEnergyConsumptionMultiplier = 1.0f;

	/** Scales the muscles-use malus (sum, across DOFs, of commanded torque as a % of max power at the DOF's current angle). See RewardEnergyConsumptionMultiplier's comment for why this has a DisplayName override. */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Tuning|Muscle Penalty", meta = (DisplayName = "Muscles Use Malus Multiplier"))
	float RewardMusclesUseMultiplier = 1.0f;

	/**
	 * Final multiplier applied to the fully-summed reward, AFTER every term
	 * above (AliveBonus/UprightWeight/BalanceWeight/TorquePenaltyWeight/
	 * TorsoHeightBonus/EnergyConsumptionMalus/MusclesUseMalus) -- see
	 * CreatureRLEnvironment::ComputeReward's return statement. Rescales the
	 * WHOLE reward the trainer sees in one place, without re-deriving every
	 * individual weight above.
	 */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Tuning|Global Scale")
	float GlobalRewardScale = 1.0f;

	/** Added AFTER GlobalRewardScale (Reward*Scale + Offset) -- see CreatureRLEnvironment::FEnvConfig::GlobalRewardOffset's comment. */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Tuning|Global Scale")
	float GlobalRewardOffset = 0.0f;

	// ----- Imitation: structural settings (see CreatureImitation.h / ImitationBake.h) -----
	//
	// These are all read ONCE, by StartTraining, because changing any of them
	// requires re-baking the reference or re-shaping the observation -- unlike
	// the weights further below, editing one mid-PIE does nothing until
	// training is restarted. That is why they live under "Muto RL|Imitation"
	// (the Agent tab) rather than "Muto RL|Tuning" (the always-visible Reward
	// Settings pane, which is for things that respond live).

	UPROPERTY(EditAnywhere, Category = "Muto RL|Imitation")
	EMutoObjectiveMode ObjectiveMode = EMutoObjectiveMode::Standing;

	/**
	 * The clip to imitate. Must be authored against the same USkeleton as
	 * SkeletalMesh, or the bake refuses it rather than silently reading the
	 * wrong bones.
	 */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Imitation")
	TObjectPtr<UAnimSequence> ReferenceMotion;

	/**
	 * PHASE 1 (single pose): the time in ReferenceMotion to freeze on. The
	 * whole clip collapses to this one frame, velocities are zero throughout,
	 * and the task is "reach and hold this pose".
	 *
	 * Ignored when bImitateFullClip is true.
	 */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Imitation", meta = (ClampMin = "0.0"))
	float ReferencePoseTime = 0.0f;

	/**
	 * PHASE 2 (animation): bake the whole clip instead of one frame, index it
	 * by phase, and give each episode a random starting phase.
	 *
	 * Turning this on also turns on the phase observation, which CHANGES THE
	 * NETWORK'S INPUT SIZE -- networks saved with it off cannot be loaded with
	 * it on. StartTraining logs the observation size on every run so the
	 * change is visible rather than surprising.
	 */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Imitation")
	bool bImitateFullClip = false;

	/** Bake rate for the clip. Independent of the source's own frame rate -- denser just interpolates, and a round rate keeps the finite-differenced reference velocities clean. */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Imitation", meta = (EditCondition = "bImitateFullClip", ClampMin = "1.0"))
	float ReferenceSampleRate = 30.0f;

	/** Whether the clip cycles. Controls both the phase wrap and the velocity finite-difference at the clip's ends -- leave it off for a one-shot motion, or the first and last frames get velocities that pretend the clip teleports back to its start. */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Imitation", meta = (EditCondition = "bImitateFullClip"))
	bool bReferenceMotionLoops = true;

	/**
	 * Start each episode AT the reference (reference state initialization), so
	 * the task is "hold/continue the motion" rather than "find it from a
	 * standing start".
	 *
	 * Strongly recommended on, and for a cyclic motion effectively mandatory:
	 * without it every episode begins at the clip's start, so the policy must
	 * master the opening before it ever experiences the rest. With it, and
	 * with bImitateFullClip, each episode picks a random phase, so all 256
	 * envs together cover the entire clip from the very first PPO iteration.
	 *
	 * It also makes the first-frame sanity check possible: right after a
	 * reset, the pose reward should read ~1.0. If it does not, the bake or the
	 * retarget is wrong, and nothing in the training curve means anything yet.
	 */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Imitation")
	bool bResetToReferencePose = true;

	/**
	 * Bones whose positions the end-effector reward tracks -- feet and hands.
	 * Matched against the same per-body bone names BuildMutoTopology produces
	 * (see BodyDebugNames); anything unmatched is logged and skipped rather
	 * than failing the bake.
	 *
	 * Empty means the end-effector term is a constant 1.0 and contributes
	 * nothing but its weight. That is a real loss for locomotion: pose error
	 * is measured per JOINT, so a small hip error and a small knee error can
	 * each look negligible while compounding into a foot landing 20cm off.
	 * The end-effector term is what notices that.
	 */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Imitation")
	TArray<FName> EndEffectorBoneNames;

	// ----- Imitation: live-tunable weights (Reward Settings pane) -----
	//
	// Under "Muto RL|Tuning|Imitation" deliberately, NOT "Muto RL|Reward|..." --
	// SAgentSolverControlPanel filters by bare FString::StartsWith, so anything
	// under "Muto RL|Reward" is ALSO caught by the Agent tab's own plain
	// "Muto RL|Reward" prefix and would show twice. "Muto RL|Tuning" is already
	// allow-listed for the Reward Settings pane alone.

	/** Weight on the joint-pose tracking term. DeepMimic's published weighting is 0.65/0.10/0.15/0.10 across these four. */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Tuning|Imitation", meta = (ClampMin = "0.0"))
	float ImitationPoseWeight = 0.65f;

	UPROPERTY(EditAnywhere, Category = "Muto RL|Tuning|Imitation", meta = (ClampMin = "0.0"))
	float ImitationVelocityWeight = 0.10f;

	UPROPERTY(EditAnywhere, Category = "Muto RL|Tuning|Imitation", meta = (ClampMin = "0.0"))
	float ImitationEndEffectorWeight = 0.15f;

	UPROPERTY(EditAnywhere, Category = "Muto RL|Tuning|Imitation", meta = (ClampMin = "0.0"))
	float ImitationRootWeight = 0.10f;

	/** Falloff rate of the pose term: exp(-Scale * mean squared joint error in radians). Larger = less forgiving. */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Tuning|Imitation|Falloff", meta = (ClampMin = "0.0"))
	float ImitationPoseErrorScale = 2.0f;

	UPROPERTY(EditAnywhere, Category = "Muto RL|Tuning|Imitation|Falloff", meta = (ClampMin = "0.0"))
	float ImitationVelocityErrorScale = 0.1f;

	UPROPERTY(EditAnywhere, Category = "Muto RL|Tuning|Imitation|Falloff", meta = (ClampMin = "0.0"))
	float ImitationEndEffectorErrorScale = 40.0f;

	UPROPERTY(EditAnywhere, Category = "Muto RL|Tuning|Imitation|Falloff", meta = (ClampMin = "0.0"))
	float ImitationRootErrorScale = 10.0f;

	/** Mean per-joint pose error (radians) above which the episode ends. 0 disables. Imitation's analogue of the standing objective's fallen-over check. */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Tuning|Imitation|Termination", meta = (ClampMin = "0.0"))
	float ImitationMaxPoseErrorRad = 0.0f;

	/** Whether imitation ALSO ends an episode on the MinUprightDot/MinHeightFraction thresholds. Turn off for a reference motion that legitimately puts the torso down (a crawl, a roll, a get-up), where those would end every episode on frame one. */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Tuning|Imitation|Termination")
	bool bImitationTerminateOnUprightAndHeight = true;

	// ----- Reset domain randomization -----
	UPROPERTY(EditAnywhere, Category = "Muto RL|Reset")
	float PosNoiseStdDev = 5.0f;

	UPROPERTY(EditAnywhere, Category = "Muto RL|Reset")
	float AngleNoiseRad = 0.1f;

	UPROPERTY(EditAnywhere, Category = "Muto RL|Reset")
	int32 ResetRandomSeed = 1234;

	// ----- Per-episode domain randomization (CreatureRLEnvironment::FDomainRandomization) -----
	//
	// The batch has carried per-env LimbStrengthScale / LimbActive / CarriedMass
	// since it was written, and both step variants consume all three in Pass 2,
	// but nothing outside the SIMD parity test ever wrote them -- so every env
	// trained on an identical, undamaged, unloaded creature. These knobs feed
	// ResetEnv, which draws fresh values at every episode reset.
	//
	// OFF by default: turning it on changes what the policy is being asked to
	// solve, so it is an explicit decision, not a silent default change. Expect
	// reward to drop when first enabled -- the task genuinely got harder.
	UPROPERTY(EditAnywhere, Category = "Muto RL|Reset|Domain Randomization")
	bool bEnableDomainRandomization = false;

	/** Per-limb muscle strength multiplier, drawn uniformly per limb per episode. Both 1.0 = no strength variation. */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Reset|Domain Randomization", meta = (EditCondition = "bEnableDomainRandomization", ClampMin = "0.0"))
	float MinLimbStrengthScale = 0.8f;

	UPROPERTY(EditAnywhere, Category = "Muto RL|Reset|Domain Randomization", meta = (EditCondition = "bEnableDomainRandomization", ClampMin = "0.0"))
	float MaxLimbStrengthScale = 1.2f;

	/**
	 * Probability that any given limb goes limp for a whole episode (its muscles
	 * produce no torque; the joints stay intact and still swing passively). 0 =
	 * never. Kept low by default -- this is the harshest of the three, since a
	 * creature that has not learned to stand on all limbs yet cannot learn to
	 * stand on fewer.
	 */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Reset|Domain Randomization", meta = (EditCondition = "bEnableDomainRandomization", ClampMin = "0.0", ClampMax = "1.0"))
	float LimbLossChance = 0.0f;

	/**
	 * Upper bound of the uniform draw for extra mass carried at the torso, in kg
	 * (same units as BodyMass). Added to body 0's mass without shifting its CoM
	 * -- see the solver's Pass 1.
	 */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Reset|Domain Randomization", meta = (EditCondition = "bEnableDomainRandomization", ClampMin = "0.0"))
	float MaxCarriedMass = 0.0f;

	/**
	 * Assembles the config ResetEnv draws from. Inline and public so the two
	 * visualizer subclasses -- which replace StartTraining and their own reset
	 * paths wholesale rather than calling Super -- can opt in with one call
	 * instead of re-reading five properties each.
	 *
	 * Max is floored at Min so an inverted range authored in the details panel
	 * degenerates to a constant rather than to FRandRange's undefined ordering.
	 */
	CreatureRLEnvironment::FDomainRandomization MakeDomainRandomization() const
	{
		CreatureRLEnvironment::FDomainRandomization Out;
		Out.bEnabled = bEnableDomainRandomization;
		Out.MinLimbStrengthScale = MinLimbStrengthScale;
		Out.MaxLimbStrengthScale = FMath::Max(MaxLimbStrengthScale, MinLimbStrengthScale);
		Out.LimbLossChance = LimbLossChance;
		Out.MaxCarriedMass = MaxCarriedMass;
		return Out;
	}

	// ----- Ground contact: velocity-level impulse, the ONLY model -----
	//
	// The penalty spring-damper was REMOVED 2026-08-14. It had no advantage that
	// survived measurement, and two active costs: its stiffness had to be
	// rescaled whenever mass changed (it silently produced a 6047-unit resting
	// sag until that was caught -- SOLVER_DEBUG_LOG.md entry 017), and it ran on
	// the OPPOSITE side of Step() from the impulse path, so keeping both wired up
	// meant two different step orderings coexisting in one file.
	//
	// The impulse model solves for the impulse that stops a contact point
	// approaching, using the true articulated effective mass, so it has no
	// stiffness to tune against mass at all.

	/**
	 * Contact stiffness as a natural frequency, in Hz -- invariant to creature mass
	 * and to timestep. Internally clamped to a quarter of the substep rate, which at
	 * the shipped PhysicsSubstepDt of 1/240 is 60 Hz -- so 45 sits below a real
	 * ceiling, not at it.
	 *
	 * RE-FIT 2026-08-21 from 15, which came from entry 018's sweep. That fit was
	 * invalid three times over (wrong joint axes, no torso collision, thin-rod
	 * inertia -- entries 022, 025, 028) and, more importantly, was measured on a
	 * PASSIVE DROP. A passive drop cannot fit this constant at all: all 24 pairs in
	 * the re-fit sweep survive it. Only under actuation do they separate, and
	 * 15/10 DIVERGES at 35.55 s of sustained torque babble while 45/10 survives the
	 * full 50 s. See SOLVER_DEBUG_LOG.md entry 029.
	 *
	 * The trade is real and was made deliberately: peak penetration on the passive
	 * impact transient gets WORSE (18.89 -> 47.95 cm), while resting penetration
	 * stays comparable and the rig stops blowing up under load. A transient the rig
	 * recovers from beats a divergence it does not.
	 */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Contact|Impulse")
	float ContactHertz = 45.0f;

	/**
	 * Contact damping ratio. 10 is a normal value for contacts (heavily damped), and
	 * the re-fit sweep independently landed back on it -- worth noting, because the
	 * underdamped end of the range is actively dangerous here: at 45 Hz, zeta = 2
	 * survives 12 s of full-amplitude actuation for only 3.88 s, where zeta = 10
	 * survives the whole run.
	 */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Contact|Impulse")
	float ContactDampingRatio = 10.0f;

	/**
	 * Solver passes per substep, with stabilization on. Was 8 until 2026-08-16:
	 * with limb collision enabled, 8 iterations reproducibly convergence-starved
	 * on the real rig (503 candidate limb pairs sharing this budget with ground
	 * contact + joint limits) and blew up a passive ragdoll drop at t=0.92s.
	 * 16 was the verified minimum that ran the same drop clean for a full
	 * second; see AgentSolver.TEMP.LimbCollisionRevertVerify's iteration sweep.
	 */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Contact|Impulse")
	int32 ContactIterations = 16;

	/**
	 * Extra passes with stabilization OFF, which take back the energy
	 * stabilization deliberately injects. See FImpulseContactParams::RelaxIterations.
	 */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Contact|Impulse")
	int32 ContactRelaxIterations = 0;

	/** Penetration left uncorrected so resting contacts settle instead of buzzing. */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Contact|Impulse")
	float ContactSlop = 0.5f;

	UPROPERTY(EditAnywhere, Category = "Muto RL|Contact")
	float ContactFrictionCoefficient = 0.8f;

	// ----- Global constraint solve (MuJoCo's formulation) -----

	/**
	 * Assemble A = J M^-1 J^T + R over every active constraint row and solve THAT,
	 * instead of relaxing each row against the bodies one at a time.
	 *
	 * This is the change SOLVER_DEBUG_LOG.md entry 024 identified as correct and
	 * deferred. Its iteration sweep (8/16/32/64/128 passes over 6 simulated
	 * seconds) diverged in every configuration and NON-MONOTONICALLY, which is the
	 * signature of Gauss-Seidel on a strongly coupled system rather than of a
	 * budget that is merely too small -- so "more iterations" and every variant of
	 * it is refuted, and the solver class is what is left.
	 *
	 * Turning this OFF returns to the per-row path, which is kept intact for
	 * exactly that comparison. See CreatureGroundContact.h's FGlobalRow.
	 */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Contact|Global Solve")
	bool bUseGlobalConstraintSolve = true;

	/**
	 * Sweeps for the global solve. Deliberately much larger than
	 * ContactIterations: a global sweep is O(rows^2) of plain arithmetic touching
	 * no bodies at all, where a per-row sweep costs a full tree pass per row. The
	 * budget entry 024 could not afford to raise is affordable here.
	 */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Contact|Global Solve", meta = (ClampMin = "1"))
	int32 GlobalSolverIterations = 64;

	// ----- Row regularization (was implemented and unreachable until 2026-08-17) -----
	//
	// Cfm and Relaxation are two independent, complementary mechanisms, both added
	// for the measured case where a body's ground row and its OWN joint-limit row
	// are strongly active at once -- an ankle or elbow folded onto its stop while
	// also bearing ground load. Neither had a caller or an editor property, so
	// neither had ever run. Cfm bounds how BIG one row's response can get;
	// Relaxation bounds how FAST two coupled rows can swing against each other.
	//
	// Defaults are deliberately conservative but NON-ZERO, because shipping them
	// at their disabled values is what left them dormant in the first place.

	/** CFM-style regularization added to each ground row's own effective-mass denominator. 0 = off. */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Contact|Regularization", meta = (ClampMin = "0.0"))
	float ContactCfm = 1.0e-8f;

	/** SOR under-relaxation on each ground row's accumulated impulse. 1 = full correction per iteration. */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Contact|Regularization", meta = (ClampMin = "0.05", ClampMax = "1.0"))
	float ContactRelaxation = 1.0f;

	/** Ceiling on the ground rows' push-out velocity, cm/s. Was reachable only as a hardcoded default. */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Contact|Regularization", meta = (ClampMin = "0.0"))
	float ContactMaxBiasVelocity = 100.0f;

	/** Upper bound on a ground row's per-step normal impulse. 0 = none. A safety net, not a tuning knob. */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Contact|Regularization", meta = (ClampMin = "0.0"))
	float ContactMaxNormalImpulse = 0.0f;

	/** CFM regularization on each joint-limit row. See ContactCfm. */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Contact|Joint Limits", meta = (ClampMin = "0.0"))
	float JointLimitCfm = 1.0e-8f;

	/** SOR under-relaxation on each joint-limit row. See ContactRelaxation. */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Contact|Joint Limits", meta = (ClampMin = "0.05", ClampMax = "1.0"))
	float JointLimitRelaxation = 1.0f;

	/** Cap on a violated limit's push-back velocity, deg/s. Was reachable only as a hardcoded default. */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Contact|Joint Limits", meta = (ClampMin = "0.0"))
	float JointLimitMaxBiasVelocityDeg = 720.0f;

	/**
	 * WELD saturated joints inside the articulated-inertia factorization.
	 *
	 * A contact's articulated effective mass is computed as though every joint
	 * above it rotates freely -- correct for an impulse on a free chain, badly
	 * wrong for a leg folded solid onto its stops. Entry 023 measured 14 kg at a
	 * foot under a 6170 kg creature, rising to only 17 kg with six leg joints
	 * saturated. Entry 024 then measured what welding buys: 18 kg -> 269 kg
	 * (14.8x) at the exact substep the heels were being dragged under.
	 *
	 * That measurement stood while the mechanism itself had no production caller
	 * -- only a diagnostic ever passed a lock set, so no real run ever used it.
	 * This turns it on. Only joints that are at a stop AND still being driven into
	 * it are welded (see FCreatureABASolver::BuildSaturatedJointLocks), which
	 * confines the one-sided approximation to the case it is right for.
	 */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Contact|Joint Limits")
	bool bWeldSaturatedJoints = true;

	/** How close to a stop counts as saturated, in degrees, for bWeldSaturatedJoints. */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Contact|Joint Limits", meta = (ClampMin = "0.0"))
	float WeldSaturationMarginDeg = 1.0f;

	/**
	 * Give EVERY body a ground-collision sphere at its joint origin, not just
	 * the ones flagged CanTouchGround in the mass asset.
	 *
	 * GROUND ONLY -- this does not add self-collision, and cannot: the contact
	 * solver tests against the ground plane and nothing else. Limbs still pass
	 * through each other.
	 *
	 * Without it the torso (body 0, ~3282 kg of the creature's 6170) has no
	 * collision at all and falls through the floor unopposed, dragging the rest
	 * of the rig with it -- which is what happens as soon as the legs stop
	 * being the first thing under. See SOLVER_DEBUG_LOG.md entry 025.
	 */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Contact")
	bool bAllBodiesCollideWithGround = true;

	/** Sphere radius for structural points on bodies whose bone has no authored Radius. Authoring real radii in the mass asset is strictly better than tuning this. */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Contact", meta = (ClampMin = "0.0"))
	float StructuralContactRadius = 10.0f;

	// ----- Joint limits, solved as constraint rows alongside contact -----

	/**
	 * Solve joint limits as constraint rows in the same sequential-impulse loop
	 * as contact (MuJoCo/PhysX/Box2D style) rather than relying on the
	 * post-integration position clamp alone.
	 *
	 * Turning this OFF returns to the pre-2026-08-14 behaviour, where a leg
	 * folded onto its stops was still modelled as freely folding -- measured at
	 * 14 kg -> 17 kg articulated effective mass at a foot under a 6170 kg
	 * creature, which is why the feet went through the floor. Kept as a switch
	 * so the two can be compared rather than argued about. See
	 * SOLVER_DEBUG_LOG.md entry 023.
	 */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Contact|Joint Limits")
	bool bSolveJointLimitsAsConstraints = true;

	/**
	 * Stiffness of the joint stop, in Hz. Higher than ContactHertz on purpose: a
	 * joint's end of travel is harder than the ground.
	 *
	 * NOTE (2026-08-21): 60 is already AT the internal ceiling of 0.25/SubstepDt, so
	 * this cannot be raised at the shipped substep rate. The re-fit took ContactHertz
	 * from 15 to 45, which narrows the intended gap from 4x to 1.33x -- the ordering
	 * still holds, but there is no headroom left to restore it if ContactHertz rises
	 * again. Buying more would mean a smaller PhysicsSubstepDt. See O-13.
	 */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Contact|Joint Limits")
	float JointLimitHertz = 60.0f;

	UPROPERTY(EditAnywhere, Category = "Muto RL|Contact|Joint Limits")
	float JointLimitDampingRatio = 5.0f;

	/** Overshoot tolerated before the row pushes back, in degrees -- the joint-space analogue of ContactSlop. */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Contact|Joint Limits")
	float JointLimitSlopDeg = 0.25f;

	/** How far before the stop the row engages, in degrees. With the speculative term this is what makes tunnelling impossible; see FJointLimitParams::MarginDeg. */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Contact|Joint Limits")
	float JointLimitMarginDeg = 3.0f;

	// ----- Limb-vs-limb collision, solved in the SAME iteration as ground + joint limits -----
	//
	// Off by default: needs to be validated in isolation (see
	// CreatureGroundContact.h's FLimbCollisionParams / BuildMutoLimbCollisionPairs)
	// before it runs live during a real training session, the same caution
	// every other contact-tuning knob in this file has needed historically.

	/**
	 * Push limbs apart from each other, not just off the ground. LIMB-VS-LIMB ONLY
	 * -- the torso and spine never participate (see BuildMutoLimbCollisionPairs),
	 * and a limb's own bodies never collide with themselves (same BodyLimbIndex is
	 * always skipped).
	 *
	 * NOW ON BY DEFAULT (was off). It had been left off pending validation in
	 * isolation, which it has since had -- three permanent tests
	 * (AgentSolver.LimbCollision.*) cover the pair-build exclusions and the
	 * separation behaviour. Meanwhile ContactIterations had already been raised
	 * 8 -> 16 specifically to give this feature enough Gauss-Seidel budget, so
	 * every training run was paying its cost while getting none of its benefit.
	 */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Contact|Limb Collision")
	bool bEnableLimbCollision = true;

	/** Same role as ContactHertz, but its own tunable -- limb-limb stiffness is not expected to need the same value as ground stiffness. */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Contact|Limb Collision")
	float LimbCollisionHertz = 30.0f;

	UPROPERTY(EditAnywhere, Category = "Muto RL|Contact|Limb Collision")
	float LimbCollisionDampingRatio = 10.0f;

	/** Penetration left uncorrected so two resting limbs settle instead of buzzing -- the limb-pair analogue of ContactSlop. */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Contact|Limb Collision")
	float LimbCollisionSlop = 0.5f;

	UPROPERTY(EditAnywhere, Category = "Muto RL|Contact|Limb Collision")
	float LimbCollisionFrictionCoefficient = 0.8f;

	/** CFM regularization on each limb-pair row. See ContactCfm. */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Contact|Limb Collision", meta = (ClampMin = "0.0"))
	float LimbCollisionCfm = 1.0e-8f;

	/** SOR under-relaxation on each limb-pair row. See ContactRelaxation. */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Contact|Limb Collision", meta = (ClampMin = "0.05", ClampMax = "1.0"))
	float LimbCollisionRelaxation = 1.0f;

	/** Ceiling on the limb-pair rows' push-out velocity, cm/s. */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Contact|Limb Collision", meta = (ClampMin = "0.0"))
	float LimbCollisionMaxBiasVelocity = 100.0f;

	// ----- Passive joint properties (MuJoCo's `armature` and `damping`) -----
	//
	// Both are the highest-payoff-per-line adoptions from MuJoCo, and both are
	// expressed in UNIT-FREE form on purpose: this solver runs in cm-kg-s while
	// every published MuJoCo value assumes SI metres, and transplanting an
	// absolute coefficient across that boundary is precisely how this project
	// ended up with a torque limit ~7 million times too small and a contact spring
	// 143x too weak (SOLVER_DEBUG_LOG.md entries 001 and 017). See
	// FCreatureTopology::DOFArmatureRatio / DOFDampingTimeConstant.

	/**
	 * Added rotor inertia on every joint, as a FRACTION of that joint's own
	 * articulated inertia: D_effective = D * (1 + ratio).
	 *
	 * This is the standard stabilizer for RL rigs and the main reason MuJoCo
	 * locomotion models are well-behaved at a few hundred Hz with ~10 solver
	 * iterations. D is the denominator of every joint acceleration and of every
	 * articulated effective mass the contact solver queries, so raising it lowers
	 * the stiffness of the whole coupled system -- which is the one lever entry
	 * 024's sweep did NOT rule out. That sweep refuted convergence RATE; system
	 * conditioning is a different axis.
	 *
	 * 0.05 (5%) is a deliberately small starting point: enough to help
	 * conditioning, small enough not to visibly slow the rig. It is fictitious
	 * inertia, so more is not better -- it damps real dynamics too.
	 */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Simulation|Passive Joints", meta = (ClampMin = "0.0"))
	float JointArmatureRatio = 0.05f;

	/**
	 * Passive viscous damping on every joint, as a TIME CONSTANT in seconds: an
	 * isolated joint loses the fraction (substep dt / this) of its own velocity per
	 * substep. 0 disables it.
	 *
	 * Unconditionally stable by construction -- the fraction is clamped at 1, so
	 * damping can remove all of a joint's velocity but never reverse it, which is
	 * the failure mode MuJoCo's `implicitfast` integrator exists to remove. No
	 * d*dt/D < 2 bound to respect.
	 *
	 * Why it matters here specifically: between its stops a ball joint in this
	 * solver is a frictionless pendulum with zero dissipation anywhere in its
	 * range, and a measured passive drop showed Head2/LowerMouth climbing from ~0
	 * to 97 and 77 rad/s in 200 ms before the rig diverged. Ball-joint cone limit
	 * rows were added to absorb that AT THE STOP; damping absorbs it across the
	 * whole range, which is both more direct and what MuJoCo relies on.
	 *
	 * 1.0 s is gentle -- 0.4% of joint velocity per substep at 240 Hz, which
	 * barely touches a one-second fall but removes ~63% of joint energy per
	 * simulated second and so cannot sustain a resonance.
	 */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Simulation|Passive Joints", meta = (ClampMin = "0.0"))
	float JointDampingTimeConstant = 1.0f;

	// ----- Solver convergence diagnostics -----

	/**
	 * Body index whose accumulated ground-contact normal impulse is recorded once
	 * per solver iteration, or INDEX_NONE for none. Paired with WatchJointLimitDOF
	 * this answers the question entry 024 left open -- whether a body that is
	 * simultaneously grounded and against its own joint stop is converging or
	 * oscillating -- which stayed open partly because the instrument for it
	 * (CreatureGroundContact::FIterationDebugLog) was written and never wired to
	 * anything.
	 *
	 * Costs nothing when both watches are INDEX_NONE: no log object is created and
	 * nothing is recorded in the hot path.
	 */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Diagnostics")
	int32 WatchContactBody = INDEX_NONE;

	/** DOF index whose accumulated joint-limit impulse is recorded per iteration. See WatchContactBody. */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Diagnostics")
	int32 WatchJointLimitDOF = INDEX_NONE;

	/**
	 * Log the global solve's per-iteration constraint residual (max |Cdot + bias|
	 * over all rows) for env 0, every this many substeps. <= 0 disables.
	 *
	 * This number is only obtainable from the assembled system -- the per-row path
	 * never forms a residual, which is a large part of why "is it converging?"
	 * remained unanswered through entries 016-024. A residual that falls
	 * monotonically toward zero means the solve is working; one that plateaus or
	 * oscillates is the direct evidence that was missing.
	 */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Diagnostics", meta = (ClampMin = "0"))
	int32 LogSolverResidualEverySubsteps = 0;

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
	 * Timeout bumped from the engine default (10s), originally to 60s for the
	 * very first handshake (the Python subprocess has to import torch and
	 * friends on a cold start, and a too-short timeout there looks identical
	 * to a hard failure), then to 900s (2026-08-12) for a much more serious
	 * reason: this single value governs EVERY communication with the trainer,
	 * not just the handshake -- including the ReceiveNetworks call that waits
	 * for a full PPO iteration to finish on the Python side.
	 *
	 * A real training run measured that wait at ~375s (10:45:39 -> 10:51:54 in
	 * the log, one PPO iteration over MaximumRecordedStepsPerIteration=10000
	 * steps), i.e. more than 6x past the old 60s timeout. Blowing that
	 * timeout is not a recoverable "try again" -- it sets bHasTrainingFailed
	 * and calls EndTraining() -> DoneTraining() -> RevertGameSettings(), which
	 * CRASHES THE EDITOR when reached from the background training thread (see
	 * RunTrainingThreadLoop's comment for the full mechanism). So this timeout
	 * must comfortably exceed the slowest expected PPO iteration.
	 *
	 * Tradeoff: if the Python process genuinely dies, the training thread sits
	 * blocked in this wait, and EndPlay's TrainingThreadFuture.Wait() blocks
	 * with it -- stopping PIE can hang for up to this long. That's strictly
	 * better than a guaranteed editor crash on every slow iteration, but it's
	 * why this isn't set even higher. If PPO iterations get slower (more envs,
	 * bigger networks, larger MaximumRecordedStepsPerIteration), raise this to
	 * match rather than leaving it to fire.
	 */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Training", meta = (EditCondition = "!bUseSocketCommunicator"))
	FLearningAgentsSharedMemoryCommunicatorSettings SharedMemoryCommunicatorSettings;

	/**
	 * The default (shared-memory) communicator uses Windows named shared
	 * memory, which can fail with a handshake/timeout error on some
	 * machine configs (session/privilege mismatches between the editor and
	 * the spawned Python subprocess, AV interference, etc. -- see project
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

	// ----- Network assets -- separate Load/Save trios on purpose (2026-08-21):
	// a single shared trio would force overwriting your seed checkpoint the
	// moment you save trained results, since Save and Load would target the
	// same asset. Save writes Policy's live weights out; Load hot-swaps
	// Policy's weights in -- see SaveTrainedNetworksToAssets/
	// LoadTrainedNetworksFromAssets. Any slot left unassigned is skipped.

	/** Encoder network asset LoadTrainedNetworksFromAssets reads from. */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Learning")
	TObjectPtr<ULearningAgentsNeuralNetwork> LoadEncoderNetworkAsset;

	/** Policy network asset LoadTrainedNetworksFromAssets reads from. */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Learning")
	TObjectPtr<ULearningAgentsNeuralNetwork> LoadPolicyNetworkAsset;

	/** Decoder network asset LoadTrainedNetworksFromAssets reads from. */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Learning")
	TObjectPtr<ULearningAgentsNeuralNetwork> LoadDecoderNetworkAsset;

	/** Encoder network asset SaveTrainedNetworksToAssets writes to. */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Learning")
	TObjectPtr<ULearningAgentsNeuralNetwork> SaveEncoderNetworkAsset;

	/** Policy network asset SaveTrainedNetworksToAssets writes to. */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Learning")
	TObjectPtr<ULearningAgentsNeuralNetwork> SavePolicyNetworkAsset;

	/** Decoder network asset SaveTrainedNetworksToAssets writes to. */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Learning")
	TObjectPtr<ULearningAgentsNeuralNetwork> SaveDecoderNetworkAsset;

	/**
	 * Optional curriculum: if set, StartTraining() applies its entry node's
	 * Params (a UAgentSolverPreset) onto this actor before anything else
	 * runs, instead of whatever was left in the Details panel/a manually
	 * loaded preset. From then on, TickLearningProgramTransitions() (called
	 * once per training step from RunOneTrainingStep) watches the current
	 * node's Transitions and hot-swaps to the next node's Params the same
	 * way -- see LearningProgram.h's top comment for the full design.
	 *
	 * Leave unset to train exactly as before (single preset/Details-panel
	 * values, no automatic mid-run parameter changes).
	 */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Learning Program")
	TObjectPtr<ULearningProgram> ActiveLearningProgram;

	/** If true and a snapshot exists in NetworkSnapshotDirectory, StartTraining loads it into the freshly created Policy before the first training step -- resumes a previous run instead of starting from random weights. */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Training")
	bool bLoadSnapshotOnStart = false;

	/** If true, EndPlay saves the current networks to NetworkSnapshotDirectory before tearing down -- so simply stopping PIE never silently discards trained progress (see SaveTrainedNetworks's comment: this is otherwise the ONLY thing that persists trained weights at all). */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Training")
	bool bAutoSaveOnEndPlay = true;

	/**
	 * Periodically calls SaveTrainedNetworks() from the training thread while
	 * running (real wall-clock seconds, not sim time) -- independent of
	 * bAutoSaveOnEndPlay/EndPlay, since a hard engine assertion (e.g. a NaN
	 * that reaches Learning Agents' own array-finite check) terminates the
	 * process without running EndPlay at all, so that safety net alone
	 * cannot protect a long unattended run against losing everything to a
	 * crash. <= 0 disables periodic autosave.
	 */
	UPROPERTY(EditAnywhere, Category = "Muto RL|Training")
	float AutoSaveIntervalSeconds = 300.0f;

	// ----- Simulation state (plain C++, not UObjects -- the batched solver, exactly as used
	// by AgentSolver's automation tests) -----
	FCreatureBatchState Batch;
	FCreatureABASolver Solver;
	TArray<CreatureGroundContact::FContactPointDef> ContactPoints;
	TArray<CreatureGroundContact::FContactPointState> ContactStates;
	// Accumulated contact impulses, persisted across steps -- this IS the warm
	// start, so it must outlive a single StepPhysicsSubstepped call. Unused by
	// the penalty path.
	CreatureGroundContact::FImpulseContactCache ContactImpulseCache;
	// Static, all-different-limb candidate list -- built once in StartTraining()
	// (see BuildMutoLimbCollisionPairs), reused every substep. Per-step
	// broadphase (which candidates actually overlap right now) happens inside
	// ResolveGroundContactImpulses since poses change every substep.
	TArray<CreatureGroundContact::FLimbPairDef> LimbCollisionPairs;
	/**
	 * Per-body-per-env weld set for bWeldSaturatedJoints, rebuilt each substep by
	 * FCreatureABASolver::BuildSaturatedJointLocks. A member rather than a local
	 * because it is rebuilt every substep for every env.
	 */
	TArray<uint8> SaturatedJointLocks;
	CreatureRLEnvironment::FEnvConfig Config;
	FVector StandingTorsoPos = FVector::ZeroVector;
	FQuat StandingTorsoRot = FQuat::Identity;
	FRandomStream ResetStream;
	TArray<FName> BodyDebugNames; // body index -> bone name (index 0 is the synthetic "Torso" label, not a real bone)

	// ----- Imitation runtime state -----
	//
	// Baked once by StartTraining on the game thread, then READ-ONLY for the
	// rest of the run -- every env shares this one copy. Animation evaluation
	// is not safe to run from the background training thread, and re-sampling
	// per step would be pure waste when the result is a few hundred KB of
	// plain floats.
	CreatureImitation::FReferenceMotion ReferenceMotionBaked;

	/**
	 * Per-env phase offset into the reference clip, drawn fresh at each
	 * episode reset (reference state initialization). Sized NumEnvs.
	 *
	 * Written by ResetAgentEpisode and read by GatherAgentReward/
	 * GatherAgentObservation -- all on the same background training thread,
	 * one env at a time, so no synchronization is needed beyond the
	 * TrainingStepLock the step itself already takes.
	 */
	TArray<float> EnvPhaseOffset;

	/** Simulated seconds elapsed in each env's current episode, used with EnvPhaseOffset to index the clip. Advanced once per training step. */
	TArray<float> EnvEpisodeTime;

	/**
	 * Height the torso stands at in the rest pose -- the baseline the
	 * reference frame's own rest-relative height is applied on top of (see
	 * CreatureImitation::FReferenceFrame::RootHeightAboveRest). Equal to
	 * Config.TargetTorsoHeight; kept as its own name so the root reward reads
	 * as what it means rather than borrowing an unrelated field.
	 */
	float RestTorsoHeight = 100.0f;

	/** Fills OutTarget for one env at the current episode time, sampling ReferenceMotionBaked by phase. Returns false (leaving Frame null) when there is no baked reference, which makes every consumer fall back to the standing objective. */
	bool BuildImitationTarget(int32 EnvIndex, CreatureRLEnvironment::FImitationTarget& OutTarget, CreatureImitation::FReferenceFrame& OutFrameStorage) const;

	/** Phase in [0,1) for one env right now -- what the phase observation encodes. Zero when there is no baked reference or the clip is a single pose. */
	float GetEnvPhase(int32 EnvIndex) const;

	/** Copies the live-tunable imitation weights/falloffs from the UPROPERTYs into Config, so Details-panel edits take effect without restarting training. Called once per agent per step, alongside the other tuning knobs. */
	void ApplyLiveImitationTuning();

	/**
	 * Per-episode imitation reset: draws this env's starting phase (random for
	 * a full clip, zero for a single pose) and, when bResetToReferencePose,
	 * writes the reference frame into the env's joint state.
	 *
	 * Called from ResetAgentEpisode AFTER CreatureRLEnvironment::ResetEnv, so
	 * it overwrites the rest pose that call just wrote rather than being
	 * overwritten by it. No-ops entirely outside imitation mode.
	 */
	void ResetImitationEpisode(int32 EnvIndex);

	/** Advances every env's episode clock by DeltaSeconds, so the reference phase tracks simulated time. Called once per training step, not once per agent. */
	void AdvanceImitationClock(float DeltaSeconds);

	// ----- UI status readouts (see GetAverageReward/GetTrainingStepCount/GetEpisodeCount) -----
	// Public and atomic, not UPROPERTY -- written from the background training
	// thread (UMutoRLTrainingEnvironment/RunOneTrainingStep, both in this same
	// .cpp), read from the game thread by the control panel's Slate bindings.
	// Single-writer/multi-reader, so plain atomics (no lock) are sufficient --
	// same reasoning as ConcurrentStepDepth below.
	std::atomic<float> AverageReward{0.0f};
	std::atomic<int32> TrainingStepCount{0};
	std::atomic<int32> EpisodeCount{0};
	// Same EMA/single-writer pattern as AverageReward, one per ComputeReward
	// component -- see GetLastTorsoHeightBonus/GetLastEnergyConsumptionMalus/
	// GetLastMusclesUseMalus.
	std::atomic<float> LastTorsoHeightBonus{0.0f};
	std::atomic<float> LastEnergyConsumptionMalus{0.0f};
	std::atomic<float> LastMusclesUseMalus{0.0f};
	// Imitation's four terms, same pattern again. Each is in (0,1] by
	// construction, so they read directly on the control panel's line graphs
	// with no rescaling -- and a pose reward pinned near 1.0 right after a
	// reset is the single fastest way to confirm the retarget is correct.
	std::atomic<float> LastPoseReward{0.0f};
	std::atomic<float> LastVelocityReward{0.0f};
	std::atomic<float> LastEndEffectorReward{0.0f};
	std::atomic<float> LastRootReward{0.0f};

protected:
	/**
	 * Set as the very first statement in StartTraining(), before any asset
	 * loading/topology building/Manager creation -- the "already started"
	 * guard checks THIS, not Trainer. Trainer isn't assigned until near the
	 * end of StartTraining() (after Manager/Interactor/Agents already
	 * exist), so a re-entrant or duplicate call arriving while an earlier
	 * call is still mid-setup would sail past a Trainer-based guard and
	 * build a SECOND Manager + a second full set of NumEnvs agents. Found
	 * while investigating a "Tried to add experience from episode that is
	 * still running..." crash whose log showed the same AgentId appearing
	 * twice in a single LAManager::ResetAgents call -- direct evidence of a
	 * duplicate agent registration; a double StartTraining() call is the
	 * most plausible source of that. Not proven to be the exact trigger,
	 * but closing this window is correct regardless.
	 */
	bool bStartTrainingCalled = false;

	// Transient -- all six are created fresh inside StartTraining() every
	// session, never meant to outlive it. Without Transient, whatever these
	// point to when the level happens to be saved gets serialized straight
	// into the .umap on disk -- and since Interactor/TrainingEnvironment's
	// actual runtime class (ULearningAgentsInteractor/
	// ULearningAgentsTrainingEnvironment's own concrete Muto subclasses) is
	// declared UCLASS(Abstract) at the ENGINE base level, reloading that
	// baked-in reference on the next PIE start fails with "Class which was
	// marked abstract was trying to be loaded ... It will be nulled out on
	// save" (LogUObjectGlobals) -- confirmed present on both
	// MutoRLTrainingDriver_1 and MutoRLVisualizerActor_0 every single PIE
	// start, meaning TrainingLvl.umap already has one of these baked in from
	// a save made while training was active.
	UPROPERTY(Transient)
	TObjectPtr<ULearningAgentsManager> Manager;

	UPROPERTY(Transient)
	TObjectPtr<ULearningAgentsInteractor> Interactor;

	UPROPERTY(Transient)
	TObjectPtr<ULearningAgentsPolicy> Policy;

	UPROPERTY(Transient)
	TObjectPtr<ULearningAgentsCritic> Critic;

	UPROPERTY(Transient)
	TObjectPtr<ULearningAgentsTrainingEnvironment> TrainingEnvironment;

	UPROPERTY(Transient)
	TObjectPtr<ULearningAgentsPPOTrainer> Trainer;

	// Same reasoning as Manager/Interactor/Policy/etc. above -- up to NumEnvs
	// (256) plain UObjects created fresh in StartTraining(), never meant to
	// be baked into the level.
	UPROPERTY(Transient)
	TArray<TObjectPtr<UObject>> AgentObjects;

	TArray<int32> AgentIdToEnvIndex;

	/**
	 * Rest-pose standing height estimate: walks the parent chain from one
	 * contact point's body up to the torso, composing each body's REAL
	 * bind-pose rotation (BodyRestRotInParent) and joint offset -- the same
	 * relation the solver's own forward kinematics reduces to at
	 * JointPos==0 -- to find how far below StandingTorsoRotIn (in world Z)
	 * that contact point sits at rest. Override via TargetTorsoHeightOverride
	 * if it still looks wrong for the real rig.
	 */
	// Public (rather than protected like the members around it) purely so that
	// diagnostics/tests can reproduce the driver's EXACT standing pose instead
	// of approximating it -- it is a pure function of its arguments and touches
	// no instance state, so exposing it carries no risk.
public:
	static float ComputeDefaultStandingHeight(const FCreatureTopology& Topo, const TArray<CreatureGroundContact::FContactPointDef>& InContactPoints, const FQuat& StandingTorsoRotIn);

protected:
	/** See GetNetworkAccessLock(). */
	FCriticalSection NetworkAccessLock;

	/**
	 * Serializes the whole of RunOneTrainingStep(). Added 2026-08-12 while
	 * chasing the recurring "Tried to add experience from episode that is
	 * still running..." assert (LearningExperience.cpp:401).
	 *
	 * Evidence that this is a concurrency problem, not the duplicate-agent-
	 * registration theory that the bStartTrainingCalled fix was built around
	 * (that theory is now DISPROVEN -- the registry guard in RunOneTrainingStep
	 * checks GetAllAgentIds().Num() every step and has never once fired, so
	 * the Manager's OccupiedAgentIds is clean):
	 *   - Our own GatherAgentCompletion callback logs every terminating agent
	 *     TWICE, with identical AgentId AND EnvIndex.
	 *   - The engine's own ULearningAgentsManager::ResetAgents logs
	 *     "Resetting Agents [47 47 111 111]" -- a SINGLE call. That function
	 *     does OnEventAgentIds.Reset() and then one Add() per passed id, and
	 *     OnEventAgentIds is a MEMBER array, so a clean [47, 111] input can
	 *     only print doubled if two threads are interleaving inside it.
	 * Both are explained exactly by two threads running RunTraining()
	 * concurrently against the same Trainer/Manager, which also races the
	 * completion state that AddEpisodes then asserts on.
	 *
	 * Where the second thread comes from is still UNKNOWN -- StartTraining is
	 * guarded (bStartTrainingCalled) and launches exactly one Async thread.
	 * ConcurrentStepDepth below is what will actually answer that; this lock
	 * makes the corruption impossible in the meantime.
	 */
	FCriticalSection TrainingStepLock;

	/**
	 * Guards ONLY CurrentLearningProgramNodeId/NodeEntryTrainingStepCount --
	 * see GetLearningProgramLiveState()'s comment for why this is a separate
	 * lock from TrainingStepLock rather than reusing it: TrainingStepLock is
	 * held for a whole (potentially minutes-long) training step, and taking
	 * it from the node graph editor's polling Tick() would freeze the editor
	 * for that long. This lock is only ever held for a trivial copy/assign
	 * on both sides.
	 */
	FCriticalSection LearningProgramStateLock;

	/**
	 * Re-entrancy detector for RunOneTrainingStep() -- incremented on entry,
	 * decremented on exit, BEFORE TrainingStepLock is taken, so it observes
	 * would-be concurrency rather than being hidden by the lock. Any value
	 * above 1 proves a second thread is in there and logs both thread ids.
	 * See TrainingStepLock's comment.
	 */
	std::atomic<int32> ConcurrentStepDepth{0};

	/** NetworkSnapshotDirectory.Path, or ProjectSavedDir/MutoRL/Snapshots if that's empty. */
	FString GetSnapshotDirectory() const;

	/** FPlatformTime::Seconds() timestamp of the last periodic autosave (see AutoSaveIntervalSeconds); set in StartTraining(). */
	double LastAutoSaveTime = 0.0;

	/**
	 * Clears external forces, applies ground contact, and steps Batch/Solver
	 * repeatedly at (at most) PhysicsSubstepDt each, covering TotalDt overall
	 * -- see PhysicsSubstepDt's comment for why this exists. TotalDt is
	 * clamped to a sane max per call (avoids a substep storm after an editor
	 * hitch, e.g. in the visualizer's real-wall-clock DeltaTime). Shared by
	 * RunOneTrainingStep() (TotalDt = FixedDt) and AMutoRLVisualizerActor::
	 * Tick() (TotalDt = frame DeltaTime) -- operates on whichever Batch/
	 * Solver/ContactPoints/Config the calling instance owns.
	 */
	void StepPhysicsSubstepped(float TotalDt);

	/**
	 * Writes JointArmatureRatio / JointDampingTimeConstant into every DOF of a
	 * freshly built topology. Shared by all three StartTraining implementations
	 * (this one, AMutoRLVisualizerActor's and AMutoRagdollVisualizerActor's) --
	 * each replaces StartTraining entirely rather than calling Super, which is
	 * exactly how LimbCollisionPairs ended up silently unpopulated in both
	 * visualizers, so anything the topology needs from driver settings belongs in
	 * one shared place.
	 *
	 * These are tuning knobs, not authored rig data, which is why they are applied
	 * here rather than inside BuildMutoTopology: the mass/muscle assets have no
	 * field for either, and inventing one would imply a per-bone source that does
	 * not exist. Uniform across DOFs for the same reason.
	 */
	void ApplyPassiveJointDefaults(FCreatureTopology& Topo) const;

	/** Fills every params struct StepPhysicsSubstepped needs from this actor's properties. One place, so all three subclasses agree. */
	void BuildContactParams(
		CreatureGroundContact::FImpulseContactParams& OutContact,
		CreatureGroundContact::FJointLimitParams& OutLimits,
		CreatureGroundContact::FLimbCollisionParams& OutLimbCollision) const;

	/**
	 * One RunTraining()+physics-step iteration. Returns false if training
	 * failed or has completed (caller should stop calling it) -- shared by
	 * StartTraining()'s one synchronous call and RunTrainingThreadLoop().
	 */
	bool RunOneTrainingStep();

	/** Calls RunOneTrainingStep() in a loop until it returns false or a stop is requested; runs entirely off the game thread (see StartTraining()). */
	void RunTrainingThreadLoop();

	/** Shared body of EndPlay/StopTraining -- see StopTraining()'s comment. */
	void StopTrainingInternal();

	/**
	 * Applies Node->Params to this actor (UAgentSolverPreset::ApplyToDriver)
	 * and resets NodeEntryTrainingStepCount to the current live counter, so
	 * the next node's StepsSinceNodeEntry transitions measure progress
	 * from THIS point, not from the start of training. No-ops if Node is
	 * null. Called once from StartTraining (the entry node) and again from
	 * TickLearningProgramTransitions whenever a transition fires.
	 */
	void ApplyLearningProgramNode(ULearningProgramNode* Node);

	/**
	 * Evaluates CurrentLearningProgramNode's Transitions against how far
	 * EpisodeCount/TrainingStepCount have advanced since it was entered, and
	 * calls ApplyLearningProgramNode on the first one whose threshold is met
	 * (order in the Transitions array breaks ties). No-ops entirely when
	 * ActiveLearningProgram is unset. Called once per training step from
	 * RunOneTrainingStep, on the background training thread -- same thread
	 * GatherAgentReward_Implementation already re-syncs these exact
	 * UPROPERTYs into Config from, so writing them here needs no additional
	 * locking (see that function's comment).
	 */
	void TickLearningProgramTransitions();

	/** NodeId of whichever ULearningProgramNode is currently active, or an invalid Guid if ActiveLearningProgram is unset. Background-thread-only, like the two functions above. */
	FGuid CurrentLearningProgramNodeId;

	/** TrainingStepCount snapshotted at the moment CurrentLearningProgramNodeId was entered -- see ELearningProgramConditionType::StepsSinceNodeEntry's comment for why that condition measures progress since node entry, not since training started. */
	int32 NodeEntryTrainingStepCount = 0;

	std::atomic<bool> bStopTrainingThreadRequested{false};
	TFuture<void> TrainingThreadFuture;
};
