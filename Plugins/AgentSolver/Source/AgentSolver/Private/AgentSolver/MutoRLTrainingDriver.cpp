#include "AgentSolver/MutoRLTrainingDriver.h"

#include "LearningAgentsManager.h"
#include "LearningAgentsObservations.h"
#include "LearningAgentsActions.h"
#include "LearningAgentsCompletions.h"
#include "LearningAgentsNeuralNetwork.h"
#include "Async/Async.h"
#include "Misc/ScopeLock.h"
#include "Misc/Paths.h"
#include "HAL/FileManager.h"
#include "HAL/PlatformTime.h"

#if WITH_EDITOR
#include "PhysicsSolver/MutoTopology.h"
#include "Engine/SkeletalMesh.h"
#endif

// ================================ UMutoRLInteractor ================================

void UMutoRLInteractor::SpecifyAgentObservation_Implementation(FLearningAgentsObservationSchemaElement& OutObservationSchemaElement, ULearningAgentsObservationSchema* InObservationSchema)
{
	const AMutoRLTrainingDriver* Driver = GetTypedOuter<AMutoRLTrainingDriver>();
	check(Driver);
	const int32 ObservationSize = CreatureRLEnvironment::GetObservationSize(Driver->Batch.GetTopology(), Driver->ContactPoints.Num());
	OutObservationSchemaElement = ULearningAgentsObservations::SpecifyContinuousObservation(InObservationSchema, ObservationSize);
}

void UMutoRLInteractor::GatherAgentObservation_Implementation(FLearningAgentsObservationObjectElement& OutObservationObjectElement, ULearningAgentsObservationObject* InObservationObject, const int32 AgentId)
{
	AMutoRLTrainingDriver* Driver = GetTypedOuter<AMutoRLTrainingDriver>();
	check(Driver);
	const int32 EnvIndex = Driver->GetEnvIndexForAgent(AgentId);

	TArray<float> Observation;
	CreatureRLEnvironment::ComputeObservations(Driver->Batch, EnvIndex, Driver->Config, Driver->ContactPoints, Driver->ContactStates, Driver->Batch.GetNumEnvs(), Observation);
	OutObservationObjectElement = ULearningAgentsObservations::MakeContinuousObservation(InObservationObject, Observation);
}

void UMutoRLInteractor::SpecifyAgentAction_Implementation(FLearningAgentsActionSchemaElement& OutActionSchemaElement, ULearningAgentsActionSchema* InActionSchema)
{
	const AMutoRLTrainingDriver* Driver = GetTypedOuter<AMutoRLTrainingDriver>();
	check(Driver);
	OutActionSchemaElement = ULearningAgentsActions::SpecifyContinuousAction(InActionSchema, Driver->Batch.GetTopology().NumDOF);
}

void UMutoRLInteractor::PerformAgentAction_Implementation(const ULearningAgentsActionObject* InActionObject, const FLearningAgentsActionObjectElement& InActionObjectElement, const int32 AgentId)
{
	AMutoRLTrainingDriver* Driver = GetTypedOuter<AMutoRLTrainingDriver>();
	check(Driver);
	const int32 EnvIndex = Driver->GetEnvIndexForAgent(AgentId);

	TArray<float> Actions;
	ULearningAgentsActions::GetContinuousAction(Actions, InActionObject, InActionObjectElement);
	CreatureRLEnvironment::ApplyActions(Driver->Batch, EnvIndex, Actions, Driver->Config);
}

// ============================= UMutoRLTrainingEnvironment =============================

void UMutoRLTrainingEnvironment::GatherAgentReward_Implementation(float& OutReward, const int32 AgentId)
{
	AMutoRLTrainingDriver* Driver = GetTypedOuter<AMutoRLTrainingDriver>();
	check(Driver);
	const int32 EnvIndex = Driver->GetEnvIndexForAgent(AgentId);
	float TorsoHeightBonus = 0.0f;
	float EnergyConsumptionMalus = 0.0f;
	float MusclesUseMalus = 0.0f;
	OutReward = CreatureRLEnvironment::ComputeReward(Driver->Batch, EnvIndex, Driver->Config, Driver->ContactPoints, Driver->ContactStates, Driver->Batch.GetNumEnvs(),
		&TorsoHeightBonus, &EnergyConsumptionMalus, &MusclesUseMalus);

	// See AverageReward's comment -- slow-moving EMA (alpha small on purpose:
	// this runs once per agent per step, i.e. up to NumEnvs samples per
	// training step, so a large alpha would make the UI readout jitter with
	// the per-agent noise instead of showing a trend).
	static constexpr float RewardEmaAlpha = 0.01f;
	const float PreviousAverage = Driver->AverageReward.load(std::memory_order_relaxed);
	Driver->AverageReward.store(FMath::Lerp(PreviousAverage, OutReward, RewardEmaAlpha), std::memory_order_relaxed);

	// Same EMA treatment for the Reward Settings pane's visualization graphs
	// -- see LastTorsoHeightBonus/LastEnergyConsumptionMalus/LastMusclesUseMalus.
	Driver->LastTorsoHeightBonus.store(FMath::Lerp(Driver->LastTorsoHeightBonus.load(std::memory_order_relaxed), TorsoHeightBonus, RewardEmaAlpha), std::memory_order_relaxed);
	Driver->LastEnergyConsumptionMalus.store(FMath::Lerp(Driver->LastEnergyConsumptionMalus.load(std::memory_order_relaxed), EnergyConsumptionMalus, RewardEmaAlpha), std::memory_order_relaxed);
	Driver->LastMusclesUseMalus.store(FMath::Lerp(Driver->LastMusclesUseMalus.load(std::memory_order_relaxed), MusclesUseMalus, RewardEmaAlpha), std::memory_order_relaxed);
}

void UMutoRLTrainingEnvironment::GatherAgentCompletion_Implementation(ELearningAgentsCompletion& OutCompletion, const int32 AgentId)
{
	AMutoRLTrainingDriver* Driver = GetTypedOuter<AMutoRLTrainingDriver>();
	check(Driver);
	const int32 EnvIndex = Driver->GetEnvIndexForAgent(AgentId);
	// Termination (not Truncation): a fall is treated as "zero future reward
	// expected", not an arbitrary cutoff the critic should try to estimate
	// past — matches the standing/balance objective (there's no natural
	// "ran out of runway" truncation case here, only max-episode-length,
	// which the PPOTrainer/TrainingEnvironment already handle internally
	// via MaxEpisodeStepNum).
	OutCompletion = CreatureRLEnvironment::IsTerminated(Driver->Batch, EnvIndex, Driver->Config,
		&Driver->ContactStates, Driver->ContactPoints.Num(), Driver->Batch.GetNumEnvs())
		? ELearningAgentsCompletion::Termination
		: ELearningAgentsCompletion::Running;

	// Diagnostic for the "Tried to add experience from episode that is still
	// running..." assert (LearningExperience.cpp:401-402, FReplayBuffer::
	// AddEpisodes) recurring in ProcessExperience. Traced the full engine
	// call path (RunTraining -> GatherCompletions -> ProcessExperience ->
	// EvaluateEndOfEpisodeCompletions -> SetAllCompletions ->
	// SetResetInstancesFromCompletions -> GatherObservations -> AddEpisodes)
	// and found no logical inconsistency: the same TrainingEnvironment::
	// AllCompletions array is read to build the reset-instance set and then
	// again inside AddEpisodes moments later, single-threaded, nothing in
	// between but a pure observation gather. This function (our own
	// ELearningAgentsCompletion source, feeding the "AgentCompletions" half
	// of that Or()) is the only piece of the chain we can actually observe —
	// logging every non-Running result (rare; episodes are normally long) so
	// that if this recurs, the log tail shows exactly which AgentId/EnvIndex
	// went non-Running and how many envs/agents existed at that moment,
	// instead of just the bare assert.
	// Thread id and owning-actor name are logged because these lines come out
	// DOUBLED (same AgentId, same EnvIndex) in every crash log so far. The two
	// candidate explanations produce different fingerprints here, which is the
	// whole point of logging them:
	//   - same actor name + DIFFERENT thread ids  -> two threads racing one
	//     driver (see TrainingStepLock), the leading theory;
	//   - DIFFERENT actor names                   -> two AMutoRLTrainingDriver
	//     actors placed in the level, each stepping its own trainer with the
	//     same ResetRandomSeed, so they terminate identically and merely LOOK
	//     like one doubled stream.
	if (OutCompletion != ELearningAgentsCompletion::Running)
	{
		UE_LOG(LogTemp, Log, TEXT("MutoRL: AgentId=%d EnvIndex=%d completion=Termination (NumEnvs=%d, actor=%s, thread=%u)"),
			AgentId, EnvIndex, Driver->Batch.GetNumEnvs(), *Driver->GetName(), FPlatformTLS::GetCurrentThreadId());
	}
}

void UMutoRLTrainingEnvironment::ResetAgentEpisode_Implementation(const int32 AgentId)
{
	AMutoRLTrainingDriver* Driver = GetTypedOuter<AMutoRLTrainingDriver>();
	check(Driver);
	const int32 EnvIndex = Driver->GetEnvIndexForAgent(AgentId);
	CreatureRLEnvironment::ResetEnv(Driver->Batch, EnvIndex, Driver->StandingTorsoPos, Driver->StandingTorsoRot,
		Driver->ResetStream, Driver->PosNoiseStdDev, Driver->AngleNoiseRad, Driver->MakeDomainRandomization());
	// ResetEnv only touches Batch -- ContactStates is a separate array this
	// driver owns, and a non-finite NormalForce that triggered this reset
	// would otherwise survive it untouched (see IsContactStateValid's
	// comment). Per-env, so other envs sharing this same array are
	// unaffected.
	CreatureRLEnvironment::ClearContactStatesForEnv(Driver->ContactStates, Driver->ContactPoints.Num(), EnvIndex, Driver->Batch.GetNumEnvs());

	Driver->EpisodeCount.fetch_add(1, std::memory_order_relaxed);
}

// ================================ AMutoRLTrainingDriver ================================

AMutoRLTrainingDriver::AMutoRLTrainingDriver()
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.bStartWithTickEnabled = false; // enabled once training actually starts (see StartTraining)

	// See SharedMemoryCommunicatorSettings' comment for the full reasoning:
	// this one value governs every trainer communication, including the
	// ReceiveNetworks wait for a whole PPO iteration (measured at ~375s in a
	// real run), and blowing it does not merely fail the step — it reaches
	// RevertGameSettings() on the background training thread and hard-crashes
	// the editor. 900s is chosen to comfortably clear the slowest observed
	// iteration.
	SharedMemoryCommunicatorSettings.Timeout = 900.0f;
	// The socket path (bUseSocketCommunicator) has its own separate timeout
	// that still sat at the engine's 10s default — flipping that switch would
	// otherwise have made the crash above near-instant and constant.
	SocketCommunicatorSettings.Timeout = 900.0f;

	// Off by default in the plugin (FLearningAgentsPPOTrainingSettings::
	// bUseGradNormMaxClipping = false) — turned on here because an
	// unclipped gradient spike (a bad batch, a poorly-calibrated critic
	// early on, etc.) can push the policy/critic/encoder/decoder weights
	// themselves to NaN in a single PPO update. Once that happens EVERY
	// subsequent inference produces NaN output regardless of how clean the
	// input observation is — confirmed to be exactly this failure mode, not
	// a corrupted-observation one: the crash both times fired at
	// LearningNeuralNetwork.cpp:350 (Array::Check(Output), AFTER the
	// network runs), never at line 317 (Array::Check(Input), BEFORE it) —
	// i.e. IsBodyStateValid/IsTerminated's finite-input guard (see
	// CreatureRLEnvironment.h) was working the whole time; the input was
	// never the problem. GradNormMax keeps its plugin default (0.5); raise
	// it if this makes training visibly slower/more timid than it needs to
	// be, lower it if NaN weights still recur.
	TrainingSettings.bUseGradNormMaxClipping = true;

	// Also off by default (0.0f). Gradient-norm clipping above only rescales
	// a gradient that's large-but-FINITE — it cannot fix one that's already
	// NaN, and the SAME crash (see the comment above for how this was
	// diagnosed) recurred a third time even with clipping on. The classic
	// way a PPO gradient becomes NaN in the first place (not just large) is
	// the policy's own predicted action standard deviation collapsing
	// toward zero as it converges — log(std)/1/std terms in the
	// log-probability and entropy math blow up well before std actually
	// reaches literal zero in float32. A small entropy bonus directly
	// discourages that collapse by rewarding the policy for keeping some
	// action noise — the standard PPO fix for exactly this failure mode.
	TrainingSettings.ActionEntropyWeight = 0.01f;

	// Confirmed the crash recurred a FOURTH time even with the two mitigations
	// above (grad-norm clipping + entropy bonus), so this round the actual
	// Python-side trainer (Engine/Plugins/Experimental/LearningAgents/Content/
	// Python/ppo.py and train_common.py — this project has no Python training
	// code of its own; Learning Agents ships and runs its own) was read
	// directly instead of guessing another C++-side knob. Two things
	// confirmed there:
	//  1) independent_normal_log_prob() clips log_std on the UPPER side only
	//     (torch.clip(log_std, None, 10.0)) — there is NO lower bound. If
	//     log_std drifts very negative, std=exp(log_std)->~0 and the
	//     log-prob's (value-mean)^2/(2*std^2) term (and its gradient, which
	//     scales as ~exp(-2*log_std)) can reach +-inf in float32 well before
	//     std is literally zero. That is a real, inherent gap in the engine's
	//     numerics, not something this project can patch (editing engine
	//     source would apply globally and not survive an engine
	//     update/reinstall).
	//  2) The entropy bonus is a SOFT, gradual pressure against that collapse
	//     (loss_ent rewards larger log_std on average) but has no hard floor,
	//     so a single unlucky batch can still push one action dimension's
	//     log_std low enough to blow up before entropy has time to correct
	//     it — consistent with the crash surviving ActionEntropyWeight=0.01f
	//     for a 4th round.
	// action_reg_weight (schema_regularization -> abs(mean)+abs(log_std),
	// see ppo.py) is a SEPARATE, more direct lever: an L1 penalty applied
	// every single step directly on |log_std| itself (not mediated through
	// the entropy formula), so it pushes back on extreme log_std
	// independently of whatever the entropy term is doing. Its plugin
	// default (0.001) was never touched by this project until now; raised
	// 10x here as the next, more targeted line of defense. If NaN weights
	// still recur after this, the next lever to reach for is
	// LearningRatePolicy (currently plugin default 0.0001) — a smaller
	// policy step reduces how far log_std can move in a single bad update,
	// which is complementary to (not a replacement for) this.
	TrainingSettings.ActionRegularizationWeight = 0.01f;
}

float AMutoRLTrainingDriver::ComputeDefaultStandingHeight(const FCreatureTopology& Topo, const TArray<CreatureGroundContact::FContactPointDef>& InContactPoints, const FQuat& StandingTorsoRotIn)
{
	if (InContactPoints.Num() == 0)
	{
		return 100.0f;
	}

	// Take the WORST CASE (largest required height) across EVERY contact
	// point, not just the first one — different limbs (arms vs. legs) have
	// different lengths/rest configurations, so one reference point can
	// undershoot for the others.
	float MaxRequiredHeight = 10.0f;
	for (const CreatureGroundContact::FContactPointDef& Point : InContactPoints)
	{
		// Build the torso->contact-body path (root-adjacent first) so it can
		// be walked forward, composing rotations exactly like the solver's
		// own FK does at JointPos==0 (see CreatureBatchSolver.h Pass 1): at
		// rest, both ball and revolute joints reduce to WorldRot[Body] =
		// WorldRot[Parent] * BodyRestRotInParent[Body] (zero joint angle/
		// identity RelRot). Ignoring these rotations (as this function used
		// to, summing raw local Z offsets) is only exact if every
		// intermediate rest rotation is close to identity — which this
		// rig's real bind pose is not.
		TArray<int32> PathFromTorso;
		int32 Body = Point.BodyIndex;
		while (Body != 0 && Body != INDEX_NONE && Topo.BodyJointOffsetInParent.IsValidIndex(Body))
		{
			PathFromTorso.Add(Body);
			Body = Topo.BodyParent[Body];
		}

		FQuat WorldRot = StandingTorsoRotIn;
		float WorldZOffsetAtBodyOrigin = 0.0f;
		for (int32 i = PathFromTorso.Num() - 1; i >= 0; --i)
		{
			const int32 PathBody = PathFromTorso[i];
			WorldZOffsetAtBodyOrigin += static_cast<float>(WorldRot.RotateVector(Topo.BodyJointOffsetInParent[PathBody]).Z);
			WorldRot = (WorldRot * Topo.BodyRestRotInParent[PathBody]).GetNormalized();
		}
		// WorldRot is now this contact body's own full rest rotation. The
		// point defines a CAPSULE (see FContactPointDef::CapsuleHalfHeight),
		// not a single sphere — LocalOffset (e.g. the "Toe" point sits
		// further out than its body's own origin, toward where the Tip bone
		// attaches) is the tip/END cap, and pulling back CapsuleHalfHeight*2
		// along that same axis gives the START cap. Check both: whichever
		// end is lower determines the worst-case required height (skipping
		// the start cap was systematically most wrong exactly at the tips,
		// before CapsuleHalfHeight existed; skipping the tip once a capsule
		// is long enough to have its start cap lower would be the same
		// mistake in reverse).
		// Shared derivation — this used to inline `LocalOffset.GetSafeNormal()`,
		// which returns the ZERO vector for an interior body's zero LocalOffset and
		// therefore silently collapsed both caps onto the body origin regardless of
		// the authored half height. Benign here (the two candidates were merely
		// equal, so the max was still correct for the tip) but wrong for the same
		// reason the ground gather was wrong, and it would have understated the
		// required height for any body whose capsule reaches lower than its origin.
		// See CreatureGroundContact::GetCapsuleLocalEnds.
		FVector LocalTip, LocalStart;
		CreatureGroundContact::GetCapsuleLocalEnds(Point.LocalOffset, Point.CapsuleHalfHeight, LocalTip, LocalStart);

		const float WorldZOffsetAtTip = WorldZOffsetAtBodyOrigin + static_cast<float>(WorldRot.RotateVector(LocalTip).Z);
		const float WorldZOffsetAtStart = WorldZOffsetAtBodyOrigin + static_cast<float>(WorldRot.RotateVector(LocalStart).Z);

		// The actual ground-touching surface is Point.Radius further down
		// than each cap's center (see FContactPointDef::Radius) — required
		// height must account for the bone's real thickness, not just its
		// center.
		MaxRequiredHeight = FMath::Max(MaxRequiredHeight, -WorldZOffsetAtTip + Point.Radius); // offsets point downward (negative world Z); height is the positive distance
		MaxRequiredHeight = FMath::Max(MaxRequiredHeight, -WorldZOffsetAtStart + Point.Radius);
	}

	// Small deliberate initial penetration, not an exact ground touch:
	// ApplyGroundContactForces' Penetration>0.0f check is strict, so a point
	// placed at EXACTLY Z=0 registers zero contact force until gravity has
	// already pulled it slightly past the ground — i.e. a brief real
	// free-fall gap before contact ever engages, confirmed by direct
	// measurement (a diagnostic run from the exact-Z=0 placement showed
	// near-unimpeded free-fall velocity, ~98 u/s after 0.1s at
	// Gravity=980, matching g*t almost exactly, before the resulting
	// high-velocity first contact contributed to a later numerical
	// blowup). A few units of pre-existing penetration makes the spring
	// engage from the very first step instead.
	constexpr float InitialPenetrationMargin = 2.0f;
	return FMath::Max(10.0f, MaxRequiredHeight - InitialPenetrationMargin);
}

int32 AMutoRLTrainingDriver::GetEnvIndexForAgent(int32 AgentId) const
{
	return AgentIdToEnvIndex.IsValidIndex(AgentId) ? AgentIdToEnvIndex[AgentId] : INDEX_NONE;
}

void AMutoRLTrainingDriver::BeginPlay()
{
	Super::BeginPlay();

	// BeginPlay only ever runs when a world is actually starting play (PIE or
	// packaged game) -- never for the plain editor world, which never calls
	// BeginPlay on its placed actors at all. So it's always correct to clear
	// this here: a stray direct StartTraining() call against the EDITOR-WORLD
	// source object (e.g. the pre-IsPIERunning()-gating "Play button running
	// training against the editor world" bug) would otherwise leave
	// bStartTrainingCalled stuck true on that object for the rest of the
	// editor session -- and since PIE duplicates that object's CURRENT
	// in-memory state at the start of every subsequent session, every future
	// PIE run would inherit the stale flag, hit the guard in StartTraining(),
	// and silently return with no Trainer ever created (TrainingStepCount
	// stuck at 0, IsTrainingActive() permanently false, Stop button never
	// enabling) -- with nothing to naturally reset it, since the poisoned
	// editor-world object's own BeginPlay never fires again to clear it.
	bStartTrainingCalled = false;

	if (bAutoStartOnBeginPlay)
	{
		StartTraining();
	}
}

void AMutoRLTrainingDriver::StartTraining()
{
#if WITH_EDITOR
	// See bStartTrainingCalled's comment — this must be the very first check,
	// before any setup work, and must be set to true before returning below,
	// so a duplicate/re-entrant call is blocked no matter how far a prior
	// call has progressed (a Trainer-based guard alone left a window where
	// Manager/Interactor/Agents could already exist but Trainer did not yet).
	if (bStartTrainingCalled)
	{
		UE_LOG(LogTemp, Warning, TEXT("AMutoRLTrainingDriver::StartTraining: already started, ignoring."));
		return;
	}
	bStartTrainingCalled = true;

	if (!SkeletalMesh || !MassAsset || !MuscleAsset)
	{
		UE_LOG(LogTemp, Error, TEXT("AMutoRLTrainingDriver::StartTraining: SkeletalMesh/MassAsset/MuscleAsset must all be assigned in the Details panel."));
		// Nothing created yet (no Manager/agents) — safe to allow a retry
		// once the missing asset is assigned, unlike the failure paths
		// below this point.
		bStartTrainingCalled = false;
		return;
	}

	// ---- Build the topology + batch (same path MutoTopologyTest.cpp validates) ----
	FCreatureTopology Topo;
	TArray<FString> Warnings;
	BodyDebugNames.Reset();
	if (!MutoTopology::BuildMutoTopology(SkeletalMesh, MassAsset, MuscleAsset, Topo, Warnings, &BodyDebugNames))
	{
		UE_LOG(LogTemp, Error, TEXT("AMutoRLTrainingDriver::StartTraining: BuildMutoTopology failed."));
		// Still nothing created yet — same reasoning as above.
		bStartTrainingCalled = false;
		return;
	}
	for (const FString& Warning : Warnings)
	{
		UE_LOG(LogTemp, Warning, TEXT("AMutoRLTrainingDriver: BuildMutoTopology warning: %s"), *Warning);
	}

	// Armature/damping are driver tuning knobs, not authored rig data — see
	// ApplyPassiveJointDefaults. Must run BEFORE Batch.Init, which copies the
	// topology by value.
	ApplyPassiveJointDefaults(Topo);

	Batch.Init(Topo, NumEnvs);
	ContactPoints = CreatureGroundContact::BuildMutoContactPoints(Topo, MassAsset, BodyDebugNames, bAllBodiesCollideWithGround, StructuralContactRadius);
	if (ContactPoints.Num() == 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("AMutoRLTrainingDriver::StartTraining: no ground-contact points derived — check MassProfile_Muto's CanTouchGround flags."));
	}
	LimbCollisionPairs = CreatureGroundContact::BuildMutoLimbCollisionPairs(Topo);
	// Pre-size (all-untouching) so the very first GatherAgentObservation/
	// GatherAgentReward — called by RunTraining()'s initial RunInference(),
	// BEFORE Tick() ever reaches its own ApplyGroundContactForces call below
	// — doesn't index into a still-empty array. Real values get written in
	// from the second tick onward.
	ContactStates.SetNumZeroed(ContactPoints.Num() * NumEnvs);

	// NOT FQuat::Identity: Pelvis's own bind-pose rotation relative to world/
	// component space isn't necessarily identity (the skeleton's root bone
	// above it may itself be rotated) — see MutoTopology.h's comment on
	// TorsoRestInComponentSpace. Computed before TargetTorsoHeight below,
	// since that computation needs it too.
	StandingTorsoRot = Topo.BodyRestRotInParent[0];

	Config.GroundZ = 0.0f;
	Config.TargetTorsoHeight = TargetTorsoHeightOverride > 0.0f ? TargetTorsoHeightOverride : ComputeDefaultStandingHeight(Topo, ContactPoints, StandingTorsoRot);
	// The torso-local axis that rotates TO world-up at the (known-correct)
	// standing pose — NOT necessarily local +Z; confirmed by direct
	// measurement that Muto's Pelvis bone convention does not have local +Z
	// pointing up (dot(TorsoLocalZ, WorldUp) == -0.675 at the correct
	// standing pose), so defaulting to FVector::UpVector here would make
	// "upright" permanently unsatisfiable regardless of the policy.
	Config.LocalUpAxis = StandingTorsoRot.UnrotateVector(FVector::UpVector);
	Config.MaxTorquePerDOF = MaxTorquePerDOF;
	Config.MinUprightDot = MinUprightDot;
	Config.MinHeightFraction = MinHeightFraction;
	Config.AliveBonus = AliveBonus;
	Config.UprightWeight = UprightWeight;
	Config.BalanceWeight = BalanceWeight;
	Config.TorquePenaltyWeight = TorquePenaltyWeight;
	Config.RewardHeightTarget = RewardHeightTarget;
	Config.RewardHeightMultiplier = RewardHeightMultiplier;
	Config.RewardEnergyConsumptionMultiplier = RewardEnergyConsumptionMultiplier;
	Config.RewardMusclesUseMultiplier = RewardMusclesUseMultiplier;

	StandingTorsoPos = FVector(0.0f, 0.0f, Config.TargetTorsoHeight);
	ResetStream = FRandomStream(ResetRandomSeed);

	UE_LOG(LogTemp, Log, TEXT("AMutoRLTrainingDriver: %d bodies, %d DOF, %d contact points, %d envs, TargetTorsoHeight=%.1f"),
		Topo.NumBodies, Topo.NumDOF, ContactPoints.Num(), NumEnvs, Config.TargetTorsoHeight);

	// ---- Wire up Learning Agents (see class comment: one headless driver, no per-env actors) ----
	Manager = NewObject<ULearningAgentsManager>(this, TEXT("LAManager"));
	// ORDER IS LOAD-BEARING — RegisterComponent() BEFORE SetMaxAgentNum().
	//
	// Root cause of the long-running "Tried to add experience from episode
	// that is still running..." crash (LearningExperience.cpp:401), found
	// 2026-08-12 from a log showing "LAManager: Adding Agents [0 1 1 2 2 3 3
	// 4 ... 128 127 127 126 126 125 125 124]" — duplicate AgentIds handed out
	// by the very first AddAgents call.
	//
	// Both ULearningAgentsManager::OnRegister() (which RegisterComponent()
	// triggers) and SetMaxAgentNum() seed the manager's VacantAgentIds/Agents
	// pools, and SetMaxAgentNum's loop is deliberately INCREMENTAL:
	//     for (AgentId = MaxAgentNum - 1; AgentId >= PreviousMaxAgentNum; ...)
	// i.e. it only adds the NEWLY-widened range, because it assumes
	// OnRegister() already seeded [0, PreviousMaxAgentNum). Calling it first
	// (MaxAgentNum still at its default of 1) meant it seeded 255..1, and
	// then OnRegister() seeded a SECOND, overlapping 255..0 on top — so the
	// vacant-id pool contained duplicates and AddAgents could hand the same
	// AgentId to two different agents.
	//
	// Every symptom followed from that one duplicate pool: our own
	// GatherAgentCompletion logging each terminating agent twice, the
	// engine's own "Resetting Agents [47 47 111 111]", and finally
	// AddEpisodes seeing the same instance twice — the second visit finds the
	// episode already consumed/reset and asserts. It was NOT concurrency (a
	// re-entrancy detector never fired) and NOT a count mismatch (256 agents
	// really were added — just not with 256 DISTINCT ids, which is exactly
	// why an earlier Num()-only guard never caught it).
	//
	// Registering first lets OnRegister() seed from the default MaxAgentNum=1
	// ([0]), after which SetMaxAgentNum(NumEnvs) appends 255..1 — together
	// exactly one entry per id, 0..NumEnvs-1, no overlap.
	Manager->RegisterComponent();
	Manager->SetMaxAgentNum(NumEnvs);
	ULearningAgentsManager* ManagerRaw = Manager;

	ULearningAgentsInteractor* InteractorRaw = ULearningAgentsInteractor::MakeInteractor(ManagerRaw, UMutoRLInteractor::StaticClass());
	Interactor = InteractorRaw;

	ULearningAgentsPolicy* PolicyRaw = ULearningAgentsPolicy::MakePolicy(
		ManagerRaw, InteractorRaw, ULearningAgentsPolicy::StaticClass(), TEXT("Policy"),
		nullptr, nullptr, nullptr, true, true, true, PolicySettings);
	Policy = PolicyRaw;

	// See SaveTrainedNetworks's comment: MakePolicy above always starts from
	// fresh (nullptr-asset, reinitialized) networks — this is the only way
	// to carry a previous run's weights forward instead of always starting
	// from random init.
	if (bLoadSnapshotOnStart)
	{
		LoadTrainedNetworks();
	}

	ULearningAgentsCritic* CriticRaw = ULearningAgentsCritic::MakeCritic(
		ManagerRaw, InteractorRaw, PolicyRaw, ULearningAgentsCritic::StaticClass(), TEXT("Critic"),
		nullptr, true, CriticSettings);
	Critic = CriticRaw;

	ULearningAgentsTrainingEnvironment* TrainingEnvRaw = ULearningAgentsTrainingEnvironment::MakeTrainingEnvironment(ManagerRaw, UMutoRLTrainingEnvironment::StaticClass());
	TrainingEnvironment = TrainingEnvRaw;

	const FLearningAgentsCommunicator Communicator = bUseSocketCommunicator
		? ULearningAgentsCommunicatorLibrary::MakeSocketTrainingProcess(TrainerProcessSettings, SocketCommunicatorSettings)
		: ULearningAgentsCommunicatorLibrary::MakeSharedMemoryTrainingProcess(TrainerProcessSettings, SharedMemoryCommunicatorSettings);
	if (!Communicator.Trainer.IsValid())
	{
		UE_LOG(LogTemp, Error, TEXT("AMutoRLTrainingDriver::StartTraining: failed to launch/connect to the Python training process (%s communicator) — is the engine's Python environment set up (PythonFoundationPackages)?"),
			bUseSocketCommunicator ? TEXT("socket") : TEXT("shared memory"));
		// Deliberately NOT resetting bStartTrainingCalled here: Manager/
		// Interactor/Policy/Critic/TrainingEnvironment already exist at this
		// point. Allowing a retry would build a second full set of those
		// against the same actor and (if the first attempt's connection
		// eventually completed asynchronously after this point returned)
		// risks exactly the duplicate-agent-registration race this flag
		// exists to prevent. A failed connection here means this actor
		// instance needs a fresh PIE session / new actor, not a re-click.
		return;
	}

	ULearningAgentsPPOTrainer* TrainerRaw = ULearningAgentsPPOTrainer::MakePPOTrainer(
		ManagerRaw, InteractorRaw, TrainingEnvRaw, PolicyRaw, CriticRaw, Communicator,
		ULearningAgentsPPOTrainer::StaticClass(), TEXT("PPOTrainer"), TrainerSettings);
	Trainer = TrainerRaw;

	// ---- Agents: one plain UObject per env, AgentId <-> EnvIndex via AgentIdToEnvIndex ----
	TArray<UObject*> AgentPtrs;
	AgentPtrs.Reserve(NumEnvs);
	AgentObjects.Reserve(NumEnvs);
	for (int32 Env = 0; Env < NumEnvs; ++Env)
	{
		UObject* AgentObject = NewObject<UObject>(this);
		AgentObjects.Add(AgentObject);
		AgentPtrs.Add(AgentObject);
	}
	TArray<int32> NewAgentIds;
	Manager->AddAgents(NewAgentIds, AgentPtrs);

	AgentIdToEnvIndex.Init(INDEX_NONE, Manager->GetMaxAgentNum());
	for (int32 Env = 0; Env < NewAgentIds.Num(); ++Env)
	{
		AgentIdToEnvIndex[NewAgentIds[Env]] = Env;
	}

	UE_LOG(LogTemp, Log, TEXT("AMutoRLTrainingDriver: training started (%d agents)."), NewAgentIds.Num());

	// See AutoSaveIntervalSeconds — starts the clock now so the first
	// periodic autosave fires one full interval from here, not immediately.
	LastAutoSaveTime = FPlatformTime::Seconds();

	// The very first RunOneTrainingStep() call has to happen synchronously,
	// here, on the game thread: RunTraining()'s internal BeginTraining()
	// (triggered because !Trainer->IsTraining() yet) touches global CVars
	// (VSync/MaxFPS/fixed-timestep, via ApplyGameSettings), which are not
	// safe to set from a background thread. Every call after this one has
	// IsTraining()==true, so RunTraining() never takes that CVar-setting
	// path again, and the loop is safe to hand off from here on.
	if (!RunOneTrainingStep())
	{
		return;
	}

	bStopTrainingThreadRequested = false;
	TrainingThreadFuture = Async(EAsyncExecution::Thread, [this]() { RunTrainingThreadLoop(); });
#else
	UE_LOG(LogTemp, Error, TEXT("AMutoRLTrainingDriver::StartTraining: editor-only (needs MutoTopology.h/MassMuscleProfile) — not available in this build."));
#endif // WITH_EDITOR
}

bool AMutoRLTrainingDriver::RunOneTrainingStep()
{
	// See TrainingStepLock/ConcurrentStepDepth's comments. The depth counter
	// is deliberately bumped BEFORE the lock is taken — taking the lock first
	// would serialize the threads and hide the very thing being measured.
	// Records the FIRST thread's id so a collision can name both sides.
	static std::atomic<uint32> FirstStepThreadId{0};
	const uint32 ThisThreadId = FPlatformTLS::GetCurrentThreadId();
	uint32 ExpectedNoOwner = 0;
	FirstStepThreadId.compare_exchange_strong(ExpectedNoOwner, ThisThreadId);

	const int32 Depth = ConcurrentStepDepth.fetch_add(1, std::memory_order_acq_rel) + 1;
	if (Depth > 1)
	{
		UE_LOG(LogTemp, Error, TEXT("AMutoRLTrainingDriver: CONCURRENT RunOneTrainingStep detected (depth=%d). This thread=%u, first-seen training thread=%u, game thread=%s. This is the cause of the duplicated agent callbacks and the AddEpisodes assert — see TrainingStepLock's comment."),
			Depth, ThisThreadId, FirstStepThreadId.load(), IsInGameThread() ? TEXT("yes") : TEXT("no"));
	}
	ON_SCOPE_EXIT{ ConcurrentStepDepth.fetch_sub(1, std::memory_order_acq_rel); };

	// Serializes the whole step so that even if a second thread does get in
	// here, it waits rather than interleaving inside Learning Agents' shared
	// scratch arrays (OnEventAgentIds/CompletionBuffer/ResetInstances) and
	// corrupting the completion state AddEpisodes asserts on.
	FScopeLock StepLock(&TrainingStepLock);

	if (Trainer->HasTrainingFailed())
	{
		UE_LOG(LogTemp, Error, TEXT("AMutoRLTrainingDriver: training process failed — stopping. Check the training subprocess log."));
		return false;
	}

	// Diagnostic for the "Tried to add experience from episode that is still
	// running..." crash (LearningExperience.cpp:401-402). Traced it to
	// Manager's own internal agent registry (OccupiedAgentIds, read via
	// GetAllAgentIds()/GetAllAgentSet()) containing a duplicate AgentId — the
	// crash log showed every terminating agent logged twice with identical
	// AgentId+EnvIndex, and "LAManager: Resetting Agents [117 117]" is direct
	// engine-side proof of the same duplicate. A prior fix (bStartTrainingCalled,
	// closing a StartTraining() re-entrancy window) did NOT stop this from
	// recurring, so the duplicate is being introduced some other way, still
	// unknown. Since Manager->AddAgent(s) are the only two engine functions
	// that ever grow OccupiedAgentIds, and we call AddAgents exactly once,
	// this checks for the corruption BEFORE every RunTraining() call (not
	// after — the crash logs show the duplicate already present across
	// several preceding cycles before the one that actually asserts, so
	// catching it here has a real chance of stopping training cleanly
	// instead of running until the engine's own hard, unrecoverable assert
	// eventually fires) and pinpoints exactly which step it first appears on.
	// NOTE (2026-08-12): the ARRAY check below has never once fired across
	// several crashes, which is what disproved the duplicate-registration
	// theory. But the engine does NOT iterate that array in the hot path —
	// ProcessExperience/GatherCompletions iterate Manager->GetAllAgentSet(),
	// which returns an FIndexSet: either a slice (start+count) or a
	// NON-OWNING TArrayView into OccupiedAgentIds' buffer (LearningArray.h).
	// The two are separate objects that only agree if UpdateAgentSets() ran
	// after the last mutation, so a stale/mismatched view is invisible to an
	// array-only check while still doubling everything downstream. Checking
	// BOTH, and checking them against each other, is what actually covers the
	// observed symptom (every AgentId appearing exactly twice).
	// Checks UNIQUENESS, not just count. The original version of this guard
	// only compared Num() against NumEnvs and never fired even while the bug
	// was actively corrupting every run — because 256 agents really were
	// registered, just sharing ~129 distinct ids between them. Counting alone
	// is blind to exactly the failure that actually happened (see the
	// RegisterComponent/SetMaxAgentNum ordering comment in StartTraining).
	const TArray<int32>& AgentIdArray = Manager->GetAllAgentIds();
	TMap<int32, int32> IdCounts;
	IdCounts.Reserve(AgentIdArray.Num());
	for (const int32 Id : AgentIdArray)
	{
		IdCounts.FindOrAdd(Id)++;
	}
	if (AgentIdArray.Num() != NumEnvs || IdCounts.Num() != NumEnvs)
	{
		FString DupList;
		for (const TPair<int32, int32>& Pair : IdCounts)
		{
			if (Pair.Value > 1)
			{
				DupList += FString::Printf(TEXT("%d(x%d) "), Pair.Key, Pair.Value);
			}
		}
		UE_LOG(LogTemp, Error, TEXT("AMutoRLTrainingDriver: Manager agent registry corrupted — %d entries but only %d DISTINCT AgentIds, expected %d of each. Duplicates: %s. Stopping training and saving a snapshot instead of letting this reach the engine's unrecoverable AddEpisodes assert."),
			AgentIdArray.Num(), IdCounts.Num(), NumEnvs, *DupList);
		SaveTrainedNetworks();
		return false;
	}

	// Gathers reward/completion for the PREVIOUS step's actions, resets any
	// completed agents' episodes (-> UMutoRLTrainingEnvironment::ResetAgentEpisode),
	// processes experience, and runs inference on the CURRENT (post-previous-step)
	// physics state to produce this step's actions (-> UMutoRLInteractor::PerformAgentAction,
	// writes Batch.JointTorque). Locked because this both reads AND
	// (via PPO's periodic weight updates) writes the same live network
	// objects AMutoRLVisualizerActor::Tick() reads on the game thread.
	const double RunTrainingStartTime = FPlatformTime::Seconds();
	{
		FScopeLock Lock(&NetworkAccessLock);
		Trainer->RunTraining(TrainingSettings, GameSettings);
	}
	const double RunTrainingSeconds = FPlatformTime::Seconds() - RunTrainingStartTime;

	// RunTraining() calls BeginTraining() whenever !IsTraining() — including
	// once training has legitimately finished (TrainingSettings.NumberOfIterations
	// reached), which would otherwise silently restart it from scratch on the
	// very next call. Stop here instead.
	if (!Trainer->IsTraining())
	{
		UE_LOG(LogTemp, Log, TEXT("AMutoRLTrainingDriver: training completed (%d iterations) — stopping."), TrainingSettings.NumberOfIterations);
		return false;
	}

	// Advance the physics using the actions just written, internally
	// substepped for contact stability (see PhysicsSubstepDt's comment).
	// Batch/Solver are mutated here and nowhere else in this driver's own
	// game/training-thread logic (the visualizer owns its own separate
	// Batch/Solver) -- but the control panel's viewport now reads Batch from
	// the game thread for live display, under the SAME TrainingStepLock this
	// whole function already holds (see StepLock above and GetTrainingStepLock's
	// comment), which is what actually makes that safe.
	const double StepPhysicsStartTime = FPlatformTime::Seconds();
	StepPhysicsSubstepped(FixedDt);
	const double StepPhysicsSeconds = FPlatformTime::Seconds() - StepPhysicsStartTime;

	// Agent+physics-tick heartbeat -- shares the "[AS-TRACE]" prefix with the
	// embedded viewport's own mesh-show heartbeat and the Ragdoll/Visualizer
	// physics-tick heartbeats, so all three sources' traces read the same
	// way. Logged every step (not throttled to once/second like the others)
	// because this runs on the BACKGROUND training thread, where a step is
	// typically the slow, infrequent one (seconds, not milliseconds) rather
	// than a 30-60fps tick -- the whole point here is the timing breakdown:
	// RunTrainingSeconds is Learning Agents' own inference+PPO-sync cost
	// (dominated by the Python subprocess round-trip when the replay buffer
	// syncs), StepPhysicsSeconds is this project's own physics solver cost --
	// separates "training is slow because of Learning Agents/Python" from
	// "training is slow because of our own physics step" instead of leaving
	// both bundled into one unexplained per-step wall-clock number.
	UE_LOG(LogTemp, Log, TEXT("[AS-TRACE] AMutoRLTrainingDriver: agent+physics-tick heartbeat -- step=%d runTrainingMs=%.1f stepPhysicsMs=%.1f torsoZ(env0)=%.2f."),
		TrainingStepCount.load(std::memory_order_relaxed) + 1, RunTrainingSeconds * 1000.0, StepPhysicsSeconds * 1000.0, (float)Batch.GetBodyPos(0, 0).Z);

	// See AutoSaveIntervalSeconds's comment: this is the only autosave that
	// can protect an unattended run against a hard engine crash (a NaN
	// assertion terminates the process before EndPlay ever runs).
	if (AutoSaveIntervalSeconds > 0.0f)
	{
		const double Now = FPlatformTime::Seconds();
		if (Now - LastAutoSaveTime >= AutoSaveIntervalSeconds)
		{
			LastAutoSaveTime = Now;
			SaveTrainedNetworks();
		}
	}

	TrainingStepCount.fetch_add(1, std::memory_order_relaxed);
	return true;
}

void AMutoRLTrainingDriver::ApplyPassiveJointDefaults(FCreatureTopology& Topo) const
{
	for (int32 DOF = 0; DOF < Topo.NumDOF; ++DOF)
	{
		Topo.DOFArmatureRatio[DOF] = FMath::Max(0.0f, JointArmatureRatio);
		Topo.DOFDampingTimeConstant[DOF] = FMath::Max(0.0f, JointDampingTimeConstant);
	}
}

void AMutoRLTrainingDriver::BuildContactParams(
	CreatureGroundContact::FImpulseContactParams& OutContact,
	CreatureGroundContact::FJointLimitParams& OutLimits,
	CreatureGroundContact::FLimbCollisionParams& OutLimbCollision) const
{
	OutContact.GroundZ = Config.GroundZ;
	OutContact.ContactHertz = ContactHertz;
	OutContact.DampingRatio = ContactDampingRatio;
	OutContact.Slop = ContactSlop;
	OutContact.FrictionCoefficient = ContactFrictionCoefficient;
	OutContact.Iterations = ContactIterations;
	OutContact.RelaxIterations = ContactRelaxIterations;
	// Previously left at their struct defaults with no way to reach them from the
	// editor -- Cfm and Relaxation in particular had no caller at all, so the two
	// mechanisms written specifically for the coupled ground-row-vs-joint-limit-row
	// instability had never once run.
	OutContact.Cfm = ContactCfm;
	OutContact.Relaxation = ContactRelaxation;
	OutContact.MaxBiasVelocity = ContactMaxBiasVelocity;
	OutContact.MaxNormalImpulse = ContactMaxNormalImpulse;
	OutContact.bUseGlobalSolve = bUseGlobalConstraintSolve;
	OutContact.GlobalIterations = GlobalSolverIterations;

	OutLimits.bEnabled = bSolveJointLimitsAsConstraints;
	OutLimits.Hertz = JointLimitHertz;
	OutLimits.DampingRatio = JointLimitDampingRatio;
	OutLimits.SlopDeg = JointLimitSlopDeg;
	OutLimits.MarginDeg = JointLimitMarginDeg;
	OutLimits.Cfm = JointLimitCfm;
	OutLimits.Relaxation = JointLimitRelaxation;
	OutLimits.MaxBiasVelocityDeg = JointLimitMaxBiasVelocityDeg;

	OutLimbCollision.bEnabled = bEnableLimbCollision;
	OutLimbCollision.Hertz = LimbCollisionHertz;
	OutLimbCollision.DampingRatio = LimbCollisionDampingRatio;
	OutLimbCollision.Slop = LimbCollisionSlop;
	OutLimbCollision.FrictionCoefficient = LimbCollisionFrictionCoefficient;
	OutLimbCollision.Cfm = LimbCollisionCfm;
	OutLimbCollision.Relaxation = LimbCollisionRelaxation;
	OutLimbCollision.MaxBiasVelocity = LimbCollisionMaxBiasVelocity;
}

void AMutoRLTrainingDriver::StepPhysicsSubstepped(float TotalDt)
{
	// Avoids a substep storm if TotalDt is ever unexpectedly large (e.g. a
	// caller passing real, hitch-prone wall-clock time) — 0.1s is still 24
	// substeps at the default PhysicsSubstepDt, plenty for a single call.
	const float ClampedTotalDt = FMath::Min(TotalDt, 0.1f);
	const float SafeSubstepDt = FMath::Max(PhysicsSubstepDt, KINDA_SMALL_NUMBER);
	const int32 NumSubsteps = FMath::Max(1, FMath::RoundToInt(ClampedTotalDt / SafeSubstepDt));
	const float ActualSubstepDt = ClampedTotalDt / NumSubsteps;

	const int32 CurrentNumEnvs = Batch.GetNumEnvs();

	CreatureGroundContact::FImpulseContactParams ContactParams;
	CreatureGroundContact::FJointLimitParams LimitParams;
	CreatureGroundContact::FLimbCollisionParams LimbCollisionParams;
	BuildContactParams(ContactParams, LimitParams, LimbCollisionParams);

	const bool bWantResidualLog = LogSolverResidualEverySubsteps > 0 && bUseGlobalConstraintSolve;
	const bool bWantWatchLog = WatchContactBody != INDEX_NONE || WatchJointLimitDOF != INDEX_NONE;

	// [AS-TRACE] phase-timing accumulators -- see this function's own
	// heartbeat log below. Isolates which of the 4 per-substep phases
	// (ABA integration, joint damping, saturated-joint-lock rebuild, ground
	// contact + joint limit + limb collision resolution) actually dominates
	// the ~500-700ms/step cost the control panel's own [AS-TRACE] heartbeat
	// (AMutoRLTrainingDriver: agent+physics-tick heartbeat) already showed
	// was the real training bottleneck (Learning Agents' own side was only
	// ~3-5ms/step) -- summed across every substep in this one call, not
	// reset between calls, so this function's log entry reports one full
	// RunOneTrainingStep()'s worth of physics cost, matching the granularity
	// that heartbeat already logs at.
	double StepSeconds = 0.0;
	double DampingSeconds = 0.0;
	double WeldLockSeconds = 0.0;
	double ContactSeconds = 0.0;

	for (int32 Substep = 0; Substep < NumSubsteps; ++Substep)
	{
		// Contact is resolved AFTER integration: an impulse corrects the
		// velocities Step() just produced. (The removed penalty model had to run
		// BEFORE, since it staged a force for the bias pass to pick up — the two
		// sat on opposite sides of Step(), which was a standing source of
		// confusion and one reason for dropping it.)
		//
		// Resolving inside this loop means contact is solved once per SUBSTEP.
		{
			const double T0 = FPlatformTime::Seconds();
			Solver.Step(Batch, ActualSubstepDt, Gravity);
			StepSeconds += FPlatformTime::Seconds() - T0;
		}

		// Passive joint damping, BEFORE contact so contact solves against the
		// damped velocities rather than fighting them. It is a velocity-level
		// impulse (see FCreatureABASolver::ApplyJointDamping — the torque form was
		// tried and measurably reverses joints), so it belongs on this side of
		// Step() alongside contact, not inside the force pass. Returns immediately
		// when no joint has a damping time constant.
		{
			const double T0 = FPlatformTime::Seconds();
			Solver.ApplyJointDamping(Batch, ActualSubstepDt);
			DampingSeconds += FPlatformTime::Seconds() - T0;
		}

		// Weld set for the factorization, rebuilt every substep because which
		// joints are saturated is per-substep, per-env state. Built BEFORE
		// ResolveGroundContactImpulses because that is what calls
		// ComputeArticulatedInertias.
		const uint8* Locks = nullptr;
		int32 LockStride = 0;
		if (bWeldSaturatedJoints)
		{
			const double T0 = FPlatformTime::Seconds();
			Solver.BuildSaturatedJointLocks(Batch, WeldSaturationMarginDeg, SaturatedJointLocks);
			WeldLockSeconds += FPlatformTime::Seconds() - T0;
			Locks = SaturatedJointLocks.GetData();
			LockStride = CurrentNumEnvs;
		}

		const bool bLogThisSubstep = bWantResidualLog && (Substep % LogSolverResidualEverySubsteps == 0);
		CreatureGroundContact::FIterationDebugLog DebugLog;
		DebugLog.WatchBody = WatchContactBody;
		DebugLog.WatchDOF = WatchJointLimitDOF;

		{
			const double T0 = FPlatformTime::Seconds();
			CreatureGroundContact::ResolveGroundContactImpulses(
				Batch, Batch.GetTopology(), ContactPoints, ContactParams,
				Solver, ActualSubstepDt, ContactImpulseCache, &ContactStates, &LimitParams,
				LimbCollisionPairs, &LimbCollisionParams,
				(bLogThisSubstep || bWantWatchLog) ? &DebugLog : nullptr,
				Locks, LockStride);
			ContactSeconds += FPlatformTime::Seconds() - T0;
		}

		if (bLogThisSubstep && DebugLog.GlobalResidualPerIteration.Num() > 0)
		{
			const TArray<float>& R = DebugLog.GlobalResidualPerIteration;
			UE_LOG(LogTemp, Log, TEXT("MutoRL solver: rows=%d  residual first=%.4g  mid=%.4g  last=%.4g  (%d sweeps)"),
				DebugLog.GlobalNumRows, R[0], R[R.Num() / 2], R.Last(), R.Num());
		}
	}

	// bUseGlobalSolve added here specifically to settle whether contactMs is
	// actually going through the parallelized SolveConstraintsGlobalForEnv
	// path (ResolveGroundContactImpulses's `if (Params.bUseGlobalSolve)`) or
	// the OTHER, still-sequential per-row path -- bUseGlobalConstraintSolve
	// defaults to true on the C++ class, but a per-instance level override
	// (e.g. from earlier contact-tuning experiments) would silently make the
	// 2026-08-25 parallelization dead code for this specific placed actor.
	UE_LOG(LogTemp, Log, TEXT("[AS-TRACE] AMutoRLTrainingDriver::StepPhysicsSubstepped: %d substeps -- stepMs=%.1f dampingMs=%.1f weldLockMs=%.1f contactMs=%.1f bUseGlobalSolve=%d (contact = ground+jointLimit+limbCollision combined)."),
		NumSubsteps, StepSeconds * 1000.0, DampingSeconds * 1000.0, WeldLockSeconds * 1000.0, ContactSeconds * 1000.0, ContactParams.bUseGlobalSolve ? 1 : 0);
}

// KNOWN STRUCTURAL HAZARD (2026-08-12) — read before touching this loop.
//
// ULearningAgentsPPOTrainer has THREE internal paths that end training, and
// every one of them calls UE::Learning::Agents::RevertGameSettings()
// (LearningAgentsTrainer.cpp:126), which unconditionally calls
// UGameUserSettings::ApplySettings(), which unconditionally constructs an
// FGlobalComponentRecreateRenderStateContext (GameUserSettings.cpp:526) ->
// FlushRenderingCommands() -> check(IsInGameThread()). Running any of those
// from this background thread is therefore a guaranteed editor crash:
//
//   1. ReceiveNetworks timeout   -> bHasTrainingFailed = true; EndTraining()
//   2. SendReplayBuffer failure  -> bHasTrainingFailed = true; EndTraining()
//   3. Trainer reports Completed -> DoneTraining()
//
// This is not fixable from our side: RevertGameSettings is reached from
// inside Trainer->RunTraining(), so there is no interception point, and no
// combination of FLearningAgentsTrainingGameSettings flags can disable it
// (the apply side is flag-gated, the revert side is not). Confirmed by
// reading the engine source, after a real run crashed via path 1.
//
// What keeps this from firing in practice:
//   - Path 1 is the one that actually bit us; the communicator Timeout is now
//     900s (see SharedMemoryCommunicatorSettings) so a slow-but-healthy PPO
//     iteration no longer trips it.
//   - Path 3 needs TrainingSettings.NumberOfIterations (default 1,000,000)
//     to be exhausted, which no realistic run reaches. Do NOT lower it to a
//     small value expecting a clean finish — that converts normal completion
//     into a crash. Stop training via EndPlay instead, which calls
//     EndTraining() from the game thread, where it is safe (and which is
//     already the case).
//   - Path 2 remains an unmitigated risk, but only fires on a hard
//     shared-memory/socket write failure.
//
// The only complete fix is to call Trainer->RunTraining() from the game
// thread (the plugin's intended usage — its own samples drive it from Tick).
// That was rejected here because a PPO iteration blocks for minutes and would
// freeze the whole editor for that long, which is why this loop exists at all.
void AMutoRLTrainingDriver::RunTrainingThreadLoop()
{
	while (!bStopTrainingThreadRequested.load(std::memory_order_relaxed))
	{
		if (!RunOneTrainingStep())
		{
			break;
		}
	}
}

void AMutoRLTrainingDriver::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	StopTrainingInternal();
	Super::EndPlay(EndPlayReason);
}

void AMutoRLTrainingDriver::StopTraining()
{
	StopTrainingInternal();
}

void AMutoRLTrainingDriver::StopTrainingInternal()
{
	bStopTrainingThreadRequested = true;
	if (TrainingThreadFuture.IsValid())
	{
		TrainingThreadFuture.Wait();
	}
	// After the wait above, the training thread is guaranteed stopped — safe
	// to read the networks with no risk of racing a concurrent PPO update.
	if (bAutoSaveOnEndPlay && Policy)
	{
		SaveTrainedNetworks();
	}
	if (Trainer && Trainer->IsTraining())
	{
		Trainer->EndTraining();
	}

	// Safe here specifically because the thread is already joined (the Wait()
	// above) -- there is no in-flight StartTraining() call this could race
	// with, unlike the re-entrancy window this flag guards against DURING a
	// call. Without this, a manual Stop (the toolbar's Stop button) followed
	// by a manual Start in the SAME PIE session would hit the "already
	// started, ignoring" guard forever -- StartTraining() would never run
	// again for this actor instance until a fresh PIE session.
	bStartTrainingCalled = false;
}

FString AMutoRLTrainingDriver::GetSnapshotDirectory() const
{
	return NetworkSnapshotDirectory.Path.IsEmpty()
		? (FPaths::ProjectSavedDir() / TEXT("MutoRL/Snapshots"))
		: NetworkSnapshotDirectory.Path;
}

void AMutoRLTrainingDriver::SaveTrainedNetworks()
{
	if (!Policy)
	{
		UE_LOG(LogTemp, Error, TEXT("AMutoRLTrainingDriver::SaveTrainedNetworks: no Policy yet — start training first."));
		return;
	}

	const FString Dir = GetSnapshotDirectory();
	IFileManager::Get().MakeDirectory(*Dir, /*Tree=*/true);

	// Same lock the training thread (PPO weight updates) and
	// AMutoRLVisualizerActor's periodic refresh share — avoids reading a
	// network mid-update.
	FScopeLock Lock(&NetworkAccessLock);
	Policy->GetEncoderNetworkAsset()->SaveNetworkToSnapshot(FFilePath{ Dir / TEXT("Encoder.snapshot") });
	Policy->GetPolicyNetworkAsset()->SaveNetworkToSnapshot(FFilePath{ Dir / TEXT("Policy.snapshot") });
	Policy->GetDecoderNetworkAsset()->SaveNetworkToSnapshot(FFilePath{ Dir / TEXT("Decoder.snapshot") });
	UE_LOG(LogTemp, Log, TEXT("AMutoRLTrainingDriver: saved trained networks to %s"), *Dir);
}

void AMutoRLTrainingDriver::LoadTrainedNetworks()
{
	if (!Policy)
	{
		UE_LOG(LogTemp, Error, TEXT("AMutoRLTrainingDriver::LoadTrainedNetworks: no Policy yet — start training first, or use bLoadSnapshotOnStart to load before the first step."));
		return;
	}

	const FString Dir = GetSnapshotDirectory();
	const FString EncoderPath = Dir / TEXT("Encoder.snapshot");
	if (!IFileManager::Get().FileExists(*EncoderPath))
	{
		UE_LOG(LogTemp, Warning, TEXT("AMutoRLTrainingDriver::LoadTrainedNetworks: no snapshot found in %s — leaving current (freshly initialized) weights as-is."), *Dir);
		return;
	}

	FScopeLock Lock(&NetworkAccessLock);
	Policy->GetEncoderNetworkAsset()->LoadNetworkFromSnapshot(FFilePath{ EncoderPath });
	Policy->GetPolicyNetworkAsset()->LoadNetworkFromSnapshot(FFilePath{ Dir / TEXT("Policy.snapshot") });
	Policy->GetDecoderNetworkAsset()->LoadNetworkFromSnapshot(FFilePath{ Dir / TEXT("Decoder.snapshot") });
	UE_LOG(LogTemp, Log, TEXT("AMutoRLTrainingDriver: loaded trained networks from %s"), *Dir);
}

void AMutoRLTrainingDriver::SaveTrainedNetworksToAssets()
{
	if (!Policy)
	{
		UE_LOG(LogTemp, Error, TEXT("AMutoRLTrainingDriver::SaveTrainedNetworksToAssets: no Policy yet — start training first."));
		return;
	}
	if (!SaveEncoderNetworkAsset && !SavePolicyNetworkAsset && !SaveDecoderNetworkAsset)
	{
		UE_LOG(LogTemp, Warning, TEXT("AMutoRLTrainingDriver::SaveTrainedNetworksToAssets: no network assets assigned (SaveEncoderNetworkAsset/SavePolicyNetworkAsset/SaveDecoderNetworkAsset) — nothing to do."));
		return;
	}

	// Same lock the training thread (PPO weight updates) and
	// AMutoRLVisualizerActor's periodic refresh share — avoids reading a
	// network mid-update.
	FScopeLock Lock(&NetworkAccessLock);
	if (SaveEncoderNetworkAsset)
	{
		Policy->GetEncoderNetworkAsset()->SaveNetworkToAsset(SaveEncoderNetworkAsset);
	}
	if (SavePolicyNetworkAsset)
	{
		Policy->GetPolicyNetworkAsset()->SaveNetworkToAsset(SavePolicyNetworkAsset);
	}
	if (SaveDecoderNetworkAsset)
	{
		Policy->GetDecoderNetworkAsset()->SaveNetworkToAsset(SaveDecoderNetworkAsset);
	}
	UE_LOG(LogTemp, Log, TEXT("AMutoRLTrainingDriver: saved trained networks to network assets."));
}

void AMutoRLTrainingDriver::LoadTrainedNetworksFromAssets()
{
	if (!Policy)
	{
		UE_LOG(LogTemp, Error, TEXT("AMutoRLTrainingDriver::LoadTrainedNetworksFromAssets: no Policy yet — start training first."));
		return;
	}
	if (!LoadEncoderNetworkAsset && !LoadPolicyNetworkAsset && !LoadDecoderNetworkAsset)
	{
		UE_LOG(LogTemp, Warning, TEXT("AMutoRLTrainingDriver::LoadTrainedNetworksFromAssets: no network assets assigned (LoadEncoderNetworkAsset/LoadPolicyNetworkAsset/LoadDecoderNetworkAsset) — nothing to do."));
		return;
	}

	FScopeLock Lock(&NetworkAccessLock);
	if (LoadEncoderNetworkAsset)
	{
		Policy->GetEncoderNetworkAsset()->LoadNetworkFromAsset(LoadEncoderNetworkAsset);
	}
	if (LoadPolicyNetworkAsset)
	{
		Policy->GetPolicyNetworkAsset()->LoadNetworkFromAsset(LoadPolicyNetworkAsset);
	}
	if (LoadDecoderNetworkAsset)
	{
		Policy->GetDecoderNetworkAsset()->LoadNetworkFromAsset(LoadDecoderNetworkAsset);
	}
	UE_LOG(LogTemp, Log, TEXT("AMutoRLTrainingDriver: loaded trained networks from network assets."));
}
