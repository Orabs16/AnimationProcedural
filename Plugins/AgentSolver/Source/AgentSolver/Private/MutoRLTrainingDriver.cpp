#include "MutoRLTrainingDriver.h"

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
#include "MutoTopology.h"
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
	OutReward = CreatureRLEnvironment::ComputeReward(Driver->Batch, EnvIndex, Driver->Config, Driver->ContactPoints, Driver->ContactStates, Driver->Batch.GetNumEnvs());
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
	OutCompletion = CreatureRLEnvironment::IsTerminated(Driver->Batch, EnvIndex, Driver->Config)
		? ELearningAgentsCompletion::Termination
		: ELearningAgentsCompletion::Running;
}

void UMutoRLTrainingEnvironment::ResetAgentEpisode_Implementation(const int32 AgentId)
{
	AMutoRLTrainingDriver* Driver = GetTypedOuter<AMutoRLTrainingDriver>();
	check(Driver);
	const int32 EnvIndex = Driver->GetEnvIndexForAgent(AgentId);
	CreatureRLEnvironment::ResetEnv(Driver->Batch, EnvIndex, Driver->StandingTorsoPos, Driver->StandingTorsoRot, Driver->ResetStream, Driver->PosNoiseStdDev, Driver->AngleNoiseRad);
}

// ================================ AMutoRLTrainingDriver ================================

AMutoRLTrainingDriver::AMutoRLTrainingDriver()
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.bStartWithTickEnabled = false; // enabled once training actually starts (see StartTraining)

	// Bumped from the engine default (10s) — the first handshake has to wait
	// for the Python subprocess to import torch and friends, which can take
	// a while on a cold start and can otherwise look identical to a hard failure.
	SharedMemoryCommunicatorSettings.Timeout = 60.0f;

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
		const FVector LocalAxis = Point.LocalOffset.GetSafeNormal();
		const FVector LocalStart = Point.LocalOffset - LocalAxis * (Point.CapsuleHalfHeight * 2.0f);

		const float WorldZOffsetAtTip = WorldZOffsetAtBodyOrigin + static_cast<float>(WorldRot.RotateVector(Point.LocalOffset).Z);
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

	if (bAutoStartOnBeginPlay)
	{
		StartTraining();
	}
}

void AMutoRLTrainingDriver::StartTraining()
{
#if WITH_EDITOR
	if (Trainer)
	{
		UE_LOG(LogTemp, Warning, TEXT("AMutoRLTrainingDriver::StartTraining: already started, ignoring."));
		return;
	}

	if (!SkeletalMesh || !MassAsset || !MuscleAsset)
	{
		UE_LOG(LogTemp, Error, TEXT("AMutoRLTrainingDriver::StartTraining: SkeletalMesh/MassAsset/MuscleAsset must all be assigned in the Details panel."));
		return;
	}

	// ---- Build the topology + batch (same path MutoTopologyTest.cpp validates) ----
	FCreatureTopology Topo;
	TArray<FString> Warnings;
	BodyDebugNames.Reset();
	if (!MutoTopology::BuildMutoTopology(SkeletalMesh, MassAsset, MuscleAsset, Topo, Warnings, &BodyDebugNames))
	{
		UE_LOG(LogTemp, Error, TEXT("AMutoRLTrainingDriver::StartTraining: BuildMutoTopology failed."));
		return;
	}
	for (const FString& Warning : Warnings)
	{
		UE_LOG(LogTemp, Warning, TEXT("AMutoRLTrainingDriver: BuildMutoTopology warning: %s"), *Warning);
	}

	Batch.Init(Topo, NumEnvs);
	ContactPoints = CreatureGroundContact::BuildMutoContactPoints(Topo, MassAsset, BodyDebugNames);
	if (ContactPoints.Num() == 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("AMutoRLTrainingDriver::StartTraining: no ground-contact points derived — check MassProfile_Muto's CanTouchGround flags."));
	}
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

	StandingTorsoPos = FVector(0.0f, 0.0f, Config.TargetTorsoHeight);
	ResetStream = FRandomStream(ResetRandomSeed);

	UE_LOG(LogTemp, Log, TEXT("AMutoRLTrainingDriver: %d bodies, %d DOF, %d contact points, %d envs, TargetTorsoHeight=%.1f"),
		Topo.NumBodies, Topo.NumDOF, ContactPoints.Num(), NumEnvs, Config.TargetTorsoHeight);

	// ---- Wire up Learning Agents (see class comment: one headless driver, no per-env actors) ----
	Manager = NewObject<ULearningAgentsManager>(this, TEXT("LAManager"));
	Manager->SetMaxAgentNum(NumEnvs);
	Manager->RegisterComponent();
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
	if (Trainer->HasTrainingFailed())
	{
		UE_LOG(LogTemp, Error, TEXT("AMutoRLTrainingDriver: training process failed — stopping. Check the training subprocess log."));
		return false;
	}

	// Gathers reward/completion for the PREVIOUS step's actions, resets any
	// completed agents' episodes (-> UMutoRLTrainingEnvironment::ResetAgentEpisode),
	// processes experience, and runs inference on the CURRENT (post-previous-step)
	// physics state to produce this step's actions (-> UMutoRLInteractor::PerformAgentAction,
	// writes Batch.JointTorque). Locked because this both reads AND
	// (via PPO's periodic weight updates) writes the same live network
	// objects AMutoRLVisualizerActor::Tick() reads on the game thread.
	{
		FScopeLock Lock(&NetworkAccessLock);
		Trainer->RunTraining(TrainingSettings, GameSettings);
	}

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
	// Batch/Solver are never touched by anything other than this loop (the
	// visualizer owns its own separate Batch/Solver), so no lock needed here.
	StepPhysicsSubstepped(FixedDt);

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

	return true;
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
	CreatureGroundContact::FContactParams ContactParams;
	ContactParams.GroundZ = Config.GroundZ;
	ContactParams.SpringK = ContactSpringK;
	ContactParams.DamperK = ContactDamperK;
	ContactParams.FrictionK = ContactFrictionK;
	ContactParams.FrictionCoefficient = ContactFrictionCoefficient;

	for (int32 Substep = 0; Substep < NumSubsteps; ++Substep)
	{
		for (int32 Env = 0; Env < CurrentNumEnvs; ++Env)
		{
			Batch.ClearExternalForces(Env);
		}
		CreatureGroundContact::ApplyGroundContactForces(Batch, Batch.GetTopology(), ContactPoints, ContactParams, &ContactStates);
		Solver.Step(Batch, ActualSubstepDt, Gravity);
		if (MaxJointSpeedDegPerSec > 0.0f)
		{
			Solver.ClampJointSpeed(Batch, FMath::DegreesToRadians(MaxJointSpeedDegPerSec));
		}
	}
}

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
	Super::EndPlay(EndPlayReason);
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
