#include "AgentSolver/MutoRLVisualizer.h"

#include "LearningAgentsManager.h"
#include "Components/PoseableMeshComponent.h"
#include "Misc/ScopeLock.h"

#if WITH_EDITOR
#include "PhysicsSolver/MutoTopology.h"
#include "AgentSolver/ImitationBake.h"
#include "Animation/AnimSequence.h"
#endif

AMutoRLVisualizerActor::AMutoRLVisualizerActor()
{
	MeshComponent = CreateDefaultSubobject<UPoseableMeshComponent>(TEXT("MutoMesh"));
	RootComponent = MeshComponent;

	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.bStartWithTickEnabled = false; // enabled once StartTraining succeeds
}

void AMutoRLVisualizerActor::StartTraining()
{
#if WITH_EDITOR
	if (Policy)
	{
		UE_LOG(LogTemp, Warning, TEXT("AMutoRLVisualizerActor::StartTraining: already started, ignoring."));
		return;
	}
	if (!SourceTrainingDriver)
	{
		UE_LOG(LogTemp, Error, TEXT("AMutoRLVisualizerActor::StartTraining: SourceTrainingDriver must be assigned."));
		return;
	}
	if (!SkeletalMesh || !MassAsset || !MuscleAsset)
	{
		UE_LOG(LogTemp, Error, TEXT("AMutoRLVisualizerActor::StartTraining: SkeletalMesh/MassAsset/MuscleAsset must all be assigned."));
		return;
	}
	if (!SourceTrainingDriver->GetPolicy())
	{
		UE_LOG(LogTemp, Error, TEXT("AMutoRLVisualizerActor::StartTraining: SourceTrainingDriver has no Policy yet — start its training first."));
		return;
	}

	// ---- Build the topology + a single-env batch, purely for display ----
	FCreatureTopology Topo;
	TArray<FString> Warnings;
	BodyDebugNames.Reset();
	if (!MutoTopology::BuildMutoTopology(SkeletalMesh, MassAsset, MuscleAsset, Topo, Warnings, &BodyDebugNames))
	{
		UE_LOG(LogTemp, Error, TEXT("AMutoRLVisualizerActor::StartTraining: BuildMutoTopology failed."));
		return;
	}
	for (const FString& Warning : Warnings)
	{
		UE_LOG(LogTemp, Warning, TEXT("AMutoRLVisualizerActor: BuildMutoTopology warning: %s"), *Warning);
	}

	// See AMutoRLTrainingDriver::ApplyPassiveJointDefaults — must precede
	// Batch.Init, which copies the topology by value.
	ApplyPassiveJointDefaults(Topo);

	Batch.Init(Topo, 1);
	ContactPoints = CreatureGroundContact::BuildMutoContactPoints(Topo, MassAsset, BodyDebugNames, bAllBodiesCollideWithGround, StructuralContactRadius);
	ContactStates.SetNumZeroed(ContactPoints.Num()); // 1 env — see AMutoRLTrainingDriver's own StartTraining for why this must be pre-sized
	// This override replaces AMutoRLTrainingDriver::StartTraining() entirely
	// rather than calling Super, so LimbCollisionPairs (built there) was never
	// populated here -- bEnableLimbCollision silently did nothing regardless
	// of its value, since StepPhysicsSubstepped's LimbPairs.Num()==0 always
	// short-circuits ResolveGroundContactImpulses' pair gather.
	LimbCollisionPairs = CreatureGroundContact::BuildMutoLimbCollisionPairs(Topo);

	// See AMutoRLTrainingDriver::StartTraining's identical line — Pelvis's
	// own bind-pose rotation isn't necessarily world-identity. Computed
	// before TargetTorsoHeight below, since that computation needs it too.
	StandingTorsoRot = Topo.BodyRestRotInParent[0];

	Config.GroundZ = 0.0f;
	Config.TargetTorsoHeight = TargetTorsoHeightOverride > 0.0f ? TargetTorsoHeightOverride : ComputeDefaultStandingHeight(Topo, ContactPoints, StandingTorsoRot);
	// See AMutoRLTrainingDriver::StartTraining's identical line — local +Z
	// is not this rig's actual "up" direction.
	Config.LocalUpAxis = StandingTorsoRot.UnrotateVector(FVector::UpVector);
	Config.MaxTorquePerDOF = MaxTorquePerDOF;
	Config.MuscleActivationThresholdMultiplier = MuscleActivationThresholdMultiplier;
	Config.MinUprightDot = MinUprightDot;
	Config.MinHeightFraction = MinHeightFraction;
	Config.AliveBonus = AliveBonus;
	Config.UprightWeight = UprightWeight;
	Config.BalanceWeight = BalanceWeight;
	Config.TorquePenaltyWeight = TorquePenaltyWeight;

	StandingTorsoPos = FVector(0.0f, 0.0f, Config.TargetTorsoHeight);
	RestTorsoHeight = Config.TargetTorsoHeight;
	ResetStream = FRandomStream(ResetRandomSeed);

	// ---- Imitation, mirroring AMutoRLTrainingDriver::StartTraining ----
	//
	// This override replaces StartTraining entirely rather than calling Super
	// (see LimbCollisionPairs' comment just above for the last thing that was
	// silently missed by that), so every piece of imitation setup has to be
	// repeated here. It MUST match: the observation size the policy was
	// trained with is baked into the network's input layer, so an inference
	// run that disagrees about the phase component cannot load those weights
	// at all.
	Config.ObjectiveMode = (CreatureRLEnvironment::EObjectiveMode)(uint8)ObjectiveMode;
	ApplyLiveImitationTuning();
	ReferenceMotionBaked = CreatureImitation::FReferenceMotion();
	EnvPhaseOffset.Init(0.0f, 1);
	EnvEpisodeTime.Init(0.0f, 1);

	if (Config.ObjectiveMode == CreatureRLEnvironment::EObjectiveMode::Imitation && ReferenceMotion)
	{
		TArray<int32> EndEffectorBodies;
		TArray<FString> ImitationWarnings;
		ImitationBake::ResolveEndEffectorBodies(Topo, BodyDebugNames, EndEffectorBoneNames, EndEffectorBodies, ImitationWarnings);

		const float ClipLength = ReferenceMotion->GetPlayLength();
		const float BakeStart = bImitateFullClip ? 0.0f : FMath::Clamp(ReferencePoseTime, 0.0f, ClipLength);
		const float BakeEnd = bImitateFullClip ? ClipLength : BakeStart;

		if (!ImitationBake::BakeReferenceMotion(*SkeletalMesh, *ReferenceMotion, Topo, EndEffectorBodies,
			BakeStart, BakeEnd, ReferenceSampleRate, bReferenceMotionLoops, ReferenceMotionBaked, ImitationWarnings))
		{
			ReferenceMotionBaked = CreatureImitation::FReferenceMotion();
		}
		for (const FString& Warning : ImitationWarnings)
		{
			UE_LOG(LogTemp, Warning, TEXT("%s"), *Warning);
		}
	}
	Config.bAppendPhaseObservation = bImitateFullClip && ReferenceMotionBaked.IsValid() && !ReferenceMotionBaked.IsSingleFrame();

	CreatureRLEnvironment::ResetEnv(Batch, 0, StandingTorsoPos, StandingTorsoRot, ResetStream, 0.0f, 0.0f);
	ResetImitationEpisode(0);

	MeshComponent->SetSkinnedAssetAndUpdate(SkeletalMesh); // UPoseableMeshComponent derives from USkinnedMeshComponent, not USkeletalMeshComponent — no SetSkeletalMeshAsset here

	// ---- Minimal Learning Agents wiring: Manager + Interactor + Policy only.
	// No Critic/TrainingEnvironment/Trainer/Communicator — this never
	// trains, it only runs inference. The Policy owns its OWN, freshly
	// created (bReinitialize*=true, nullptr assets -> MakePolicy allocates
	// new ones) network objects — NOT SourceTrainingDriver's live ones, see
	// class comment for why — populated below via RefreshNetworkSnapshot.
	Manager = NewObject<ULearningAgentsManager>(this, TEXT("VizManager"));
	// RegisterComponent() BEFORE SetMaxAgentNum() — see the long comment on
	// the identical pair in AMutoRLTrainingDriver::StartTraining for why this
	// order matters (double-seeded vacant-id pool -> duplicate AgentIds).
	// This actor happened to be immune (MaxAgentNum's default is already 1,
	// so SetMaxAgentNum(1) seeds nothing either way), but leaving the wrong
	// order here would be a trap the moment anyone runs the visualizer with
	// more than one agent.
	Manager->RegisterComponent();
	Manager->SetMaxAgentNum(1);
	ULearningAgentsManager* ManagerRaw = Manager;

	ULearningAgentsInteractor* InteractorRaw = ULearningAgentsInteractor::MakeInteractor(ManagerRaw, UMutoRLInteractor::StaticClass());
	Interactor = InteractorRaw;

	ULearningAgentsPolicy* PolicyRaw = ULearningAgentsPolicy::MakePolicy(
		ManagerRaw, InteractorRaw, ULearningAgentsPolicy::StaticClass(), TEXT("VizPolicy"),
		nullptr, nullptr, nullptr,
		/*bReinitializeEncoderNetwork=*/true, /*bReinitializePolicyNetwork=*/true, /*bReinitializeDecoderNetwork=*/true,
		PolicySettings);
	Policy = PolicyRaw;

	UObject* AgentObject = NewObject<UMutoRLAgentHandle>(this);
	AgentObjects.Add(AgentObject);
	TArray<int32> NewAgentIds;
	Manager->AddAgents(NewAgentIds, { AgentObject });
	AgentIdToEnvIndex.Init(INDEX_NONE, Manager->GetMaxAgentNum());
	AgentIdToEnvIndex[NewAgentIds[0]] = 0;

	// One-time initial sync attempt -- non-blocking (see RefreshNetworkSnapshot's
	// comment for why this used to block and why that was a real editor-freeze
	// bug). If the training thread happens to hold the lock right now, this
	// is a no-op and the networks stay at their freshly-initialized random
	// weights until the very next Tick() below picks up the real ones
	// (NetworkRefreshTimer is set to the full interval, not 0, so that first
	// Tick() retries immediately instead of waiting a whole
	// NetworkRefreshInterval).
	RefreshNetworkSnapshot();
	NetworkRefreshTimer = NetworkRefreshInterval;

	BuildDebugNames();

	UpdateMeshPose();
	SetActorTickEnabled(true);
	UE_LOG(LogTemp, Log, TEXT("AMutoRLVisualizerActor: playback started, sharing weights from %s."), *SourceTrainingDriver->GetName());
#else
	UE_LOG(LogTemp, Error, TEXT("AMutoRLVisualizerActor::StartTraining: editor-only — not available in this build."));
#endif // WITH_EDITOR
}

void AMutoRLVisualizerActor::Tick(float DeltaTime)
{
	// AMutoRLTrainingDriver no longer overrides Tick (its training loop runs
	// on a background thread — see MutoRLTrainingDriver.cpp), so this would
	// resolve to plain AActor::Tick either way; called explicitly for clarity.
	AActor::Tick(DeltaTime);

	if (!Policy)
	{
		// Previously a silent no-op every frame -- if StartTraining() never
		// ran or failed before Policy got created, this actor sits doing
		// NOTHING (not even UpdateMeshPose, so the mesh never gets its FIRST
		// pose) with zero trace of why. Logged once (not every frame) via
		// bLoggedNullPolicyWarning.
		if (!bLoggedNullPolicyWarning)
		{
			bLoggedNullPolicyWarning = true;
			UE_LOG(LogTemp, Warning, TEXT("[AS-TRACE] AMutoRLVisualizerActor: Tick() no-op -- Policy is null (StartTraining likely never ran or failed before reaching MakePolicy)."));
		}
		return;
	}
	if (bLoggedNullPolicyWarning)
	{
		bLoggedNullPolicyWarning = false;
		UE_LOG(LogTemp, Log, TEXT("[AS-TRACE] AMutoRLVisualizerActor: Policy now valid, Tick() ticking normally."));
	}

	// Periodic, non-blocking snapshot refresh (see class comment) — never
	// waits on SourceTrainingDriver's training thread; just skips this
	// interval if it's currently mid-sync.
	NetworkRefreshTimer += DeltaTime;
	if (NetworkRefreshTimer >= NetworkRefreshInterval)
	{
		NetworkRefreshTimer = 0.0f;
		RefreshNetworkSnapshot();
	}

	// Captured BEFORE RunInference() so this is the exact observation the
	// policy is about to see this tick (same Batch/ContactStates snapshot
	// UMutoRLInteractor::GatherAgentObservation_Implementation will compute
	// internally a moment later from the identical inputs) — for the live AI
	// debug window (see UIControls/SAgentSolverAIDebugPanel.h), not used by
	// inference itself.
	CreatureRLEnvironment::ComputeObservations(Batch, 0, Config, ContactPoints, ContactStates, Batch.GetNumEnvs(), LastObservation,
		GetEnvPhase(0));

	// ActionNoiseScale=0: always the policy's mean (deterministic) action,
	// not a noisy exploration sample — "show me what it's learned", not
	// "generate training experience". No lock needed: Policy's networks are
	// this actor's own private copy (see class comment), never written to
	// by anything but RefreshNetworkSnapshot, which only ever runs here on
	// the game thread too.
	Policy->RunInference(0.0f);

	// LastNormalizedActions is now written directly by
	// UMutoRLInteractor::PerformAgentAction_Implementation (called inside
	// RunInference above), captured from the policy's raw output BEFORE
	// CreatureRLEnvironment::ApplyActions clamps/gates it into
	// Batch.JointTorque — see that call site's comment for why reconstructing
	// it from JointTorque (the old approach here) stopped working once
	// ApplyActions started zeroing sub-threshold commands.

	// Real wall-clock DeltaTime overall (normal-speed playback, independent
	// of whatever pacing the training run itself uses), but internally
	// substepped at PhysicsSubstepDt for contact stability — see that
	// property's comment on AMutoRLTrainingDriver. The torque command from
	// RunInference() above holds constant across all substeps.
	StepPhysicsSubstepped(DeltaTime);

	// Here the reference clip advances with WALL-CLOCK DeltaTime, unlike the
	// training driver's fixed simulated step -- this actor deliberately plays
	// back at normal speed (see StepPhysicsSubstepped's comment above), so
	// real time IS its simulated time and the reference must track it or the
	// creature would visibly lag behind the motion it is imitating.
	AdvanceImitationClock(DeltaTime);

	// Falls over -> stand back up, so a live demo doesn't just end up in a
	// heap on the ground; not part of training (this policy is never
	// updated from here), purely a display convenience.
	//
	// Passing ContactStates here is not optional in practice: a non-finite
	// ContactPointState::NormalForce from the LAST substep above is not
	// covered by IsBodyStateValid at all (ContactStates isn't part of
	// Batch), and ResetEnv doesn't touch it either -- without this, exactly
	// that value would survive untouched into the observation
	// RunInference() reads at the START of the NEXT Tick(), crashing on
	// Learning Agents' hard non-finite assert (confirmed in practice,
	// 2026-08-16).
	CreatureRLEnvironment::FImitationTarget ImitationTarget;
	CreatureImitation::FReferenceFrame ReferenceFrame;
	BuildImitationTarget(0, ImitationTarget, ReferenceFrame);

	if (CreatureRLEnvironment::IsTerminated(Batch, 0, Config, &ContactStates, ContactPoints.Num(), Batch.GetNumEnvs(), &ImitationTarget))
	{
		CreatureRLEnvironment::ResetEnv(Batch, 0, StandingTorsoPos, StandingTorsoRot, ResetStream, PosNoiseStdDev, AngleNoiseRad);
		CreatureRLEnvironment::ClearContactStatesForEnv(ContactStates, ContactPoints.Num(), 0, Batch.GetNumEnvs());
		ResetImitationEpisode(0);
	}

	UpdateMeshPose();

	// Agent+physics-tick heartbeat -- shares the "[AS-TRACE]" prefix and
	// roughly-once-a-second cadence with FAgentSolverViewportClient's
	// mesh-show heartbeat and AMutoRagdollVisualizerActor's own physics-tick
	// one, so if the embedded viewport shows a static mesh with this source
	// selected, the log directly answers "is inference+physics even
	// stepping": TorsoZ should be visibly changing between consecutive lines.
	static constexpr int32 TraceHeartbeatInterval = 60;
	if (++TraceHeartbeatCounter >= TraceHeartbeatInterval)
	{
		TraceHeartbeatCounter = 0;
		const float TorsoZ = (float)Batch.GetBodyPos(0, 0).Z;
		UE_LOG(LogTemp, Log, TEXT("[AS-TRACE] AMutoRLVisualizerActor: agent+physics-tick heartbeat -- torsoZ=%.2f."), TorsoZ);
	}
}

void AMutoRLVisualizerActor::RefreshNetworkSnapshot()
{
	ULearningAgentsPolicy* SourcePolicy = SourceTrainingDriver ? SourceTrainingDriver->GetPolicy() : nullptr;
	if (!Policy || !SourcePolicy)
	{
		return;
	}

	FCriticalSection& Lock = SourceTrainingDriver->GetNetworkAccessLock();
	if (!Lock.TryLock())
	{
		// Training thread is mid-sync right now — keep animating the last
		// snapshot rather than stalling the game thread; try again next interval.
		return;
	}

	// Fast in-memory struct copy (ULearningAgentsNeuralNetwork::
	// LoadNetworkFromAsset), not an inference/training call — this is what
	// keeps the refresh itself cheap enough to do periodically.
	Policy->GetEncoderNetworkAsset()->LoadNetworkFromAsset(SourcePolicy->GetEncoderNetworkAsset());
	Policy->GetPolicyNetworkAsset()->LoadNetworkFromAsset(SourcePolicy->GetPolicyNetworkAsset());
	Policy->GetDecoderNetworkAsset()->LoadNetworkFromAsset(SourcePolicy->GetDecoderNetworkAsset());

	Lock.Unlock();
}

void AMutoRLVisualizerActor::UpdateMeshPose()
{
	if (!MeshComponent)
	{
		return;
	}
	const FCreatureTopology& Topo = Batch.GetTopology();
	for (int32 Body = 0; Body < Topo.NumBodies; ++Body)
	{
		if (!BodyDebugNames.IsValidIndex(Body))
		{
			continue;
		}
		// Body 0 is the synthetic fused "Torso" (see BodyDebugNames' comment)
		// — the real bone it corresponds to is Pelvis; every torso-fused
		// bone we never explicitly touch (spine/head/mouth/unarticulated
		// mount bones) just keeps its authored rest-pose LOCAL transform
		// relative to whichever ancestor we DO set, so it follows rigidly —
		// exactly matching this topology's fused-body assumption.
		const FName BoneName = (Body == 0) ? TEXT("Pelvis") : BodyDebugNames[Body];
		const FTransform WorldTransform(Batch.GetBodyRot(Body, 0), Batch.GetBodyPos(Body, 0));
		MeshComponent->SetBoneTransformByName(BoneName, WorldTransform, EBoneSpaces::WorldSpace);
	}
}

namespace
{
	/**
	 * "<BodyName>" for a single-DOF joint, "<BodyName>_X/_Y/_Z" for a 3-DOF
	 * (ball) joint — found by a linear scan of Topo.BodyDOFOffset/BodyDOFCount,
	 * the same layout CreatureRLEnvironment::ComputeObservations/ApplyActions
	 * walk over per-DOF. Cheap and only ever called Topo.NumDOF times, once,
	 * from BuildDebugNames().
	 */
	FName GetDOFDisplayName(const FCreatureTopology& Topo, const TArray<FName>& BodyDebugNames, int32 DOF)
	{
		static const TCHAR* AxisSuffix[3] = { TEXT("X"), TEXT("Y"), TEXT("Z") };
		for (int32 Body = 0; Body < Topo.NumBodies; ++Body)
		{
			const int32 Offset = Topo.BodyDOFOffset[Body];
			const int32 Count = Topo.BodyDOFCount[Body];
			if (DOF >= Offset && DOF < Offset + Count)
			{
				const FName BodyName = BodyDebugNames.IsValidIndex(Body) ? BodyDebugNames[Body] : NAME_None;
				if (Count <= 1)
				{
					return BodyName;
				}
				const int32 AxisIdx = FMath::Clamp(DOF - Offset, 0, 2);
				return FName(*FString::Printf(TEXT("%s_%s"), *BodyName.ToString(), AxisSuffix[AxisIdx]));
			}
		}
		return FName(*FString::Printf(TEXT("DOF%d"), DOF));
	}
}

void AMutoRLVisualizerActor::BuildDebugNames()
{
	const FCreatureTopology& Topo = Batch.GetTopology();

	ObservationNames.Reset(CreatureRLEnvironment::GetObservationSize(Topo, ContactPoints.Num(), Config.bAppendPhaseObservation));
	static const TCHAR* FixedObservationNames[10] = {
		TEXT("TorsoUp.X"), TEXT("TorsoUp.Y"), TEXT("TorsoUp.Z"),
		TEXT("TorsoLinVel.X"), TEXT("TorsoLinVel.Y"), TEXT("TorsoLinVel.Z"),
		TEXT("TorsoAngVel.X"), TEXT("TorsoAngVel.Y"), TEXT("TorsoAngVel.Z"),
		TEXT("TorsoHeightDelta"),
	};
	for (const TCHAR* Name : FixedObservationNames)
	{
		ObservationNames.Add(FName(Name));
	}

	ActionNames.Reset(Topo.NumDOF);
	for (int32 DOF = 0; DOF < Topo.NumDOF; ++DOF)
	{
		const FName DOFName = GetDOFDisplayName(Topo, BodyDebugNames, DOF);
		ObservationNames.Add(FName(*FString::Printf(TEXT("%s.Pos"), *DOFName.ToString())));
		ObservationNames.Add(FName(*FString::Printf(TEXT("%s.Vel"), *DOFName.ToString())));
		ActionNames.Add(DOFName);
	}

	for (int32 PointIdx = 0; PointIdx < ContactPoints.Num(); ++PointIdx)
	{
		const int32 BodyIdx = ContactPoints[PointIdx].BodyIndex;
		const FName BodyName = BodyDebugNames.IsValidIndex(BodyIdx) ? BodyDebugNames[BodyIdx] : NAME_None;
		ObservationNames.Add(FName(*FString::Printf(TEXT("%s.Touch%d"), *BodyName.ToString(), PointIdx)));
		ObservationNames.Add(FName(*FString::Printf(TEXT("%s.Force%d"), *BodyName.ToString(), PointIdx)));
	}

	// Must stay in lockstep with ComputeObservations' own trailing phase pair
	// -- SAgentSolverAIDebugPanel zips these names against LastObservation by
	// index, so a missing entry silently mislabels nothing here but drops the
	// last two gauges.
	if (Config.bAppendPhaseObservation)
	{
		ObservationNames.Add(TEXT("Phase.Sin"));
		ObservationNames.Add(TEXT("Phase.Cos"));
	}
}

void AMutoRLVisualizerActor::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	// No Trainer to end here — deliberately not calling
	// AMutoRLTrainingDriver::EndPlay (which would look for a Trainer this
	// class never creates); just the plain AActor teardown.
	AActor::EndPlay(EndPlayReason);
}
