#include "MutoRLVisualizer.h"

#include "LearningAgentsManager.h"
#include "Components/PoseableMeshComponent.h"
#include "Misc/ScopeLock.h"

#if WITH_EDITOR
#include "MutoTopology.h"
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

	Batch.Init(Topo, 1);
	ContactPoints = CreatureGroundContact::BuildMutoContactPoints(Topo, MassAsset, BodyDebugNames);
	ContactStates.SetNumZeroed(ContactPoints.Num()); // 1 env — see AMutoRLTrainingDriver's own StartTraining for why this must be pre-sized

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
	Config.MinUprightDot = MinUprightDot;
	Config.MinHeightFraction = MinHeightFraction;
	Config.AliveBonus = AliveBonus;
	Config.UprightWeight = UprightWeight;
	Config.BalanceWeight = BalanceWeight;
	Config.TorquePenaltyWeight = TorquePenaltyWeight;

	StandingTorsoPos = FVector(0.0f, 0.0f, Config.TargetTorsoHeight);
	ResetStream = FRandomStream(ResetRandomSeed);
	CreatureRLEnvironment::ResetEnv(Batch, 0, StandingTorsoPos, StandingTorsoRot, ResetStream, 0.0f, 0.0f);

	MeshComponent->SetSkinnedAssetAndUpdate(SkeletalMesh); // UPoseableMeshComponent derives from USkinnedMeshComponent, not USkeletalMeshComponent — no SetSkeletalMeshAsset here

	// ---- Minimal Learning Agents wiring: Manager + Interactor + Policy only.
	// No Critic/TrainingEnvironment/Trainer/Communicator — this never
	// trains, it only runs inference. The Policy owns its OWN, freshly
	// created (bReinitialize*=true, nullptr assets -> MakePolicy allocates
	// new ones) network objects — NOT SourceTrainingDriver's live ones, see
	// class comment for why — populated below via RefreshNetworkSnapshot.
	Manager = NewObject<ULearningAgentsManager>(this, TEXT("VizManager"));
	Manager->SetMaxAgentNum(1);
	Manager->RegisterComponent();
	ULearningAgentsManager* ManagerRaw = Manager;

	ULearningAgentsInteractor* InteractorRaw = ULearningAgentsInteractor::MakeInteractor(ManagerRaw, UMutoRLInteractor::StaticClass());
	Interactor = InteractorRaw;

	ULearningAgentsPolicy* PolicyRaw = ULearningAgentsPolicy::MakePolicy(
		ManagerRaw, InteractorRaw, ULearningAgentsPolicy::StaticClass(), TEXT("VizPolicy"),
		nullptr, nullptr, nullptr,
		/*bReinitializeEncoderNetwork=*/true, /*bReinitializePolicyNetwork=*/true, /*bReinitializeDecoderNetwork=*/true,
		PolicySettings);
	Policy = PolicyRaw;

	UObject* AgentObject = NewObject<UObject>(this);
	AgentObjects.Add(AgentObject);
	TArray<int32> NewAgentIds;
	Manager->AddAgents(NewAgentIds, { AgentObject });
	AgentIdToEnvIndex.Init(INDEX_NONE, Manager->GetMaxAgentNum());
	AgentIdToEnvIndex[NewAgentIds[0]] = 0;

	// One-time initial sync: fine to block briefly here (this runs once, at
	// startup, not per-tick) so the freshly created networks above start out
	// with real trained weights rather than random init.
	RefreshNetworkSnapshot(/*bBlockUntilAvailable=*/true);
	NetworkRefreshTimer = 0.0f;

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
		return;
	}

	// Periodic, non-blocking snapshot refresh (see class comment) — never
	// waits on SourceTrainingDriver's training thread; just skips this
	// interval if it's currently mid-sync.
	NetworkRefreshTimer += DeltaTime;
	if (NetworkRefreshTimer >= NetworkRefreshInterval)
	{
		NetworkRefreshTimer = 0.0f;
		RefreshNetworkSnapshot(/*bBlockUntilAvailable=*/false);
	}

	// ActionNoiseScale=0: always the policy's mean (deterministic) action,
	// not a noisy exploration sample — "show me what it's learned", not
	// "generate training experience". No lock needed: Policy's networks are
	// this actor's own private copy (see class comment), never written to
	// by anything but RefreshNetworkSnapshot, which only ever runs here on
	// the game thread too.
	Policy->RunInference(0.0f);

	// Real wall-clock DeltaTime overall (normal-speed playback, independent
	// of whatever pacing the training run itself uses), but internally
	// substepped at PhysicsSubstepDt for contact stability — see that
	// property's comment on AMutoRLTrainingDriver. The torque command from
	// RunInference() above holds constant across all substeps.
	StepPhysicsSubstepped(DeltaTime);

	// Falls over -> stand back up, so a live demo doesn't just end up in a
	// heap on the ground; not part of training (this policy is never
	// updated from here), purely a display convenience.
	if (CreatureRLEnvironment::IsTerminated(Batch, 0, Config))
	{
		CreatureRLEnvironment::ResetEnv(Batch, 0, StandingTorsoPos, StandingTorsoRot, ResetStream, PosNoiseStdDev, AngleNoiseRad);
	}

	UpdateMeshPose();
}

void AMutoRLVisualizerActor::RefreshNetworkSnapshot(bool bBlockUntilAvailable)
{
	ULearningAgentsPolicy* SourcePolicy = SourceTrainingDriver ? SourceTrainingDriver->GetPolicy() : nullptr;
	if (!Policy || !SourcePolicy)
	{
		return;
	}

	FCriticalSection& Lock = SourceTrainingDriver->GetNetworkAccessLock();
	if (bBlockUntilAvailable)
	{
		Lock.Lock();
	}
	else if (!Lock.TryLock())
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

void AMutoRLVisualizerActor::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	// No Trainer to end here — deliberately not calling
	// AMutoRLTrainingDriver::EndPlay (which would look for a Trainer this
	// class never creates); just the plain AActor teardown.
	AActor::EndPlay(EndPlayReason);
}
