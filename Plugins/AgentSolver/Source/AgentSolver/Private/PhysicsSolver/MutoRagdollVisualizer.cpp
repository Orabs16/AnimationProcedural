#include "PhysicsSolver/MutoRagdollVisualizer.h"

#include "Components/PoseableMeshComponent.h"
#include "DrawDebugHelpers.h"
#include "Engine/Engine.h"
#include "Engine/World.h"

#if WITH_EDITOR
#include "PhysicsSolver/MutoTopology.h"
#endif

AMutoRagdollVisualizerActor::AMutoRagdollVisualizerActor()
{
	MeshComponent = CreateDefaultSubobject<UPoseableMeshComponent>(TEXT("MutoRagdollMesh"));
	RootComponent = MeshComponent;

	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.bStartWithTickEnabled = false; // enabled once StartTraining succeeds
}

void AMutoRagdollVisualizerActor::StartTraining()
{
#if WITH_EDITOR
	if (Batch.GetTopology().NumBodies > 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("AMutoRagdollVisualizerActor: already started, ignoring."));
		return;
	}
	if (!SkeletalMesh || !MassAsset || !MuscleAsset)
	{
		UE_LOG(LogTemp, Error, TEXT("AMutoRagdollVisualizerActor: SkeletalMesh/MassAsset/MuscleAsset must all be assigned."));
		return;
	}

	FCreatureTopology Topo;
	TArray<FString> Warnings;
	BodyDebugNames.Reset();
	if (!MutoTopology::BuildMutoTopology(SkeletalMesh, MassAsset, MuscleAsset, Topo, Warnings, &BodyDebugNames))
	{
		UE_LOG(LogTemp, Error, TEXT("AMutoRagdollVisualizerActor: BuildMutoTopology failed."));
		return;
	}
	for (const FString& Warning : Warnings)
	{
		UE_LOG(LogTemp, Warning, TEXT("AMutoRagdollVisualizerActor: BuildMutoTopology warning: %s"), *Warning);
	}

	// See AMutoRLTrainingDriver::ApplyPassiveJointDefaults — must precede
	// Batch.Init, which copies the topology by value.
	ApplyPassiveJointDefaults(Topo);

	Batch.Init(Topo, 1);
	ContactPoints = CreatureGroundContact::BuildMutoContactPoints(Topo, MassAsset, BodyDebugNames, bAllBodiesCollideWithGround, StructuralContactRadius);
	ContactStates.SetNumZeroed(ContactPoints.Num());
	// This override replaces AMutoRLTrainingDriver::StartTraining() entirely
	// rather than calling Super, so LimbCollisionPairs (built there) was never
	// populated here -- bEnableLimbCollision silently did nothing regardless
	// of its value. Same gap as AMutoRLVisualizerActor::StartTraining.
	LimbCollisionPairs = CreatureGroundContact::BuildMutoLimbCollisionPairs(Topo);

	// Pelvis's own bind-pose rotation is not world-identity -- see the identical
	// line in AMutoRLTrainingDriver::StartTraining.
	StandingTorsoRot = Topo.BodyRestRotInParent[0];

	Config.GroundZ = 0.0f;
	Config.TargetTorsoHeight = TargetTorsoHeightOverride > 0.0f ? TargetTorsoHeightOverride : ComputeDefaultStandingHeight(Topo, ContactPoints, StandingTorsoRot);
	Config.LocalUpAxis = StandingTorsoRot.UnrotateVector(FVector::UpVector);
	// MaxTorquePerDOF is deliberately NOT copied into Config: nothing here ever
	// writes JointTorque, so there is no actuation for it to limit. Leaving it
	// unset makes that explicit rather than incidental.

	MeshComponent->SetSkinnedAssetAndUpdate(SkeletalMesh);

	ResetRagdoll();
	SetActorTickEnabled(true);

	UE_LOG(LogTemp, Log, TEXT("AMutoRagdollVisualizerActor: passive playback started -- %d bodies, %d DOF, %d contact points, %.1fx speed."),
		Topo.NumBodies, Topo.NumDOF, ContactPoints.Num(), PlaybackSpeed);
#else
	UE_LOG(LogTemp, Error, TEXT("AMutoRagdollVisualizerActor: editor-only -- not available in this build."));
#endif // WITH_EDITOR
}

void AMutoRagdollVisualizerActor::ResetRagdoll()
{
	const FCreatureTopology& Topo = Batch.GetTopology();
	if (Topo.NumBodies == 0)
	{
		return;
	}

	// Re-seeded every reset, so the same ResetSeed always replays the same fall.
	ResetStream = FRandomStream(ResetSeed);

	const float Height = ResetTorsoHeight > 0.0f ? ResetTorsoHeight : Config.TargetTorsoHeight;
	StandingTorsoPos = FVector(0.0f, 0.0f, Height);
	const FQuat TiltedRot = (ResetTorsoTilt.Quaternion() * StandingTorsoRot).GetNormalized();

	CreatureRLEnvironment::ResetEnv(Batch, 0, StandingTorsoPos, TiltedRot, ResetStream, 0.0f, 0.0f);

	if (ResetJointJitterDeg > 0.0f)
	{
		const float JitterRad = FMath::DegreesToRadians(ResetJointJitterDeg);
		for (int32 Body = 1; Body < Topo.NumBodies; ++Body)
		{
			const int32 DOFOffset = Topo.BodyDOFOffset[Body];
			if (Topo.BodyDOFCount[Body] == 1)
			{
				Batch.JointPos[Batch.DOFIndex(DOFOffset, 0)] = ResetStream.FRandRange(-JitterRad, JitterRad);
			}
			else if (Topo.BodyDOFCount[Body] == 3)
			{
				// Ball joints carry a persistent quaternion; JointPos holds its
				// rotation-vector components. Both must be written together, or
				// the next step reads a pose the quaternion disagrees with --
				// same pairing ClampJointLimits maintains.
				const FVector RotVec(
					ResetStream.FRandRange(-JitterRad, JitterRad),
					ResetStream.FRandRange(-JitterRad, JitterRad),
					ResetStream.FRandRange(-JitterRad, JitterRad));
				Batch.SetJointRelRot(Body, 0, FQuat::MakeFromRotationVector(RotVec));
				Batch.JointPos[Batch.DOFIndex(DOFOffset + 0, 0)] = (float)RotVec.X;
				Batch.JointPos[Batch.DOFIndex(DOFOffset + 1, 0)] = (float)RotVec.Y;
				Batch.JointPos[Batch.DOFIndex(DOFOffset + 2, 0)] = (float)RotVec.Z;
			}
		}
	}

	// Bring BodyPos/BodyRot into agreement with the joint state we just wrote,
	// so the very first drawn frame is the pose we asked for rather than the
	// one left over from before the reset.
	Solver.RecomputeKinematics(Batch);

	ContactImpulseCache = CreatureGroundContact::FImpulseContactCache();
	for (CreatureGroundContact::FContactPointState& State : ContactStates)
	{
		State = CreatureGroundContact::FContactPointState();
	}

	bDiverged = false;
	DivergenceReason.Reset();
	DivergedAtSimTime = 0.0f;
	bHasLastGood = false;
	SimTime = 0.0f;
	RestTimer = 0.0f;
	NumTouchingContacts = 0;
	DeepestPenetration = 0.0f;
	FastestBodySpeed = 0.0f;
	FastestBodyIndex = INDEX_NONE;
	FastestJointSpeedDeg = 0.0f;
	FastestJointDOF = INDEX_NONE;

	UpdateMeshPose();
}

void AMutoRagdollVisualizerActor::TogglePause()
{
	bPaused = !bPaused;
}

void AMutoRagdollVisualizerActor::StepOnce()
{
	if (Batch.GetTopology().NumBodies == 0)
	{
		return;
	}
	// Deliberately works while diverged -- stepping past the freeze is
	// occasionally what you want (to confirm it really is unrecoverable), so
	// clear the flag and let it re-trigger rather than refusing.
	bDiverged = false;
	AdvancePhysics(FMath::Max(1, SubstepsPerManualStep));
	UpdateMeshPose();
}

void AMutoRagdollVisualizerActor::Tick(float DeltaTime)
{
	AActor::Tick(DeltaTime);

	const FCreatureTopology& Topo = Batch.GetTopology();
	if (Topo.NumBodies == 0)
	{
		return;
	}

	if (!bPaused && !bDiverged)
	{
		// PlaybackSpeed scales SIMULATED time only. Substep size stays
		// PhysicsSubstepDt, so slow motion is a shutter speed, not a solver
		// setting -- the trajectory is identical at any playback speed.
		const float SafeSubstepDt = FMath::Max(PhysicsSubstepDt, KINDA_SMALL_NUMBER);
		const float SimDt = FMath::Min(DeltaTime, 0.1f) * PlaybackSpeed;
		// Capped so an editor hitch cannot turn into a several-hundred-substep
		// catch-up that skips straight past whatever you were watching for.
		const int32 NumSubsteps = FMath::Clamp(FMath::RoundToInt(SimDt / SafeSubstepDt), 0, 60);
		if (NumSubsteps > 0)
		{
			AdvancePhysics(NumSubsteps);
		}
	}

	UpdateMeshPose();
	DrawDebug();
	if (bShowOnScreenStats)
	{
		DrawStats();
	}

	// Physics-tick heartbeat -- shares the "[AS-TRACE]" prefix and roughly-
	// once-a-second cadence with FAgentSolverViewportClient's mesh-show
	// heartbeat, so if the embedded viewport shows a static mesh with this
	// source selected, the log directly answers "is physics even stepping":
	// bPaused/bDiverged explain a deliberate stop, otherwise TorsoZ should be
	// visibly changing between consecutive heartbeat lines.
	static constexpr int32 TraceHeartbeatInterval = 60;
	if (++TraceHeartbeatCounter >= TraceHeartbeatInterval)
	{
		TraceHeartbeatCounter = 0;
		const float TorsoZ = Topo.NumBodies > 0 ? (float)Batch.GetBodyPos(0, 0).Z : 0.0f;
		//UE_LOG(LogTemp, Log, TEXT("[AS-TRACE] AMutoRagdollVisualizerActor: physics-tick heartbeat -- bPaused=%d bDiverged=%d simTime=%.2f torsoZ=%.2f."),
		//	bPaused, bDiverged, SimTime, TorsoZ);
	}
}

bool AMutoRagdollVisualizerActor::AdvancePhysics(int32 NumSubsteps)
{
	const FCreatureTopology& Topo = Batch.GetTopology();

	for (int32 i = 0; i < NumSubsteps; ++i)
	{
		// Snapshot per SUBSTEP, not per frame: a divergence rolled back to the
		// start of the frame would have already lost the substeps that show it
		// building. At 1 env this copy is a few KB.
		if (bFreezeOnDivergence)
		{
			LastGoodBatch = Batch;
			bHasLastGood = true;
		}

		// The defining property of this actor, restated every substep rather
		// than relied upon: no muscle ever fires here. ResetEnv already zeroes
		// these and nothing else writes them, so this is belt-and-braces -- but
		// if a torque ever DID appear, silently, this is the line that keeps
		// "ragdoll" true.
		for (int32 d = 0; d < Topo.NumDOF; ++d)
		{
			Batch.JointTorque[Batch.DOFIndex(d, 0)] = 0.0f;
		}

		// Exactly one substep: StepPhysicsSubstepped rounds TotalDt/PhysicsSubstepDt,
		// so passing one substep's worth gives one substep.
		StepPhysicsSubstepped(PhysicsSubstepDt);
		SimTime += PhysicsSubstepDt;

		const FString Bad = FindBadState();
		if (!Bad.IsEmpty())
		{
			bDiverged = true;
			DivergenceReason = Bad;
			DivergedAtSimTime = SimTime;

			if (bFreezeOnDivergence && bHasLastGood)
			{
				// Roll back to the last good substep: the frame BEFORE the
				// blow-up is the one carrying the evidence. A NaN pose draws
				// nothing at all, which is the least useful possible outcome.
				Batch = LastGoodBatch;
				SimTime -= PhysicsSubstepDt;
			}

			UE_LOG(LogTemp, Error, TEXT("AMutoRagdollVisualizerActor: DIVERGED at sim t=%.4fs -- %s%s"),
				DivergedAtSimTime, *DivergenceReason,
				(bFreezeOnDivergence && bHasLastGood) ? TEXT(" (rolled back one substep and frozen)") : TEXT(""));
			DumpState();
			return false;
		}
	}

	// ---- Readout statistics, gathered once here so DrawStats stays free ----
	NumTouchingContacts = 0;
	for (const CreatureGroundContact::FContactPointState& State : ContactStates)
	{
		if (State.bTouching)
		{
			++NumTouchingContacts;
		}
	}

	DeepestPenetration = 0.0f;
	for (const CreatureGroundContact::FContactPointDef& Point : ContactPoints)
	{
		const FVector World = Batch.GetBodyPos(Point.BodyIndex, 0) + Batch.GetBodyRot(Point.BodyIndex, 0).RotateVector(Point.LocalOffset);
		const float Penetration = Config.GroundZ - (static_cast<float>(World.Z) - Point.Radius);
		DeepestPenetration = FMath::Max(DeepestPenetration, Penetration);
	}

	FastestBodySpeed = 0.0f;
	FastestBodyIndex = INDEX_NONE;
	for (int32 Body = 0; Body < Topo.NumBodies; ++Body)
	{
		const int32 Idx = Batch.BodyIndex(Body, 0);
		const float Speed = FVector(Batch.LinVelX[Idx], Batch.LinVelY[Idx], Batch.LinVelZ[Idx]).Size();
		if (Speed > FastestBodySpeed)
		{
			FastestBodySpeed = Speed;
			FastestBodyIndex = Body;
		}
	}

	FastestJointSpeedDeg = 0.0f;
	FastestJointDOF = INDEX_NONE;
	for (int32 d = 0; d < Topo.NumDOF; ++d)
	{
		const float SpeedDeg = FMath::RadiansToDegrees(FMath::Abs(Batch.JointVel[Batch.DOFIndex(d, 0)]));
		if (SpeedDeg > FastestJointSpeedDeg)
		{
			FastestJointSpeedDeg = SpeedDeg;
			FastestJointDOF = d;
		}
	}

	// "At rest" = nothing moving anywhere, not merely the torso being still.
	if (AutoResetAfterRestSeconds > 0.0f)
	{
		const bool bAtRest = FastestBodySpeed < 1.0f && FastestJointSpeedDeg < 1.0f;
		RestTimer = bAtRest ? RestTimer + NumSubsteps * PhysicsSubstepDt : 0.0f;
		if (RestTimer >= AutoResetAfterRestSeconds)
		{
			ResetRagdoll();
		}
	}

	return true;
}

FString AMutoRagdollVisualizerActor::FindBadState() const
{
	const FCreatureTopology& Topo = Batch.GetTopology();

	for (int32 Body = 0; Body < Topo.NumBodies; ++Body)
	{
		const int32 Idx = Batch.BodyIndex(Body, 0);
		const FVector Pos(Batch.PosX[Idx], Batch.PosY[Idx], Batch.PosZ[Idx]);
		const FVector LinVel(Batch.LinVelX[Idx], Batch.LinVelY[Idx], Batch.LinVelZ[Idx]);
		const FVector AngVel(Batch.AngVelX[Idx], Batch.AngVelY[Idx], Batch.AngVelZ[Idx]);
		const FName Name = BodyDebugNames.IsValidIndex(Body) ? BodyDebugNames[Body] : FName(*FString::Printf(TEXT("body%d"), Body));

		if (Pos.ContainsNaN() || LinVel.ContainsNaN() || AngVel.ContainsNaN())
		{
			return FString::Printf(TEXT("non-finite state on body %d (%s)"), Body, *Name.ToString());
		}
		if (MaxPlausibleDistance > 0.0f && Pos.Size() > MaxPlausibleDistance)
		{
			return FString::Printf(TEXT("body %d (%s) is %.0f cm from origin (limit %.0f)"), Body, *Name.ToString(), Pos.Size(), MaxPlausibleDistance);
		}
		if (MaxPlausibleSpeed > 0.0f && LinVel.Size() > MaxPlausibleSpeed)
		{
			return FString::Printf(TEXT("body %d (%s) is moving at %.0f cm/s (limit %.0f)"), Body, *Name.ToString(), LinVel.Size(), MaxPlausibleSpeed);
		}
	}

	for (int32 d = 0; d < Topo.NumDOF; ++d)
	{
		const int32 Idx = Batch.DOFIndex(d, 0);
		if (!FMath::IsFinite(Batch.JointPos[Idx]) || !FMath::IsFinite(Batch.JointVel[Idx]))
		{
			return FString::Printf(TEXT("non-finite joint state on DOF %d"), d);
		}
	}

	return FString();
}

void AMutoRagdollVisualizerActor::DrawDebug() const
{
	UWorld* World = GetWorld();
	if (!World)
	{
		return;
	}
	const FCreatureTopology& Topo = Batch.GetTopology();

	if (bDrawGroundGrid)
	{
		constexpr int32 GridLines = 10;
		constexpr float GridSpacing = 100.0f;
		const float Extent = GridLines * GridSpacing;
		const FColor GridColor(70, 70, 80);
		for (int32 i = -GridLines; i <= GridLines; ++i)
		{
			const float Offset = i * GridSpacing;
			DrawDebugLine(World, FVector(-Extent, Offset, Config.GroundZ), FVector(Extent, Offset, Config.GroundZ), GridColor, false, -1.0f, 0, 1.0f);
			DrawDebugLine(World, FVector(Offset, -Extent, Config.GroundZ), FVector(Offset, Extent, Config.GroundZ), GridColor, false, -1.0f, 0, 1.0f);
		}
	}

	if (bDrawBones)
	{
		for (int32 Body = 1; Body < Topo.NumBodies; ++Body)
		{
			DrawDebugLine(World, Batch.GetBodyPos(Topo.BodyParent[Body], 0), Batch.GetBodyPos(Body, 0), FColor(200, 200, 210), false, -1.0f, 0, 1.5f);
		}
	}

	if (bDrawBodyFrames)
	{
		for (int32 Body = 0; Body < Topo.NumBodies; ++Body)
		{
			const FVector Pos = Batch.GetBodyPos(Body, 0);
			const FQuat Rot = Batch.GetBodyRot(Body, 0);
			DrawDebugLine(World, Pos, Pos + Rot.RotateVector(FVector::XAxisVector) * DebugDrawScale, FColor::Red, false, -1.0f, 0, 1.0f);
			DrawDebugLine(World, Pos, Pos + Rot.RotateVector(FVector::YAxisVector) * DebugDrawScale, FColor::Green, false, -1.0f, 0, 1.0f);
			DrawDebugLine(World, Pos, Pos + Rot.RotateVector(FVector::ZAxisVector) * DebugDrawScale, FColor::Blue, false, -1.0f, 0, 1.0f);
		}
	}

	if (bDrawJointAxes)
	{
		// Drawn through the joint origin in BOTH directions, because an axis is
		// a line, not a ray -- a half-length arrow reads as a direction and
		// invites the wrong question. What you are checking is orientation
		// relative to the limb: a knee's axis should cross the leg, not run
		// along it. (That exact defect shipped undetected until it was seen in
		// the viewport -- SOLVER_DEBUG_LOG.md entry 022.)
		for (int32 Body = 1; Body < Topo.NumBodies; ++Body)
		{
			if (Topo.BodyDOFCount[Body] != 1)
			{
				continue;
			}
			const FVector Origin = Batch.GetBodyPos(Body, 0);
			const FVector Axis = FCreatureABASolver::RevoluteAxisWorld(Batch, Topo, Body, 0).GetSafeNormal();
			DrawDebugLine(World, Origin - Axis * DebugDrawScale, Origin + Axis * DebugDrawScale, FColor::Yellow, false, -1.0f, 0, 2.0f);
		}
	}

	if (bDrawVelocities)
	{
		// Normalised against the fastest body this frame, so the picture stays
		// readable whether the rig is drifting or exploding -- it shows which
		// body leads, which is the useful question, not absolute magnitude
		// (that is in the readout).
		const float Denom = FMath::Max(FastestBodySpeed, KINDA_SMALL_NUMBER);
		for (int32 Body = 0; Body < Topo.NumBodies; ++Body)
		{
			const int32 Idx = Batch.BodyIndex(Body, 0);
			const FVector LinVel(Batch.LinVelX[Idx], Batch.LinVelY[Idx], Batch.LinVelZ[Idx]);
			if (LinVel.IsNearlyZero())
			{
				continue;
			}
			const FVector Pos = Batch.GetBodyPos(Body, 0);
			const float Frac = static_cast<float>(LinVel.Size()) / Denom;
			DrawDebugDirectionalArrow(World, Pos, Pos + LinVel.GetSafeNormal() * DebugDrawScale * 2.0f * Frac,
				DebugDrawScale * 0.3f, FLinearColor(Frac, 1.0f - Frac, 0.0f).ToFColor(true), false, -1.0f, 0, 1.5f);
		}
	}

	if (bDrawContactPoints)
	{
		for (int32 i = 0; i < ContactPoints.Num(); ++i)
		{
			const CreatureGroundContact::FContactPointDef& Point = ContactPoints[i];
			const FVector BodyPos = Batch.GetBodyPos(Point.BodyIndex, 0);
			const FQuat BodyRot = Batch.GetBodyRot(Point.BodyIndex, 0);
			const FVector PointWorld = BodyPos + BodyRot.RotateVector(Point.LocalOffset);

			const bool bTouching = ContactStates.IsValidIndex(i) && ContactStates[i].bTouching;
			const FColor Color = bTouching ? FColor::Green : FColor(110, 110, 120);
			const float DrawRadius = Point.Radius > 0.0f ? Point.Radius : 3.0f;
			// Drawn at the authored Radius: the collision surface is the sphere,
			// not the center, and the difference is exactly the amount by which
			// a foot looks like it is floating when it is in fact resting.
			DrawDebugSphere(World, PointWorld, DrawRadius, 10, Color, false, -1.0f, 0, bTouching ? 2.0f : 1.0f);

			if (Point.CapsuleHalfHeight > 0.0f)
			{
				// Mirrors ResolveGroundContactImpulses' own end-cap derivation.
				const FVector Axis = Point.LocalOffset.GetSafeNormal();
				const FVector OtherEnd = BodyPos + BodyRot.RotateVector(Point.LocalOffset - Axis * (Point.CapsuleHalfHeight * 2.0f));
				DrawDebugLine(World, PointWorld, OtherEnd, Color, false, -1.0f, 0, 1.0f);
				DrawDebugSphere(World, OtherEnd, DrawRadius, 10, Color, false, -1.0f, 0, 1.0f);
			}

			// A vertical tick down to the ground plane whenever the point is
			// below it -- penetration you can see without reading a number.
			const float Penetration = Config.GroundZ - (static_cast<float>(PointWorld.Z) - Point.Radius);
			if (Penetration > 0.0f)
			{
				DrawDebugLine(World, PointWorld, FVector(PointWorld.X, PointWorld.Y, Config.GroundZ), FColor::Red, false, -1.0f, 0, 3.0f);
			}
		}
	}
}

void AMutoRagdollVisualizerActor::DrawStats() const
{
	if (!GEngine)
	{
		return;
	}
	const FCreatureTopology& Topo = Batch.GetTopology();

	auto BodyLabel = [this](int32 Body) -> FString
	{
		return BodyDebugNames.IsValidIndex(Body) ? BodyDebugNames[Body].ToString() : FString::Printf(TEXT("body%d"), Body);
	};

	// Negative keys would collide across actors; using distinct positive keys
	// keeps each line stable in place instead of scrolling.
	int32 Key = 71000;
	auto Line = [&Key](const FColor& Color, const FString& Text)
	{
		GEngine->AddOnScreenDebugMessage(Key++, 0.0f, Color, Text);
	};

	Line(FColor::White, FString::Printf(TEXT("[Muto ragdoll]  passive -- no muscles, gravity + contact only")));
	Line(FColor::White, FString::Printf(TEXT("  sim t = %.3f s   %s   speed %.2fx   substep %.1f Hz"),
		SimTime,
		bDiverged ? TEXT("DIVERGED") : (bPaused ? TEXT("PAUSED") : TEXT("running")),
		PlaybackSpeed,
		1.0f / FMath::Max(PhysicsSubstepDt, KINDA_SMALL_NUMBER)));

	if (bDiverged)
	{
		Line(FColor::Red, FString::Printf(TEXT("  DIVERGED at t = %.4f s: %s"), DivergedAtSimTime, *DivergenceReason));
		Line(FColor::Red, TEXT("  showing the last good substep. StepOnce() to push past it."));
	}

	Line(FColor::Green, FString::Printf(TEXT("  contacts touching: %d / %d      deepest penetration: %.2f cm"),
		NumTouchingContacts, ContactPoints.Num(), DeepestPenetration));
	Line(FColor::Yellow, FString::Printf(TEXT("  fastest body:  %s at %.1f cm/s"),
		FastestBodyIndex != INDEX_NONE ? *BodyLabel(FastestBodyIndex) : TEXT("-"), FastestBodySpeed));
	Line(FColor::Yellow, FString::Printf(TEXT("  fastest joint: DOF %d at %.1f deg/s"),
		FastestJointDOF, FastestJointSpeedDeg));
	Line(FColor(150, 150, 160), FString::Printf(TEXT("  %d bodies, %d DOF   contact %.0f Hz / zeta %.1f / %d iters"),
		Topo.NumBodies, Topo.NumDOF, ContactHertz, ContactDampingRatio, ContactIterations));
}

void AMutoRagdollVisualizerActor::DumpState()
{
	const FCreatureTopology& Topo = Batch.GetTopology();
	if (Topo.NumBodies == 0)
	{
		return;
	}

	UE_LOG(LogTemp, Log, TEXT("=== Muto ragdoll state dump, sim t = %.4f s ==="), SimTime);
	UE_LOG(LogTemp, Log, TEXT("  %-16s %-28s %-28s %10s"), TEXT("body"), TEXT("position"), TEXT("linear velocity"), TEXT("speed"));
	for (int32 Body = 0; Body < Topo.NumBodies; ++Body)
	{
		const int32 Idx = Batch.BodyIndex(Body, 0);
		const FVector Pos(Batch.PosX[Idx], Batch.PosY[Idx], Batch.PosZ[Idx]);
		const FVector LinVel(Batch.LinVelX[Idx], Batch.LinVelY[Idx], Batch.LinVelZ[Idx]);
		UE_LOG(LogTemp, Log, TEXT("  %-16s (%8.1f %8.1f %8.1f) (%8.1f %8.1f %8.1f) %10.1f"),
			BodyDebugNames.IsValidIndex(Body) ? *BodyDebugNames[Body].ToString() : *FString::Printf(TEXT("body%d"), Body),
			Pos.X, Pos.Y, Pos.Z, LinVel.X, LinVel.Y, LinVel.Z, LinVel.Size());
	}

	UE_LOG(LogTemp, Log, TEXT("  --- joints (deg, deg/s) ---"));
	for (int32 Body = 1; Body < Topo.NumBodies; ++Body)
	{
		const int32 DOFOffset = Topo.BodyDOFOffset[Body];
		const int32 DOFCount = Topo.BodyDOFCount[Body];
		FString Line;
		for (int32 k = 0; k < DOFCount; ++k)
		{
			const int32 Idx = Batch.DOFIndex(DOFOffset + k, 0);
			Line += FString::Printf(TEXT(" [%7.2f %9.2f]"),
				FMath::RadiansToDegrees(Batch.JointPos[Idx]), FMath::RadiansToDegrees(Batch.JointVel[Idx]));
		}
		UE_LOG(LogTemp, Log, TEXT("  %-16s %s%s"),
			BodyDebugNames.IsValidIndex(Body) ? *BodyDebugNames[Body].ToString() : *FString::Printf(TEXT("body%d"), Body),
			DOFCount == 3 ? TEXT("ball ") : TEXT("hinge"), *Line);
	}

	UE_LOG(LogTemp, Log, TEXT("  --- contacts ---"));
	for (int32 i = 0; i < ContactPoints.Num(); ++i)
	{
		const CreatureGroundContact::FContactPointDef& Point = ContactPoints[i];
		const FVector World = Batch.GetBodyPos(Point.BodyIndex, 0) + Batch.GetBodyRot(Point.BodyIndex, 0).RotateVector(Point.LocalOffset);
		const float Penetration = Config.GroundZ - (static_cast<float>(World.Z) - Point.Radius);
		UE_LOG(LogTemp, Log, TEXT("  %-16s z=%8.2f  pen=%7.2f  %s  normalForce=%.1f"),
			*Point.DebugName.ToString(), World.Z, Penetration,
			(ContactStates.IsValidIndex(i) && ContactStates[i].bTouching) ? TEXT("TOUCHING") : TEXT("clear   "),
			ContactStates.IsValidIndex(i) ? ContactStates[i].NormalForce : 0.0f);
	}
}

void AMutoRagdollVisualizerActor::UpdateMeshPose()
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
		// Body 0 is the synthetic fused "Torso"; the real bone is Pelvis. Every
		// torso-fused bone we never touch keeps its authored rest-pose local
		// transform and follows rigidly -- see AMutoRLVisualizerActor's copy of
		// this loop for the full reasoning.
		const FName BoneName = (Body == 0) ? TEXT("Pelvis") : BodyDebugNames[Body];
		MeshComponent->SetBoneTransformByName(BoneName, FTransform(Batch.GetBodyRot(Body, 0), Batch.GetBodyPos(Body, 0)), EBoneSpaces::WorldSpace);
	}
}

void AMutoRagdollVisualizerActor::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	// No Trainer and no networks to save -- deliberately skipping
	// AMutoRLTrainingDriver::EndPlay, which would look for both.
	AActor::EndPlay(EndPlayReason);
}
