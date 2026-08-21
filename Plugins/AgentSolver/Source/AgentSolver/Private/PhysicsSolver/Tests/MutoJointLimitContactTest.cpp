// TEMPORARY DIAGNOSTIC -- see SOLVER_DEBUG_LOG.md entry 023.
//
// User observation (2026-08-14), watching the passive ragdoll at 0.25x:
//
//   "The legs bend perfectly as expected, but when they both reach their
//    maximum flex angle on all of their bones they don't have any way of still
//    slowing down the fall. And this is right at this time that the heels
//    perforate the ground."
//
// with a substep-by-substep timeline: feet land at ~1, arms land 30-60, heels
// perforate at ~130, feet tips follow at ~150, knees under by 180, hips by 200,
// full divergence at ~450. Crucially the ARMS stay fine until the whole model
// has already collapsed -- and the arms are the limbs that never saturate their
// joint limits, because they carry almost no weight.
//
// That splits the failure cleanly: contact WORKS on limbs whose joints are
// inside their range, and FAILS on limbs whose joints have hit the end of it.
// The suspect is therefore not the contact solver as such, but the interaction
// between contact and joint limits.
//
// The mechanism this tests for: joint limits are NOT constraints in the solver.
// `ClampJointLimits` is a post-integration POSITION clamp that snaps JointPos to
// the boundary and zeroes JointVel (CreatureBatchSolver.h:1170). Nothing in the
// impulse path knows a joint is at its limit -- `ComputeArticulatedInertias`
// reduces every revolute out as freely rotating (`D = S.I^A.S`, subtract
// `U U^T / D`), with no notion of a locked DOF. So:
//
//   1. EFFECTIVE MASS IS TOO LOW. A leg folded solid against its limits is
//      physically a rigid strut that transmits load straight to the torso; the
//      contact there must decelerate most of the creature. The solver still
//      believes the leg can fold, so it reports a small articulated effective
//      mass and computes a correspondingly small impulse.
//
//   2. WHAT LITTLE IMPULSE IS APPLIED GETS DELETED. The impulse response puts
//      most of its velocity change into flexing those same joints further. On
//      the next step the clamp zeroes exactly those joint velocities. The
//      contact does work, and the clamp throws the result away.
//
// Both are the same root cause, and both predict precisely what was observed:
// the rig absorbs the landing while the joints have room, and stops being able
// to the instant they run out.
//
// Two measurements:
//
//   A. PASSIVE DROP TRACE. Run the drop exactly as the ragdoll visualizer does
//      and, every substep, record: how many DOFs are sitting at a limit, how
//      deep the worst contact has penetrated, the articulated effective mass
//      the solver reports at that contact, and what share of that contact's
//      impulse response lands in DOFs that are at a limit AND being pushed
//      further past it -- i.e. the share the clamp is about to delete. If the
//      hypothesis holds, saturation and penetration turn up together, and they
//      do so around substep 130.
//
//   B. EFFECTIVE MASS, FREE vs SATURATED. The same foot contact measured in the
//      standing pose and again once the legs have folded onto their limits. A
//      solver that understood limits would report a far larger effective mass
//      in the second case. This reports the ratio it actually gives.
//
// Neither test asserts a fix. They characterise a mechanism, and B is the
// number any fix has to move.
//
// Run with:
//   UnrealEditor-Cmd.exe <uproject> -ExecCmds="Automation RunTests AgentSolver.TEMP.JointLimitContact; Quit"
//     -unattended -nopause -nosplash -nullrhi -log=jointlimit.log

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "PhysicsSolver/CreatureBatchState.h"
#include "PhysicsSolver/CreatureBatchSolver.h"
#include "PhysicsSolver/CreatureGroundContact.h"
#include "AgentSolver/CreatureRLEnvironment.h"

#if WITH_EDITOR
#include "PhysicsSolver/MutoTopology.h"
#include "Engine/SkeletalMesh.h"
#endif

#if WITH_DEV_AUTOMATION_TESTS && WITH_EDITOR

namespace
{
	/** Tolerance for "this DOF is sitting on its limit", in degrees. */
	constexpr float AtLimitTolDeg = 0.5f;

	/** Angle relative to the range's minimum, wrapped into [0,360) -- the SAME convention ClampJointLimits uses. */
	float WrappedFromMin(float AngleDeg, float MinDeg)
	{
		float Wrapped = FMath::Fmod(AngleDeg - MinDeg, 360.0f);
		if (Wrapped < 0.0f)
		{
			Wrapped += 360.0f;
		}
		return Wrapped;
	}

	/**
	 * -1 if DOF d is pinned at its minimum, +1 at its maximum, 0 if it is free.
	 *
	 * MUST wrap, and an earlier version of this helper did not -- it compared
	 * raw degrees against DOFRangeMin/MaxDeg directly. That is wrong for this
	 * rig, whose authored ranges are things like [283.2, 403.6]: a joint at the
	 * bind pose reads 0 deg, which is INSIDE that range only once you know 0 is
	 * 360. The unwrapped version reported "0 DOFs at a limit" throughout a run
	 * where joints had wound past -8000 deg, which is exactly backwards. It is
	 * noted here rather than quietly corrected because the wrong version's
	 * output already came within one step of being reported as a finding.
	 *
	 * Revolutes only -- ball joints use a cone on the rotation vector's
	 * magnitude rather than per-DOF ranges, so "which end" is not defined.
	 */
	int32 RevoluteLimitSide(const FCreatureBatchState& Batch, const FCreatureTopology& Topo, int32 Body, int32 Env)
	{
		if (Topo.BodyDOFCount[Body] != 1)
		{
			return 0;
		}
		const int32 DOF = Topo.BodyDOFOffset[Body];
		if (!Topo.DOFHasMuscleCurve[DOF])
		{
			return 0;
		}
		const float Width = Topo.DOFRangeMaxDeg[DOF] - Topo.DOFRangeMinDeg[DOF];
		if (Width <= 0.0f)
		{
			return 0;
		}
		const float Wrapped = WrappedFromMin(FMath::RadiansToDegrees(Batch.JointPos[Batch.DOFIndex(DOF, Env)]), Topo.DOFRangeMinDeg[DOF]);
		if (Wrapped <= AtLimitTolDeg) return -1;
		if (Wrapped >= Width - AtLimitTolDeg && Wrapped <= Width + AtLimitTolDeg) return +1;
		return 0;
	}

	/**
	 * True if DOF d is currently OUTSIDE its authored range -- the condition
	 * ClampJointLimits exists to prevent, so any occurrence means the clamp
	 * failed rather than that the joint is merely at its stop.
	 */
	bool IsOutOfRange(const FCreatureBatchState& Batch, const FCreatureTopology& Topo, int32 Body, int32 Env)
	{
		if (Topo.BodyDOFCount[Body] != 1)
		{
			return false;
		}
		const int32 DOF = Topo.BodyDOFOffset[Body];
		if (!Topo.DOFHasMuscleCurve[DOF])
		{
			return false;
		}
		const float Width = Topo.DOFRangeMaxDeg[DOF] - Topo.DOFRangeMinDeg[DOF];
		if (Width <= 0.0f)
		{
			return false;
		}
		const float Wrapped = WrappedFromMin(FMath::RadiansToDegrees(Batch.JointPos[Batch.DOFIndex(DOF, Env)]), Topo.DOFRangeMinDeg[DOF]);
		return Wrapped > Width + AtLimitTolDeg && Wrapped < 360.0f - AtLimitTolDeg;
	}

	struct FContactProbe
	{
		float EffectiveMass = 0.0f;
		float SaturatedResponseShare = 0.0f; // 0..1
		int32 ContactIndex = INDEX_NONE;
		float Penetration = 0.0f;
	};

	/**
	 * Probes the deepest-penetrating contact: the articulated effective mass the
	 * solver reports along the ground normal, and how much of the resulting
	 * joint-velocity response lands in DOFs that are already at a limit and
	 * being driven further past it (which the clamp will zero next step).
	 */
	FContactProbe ProbeWorstContact(
		FCreatureBatchState& Batch, FCreatureABASolver& Solver,
		const TArray<CreatureGroundContact::FContactPointDef>& Points, float GroundZ)
	{
		const FCreatureTopology& Topo = Batch.GetTopology();
		FContactProbe Probe;

		for (int32 i = 0; i < Points.Num(); ++i)
		{
			const CreatureGroundContact::FContactPointDef& P = Points[i];
			const FVector W = Batch.GetBodyPos(P.BodyIndex, 0) + Batch.GetBodyRot(P.BodyIndex, 0).RotateVector(P.LocalOffset);
			const float Pen = GroundZ - (static_cast<float>(W.Z) - P.Radius);
			if (Pen > Probe.Penetration)
			{
				Probe.Penetration = Pen;
				Probe.ContactIndex = i;
			}
		}
		if (Probe.ContactIndex == INDEX_NONE)
		{
			return Probe;
		}

		const CreatureGroundContact::FContactPointDef& P = Points[Probe.ContactIndex];
		const FVector W = Batch.GetBodyPos(P.BodyIndex, 0) + Batch.GetBodyRot(P.BodyIndex, 0).RotateVector(P.LocalOffset);
		const FVector Normal = FVector::UpVector;

		Solver.ComputeArticulatedInertias(Batch);

		// Unit upward impulse at the contact point, then read back both the
		// point's own velocity change (-> effective mass) and the whole tree's
		// per-DOF response.
		TArray<FSpatialVec> DeltaV;
		TArray<float> DeltaQd;
		DeltaV.SetNum(Topo.NumBodies);
		DeltaQd.SetNumZeroed(FMath::Max(1, Topo.NumDOF));

		const FVector Arm = W - Batch.GetBodyPos(P.BodyIndex, 0);
		const FSpatialVec Wrench{ FVector::CrossProduct(Arm, Normal), Normal };
		Solver.SolveImpulseResponse(Batch, 0, P.BodyIndex, Wrench, DeltaV, DeltaQd);

		const FVector PointDeltaV = DeltaV[P.BodyIndex].Lin + FVector::CrossProduct(DeltaV[P.BodyIndex].Ang, Arm);
		const float Along = static_cast<float>(FVector::DotProduct(PointDeltaV, Normal));
		Probe.EffectiveMass = Along > UE_SMALL_NUMBER ? 1.0f / Along : 0.0f;

		// Share of the joint-velocity response sitting in DOFs the clamp is
		// about to zero. Magnitude-weighted, not energy-weighted -- a plain,
		// defensible proxy, not a claim about work done.
		float TotalAbs = 0.0f;
		float SaturatedAbs = 0.0f;
		for (int32 Body = 1; Body < Topo.NumBodies; ++Body)
		{
			if (Topo.BodyDOFCount[Body] != 1)
			{
				continue; // ball joints: cone limit, no per-DOF "side" to test
			}
			const int32 DOF = Topo.BodyDOFOffset[Body];
			const float Resp = FMath::Abs(DeltaQd[DOF]);
			TotalAbs += Resp;

			const int32 Side = RevoluteLimitSide(Batch, Topo, Body, 0);
			// Only counts as deleted if the response drives it FURTHER out. A
			// joint at its limit being pushed back into range is fine, and
			// counting that would inflate the number dishonestly.
			if ((Side > 0 && DeltaQd[DOF] > 0.0f) || (Side < 0 && DeltaQd[DOF] < 0.0f))
			{
				SaturatedAbs += Resp;
			}
		}
		Probe.SaturatedResponseShare = TotalAbs > UE_SMALL_NUMBER ? SaturatedAbs / TotalAbs : 0.0f;

		return Probe;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FMutoJointLimitContact,
	"AgentSolver.TEMP.JointLimitContact",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FMutoJointLimitContact::RunTest(const FString& Parameters)
{
	UMassMuscleProfileAssetMuscle* MuscleAsset = LoadObject<UMassMuscleProfileAssetMuscle>(nullptr, TEXT("/Game/Mesh/MuscleProfile_Muto.MuscleProfile_Muto"));
	UMassMuscleProfileAssetMass* MassAsset = LoadObject<UMassMuscleProfileAssetMass>(nullptr, TEXT("/Game/Mesh/MassProfile_Muto.MassProfile_Muto"));
	USkeletalMesh* SkeletalMesh = LoadObject<USkeletalMesh>(nullptr, TEXT("/Game/Mesh/Muto.Muto"));
	if (!TestNotNull(TEXT("MuscleProfile_Muto loaded"), MuscleAsset)) return false;
	if (!TestNotNull(TEXT("MassProfile_Muto loaded"), MassAsset)) return false;
	if (!TestNotNull(TEXT("Muto skeletal mesh loaded"), SkeletalMesh)) return false;

	FCreatureTopology Topo;
	TArray<FString> Warnings;
	TArray<FName> BodyDebugNames;
	if (!TestTrue(TEXT("BuildMutoTopology succeeded"), MutoTopology::BuildMutoTopology(SkeletalMesh, MassAsset, MuscleAsset, Topo, Warnings, &BodyDebugNames))) return false;

	const TArray<CreatureGroundContact::FContactPointDef> Points = CreatureGroundContact::BuildMutoContactPoints(Topo, MassAsset, BodyDebugNames);

	float TotalMass = 0.0f;
	for (int32 Body = 0; Body < Topo.NumBodies; ++Body)
	{
		TotalMass += Topo.BodyMass[Body];
	}

	// Driver defaults, so this trace is the same run the ragdoll visualizer shows.
	CreatureGroundContact::FImpulseContactParams Params;
	Params.GroundZ = 0.0f;
	Params.ContactHertz = 15.0f;
	Params.DampingRatio = 10.0f;
	Params.Slop = 0.5f;
	Params.FrictionCoefficient = 0.8f;
	Params.Iterations = 8;
	Params.RelaxIterations = 0;

	constexpr float Dt = 1.0f / 240.0f;
	const FVector Gravity(0.0f, 0.0f, -980.0f);

	FCreatureBatchState Batch;
	Batch.Init(Topo, 1);
	FCreatureABASolver Solver;
	CreatureGroundContact::FImpulseContactCache Cache;
	TArray<CreatureGroundContact::FContactPointState> States;
	States.SetNumZeroed(Points.Num());

	const FQuat StandRot = Topo.BodyRestRotInParent[0];
	const float StandHeight = 0.0f; // set below from the same helper the driver uses
	FRandomStream Stream(12345);

	// Same standing height derivation as the driver/visualizer, so "substep N"
	// here means the same instant it does on screen.
	float TargetHeight = 0.0f;
	{
		FCreatureBatchState Probe;
		Probe.Init(Topo, 1);
		Probe.SetBodyRot(0, 0, StandRot);
		Probe.SetBodyPos(0, 0, FVector::ZeroVector);
		FCreatureABASolver ProbeSolver;
		ProbeSolver.RecomputeKinematics(Probe);
		float LowestZ = 0.0f;
		for (const CreatureGroundContact::FContactPointDef& P : Points)
		{
			const FVector W = Probe.GetBodyPos(P.BodyIndex, 0) + Probe.GetBodyRot(P.BodyIndex, 0).RotateVector(P.LocalOffset);
			LowestZ = FMath::Min(LowestZ, static_cast<float>(W.Z) - P.Radius);
		}
		TargetHeight = -LowestZ;
	}
	(void)StandHeight;

	CreatureRLEnvironment::ResetEnv(Batch, 0, FVector(0.0f, 0.0f, TargetHeight), StandRot, Stream, 0.0f, 0.0f);
	Solver.RecomputeKinematics(Batch);

	AddInfo(FString::Printf(TEXT("Rig: %d bodies, %d DOF, %d contact points, %.0f kg total. Standing height %.1f cm."),
		Topo.NumBodies, Topo.NumDOF, Points.Num(), TotalMass, TargetHeight));

	// =====================================================================
	// A. PASSIVE DROP TRACE
	// =====================================================================
	AddInfo(TEXT("=== A. Passive drop: joint-limit saturation vs contact failure ==="));
	AddInfo(TEXT("  substep | atLimit | outOfRange | laps | touching | worstPen | effMass | worst contact"));

	constexpr int32 MaxSubsteps = 720; // 3 s at 240 Hz -- past the observed ~450 divergence
	int32 FirstSaturationSubstep = INDEX_NONE;
	int32 FirstPenetrationSubstep = INDEX_NONE;
	int32 FirstOutOfRangeSubstep = INDEX_NONE;
	int32 FirstLapSkipSubstep = INDEX_NONE;
	int32 DivergedSubstep = INDEX_NONE;
	float EffMassWhileFree = 0.0f;
	float EffMassWhileSaturated = 0.0f;

	// Lap index per DOF. ClampJointLimits keeps a clamped joint in the SAME lap
	// it was already in, so the lap index can only change if the joint crossed
	// the forbidden arc without being caught -- i.e. it TUNNELLED through its
	// own limit in a single substep. Tracking it turns "the knee wound up 23
	// revolutions" into a specific, dated event.
	TArray<int32> LastLap;
	LastLap.Init(0, FMath::Max(1, Topo.NumDOF));
	int32 TotalLapSkips = 0;

	for (int32 Substep = 1; Substep <= MaxSubsteps; ++Substep)
	{
		Solver.Step(Batch, Dt, Gravity);
		CreatureGroundContact::ResolveGroundContactImpulses(Batch, Topo, Points, Params, Solver, Dt, Cache, &States);

		bool bNonFinite = false;
		for (int32 Body = 0; Body < Topo.NumBodies && !bNonFinite; ++Body)
		{
			bNonFinite = Batch.GetBodyPos(Body, 0).ContainsNaN();
		}
		if (bNonFinite)
		{
			DivergedSubstep = Substep;
			AddInfo(FString::Printf(TEXT("  substep %d: NON-FINITE -- stopping trace"), Substep));
			break;
		}

		int32 NumAtLimit = 0;
		int32 NumOutOfRange = 0;
		for (int32 Body = 1; Body < Topo.NumBodies; ++Body)
		{
			if (RevoluteLimitSide(Batch, Topo, Body, 0) != 0)
			{
				++NumAtLimit;
			}
			if (IsOutOfRange(Batch, Topo, Body, 0))
			{
				++NumOutOfRange;
				if (FirstOutOfRangeSubstep == INDEX_NONE)
				{
					FirstOutOfRangeSubstep = Substep;
				}
			}
		}

		// Lap tracking: a change means the joint crossed its whole forbidden arc
		// inside one substep and the clamp never saw it out of range.
		int32 LapSkipsThisStep = 0;
		for (int32 Body = 1; Body < Topo.NumBodies; ++Body)
		{
			if (Topo.BodyDOFCount[Body] != 1) continue;
			const int32 DOF = Topo.BodyDOFOffset[Body];
			if (!Topo.DOFHasMuscleCurve[DOF]) continue;
			const float Deg = FMath::RadiansToDegrees(Batch.JointPos[Batch.DOFIndex(DOF, 0)]);
			const int32 Lap = FMath::FloorToInt((Deg - Topo.DOFRangeMinDeg[DOF]) / 360.0f);
			if (Substep > 1 && Lap != LastLap[DOF])
			{
				++LapSkipsThisStep;
				++TotalLapSkips;
				if (FirstLapSkipSubstep == INDEX_NONE)
				{
					FirstLapSkipSubstep = Substep;
				}
			}
			LastLap[DOF] = Lap;
		}

		int32 NumTouching = 0;
		for (const CreatureGroundContact::FContactPointState& S : States)
		{
			if (S.bTouching) ++NumTouching;
		}

		const FContactProbe P = ProbeWorstContact(Batch, Solver, Points, Params.GroundZ);

		if (NumAtLimit >= 6 && FirstSaturationSubstep == INDEX_NONE)
		{
			FirstSaturationSubstep = Substep;
		}
		// "Perforated" = past the authored slop, i.e. genuinely through rather
		// than resting with the tolerance the solver deliberately allows.
		if (P.Penetration > Params.Slop * 2.0f && FirstPenetrationSubstep == INDEX_NONE)
		{
			FirstPenetrationSubstep = Substep;
		}

		if (NumAtLimit == 0 && NumTouching > 0 && EffMassWhileFree == 0.0f)
		{
			EffMassWhileFree = P.EffectiveMass;
		}
		if (NumAtLimit >= 6 && EffMassWhileSaturated == 0.0f)
		{
			EffMassWhileSaturated = P.EffectiveMass;
		}

		// Dense logging through the interesting window, sparse elsewhere.
		const bool bInteresting = (Substep <= 5) || (Substep % 10 == 0 && Substep <= 260) || (Substep % 50 == 0);
		if (bInteresting || LapSkipsThisStep > 0)
		{
			AddInfo(FString::Printf(TEXT("  %7d | %7d | %10d | %4d | %8d | %8.2f | %7.0f | %s"),
				Substep, NumAtLimit, NumOutOfRange, LapSkipsThisStep, NumTouching, P.Penetration, P.EffectiveMass,
				(P.ContactIndex != INDEX_NONE) ? *Points[P.ContactIndex].DebugName.ToString() : TEXT("-")));
		}
	}

	AddInfo(FString::Printf(TEXT("  -> 6+ revolutes at a limit from substep %d"), FirstSaturationSubstep));
	AddInfo(FString::Printf(TEXT("  -> FIRST out-of-range (clamp failed) at substep %s"),
		FirstOutOfRangeSubstep != INDEX_NONE ? *FString::FromInt(FirstOutOfRangeSubstep) : TEXT("never")));
	AddInfo(FString::Printf(TEXT("  -> FIRST lap skip (tunnelled through the limit) at substep %s; %d lap skips total"),
		FirstLapSkipSubstep != INDEX_NONE ? *FString::FromInt(FirstLapSkipSubstep) : TEXT("never"), TotalLapSkips));
	AddInfo(FString::Printf(TEXT("  -> contact penetrated past 2x slop from substep %d"), FirstPenetrationSubstep));
	AddInfo(FString::Printf(TEXT("  -> non-finite at substep %s"),
		DivergedSubstep != INDEX_NONE ? *FString::FromInt(DivergedSubstep) : TEXT("(survived the trace)")));

	// =====================================================================
	// B. EFFECTIVE MASS, FREE vs SATURATED
	// =====================================================================
	AddInfo(TEXT("=== B. Articulated effective mass at the worst contact ==="));
	AddInfo(FString::Printf(TEXT("  joints free       %8.0f kg"), EffMassWhileFree));
	AddInfo(FString::Printf(TEXT("  joints saturated  %8.0f kg"), EffMassWhileSaturated));
	AddInfo(FString::Printf(TEXT("  creature total    %8.0f kg"), TotalMass));
	if (EffMassWhileFree > 0.0f && EffMassWhileSaturated > 0.0f)
	{
		AddInfo(FString::Printf(TEXT("  ratio saturated/free = %.2fx   (saturated is %.1f%% of total creature mass)"),
			EffMassWhileSaturated / EffMassWhileFree,
			100.0f * EffMassWhileSaturated / TotalMass));
	}

	// =====================================================================
	// C. RECONCILING "AT MAXIMUM FLEX" WITH "0 DOFs AT A LIMIT"
	// =====================================================================
	// The trace above reports essentially no joints at their limits during the
	// collapse, while the same collapse watched in the viewport looks exactly
	// like joints pinned at maximum flex. Both cannot be right, and the
	// difference decides what is actually broken. Three candidates:
	//   - the leg DOFs have no authored range at all, so ClampJointLimits skips
	//     them and they rotate without any stop (would make "maximum flex" a
	//     purely visual impression of a joint that simply keeps turning),
	//   - they have a range, but one so wide it is never reached,
	//   - they have a range, reach it, and something releases them again.
	// This replays the drop and records, per leg DOF, the extreme angle reached
	// against the authored range.
	AddInfo(TEXT("=== C. Leg joint ranges vs angles actually reached ==="));

	CreatureRLEnvironment::ResetEnv(Batch, 0, FVector(0.0f, 0.0f, TargetHeight), StandRot, Stream, 0.0f, 0.0f);
	Solver.RecomputeKinematics(Batch);
	Cache = CreatureGroundContact::FImpulseContactCache();
	for (CreatureGroundContact::FContactPointState& S : States) { S = CreatureGroundContact::FContactPointState(); }

	TArray<float> MinReachedDeg, MaxReachedDeg;
	MinReachedDeg.Init(TNumericLimits<float>::Max(), FMath::Max(1, Topo.NumDOF));
	MaxReachedDeg.Init(TNumericLimits<float>::Lowest(), FMath::Max(1, Topo.NumDOF));

	for (int32 Substep = 1; Substep <= 200; ++Substep)
	{
		Solver.Step(Batch, Dt, Gravity);
		CreatureGroundContact::ResolveGroundContactImpulses(Batch, Topo, Points, Params, Solver, Dt, Cache, &States);

		bool bBad = false;
		for (int32 Body = 0; Body < Topo.NumBodies && !bBad; ++Body) { bBad = Batch.GetBodyPos(Body, 0).ContainsNaN(); }
		if (bBad) break;

		for (int32 d = 0; d < Topo.NumDOF; ++d)
		{
			const float Deg = FMath::RadiansToDegrees(Batch.JointPos[Batch.DOFIndex(d, 0)]);
			MinReachedDeg[d] = FMath::Min(MinReachedDeg[d], Deg);
			MaxReachedDeg[d] = FMath::Max(MaxReachedDeg[d], Deg);
		}
	}

	AddInfo(TEXT("  body            dof  hasRange  authored range        reached range         verdict"));
	for (int32 Body = 1; Body < Topo.NumBodies; ++Body)
	{
		const FString Name = BodyDebugNames[Body].ToString();
		const bool bIsLeg = Name.Contains(TEXT("Knee")) || Name.Contains(TEXT("Feet")) || Name.Contains(TEXT("Hips"));
		if (!bIsLeg)
		{
			continue;
		}
		const int32 DOFOffset = Topo.BodyDOFOffset[Body];
		for (int32 k = 0; k < Topo.BodyDOFCount[Body]; ++k)
		{
			const int32 d = DOFOffset + k;
			const bool bHasRange = Topo.DOFHasMuscleCurve[d] != 0;
			const float Width = Topo.DOFRangeMaxDeg[d] - Topo.DOFRangeMinDeg[d];

			FString Verdict;
			if (!bHasRange)
			{
				Verdict = TEXT("NO RANGE -- never clamped, rotates freely");
			}
			else if (Width <= 0.0f)
			{
				Verdict = TEXT("ZERO/NEGATIVE WIDTH -- clamp skips it");
			}
			else
			{
				const float Span = MaxReachedDeg[d] - MinReachedDeg[d];
				Verdict = FString::Printf(TEXT("used %.0f%% of its %.0f deg range"), 100.0f * Span / Width, Width);
			}

			AddInfo(FString::Printf(TEXT("  %-14s %4d  %8s  [%8.1f %8.1f]  [%8.1f %8.1f]  %s"),
				*Name, d, bHasRange ? TEXT("yes") : TEXT("NO"),
				Topo.DOFRangeMinDeg[d], Topo.DOFRangeMaxDeg[d],
				MinReachedDeg[d], MaxReachedDeg[d], *Verdict));
		}
	}

	// =====================================================================
	// D. BEFORE / AFTER: joint limits as constraint rows
	// =====================================================================
	// The same drop, once with limits handled only by the post-integration
	// position clamp (the old behaviour) and once with them solved as rows in
	// the same sequential-impulse loop as contact. Same seed, same everything
	// else, so the difference is attributable.
	AddInfo(TEXT("=== D. Position clamp vs constraint rows ==="));

	struct FRunResult
	{
		int32 FirstOutOfRange = INDEX_NONE;
		int32 FirstLapSkip = INDEX_NONE;
		int32 TotalLapSkips = 0;
		int32 Diverged = INDEX_NONE;
		float PenAt150 = 0.0f;
		float PenAt250 = 0.0f;
		float MaxPen = 0.0f;
		float EffMassAtSaturation = 0.0f;
		float TorsoZAtEnd = 0.0f;
	};

	auto RunDrop = [&](const CreatureGroundContact::FJointLimitParams* Limits) -> FRunResult
	{
		FRunResult R;
		FCreatureBatchState B;
		B.Init(Topo, 1);
		FCreatureABASolver S;
		CreatureGroundContact::FImpulseContactCache C;
		TArray<CreatureGroundContact::FContactPointState> St;
		St.SetNumZeroed(Points.Num());
		FRandomStream Seeded(12345);
		CreatureRLEnvironment::ResetEnv(B, 0, FVector(0.0f, 0.0f, TargetHeight), StandRot, Seeded, 0.0f, 0.0f);
		S.RecomputeKinematics(B);

		TArray<int32> Laps;
		Laps.Init(0, FMath::Max(1, Topo.NumDOF));

		for (int32 Substep = 1; Substep <= MaxSubsteps; ++Substep)
		{
			S.Step(B, Dt, Gravity);
			CreatureGroundContact::ResolveGroundContactImpulses(B, Topo, Points, Params, S, Dt, C, &St, Limits);

			bool bBad = false;
			for (int32 Body = 0; Body < Topo.NumBodies && !bBad; ++Body) { bBad = B.GetBodyPos(Body, 0).ContainsNaN(); }
			if (bBad) { R.Diverged = Substep; break; }

			int32 NumAtLimit = 0;
			for (int32 Body = 1; Body < Topo.NumBodies; ++Body)
			{
				if (RevoluteLimitSide(B, Topo, Body, 0) != 0) ++NumAtLimit;
				if (IsOutOfRange(B, Topo, Body, 0) && R.FirstOutOfRange == INDEX_NONE) R.FirstOutOfRange = Substep;
			}
			for (int32 Body = 1; Body < Topo.NumBodies; ++Body)
			{
				if (Topo.BodyDOFCount[Body] != 1) continue;
				const int32 DOF = Topo.BodyDOFOffset[Body];
				if (!Topo.DOFHasMuscleCurve[DOF]) continue;
				const float Deg = FMath::RadiansToDegrees(B.JointPos[B.DOFIndex(DOF, 0)]);
				const int32 Lap = FMath::FloorToInt((Deg - Topo.DOFRangeMinDeg[DOF]) / 360.0f);
				if (Substep > 1 && Lap != Laps[DOF])
				{
					++R.TotalLapSkips;
					if (R.FirstLapSkip == INDEX_NONE) R.FirstLapSkip = Substep;
				}
				Laps[DOF] = Lap;
			}

			const FContactProbe P = ProbeWorstContact(B, S, Points, Params.GroundZ);
			R.MaxPen = FMath::Max(R.MaxPen, P.Penetration);
			if (Substep == 150) R.PenAt150 = P.Penetration;
			if (Substep == 250) R.PenAt250 = P.Penetration;
			if (NumAtLimit >= 6 && R.EffMassAtSaturation == 0.0f) R.EffMassAtSaturation = P.EffectiveMass;
			R.TorsoZAtEnd = static_cast<float>(B.GetBodyPos(0, 0).Z);
		}
		return R;
	};

	CreatureGroundContact::FJointLimitParams LimitsOn;
	LimitsOn.bEnabled = true;

	const FRunResult Before = RunDrop(nullptr);
	const FRunResult After = RunDrop(&LimitsOn);

	auto Fmt = [](int32 V) { return V != INDEX_NONE ? FString::FromInt(V) : FString(TEXT("never")); };

	AddInfo(TEXT("                              position clamp    constraint rows"));
	AddInfo(FString::Printf(TEXT("  first out-of-range substep  %14s   %15s"), *Fmt(Before.FirstOutOfRange), *Fmt(After.FirstOutOfRange)));
	AddInfo(FString::Printf(TEXT("  first tunnelled limit       %14s   %15s"), *Fmt(Before.FirstLapSkip), *Fmt(After.FirstLapSkip)));
	AddInfo(FString::Printf(TEXT("  total lap skips             %14d   %15d"), Before.TotalLapSkips, After.TotalLapSkips));
	AddInfo(FString::Printf(TEXT("  penetration @ substep 150   %14.2f   %15.2f"), Before.PenAt150, After.PenAt150));
	AddInfo(FString::Printf(TEXT("  penetration @ substep 250   %14.2f   %15.2f"), Before.PenAt250, After.PenAt250));
	AddInfo(FString::Printf(TEXT("  max penetration             %14.2f   %15.2f"), Before.MaxPen, After.MaxPen));
	AddInfo(FString::Printf(TEXT("  effMass once 6 joints locked%14.0f   %15.0f"), Before.EffMassAtSaturation, After.EffMassAtSaturation));
	AddInfo(FString::Printf(TEXT("  torso Z at end of trace     %14.1f   %15.1f"), Before.TorsoZAtEnd, After.TorsoZAtEnd));
	AddInfo(FString::Printf(TEXT("  diverged at substep         %14s   %15s"), *Fmt(Before.Diverged), *Fmt(After.Diverged)));

	// =====================================================================
	// E. WHY IS THE FOOT STILL 17 kg?
	// =====================================================================
	// Constraint rows do not change a row's effective mass, and cannot: the
	// diagonal a sequential-impulse row divides by is n.(J M^-1 J^T).n for the
	// UNCONSTRAINED system, by construction. What is supposed to produce
	// rigid-strut behaviour is CONVERGENCE of the coupled iteration -- contact
	// pushes, the limit rows resist, the next sweep carries a little more load
	// up the chain. For a serial leg that is Gauss-Seidel on a chain, which
	// needs at least as many sweeps as there are links just to propagate once.
	//
	// So: does throwing iterations at it converge to the braced answer? If yes,
	// the problem is convergence rate and the fixes are structural (better
	// ordering, block solves). If no, iteration is not the missing ingredient.
	AddInfo(TEXT("=== E. Does the coupled solve converge with more iterations? ==="));
	AddInfo(TEXT("  iters | pen@150 | pen@250 | maxPen | diverged"));

	for (int32 Iters : { 8, 16, 32, 64, 128 })
	{
		CreatureGroundContact::FImpulseContactParams SweepParams = Params;
		SweepParams.Iterations = Iters;

		FCreatureBatchState B; B.Init(Topo, 1);
		FCreatureABASolver S;
		CreatureGroundContact::FImpulseContactCache C;
		TArray<CreatureGroundContact::FContactPointState> St; St.SetNumZeroed(Points.Num());
		FRandomStream Seeded(12345);
		CreatureRLEnvironment::ResetEnv(B, 0, FVector(0.0f, 0.0f, TargetHeight), StandRot, Seeded, 0.0f, 0.0f);
		S.RecomputeKinematics(B);

		float Pen150 = 0.0f, Pen250 = 0.0f, MaxPen = 0.0f;
		int32 Div = INDEX_NONE;
		// 1440 substeps = 6 s at 240 Hz. Deliberately far past the ~1.5 s the
		// old configuration ever reached: "survived 400 substeps" would be a
		// weaker claim than it sounds, since 400 is only 1.67 s.
		constexpr int32 SweepSubsteps = 1440;
		for (int32 Substep = 1; Substep <= SweepSubsteps; ++Substep)
		{
			S.Step(B, Dt, Gravity);
			CreatureGroundContact::ResolveGroundContactImpulses(B, Topo, Points, SweepParams, S, Dt, C, &St, &LimitsOn);
			bool bBad = false;
			for (int32 Body = 0; Body < Topo.NumBodies && !bBad; ++Body) { bBad = B.GetBodyPos(Body, 0).ContainsNaN(); }
			if (bBad) { Div = Substep; break; }
			const FContactProbe P = ProbeWorstContact(B, S, Points, Params.GroundZ);
			MaxPen = FMath::Max(MaxPen, P.Penetration);
			if (Substep == 150) Pen150 = P.Penetration;
			if (Substep == 250) Pen250 = P.Penetration;
		}
		AddInfo(FString::Printf(TEXT("  %5d | %7.2f | %7.2f | %6.0f | %s"),
			Iters, Pen150, Pen250, MaxPen, Div != INDEX_NONE ? *FString::FromInt(Div) : TEXT("no")));
	}

	// =====================================================================
	// F. WHAT THE FOOT WOULD REPORT IF SATURATED JOINTS WERE WELDED
	// =====================================================================
	// The proposed structural fix, measured before committing to it: lock every
	// joint that is at a stop AND being pushed into it, inside the articulated
	// inertia factorization itself. Then the effective mass at the foot IS the
	// rigid-strut value and contact sizes its impulse correctly on the FIRST
	// iteration, with no convergence to wait for.
	AddInfo(TEXT("=== F. Effective mass at the foot with saturated joints welded ==="));

	{
		FCreatureBatchState B; B.Init(Topo, 1);
		FCreatureABASolver S;
		CreatureGroundContact::FImpulseContactCache C;
		TArray<CreatureGroundContact::FContactPointState> St; St.SetNumZeroed(Points.Num());
		FRandomStream Seeded(12345);
		CreatureRLEnvironment::ResetEnv(B, 0, FVector(0.0f, 0.0f, TargetHeight), StandRot, Seeded, 0.0f, 0.0f);
		S.RecomputeKinematics(B);

		TArray<uint8> Locked;
		Locked.SetNumZeroed(Topo.NumBodies);

		AddInfo(TEXT("  substep | locked | effMass free | effMass welded | ratio"));
		for (int32 Substep = 1; Substep <= 260; ++Substep)
		{
			S.Step(B, Dt, Gravity);
			CreatureGroundContact::ResolveGroundContactImpulses(B, Topo, Points, Params, S, Dt, C, &St, &LimitsOn);
			bool bBad = false;
			for (int32 Body = 0; Body < Topo.NumBodies && !bBad; ++Body) { bBad = B.GetBodyPos(Body, 0).ContainsNaN(); }
			if (bBad) { AddInfo(FString::Printf(TEXT("  non-finite at %d"), Substep)); break; }

			if (Substep % 20 != 0) continue;

			// Which joints are pinned at a stop right now?
			int32 NumLocked = 0;
			for (int32 Body = 1; Body < Topo.NumBodies; ++Body)
			{
				Locked[Body] = (RevoluteLimitSide(B, Topo, Body, 0) != 0) ? 1 : 0;
				NumLocked += Locked[Body];
			}

			// Worst contact, measured both ways against the SAME configuration.
			const FContactProbe Free = ProbeWorstContact(B, S, Points, Params.GroundZ);
			if (Free.ContactIndex == INDEX_NONE) continue;

			const CreatureGroundContact::FContactPointDef& CP = Points[Free.ContactIndex];
			const FVector W = B.GetBodyPos(CP.BodyIndex, 0) + B.GetBodyRot(CP.BodyIndex, 0).RotateVector(CP.LocalOffset);
			S.ComputeArticulatedInertias(B, Locked.GetData());
			const FVector Resp = S.ImpulseResponseAtPoint(B, 0, CP.BodyIndex, W, FVector::UpVector);
			const float Along = static_cast<float>(FVector::DotProduct(Resp, FVector::UpVector));
			const float WeldedMass = Along > UE_SMALL_NUMBER ? 1.0f / Along : 0.0f;

			AddInfo(FString::Printf(TEXT("  %7d | %6d | %12.0f | %14.0f | %.1fx"),
				Substep, NumLocked, Free.EffectiveMass, WeldedMass,
				Free.EffectiveMass > 0.0f ? WeldedMass / Free.EffectiveMass : 0.0f));
		}
	}

	// =====================================================================
	// G. GROUND COLLISION ON EVERY BODY
	// =====================================================================
	// Once the legs stopped being the first thing through the floor, the TORSO
	// became first instead -- and it has no contact point at all, so nothing
	// objected while it dragged the rig down behind it. This gives every body a
	// ground sphere at its joint origin. Ground only; no self-collision.
	AddInfo(TEXT("=== G. Authored contact points only vs every body ==="));

	const TArray<CreatureGroundContact::FContactPointDef> AllPoints =
		CreatureGroundContact::BuildMutoContactPoints(Topo, MassAsset, BodyDebugNames, /*bAllBodies=*/true, /*Fallback=*/10.0f);

	AddInfo(FString::Printf(TEXT("  contact points: %d authored -> %d with every body"), Points.Num(), AllPoints.Num()));

	auto RunWithPoints = [&](const TArray<CreatureGroundContact::FContactPointDef>& UsePoints, int32 Iters) -> FRunResult
	{
		CreatureGroundContact::FImpulseContactParams P2 = Params;
		P2.Iterations = Iters;

		FRunResult R;
		FCreatureBatchState B; B.Init(Topo, 1);
		FCreatureABASolver S;
		CreatureGroundContact::FImpulseContactCache C;
		TArray<CreatureGroundContact::FContactPointState> St; St.SetNumZeroed(UsePoints.Num());
		FRandomStream Seeded(12345);

		// Standing height must come from the SAME point set, or the rig starts
		// intersecting its own new geometry and the comparison is meaningless.
		float Lowest = 0.0f;
		{
			FCreatureBatchState Pr; Pr.Init(Topo, 1);
			Pr.SetBodyRot(0, 0, StandRot); Pr.SetBodyPos(0, 0, FVector::ZeroVector);
			FCreatureABASolver Ps; Ps.RecomputeKinematics(Pr);
			for (const CreatureGroundContact::FContactPointDef& CP : UsePoints)
			{
				const FVector W = Pr.GetBodyPos(CP.BodyIndex, 0) + Pr.GetBodyRot(CP.BodyIndex, 0).RotateVector(CP.LocalOffset);
				Lowest = FMath::Min(Lowest, static_cast<float>(W.Z) - CP.Radius);
			}
		}
		CreatureRLEnvironment::ResetEnv(B, 0, FVector(0.0f, 0.0f, -Lowest), StandRot, Seeded, 0.0f, 0.0f);
		S.RecomputeKinematics(B);

		for (int32 Substep = 1; Substep <= 1440; ++Substep)
		{
			S.Step(B, Dt, Gravity);
			CreatureGroundContact::ResolveGroundContactImpulses(B, Topo, UsePoints, P2, S, Dt, C, &St, &LimitsOn);
			bool bBad = false;
			for (int32 Body = 0; Body < Topo.NumBodies && !bBad; ++Body) { bBad = B.GetBodyPos(Body, 0).ContainsNaN(); }
			if (bBad) { R.Diverged = Substep; break; }

			const FContactProbe P = ProbeWorstContact(B, S, UsePoints, Params.GroundZ);
			R.MaxPen = FMath::Max(R.MaxPen, P.Penetration);
			if (Substep == 150) R.PenAt150 = P.Penetration;
			if (Substep == 250) R.PenAt250 = P.Penetration;
			R.TorsoZAtEnd = static_cast<float>(B.GetBodyPos(0, 0).Z);
		}
		return R;
	};

	AddInfo(TEXT("                          authored only    every body"));
	for (int32 Iters : { 8, 32 })
	{
		const FRunResult A = RunWithPoints(Points, Iters);
		const FRunResult Bres = RunWithPoints(AllPoints, Iters);
		AddInfo(FString::Printf(TEXT("  [%d iters] pen@250     %13.2f  %12.2f"), Iters, A.PenAt250, Bres.PenAt250));
		AddInfo(FString::Printf(TEXT("  [%d iters] maxPen      %13.0f  %12.0f"), Iters, A.MaxPen, Bres.MaxPen));
		AddInfo(FString::Printf(TEXT("  [%d iters] torsoZ end  %13.1f  %12.1f"), Iters, A.TorsoZAtEnd, Bres.TorsoZAtEnd));
		AddInfo(FString::Printf(TEXT("  [%d iters] diverged    %13s  %12s"), Iters, *Fmt(A.Diverged), *Fmt(Bres.Diverged)));
	}

	// =====================================================================
	// H. LONG RUN -- is "never diverged" a fact about the solver or the window?
	// =====================================================================
	// Section G's 1440 substeps is 6 s. The old configuration failed by ~2.2 s,
	// so 6 s already means something -- but a 400-substep window flattered an
	// earlier result in this very file (entry 024), and "survived the trace" is
	// a claim about the trace until the trace is long enough to be boring.
	// 12000 substeps = 50 s.
	AddInfo(TEXT("=== H. 50-second passive run, every body colliding ==="));
	{
		FCreatureBatchState B; B.Init(Topo, 1);
		FCreatureABASolver S;
		CreatureGroundContact::FImpulseContactCache C;
		TArray<CreatureGroundContact::FContactPointState> St; St.SetNumZeroed(AllPoints.Num());
		FRandomStream Seeded(12345);

		float Lowest = 0.0f;
		{
			FCreatureBatchState Pr; Pr.Init(Topo, 1);
			Pr.SetBodyRot(0, 0, StandRot); Pr.SetBodyPos(0, 0, FVector::ZeroVector);
			FCreatureABASolver Ps; Ps.RecomputeKinematics(Pr);
			for (const CreatureGroundContact::FContactPointDef& CP : AllPoints)
			{
				const FVector W = Pr.GetBodyPos(CP.BodyIndex, 0) + Pr.GetBodyRot(CP.BodyIndex, 0).RotateVector(CP.LocalOffset);
				Lowest = FMath::Min(Lowest, static_cast<float>(W.Z) - CP.Radius);
			}
		}
		CreatureRLEnvironment::ResetEnv(B, 0, FVector(0.0f, 0.0f, -Lowest), StandRot, Seeded, 0.0f, 0.0f);
		S.RecomputeKinematics(B);

		constexpr int32 LongSubsteps = 12000;
		int32 Div = INDEX_NONE;
		float MaxPen = 0.0f, MaxSpeed = 0.0f;
		for (int32 Substep = 1; Substep <= LongSubsteps; ++Substep)
		{
			S.Step(B, Dt, Gravity);
			CreatureGroundContact::ResolveGroundContactImpulses(B, Topo, AllPoints, Params, S, Dt, C, &St, &LimitsOn);

			bool bBad = false;
			for (int32 Body = 0; Body < Topo.NumBodies && !bBad; ++Body) { bBad = B.GetBodyPos(Body, 0).ContainsNaN(); }
			if (bBad) { Div = Substep; break; }

			if (Substep % 20 == 0)
			{
				const FContactProbe P = ProbeWorstContact(B, S, AllPoints, Params.GroundZ);
				MaxPen = FMath::Max(MaxPen, P.Penetration);
			}
			for (int32 Body = 0; Body < Topo.NumBodies; ++Body)
			{
				const int32 Idx = B.BodyIndex(Body, 0);
				MaxSpeed = FMath::Max(MaxSpeed, (float)FVector(B.LinVelX[Idx], B.LinVelY[Idx], B.LinVelZ[Idx]).Size());
			}

			if (Substep % 2400 == 0)
			{
				AddInfo(FString::Printf(TEXT("  t=%5.1f s   torsoZ=%8.2f   maxPen so far=%7.2f   maxSpeed so far=%9.1f"),
					Substep * Dt, B.GetBodyPos(0, 0).Z, MaxPen, MaxSpeed));
			}
		}
		AddInfo(FString::Printf(TEXT("  -> %s over 50 s"),
			Div != INDEX_NONE ? *FString::Printf(TEXT("DIVERGED at substep %d"), Div) : TEXT("survived, finite throughout")));
	}

	// This test characterises a mechanism; it does not assert a fix. The only
	// thing it fails on is being unable to run the scenario at all.
	TestTrue(TEXT("Trace produced a contact probe at some point"), EffMassWhileFree > 0.0f || EffMassWhileSaturated > 0.0f);

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS && WITH_EDITOR
