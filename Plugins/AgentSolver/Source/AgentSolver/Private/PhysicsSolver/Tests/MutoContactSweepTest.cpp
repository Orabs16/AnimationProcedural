// TEMPORARY diagnostic — re-fits the shipped ground-contact constants against the
// CURRENT rig. This is OPEN_ITEMS.md's O-07, which three separate log entries said
// was overdue:
//
//   entry 022 — every contact number in entries 013-021 was measured on a rig whose
//               26 revolutes rotated about the WRONG AXES.
//   entry 025 — those same runs had NO TORSO COLLISION.
//   entry 028 — and they were fitted against a mass matrix built from the thin-rod
//               inertia placeholder, whose axial term moved by up to 77.5x when the
//               authored capsule radii replaced it.
//
// ContactHertz=15 / DampingRatio=10 come from entry 018's sweep, so they are fitted
// to a creature that differed from this one in its joint axes, its collision set AND
// how hard its bodies are to rotate. Nothing about that fit is trustworthy.
//
// TWO REGIMES, because every previous sweep in this project's history used only the
// first, and O-08 records that nobody has ever tested the actuated case:
//
//   DROP  - passive drop from the standing pose, zero torque. Comparable to every
//           historical number, and a worst case for penetration.
//   DRIVE - the same drop with TORQUE BABBLE at the real MaxTorquePerDOF, held for
//           4 substeps per draw to match the driver's decision/substep ratio. This
//           is the first time the muscle multipliers from entry 027 (0.5x-5x, see
//           X-08) have been exercised by anything at all.
//
// SELECTION RULE, stated before the numbers so it cannot be fitted to them:
//   1. HARD GATES, both regimes: no divergence, maxPen < 60 cm, maxJointSpeed < 500.
//   2. Among survivors, minimise RESTING penetration (mean over the last quarter of
//      the run). A creature that must learn to stand needs its feet to stop sinking;
//      that is the quality metric that matters for training.
//   3. Ties broken by lower PEAK penetration, which is the impact transient O-06
//      tracks and which is re-paid at every episode reset.
//
// Delete once the numbers are recorded in SOLVER_DEBUG_LOG.md, the way entry 021
// deleted four answered diagnostics.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "PhysicsSolver/CreatureBatchSolver.h"
#include "PhysicsSolver/CreatureGroundContact.h"
#include "AgentSolver/CreatureRLEnvironment.h"
#include "AgentSolver/MutoRLTrainingDriver.h"

#if WITH_EDITOR
#include "PhysicsSolver/MutoTopology.h"
#include "Engine/SkeletalMesh.h"
#include "UMassMuscleProfileAsset.h"
#endif

#if WITH_DEV_AUTOMATION_TESTS && WITH_EDITOR

namespace MutoContactSweep
{
	constexpr float Dt = 1.0f / 240.0f;
	const FVector Gravity(0.0f, 0.0f, -980.0f);

	/** Everything one run reports. */
	struct FResult
	{
		float MaxPen = 0.0f;
		float RestPen = 0.0f;   // mean over the last quarter — a single last-sample reading is noise
		float TorsoZ = 0.0f;
		float MaxJointSpeed = 0.0f;
		double MsPerSubstep = 0.0;
		int32 DivergedAt = INDEX_NONE;

		bool Survived() const { return DivergedAt == INDEX_NONE; }
		bool PassesGates() const { return Survived() && MaxPen < 60.0f && MaxJointSpeed < 500.0f; }
	};

	/**
	 * One run at a given (Hertz, DampingRatio). Everything else is held at the
	 * shipped defaults, so this measures the two constants O-07 names and not a
	 * combination of them with something else.
	 */
	static FResult Run(
		USkeletalMesh* SkeletalMesh,
		UMassMuscleProfileAssetMass* MassAsset,
		UMassMuscleProfileAssetMuscle* MuscleAsset,
		float ContactHertz,
		float DampingRatio,
		float TorqueScale,        // 0 = passive drop; fraction of MaxTorquePerDOF otherwise
		bool bNeutraliseMuscleStrength, // force every DOF's authored strength to 1, to isolate X-08
		int32 NumSubsteps)
	{
		const bool bDriven = TorqueScale > 0.0f;
		FResult R;

		FCreatureTopology Topo;
		TArray<FString> Warnings;
		TArray<FName> BodyDebugNames;
		if (!MutoTopology::BuildMutoTopology(SkeletalMesh, MassAsset, MuscleAsset, Topo, Warnings, &BodyDebugNames))
		{
			R.DivergedAt = 0;
			return R;
		}
		// Shipped passive-joint defaults (AMutoRLTrainingDriver::ApplyPassiveJointDefaults).
		for (int32 DOF = 0; DOF < Topo.NumDOF; ++DOF)
		{
			Topo.DOFArmatureRatio[DOF] = 0.05f;
			Topo.DOFDampingTimeConstant[DOF] = 1.0f;
			if (bNeutraliseMuscleStrength)
			{
				// The pre-entry-027 world: authored curves still applied, but the scalar
				// strengths pinned at 1 instead of spanning [0.5, 5.0]. Isolates whether
				// a divergence is the contact constants' fault or X-08's.
				Topo.DOFExtensionStrength[DOF] = 1.0f;
				Topo.DOFFlexionStrength[DOF] = 1.0f;
			}
		}

		const TArray<CreatureGroundContact::FContactPointDef> Points =
			CreatureGroundContact::BuildMutoContactPoints(Topo, MassAsset, BodyDebugNames, /*bAllBodies=*/true, /*Fallback=*/10.0f);
		const TArray<CreatureGroundContact::FLimbPairDef> Pairs =
			CreatureGroundContact::BuildMutoLimbCollisionPairs(Topo);

		CreatureGroundContact::FImpulseContactParams Params;
		Params.GroundZ = 0.0f;
		Params.ContactHertz = ContactHertz;   // swept
		Params.DampingRatio = DampingRatio;   // swept
		Params.Slop = 0.5f;
		Params.FrictionCoefficient = 0.8f;
		Params.Iterations = 16;
		Params.RelaxIterations = 0;
		Params.bUseGlobalSolve = true;
		Params.GlobalIterations = 64;
		Params.Cfm = 1.0e-8f;

		CreatureGroundContact::FJointLimitParams Limits;
		Limits.bEnabled = true;
		Limits.Hertz = 60.0f;
		Limits.DampingRatio = 5.0f;
		Limits.SlopDeg = 0.25f;
		Limits.MarginDeg = 3.0f;
		Limits.Cfm = 1.0e-8f;

		CreatureGroundContact::FLimbCollisionParams LimbCol;
		LimbCol.bEnabled = true;
		LimbCol.Hertz = 30.0f;
		LimbCol.DampingRatio = 10.0f;
		LimbCol.Slop = 0.5f;
		LimbCol.Cfm = 1.0e-8f;

		FCreatureBatchState Batch;
		Batch.Init(Topo, 1);
		FCreatureABASolver Solver;
		CreatureGroundContact::FImpulseContactCache Cache;
		TArray<CreatureGroundContact::FContactPointState> States;
		States.SetNumZeroed(Points.Num());
		TArray<uint8> Locks;

		const FQuat StandRot = Topo.BodyRestRotInParent[0];
		const float StandHeight = AMutoRLTrainingDriver::ComputeDefaultStandingHeight(Topo, Points, StandRot);
		FRandomStream Stream(12345);
		CreatureRLEnvironment::ResetEnv(Batch, 0, FVector(0.0f, 0.0f, StandHeight), StandRot, Stream, 0.0f, 0.0f);
		Solver.RecomputeKinematics(Batch);

		// Torque babble. Same seed for every configuration in the sweep, so two
		// configs see the SAME actuation and differ only in the contact constants
		// -- otherwise this would be comparing noise draws.
		FRandomStream TorqueStream(777);
		constexpr float MaxTorquePerDOF = 5.0e7f; // FEnvConfig's shipped value
		constexpr int32 SubstepsPerDecision = 4;  // 1/60 decision over 1/240 substeps

		const int32 RestWindowStart = NumSubsteps - NumSubsteps / 4;
		double RestPenSum = 0.0;
		int32 RestPenCount = 0;

		const double StartTime = FPlatformTime::Seconds();
		int32 SubstepsRun = 0;

		for (int32 Substep = 1; Substep <= NumSubsteps; ++Substep)
		{
			SubstepsRun = Substep;

			if (bDriven && ((Substep - 1) % SubstepsPerDecision) == 0)
			{
				// A fresh action every decision, held across the substeps between --
				// exactly how ApplyActions + StepPhysicsSubstepped behave. Drawn in
				// [-1,1] and scaled, i.e. the same range a policy can emit, so the
				// muscle multiplier sees realistic input.
				for (int32 DOF = 0; DOF < Topo.NumDOF; ++DOF)
				{
					Batch.JointTorque[Batch.DOFIndex(DOF, 0)] = TorqueStream.FRandRange(-1.0f, 1.0f) * MaxTorquePerDOF * TorqueScale;
				}
			}

			Solver.Step(Batch, Dt, Gravity);
			Solver.ApplyJointDamping(Batch, Dt);

			Solver.BuildSaturatedJointLocks(Batch, 1.0f, Locks);

			CreatureGroundContact::ResolveGroundContactImpulses(
				Batch, Topo, Points, Params, Solver, Dt, Cache, &States, &Limits,
				Pairs, &LimbCol, nullptr, Locks.GetData(), 1);

			if (!CreatureRLEnvironment::IsBodyStateValid(Batch, 0))
			{
				R.DivergedAt = Substep;
				break;
			}

			float Pen = 0.0f;
			for (const CreatureGroundContact::FContactPointDef& P : Points)
			{
				FVector Surfaces[2];
				const int32 NumEnds = CreatureGroundContact::GetContactPointWorldSurfaces(Batch, P, 0, FVector::UpVector, Surfaces);
				for (int32 e = 0; e < NumEnds; ++e)
				{
					Pen = FMath::Max(Pen, -(float)Surfaces[e].Z);
				}
			}
			R.MaxPen = FMath::Max(R.MaxPen, Pen);
			if (Substep >= RestWindowStart)
			{
				RestPenSum += Pen;
				++RestPenCount;
			}

			for (int32 DOF = 0; DOF < Topo.NumDOF; ++DOF)
			{
				R.MaxJointSpeed = FMath::Max(R.MaxJointSpeed, FMath::Abs(Batch.JointVel[Batch.DOFIndex(DOF, 0)]));
			}
		}

		R.MsPerSubstep = 1000.0 * (FPlatformTime::Seconds() - StartTime) / FMath::Max(1, SubstepsRun);
		R.RestPen = RestPenCount > 0 ? (float)(RestPenSum / RestPenCount) : 0.0f;
		R.TorsoZ = R.Survived() ? (float)Batch.GetBodyPos(0, 0).Z : 0.0f;
		return R;
	}

	static FString Format(const FResult& R)
	{
		return FString::Printf(TEXT("%9.2f | %8.2f | %9.2f | %11.1f | %8.3f | %s"),
			R.MaxPen, R.RestPen, R.TorsoZ, R.MaxJointSpeed, R.MsPerSubstep,
			R.Survived() ? TEXT("never") : *FString::Printf(TEXT("substep %d (%.2f s)"), R.DivergedAt, R.DivergedAt * Dt));
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FMutoContactSweep,
	"AgentSolver.TEMP.ContactSweep",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)


bool FMutoContactSweep::RunTest(const FString& Parameters)
{
	using namespace MutoContactSweep;

	UMassMuscleProfileAssetMuscle* MuscleAsset = LoadObject<UMassMuscleProfileAssetMuscle>(nullptr, TEXT("/Game/Mesh/MuscleProfile_Muto.MuscleProfile_Muto"));
	UMassMuscleProfileAssetMass* MassAsset = LoadObject<UMassMuscleProfileAssetMass>(nullptr, TEXT("/Game/Mesh/MassProfile_Muto.MassProfile_Muto"));
	USkeletalMesh* SkeletalMesh = LoadObject<USkeletalMesh>(nullptr, TEXT("/Game/Mesh/Muto.Muto"));
	if (!TestNotNull(TEXT("MuscleProfile_Muto loaded"), MuscleAsset)) return false;
	if (!TestNotNull(TEXT("MassProfile_Muto loaded"), MassAsset)) return false;
	if (!TestNotNull(TEXT("Muto skeletal mesh loaded"), SkeletalMesh)) return false;

	// 6 s passively (everything survives that regime, so its job is ranking quality)
	// and 12 s driven. The 12 is not arbitrary: a first pass at 6 s driven picked a
	// winner that then diverged at 12.33 s in the confirmation run. Selecting on a
	// window shorter than the failure mode is how you fit to noise.
	constexpr int32 DropSubsteps = 1440;   // 6 s
	constexpr int32 DriveSubsteps = 2880;  // 12 s

	// The amplitude a randomly-initialised policy actually explores at. A uniform
	// draw over the FULL +/-1 action range on all 68 DOFs is not that -- it is a
	// worst case, and it is measured separately in the ladder at the end rather
	// than being used to pick the shipped constants.
	constexpr float ExploreScale = 0.3f;

	const TArray<float> HertzValues{ 10.0f, 15.0f, 20.0f, 30.0f, 45.0f, 60.0f };
	const TArray<float> ZetaValues{ 2.0f, 5.0f, 10.0f, 20.0f };

	AddInfo(TEXT("O-07 re-fit. Real 41-body rig, capsule inertia, shipped everything-else."));
	AddInfo(TEXT("restPen = MEAN penetration over the last quarter of the run."));
	AddInfo(FString::Printf(TEXT("Passive %.0f s; driven %.0f s at %.0f%% of MaxTorquePerDOF."),
		DropSubsteps * Dt, DriveSubsteps * Dt, ExploreScale * 100.0f));

	struct FCell { float Hz; float Zeta; FResult Drop; FResult Drive; };
	TArray<FCell> Cells;

	for (int32 Pass = 0; Pass < 2; ++Pass)
	{
		const bool bDriven = (Pass == 1);
		AddInfo(TEXT(""));
		AddInfo(bDriven
			? TEXT("=== DRIVEN (torque babble at 30% amplitude, 12 s) ===")
			: TEXT("=== PASSIVE DROP (zero torque, 6 s) ==="));
		AddInfo(TEXT("   Hz  zeta |    maxPen |  restPen |    torsoZ |    maxSpeed |   ms/sub | diverged"));

		int32 Cell = 0;
		for (float Hz : HertzValues)
		{
			for (float Zeta : ZetaValues)
			{
				const FResult R = Run(SkeletalMesh, MassAsset, MuscleAsset, Hz, Zeta,
					bDriven ? ExploreScale : 0.0f, /*bNeutralise=*/false,
					bDriven ? DriveSubsteps : DropSubsteps);
				if (!bDriven) { Cells.Add(FCell{ Hz, Zeta, R, FResult{} }); }
				else          { Cells[Cell].Drive = R; }
				++Cell;
				AddInfo(FString::Printf(TEXT(" %4.0f  %4.0f | %s"), Hz, Zeta, *Format(R)));
			}
		}
	}

	// ---- Apply the selection rule ----
	AddInfo(TEXT(""));
	AddInfo(TEXT("=== SELECTION (gates: survives BOTH regimes, maxPen < 60, maxSpeed < 500) ==="));

	int32 BestIdx = INDEX_NONE;
	int32 NumPassing = 0;
	for (int32 i = 0; i < Cells.Num(); ++i)
	{
		const FCell& C = Cells[i];
		if (!C.Drop.PassesGates() || !C.Drive.PassesGates()) { continue; }
		++NumPassing;
		if (BestIdx == INDEX_NONE) { BestIdx = i; continue; }
		const FCell& B = Cells[BestIdx];
		// Ranked on the WORSE of the two regimes, so a pair that is excellent
		// passively and mediocre under load cannot win on the passive half alone.
		const float CRest = FMath::Max(C.Drop.RestPen, C.Drive.RestPen);
		const float BRest = FMath::Max(B.Drop.RestPen, B.Drive.RestPen);
		const float CPeak = FMath::Max(C.Drop.MaxPen, C.Drive.MaxPen);
		const float BPeak = FMath::Max(B.Drop.MaxPen, B.Drive.MaxPen);
		if (CRest < BRest - 1.0e-3f || (FMath::IsNearlyEqual(CRest, BRest, 1.0e-3f) && CPeak < BPeak))
		{
			BestIdx = i;
		}
	}

	AddInfo(FString::Printf(TEXT("%d of %d pairs pass both regimes."), NumPassing, Cells.Num()));
	TestTrue(TEXT("at least one (Hertz, DampingRatio) pair survives both regimes"), BestIdx != INDEX_NONE);
	if (BestIdx == INDEX_NONE) { return false; }

	AddInfo(FString::Printf(TEXT("Leader on the 12 s gate: ContactHertz=%.0f DampingRatio=%.0f"), Cells[BestIdx].Hz, Cells[BestIdx].Zeta));

	for (const FCell& C : Cells)
	{
		if (FMath::IsNearlyEqual(C.Hz, 15.0f) && FMath::IsNearlyEqual(C.Zeta, 10.0f))
		{
			AddInfo(TEXT("INCUMBENT (15, 10):"));
			AddInfo(FString::Printf(TEXT("   drop : %s"), *Format(C.Drop)));
			AddInfo(FString::Printf(TEXT("   drive: %s"), *Format(C.Drive)));
			break;
		}
	}

	// ---- FINALIST ROUND ----
	//
	// Gate 1 of the selection rule says "no divergence". It was being tested over
	// 12 s, and 12 s is not long enough: a first pass at 6 s picked a winner that
	// died at 12.33 s, and the 12 s pass picked one that died at 28.36 s. Each time
	// the gate was shorter than the failure mode. So the gate is extended rather
	// than the rule changed -- every pair that passed 12 s is re-run for the full
	// 50 s in BOTH regimes, and only survivors are ranked.
	AddInfo(TEXT(""));
	AddInfo(TEXT("=== FINALIST ROUND: every 12 s survivor re-run for 50 s, both regimes ==="));
	AddInfo(TEXT("   Hz  zeta | regime |    maxPen |  restPen |    torsoZ |    maxSpeed |   ms/sub | diverged"));

	struct FFinal { int32 Idx; FResult Drop; FResult Drive; };
	TArray<FFinal> Finalists;
	for (int32 i = 0; i < Cells.Num(); ++i)
	{
		if (!Cells[i].Drop.PassesGates() || !Cells[i].Drive.PassesGates()) { continue; }
		const FResult D  = Run(SkeletalMesh, MassAsset, MuscleAsset, Cells[i].Hz, Cells[i].Zeta, 0.0f, false, 12000);
		const FResult Dr = Run(SkeletalMesh, MassAsset, MuscleAsset, Cells[i].Hz, Cells[i].Zeta, ExploreScale, false, 12000);
		AddInfo(FString::Printf(TEXT(" %4.0f  %4.0f | drop   | %s"), Cells[i].Hz, Cells[i].Zeta, *Format(D)));
		AddInfo(FString::Printf(TEXT(" %4.0f  %4.0f | drive  | %s"), Cells[i].Hz, Cells[i].Zeta, *Format(Dr)));
		if (D.PassesGates() && Dr.PassesGates())
		{
			Finalists.Add(FFinal{ i, D, Dr });
		}
	}

	AddInfo(TEXT(""));
	AddInfo(FString::Printf(TEXT("%d of %d twelve-second survivors also survive 50 s in both regimes."), Finalists.Num(), NumPassing));

	int32 WinIdx = INDEX_NONE;
	for (int32 i = 0; i < Finalists.Num(); ++i)
	{
		if (WinIdx == INDEX_NONE) { WinIdx = i; continue; }
		const float CRest = FMath::Max(Finalists[i].Drop.RestPen, Finalists[i].Drive.RestPen);
		const float BRest = FMath::Max(Finalists[WinIdx].Drop.RestPen, Finalists[WinIdx].Drive.RestPen);
		const float CPeak = FMath::Max(Finalists[i].Drop.MaxPen, Finalists[i].Drive.MaxPen);
		const float BPeak = FMath::Max(Finalists[WinIdx].Drop.MaxPen, Finalists[WinIdx].Drive.MaxPen);
		if (CRest < BRest - 1.0e-3f || (FMath::IsNearlyEqual(CRest, BRest, 1.0e-3f) && CPeak < BPeak))
		{
			WinIdx = i;
		}
	}

	TestTrue(TEXT("at least one pair survives 50 s in both regimes"), WinIdx != INDEX_NONE);
	if (WinIdx == INDEX_NONE) { return false; }

	const FCell& Best = Cells[Finalists[WinIdx].Idx];
	AddInfo(FString::Printf(TEXT("WINNER: ContactHertz=%.0f  DampingRatio=%.0f"), Best.Hz, Best.Zeta));
	AddInfo(FString::Printf(TEXT("   drop  50 s: %s"), *Format(Finalists[WinIdx].Drop)));
	AddInfo(FString::Printf(TEXT("   drive 50 s: %s"), *Format(Finalists[WinIdx].Drive)));

	// ---- Where the actuated ceiling actually is ----
	//
	// The first pass at this sweep found that NO (Hz, zeta) pair survived sustained
	// full-amplitude babble, which means the binding constraint is not the contact
	// constants at all. This ladder locates it: amplitude is raised at the WINNING
	// constants, and each rung is repeated with the authored muscle strengths pinned
	// to 1. If the neutralised column survives where the authored one does not, the
	// limit is X-08 (0.5x-5x delivered torque) and not contact stiffness.
	AddInfo(TEXT(""));
	AddInfo(TEXT("=== ACTUATION CEILING at the winning constants (12 s each) ==="));
	AddInfo(TEXT(" amplitude | strengths   |    maxPen |  restPen |    torsoZ |    maxSpeed |   ms/sub | diverged"));
	const TArray<float> Amplitudes{ 0.3f, 0.6f, 1.0f };
	for (float Amp : Amplitudes)
	{
		for (int32 Neutral = 0; Neutral < 2; ++Neutral)
		{
			const FResult R = Run(SkeletalMesh, MassAsset, MuscleAsset, Best.Hz, Best.Zeta,
				Amp, Neutral != 0, DriveSubsteps);
			AddInfo(FString::Printf(TEXT("      %4.0f%% | %-11s | %s"),
				Amp * 100.0f, Neutral ? TEXT("pinned to 1") : TEXT("authored"), *Format(R)));
		}
	}

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS && WITH_EDITOR
