// TEMPORARY diagnostic for the 2026-08-17 solver upgrade: does the REAL 41-body
// rig still survive a sustained passive ground contact with every new mechanism
// enabled, and how does it compare to the configuration entry 025 measured?
//
// The permanent AgentSolver.SolverUpgrade test proves each new mechanism is
// individually correct on synthetic rigs. It cannot answer the only question
// that decides whether this work was worth doing: entry 025 is the FIRST
// configuration in this project's history to survive sustained contact (50 s,
// torso drifting 0.01 cm over the last 40), and a regression there would matter
// more than any of the individual wins.
//
// So this runs the real rig four ways over the same passive drop and prints the
// numbers side by side:
//   BASELINE  - entry 025's configuration: per-row solve, no armature, no
//               damping, no welding, no limb collision, no Cfm/SOR.
//   +GLOBAL   - baseline but with the assembled-constraint solve.
//   +PASSIVE  - +GLOBAL plus armature and joint damping.
//   ALL       - the shipped defaults: everything above plus welding, limb
//               collision and the regularizers.
//
// Delete once the numbers are recorded in SOLVER_DEBUG_LOG.md, exactly as entry
// 021 deleted four penalty-only diagnostics whose questions were answered.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "CreatureBatchSolver.h"
#include "CreatureGroundContact.h"
#include "CreatureRLEnvironment.h"
#include "MutoRLTrainingDriver.h"

#if WITH_EDITOR
#include "MutoTopology.h"
#include "Engine/SkeletalMesh.h"
#include "UMassMuscleProfileAsset.h"
#endif

#if WITH_DEV_AUTOMATION_TESTS && WITH_EDITOR

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FMutoRigUpgradeCheck,
	"AgentSolver.TEMP.RigUpgradeCheck",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FMutoRigUpgradeCheck::RunTest(const FString& Parameters)
{
	UMassMuscleProfileAssetMuscle* MuscleAsset = LoadObject<UMassMuscleProfileAssetMuscle>(nullptr, TEXT("/Game/Mesh/MuscleProfile_Muto.MuscleProfile_Muto"));
	UMassMuscleProfileAssetMass* MassAsset = LoadObject<UMassMuscleProfileAssetMass>(nullptr, TEXT("/Game/Mesh/MassProfile_Muto.MassProfile_Muto"));
	USkeletalMesh* SkeletalMesh = LoadObject<USkeletalMesh>(nullptr, TEXT("/Game/Mesh/Muto.Muto"));
	if (!TestNotNull(TEXT("MuscleProfile_Muto loaded"), MuscleAsset)) return false;
	if (!TestNotNull(TEXT("MassProfile_Muto loaded"), MassAsset)) return false;
	if (!TestNotNull(TEXT("Muto skeletal mesh loaded"), SkeletalMesh)) return false;

	constexpr float Dt = 1.0f / 240.0f;
	const FVector Gravity(0.0f, 0.0f, -980.0f);
	int32 NumSubsteps = 2400; // 10 s for the comparison table

	struct FConfig
	{
		const TCHAR* Name;
		bool bGlobal;
		float ArmatureRatio;
		float DampingT;
		bool bWeld;
		bool bLimbCollision;
		float Cfm;
	};
	// ONE CHANGE AT A TIME, which is this project's own standing ground rule and
	// which the first version of this table broke: it went from "+PASSIVE
	// diverges" straight to "ALL survives" while turning on three things at once
	// (welding, limb collision, Cfm), so it could not say which of them mattered.
	//
	// The per-row rows are the control that matters most. If the per-row solve
	// with the same additions also survives, then the global solve is not what
	// fixed this and should not be credited for it.
	const FConfig Configs[] = {
		//                     global  armature  dampT   weld   limbCol  cfm
		{ TEXT("BASELINE    "), false,  0.0f,    0.0f,   false, false,   0.0f    },
		{ TEXT("row+passive "), false,  0.05f,   1.0f,   false, false,   0.0f    },
		{ TEXT("row+weld    "), false,  0.0f,    0.0f,   true,  false,   0.0f    },
		{ TEXT("row+limbcol "), false,  0.0f,    0.0f,   false, true,    0.0f    },
		{ TEXT("global only "), true,   0.0f,    0.0f,   false, false,   0.0f    },
		{ TEXT("glob+passive"), true,   0.05f,   1.0f,   false, false,   0.0f    },
		{ TEXT("glob+pas+wel"), true,   0.05f,   1.0f,   true,  false,   0.0f    },
		{ TEXT("glob+pas+lim"), true,   0.05f,   1.0f,   false, true,    0.0f    },
		{ TEXT("ALL row     "), false,  0.05f,   1.0f,   true,  true,    1.0e-8f },
		{ TEXT("ALL         "), true,   0.05f,   1.0f,   true,  true,    1.0e-8f },
	};

	AddInfo(TEXT("Real Muto rig, passive drop (zero torque), 10 s at 240 Hz."));
	AddInfo(TEXT("NOTE: BASELINE is entry 025's configuration, but on the CURRENT 41-body rig."));
	AddInfo(TEXT("Entry 025's '50 s stable' was measured on the 35-body rig, before the"));
	AddInfo(TEXT("2026-08-16 spine articulation -- so it is not a baseline for this rig."));
	AddInfo(TEXT(""));
	AddInfo(TEXT("  config       | maxPen    | pen@end | torsoZ end | maxJointSpeed | ms/substep | diverged"));

	for (const FConfig& Cfg : Configs)
	{
		FCreatureTopology Topo;
		TArray<FString> Warnings;
		TArray<FName> BodyDebugNames;
		if (!MutoTopology::BuildMutoTopology(SkeletalMesh, MassAsset, MuscleAsset, Topo, Warnings, &BodyDebugNames))
		{
			AddError(TEXT("BuildMutoTopology failed"));
			return false;
		}
		for (int32 DOF = 0; DOF < Topo.NumDOF; ++DOF)
		{
			Topo.DOFArmatureRatio[DOF] = Cfg.ArmatureRatio;
			Topo.DOFDampingTimeConstant[DOF] = Cfg.DampingT;
		}

		// Ground collision on every body — entry 025's change, and the reason the
		// baseline is stable at all. Held constant across configs.
		const TArray<CreatureGroundContact::FContactPointDef> Points =
			CreatureGroundContact::BuildMutoContactPoints(Topo, MassAsset, BodyDebugNames, /*bAllBodies=*/true, /*Fallback=*/10.0f);
		const TArray<CreatureGroundContact::FLimbPairDef> Pairs =
			CreatureGroundContact::BuildMutoLimbCollisionPairs(Topo);

		CreatureGroundContact::FImpulseContactParams Params;
		Params.GroundZ = 0.0f;
		Params.ContactHertz = 15.0f;
		Params.DampingRatio = 10.0f;
		Params.Slop = 0.5f;
		Params.FrictionCoefficient = 0.8f;
		Params.Iterations = 16;
		Params.RelaxIterations = 0;
		Params.bUseGlobalSolve = Cfg.bGlobal;
		Params.GlobalIterations = 64;
		Params.Cfm = Cfg.Cfm;

		CreatureGroundContact::FJointLimitParams Limits;
		Limits.bEnabled = true;
		Limits.Hertz = 60.0f;
		Limits.DampingRatio = 5.0f;
		Limits.SlopDeg = 0.25f;
		Limits.MarginDeg = 3.0f;
		Limits.Cfm = Cfg.Cfm;

		CreatureGroundContact::FLimbCollisionParams LimbCol;
		LimbCol.bEnabled = Cfg.bLimbCollision;
		LimbCol.Hertz = 30.0f;
		LimbCol.DampingRatio = 10.0f;
		LimbCol.Slop = 0.5f;
		LimbCol.Cfm = Cfg.Cfm;

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

		float MaxPen = 0.0f;
		float MaxJointSpeed = 0.0f;
		int32 DivergedAt = INDEX_NONE;
		float FinalPen = 0.0f;
		int32 SubstepsRun = 0;
		// Wall-clock cost per substep for ONE env. Entry 025 closed with "cost went
		// up ... not measured; it will matter for training throughput with many
		// envs", and it stayed unmeasured. This is a single-env number on a debug
		// editor build, so it is an upper bound and a RELATIVE comparison between
		// configurations, not a throughput figure.
		const double StartTime = FPlatformTime::Seconds();

		for (int32 Substep = 1; Substep <= NumSubsteps; ++Substep)
		{
			SubstepsRun = Substep;
			Batch.ClearExternalForces(0);
			Solver.Step(Batch, Dt, Gravity);
			Solver.ApplyJointDamping(Batch, Dt);

			const uint8* LockPtr = nullptr;
			int32 LockStride = 0;
			if (Cfg.bWeld)
			{
				Solver.BuildSaturatedJointLocks(Batch, 1.0f, Locks);
				LockPtr = Locks.GetData();
				LockStride = 1;
			}

			CreatureGroundContact::ResolveGroundContactImpulses(
				Batch, Topo, Points, Params, Solver, Dt, Cache, &States, &Limits,
				Pairs, &LimbCol, nullptr, LockPtr, LockStride);

			if (!CreatureRLEnvironment::IsBodyStateValid(Batch, 0))
			{
				DivergedAt = Substep;
				break;
			}

			// Deepest penetration of any contact surface this substep.
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
			MaxPen = FMath::Max(MaxPen, Pen);
			FinalPen = Pen;

			for (int32 DOF = 0; DOF < Topo.NumDOF; ++DOF)
			{
				MaxJointSpeed = FMath::Max(MaxJointSpeed, FMath::Abs(Batch.JointVel[Batch.DOFIndex(DOF, 0)]));
			}
		}

		const double MsPerSubstep = 1000.0 * (FPlatformTime::Seconds() - StartTime) / FMath::Max(1, SubstepsRun);
		const float TorsoZ = (DivergedAt == INDEX_NONE) ? (float)Batch.GetBodyPos(0, 0).Z : 0.0f;
		AddInfo(FString::Printf(TEXT("  %s | %9.2f | %7.2f | %10.2f | %13.1f | %10.3f | %s"),
			Cfg.Name, MaxPen, FinalPen, TorsoZ, MaxJointSpeed, MsPerSubstep,
			DivergedAt == INDEX_NONE ? TEXT("never") : *FString::Printf(TEXT("substep %d (%.2f s)"), DivergedAt, DivergedAt * Dt)));

		// Only the SHIPPED default is asserted. The rest are controls, and several
		// are expected to diverge -- that is the information the table exists to
		// carry, and turning it into a test failure would just make the table
		// unrunnable.
		if (FString(Cfg.Name).TrimStartAndEnd() == TEXT("ALL"))
		{
			TestTrue(TEXT("shipped defaults: the rig survives 10 s of sustained contact"),
				DivergedAt == INDEX_NONE);
			TestTrue(TEXT("shipped defaults: penetration stays physically plausible"), MaxPen < 60.0f);
			TestTrue(TEXT("shipped defaults: no joint reaches an absurd speed"), MaxJointSpeed < 500.0f);
		}
	}

	// =====================================================================
	// 50-SECOND CONFIRMATION, shipped defaults only
	// =====================================================================
	// Because a window chosen before knowing what the answer looks like has
	// flattered a result in this project at least three times (entries 011, 023,
	// 024 -- entry 024 nearly reported "32 iterations fixes it" off a 400-substep
	// trace). "Survived the trace" is a claim about the trace until the trace is
	// boring, so this runs the same length entry 025 used and samples along the
	// way to show whether it is actually settled or still creeping.
	{
		AddInfo(TEXT(""));
		AddInfo(TEXT("50 s confirmation run, shipped defaults:"));

		FCreatureTopology Topo;
		TArray<FString> Warnings;
		TArray<FName> BodyDebugNames;
		MutoTopology::BuildMutoTopology(SkeletalMesh, MassAsset, MuscleAsset, Topo, Warnings, &BodyDebugNames);
		for (int32 DOF = 0; DOF < Topo.NumDOF; ++DOF)
		{
			Topo.DOFArmatureRatio[DOF] = 0.05f;
			Topo.DOFDampingTimeConstant[DOF] = 1.0f;
		}

		const TArray<CreatureGroundContact::FContactPointDef> Points =
			CreatureGroundContact::BuildMutoContactPoints(Topo, MassAsset, BodyDebugNames, true, 10.0f);
		const TArray<CreatureGroundContact::FLimbPairDef> Pairs =
			CreatureGroundContact::BuildMutoLimbCollisionPairs(Topo);

		CreatureGroundContact::FImpulseContactParams Params;
		Params.GroundZ = 0.0f;
		Params.ContactHertz = 15.0f;
		Params.DampingRatio = 10.0f;
		Params.Slop = 0.5f;
		Params.Iterations = 16;
		Params.RelaxIterations = 0;
		Params.bUseGlobalSolve = true;
		Params.GlobalIterations = 64;
		Params.Cfm = 1.0e-8f;

		CreatureGroundContact::FJointLimitParams Limits;
		Limits.bEnabled = true;
		Limits.Hertz = 60.0f;
		Limits.DampingRatio = 5.0f;
		Limits.Cfm = 1.0e-8f;

		CreatureGroundContact::FLimbCollisionParams LimbCol;
		LimbCol.bEnabled = true;
		LimbCol.Hertz = 30.0f;
		LimbCol.DampingRatio = 10.0f;
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

		constexpr int32 LongSubsteps = 12000; // 50 s
		float MaxPen = 0.0f;
		float MaxSpeed = 0.0f;
		int32 DivergedAt = INDEX_NONE;

		for (int32 Substep = 1; Substep <= LongSubsteps; ++Substep)
		{
			Batch.ClearExternalForces(0);
			Solver.Step(Batch, Dt, Gravity);
			Solver.ApplyJointDamping(Batch, Dt);
			Solver.BuildSaturatedJointLocks(Batch, 1.0f, Locks);
			CreatureGroundContact::ResolveGroundContactImpulses(
				Batch, Topo, Points, Params, Solver, Dt, Cache, &States, &Limits,
				Pairs, &LimbCol, nullptr, Locks.GetData(), 1);

			if (!CreatureRLEnvironment::IsBodyStateValid(Batch, 0)) { DivergedAt = Substep; break; }

			for (const CreatureGroundContact::FContactPointDef& P : Points)
			{
				FVector Surfaces[2];
				const int32 NumEnds = CreatureGroundContact::GetContactPointWorldSurfaces(Batch, P, 0, FVector::UpVector, Surfaces);
				for (int32 e = 0; e < NumEnds; ++e) { MaxPen = FMath::Max(MaxPen, -(float)Surfaces[e].Z); }
			}
			for (int32 b = 0; b < Topo.NumBodies; ++b)
			{
				const int32 Idx = Batch.BodyIndex(b, 0);
				MaxSpeed = FMath::Max(MaxSpeed, (float)FVector(Batch.LinVelX[Idx], Batch.LinVelY[Idx], Batch.LinVelZ[Idx]).Size());
			}

			if (Substep % 2400 == 0)
			{
				AddInfo(FString::Printf(TEXT("  t=%5.1f s  torsoZ=%8.2f  maxPen=%7.2f  maxSpeed=%9.1f"),
					Substep * Dt, Batch.GetBodyPos(0, 0).Z, MaxPen, MaxSpeed));
			}
		}

		TestTrue(TEXT("shipped defaults: survives 50 s of sustained contact"), DivergedAt == INDEX_NONE);
		if (DivergedAt != INDEX_NONE)
		{
			AddError(FString::Printf(TEXT("diverged at substep %d (%.2f s)"), DivergedAt, DivergedAt * Dt));
		}
	}

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS && WITH_EDITOR
