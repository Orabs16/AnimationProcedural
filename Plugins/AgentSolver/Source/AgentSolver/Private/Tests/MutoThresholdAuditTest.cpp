// TEMPORARY DIAGNOSTIC — see SOLVER_DEBUG_LOG.md entry 020.
//
// Entry 019 predicted that a dt-sensitive or scale-sensitive ABSOLUTE epsilon
// is behind the remaining contact failure, on the grounds that performance got
// WORSE as the timestep shrank — the same signature that caught the rotation
// threshold in entry 012. A convergent method cannot degrade under refinement.
//
// This measures the quantities those epsilons are compared against, during a
// real simulation, rather than reasoning about them.
//
// The two suspects from the static audit, both in CreatureGroundContact.h:
//     if (A.InvMassN <= KINDA_SMALL_NUMBER) continue;      // skips the contact
//     FMath::Max(A.InvMassT1, KINDA_SMALL_NUMBER)          // caps m_eff
// InvMass is 1/m_eff, so KINDA_SMALL_NUMBER (1e-4) means any contact whose
// ARTICULATED effective mass exceeds 10,000 kg is silently dropped or clamped.
// The creature masses 6170 kg, so bracing contacts can plausibly exceed that —
// this reports whether they actually do.
//
// Also reports the conditioning of the articulated inertias (determinants and
// diagonal magnitudes) against the absolute thresholds in SpatialAlgebra.h:
//     Inverse3x3:     |Det| > KINDA_SMALL_NUMBER
//     SolveSpatial6:  |pivot| < KINDA_SMALL_NUMBER -> skip
// These are compared against quantities that scale as mass*length^2, which for
// this rig is ~1e6-1e8, so they should be enormously safe. Confirmed, not assumed.
//
// Run with:
//   UnrealEditor-Cmd.exe <uproject> -ExecCmds="Automation RunTests AgentSolver.TEMP.ThresholdAudit; Quit"
//     -unattended -nopause -nosplash -nullrhi -log=thresh.log

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "CreatureBatchState.h"
#include "CreatureBatchSolver.h"
#include "CreatureGroundContact.h"
#include "CreatureRLEnvironment.h"
#include "MutoRLTrainingDriver.h"

#if WITH_EDITOR
#include "MutoTopology.h"
#include "Engine/SkeletalMesh.h"
#endif

#if WITH_DEV_AUTOMATION_TESTS && WITH_EDITOR

using namespace CreatureGroundContact;

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FMutoThresholdAudit,
	"AgentSolver.TEMP.ThresholdAudit",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FMutoThresholdAudit::RunTest(const FString& Parameters)
{
	UMassMuscleProfileAssetMuscle* MuscleAsset = LoadObject<UMassMuscleProfileAssetMuscle>(nullptr, TEXT("/Game/Mesh/MuscleProfile_Muto.MuscleProfile_Muto"));
	UMassMuscleProfileAssetMass* MassAsset = LoadObject<UMassMuscleProfileAssetMass>(nullptr, TEXT("/Game/Mesh/MassProfile_Muto.MassProfile_Muto"));
	USkeletalMesh* SkeletalMesh = LoadObject<USkeletalMesh>(nullptr, TEXT("/Game/Mesh/Muto.Muto"));
	if (!TestNotNull(TEXT("MuscleProfile_Muto loaded"), MuscleAsset)) return false;
	if (!TestNotNull(TEXT("MassProfile_Muto loaded"), MassAsset)) return false;
	if (!TestNotNull(TEXT("Muto skeletal mesh loaded"), SkeletalMesh)) return false;

	FCreatureTopology Topo;
	TArray<FString> Warnings;
	TArray<FName> Names;
	if (!TestTrue(TEXT("BuildMutoTopology"), MutoTopology::BuildMutoTopology(SkeletalMesh, MassAsset, MuscleAsset, Topo, Warnings, &Names))) return false;

	const TArray<FContactPointDef> Points = BuildMutoContactPoints(Topo, MassAsset, Names);
	const FQuat StandRot = Topo.BodyRestRotInParent[0];
	const float StandH = AMutoRLTrainingDriver::ComputeDefaultStandingHeight(Topo, Points, StandRot);
	const FVector Gravity(0.0f, 0.0f, -980.0f);
	const FVector N = FVector::UpVector;

	float TotalMass = 0.0f;
	for (int32 b = 0; b < Topo.NumBodies; ++b) TotalMass += Topo.BodyMass[b];

	AddInfo(FString::Printf(TEXT("Rig: %d bodies, %.0f kg. KINDA_SMALL_NUMBER = %.1e"),
		Topo.NumBodies, TotalMass, KINDA_SMALL_NUMBER));
	AddInfo(FString::Printf(TEXT("A contact is SKIPPED when 1/m_eff <= %.1e, i.e. when m_eff >= %.0f kg."),
		KINDA_SMALL_NUMBER, 1.0f / KINDA_SMALL_NUMBER));
	AddInfo(TEXT(""));

	// ---- measure the actual inverse effective masses during a run ----
	for (const int32 Hz : { 240, 960, 1920 })
	{
		const float Dt = 1.0f / Hz;
		const int32 Steps = Hz * 2;

		FCreatureBatchState B;
		B.Init(Topo, 8);
		FCreatureABASolver Solver;
		FRandomStream S(1234);
		for (int32 Env = 0; Env < 8; ++Env)
			CreatureRLEnvironment::ResetEnv(B, Env, FVector(0, 0, StandH), StandRot, S, 0.0f, 0.0f);
		Solver.Step(B, 0.0f, Gravity);
		for (int32 D = 0; D < Topo.NumDOF; ++D)
			for (int32 Env = 0; Env < 8; ++Env)
				B.JointTorque[B.DOFIndex(D, Env)] = 0.0f;

		FImpulseContactParams Imp;
		Imp.GroundZ = 0.0f; Imp.ContactHertz = 15.0f; Imp.DampingRatio = 10.0f;
		Imp.Slop = 0.5f; Imp.FrictionCoefficient = 0.8f;
		Imp.Iterations = 8; Imp.RelaxIterations = 2;
		FImpulseContactCache Cache;

		float MinInvN = TNumericLimits<float>::Max();
		float MaxMEff = 0.0f;
		int32 NumSkipped = 0, NumSampled = 0, NumClampedT = 0;

		for (int32 Step = 0; Step < Steps; ++Step)
		{
			for (int32 Env = 0; Env < 8; ++Env) B.ClearExternalForces(Env);
			Solver.Step(B, Dt, Gravity);

			// Sample the same quantity the resolver tests, before resolving.
			Solver.ComputeArticulatedInertias(B);
			for (int32 Env = 0; Env < 8; ++Env)
			{
				for (const FContactPointDef& P : Points)
				{
					const FVector WorldPoint = B.GetBodyPos(P.BodyIndex, Env)
						+ B.GetBodyRot(P.BodyIndex, Env).RotateVector(P.LocalOffset) - P.Radius * N;
					if (Imp.GroundZ - static_cast<float>(WorldPoint.Z) <= 0.0f) continue;

					const float InvN = (float)FVector::DotProduct(
						Solver.ImpulseResponseAtPoint(B, Env, P.BodyIndex, WorldPoint, N), N);
					if (!FMath::IsFinite(InvN)) continue;
					++NumSampled;
					MinInvN = FMath::Min(MinInvN, InvN);
					if (InvN > 0.0f) MaxMEff = FMath::Max(MaxMEff, 1.0f / InvN);
					if (InvN <= KINDA_SMALL_NUMBER) ++NumSkipped;
					// Tangential clamp fires on the same scale.
					FVector T1, T2; BuildTangentBasis(N, T1, T2);
					const float InvT = (float)FVector::DotProduct(
						Solver.ImpulseResponseAtPoint(B, Env, P.BodyIndex, WorldPoint, T1), T1);
					if (FMath::IsFinite(InvT) && InvT <= KINDA_SMALL_NUMBER) ++NumClampedT;
				}
			}

			ResolveGroundContactImpulses(B, Topo, Points, Imp, Solver, Dt, Cache, nullptr);

			bool bBad = false;
			for (int32 i = 0; i < B.PosX.Num() && !bBad; ++i)
				if (!FMath::IsFinite(B.PosX[i])) bBad = true;
			if (bBad)
			{
				AddInfo(FString::Printf(TEXT("   @%4d Hz: diverged at t=%.3fs (audit stops here)"), Hz, Step * Dt));
				break;
			}
		}

		AddInfo(FString::Printf(
			TEXT("   @%4d Hz: sampled %d contacts | min(1/m_eff)=%.4g -> max m_eff=%.0f kg | SKIPPED=%d (%.2f%%) | T-clamped=%d"),
			Hz, NumSampled, (NumSampled > 0) ? MinInvN : 0.0f, MaxMEff,
			NumSkipped, (NumSampled > 0) ? 100.0f * NumSkipped / NumSampled : 0.0f, NumClampedT));
	}

	// ---- conditioning of the articulated inertias vs the SpatialAlgebra epsilons ----
	AddInfo(TEXT(""));
	AddInfo(TEXT("---- articulated inertia conditioning at the standing pose ----"));
	{
		FCreatureBatchState B;
		B.Init(Topo, 8);
		FCreatureABASolver Solver;
		FRandomStream S(1234);
		for (int32 Env = 0; Env < 8; ++Env)
			CreatureRLEnvironment::ResetEnv(B, Env, FVector(0, 0, StandH), StandRot, S, 0.0f, 0.0f);
		Solver.Step(B, 0.0f, Gravity);
		Solver.ComputeArticulatedInertias(B);

		AddInfo(TEXT("   (Inverse3x3 zeroes the inverse when |Det| <= 1e-4; SolveSpatial6 skips a"));
		AddInfo(TEXT("    column when |pivot| < 1e-4. Both compare an ABSOLUTE epsilon against"));
		AddInfo(TEXT("    quantities that scale as mass*length^2.)"));
		AddInfo(TEXT("   body  name             Irot[0][0]      |Det(Irot)|   MBlock[0][0]"));

		float MinDet = TNumericLimits<float>::Max();
		for (int32 b = 0; b < Topo.NumBodies; ++b)
		{
			const FSpatialInertia I = Solver.DebugArticulatedInertia(B, b, 0);
			const FMat3& R = I.Irot;
			const float Det =
				R.M[0][0] * (R.M[1][1] * R.M[2][2] - R.M[1][2] * R.M[2][1])
				- R.M[0][1] * (R.M[1][0] * R.M[2][2] - R.M[1][2] * R.M[2][0])
				+ R.M[0][2] * (R.M[1][0] * R.M[2][1] - R.M[1][1] * R.M[2][0]);
			MinDet = FMath::Min(MinDet, FMath::Abs(Det));
			if (b < 6 || b == Topo.NumBodies - 1)
			{
				AddInfo(FString::Printf(TEXT("   %4d  %-14s %13.4g %16.4g %14.4g"),
					b, *Names[b].ToString(), R.M[0][0], FMath::Abs(Det), I.MBlock.M[0][0]));
			}
		}
		AddInfo(FString::Printf(TEXT("   min |Det(Irot)| over all bodies = %.4g  (threshold 1e-4 -> margin %.3g x)"),
			MinDet, MinDet / KINDA_SMALL_NUMBER));
	}

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS && WITH_EDITOR
