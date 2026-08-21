// TEMPORARY DIAGNOSTIC — see SOLVER_DEBUG_LOG.md entry 004.
//
// Suspect 4: SoA/SIMD padding contamination and scalar/batched parity.
//
// Two independent questions, answered separately because they have different
// failure modes:
//
//   PARITY  — StepScalar vs StepSIMD from an identical initial state. These
//             implement the same recursion twice; any structural divergence is
//             a bug in one of them. Roundoff-level drift (~1e-6 relative,
//             growing slowly) is expected and fine, because the two paths
//             legitimately differ in operation ORDER. A sudden jump, or drift
//             that grows geometrically, is not.
//
//   PADDING — NumEnvs=5 pads to 8, leaving lanes 5-7 unused. Run the real envs
//             twice: once with padding lanes as Init() leaves them, once with
//             them deliberately poisoned (NaN / huge values). If envs 0-4 differ
//             AT ALL between the two runs, padding is contaminating real lanes.
//             This is an exact-equality check — there is no legitimate reason
//             for a padding lane to change a real lane by even one ULP.
//
// Runs are kept SHORT (1.0 s). Entry 003 established the rig is well behaved
// to t~1.5s and explodes after, so a longer run would measure chaos rather
// than parity.
//
// Run with:
//   UnrealEditor-Cmd.exe <uproject> -ExecCmds="Automation RunTests AgentSolver.TEMP.ScalarSIMDParity; Quit"
//     -unattended -nopause -nosplash -nullrhi -log=parity.log

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "PhysicsSolver/CreatureBatchState.h"
#include "PhysicsSolver/CreatureBatchSolver.h"
#include "PhysicsSolver/CreatureGroundContact.h"
#include "AgentSolver/MutoRLTrainingDriver.h"
#include "AgentSolver/CreatureRLEnvironment.h"

#if WITH_EDITOR
#include "PhysicsSolver/MutoTopology.h"
#include "Engine/SkeletalMesh.h"
#endif

#if WITH_DEV_AUTOMATION_TESTS && WITH_EDITOR

using namespace CreatureGroundContact;

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FMutoScalarSIMDParity,
	"AgentSolver.TEMP.ScalarSIMDParity",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

namespace
{
	struct FNamedArray { const TCHAR* Name; const TArray<float>* Arr; };

	TArray<FNamedArray> StateArrays(const FCreatureBatchState& B)
	{
		return {
			{ TEXT("PosX"), &B.PosX }, { TEXT("PosY"), &B.PosY }, { TEXT("PosZ"), &B.PosZ },
			{ TEXT("RotX"), &B.RotX }, { TEXT("RotY"), &B.RotY }, { TEXT("RotZ"), &B.RotZ }, { TEXT("RotW"), &B.RotW },
			{ TEXT("LinVelX"), &B.LinVelX }, { TEXT("LinVelY"), &B.LinVelY }, { TEXT("LinVelZ"), &B.LinVelZ },
			{ TEXT("AngVelX"), &B.AngVelX }, { TEXT("AngVelY"), &B.AngVelY }, { TEXT("AngVelZ"), &B.AngVelZ },
			{ TEXT("JointPos"), &B.JointPos }, { TEXT("JointVel"), &B.JointVel },
			{ TEXT("JointRelRotX"), &B.JointRelRotX }, { TEXT("JointRelRotY"), &B.JointRelRotY },
			{ TEXT("JointRelRotZ"), &B.JointRelRotZ }, { TEXT("JointRelRotW"), &B.JointRelRotW },
		};
	}

	/** Worst RELATIVE difference over the given envs only (padding lanes skipped). */
	struct FDiff { double Worst = 0.0; FString Where; };

	FDiff CompareEnvs(const FCreatureBatchState& A, const FCreatureBatchState& B,
	                  int32 NumRealEnvs, int32 Padded, int32 NumBodies, int32 NumDOF)
	{
		FDiff Out;
		const TArray<FNamedArray> AA = StateArrays(A);
		const TArray<FNamedArray> BB = StateArrays(B);

		for (int32 k = 0; k < AA.Num(); ++k)
		{
			const TArray<float>& X = *AA[k].Arr;
			const TArray<float>& Y = *BB[k].Arr;
			if (X.Num() != Y.Num()) continue;

			// Slot count differs between body-indexed and DOF-indexed arrays.
			const int32 Slots = X.Num() / Padded;
			for (int32 Slot = 0; Slot < Slots; ++Slot)
			{
				for (int32 Env = 0; Env < NumRealEnvs; ++Env)
				{
					const int32 i = Slot * Padded + Env;
					const double a = X[i], b = Y[i];
					if (!FMath::IsFinite(a) || !FMath::IsFinite(b))
					{
						Out.Worst = TNumericLimits<double>::Max();
						Out.Where = FString::Printf(TEXT("%s[slot %d, env %d] NON-FINITE (%g vs %g)"), AA[k].Name, Slot, Env, a, b);
						return Out;
					}
					const double Denom = FMath::Max(1.0, FMath::Max(FMath::Abs(a), FMath::Abs(b)));
					const double Rel = FMath::Abs(a - b) / Denom;
					if (Rel > Out.Worst)
					{
						Out.Worst = Rel;
						Out.Where = FString::Printf(TEXT("%s[slot %d, env %d] %.9g vs %.9g"), AA[k].Name, Slot, Env, a, b);
					}
				}
			}
		}
		return Out;
	}

	/** Exact bitwise-equality check over real envs — used for the padding test. */
	bool ExactlyEqualOverEnvs(const FCreatureBatchState& A, const FCreatureBatchState& B,
	                          int32 NumRealEnvs, int32 Padded, FString& OutWhere)
	{
		const TArray<FNamedArray> AA = StateArrays(A);
		const TArray<FNamedArray> BB = StateArrays(B);
		for (int32 k = 0; k < AA.Num(); ++k)
		{
			const TArray<float>& X = *AA[k].Arr;
			const TArray<float>& Y = *BB[k].Arr;
			if (X.Num() != Y.Num()) continue;
			const int32 Slots = X.Num() / Padded;
			for (int32 Slot = 0; Slot < Slots; ++Slot)
			{
				for (int32 Env = 0; Env < NumRealEnvs; ++Env)
				{
					const int32 i = Slot * Padded + Env;
					if (X[i] != Y[i])
					{
						OutWhere = FString::Printf(TEXT("%s[slot %d, env %d]: %.9g vs %.9g"),
							AA[k].Name, Slot, Env, X[i], Y[i]);
						return false;
					}
				}
			}
		}
		return true;
	}
}

bool FMutoScalarSIMDParity::RunTest(const FString& Parameters)
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

	const TArray<FContactPointDef> Points = BuildMutoContactPoints(Topo, MassAsset, BodyDebugNames);
	const FQuat StandingRot = Topo.BodyRestRotInParent[0];
	const float StandHeight = AMutoRLTrainingDriver::ComputeDefaultStandingHeight(Topo, Points, StandingRot);
	const FVector StandPos(0.0f, 0.0f, StandHeight);
	const FVector Gravity(0.0f, 0.0f, -980.0f);

	constexpr float Dt = 1.0f / 60.0f;
	constexpr int32 NumSteps = 60; // 1.0 s — inside the well-behaved window

	// Small reset noise so the envs are NOT identical to each other; an
	// all-identical batch could mask a lane-indexing bug by making every lane
	// carry the same value.
	auto InitBatch = [&](FCreatureBatchState& B, int32 NumEnvs, const FCreatureTopology& T)
	{
		B.Init(T, NumEnvs);
		FRandomStream S(4242);
		for (int32 Env = 0; Env < NumEnvs; ++Env)
		{
			CreatureRLEnvironment::ResetEnv(B, Env, StandPos, StandingRot, S, 5.0f, 0.05f);
		}
	};

	// ================= PART 1: scalar vs SIMD parity =================
	AddInfo(TEXT("================ PART 1: StepScalar vs StepSIMD ================"));
	AddInfo(TEXT("Identical initial state, 8 envs (no padding), 1.0 s at 60 Hz, passive."));
	{
		constexpr int32 NumEnvs = 8;
		FCreatureBatchState ScalarBatch, SimdBatch;
		InitBatch(ScalarBatch, NumEnvs, Topo);
		InitBatch(SimdBatch, NumEnvs, Topo);

		FCreatureABASolver ScalarSolver, SimdSolver;
		ScalarSolver.StepScalar(ScalarBatch, 0.0f, Gravity);
		SimdSolver.StepSIMD(SimdBatch, 0.0f, Gravity);

		for (int32 DOF = 0; DOF < Topo.NumDOF; ++DOF)
			for (int32 Env = 0; Env < NumEnvs; ++Env)
			{
				ScalarBatch.JointTorque[ScalarBatch.DOFIndex(DOF, Env)] = 0.0f;
				SimdBatch.JointTorque[SimdBatch.DOFIndex(DOF, Env)] = 0.0f;
			}

		FString FirstBigWhere;
		int32 FirstBigStep = INDEX_NONE;
		for (int32 Step = 0; Step < NumSteps; ++Step)
		{
			ScalarSolver.StepScalar(ScalarBatch, Dt, Gravity);
			SimdSolver.StepSIMD(SimdBatch, Dt, Gravity);

			const FDiff D = CompareEnvs(ScalarBatch, SimdBatch, NumEnvs,
				ScalarBatch.GetPaddedNumEnvs(), Topo.NumBodies, Topo.NumDOF);

			if (FirstBigStep == INDEX_NONE && D.Worst > 1e-3)
			{
				FirstBigStep = Step;
				FirstBigWhere = D.Where;
			}
			if (Step % 10 == 0 || Step == NumSteps - 1)
			{
				AddInfo(FString::Printf(TEXT("  step %3d (t=%.3fs)  worst relative diff = %.3e   @ %s"),
					Step, Step * Dt, D.Worst, *D.Where));
			}
		}

		if (FirstBigStep != INDEX_NONE)
		{
			AddInfo(FString::Printf(TEXT("  VERDICT: paths DIVERGE — exceeded 1e-3 relative at step %d (%s)"),
				FirstBigStep, *FirstBigWhere));
		}
		else
		{
			AddInfo(TEXT("  VERDICT: scalar and SIMD agree to within 1e-3 relative for the whole run."));
		}
	}

	// ================= PART 2: padding contamination =================
	AddInfo(TEXT(""));
	AddInfo(TEXT("================ PART 2: padding-lane contamination ================"));
	AddInfo(TEXT("NumEnvs=5 -> pads to 8, so lanes 5,6,7 are unused. Real envs must be"));
	AddInfo(TEXT("BITWISE identical whether padding is clean or poisoned."));
	{
		constexpr int32 NumEnvs = 5;

		FCreatureBatchState Clean, Poisoned;
		InitBatch(Clean, NumEnvs, Topo);
		InitBatch(Poisoned, NumEnvs, Topo);

		// Poison every padding lane (env index >= NumEnvs) of every state array.
		const int32 Padded = Poisoned.GetPaddedNumEnvs();
		AddInfo(FString::Printf(TEXT("  NumEnvs=%d PaddedNumEnvs=%d -> poisoning lanes %d..%d"),
			NumEnvs, Padded, NumEnvs, Padded - 1));

		auto Poison = [&](TArray<float>& Arr)
		{
			const int32 Slots = Arr.Num() / Padded;
			for (int32 Slot = 0; Slot < Slots; ++Slot)
				for (int32 Env = NumEnvs; Env < Padded; ++Env)
					Arr[Slot * Padded + Env] = (Env % 2 == 0)
						? std::numeric_limits<float>::quiet_NaN()
						: 1.0e30f;
		};
		Poison(Poisoned.PosX); Poison(Poisoned.PosY); Poison(Poisoned.PosZ);
		Poison(Poisoned.RotX); Poison(Poisoned.RotY); Poison(Poisoned.RotZ); Poison(Poisoned.RotW);
		Poison(Poisoned.LinVelX); Poison(Poisoned.LinVelY); Poison(Poisoned.LinVelZ);
		Poison(Poisoned.AngVelX); Poison(Poisoned.AngVelY); Poison(Poisoned.AngVelZ);
		Poison(Poisoned.JointPos); Poison(Poisoned.JointVel); Poison(Poisoned.JointTorque);
		Poison(Poisoned.JointRelRotX); Poison(Poisoned.JointRelRotY);
		Poison(Poisoned.JointRelRotZ); Poison(Poisoned.JointRelRotW);
		Poison(Poisoned.LimbStrengthScale);

		FCreatureABASolver S1, S2;
		S1.StepSIMD(Clean, 0.0f, Gravity);
		S2.StepSIMD(Poisoned, 0.0f, Gravity);

		for (int32 DOF = 0; DOF < Topo.NumDOF; ++DOF)
			for (int32 Env = 0; Env < NumEnvs; ++Env)
			{
				Clean.JointTorque[Clean.DOFIndex(DOF, Env)] = 0.0f;
				Poisoned.JointTorque[Poisoned.DOFIndex(DOF, Env)] = 0.0f;
			}

		int32 FirstMismatch = INDEX_NONE;
		FString Where;
		for (int32 Step = 0; Step < NumSteps; ++Step)
		{
			S1.StepSIMD(Clean, Dt, Gravity);
			S2.StepSIMD(Poisoned, Dt, Gravity);

			FString W;
			if (!ExactlyEqualOverEnvs(Clean, Poisoned, NumEnvs, Padded, W))
			{
				FirstMismatch = Step;
				Where = W;
				break;
			}
		}

		if (FirstMismatch != INDEX_NONE)
		{
			AddInfo(FString::Printf(TEXT("  VERDICT: CONTAMINATION — real envs diverged at step %d: %s"),
				FirstMismatch, *Where));
		}
		else
		{
			AddInfo(FString::Printf(TEXT("  VERDICT: clean — real envs bitwise identical for all %d steps"), NumSteps));
			AddInfo(TEXT("           despite NaN/1e30 in every padding lane."));
		}
	}

	// ================= PART 3: parity in a BOUNDED regime =================
	// Part 1 showed the two paths drifting apart, but Entry 003 established the
	// gravity-driven system is genuinely divergent — so ANY difference between
	// two implementations, including pure float32 roundoff, is amplified
	// exponentially. That makes Part 1 unable to distinguish "SIMD bug" from
	// "chaos amplifying roundoff".
	//
	// This repeats the comparison with gravity OFF and a small joint kick. Per
	// Entry 003 (ZG rows) that regime stays bounded over 5 s. If the two paths
	// track each other at roundoff level HERE, the implementations agree
	// structurally and Part 1's divergence is chaos, not a second bug. If they
	// separate here too, there is a genuine SIMD/scalar discrepancy.
	AddInfo(TEXT(""));
	AddInfo(TEXT("================ PART 3: parity, BOUNDED regime (zero gravity) ================"));
	AddInfo(TEXT("Gravity OFF + 0.5 rad/s joint kick — bounded per Entry 003, so roundoff is"));
	AddInfo(TEXT("not exponentially amplified and a real implementation gap would stand out."));
	{
		constexpr int32 NumEnvs = 8;
		const FVector NoGravity = FVector::ZeroVector;

		FCreatureBatchState ScalarBatch, SimdBatch;
		InitBatch(ScalarBatch, NumEnvs, Topo);
		InitBatch(SimdBatch, NumEnvs, Topo);

		FCreatureABASolver ScalarSolver, SimdSolver;
		ScalarSolver.StepScalar(ScalarBatch, 0.0f, NoGravity);
		SimdSolver.StepSIMD(SimdBatch, 0.0f, NoGravity);

		for (int32 DOF = 0; DOF < Topo.NumDOF; ++DOF)
			for (int32 Env = 0; Env < NumEnvs; ++Env)
			{
				ScalarBatch.JointTorque[ScalarBatch.DOFIndex(DOF, Env)] = 0.0f;
				SimdBatch.JointTorque[SimdBatch.DOFIndex(DOF, Env)] = 0.0f;
				ScalarBatch.JointVel[ScalarBatch.DOFIndex(DOF, Env)] = 0.5f;
				SimdBatch.JointVel[SimdBatch.DOFIndex(DOF, Env)] = 0.5f;
			}

		double FirstDiff = -1.0, LastDiff = 0.0;
		for (int32 Step = 0; Step < NumSteps; ++Step)
		{
			ScalarSolver.StepScalar(ScalarBatch, Dt, NoGravity);
			SimdSolver.StepSIMD(SimdBatch, Dt, NoGravity);

			const FDiff D = CompareEnvs(ScalarBatch, SimdBatch, NumEnvs,
				ScalarBatch.GetPaddedNumEnvs(), Topo.NumBodies, Topo.NumDOF);
			if (FirstDiff < 0.0) FirstDiff = D.Worst;
			LastDiff = D.Worst;

			if (Step % 10 == 0 || Step == NumSteps - 1)
			{
				AddInfo(FString::Printf(TEXT("  step %3d (t=%.3fs)  worst relative diff = %.3e   @ %s"),
					Step, Step * Dt, D.Worst, *D.Where));
			}
		}

		const double Growth = (FirstDiff > 0.0) ? (LastDiff / FirstDiff) : 0.0;
		AddInfo(FString::Printf(TEXT("  growth over %d steps: %.3e -> %.3e  (%.1fx)"),
			NumSteps, FirstDiff, LastDiff, Growth));
		AddInfo(TEXT("  Compare against Part 1's growth. Similar growth in BOTH regimes => a real"));
		AddInfo(TEXT("  implementation gap. Growth ONLY in Part 1 => chaos amplifying roundoff."));
	}

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS && WITH_EDITOR
