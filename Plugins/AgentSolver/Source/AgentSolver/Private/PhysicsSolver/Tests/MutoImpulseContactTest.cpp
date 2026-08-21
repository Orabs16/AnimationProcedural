// TEMPORARY DIAGNOSTIC — see SOLVER_DEBUG_LOG.md entries 014, 018, 019, 021.
//
// Velocity-level impulse contact: correctness gate, then the two parameter
// sweeps that still have open questions attached.
//
// Rewritten 2026-08-14 when the penalty model was removed. Roughly half of the
// previous version was head-to-head comparison against it, plus a one-off
// diagnosis of the stale driver constants (recorded in entry 017, no longer
// needing to run every time). What remains is what still earns its runtime:
//
//   GATE      articulated effective mass vs closed form. Must pass exactly, or
//             nothing else here means anything.
//   SOFTNESS  ContactHertz / DampingRatio sweep on an actuated stand.
//   PD CHECK  the same actuation with contact DISABLED, to prove the harness
//             is not what is diverging — a control that caught a real mistake
//             in this test once already (entry 018).
//   SUBSTEP   sub-steps vs iterations at matched work.
//
// Run with:
//   UnrealEditor-Cmd.exe <uproject> -ExecCmds="Automation RunTests AgentSolver.TEMP.ImpulseContact; Quit"
//     -unattended -nopause -nosplash -nullrhi -log=impulse.log

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "PhysicsSolver/CreatureBatchState.h"
#include "PhysicsSolver/CreatureBatchSolver.h"
#include "PhysicsSolver/CreatureGroundContact.h"
#include "AgentSolver/CreatureRLEnvironment.h"
#include "AgentSolver/MutoRLTrainingDriver.h"

#if WITH_EDITOR
#include "PhysicsSolver/MutoTopology.h"
#include "Engine/SkeletalMesh.h"
#endif

#if WITH_DEV_AUTOMATION_TESTS && WITH_EDITOR

using namespace CreatureGroundContact;

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FMutoImpulseContact,
	"AgentSolver.TEMP.ImpulseContact",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

namespace ImpulseContactTest
{
	double TotalEnergy(const FCreatureBatchState& B, const FCreatureTopology& T, const FVector& G, bool& bFinite)
	{
		double E = 0.0; bFinite = true;
		for (int32 Body = 0; Body < T.NumBodies; ++Body)
		{
			const int32 I = B.BodyIndex(Body, 0);
			const FQuat Rot = B.GetBodyRot(Body, 0);
			const FVector Pos = B.GetBodyPos(Body, 0);
			const FVector W(B.AngVelX[I], B.AngVelY[I], B.AngVelZ[I]);
			const FVector V(B.LinVelX[I], B.LinVelY[I], B.LinVelZ[I]);
			const FVector CoMOff = Rot.RotateVector(T.BodyLocalCoMOffset[Body]);
			const FVector CoMVel = V + FVector::CrossProduct(W, CoMOff);
			const FMat3 R = FMat3::FromRotation(Rot);
			const FMat3 IW = R * FMat3::Diagonal(T.BodyInertiaDiagLocal[Body]) * R.Transpose();
			const double M = T.BodyMass[Body];
			const double KE = 0.5 * M * CoMVel.SizeSquared() + 0.5 * FVector::DotProduct(W, IW * W);
			const double PE = -M * FVector::DotProduct(G, Pos + CoMOff);
			if (!FMath::IsFinite(KE) || !FMath::IsFinite(PE)) { bFinite = false; return E; }
			E += KE + PE;
		}
		return E;
	}

	/** Holds the reset pose. Gains at or below Kp=5e7 are stable with contact off. */
	void ApplyPD(FCreatureBatchState& B, const FCreatureTopology& T, const TArray<float>& Target,
	             float Kp, float Kd, float MaxTau)
	{
		for (int32 D = 0; D < T.NumDOF; ++D)
		{
			for (int32 Env = 0; Env < 8; ++Env)
			{
				const int32 I = B.DOFIndex(D, Env);
				const float Err = B.JointPos[I] - Target[D * 8 + Env];
				B.JointTorque[I] = FMath::Clamp(-Kp * Err - Kd * B.JointVel[I], -MaxTau, MaxTau);
			}
		}
	}

	bool AnyNonFinite(const FCreatureBatchState& B)
	{
		for (int32 i = 0; i < B.PosX.Num(); ++i)
			if (!FMath::IsFinite(B.PosX[i]) || !FMath::IsFinite(B.LinVelX[i])) return true;
		for (int32 i = 0; i < B.JointVel.Num(); ++i)
			if (!FMath::IsFinite(B.JointVel[i])) return true;
		return false;
	}
}

bool FMutoImpulseContact::RunTest(const FString& Parameters)
{
	using namespace ImpulseContactTest;

	// ================= GATE =================
	// The articulated effective mass must reproduce closed form exactly, or no
	// number below carries information. Single free rigid body:
	//   through the CoM      -> m_eff = m
	//   at an offset lever   -> 1/m_eff = 1/m + (r x n)^T I^-1 (r x n)
	{
		AddInfo(TEXT("======== GATE: impulse response vs closed form ========"));
		constexpr float M = 10.0f;
		const FVector IDiag(200.0f, 500.0f, 900.0f);

		FCreatureTopology T1;
		T1.NumLimbs = 1;
		T1.Build({ 0 }, { 0 }, { INDEX_NONE });
		T1.BodyMass[0] = M;
		T1.BodyInertiaDiagLocal[0] = IDiag;
		T1.BodyLocalCoMOffset[0] = FVector::ZeroVector;

		FCreatureBatchState B1;
		B1.Init(T1, 8);
		B1.SetBodyPos(0, 0, FVector::ZeroVector);
		B1.SetBodyRot(0, 0, FQuat::Identity);

		FCreatureABASolver S1;
		S1.Step(B1, 0.0f, FVector::ZeroVector);
		S1.ComputeArticulatedInertias(B1);

		const FVector Resp = S1.ImpulseResponseAtPoint(B1, 0, 0, FVector::ZeroVector, FVector(0, 0, 1));
		const float InvM = static_cast<float>(Resp.Z);
		const float GotM = (InvM > 0.0f) ? 1.0f / InvM : 0.0f;
		const bool bGateA = FMath::Abs(GotM - M) / M < 1e-4f;
		AddInfo(FString::Printf(TEXT("   through CoM : m_eff expected=%.4f got=%.4f  %s"),
			M, GotM, bGateA ? TEXT("OK") : TEXT("<<<< WRONG")));

		const FVector R(100.0f, 0.0f, 0.0f), N(0.0f, 0.0f, 1.0f);
		const FVector RxN = FVector::CrossProduct(R, N);
		const float ExpInv = 1.0f / M
			+ (float)(RxN.X * RxN.X / IDiag.X + RxN.Y * RxN.Y / IDiag.Y + RxN.Z * RxN.Z / IDiag.Z);
		const float GotInv = (float)FVector::DotProduct(S1.ImpulseResponseAtPoint(B1, 0, 0, R, N), N);
		const bool bGateB = FMath::Abs(GotInv - ExpInv) / ExpInv < 1e-4f;
		AddInfo(FString::Printf(TEXT("   offset lever: 1/m_eff expected=%.6f got=%.6f  %s"),
			ExpInv, GotInv, bGateB ? TEXT("OK") : TEXT("<<<< WRONG")));

		TestTrue(TEXT("GATE: effective mass through CoM matches closed form"), bGateA);
		TestTrue(TEXT("GATE: effective mass at an offset lever matches closed form"), bGateB);
		AddInfo(TEXT(""));
	}

	// ================= full rig =================
	UMassMuscleProfileAssetMuscle* MuscleAsset = LoadObject<UMassMuscleProfileAssetMuscle>(nullptr, TEXT("/Game/Mesh/MuscleProfile_Muto.MuscleProfile_Muto"));
	UMassMuscleProfileAssetMass* MassAsset = LoadObject<UMassMuscleProfileAssetMass>(nullptr, TEXT("/Game/Mesh/MassProfile_Muto.MassProfile_Muto"));
	USkeletalMesh* SkeletalMesh = LoadObject<USkeletalMesh>(nullptr, TEXT("/Game/Mesh/Muto.Muto"));
	if (!TestNotNull(TEXT("MuscleProfile_Muto loaded"), MuscleAsset)) return false;
	if (!TestNotNull(TEXT("MassProfile_Muto loaded"), MassAsset)) return false;
	if (!TestNotNull(TEXT("Muto skeletal mesh loaded"), SkeletalMesh)) return false;

	FCreatureTopology T;
	TArray<FString> Warnings;
	TArray<FName> Names;
	if (!TestTrue(TEXT("BuildMutoTopology"), MutoTopology::BuildMutoTopology(SkeletalMesh, MassAsset, MuscleAsset, T, Warnings, &Names))) return false;

	const TArray<FContactPointDef> Points = BuildMutoContactPoints(T, MassAsset, Names);
	const FQuat StandRot = T.BodyRestRotInParent[0];
	const float StandH = AMutoRLTrainingDriver::ComputeDefaultStandingHeight(T, Points, StandRot);
	const FVector Gravity(0.0f, 0.0f, -980.0f);

	constexpr float Kp = 5.0e7f;   // measured stable with contact off; 5e8 is NOT
	constexpr float Kd = 5.0e5f;
	constexpr float MaxTau = 5.0e7f;
	constexpr float Seconds = 5.0f;

	auto Setup = [&](FCreatureBatchState& B, FCreatureABASolver& Solver, TArray<float>& Target)
	{
		B.Init(T, 8);
		FRandomStream S(1234);
		for (int32 Env = 0; Env < 8; ++Env)
			CreatureRLEnvironment::ResetEnv(B, Env, FVector(0, 0, StandH), StandRot, S, 0.0f, 0.0f);
		Solver.Step(B, 0.0f, Gravity);
		Target.SetNumZeroed(T.NumDOF * 8);
		for (int32 D = 0; D < T.NumDOF; ++D)
			for (int32 Env = 0; Env < 8; ++Env)
				Target[D * 8 + Env] = B.JointPos[B.DOFIndex(D, Env)];
	};

	// ---- PD CONTROL: contact disabled ----
	// Proves the actuation is not itself the thing diverging. This caught a real
	// error once: at Kp=5e8 the controller was unstable on its own and the
	// "contact" sweep was partly measuring it.
	AddInfo(TEXT("======== CONTROL: PD actuation, contact DISABLED ========"));
	for (const float KpTest : { 5.0e8f, 5.0e7f })
	{
		FCreatureBatchState B; FCreatureABASolver Solver; TArray<float> Target;
		Setup(B, Solver, Target);
		constexpr int32 Hz = 240;
		const float Dt = 1.0f / Hz;
		int32 Div = -1;
		for (int32 Step = 0; Step < FMath::RoundToInt(Seconds * Hz); ++Step)
		{
			ApplyPD(B, T, Target, KpTest, KpTest * 0.01f, MaxTau);
			Solver.Step(B, Dt, Gravity);
			if (AnyNonFinite(B)) { Div = Step; break; }
		}
		AddInfo(Div >= 0
			? FString::Printf(TEXT("   Kp=%.0e  DIVERGED at t=%.3fs  <<<< controller unstable on its own"), KpTest, Div * Dt)
			: FString::Printf(TEXT("   Kp=%.0e  finite %.1fs"), KpTest, Seconds));
	}

	// ---- SOFTNESS SWEEP ----
	AddInfo(TEXT(""));
	AddInfo(TEXT("======== SOFTNESS SWEEP (actuated stand, 240 Hz) ========"));
	AddInfo(TEXT("Hertz and DampingRatio are invariant to mass and timestep, so these"));
	AddInfo(TEXT("numbers transfer to any rig — unlike the stiffness they replaced."));
	struct FSoft { float Hz, Zeta; };
	for (const FSoft& Sf : { FSoft{60,10}, FSoft{30,10}, FSoft{15,10}, FSoft{30,2}, FSoft{30,30}, FSoft{8,10} })
	{
		FCreatureBatchState B; FCreatureABASolver Solver; TArray<float> Target;
		Setup(B, Solver, Target);
		constexpr int32 Hz = 240;
		const float Dt = 1.0f / Hz;

		FImpulseContactParams P;
		P.GroundZ = 0.0f; P.ContactHertz = Sf.Hz; P.DampingRatio = Sf.Zeta;
		P.Slop = 0.5f; P.FrictionCoefficient = 0.8f;
		P.Iterations = 8; P.RelaxIterations = 2;
		FImpulseContactCache Cache;

		int32 Div = -1;
		for (int32 Step = 0; Step < FMath::RoundToInt(Seconds * Hz); ++Step)
		{
			ApplyPD(B, T, Target, Kp, Kd, MaxTau);
			Solver.Step(B, Dt, Gravity);
			ResolveGroundContactImpulses(B, T, Points, P, Solver, Dt, Cache, nullptr);
			if (AnyNonFinite(B)) { Div = Step; break; }
		}
		AddInfo(Div >= 0
			? FString::Printf(TEXT("   hz=%5.1f zeta=%4.1f  DIVERGED at t=%.3fs"), Sf.Hz, Sf.Zeta, Div * Dt)
			: FString::Printf(TEXT("   hz=%5.1f zeta=%4.1f  finite %.1fs | torsoZ=%.1f"),
				Sf.Hz, Sf.Zeta, Seconds, static_cast<float>(B.GetBodyPos(0, 0).Z)));
	}

	// ---- SUBSTEPS vs ITERATIONS at matched work ----
	AddInfo(TEXT(""));
	AddInfo(TEXT("======== SUBSTEPS vs ITERATIONS at matched work ========"));
	AddInfo(TEXT("Box2D reports sub-steps beating iterations at equal cost. Measured here"));
	AddInfo(TEXT("there is an interior optimum instead — see entry 019."));
	struct FBudget { int32 Hz, Iters; };
	for (const FBudget& Bg : { FBudget{120,16}, FBudget{240,8}, FBudget{480,4}, FBudget{960,2}, FBudget{1920,1} })
	{
		FCreatureBatchState B; FCreatureABASolver Solver; TArray<float> Target;
		Setup(B, Solver, Target);
		const float Dt = 1.0f / Bg.Hz;

		FImpulseContactParams P;
		P.GroundZ = 0.0f; P.ContactHertz = 15.0f; P.DampingRatio = 10.0f;
		P.Slop = 0.5f; P.FrictionCoefficient = 0.8f;
		P.Iterations = Bg.Iters; P.RelaxIterations = FMath::Max(1, Bg.Iters / 4);
		FImpulseContactCache Cache;

		int32 Div = -1;
		for (int32 Step = 0; Step < FMath::RoundToInt(Seconds * Bg.Hz); ++Step)
		{
			ApplyPD(B, T, Target, Kp, Kd, MaxTau);
			Solver.Step(B, Dt, Gravity);
			ResolveGroundContactImpulses(B, T, Points, P, Solver, Dt, Cache, nullptr);
			if (AnyNonFinite(B)) { Div = Step; break; }
		}
		AddInfo(Div >= 0
			? FString::Printf(TEXT("   %4d Hz x %2d iters  DIVERGED at t=%.3fs"), Bg.Hz, Bg.Iters, Div * Dt)
			: FString::Printf(TEXT("   %4d Hz x %2d iters  finite %.1fs | torsoZ=%.1f"),
				Bg.Hz, Bg.Iters, Seconds, static_cast<float>(B.GetBodyPos(0, 0).Z)));
	}

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS && WITH_EDITOR
