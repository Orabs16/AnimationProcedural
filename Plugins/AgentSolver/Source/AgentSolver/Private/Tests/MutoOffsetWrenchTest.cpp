// TEMPORARY DIAGNOSTIC — see SOLVER_DEBUG_LOG.md entry 010.
//
// Isolates the EXTERNAL WRENCH path on a SINGLE free rigid body — no joints, no
// contact, no gravity, no articulation. Everything the contact bug could
// possibly involve is stripped away except one thing: a force applied at an
// OFFSET from the body's reference point.
//
// Why this specific test: AgentSolver.PendulumEnergyConservation already proves
// a force applied AT the reference point (zero lever) has the correct sign — it
// holds the anchor still by exactly cancelling gravity. Contact differs from
// that in one respect only: it has a LEVER, so it contributes a TORQUE via
// ApplyForceAtPoint's tau = (WorldPoint - BodyPos) x Force. The minimal-rig
// sweep showed the energy residual dE - W converging on EXACTLY -2W as dt -> 0,
// i.e. the system gains precisely what the wrench should remove — a structural
// inversion that survives the continuum limit, not an integration artifact.
//
// This checks the offset-force path against closed-form rigid-body dynamics:
//     a_com = F / m
//     alpha = I_com^-1 * (r_from_com x F)
//     dE/dt = F . v_application_point
// If the solver disagrees with these for a single body, the bug is in the
// wrench handling itself and has nothing to do with articulation or contact.
//
// Run with:
//   UnrealEditor-Cmd.exe <uproject> -ExecCmds="Automation RunTests AgentSolver.TEMP.OffsetWrench; Quit"
//     -unattended -nopause -nosplash -nullrhi -log=offsetwrench.log

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "CreatureBatchState.h"
#include "CreatureBatchSolver.h"

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FMutoOffsetWrench,
	"AgentSolver.TEMP.OffsetWrench",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FMutoOffsetWrench::RunTest(const FString& Parameters)
{
	const FVector NoGravity = FVector::ZeroVector;

	// Single free body. CoM AT the reference point first (simplest case), then
	// offset, so a CoM-offset bug and a lever bug can be told apart.
	struct FCase
	{
		const TCHAR* Label;
		FVector CoMOffset;   // body-local, reference point -> CoM
		FVector ForceAt;     // body-local, where the force is applied
		FVector Force;       // world
	};

	const FCase Cases[] = {
		{ TEXT("force AT ref point, CoM at ref  "), FVector::ZeroVector,        FVector::ZeroVector,        FVector(0, 0, 1000.0f) },
		{ TEXT("force OFFSET +X,   CoM at ref   "), FVector::ZeroVector,        FVector(100.0f, 0, 0),      FVector(0, 0, 1000.0f) },
		{ TEXT("force OFFSET +X,   CoM offset +X"), FVector(50.0f, 0, 0),       FVector(100.0f, 0, 0),      FVector(0, 0, 1000.0f) },
		{ TEXT("force OFFSET +Y,   CoM at ref   "), FVector::ZeroVector,        FVector(0, 100.0f, 0),      FVector(0, 0, 1000.0f) },
	};

	constexpr float Mass = 10.0f;
	const FVector InertiaDiag(200.0f, 500.0f, 900.0f);

	for (const FCase& C : Cases)
	{
		FCreatureTopology Topo;
		Topo.NumLimbs = 1;
		Topo.Build({ 0 }, { 0 }, { INDEX_NONE });
		Topo.BodyMass[0] = Mass;
		Topo.BodyInertiaDiagLocal[0] = InertiaDiag;
		Topo.BodyLocalCoMOffset[0] = C.CoMOffset;

		FCreatureBatchState B;
		B.Init(Topo, 1);
		FCreatureABASolver Solver;

		B.SetBodyPos(0, 0, FVector::ZeroVector);
		B.SetBodyRot(0, 0, FQuat::Identity);

		// ---- Analytic expectation, one step from rest ----
		// Identity rotation, so world == local and I_world == diag(InertiaDiag).
		const FVector RFromCoM = C.ForceAt - C.CoMOffset;      // CoM -> application point
		const FVector TorqueAboutCoM = FVector::CrossProduct(RFromCoM, C.Force);
		const FVector ExpectedAlpha(
			TorqueAboutCoM.X / InertiaDiag.X,
			TorqueAboutCoM.Y / InertiaDiag.Y,
			TorqueAboutCoM.Z / InertiaDiag.Z);
		const FVector ExpectedACoM = C.Force / Mass;

		constexpr float Dt = 1.0f / 10000.0f; // tiny step: one step from rest ~ the continuum limit
		B.ClearExternalForces(0);
		B.ApplyForceAtPoint(0, 0, C.Force, B.GetBodyPos(0, 0) + C.ForceAt);
		Solver.Step(B, Dt, NoGravity);

		const FVector AngVel(B.AngVelX[0], B.AngVelY[0], B.AngVelZ[0]);
		const FVector LinVel(B.LinVelX[0], B.LinVelY[0], B.LinVelZ[0]);
		const FVector CoMVel = LinVel + FVector::CrossProduct(AngVel, C.CoMOffset);

		const FVector GotAlpha = AngVel / Dt;
		const FVector GotACoM = CoMVel / Dt;

		AddInfo(FString::Printf(TEXT("---- %s ----"), C.Label));
		AddInfo(FString::Printf(TEXT("   alpha  expected=(%9.3f %9.3f %9.3f)  got=(%9.3f %9.3f %9.3f)"),
			ExpectedAlpha.X, ExpectedAlpha.Y, ExpectedAlpha.Z, GotAlpha.X, GotAlpha.Y, GotAlpha.Z));
		AddInfo(FString::Printf(TEXT("   a_com  expected=(%9.3f %9.3f %9.3f)  got=(%9.3f %9.3f %9.3f)"),
			ExpectedACoM.X, ExpectedACoM.Y, ExpectedACoM.Z, GotACoM.X, GotACoM.Y, GotACoM.Z));

		const float AlphaErr = (GotAlpha - ExpectedAlpha).Size() / FMath::Max(1.0f, (float)ExpectedAlpha.Size());
		const float AccErr = (GotACoM - ExpectedACoM).Size() / FMath::Max(1.0f, (float)ExpectedACoM.Size());
		AddInfo(FString::Printf(TEXT("   relative error: alpha=%.4g  a_com=%.4g   %s"),
			AlphaErr, AccErr,
			(AlphaErr < 1e-3f && AccErr < 1e-3f) ? TEXT("OK") : TEXT("<<<< MISMATCH")));

		// ---- Energy balance over a longer run, still single body ----
		// Sustained constant force. dE must equal the work done at the
		// application point: W = F . v_applicationPoint * dt, summed.
		{
			FCreatureBatchState B2;
			B2.Init(Topo, 1);
			FCreatureABASolver S2;
			B2.SetBodyPos(0, 0, FVector::ZeroVector);
			B2.SetBodyRot(0, 0, FQuat::Identity);

			auto Energy = [&](const FCreatureBatchState& S) -> double
			{
				const FQuat Rot = S.GetBodyRot(0, 0);
				const FVector W(S.AngVelX[0], S.AngVelY[0], S.AngVelZ[0]);
				const FVector V(S.LinVelX[0], S.LinVelY[0], S.LinVelZ[0]);
				const FVector CoMOff = Rot.RotateVector(C.CoMOffset);
				const FVector VC = V + FVector::CrossProduct(W, CoMOff);
				const FMat3 R = FMat3::FromRotation(Rot);
				const FMat3 IW = R * FMat3::Diagonal(InertiaDiag) * R.Transpose();
				return 0.5 * Mass * VC.SizeSquared() + 0.5 * FVector::DotProduct(W, IW * W);
			};

			constexpr float Dt2 = 1.0f / 4000.0f;
			constexpr int32 Steps = 400; // 0.1 s
			double WorkTotal = 0.0;
			const double E0 = Energy(B2);

			for (int32 s = 0; s < Steps; ++s)
			{
				B2.ClearExternalForces(0);
				const FQuat Rot = B2.GetBodyRot(0, 0);
				const FVector AppWorld = B2.GetBodyPos(0, 0) + Rot.RotateVector(C.ForceAt);
				B2.ApplyForceAtPoint(0, 0, C.Force, AppWorld);

				// Work uses the velocity of the APPLICATION point, pre-step.
				const FVector Wv(B2.AngVelX[0], B2.AngVelY[0], B2.AngVelZ[0]);
				const FVector Vv(B2.LinVelX[0], B2.LinVelY[0], B2.LinVelZ[0]);
				const FVector VApp = Vv + FVector::CrossProduct(Wv, AppWorld - B2.GetBodyPos(0, 0));
				WorkTotal += FVector::DotProduct(C.Force, VApp) * Dt2;

				S2.Step(B2, Dt2, NoGravity);
			}
			const double E1 = Energy(B2);
			const double dE = E1 - E0;
			const double Resid = dE - WorkTotal;
			const double Denom = FMath::Max(1.0, FMath::Max(FMath::Abs(WorkTotal), FMath::Abs(dE)));
			AddInfo(FString::Printf(
				TEXT("   energy: dE=%12.5g  W=%12.5g  dE-W=%12.5g  ratio=%+.4f  %s"),
				dE, WorkTotal, Resid, Resid / Denom,
				(FMath::Abs(Resid / Denom) < 0.02) ? TEXT("OK") : TEXT("<<<< ENERGY MISMATCH")));
		}
	}

	AddInfo(TEXT(""));
	AddInfo(TEXT("READ: a mismatch here means the external-wrench path is wrong for a SINGLE"));
	AddInfo(TEXT("rigid body, independent of joints and contact — the smallest possible cause."));

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
