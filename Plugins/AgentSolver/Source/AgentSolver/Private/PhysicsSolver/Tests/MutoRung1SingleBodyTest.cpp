// TEMPORARY DIAGNOSTIC — see SOLVER_DEBUG_LOG.md entry 005.
//
// Rung 1 of the minimal-core ladder: a SINGLE rigid body, no joints, no
// contact, no gravity. One body has no parent, so the velocity-product
// transport term C is identically zero here and this test is unaffected by
// that change either way — it probes the pre-existing solver core only.
//
// Zero gravity + no external force means the exact answer is known:
//   - the CoM's velocity is CONSTANT (Newton's first law)
//   - angular momentum in world space is CONSTANT
//   - kinetic energy is CONSTANT
// A free tumbling body must satisfy all three. Any drift is a bug in the
// single-body core, upstream of every articulation concern.
//
// WHY THIS EXISTS: the equation the solver assembles is Featherstone's
//     f = I a + v x* (I v)
// in which `a` is the SPATIAL acceleration. But Pass 3a integrates the stored
// linear velocity directly as `LinVel += A.Lin * Dt`, which is only valid if
// A.Lin is the CLASSICAL acceleration of the body's reference material point.
// The two differ by w x v. For a free body with its reference AT the CoM the
// equation yields A.Lin = f/m - w x v, so integrating it as-is injects a
// spurious -w x v drift that is perpendicular to the velocity — i.e. it turns
// a straight fall into a curve, and grows with w. Muto's torso has
// BodyLocalCoMOffset[0] == ZeroVector, so it is exactly this case, and the
// measured blowup was in LinVelX (HORIZONTAL) during a vertical fall.
//
// This test decides that question with no articulation in the way.
//
// Run with:
//   UnrealEditor-Cmd.exe <uproject> -ExecCmds="Automation RunTests AgentSolver.TEMP.Rung1SingleBody; Quit"
//     -unattended -nopause -nosplash -nullrhi -log=rung1.log

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "PhysicsSolver/CreatureBatchState.h"
#include "PhysicsSolver/CreatureBatchSolver.h"

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FMutoRung1SingleBody,
	"AgentSolver.TEMP.Rung1SingleBody",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FMutoRung1SingleBody::RunTest(const FString& Parameters)
{
	const FVector NoGravity = FVector::ZeroVector;

	struct FCase { const TCHAR* Label; FVector CoMOffset; int32 Hz; bool bUseSIMD; };
	const FCase Cases[] = {
		{ TEXT("ref AT CoM,      60Hz, scalar"), FVector::ZeroVector,        60, false },
		{ TEXT("ref AT CoM,     960Hz, scalar"), FVector::ZeroVector,       960, false },
		{ TEXT("ref AT CoM,      60Hz, SIMD  "), FVector::ZeroVector,        60, true  },
		{ TEXT("ref OFF CoM,     60Hz, scalar"), FVector(10.0f, 0.0f, 0.0f), 60, false },
	};

	for (const FCase& Case : Cases)
	{
		FCreatureTopology Topo;
		Topo.NumLimbs = 1;
		Topo.Build({ 0 }, { 0 }, { INDEX_NONE }); // one body, zero DOF
		Topo.BodyMass[0] = 10.0f;
		// Deliberately asymmetric so the body genuinely tumbles (a symmetric
		// inertia would spin about a fixed axis and hide the bug).
		Topo.BodyInertiaDiagLocal[0] = FVector(200.0f, 500.0f, 900.0f);
		Topo.BodyLocalCoMOffset[0] = Case.CoMOffset;

		FCreatureBatchState Batch;
		Batch.Init(Topo, 1);
		FCreatureABASolver Solver;

		const FVector V0(10.0f, 0.0f, 0.0f);
		const FVector W0(1.0f, 2.0f, 0.5f);
		Batch.SetBodyPos(0, 0, FVector::ZeroVector);
		Batch.SetBodyRot(0, 0, FQuat::Identity);
		Batch.LinVelX[0] = (float)V0.X; Batch.LinVelY[0] = (float)V0.Y; Batch.LinVelZ[0] = (float)V0.Z;
		Batch.AngVelX[0] = (float)W0.X; Batch.AngVelY[0] = (float)W0.Y; Batch.AngVelZ[0] = (float)W0.Z;

		auto CoMVelocity = [&]() -> FVector
		{
			const FQuat Rot = Batch.GetBodyRot(0, 0);
			const FVector AngVel(Batch.AngVelX[0], Batch.AngVelY[0], Batch.AngVelZ[0]);
			const FVector LinVel(Batch.LinVelX[0], Batch.LinVelY[0], Batch.LinVelZ[0]);
			return LinVel + FVector::CrossProduct(AngVel, Rot.RotateVector(Topo.BodyLocalCoMOffset[0]));
		};
		auto AngularMomentum = [&]() -> FVector
		{
			const FQuat Rot = Batch.GetBodyRot(0, 0);
			const FVector AngVel(Batch.AngVelX[0], Batch.AngVelY[0], Batch.AngVelZ[0]);
			const FMat3 RotM = FMat3::FromRotation(Rot);
			const FMat3 IWorld = RotM * FMat3::Diagonal(Topo.BodyInertiaDiagLocal[0]) * RotM.Transpose();
			return IWorld * AngVel;
		};
		auto KineticEnergy = [&]() -> double
		{
			const FVector AngVel(Batch.AngVelX[0], Batch.AngVelY[0], Batch.AngVelZ[0]);
			const FQuat Rot = Batch.GetBodyRot(0, 0);
			const FMat3 RotM = FMat3::FromRotation(Rot);
			const FMat3 IWorld = RotM * FMat3::Diagonal(Topo.BodyInertiaDiagLocal[0]) * RotM.Transpose();
			const FVector Vc = CoMVelocity();
			return 0.5 * Topo.BodyMass[0] * Vc.SizeSquared()
				+ 0.5 * FVector::DotProduct(AngVel, IWorld * AngVel);
		};

		const FVector VcRef = CoMVelocity();
		const FVector LRef = AngularMomentum();
		const double KERef = KineticEnergy();

		const float Dt = 1.0f / Case.Hz;
		const int32 Steps = Case.Hz * 5; // 5 simulated seconds

		for (int32 s = 0; s < Steps; ++s)
		{
			if (Case.bUseSIMD) Solver.StepSIMD(Batch, Dt, NoGravity);
			else               Solver.StepScalar(Batch, Dt, NoGravity);
		}

		const FVector VcEnd = CoMVelocity();
		const FVector LEnd = AngularMomentum();
		const double KEEnd = KineticEnergy();

		AddInfo(FString::Printf(TEXT("---- %s ----"), Case.Label));
		AddInfo(FString::Printf(TEXT("   CoM vel : (%8.4f %8.4f %8.4f) -> (%12.4f %12.4f %12.4f)   |drift| = %.4g"),
			VcRef.X, VcRef.Y, VcRef.Z, VcEnd.X, VcEnd.Y, VcEnd.Z, (VcEnd - VcRef).Size()));
		AddInfo(FString::Printf(TEXT("   AngMom  : |L| %12.4f -> %12.4f    rel change = %.4g"),
			LRef.Size(), LEnd.Size(),
			LRef.Size() > 0.0 ? FMath::Abs(LEnd.Size() - LRef.Size()) / LRef.Size() : 0.0));
		AddInfo(FString::Printf(TEXT("   KE      : %12.4f -> %12.4f    rel change = %.4g"),
			KERef, KEEnd, KERef > 0.0 ? FMath::Abs(KEEnd - KERef) / KERef : 0.0));
	}

	AddInfo(TEXT(""));
	AddInfo(TEXT("EXACT ANSWER: CoM velocity drift == 0, |L| unchanged, KE unchanged."));
	AddInfo(TEXT("A CoM-velocity drift with the reference AT the CoM can only come from the"));
	AddInfo(TEXT("spatial-vs-classical acceleration mismatch in Pass 3a's integration."));

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
