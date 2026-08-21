// Permanent guards for the 2026-08-17 solver upgrade: per-DOF armature and
// damping (MuJoCo's `armature`/`damping`), the global assembled-constraint solve
// (MuJoCo's J M^-1 J^T + R), the capsule end-cap derivation, saturated-joint
// welding, and the ball-joint reduced-bias fix found while adding armature.
//
// WHY THIS IS PERMANENT AND NOT A TEMP DIAGNOSTIC. Every one of these guards an
// invariant that the existing suite is structurally blind to:
//
//  - Every synthetic test topology leaves DOFArmatureRatio and
//    DOFDampingTimeConstant at their zeroed defaults, so the whole existing
//    suite passing proves only that armature and damping are exact no-ops when
//    off. It says nothing about whether they are CORRECT when on. Parts C, D
//    and E turn them on and check the arithmetic against closed form.
//  - The ball-joint reduced-bias bug (Part F) survived from the day joint-space
//    impulses were added because nothing asserted the one property it breaks:
//    an impulse INTERNAL to a floating-base tree cannot change the system's
//    total angular momentum. That is a one-line physical law and it is now
//    checked.
//  - The capsule end-cap bug (Part A) survived because it produced a duplicate
//    row rather than a wrong number, and no test counted rows.
//
// Nothing here is a sweep or a trace. Each part asserts a specific closed-form
// or conservation property, so a failure names a mechanism rather than a
// symptom.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "PhysicsSolver/CreatureBatchSolver.h"
#include "PhysicsSolver/CreatureGroundContact.h"

#if WITH_DEV_AUTOMATION_TESTS

using namespace CreatureGroundContact;

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
	FMutoSolverUpgradeTest,
	"AgentSolver.SolverUpgrade",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

namespace
{
	/** Root + one child, child type chosen by DOFCount (1 = revolute, 3 = ball). */
	FCreatureTopology MakeTwoBodyChain(int32 ChildDOFCount)
	{
		FCreatureTopology Topo;
		TArray<int32> Parent = { 0, 0 };
		TArray<int32> DOFCount = { 0, ChildDOFCount };
		TArray<int32> Limb = { INDEX_NONE, 0 };
		Topo.NumLimbs = 1;
		Topo.Build(Parent, DOFCount, Limb);

		Topo.BodyMass[0] = 10.0f;
		Topo.BodyInertiaDiagLocal[0] = FVector(200.0f, 200.0f, 200.0f);
		Topo.BodyLocalCoMOffset[0] = FVector::ZeroVector;

		Topo.BodyMass[1] = 4.0f;
		Topo.BodyInertiaDiagLocal[1] = FVector(60.0f, 60.0f, 60.0f);
		Topo.BodyJointOffsetInParent[1] = FVector(30.0f, 0.0f, 0.0f);
		Topo.BodyLocalCoMOffset[1] = FVector(15.0f, 0.0f, 0.0f);
		Topo.BodyJointAxisLocal[1] = FVector(0.0f, 0.0f, 1.0f); // revolute about world +Z at rest
		return Topo;
	}

	/** Total angular momentum about the world origin, summed over every body. */
	FVector TotalAngularMomentum(const FCreatureBatchState& Batch, int32 Env)
	{
		const FCreatureTopology& Topo = Batch.GetTopology();
		FVector L = FVector::ZeroVector;
		for (int32 Body = 0; Body < Topo.NumBodies; ++Body)
		{
			const int32 Idx = Batch.BodyIndex(Body, Env);
			const FQuat Rot = Batch.GetBodyRot(Body, Env);
			const FVector Pos = Batch.GetBodyPos(Body, Env);
			const FVector W(Batch.AngVelX[Idx], Batch.AngVelY[Idx], Batch.AngVelZ[Idx]);
			const FVector V(Batch.LinVelX[Idx], Batch.LinVelY[Idx], Batch.LinVelZ[Idx]);

			const float Mass = Topo.BodyMass[Body];
			const FVector CoMOffset = Rot.RotateVector(Topo.BodyLocalCoMOffset[Body]);
			const FVector CoMPos = Pos + CoMOffset;
			// Velocity of the CoM, given the body's reference point moves at V.
			const FVector CoMVel = V + FVector::CrossProduct(W, CoMOffset);

			// Rotational part about the CoM, then the orbital part.
			const FMat3 RotM = FMat3::FromRotation(Rot);
			const FMat3 IAboutCoM = RotM * FMat3::Diagonal(Topo.BodyInertiaDiagLocal[Body]) * RotM.Transpose();
			L += IAboutCoM * W;
			L += FVector::CrossProduct(CoMPos, Mass * CoMVel);
		}
		return L;
	}
}

bool FMutoSolverUpgradeTest::RunTest(const FString& Parameters)
{
	const FVector NoGravity = FVector::ZeroVector;
	constexpr float Dt = 1.0f / 240.0f;

	// =====================================================================
	// A. CAPSULE END CAPS — the duplicate-row bug
	// =====================================================================
	// A zero tip offset with a real half height used to yield two COINCIDENT
	// ends while still reporting two of them, producing two independent contact
	// rows at one world point. FElbow3_*/BElbow3_* are flagged CanTouchGround
	// and measured at leverArm = 0.00, so this fired on the real rig.
	{
		FVector Tip, Base;

		// Interior body: no tip offset, real half height -> a capsule along local
		// +X, two DISTINCT ends.
		const int32 NumInterior = GetCapsuleLocalEnds(FVector::ZeroVector, 50.0f, Tip, Base);
		TestEqual(TEXT("A: interior body reports 2 distinct capsule ends"), NumInterior, 2);
		TestTrue(TEXT("A: interior capsule ends are actually distinct"),
			FVector::Dist(Tip, Base) > 1.0f);
		TestTrue(TEXT("A: interior capsule base sits at the body origin"), Base.IsNearlyZero());
		TestTrue(TEXT("A: interior capsule runs along local +X toward the child"),
			Tip.X > 99.0f && FMath::Abs(Tip.Y) < KINDA_SMALL_NUMBER && FMath::Abs(Tip.Z) < KINDA_SMALL_NUMBER);

		// Degenerate: no offset, no half height -> a sphere, which is ONE row.
		const int32 NumSphere = GetCapsuleLocalEnds(FVector::ZeroVector, 0.0f, Tip, Base);
		TestEqual(TEXT("A: zero-length capsule collapses to a single end"), NumSphere, 1);

		// Leaf body with a fused tip: base pulled back along the tip axis.
		const int32 NumLeaf = GetCapsuleLocalEnds(FVector(0.0f, 0.0f, -100.0f), 20.0f, Tip, Base);
		TestEqual(TEXT("A: leaf body with a fused tip reports 2 ends"), NumLeaf, 2);
		TestTrue(TEXT("A: leaf capsule tip is the authored offset"), Tip.Equals(FVector(0.0f, 0.0f, -100.0f), 0.01f));
		TestTrue(TEXT("A: leaf capsule base is pulled back 2x half-height along the tip axis"),
			Base.Equals(FVector(0.0f, 0.0f, -60.0f), 0.01f));

		// A tip offset SHORTER than the pull-back must still give distinct ends
		// rather than folding past the origin into a coincidence.
		const int32 NumShort = GetCapsuleLocalEnds(FVector(5.0f, 0.0f, 0.0f), 2.5f, Tip, Base);
		TestEqual(TEXT("A: short capsule still reports 2 distinct ends"), NumShort, 2);
	}

	// =====================================================================
	// B. ARMATURE AT ZERO IS AN EXACT NO-OP
	// =====================================================================
	// The ball-joint reduction had to move from the D == Irot special case to the
	// general I^a = I^A - U D^-1 U^T form. Written naively that reintroduces
	// catastrophic cancellation; written via the r/(1+r) identities it vanishes
	// identically at r = 0. This asserts the factors themselves, which is the
	// cheapest place to catch a regression in that algebra.
	{
		FCreatureTopology Topo = MakeTwoBodyChain(3);
		const FCreatureABASolver::FJointPassiveFactors Off =
			FCreatureABASolver::GetJointPassiveFactors(Topo, 1, Dt);
		TestEqual(TEXT("B: ArmatureScale is exactly 1 when the ratio is 0"), Off.ArmatureScale, 1.0f);
		TestEqual(TEXT("B: ArmatureFrac is exactly 0 when the ratio is 0"), Off.ArmatureFrac, 0.0f);
		TestFalse(TEXT("B: damping is off when the time constant is 0"), Off.HasDamping());

		// r = 1 -> D doubles, and the reduced-block coefficient is r/(1+r) = 1/2.
		for (int32 k = 0; k < 3; ++k) { Topo.DOFArmatureRatio[k] = 1.0f; }
		const FCreatureABASolver::FJointPassiveFactors On =
			FCreatureABASolver::GetJointPassiveFactors(Topo, 1, Dt);
		TestEqual(TEXT("B: ArmatureScale is 1+r"), On.ArmatureScale, 2.0f);
		TestEqual(TEXT("B: ArmatureFrac is r/(1+r)"), On.ArmatureFrac, 0.5f);

		// Damping time constant equal to one step removes exactly all of it; the
		// clamp at 1 is what makes explicit damping unconditionally stable.
		Topo.DOFDampingTimeConstant[0] = Dt;
		const FCreatureABASolver::FJointPassiveFactors Crit =
			FCreatureABASolver::GetJointPassiveFactors(Topo, 1, Dt);
		TestEqual(TEXT("B: DampFrac is dt/T"), Crit.DampFrac, 1.0f);
		Topo.DOFDampingTimeConstant[0] = Dt * 0.1f; // T < dt: would overshoot if unclamped
		const FCreatureABASolver::FJointPassiveFactors Over =
			FCreatureABASolver::GetJointPassiveFactors(Topo, 1, Dt);
		TestEqual(TEXT("B: DampFrac is clamped at 1 so damping can never reverse a joint"), Over.DampFrac, 1.0f);

		// Dt <= 0 must disable damping rather than divide by it — several
		// diagnostics call Step() with Dt == 0 purely to refresh kinematics.
		const FCreatureABASolver::FJointPassiveFactors ZeroDt =
			FCreatureABASolver::GetJointPassiveFactors(Topo, 1, 0.0f);
		TestEqual(TEXT("B: Dt == 0 disables damping instead of dividing by it"), ZeroDt.DampFrac, 0.0f);
	}

	// =====================================================================
	// C. ARMATURE SCALES THE JOINT'S OWN INERTIA BY (1+r)
	// =====================================================================
	// Armature multiplies D, and D is the denominator of every joint
	// acceleration: qddot = (u - U^T a_parent) / D. So qddot scales as 1/(1+r)
	// ONLY when a_parent is negligible.
	//
	// That caveat is not pedantry, it is what the first version of this test got
	// wrong. On a FREE-FLOATING 2-body chain the parent recoils from the joint
	// torque, which amplifies the relative acceleration by a large,
	// configuration-dependent factor (measured ~20x on this rig) -- so armature
	// there competes against the RELATIVE inertia of the pair, not against D, and
	// r=1 was measured cutting qddot by 20x rather than 2x. Both numbers are
	// correct physics; only the naive expectation was wrong. The same misreading
	// is exactly what made the torque-based damping formulation fail, so it is
	// worth pinning down in both directions here.
	//
	// Part C1 pins the exact 1/(1+r) law against a near-fixed base. Part C2 pins
	// the property that actually matters on a real rig: more armature always means
	// less joint acceleration, monotonically, whatever the base is doing.
	{
		// ---- C1: heavy root, so a_parent ~ 0 and qddot ~ tau/D ----
		auto MeasureAgainstFixedBase = [&](float Ratio) -> float
		{
			FCreatureTopology Topo = MakeTwoBodyChain(1);
			// Root inertia 5e6 against a child D of ~1e3: the recoil term is then
			// four orders of magnitude down, well inside the 2% tolerance below.
			Topo.BodyMass[0] = 1.0e5f;
			Topo.BodyInertiaDiagLocal[0] = FVector(5.0e6f, 5.0e6f, 5.0e6f);
			Topo.DOFArmatureRatio[0] = Ratio;
			FCreatureBatchState Batch;
			Batch.Init(Topo, 1);
			Batch.JointTorque[Batch.DOFIndex(0, 0)] = 1000.0f;
			FCreatureABASolver Solver;
			Solver.Step(Batch, Dt, NoGravity);
			return Batch.JointVel[Batch.DOFIndex(0, 0)] / Dt; // qd started at 0
		};

		const float Fixed0 = MeasureAgainstFixedBase(0.0f);
		const float Fixed1 = MeasureAgainstFixedBase(1.0f);
		const float Fixed3 = MeasureAgainstFixedBase(3.0f);
		AddInfo(FString::Printf(TEXT("C1: qddot vs near-fixed base  r=0: %.6g  r=1: %.6g  r=3: %.6g"),
			Fixed0, Fixed1, Fixed3));
		TestTrue(TEXT("C1: a torque produces a nonzero joint acceleration at r=0"), FMath::Abs(Fixed0) > 1e-3f);
		TestTrue(TEXT("C1: r=1 halves the joint acceleration (D scaled by 1+r)"),
			FMath::IsNearlyEqual(Fixed1 / Fixed0, 0.5f, 0.02f));
		TestTrue(TEXT("C1: r=3 quarters the joint acceleration"),
			FMath::IsNearlyEqual(Fixed3 / Fixed0, 0.25f, 0.02f));

		// ---- C2: free-floating base, monotonic reduction ----
		auto MeasureFree = [&](float Ratio) -> float
		{
			FCreatureTopology Topo = MakeTwoBodyChain(1);
			Topo.DOFArmatureRatio[0] = Ratio;
			FCreatureBatchState Batch;
			Batch.Init(Topo, 1);
			Batch.JointTorque[Batch.DOFIndex(0, 0)] = 1000.0f;
			FCreatureABASolver Solver;
			Solver.Step(Batch, Dt, NoGravity);
			return Batch.JointVel[Batch.DOFIndex(0, 0)] / Dt;
		};

		const float Free0 = MeasureFree(0.0f);
		const float Free1 = MeasureFree(1.0f);
		const float Free3 = MeasureFree(3.0f);
		AddInfo(FString::Printf(TEXT("C2: qddot, free base  r=0: %.6g  r=1: %.6g  r=3: %.6g  (parent recoil amplifies)"),
			Free0, Free1, Free3));
		TestTrue(TEXT("C2: armature strictly reduces joint acceleration on a free base"),
			FMath::Abs(Free1) < FMath::Abs(Free0));
		TestTrue(TEXT("C2: more armature reduces it further (monotonic)"),
			FMath::Abs(Free3) < FMath::Abs(Free1));
		TestTrue(TEXT("C2: armature never flips the sign of the response"), Free1 * Free0 > 0.0f);

		// ---- C3: the same law on a BALL joint ----
		// The ball-joint path needed the general I^a = I^A - U D^-1 U^T form, via
		// the r/(1+r) identities. Against a near-fixed base its qddot must obey the
		// same 1/(1+r) scaling as the revolute, which is the cheapest check that
		// those identities were derived correctly.
		auto MeasureBallAgainstFixedBase = [&](float Ratio) -> float
		{
			FCreatureTopology Topo = MakeTwoBodyChain(3);
			Topo.BodyMass[0] = 1.0e5f;
			Topo.BodyInertiaDiagLocal[0] = FVector(5.0e6f, 5.0e6f, 5.0e6f);
			for (int32 k = 0; k < 3; ++k) { Topo.DOFArmatureRatio[k] = Ratio; }
			FCreatureBatchState Batch;
			Batch.Init(Topo, 1);
			Batch.JointTorque[Batch.DOFIndex(2, 0)] = 1000.0f; // about the joint's Z
			FCreatureABASolver Solver;
			Solver.Step(Batch, Dt, NoGravity);
			return Batch.JointVel[Batch.DOFIndex(2, 0)] / Dt;
		};

		const float Ball0 = MeasureBallAgainstFixedBase(0.0f);
		const float Ball1 = MeasureBallAgainstFixedBase(1.0f);
		AddInfo(FString::Printf(TEXT("C3: ball qddot vs near-fixed base  r=0: %.6g  r=1: %.6g"), Ball0, Ball1));
		TestTrue(TEXT("C3: a ball joint accelerates under torque at r=0"), FMath::Abs(Ball0) > 1e-3f);
		TestTrue(TEXT("C3: r=1 halves a ball joint's acceleration too"),
			FMath::IsNearlyEqual(Ball1 / Ball0, 0.5f, 0.05f));
	}

	// =====================================================================
	// D. ARMATURE AT r=0 PRESERVES BALL-JOINT MOMENTUM CONSERVATION
	// =====================================================================
	// Entry 012 established that a closed chain containing a ball joint conserves
	// angular momentum to ~1e-6 once the joint-frame bug was fixed. The
	// generalized reduction added for armature must not disturb that at r = 0 --
	// if the r/(1+r) identities were written with a subtraction instead, float32
	// residue on inertias of this magnitude would show up here first.
	{
		FCreatureTopology Topo = MakeTwoBodyChain(3);
		FCreatureBatchState Batch;
		Batch.Init(Topo, 1);
		// Spin the ball joint; no gravity, no external force -> closed system.
		Batch.JointVel[Batch.DOFIndex(0, 0)] = 0.7f;
		Batch.JointVel[Batch.DOFIndex(1, 0)] = -0.4f;
		Batch.JointVel[Batch.DOFIndex(2, 0)] = 0.3f;

		FCreatureABASolver Solver;
		Solver.RecomputeKinematics(Batch);
		const FVector L0 = TotalAngularMomentum(Batch, 0);
		for (int32 i = 0; i < 240; ++i)
		{
			Solver.Step(Batch, Dt, NoGravity);
		}
		const FVector L1 = TotalAngularMomentum(Batch, 0);
		const float Drift = (float)(L1 - L0).Size() / FMath::Max(1.0f, (float)L0.Size());

		AddInfo(FString::Printf(TEXT("D: |L0|=%.6g  |L1|=%.6g  relative drift=%.3e"),
			L0.Size(), L1.Size(), Drift));
		TestTrue(TEXT("D: ball-joint chain conserves angular momentum at r=0 (armature added no residue)"),
			Drift < 1.0e-3f);
	}

	// =====================================================================
	// E. DAMPING REMOVES EXACTLY dt/T OF JOINT VELOCITY PER SUBSTEP
	// =====================================================================
	// The time-constant parameterization's defining property. It holds EXACTLY --
	// including on a free-floating base -- precisely because ApplyJointDamping
	// measures the whole-tree response gain and solves for the impulse that
	// achieves the requested change, rather than applying a torque and hoping.
	//
	// This is the test that caught the original torque-based formulation. Under
	// that version, T=10dt removed 197% of the joint's velocity (reversing it) and
	// T=dt overshot by 19x, because the parent's recoil amplifies a joint torque
	// by an unbounded configuration-dependent factor. Those numbers are recorded
	// here so the failure mode cannot quietly return.
	{
		auto MeasureDecay = [&](float TimeConst, float Qd0) -> float
		{
			FCreatureTopology Topo = MakeTwoBodyChain(1);
			Topo.DOFDampingTimeConstant[0] = TimeConst;
			FCreatureBatchState Batch;
			Batch.Init(Topo, 1);
			Batch.JointVel[Batch.DOFIndex(0, 0)] = Qd0;
			FCreatureABASolver Solver;
			Solver.RecomputeKinematics(Batch);
			// Damping only — no Step(), so nothing else can move the joint and the
			// fraction removed is attributable to damping alone.
			Solver.ApplyJointDamping(Batch, Dt);
			return Batch.JointVel[Batch.DOFIndex(0, 0)] / Qd0;
		};

		// T = 10*dt -> keeps exactly 90%. Tight tolerance on purpose: this is a
		// solved quantity, not an approximated one.
		const float Keep10 = MeasureDecay(Dt * 10.0f, 2.0f);
		AddInfo(FString::Printf(TEXT("E: T=10dt keeps %.5f of joint velocity (expect 0.90)"), Keep10));
		TestTrue(TEXT("E: T=10dt removes exactly 10% of joint velocity"),
			FMath::IsNearlyEqual(Keep10, 0.9f, 0.005f));

		const float Keep4 = MeasureDecay(Dt * 4.0f, -3.0f);
		AddInfo(FString::Printf(TEXT("E: T=4dt keeps %.5f (expect 0.75, negative qd)"), Keep4));
		TestTrue(TEXT("E: the fraction is independent of the sign and size of qd"),
			FMath::IsNearlyEqual(Keep4, 0.75f, 0.005f));

		// T = dt -> removes all of it, and must NOT overshoot into a reversal.
		const float KeepAll = MeasureDecay(Dt, 2.0f);
		AddInfo(FString::Printf(TEXT("E: T=dt keeps %.5f (expect ~0)"), KeepAll));
		TestTrue(TEXT("E: T=dt removes essentially all joint velocity"), FMath::Abs(KeepAll) < 0.01f);

		// T far below dt is the unconditional-stability case: clamped, no reversal.
		const float KeepTiny = MeasureDecay(Dt * 0.01f, 2.0f);
		AddInfo(FString::Printf(TEXT("E: T=dt/100 keeps %.5f (clamped at full removal, NOT a reversal)"), KeepTiny));
		TestTrue(TEXT("E: T well below dt stays clamped rather than exploding"),
			FMath::Abs(KeepTiny) < 0.01f);
		TestTrue(TEXT("E: damping never reverses the joint at any time constant"), KeepTiny > -0.01f);

		// Ball joints: the isotropic path, damping the joint's own spin direction.
		{
			FCreatureTopology Topo = MakeTwoBodyChain(3);
			for (int32 k = 0; k < 3; ++k) { Topo.DOFDampingTimeConstant[k] = Dt * 5.0f; }
			FCreatureBatchState Batch;
			Batch.Init(Topo, 1);
			const FVector W0(0.6f, -0.8f, 0.3f);
			Batch.JointVel[Batch.DOFIndex(0, 0)] = (float)W0.X;
			Batch.JointVel[Batch.DOFIndex(1, 0)] = (float)W0.Y;
			Batch.JointVel[Batch.DOFIndex(2, 0)] = (float)W0.Z;
			FCreatureABASolver Solver;
			Solver.RecomputeKinematics(Batch);
			Solver.ApplyJointDamping(Batch, Dt);
			const FVector W1(
				Batch.JointVel[Batch.DOFIndex(0, 0)],
				Batch.JointVel[Batch.DOFIndex(1, 0)],
				Batch.JointVel[Batch.DOFIndex(2, 0)]);
			const float Keep = (float)(W1.Size() / W0.Size());
			AddInfo(FString::Printf(TEXT("E: ball joint T=5dt keeps %.5f of |w| (expect 0.80)"), Keep));
			TestTrue(TEXT("E: ball-joint damping removes exactly dt/T of the spin magnitude"),
				FMath::IsNearlyEqual(Keep, 0.8f, 0.01f));
			// Isotropic means the direction is preserved.
			const float DirDot = (float)FVector::DotProduct(W0.GetSafeNormal(), W1.GetSafeNormal());
			TestTrue(TEXT("E: ball-joint damping preserves the spin direction (isotropic)"), DirDot > 0.999f);
		}

		// Damping must be MOMENTUM CONSERVING -- it is an internal joint torque,
		// so the parent takes its share of the reaction. A post-hoc scaling of
		// JointVel (the other obvious cheap implementation) would fail this.
		{
			FCreatureTopology Topo = MakeTwoBodyChain(1);
			Topo.DOFDampingTimeConstant[0] = Dt * 4.0f;
			FCreatureBatchState Batch;
			Batch.Init(Topo, 1);
			Batch.SetBodyRot(0, 0, FQuat(FVector(0.3f, 0.8f, 0.5f).GetSafeNormal(), 0.6f));
			Batch.JointVel[Batch.DOFIndex(0, 0)] = 3.0f;
			FCreatureABASolver Solver;
			Solver.RecomputeKinematics(Batch);
			const FVector LBefore = TotalAngularMomentum(Batch, 0);
			Solver.ApplyJointDamping(Batch, Dt);
			Solver.RecomputeKinematics(Batch);
			const FVector LAfter = TotalAngularMomentum(Batch, 0);
			const float Drift = (float)(LAfter - LBefore).Size() / FMath::Max(1.0f, (float)LBefore.Size());
			AddInfo(FString::Printf(TEXT("E: damping momentum drift=%.3e (|L|=%.6g)"), Drift, LBefore.Size()));
			TestTrue(TEXT("E: damping conserves total angular momentum (parent takes the reaction)"),
				Drift < 1.0e-2f);
		}

		// And it must genuinely dissipate over a run, not merely rescale one step.
		FCreatureTopology Topo = MakeTwoBodyChain(1);
		Topo.DOFDampingTimeConstant[0] = 0.05f;
		FCreatureBatchState Batch;
		Batch.Init(Topo, 1);
		Batch.JointVel[Batch.DOFIndex(0, 0)] = 5.0f;
		FCreatureABASolver Solver;
		Solver.RecomputeKinematics(Batch);
		for (int32 i = 0; i < 240; ++i)
		{
			Solver.Step(Batch, Dt, NoGravity);
			Solver.ApplyJointDamping(Batch, Dt);
		}
		const float Qd1s = Batch.JointVel[Batch.DOFIndex(0, 0)];
		AddInfo(FString::Printf(TEXT("E: after 1 s at T=0.05, qd went 5.0 -> %.6g"), Qd1s));
		TestTrue(TEXT("E: damping dissipates joint velocity over time"), FMath::Abs(Qd1s) < 0.5f);
	}

	// =====================================================================
	// F. AN INTERNAL JOINT IMPULSE CONSERVES TOTAL ANGULAR MOMENTUM
	// =====================================================================
	// THE BALL-JOINT REDUCED-BIAS BUG. SolveImpulseResponse's ball branch
	// hardcoded the reduced bias's ANGULAR part to zero, which is correct only
	// when the joint impulse is zero. Once ball-joint cone limit rows began
	// passing a real joint impulse through that branch (2026-08-16), the angular
	// reaction a hip or spine limit should transmit to its parent was silently
	// discarded -- so the limit braced against its own child subtree only.
	//
	// A joint impulse is INTERNAL: it acts equally and oppositely across the
	// joint, so it cannot change the system's total angular momentum. Dropping
	// the parent's share breaks exactly that, which makes this the sharpest
	// available check and one that needs no reference implementation.
	{
		FCreatureTopology Topo = MakeTwoBodyChain(3);
		FCreatureBatchState Batch;
		Batch.Init(Topo, 1);
		// Off-axis pose, so the reduced H block is non-trivial and a dropped
		// angular term cannot cancel by symmetry.
		Batch.SetBodyRot(0, 0, FQuat(FVector(0.3f, 0.8f, 0.5f).GetSafeNormal(), 0.6f));
		Batch.SetJointRelRot(1, 0, FQuat(FVector(0.2f, -0.7f, 0.4f).GetSafeNormal(), 0.5f));

		FCreatureABASolver Solver;
		Solver.RecomputeKinematics(Batch);
		Solver.ComputeArticulatedInertias(Batch);

		// Give the system real motion first, so |L| is large and the conservation
		// check is genuinely RELATIVE. Starting from rest would leave |L0| == 0 and
		// turn the assertion into a loose absolute tolerance that a broken solver
		// could still pass simply by producing small numbers.
		Batch.JointVel[Batch.DOFIndex(0, 0)] = 1.1f;
		Batch.JointVel[Batch.DOFIndex(1, 0)] = -0.6f;
		Batch.JointVel[Batch.DOFIndex(2, 0)] = 0.8f;
		Batch.AngVelZ[Batch.BodyIndex(0, 0)] = 0.9f;
		Solver.RecomputeKinematics(Batch);

		const FVector LBefore = TotalAngularMomentum(Batch, 0);
		const FVector Dir = FVector(0.1f, 0.9f, -0.4f).GetSafeNormal();
		Solver.ApplyBallJointImpulse(Batch, 0, Topo.BodyDOFOffset[1], Dir, 250.0f);
		Solver.RecomputeKinematics(Batch);
		const FVector LAfter = TotalAngularMomentum(Batch, 0);

		const float Scale = FMath::Max((float)LBefore.Size(), KINDA_SMALL_NUMBER);
		const float Drift = (float)(LAfter - LBefore).Size() / Scale;
		AddInfo(FString::Printf(TEXT("F: ball-joint impulse  |L| %.6g -> %.6g  relative change=%.3e"),
			LBefore.Size(), LAfter.Size(), Drift));
		TestTrue(TEXT("F: |L| is large enough for the relative check to mean something"), LBefore.Size() > 100.0f);
		TestTrue(TEXT("F: a ball-joint impulse conserves total angular momentum (reduced-bias fix)"),
			Drift < 1.0e-2f);
		// The impulse itself is large: 250 along a ~30 cm lever is order 7500 of
		// angular impulse, so a dropped parent reaction could not hide inside the
		// tolerance above.
		TestTrue(TEXT("F: the applied impulse actually perturbed the joint"),
			FMath::Abs(Batch.JointVel[Batch.DOFIndex(0, 0)] - 1.1f)
			+ FMath::Abs(Batch.JointVel[Batch.DOFIndex(1, 0)] + 0.6f)
			+ FMath::Abs(Batch.JointVel[Batch.DOFIndex(2, 0)] - 0.8f) > 1.0e-3f);

		// The parent MUST react. Before the fix its share of the reaction was
		// dropped, so this is the direct positive statement of the same fix.
		const int32 RootIdx = Batch.BodyIndex(0, 0);
		const FVector RootW(Batch.AngVelX[RootIdx], Batch.AngVelY[RootIdx], Batch.AngVelZ[RootIdx]);
		AddInfo(FString::Printf(TEXT("F: root angular velocity after the internal impulse: %s"), *RootW.ToString()));
		TestTrue(TEXT("F: the parent body reacts to a child ball joint's impulse"), RootW.Size() > 1e-4f);

		// Same law for a REVOLUTE joint impulse — that branch was always correct,
		// so this is the control that says the test itself is sound.
		FCreatureTopology TopoRev = MakeTwoBodyChain(1);
		FCreatureBatchState BatchRev;
		BatchRev.Init(TopoRev, 1);
		BatchRev.SetBodyRot(0, 0, FQuat(FVector(0.3f, 0.8f, 0.5f).GetSafeNormal(), 0.6f));
		FCreatureABASolver SolverRev;
		SolverRev.RecomputeKinematics(BatchRev);
		SolverRev.ComputeArticulatedInertias(BatchRev);
		const FVector LRevBefore = TotalAngularMomentum(BatchRev, 0);
		SolverRev.ApplyJointImpulse(BatchRev, 0, 0, 250.0f);
		SolverRev.RecomputeKinematics(BatchRev);
		const FVector LRevAfter = TotalAngularMomentum(BatchRev, 0);
		const float DriftRev = (float)(LRevAfter - LRevBefore).Size() / FMath::Max(1.0f, (float)LRevBefore.Size());
		AddInfo(FString::Printf(TEXT("F: revolute control  relative change=%.3e"), DriftRev));
		TestTrue(TEXT("F: control — a revolute joint impulse also conserves angular momentum"),
			DriftRev < 1.0e-2f);
	}

	// =====================================================================
	// G. WELDING A SATURATED JOINT RAISES THE EFFECTIVE MASS AT A CONTACT
	// =====================================================================
	// Entry 024 measured 14.8x on the real rig and then never wired the mechanism
	// into production. This asserts the direction and rough magnitude on a
	// synthetic chain, so the wiring cannot silently become a no-op again --
	// which is exactly what happened once before, when locking the factorization
	// alone reported a clean 1.0x because SolveImpulseResponse redid the
	// reduction itself.
	{
		FCreatureTopology Topo = MakeTwoBodyChain(1);
		// THE AXIS MATTERS, and getting it wrong made the first version of this
		// test report a meaningless 1.00x. The chain runs along +X and the probe
		// pushes along +Z, so the torque it generates about the joint origin is
		// about Y. With the default Z axis that torque is orthogonal to the joint,
		// the joint cannot move under this load at all, and welding it is
		// correctly a no-op -- the test was measuring nothing. Y is the axis this
		// probe actually loads.
		Topo.BodyJointAxisLocal[1] = FVector(0.0f, 1.0f, 0.0f);

		FCreatureBatchState Batch;
		Batch.Init(Topo, 1);
		FCreatureABASolver Solver;
		Solver.RecomputeKinematics(Batch);

		// Probe point out at the child's far end, where a foot would be.
		const FVector Probe = Batch.GetBodyPos(1, 0) + FVector(30.0f, 0.0f, 0.0f);
		const FVector Dir = FVector::UpVector;

		Solver.ComputeArticulatedInertias(Batch);
		const float FreeAlong = (float)FVector::DotProduct(
			Solver.ImpulseResponseAtPoint(Batch, 0, 1, Probe, Dir), Dir);

		TArray<uint8> Locked;
		Locked.SetNumZeroed(Topo.NumBodies * Batch.GetNumEnvs());
		Locked[1 * Batch.GetNumEnvs() + 0] = 1;
		Solver.ComputeArticulatedInertias(Batch, Locked.GetData(), Batch.GetNumEnvs());
		const float WeldAlong = (float)FVector::DotProduct(
			Solver.ImpulseResponseAtPoint(Batch, 0, 1, Probe, Dir), Dir);

		const float FreeMass = FreeAlong > UE_SMALL_NUMBER ? 1.0f / FreeAlong : 0.0f;
		const float WeldMass = WeldAlong > UE_SMALL_NUMBER ? 1.0f / WeldAlong : 0.0f;
		AddInfo(FString::Printf(TEXT("G: effective mass at the probe  free=%.3f  welded=%.3f  ratio=%.2fx"),
			FreeMass, WeldMass, FreeMass > 0.0f ? WeldMass / FreeMass : 0.0f));
		TestTrue(TEXT("G: welding a saturated joint RAISES the effective mass at a contact"),
			WeldMass > FreeMass * 1.05f);

		// And the lock must be per-env addressable, or a batched run would weld
		// env 0's saturated joints in all of them.
		TestEqual(TEXT("G: lock array is sized per body per env"),
			Locked.Num(), Topo.NumBodies * Batch.GetNumEnvs());
	}

	// =====================================================================
	// H. THE SATURATED-JOINT LOCK BUILDER ONLY LOCKS JOINTS BEING PUSHED IN
	// =====================================================================
	// Welding is a one-sided approximation on a one-sided constraint. Locking a
	// joint that is LEAVING its stop would remove a direction it is legitimately
	// free to move in, so the builder must not.
	{
		FCreatureTopology Topo = MakeTwoBodyChain(1);
		// Authored range [0, 90] deg, and the joint sits right at the lower stop.
		Topo.DOFHasMuscleCurve[0] = 1;
		Topo.DOFRangeMinDeg[0] = 0.0f;
		Topo.DOFRangeMaxDeg[0] = 90.0f;

		FCreatureBatchState Batch;
		Batch.Init(Topo, 1);
		FCreatureABASolver Solver;
		TArray<uint8> Locked;

		const int32 DOFIdx = Batch.DOFIndex(0, 0);
		Batch.JointPos[DOFIdx] = 0.0f;

		Batch.JointVel[DOFIdx] = -1.0f; // driving INTO the lower stop
		Solver.BuildSaturatedJointLocks(Batch, 1.0f, Locked);
		TestEqual(TEXT("H: a joint driven into its stop is locked"), (int32)Locked[1 * Batch.GetNumEnvs()], 1);

		Batch.JointVel[DOFIdx] = +1.0f; // leaving the stop, back into range
		Solver.BuildSaturatedJointLocks(Batch, 1.0f, Locked);
		TestEqual(TEXT("H: a joint leaving its stop is NOT locked"), (int32)Locked[1 * Batch.GetNumEnvs()], 0);

		// Mid-range with no approach: never locked.
		Batch.JointPos[DOFIdx] = FMath::DegreesToRadians(45.0f);
		Batch.JointVel[DOFIdx] = -1.0f;
		Solver.BuildSaturatedJointLocks(Batch, 1.0f, Locked);
		TestEqual(TEXT("H: a mid-range joint is never locked"), (int32)Locked[1 * Batch.GetNumEnvs()], 0);

		// A joint with no authored range must never be locked, matching
		// ClampJointLimits' own no-op in that case.
		FCreatureTopology TopoNoRange = MakeTwoBodyChain(1);
		FCreatureBatchState BatchNoRange;
		BatchNoRange.Init(TopoNoRange, 1);
		BatchNoRange.JointVel[BatchNoRange.DOFIndex(0, 0)] = -100.0f;
		Solver.BuildSaturatedJointLocks(BatchNoRange, 1.0f, Locked);
		TestEqual(TEXT("H: a joint with no authored range is never locked"),
			(int32)Locked[1 * BatchNoRange.GetNumEnvs()], 0);
	}

	// =====================================================================
	// I. GLOBAL SOLVE vs PER-ROW SOLVE — they must agree on a settled contact
	// =====================================================================
	// Not a bit-for-bit comparison: these are different algorithms on the same
	// system, and the global one converges further. What must hold is that they
	// agree on the physically determined answer -- a body resting on the ground
	// settles at the same height either way. A disagreement here means the
	// assembled Jacobian does not describe the same constraints the per-row path
	// solves, which is the failure mode worth catching.
	{
		auto RunDrop = [&](bool bGlobal, float& OutFinalZ, float& OutFinalSpeed, bool& bOutFinite)
		{
			FCreatureTopology Topo;
			TArray<int32> Parent = { 0 };
			TArray<int32> DOFCount = { 0 };
			TArray<int32> Limb = { INDEX_NONE };
			Topo.NumLimbs = 0;
			Topo.Build(Parent, DOFCount, Limb);
			Topo.BodyMass[0] = 5.0f;
			Topo.BodyInertiaDiagLocal[0] = FVector(50.0f, 50.0f, 50.0f);

			FImpulseContactParams Params;
			Params.GroundZ = 0.0f;
			Params.ContactHertz = 30.0f;
			Params.DampingRatio = 10.0f;
			Params.Slop = 0.1f;
			Params.Iterations = 8;
			Params.GlobalIterations = 32;
			Params.RelaxIterations = 0;
			Params.bUseGlobalSolve = bGlobal;
			// Off for the comparison so the two paths are solving an identical
			// system rather than differing by regularization too.
			Params.Cfm = 0.0f;
			Params.Relaxation = 1.0f;
			FImpulseContactCache Cache;

			TArray<FContactPointDef> Points;
			Points.Add({ 0, FVector(0.0f, 0.0f, -10.0f), TEXT("Foot"), 0 });

			FCreatureBatchState Batch;
			Batch.Init(Topo, 1);
			Batch.SetBodyPos(0, 0, FVector(0.0f, 0.0f, 50.0f));
			FCreatureABASolver Solver;

			bOutFinite = true;
			for (int32 i = 0; i < 480; ++i)
			{
				Solver.Step(Batch, Dt, FVector(0.0f, 0.0f, -980.0f));
				ResolveGroundContactImpulses(Batch, Topo, Points, Params, Solver, Dt, Cache);
				if (Batch.GetBodyPos(0, 0).ContainsNaN()) { bOutFinite = false; break; }
			}
			const int32 Idx = Batch.BodyIndex(0, 0);
			OutFinalZ = (float)Batch.GetBodyPos(0, 0).Z - 10.0f; // contact-point height
			OutFinalSpeed = FVector(Batch.LinVelX[Idx], Batch.LinVelY[Idx], Batch.LinVelZ[Idx]).Size();
		};

		float ZPerRow = 0.0f, SpeedPerRow = 0.0f;
		float ZGlobal = 0.0f, SpeedGlobal = 0.0f;
		bool bFinitePerRow = false, bFiniteGlobal = false;
		RunDrop(false, ZPerRow, SpeedPerRow, bFinitePerRow);
		RunDrop(true, ZGlobal, SpeedGlobal, bFiniteGlobal);

		AddInfo(FString::Printf(TEXT("I: per-row  contactZ=%+.4f  speed=%.4f  finite=%s"),
			ZPerRow, SpeedPerRow, bFinitePerRow ? TEXT("yes") : TEXT("NO")));
		AddInfo(FString::Printf(TEXT("I: global   contactZ=%+.4f  speed=%.4f  finite=%s"),
			ZGlobal, SpeedGlobal, bFiniteGlobal ? TEXT("yes") : TEXT("NO")));

		TestTrue(TEXT("I: per-row solve stays finite"), bFinitePerRow);
		TestTrue(TEXT("I: global solve stays finite"), bFiniteGlobal);
		TestTrue(TEXT("I: global solve settles the contact point on the ground"),
			FMath::Abs(ZGlobal) < 2.0f);
		TestTrue(TEXT("I: global solve comes to rest"), SpeedGlobal < 5.0f);
		TestTrue(TEXT("I: both solvers agree on the resting height to within a unit"),
			FMath::Abs(ZGlobal - ZPerRow) < 1.0f);
	}

	// =====================================================================
	// J. THE GLOBAL SOLVE'S RESIDUAL ACTUALLY FALLS
	// =====================================================================
	// The observable the per-row path could never provide. If the assembled
	// matrix or the sweep were wrong, the residual would plateau or grow rather
	// than decrease -- and "is it converging?" is the question entries 016-024
	// could not answer for want of exactly this number.
	{
		FCreatureTopology Topo;
		TArray<int32> Parent = { 0 };
		TArray<int32> DOFCount = { 0 };
		TArray<int32> Limb = { INDEX_NONE };
		Topo.NumLimbs = 0;
		Topo.Build(Parent, DOFCount, Limb);
		Topo.BodyMass[0] = 5.0f;
		Topo.BodyInertiaDiagLocal[0] = FVector(50.0f, 50.0f, 50.0f);

		FImpulseContactParams Params;
		Params.GroundZ = 0.0f;
		Params.Slop = 0.1f;
		Params.GlobalIterations = 24;
		Params.bUseGlobalSolve = true;
		FImpulseContactCache Cache;

		// Four points, so several rows share one body and genuinely couple.
		TArray<FContactPointDef> Points;
		Points.Add({ 0, FVector(+8.0f, +8.0f, -10.0f), TEXT("P0"), 0 });
		Points.Add({ 0, FVector(-8.0f, +8.0f, -10.0f), TEXT("P1"), 0 });
		Points.Add({ 0, FVector(+8.0f, -8.0f, -10.0f), TEXT("P2"), 0 });
		Points.Add({ 0, FVector(-8.0f, -8.0f, -10.0f), TEXT("P3"), 0 });

		FCreatureBatchState Batch;
		Batch.Init(Topo, 1);
		// Start already penetrating and moving down, so every row is active.
		Batch.SetBodyPos(0, 0, FVector(0.0f, 0.0f, 8.0f));
		Batch.LinVelZ[Batch.BodyIndex(0, 0)] = -200.0f;

		FCreatureABASolver Solver;
		FIterationDebugLog Log;
		ResolveGroundContactImpulses(Batch, Topo, Points, Params, Solver, Dt, Cache,
			nullptr, nullptr, TArray<FLimbPairDef>(), nullptr, &Log);

		AddInfo(FString::Printf(TEXT("J: assembled %d rows, %d residual samples (1 pre-sweep + %d sweeps)"),
			Log.GlobalNumRows, Log.GlobalResidualPerIteration.Num(), Params.GlobalIterations));
		TestTrue(TEXT("J: the debug log recorded a residual series"),
			Log.GlobalResidualPerIteration.Num() >= 4);
		TestEqual(TEXT("J: the series is one pre-sweep sample plus one per sweep"),
			Log.GlobalResidualPerIteration.Num(), Params.GlobalIterations + 1);
		// 4 points x (1 normal + 2 friction) = 12 rows.
		TestEqual(TEXT("J: four single-sphere contact points assemble 12 rows"), Log.GlobalNumRows, 12);

		if (Log.GlobalResidualPerIteration.Num() >= 4)
		{
			const float First = Log.GlobalResidualPerIteration[0];
			const float Last = Log.GlobalResidualPerIteration.Last();
			AddInfo(FString::Printf(TEXT("J: residual  first=%.6g  last=%.6g  (%.1f%% reduction)"),
				First, Last, 100.0f * (1.0f - Last / FMath::Max(First, KINDA_SMALL_NUMBER))));
			TestTrue(TEXT("J: the constraint residual decreases across sweeps"), Last < First);
			TestTrue(TEXT("J: the residual falls substantially, not marginally"), Last < First * 0.5f);
			TestTrue(TEXT("J: the residual is finite throughout"), FMath::IsFinite(Last));
			// Monotone non-increasing is NOT asserted: four coplanar points on one
			// rigid body make A rank-deficient (the normal rows are redundant), and
			// projected SOR on a singular system is allowed to wander between
			// equally valid impulse distributions while the BODY's motion converges.
			// That redundancy is also why this case does not reach ~0 in 24 sweeps.
		}

		// A well-conditioned case -- ONE contact point, so A has full rank -- must
		// converge properly rather than merely improve. This separates "the solver
		// works" from "the test rig was degenerate".
		{
			FCreatureBatchState B2;
			B2.Init(Topo, 1);
			B2.SetBodyPos(0, 0, FVector(0.0f, 0.0f, 8.0f));
			B2.LinVelZ[B2.BodyIndex(0, 0)] = -200.0f;

			TArray<FContactPointDef> OnePoint;
			OnePoint.Add({ 0, FVector(0.0f, 0.0f, -10.0f), TEXT("Solo"), 0 });

			FImpulseContactCache Cache2;
			FCreatureABASolver Solver2;
			FIterationDebugLog Log2;
			ResolveGroundContactImpulses(B2, Topo, OnePoint, Params, Solver2, Dt, Cache2,
				nullptr, nullptr, TArray<FLimbPairDef>(), nullptr, &Log2);

			TestEqual(TEXT("J: one contact point assembles 3 rows (normal + 2 friction)"), Log2.GlobalNumRows, 3);
			if (Log2.GlobalResidualPerIteration.Num() >= 4)
			{
				const TArray<float>& R2 = Log2.GlobalResidualPerIteration;
				const float Initial = R2[0];
				const float Final = R2.Last();
				const float SecondToLast = R2[R2.Num() - 2];
				AddInfo(FString::Printf(TEXT("J: full-rank residual  initial=%.6g  final=%.6g  (%.1f%% reduction)"),
					Initial, Final, 100.0f * (1.0f - Final / FMath::Max(Initial, KINDA_SMALL_NUMBER))));

				// NOT "converges to zero". A SOFT constraint is compliant by
				// construction: the fixed point of
				//   Delta = -MassScale*(Cdot+Bias)*InvDiag - ImpulseScale*Lambda
				// has Delta == 0 at (Cdot+Bias) = -ImpulseScale*Lambda/(MassScale*InvDiag),
				// which is NONZERO and proportional to the accumulated impulse. That
				// residual floor IS the compliance -- it is the same regularization
				// MuJoCo's R > 0 provides, and driving it to zero would be the hard
				// constraint that entries 015-018 established is the unstable regime.
				// So the claims are: a large reduction, and a STATIONARY tail.
				TestTrue(TEXT("J: a full-rank system reduces the residual by >80%"),
					Final < Initial * 0.2f);
				TestTrue(TEXT("J: the residual has stopped moving by the last sweep (converged, not drifting)"),
					FMath::Abs(Final - SecondToLast) < FMath::Max(1.0e-4f, Final * 1.0e-3f));
				TestTrue(TEXT("J: the residual is finite"), FMath::IsFinite(Final));
			}
		}
	}

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
