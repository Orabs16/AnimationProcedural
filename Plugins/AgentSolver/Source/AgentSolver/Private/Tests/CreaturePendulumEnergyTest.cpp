// Validates the ABA solver's core dynamics against a classic physics sanity
// check: a 2-body (anchor + one revolute-jointed rod) simple pendulum,
// released from rest at a large displacement angle, should conserve total
// mechanical energy (KE+PE) under gravity alone — semi-implicit Euler gives
// bounded oscillation around the true value, not a secular drift. This is
// the pendulum validation called out in the project roadmap (do this before
// trusting the solver on the full multi-limb topology).

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "CreatureBatchSolver.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace CreaturePendulumEnergyTest
{
	// A uniform rod, pivoting about world Y (swings in the X-Z plane),
	// hanging straight down at JointAngle=0.
	constexpr float RodLength = 100.0f;
	constexpr float RodMass = 5.0f;
	const FVector Gravity(0.0f, 0.0f, -980.0f);

	static FCreatureTopology BuildPendulumTopology()
	{
		FCreatureTopology Topo;

		TArray<int32> BodyParent = { 0, 0 };
		TArray<int32> BodyDOFCount = { 0, 1 };
		TArray<int32> BodyLimbIndex = { INDEX_NONE, 0 };

		Topo.NumLimbs = 1;
		Topo.Build(BodyParent, BodyDOFCount, BodyLimbIndex);

		// Body 0: anchor. This solver's root is always a free-floating 6-DOF
		// base (see CreatureBatchSolver.h) — there's no literal "fixed base"
		// mode — so a near-fixed anchor is approximated with overwhelming
		// mass/inertia relative to the pendulum (checked below via
		// MaxAnchorDisplacement, not just assumed).
		Topo.BodyMass[0] = 1.0e5f;
		Topo.BodyInertiaDiagLocal[0] = FVector(1.0e7f, 1.0e7f, 1.0e7f);

		// Body 1: the rod. Pivot coincides with the anchor's origin; CoM
		// hangs RodLength below it at rest (JointAngle=0).
		Topo.BodyJointAxisLocal[1] = FVector(0.0f, 1.0f, 0.0f);
		Topo.BodyJointOffsetInParent[1] = FVector::ZeroVector;
		Topo.BodyLocalCoMOffset[1] = FVector(0.0f, 0.0f, -RodLength);
		Topo.BodyMass[1] = RodMass;
		const float IPerp = (1.0f / 12.0f) * RodMass * RodLength * RodLength; // thin rod, axis perpendicular to its length
		Topo.BodyInertiaDiagLocal[1] = FVector(IPerp, IPerp, 0.05f * IPerp); // ~0 (not exactly, for numerical safety) about its own length

		return Topo;
	}

	static float ComputeTotalEnergy(const FCreatureBatchState& Batch, const FCreatureTopology& Topo, int32 Env)
	{
		float TotalEnergy = 0.0f;
		// Body 0 (the anchor) is excluded deliberately: it's a modeling trick
		// (overwhelming mass/inertia standing in for a fixed mount, see
		// BuildPendulumTopology), not a real mass whose PE should count. Its
		// mass is so large that even the tiny residual position drift left
		// over from imperfect gravity cancellation gets amplified into a
		// spurious PE number many orders of magnitude bigger than the
		// pendulum's actual energy — measured, not assumed: with the anchor
		// included this test previously reported energy blowing up to -1.6e8
		// while JointAngle/JointVel traced a perfectly clean, bounded
		// oscillation the whole time. What we actually want to validate is
		// the pendulum body's own mechanical energy.
		for (int32 Body = 1; Body < Topo.NumBodies; ++Body)
		{
			const int32 Idx = Batch.BodyIndex(Body, Env);
			const FQuat Rot = Batch.GetBodyRot(Body, Env);
			const FVector Pos = Batch.GetBodyPos(Body, Env);
			const FVector AngVel(Batch.AngVelX[Idx], Batch.AngVelY[Idx], Batch.AngVelZ[Idx]);
			const FVector LinVel(Batch.LinVelX[Idx], Batch.LinVelY[Idx], Batch.LinVelZ[Idx]);

			// KE = 0.5*m*|v_com|^2 + 0.5*w.(I_com*w) — same inertia-about-CoM
			// interpretation the solver itself uses (CreatureBatchSolver.h's
			// "own rigid-body inertia" pass), so a real dynamics bug shows up
			// here rather than being masked by a different convention.
			const FVector CoMOffsetWorld = Rot.RotateVector(Topo.BodyLocalCoMOffset[Body]);
			const FVector CoMPos = Pos + CoMOffsetWorld;
			const FVector CoMVel = LinVel + FVector::CrossProduct(AngVel, CoMOffsetWorld);

			const FMat3 RotM = FMat3::FromRotation(Rot);
			const FMat3 IComWorld = RotM * FMat3::Diagonal(Topo.BodyInertiaDiagLocal[Body]) * RotM.Transpose();

			const float Mass = Topo.BodyMass[Body];
			const float KE = 0.5f * Mass * static_cast<float>(CoMVel.SizeSquared())
				+ 0.5f * static_cast<float>(FVector::DotProduct(AngVel, IComWorld * AngVel));
			const float PE = -Mass * static_cast<float>(FVector::DotProduct(Gravity, CoMPos));

			TotalEnergy += KE + PE;
		}
		return TotalEnergy;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCreaturePendulumEnergyTest, "AgentSolver.PendulumEnergyConservation", EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FCreaturePendulumEnergyTest::RunTest(const FString& Parameters)
{
	using namespace CreaturePendulumEnergyTest;

	const FCreatureTopology Topo = BuildPendulumTopology();
	constexpr int32 NumEnvs = 1;
	constexpr float Dt = 1.0f / 240.0f;
	constexpr int32 NumSteps = 2000; // ~8.3s — several swing periods at this length/mass

	FCreatureBatchState Batch;
	Batch.Init(Topo, NumEnvs);

	// Release from rest at 90 degrees off the stable hang position.
	Batch.JointPos[Batch.DOFIndex(0, 0)] = FMath::DegreesToRadians(90.0f);

	// A huge mass/inertia only resists ACCELERATION FROM EXTERNAL FORCES
	// (the pendulum's reaction) — it does nothing against gravity itself,
	// which accelerates every mass equally regardless of size (equivalence
	// principle). Without this, the "anchor" just free-falls at -980 like
	// anything else. Applying a constant counter-force once, uncleared,
	// exactly cancels gravity's pull on body 0 every step (both terms are
	// constant), simulating a fixed mount's support reaction.
	Batch.ApplyForceAtPoint(0, 0, -Topo.BodyMass[0] * Gravity, Batch.GetBodyPos(0, 0));

	FCreatureABASolver Solver;

	// Energy deviation is measured against this problem's characteristic
	// energy scale (roughly the KE at the bottom of the swing), not against
	// the initial total energy — E0 depends on the arbitrary PE reference
	// height and can land near zero for some release angles, which would
	// make a "% of E0" tolerance meaningless.
	const float EnergyScale = RodMass * FMath::Abs(Gravity.Z) * RodLength;

	float MinEnergy = TNumericLimits<float>::Max();
	float MaxEnergy = TNumericLimits<float>::Lowest();
	float MaxAnchorDisplacement = 0.0f;
	bool bAnyNonFinite = false;
	for (int32 Step = 0; Step < NumSteps; ++Step)
	{
		Solver.Step(Batch, Dt, Gravity);

		const float E = ComputeTotalEnergy(Batch, Topo, 0);
		if (!FMath::IsFinite(E)) { bAnyNonFinite = true; break; }
		MinEnergy = FMath::Min(MinEnergy, E);
		MaxEnergy = FMath::Max(MaxEnergy, E);

		MaxAnchorDisplacement = FMath::Max(MaxAnchorDisplacement, static_cast<float>(Batch.GetBodyPos(0, 0).Size()));
	}

	AddInfo(FString::Printf(TEXT("EnergyScale=%.1f MinE=%.2f MaxE=%.2f (spread %.4f%% of EnergyScale) MaxAnchorDisplacement=%.4f"),
		EnergyScale, MinEnergy, MaxEnergy, 100.0f * (MaxEnergy - MinEnergy) / EnergyScale, MaxAnchorDisplacement));

	TestFalse(TEXT("No NaN/Inf energy over the run"), bAnyNonFinite);
	// Semi-implicit Euler doesn't conserve energy exactly even for a correct
	// solver — it oscillates within a bounded band around the true value.
	// 5% comfortably covers that expected band (measured: ~3.3% over ~8s /
	// several swing periods at Dt=1/240) while still catching a real bug,
	// which would show up as unbounded/secular drift, not a bounded spread.
	TestTrue(TEXT("Energy stays within 5% of characteristic scale (bounded oscillation, not secular drift)"),
		(MaxEnergy - MinEnergy) < 0.05f * EnergyScale);
	// Sanity check on the fixed-anchor approximation itself, not a strict
	// physical requirement — some drift is expected from a finite (if huge)
	// mass ratio absorbing the pendulum's reaction force/torque.
	TestTrue(TEXT("Anchor stays effectively fixed (<5 units displacement, vs. RodLength=100)"), MaxAnchorDisplacement < 5.0f);

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
