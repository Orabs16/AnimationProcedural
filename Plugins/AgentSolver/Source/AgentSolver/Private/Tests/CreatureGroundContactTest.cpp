// Validates CreatureGroundContact.h: a synthetic drop test with precise
// physical assertions (does a falling body settle on the ground plane
// without sinking through or bouncing forever), plus a structural check
// that BuildMutoContactPoints derives the right 18 points across the real
// Muto topology's 6 ground-contact limbs (F/B/Hips — not M).

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "CreatureGroundContact.h"
#include "CreatureBatchSolver.h"

#if WITH_EDITOR
#include "MutoTopology.h" // for MutoTopology::BuildMutoTopology below (CreatureGroundContact.h no longer pulls this in transitively)
#endif

#if WITH_DEV_AUTOMATION_TESTS

using namespace CreatureGroundContact;

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCreatureGroundContactDropTest, "AgentSolver.GroundContact.Drop", EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FCreatureGroundContactDropTest::RunTest(const FString& Parameters)
{
	// Single free-floating body (no limbs) with one contact point offset
	// below its own reference point, like a foot hanging under a torso.
	// Isolates the contact primitive itself from limb-chain dynamics.
	FCreatureTopology Topo;
	TArray<int32> BodyParent = { 0 };
	TArray<int32> BodyDOFCount = { 0 };
	TArray<int32> BodyLimbIndex = { INDEX_NONE };
	Topo.NumLimbs = 0;
	Topo.Build(BodyParent, BodyDOFCount, BodyLimbIndex);
	Topo.BodyMass[0] = 5.0f;
	Topo.BodyInertiaDiagLocal[0] = FVector(50.0f, 50.0f, 50.0f);

	constexpr float FootOffset = 10.0f; // contact point sits 10 units below the body's own reference point
	constexpr float DropHeight = 50.0f;
	const FVector Gravity(0.0f, 0.0f, -980.0f);

	// Velocity-level contact. Note there is no stiffness to derive from the
	// body's mass here: the solver computes the articulated effective mass
	// itself, so the same Hertz/DampingRatio pair applies to a 5 kg test body
	// and to the 6170 kg creature alike. That mass-invariance is the main reason
	// the penalty model was retired (SOLVER_DEBUG_LOG.md entry 021).
	FImpulseContactParams Params;
	Params.GroundZ = 0.0f;
	Params.ContactHertz = 30.0f;
	Params.DampingRatio = 10.0f;
	Params.Slop = 0.1f; // small: this test checks resting height to ~1 unit
	Params.FrictionCoefficient = 0.8f;
	Params.Iterations = 8;
	Params.RelaxIterations = 0; // see FImpulseContactParams::RelaxIterations — ordering-dependent
	FImpulseContactCache Cache;

	TArray<FContactPointDef> Points;
	Points.Add({ 0, FVector(0.0f, 0.0f, -FootOffset), TEXT("Foot"), 0 });

	FCreatureBatchState Batch;
	constexpr int32 NumEnvs = 1;
	Batch.Init(Topo, NumEnvs);
	Batch.SetBodyPos(0, 0, FVector(0.0f, 0.0f, DropHeight));

	FCreatureABASolver Solver;
	constexpr float Dt = 1.0f / 240.0f;
	constexpr int32 NumSteps = 480; // 2s — well past the ~0.32s fall + settle time at these constants

	bool bAnyNonFinite = false;
	float FinalPointZ = 0.0f;
	float FinalSpeed = 0.0f;
	bool bSettledWithinBand = true; // checked over the tail of the run, not just the last step
	for (int32 Step = 0; Step < NumSteps; ++Step)
	{
		Batch.ClearExternalForces(0);
		// Impulse contact resolves AFTER integration — it corrects the velocity
		// Step() just produced, rather than staging a force for it to consume.
		Solver.Step(Batch, Dt, Gravity);
		ResolveGroundContactImpulses(Batch, Topo, Points, Params, Solver, Dt, Cache);

		const FVector Pos = Batch.GetBodyPos(0, 0);
		const int32 Idx = Batch.BodyIndex(0, 0);
		const FVector LinVel(Batch.LinVelX[Idx], Batch.LinVelY[Idx], Batch.LinVelZ[Idx]);

		if (!FMath::IsFinite(Pos.X) || !FMath::IsFinite(Pos.Y) || !FMath::IsFinite(Pos.Z) || !FMath::IsFinite(LinVel.Size()))
		{
			bAnyNonFinite = true;
			break;
		}

		FinalPointZ = static_cast<float>(Pos.Z) - FootOffset;
		FinalSpeed = static_cast<float>(LinVel.Size());

		// Over the last 10% of the run, the contact point should stay close
		// to the ground and moving slowly — settled, not still falling,
		// sinking through, or bouncing.
		if (Step > NumSteps * 9 / 10)
		{
			if (FMath::Abs(FinalPointZ - Params.GroundZ) > 3.0f || FinalSpeed > 5.0f)
			{
				bSettledWithinBand = false;
			}
		}
	}

	AddInfo(FString::Printf(TEXT("Final contact point Z=%.4f (GroundZ=%.1f) speed=%.4f"), FinalPointZ, Params.GroundZ, FinalSpeed));

	TestFalse(TEXT("No NaN/Inf over the run"), bAnyNonFinite);
	TestTrue(TEXT("Settles near the ground plane without sinking through or bouncing indefinitely"), bSettledWithinBand);

	return true;
}

#if WITH_EDITOR

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCreatureGroundContactMutoTest, "AgentSolver.GroundContact.MutoWiring", EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FCreatureGroundContactMutoTest::RunTest(const FString& Parameters)
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
	for (const FString& Warning : Warnings)
	{
		AddError(FString::Printf(TEXT("BuildMutoTopology warning: %s"), *Warning));
	}

	const TArray<FContactPointDef> Points = BuildMutoContactPoints(Topo, MassAsset, BodyDebugNames);

	// 10, not the full 12 CanTouchGround=true bones in the authored
	// MassProfile_Muto (Elbow3+Hand on F and B (2x2x2=8) plus Feet+FeetTip on
	// Hips (2x2=4)): FeetTip_L/_R are flagged CanTouchGround but, per the
	// 2026-08-11 Bone/Child semantics correction (see MutoTopology.h), Tip
	// bones are no longer their own ABA body (no muscle is ever authored to
	// independently rotate them) — BuildMutoContactPoints only iterates real
	// bodies, so those 2 flagged-but-nonexistent points are silently
	// skipped, leaving 8 (Elbow3+Hand) + 2 (Feet_L/_R only) = 10.
	TestEqual(TEXT("10 contact points (CanTouchGround-flagged REAL bodies across 6 ground-contact limbs)"), Points.Num(), 10);

	TSet<int32> LimbIndicesSeen;
	bool bAllBodyIndicesValid = true;
	for (const FContactPointDef& Point : Points)
	{
		LimbIndicesSeen.Add(Point.LimbIndex);
		if (Point.BodyIndex <= 0 || Point.BodyIndex >= Topo.NumBodies)
		{
			AddError(FString::Printf(TEXT("Contact point '%s' has out-of-range BodyIndex %d"), *Point.DebugName.ToString(), Point.BodyIndex));
			bAllBodyIndicesValid = false;
		}
	}
	TestTrue(TEXT("All contact point body indices are valid non-root bodies"), bAllBodyIndicesValid);
	// F_L=0, F_R=1, B_L=2, B_R=3, M_L=4, M_R=5, Hips_L=6, Hips_R=7 — M excluded.
	TestEqual(TEXT("6 distinct grounded limbs represented"), LimbIndicesSeen.Num(), 6);
	TestFalse(TEXT("M_L (limb index 4) excluded"), LimbIndicesSeen.Contains(4));
	TestFalse(TEXT("M_R (limb index 5) excluded"), LimbIndicesSeen.Contains(5));

	// Smoke test only (per plan: not a "does it stand up" test, which needs
	// proper initial placement) — just confirm the full real topology with
	// all 18 points active doesn't NaN over a few steps.
	FCreatureBatchState Batch;
	Batch.Init(Topo, 1);
	FCreatureABASolver Solver;

	// Identical settings to the drop test above, deliberately: the impulse model
	// derives its own effective mass, so nothing needs rescaling between a 5 kg
	// synthetic body and the real multi-tonne rig. Under the old penalty model
	// this block had to carry its own hand-scaled SpringK/DamperK and a comment
	// explaining the derivation — that whole class of per-rig retuning is what
	// retiring it removed.
	FImpulseContactParams Params;
	Params.ContactHertz = 30.0f;
	Params.DampingRatio = 10.0f;
	Params.FrictionCoefficient = 0.8f;
	Params.Iterations = 8;
	Params.RelaxIterations = 0; // see FImpulseContactParams::RelaxIterations — ordering-dependent
	FImpulseContactCache Cache;
	constexpr float Dt = 1.0f / 240.0f;

	bool bAnyNonFinite = false;
	for (int32 Step = 0; Step < 10 && !bAnyNonFinite; ++Step)
	{
		Batch.ClearExternalForces(0);
		Solver.Step(Batch, Dt);
		ResolveGroundContactImpulses(Batch, Topo, Points, Params, Solver, Dt, Cache);

		for (int32 Body = 0; Body < Topo.NumBodies; ++Body)
		{
			const FVector Pos = Batch.GetBodyPos(Body, 0);
			if (!FMath::IsFinite(Pos.X) || !FMath::IsFinite(Pos.Y) || !FMath::IsFinite(Pos.Z))
			{
				AddError(FString::Printf(TEXT("Non-finite position at step %d, body %d"), Step, Body));
				bAnyNonFinite = true;
				break;
			}
		}
	}
	TestFalse(TEXT("No NaN/Inf over a few steps with contact forces active on the real topology"), bAnyNonFinite);

	return true;
}

#endif // WITH_EDITOR
#endif // WITH_DEV_AUTOMATION_TESTS
