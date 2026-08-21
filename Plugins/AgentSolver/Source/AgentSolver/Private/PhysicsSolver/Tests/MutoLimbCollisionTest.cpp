// Validates the limb-vs-limb collision addition to CreatureGroundContact.h:
// (1) BuildMutoLimbCollisionPairs never pairs two bodies on the SAME limb and
// never pairs a body with no authored collision radius, and (2) two
// different-limb bodies made to overlap actually separate under
// ResolveGroundContactImpulses's new limb-pair rows, without leaking total
// system momentum -- the latter specifically exercises the superposition
// math in FCreatureABASolver::PairImpulseResponseAtPoint/
// ApplyPairImpulseAtPoints, since a sign or arm error there would show up as
// a momentum leak rather than a crash.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "PhysicsSolver/CreatureGroundContact.h"
#include "PhysicsSolver/CreatureBatchSolver.h"

#if WITH_DEV_AUTOMATION_TESTS

using namespace CreatureGroundContact;

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLimbCollisionPairBuildTest, "AgentSolver.LimbCollision.SameLimbExcluded", EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FLimbCollisionPairBuildTest::RunTest(const FString& Parameters)
{
	// Root (body 0) + 4 limb bodies: 1 and 2 share limb 0 (e.g. two bones on
	// the same leg), 3 is alone on limb 1, 4 is on limb 2 but has NO authored
	// collision radius (e.g. a shoulder/upper-arm segment never given one).
	FCreatureTopology Topo;
	TArray<int32> BodyParent = { 0, 0, 0, 0, 0 };
	TArray<int32> BodyDOFCount = { 0, 1, 1, 1, 1 };
	TArray<int32> BodyLimbIndex = { INDEX_NONE, 0, 0, 1, 2 };
	Topo.NumLimbs = 3;
	Topo.Build(BodyParent, BodyDOFCount, BodyLimbIndex);
	Topo.BodyRadius[1] = 2.0f;
	Topo.BodyRadius[2] = 2.0f;
	Topo.BodyRadius[3] = 2.0f;
	Topo.BodyRadius[4] = 0.0f; // deliberately unauthored

	const TArray<FLimbPairDef> Pairs = BuildMutoLimbCollisionPairs(Topo);

	auto ContainsPair = [&Pairs](int32 A, int32 B) -> bool
	{
		for (const FLimbPairDef& P : Pairs)
		{
			if ((P.BodyA == A && P.BodyB == B) || (P.BodyA == B && P.BodyB == A))
			{
				return true;
			}
		}
		return false;
	};

	TestFalse(TEXT("Bodies 1 and 2 (same limb) are never paired"), ContainsPair(1, 2));
	TestFalse(TEXT("Body 4 (no authored radius) is never paired with 1"), ContainsPair(1, 4));
	TestFalse(TEXT("Body 4 (no authored radius) is never paired with 2"), ContainsPair(2, 4));
	TestFalse(TEXT("Body 4 (no authored radius) is never paired with 3"), ContainsPair(3, 4));
	TestTrue(TEXT("Bodies 1 and 3 (different limbs, both radius>0) are paired"), ContainsPair(1, 3));
	TestTrue(TEXT("Bodies 2 and 3 (different limbs, both radius>0) are paired"), ContainsPair(2, 3));
	TestEqual(TEXT("Exactly the 2 valid cross-limb pairs were built"), Pairs.Num(), 2);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLimbSpineCollisionPairBuildTest, "AgentSolver.LimbCollision.SpineExcludedFromPairs", EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FLimbSpineCollisionPairBuildTest::RunTest(const FString& Parameters)
{
	// LIMB-VS-LIMB ONLY, deliberately (see BuildMutoLimbCollisionPairs'
	// comment for why this was reverted 2026-08-16 after briefly including
	// the spine: pairing every limb against the newly-articulated spine
	// took the candidate count from ~503 to 701, all sharing the same
	// 8-iteration solver budget, and reproducibly blew up a passive ragdoll
	// drop). Spine bodies (BodyLimbIndex==INDEX_NONE) must never be
	// candidates AT ALL, even with a real authored radius -- not just
	// excluded from adjacency the way same-limb bodies are.
	//
	// Chain: Pelvis(0) -> Back1(1) -> Head1(2) -> FShoulder(3, mount, limb 0) -> FElbow(4, limb 0); BShoulder(5, limb 1).
	FCreatureTopology Topo;
	TArray<int32> BodyParent = { 0, 0, 1, 2, 3, 0 };
	TArray<int32> BodyDOFCount = { 0, 3, 3, 3, 1, 3 };
	TArray<int32> BodyLimbIndex = { INDEX_NONE, INDEX_NONE, INDEX_NONE, 0, 0, 1 };
	Topo.NumLimbs = 2;
	Topo.Build(BodyParent, BodyDOFCount, BodyLimbIndex);
	for (int32 Body = 1; Body <= 5; ++Body)
	{
		Topo.BodyRadius[Body] = 2.0f; // every body has real authored radius, spine included
	}

	const TArray<FLimbPairDef> Pairs = BuildMutoLimbCollisionPairs(Topo);

	auto ContainsPair = [&Pairs](int32 A, int32 B) -> bool
	{
		for (const FLimbPairDef& P : Pairs)
		{
			if ((P.BodyA == A && P.BodyB == B) || (P.BodyA == B && P.BodyB == A))
			{
				return true;
			}
		}
		return false;
	};

	TestFalse(TEXT("FShoulder never pairs with Back1 -- spine bodies are never candidates, even with real radius"), ContainsPair(3, 1));
	TestFalse(TEXT("FShoulder never pairs with Head1 -- spine bodies are never candidates, even with real radius"), ContainsPair(3, 2));
	TestFalse(TEXT("FElbow never pairs with Back1 -- spine bodies are never candidates, even with real radius"), ContainsPair(4, 1));
	TestFalse(TEXT("Back1 and Head1 never pair -- neither is ever a candidate"), ContainsPair(1, 2));
	TestFalse(TEXT("FElbow and FShoulder never pair (same limb)"), ContainsPair(4, 3));
	TestTrue(TEXT("FShoulder (limb 0) DOES pair with BShoulder (limb 1) -- ordinary limb-vs-limb"), ContainsPair(3, 5));
	TestTrue(TEXT("FElbow (limb 0) DOES pair with BShoulder (limb 1) -- ordinary limb-vs-limb"), ContainsPair(4, 5));
	TestEqual(TEXT("Exactly the 2 valid limb-vs-limb pairs were built"), Pairs.Num(), 2);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLimbCollisionSeparationTest, "AgentSolver.LimbCollision.Separation", EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FLimbCollisionSeparationTest::RunTest(const FString& Parameters)
{
	// Root + two ball-jointed "limb" bodies on DIFFERENT limbs, placed so
	// their collision spheres deeply overlap at rest. No gravity, no ground
	// -- isolates the pair-contact resolution itself from anything else.
	FCreatureTopology Topo;
	TArray<int32> BodyParent = { 0, 0, 0 };
	TArray<int32> BodyDOFCount = { 0, 3, 3 };
	TArray<int32> BodyLimbIndex = { INDEX_NONE, 0, 1 };
	Topo.NumLimbs = 2;
	Topo.Build(BodyParent, BodyDOFCount, BodyLimbIndex);

	Topo.BodyMass[0] = 50.0f;
	Topo.BodyInertiaDiagLocal[0] = FVector(50.0f, 50.0f, 50.0f);

	Topo.BodyMass[1] = 5.0f;
	Topo.BodyInertiaDiagLocal[1] = FVector(5.0f, 5.0f, 5.0f);
	Topo.BodyJointOffsetInParent[1] = FVector(0.0f, -1.0f, 0.0f);
	Topo.BodyRadius[1] = 3.0f;

	Topo.BodyMass[2] = 5.0f;
	Topo.BodyInertiaDiagLocal[2] = FVector(5.0f, 5.0f, 5.0f);
	Topo.BodyJointOffsetInParent[2] = FVector(0.0f, 1.0f, 0.0f);
	Topo.BodyRadius[2] = 3.0f;

	// Origins 2 apart, radii sum to 6 -- starts deeply (4 units) overlapping.
	const TArray<FLimbPairDef> LimbPairs = { { 1, 2 } };

	FLimbCollisionParams LimbCollision;
	LimbCollision.bEnabled = true;
	LimbCollision.Hertz = 30.0f;
	LimbCollision.DampingRatio = 10.0f;
	LimbCollision.Slop = 0.5f;
	LimbCollision.FrictionCoefficient = 0.8f;

	FCreatureBatchState Batch;
	Batch.Init(Topo, 1);
	FCreatureABASolver Solver;
	FImpulseContactCache Cache;
	const TArray<FContactPointDef> NoGroundPoints; // ground/joint-limit rows unused in this test
	const FImpulseContactParams UnusedGroundParams;

	constexpr float Dt = 1.0f / 240.0f;
	constexpr int32 NumSteps = 480; // 2s

	auto TotalMomentum = [&Batch, &Topo]() -> FVector
	{
		FVector P = FVector::ZeroVector;
		for (int32 Body = 0; Body < Topo.NumBodies; ++Body)
		{
			const int32 Idx = Batch.BodyIndex(Body, 0);
			const FVector V(Batch.LinVelX[Idx], Batch.LinVelY[Idx], Batch.LinVelZ[Idx]);
			P += Topo.BodyMass[Body] * V;
		}
		return P;
	};

	const float InitialDistance = static_cast<float>((Batch.GetBodyPos(2, 0) - Batch.GetBodyPos(1, 0)).Size());

	bool bAnyNonFinite = false;
	float MaxMomentumMag = 0.0f;
	float FinalDistance = InitialDistance;
	for (int32 Step = 0; Step < NumSteps; ++Step)
	{
		Solver.Step(Batch, Dt, FVector::ZeroVector); // no gravity -- isolates pair contact
		ResolveGroundContactImpulses(
			Batch, Topo, NoGroundPoints, UnusedGroundParams, Solver, Dt, Cache,
			/*OutState*/ nullptr, /*JointLimits*/ nullptr, LimbPairs, &LimbCollision);

		const FVector Pos1 = Batch.GetBodyPos(1, 0);
		const FVector Pos2 = Batch.GetBodyPos(2, 0);
		if (!FMath::IsFinite(Pos1.X) || !FMath::IsFinite(Pos1.Z) || !FMath::IsFinite(Pos2.X) || !FMath::IsFinite(Pos2.Z))
		{
			bAnyNonFinite = true;
			break;
		}

		FinalDistance = static_cast<float>((Pos2 - Pos1).Size());
		MaxMomentumMag = FMath::Max(MaxMomentumMag, static_cast<float>(TotalMomentum().Size()));
	}

	AddInfo(FString::Printf(TEXT("Distance %.3f -> %.3f (radii sum 6.0), max |total momentum| over run = %.4f"), InitialDistance, FinalDistance, MaxMomentumMag));

	TestFalse(TEXT("No NaN/Inf over the run"), bAnyNonFinite);
	TestTrue(TEXT("The two different-limb bodies separate (distance increases from the initial deep overlap)"), FinalDistance > InitialDistance + 1.0f);

	// Started at rest with no gravity/ground -- total momentum should stay
	// near zero throughout, since a limb-pair contact impulse is an internal
	// force to the whole tree (equal-and-opposite at the two contact
	// points). A sign/arm bug in the pair superposition math would leak
	// momentum instead of just failing to separate the bodies.
	TestTrue(TEXT("Total system momentum stays near zero (no leak from the pair-impulse superposition)"), MaxMomentumMag < 50.0f);

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
