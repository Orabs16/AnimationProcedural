// Validates the topology MutoTopology::BuildMutoTopology produces from the
// real, authored Muto rig/profile assets — structural checks (tree validity,
// expected body/DOF counts) plus an end-to-end smoke test (a few solver
// steps under gravity alone, checking for NaN/Inf) against the full 8-limb
// topology, which is far larger/more irregular than the synthetic topology
// CreatureBatchSolverSIMDTest.cpp uses.

#include "CoreMinimal.h"

#if WITH_EDITOR

#include "Misc/AutomationTest.h"
#include "PhysicsSolver/MutoTopology.h"
#include "PhysicsSolver/CreatureBatchSolver.h"

#if WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMutoTopologyTest, "AgentSolver.MutoTopology", EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FMutoTopologyTest::RunTest(const FString& Parameters)
{
	UMassMuscleProfileAssetMuscle* MuscleAsset = LoadObject<UMassMuscleProfileAssetMuscle>(nullptr, TEXT("/Game/Mesh/MuscleProfile_Muto.MuscleProfile_Muto"));
	UMassMuscleProfileAssetMass* MassAsset = LoadObject<UMassMuscleProfileAssetMass>(nullptr, TEXT("/Game/Mesh/MassProfile_Muto.MassProfile_Muto"));
	USkeletalMesh* SkeletalMesh = LoadObject<USkeletalMesh>(nullptr, TEXT("/Game/Mesh/Muto.Muto"));

	if (!TestNotNull(TEXT("MuscleProfile_Muto loaded"), MuscleAsset)) return false;
	if (!TestNotNull(TEXT("MassProfile_Muto loaded"), MassAsset)) return false;
	if (!TestNotNull(TEXT("Muto skeletal mesh loaded"), SkeletalMesh)) return false;

	FCreatureTopology Topo;
	TArray<FString> Warnings;
	TArray<FName> DebugNames;
	const bool bBuilt = MutoTopology::BuildMutoTopology(SkeletalMesh, MassAsset, MuscleAsset, Topo, Warnings, &DebugNames);

	for (const FString& Warning : Warnings)
	{
		AddError(FString::Printf(TEXT("BuildMutoTopology warning: %s"), *Warning));
	}
	if (!TestTrue(TEXT("BuildMutoTopology succeeded"), bBuilt)) return false;

	// 1 Pelvis + 6 spine/head/jaw bodies (2026-08-16: independently articulated,
	// each a 3-DOF ball -- Back1-3, Head1-2, LowerMouth. UpperMouth and Chin
	// are leaves with no muscle of their own, same as a limb's Tip bone --
	// GetMutoTorsoLeafBones -- so they get no ABA body, just fused rigidly
	// into their real parent) + 8 limbs x (5 F-shaped + 5 B-shaped + 3
	// M-shaped + 4 Hips-shaped)/4 limb-types, i.e. (5+5+3+4) joint bones x 2
	// sides = 34.
	const int32 ExpectedBodies = 1 + 6 + (5 + 5 + 3 + 4) * 2;
	const int32 ExpectedDOF = 6 * 3 + (7 + 7 + 5 + 6) * 2; // spine: 6 x 3-DOF ball; per limb: 3 (ball) + (chain length - 1) x 1 (yaw)
	TestEqual(TEXT("NumBodies"), Topo.NumBodies, ExpectedBodies);
	TestEqual(TEXT("NumDOF"), Topo.NumDOF, ExpectedDOF);
	TestEqual(TEXT("NumLimbs"), Topo.NumLimbs, 8);
	TestEqual(TEXT("DebugNames.Num() matches NumBodies"), DebugNames.Num(), Topo.NumBodies);

	// Tree validity: BodyParent[i] < i for every non-root body (required by the
	// solver — a single forward pass over indices must visit parents before children).
	bool bTreeValid = true;
	for (int32 Body = 1; Body < Topo.NumBodies; ++Body)
	{
		if (Topo.BodyParent[Body] >= Body)
		{
			AddError(FString::Printf(TEXT("Body %d (%s) has parent %d, which is not < Body"), Body, *DebugNames[Body].ToString(), Topo.BodyParent[Body]));
			bTreeValid = false;
		}
	}
	TestTrue(TEXT("BodyParent[i] < i for all i"), bTreeValid);

	// Every body should have positive mass and a non-degenerate inertia diagonal
	// (both placeholders, but a zero anywhere would make the solver's inertia
	// composition singular).
	bool bMassInertiaValid = true;
	for (int32 Body = 0; Body < Topo.NumBodies; ++Body)
	{
		if (Topo.BodyMass[Body] <= 0.0f || Topo.BodyInertiaDiagLocal[Body].GetMin() <= 0.0f)
		{
			AddError(FString::Printf(TEXT("Body %d (%s) has non-positive mass (%.4f) or inertia (%s)"),
				Body, *DebugNames[Body].ToString(), Topo.BodyMass[Body], *Topo.BodyInertiaDiagLocal[Body].ToString()));
			bMassInertiaValid = false;
		}
	}
	TestTrue(TEXT("All bodies have positive mass/inertia"), bMassInertiaValid);

	// ---- Authored per-muscle / per-bone data actually reaches the topology ----
	//
	// Both of these were write-only before 2026-08-21: the MassMuscleProfile tool
	// let you edit ExtensionStrength/FlexionStrength and mirrored them L/R, and
	// InitializeFromSkeletalMesh stamped a BoneIndex into every mass entry, and
	// BuildMutoTopology read neither. Nothing failed when they drifted, which is
	// exactly why this is asserted rather than assumed.
	{
		int32 NumBoneIndexResolved = 0;
		bool bBoneIndexNamesAgree = true;
		const FReferenceSkeleton& RefSkel = SkeletalMesh->GetRefSkeleton();
		for (int32 Body = 0; Body < Topo.NumBodies; ++Body)
		{
			const int32 BoneIdx = Topo.BodyBoneIndex.IsValidIndex(Body) ? Topo.BodyBoneIndex[Body] : INDEX_NONE;
			if (BoneIdx == INDEX_NONE) continue;
			++NumBoneIndexResolved;
			// The index must name the same bone the debug name does -- an
			// off-by-one here would silently give every body its NEIGHBOUR's
			// authored mass, radius and capsule, which is a plausible-looking rig
			// rather than an obviously broken one.
			if (RefSkel.GetBoneName(BoneIdx) != DebugNames[Body]) bBoneIndexNamesAgree = false;
		}
		TestEqual(TEXT("Every body resolved a reference-skeleton bone index"), NumBoneIndexResolved, Topo.NumBodies);
		TestTrue(TEXT("BodyBoneIndex names the same bone as BodyDebugNames"), bBoneIndexNamesAgree);

		int32 NumCurveDOFs = 0, NumNonUnitStrength = 0;
		float MinStrength = TNumericLimits<float>::Max(), MaxStrength = TNumericLimits<float>::Lowest();
		bool bAllStrengthsSane = true;
		for (int32 DOF = 0; DOF < Topo.NumDOF; ++DOF)
		{
			if (!Topo.DOFHasMuscleCurve[DOF]) continue;
			++NumCurveDOFs;
			const float Ext = Topo.DOFExtensionStrength[DOF];
			const float Flex = Topo.DOFFlexionStrength[DOF];
			if (!FMath::IsFinite(Ext) || !FMath::IsFinite(Flex) || Ext < 0.0f || Flex < 0.0f) bAllStrengthsSane = false;
			if (!FMath::IsNearlyEqual(Ext, 1.0f) || !FMath::IsNearlyEqual(Flex, 1.0f)) ++NumNonUnitStrength;
			MinStrength = FMath::Min3(MinStrength, Ext, Flex);
			MaxStrength = FMath::Max3(MaxStrength, Ext, Flex);
		}
		TestTrue(TEXT("Some DOF carries an authored muscle curve"), NumCurveDOFs > 0);
		TestTrue(TEXT("Every authored muscle strength is finite and non-negative"), bAllStrengthsSane);
		AddInfo(FString::Printf(TEXT("Authored muscle strengths: %d/%d curve DOFs, %d differ from 1.0, range [%.3f, %.3f]"),
			NumCurveDOFs, Topo.NumDOF, NumNonUnitStrength, MinStrength, MaxStrength));
	}

	// ---- Capsule inertia against closed-form limits ----
	//
	// Checked against the two shapes whose inertia is textbook, approached from
	// inside the general formula rather than special-cased: a capsule with no
	// cylinder is a SPHERE, and a capsule with no radius is a THIN ROD. If the
	// volume-weighted mass split or the caps' parallel-axis term is wrong, one of
	// these two disagrees with a number that is not in dispute.
	{
		constexpr float M = 3.0f;
		constexpr float R = 7.0f;
		constexpr float L = 40.0f;

		// L = 0: pure sphere. I = (2/5) m r^2 about every axis, isotropic.
		const FVector Sphere = MutoTopology::CapsuleInertiaDiagLocal(M, R, 0.0f);
		const float SphereExpected = 0.4f * M * R * R;
		TestTrue(TEXT("capsule with no cylinder is a sphere (axial)"), FMath::IsNearlyEqual(Sphere.X, SphereExpected, SphereExpected * 1.0e-4f));
		TestTrue(TEXT("capsule with no cylinder is a sphere (transverse)"), FMath::IsNearlyEqual(Sphere.Y, SphereExpected, SphereExpected * 1.0e-4f));
		TestTrue(TEXT("a sphere's inertia is isotropic"), FMath::IsNearlyEqual(Sphere.X, Sphere.Y, SphereExpected * 1.0e-4f));

		// R -> 0: thin rod. Axial -> 0, transverse -> m L^2 / 12. Approached with a
		// tiny but non-zero radius, since R == 0 takes the fallback path instead.
		const FVector Rod = MutoTopology::CapsuleInertiaDiagLocal(M, L * 1.0e-4f, L);
		const float RodExpected = M * L * L / 12.0f;
		TestTrue(TEXT("vanishing radius gives a thin rod's transverse inertia"), FMath::IsNearlyEqual(Rod.Y, RodExpected, RodExpected * 1.0e-3f));
		TestTrue(TEXT("vanishing radius gives ~zero axial inertia"), Rod.X < RodExpected * 1.0e-6f);

		// A bone with no authored radius must keep the numbers the old thin-rod
		// model gave it, EXACTLY -- not "to within a few percent". The first
		// implementation tried to hit this with an equivalent capsule radius of
		// L/sqrt(120) and missed by 2.2% axially and 29% transversely, which this
		// assertion caught. The fallback returns the rod directly now.
		const FVector Fallback = MutoTopology::CapsuleInertiaDiagLocal(M, 0.0f, L);
		const float OldAxial = 0.05f * RodExpected;
		TestTrue(TEXT("no authored radius reproduces the old axial term exactly"), FMath::IsNearlyEqual(Fallback.X, OldAxial, OldAxial * 1.0e-5f));
		TestTrue(TEXT("no authored radius reproduces the old transverse term exactly"), FMath::IsNearlyEqual(Fallback.Y, RodExpected, RodExpected * 1.0e-5f));

		// Monotonic in radius, both axes -- a fatter bone is never easier to spin.
		const FVector Thin = MutoTopology::CapsuleInertiaDiagLocal(M, R, L);
		const FVector Fat = MutoTopology::CapsuleInertiaDiagLocal(M, R * 2.0f, L);
		TestTrue(TEXT("axial inertia grows with radius"), Fat.X > Thin.X);
		TestTrue(TEXT("transverse inertia grows with radius"), Fat.Y > Thin.Y);
	}

	// ---- What the switch to authored radii actually did to this rig ----
	// Reported, not asserted: the numbers are the point, and pinning them would
	// just re-encode the authored asset in a test.
	{
		float WorstAxialRatio = 1.0f, BestAxialRatio = 1.0f;
		int32 WorstBody = INDEX_NONE, BestBody = INDEX_NONE;
		for (int32 Body = 1; Body < Topo.NumBodies; ++Body)
		{
			const float BoneLen = 2.0f * static_cast<float>(Topo.BodyLocalCoMOffset[Body].Size());
			if (BoneLen <= 0.0f || Topo.BodyMass[Body] <= 0.0f) continue;
			const float OldAxial = 0.05f * (Topo.BodyMass[Body] * BoneLen * BoneLen / 12.0f);
			if (OldAxial <= 0.0f) continue;
			const float Ratio = static_cast<float>(Topo.BodyInertiaDiagLocal[Body].X) / OldAxial;
			if (Ratio > WorstAxialRatio) { WorstAxialRatio = Ratio; WorstBody = Body; }
			if (Ratio < BestAxialRatio) { BestAxialRatio = Ratio; BestBody = Body; }
		}
		AddInfo(FString::Printf(TEXT("Axial inertia vs the old thin-rod fudge: largest increase %.1fx on %s, largest decrease %.2fx on %s"),
			WorstAxialRatio, WorstBody != INDEX_NONE ? *DebugNames[WorstBody].ToString() : TEXT("-"),
			BestAxialRatio, BestBody != INDEX_NONE ? *DebugNames[BestBody].ToString() : TEXT("-")));
	}

	// End-to-end smoke test: step the full topology under gravity alone and
	// check for NaN/Inf. This is a much larger/more irregular tree (43 bodies,
	// mixed ball+revolute, real non-uniform offsets) than the synthetic
	// topology CreatureBatchSolverSIMDTest.cpp exercises.
	FCreatureBatchState Batch;
	constexpr int32 NumEnvs = 4;
	Batch.Init(Topo, NumEnvs);

	FCreatureABASolver Solver;
	constexpr float Dt = 1.0f / 60.0f;
	bool bAnyNonFinite = false;
	for (int32 Step = 0; Step < 10; ++Step)
	{
		Solver.Step(Batch, Dt);
		for (int32 Body = 0; Body < Topo.NumBodies && !bAnyNonFinite; ++Body)
		{
			for (int32 Env = 0; Env < NumEnvs; ++Env)
			{
				const FVector Pos = Batch.GetBodyPos(Body, Env);
				if (!FMath::IsFinite(Pos.X) || !FMath::IsFinite(Pos.Y) || !FMath::IsFinite(Pos.Z))
				{
					AddError(FString::Printf(TEXT("Non-finite position at step %d, body %d (%s), env %d"), Step, Body, *DebugNames[Body].ToString(), Env));
					bAnyNonFinite = true;
					break;
				}
			}
		}
		if (bAnyNonFinite) break;
	}
	TestFalse(TEXT("No NaN/Inf after 10 steps under gravity"), bAnyNonFinite);

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
#endif // WITH_EDITOR
