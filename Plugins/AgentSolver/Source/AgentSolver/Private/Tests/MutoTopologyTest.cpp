// Validates the topology MutoTopology::BuildMutoTopology produces from the
// real, authored Muto rig/profile assets — structural checks (tree validity,
// expected body/DOF counts) plus an end-to-end smoke test (a few solver
// steps under gravity alone, checking for NaN/Inf) against the full 8-limb
// topology, which is far larger/more irregular than the synthetic topology
// CreatureBatchSolverSIMDTest.cpp uses.

#include "CoreMinimal.h"

#if WITH_EDITOR

#include "Misc/AutomationTest.h"
#include "MutoTopology.h"
#include "CreatureBatchSolver.h"

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

	// 1 torso + 8 limbs x (5 F-shaped + 5 B-shaped + 3 M-shaped + 4 Hips-shaped)/4 limb-types,
	// i.e. (5+5+3+4) joint bones x 2 sides = 34, plus the torso.
	const int32 ExpectedBodies = 1 + (5 + 5 + 3 + 4) * 2;
	const int32 ExpectedDOF = (7 + 7 + 5 + 6) * 2; // per limb: 3 (ball) + (chain length - 1) x 1 (yaw)
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

	// End-to-end smoke test: step the full topology under gravity alone and
	// check for NaN/Inf. This is a much larger/more irregular tree (35 bodies,
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
