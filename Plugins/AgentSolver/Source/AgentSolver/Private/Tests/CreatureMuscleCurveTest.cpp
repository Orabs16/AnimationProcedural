// Validates the muscle strength-vs-angle curve wiring added to
// FCreatureABASolver's Pass 2 (see CreatureBatchSolver.h's
// ComputeMuscleMultipliers): a single revolute joint's effective torque
// should scale with its authored ExtensionStrengthCurve/FlexionStrengthCurve,
// evaluated at the joint's current angle normalized into its authored
// [MinRange,MaxRange]. Built entirely from a synthetic topology (no
// MassMuscleProfile/editor assets needed) so it covers the general solver
// wiring independent of MutoTopology's specific muscle-to-DOF mapping.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "CreatureBatchSolver.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace CreatureMuscleCurveTest
{
	static FCreatureTopology BuildSingleJointTopology()
	{
		FCreatureTopology Topo;
		TArray<int32> BodyParent = { 0, 0 };
		TArray<int32> BodyDOFCount = { 0, 1 };
		TArray<int32> BodyLimbIndex = { INDEX_NONE, 0 };
		Topo.NumLimbs = 1;
		Topo.Build(BodyParent, BodyDOFCount, BodyLimbIndex);

		Topo.BodyMass[0] = 1.0e5f;
		Topo.BodyInertiaDiagLocal[0] = FVector(1.0e7f, 1.0e7f, 1.0e7f);

		Topo.BodyJointAxisLocal[1] = FVector(0.0f, 0.0f, 1.0f);
		Topo.BodyJointOffsetInParent[1] = FVector::ZeroVector;
		Topo.BodyLocalCoMOffset[1] = FVector(50.0f, 0.0f, 0.0f);
		Topo.BodyMass[1] = 5.0f;
		Topo.BodyInertiaDiagLocal[1] = FVector(0.3f, 0.3f, 0.15f);

		// Weak at the bottom of the range (t=0), strong at the top (t=1) —
		// opposite for flexion, so extension-direction torque at each extreme
		// exercises a clearly different multiplier.
		Topo.DOFExtensionCurve[0].GetRichCurve()->Reset();
		Topo.DOFExtensionCurve[0].GetRichCurve()->AddKey(0.0f, 0.1f);
		Topo.DOFExtensionCurve[0].GetRichCurve()->AddKey(1.0f, 1.0f);
		Topo.DOFFlexionCurve[0].GetRichCurve()->Reset();
		Topo.DOFFlexionCurve[0].GetRichCurve()->AddKey(0.0f, 1.0f);
		Topo.DOFFlexionCurve[0].GetRichCurve()->AddKey(1.0f, 0.1f);
		Topo.DOFRangeMinDeg[0] = -45.0f;
		Topo.DOFRangeMaxDeg[0] = 45.0f;
		Topo.DOFHasMuscleCurve[0] = 1;

		return Topo;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCreatureMuscleCurveTest, "AgentSolver.MuscleCurve", EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FCreatureMuscleCurveTest::RunTest(const FString& Parameters)
{
	using namespace CreatureMuscleCurveTest;

	const FCreatureTopology Topo = BuildSingleJointTopology();
	constexpr float Dt = 1.0f / 240.0f;
	constexpr float CommandedTorque = 1000.0f;

	// Env 0: joint at -45 deg (t=0, weak extension multiplier ~0.1).
	// Env 1: joint at +45 deg (t=1, strong extension multiplier ~1.0).
	FCreatureBatchState Batch;
	constexpr int32 NumEnvs = 2;
	Batch.Init(Topo, NumEnvs);
	Batch.JointPos[Batch.DOFIndex(0, 0)] = FMath::DegreesToRadians(-45.0f);
	Batch.JointPos[Batch.DOFIndex(0, 1)] = FMath::DegreesToRadians(45.0f);
	Batch.JointTorque[Batch.DOFIndex(0, 0)] = CommandedTorque; // positive -> extension direction
	Batch.JointTorque[Batch.DOFIndex(0, 1)] = CommandedTorque;

	FCreatureABASolver Solver;
	Solver.Step(Batch, Dt, FVector::ZeroVector); // no gravity — isolate the torque-multiplier effect

	const float WeakJointVel = Batch.JointVel[Batch.DOFIndex(0, 0)];
	const float StrongJointVel = Batch.JointVel[Batch.DOFIndex(0, 1)];

	AddInfo(FString::Printf(TEXT("WeakJointVel(t=0, mul~0.1)=%.6f StrongJointVel(t=1, mul~1.0)=%.6f ratio=%.4f"),
		WeakJointVel, StrongJointVel, WeakJointVel / StrongJointVel));

	TestTrue(TEXT("No NaN/Inf"), FMath::IsFinite(WeakJointVel) && FMath::IsFinite(StrongJointVel));
	TestTrue(TEXT("Both accelerate in the commanded (positive) direction"), WeakJointVel > 0.0f && StrongJointVel > 0.0f);
	TestTrue(TEXT("Weak-multiplier env accelerates markedly less than the strong-multiplier env"), WeakJointVel < 0.3f * StrongJointVel);

	// Same check via StepScalar (the independent reference implementation),
	// confirming the multiplier wiring isn't a StepSIMD-only artifact.
	FCreatureBatchState BatchScalar;
	BatchScalar.Init(Topo, NumEnvs);
	BatchScalar.JointPos[BatchScalar.DOFIndex(0, 0)] = FMath::DegreesToRadians(-45.0f);
	BatchScalar.JointPos[BatchScalar.DOFIndex(0, 1)] = FMath::DegreesToRadians(45.0f);
	BatchScalar.JointTorque[BatchScalar.DOFIndex(0, 0)] = CommandedTorque;
	BatchScalar.JointTorque[BatchScalar.DOFIndex(0, 1)] = CommandedTorque;
	Solver.StepScalar(BatchScalar, Dt, FVector::ZeroVector);

	const float WeakJointVelScalar = BatchScalar.JointVel[BatchScalar.DOFIndex(0, 0)];
	const float StrongJointVelScalar = BatchScalar.JointVel[BatchScalar.DOFIndex(0, 1)];
	TestTrue(TEXT("StepScalar: weak-multiplier env also accelerates markedly less"), WeakJointVelScalar < 0.3f * StrongJointVelScalar);

	// A DOF with no authored curve (DOFHasMuscleCurve==false) must behave
	// exactly as before this feature existed: multiplier 1, full torque.
	FCreatureTopology TopoNoCurve = BuildSingleJointTopology();
	TopoNoCurve.DOFHasMuscleCurve[0] = 0;
	FCreatureBatchState BatchNoCurve;
	BatchNoCurve.Init(TopoNoCurve, 1);
	BatchNoCurve.JointPos[BatchNoCurve.DOFIndex(0, 0)] = FMath::DegreesToRadians(-45.0f);
	BatchNoCurve.JointTorque[BatchNoCurve.DOFIndex(0, 0)] = CommandedTorque;
	Solver.Step(BatchNoCurve, Dt, FVector::ZeroVector);
	const float NoCurveJointVel = BatchNoCurve.JointVel[BatchNoCurve.DOFIndex(0, 0)];
	TestTrue(TEXT("No-curve DOF gets full (unscaled) torque, matching the strong-multiplier case"),
		FMath::IsNearlyEqual(NoCurveJointVel, StrongJointVel, 0.01f * StrongJointVel));

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
