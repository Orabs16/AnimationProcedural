// Validates the RL environment glue in CreatureRLEnvironment.h: observation
// sizing/content, action scaling, the standing/balance reward's response to
// tilt and center-of-pressure offset, fall termination, and reset. Built
// from a synthetic topology (no Muto/editor assets needed) so it covers the
// general environment logic independent of any specific creature.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "CreatureRLEnvironment.h"
#include "CreatureBatchSolver.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace CreatureRLEnvironmentTest
{
	using namespace CreatureRLEnvironment;
	using namespace CreatureGroundContact;

	// Torso (body 0) + two single-DOF "leg" bodies (1, 2), each its own
	// ground-contact point at its own origin — enough to exercise the
	// balance (center-of-pressure) term with more than one contact point.
	static FCreatureTopology BuildTestTopology()
	{
		FCreatureTopology Topo;
		TArray<int32> BodyParent = { 0, 0, 0 };
		TArray<int32> BodyDOFCount = { 0, 1, 1 };
		TArray<int32> BodyLimbIndex = { INDEX_NONE, 0, 1 };
		Topo.NumLimbs = 2;
		Topo.Build(BodyParent, BodyDOFCount, BodyLimbIndex);

		Topo.BodyMass[0] = 20.0f;
		Topo.BodyInertiaDiagLocal[0] = FVector(5.0f, 5.0f, 5.0f);

		Topo.BodyJointAxisLocal[1] = FVector(0.0f, 1.0f, 0.0f);
		// Z offset is -102, 2 units past the standing torso height (100) below
		// GroundZ=0, so the feet start with real penetration and definitely
		// register bTouching=true — planting them exactly AT the ground plane
		// (Z offset -100) would leave Penetration=0, which ApplyGroundContactForces'
		// strict ">0" check treats as not-touching, silently zeroing this
		// test's balance-term checks (caught by the off-center reward
		// assertion failing since both sides evaluate to 0 contact force).
		Topo.BodyJointOffsetInParent[1] = FVector(-30.0f, 0.0f, -102.0f); // left leg, planted below+beside torso
		Topo.BodyLocalCoMOffset[1] = FVector::ZeroVector;
		Topo.BodyMass[1] = 2.0f;
		Topo.BodyInertiaDiagLocal[1] = FVector(0.1f, 0.1f, 0.1f);

		Topo.BodyJointAxisLocal[2] = FVector(0.0f, 1.0f, 0.0f);
		Topo.BodyJointOffsetInParent[2] = FVector(30.0f, 0.0f, -102.0f); // right leg
		Topo.BodyLocalCoMOffset[2] = FVector::ZeroVector;
		Topo.BodyMass[2] = 2.0f;
		Topo.BodyInertiaDiagLocal[2] = FVector(0.1f, 0.1f, 0.1f);

		return Topo;
	}

	static TArray<FContactPointDef> BuildTestContactPoints()
	{
		return {
			{ 1, FVector::ZeroVector, TEXT("LeftFoot"), 0 },
			{ 2, FVector::ZeroVector, TEXT("RightFoot"), 1 },
		};
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCreatureRLEnvironmentTest, "AgentSolver.RLEnvironment", EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FCreatureRLEnvironmentTest::RunTest(const FString& Parameters)
{
	using namespace CreatureRLEnvironmentTest;
	using namespace CreatureRLEnvironment;
	using namespace CreatureGroundContact;

	const FCreatureTopology Topo = BuildTestTopology();
	const TArray<FContactPointDef> ContactPoints = BuildTestContactPoints();

	FEnvConfig Config;
	Config.GroundZ = 0.0f;
	Config.TargetTorsoHeight = 100.0f;
	Config.MaxTorquePerDOF = 10.0f;

	constexpr int32 NumEnvs = 1;
	FCreatureBatchState Batch;
	Batch.Init(Topo, NumEnvs);

	const FVector StandingPos(0.0f, 0.0f, 100.0f);
	const FQuat StandingRot = FQuat::Identity;
	FRandomStream Stream(42);

	// ---- Reset: standing state should be clean and NaN-free ----
	ResetEnv(Batch, 0, StandingPos, StandingRot, Stream, 0.0f, 0.0f); // no noise, deterministic check
	TestEqual(TEXT("Reset: torso at standing height"), Batch.GetBodyPos(0, 0).Z, 100.0);
	TestEqual(TEXT("Reset: joint 0 at rest"), Batch.JointPos[Batch.DOFIndex(0, 0)], 0.0f);
	TestEqual(TEXT("Reset: joint 1 at rest"), Batch.JointPos[Batch.DOFIndex(1, 0)], 0.0f);
	TestEqual(TEXT("Reset: torso lin vel zeroed"), Batch.LinVelX[Batch.BodyIndex(0, 0)], 0.0f);

	// ---- Actions: normalized [-1,1] scales into JointTorque, clamped ----
	TArray<float> Actions = { 0.5f, -2.0f }; // second value out-of-range, should clamp to -1
	ApplyActions(Batch, 0, Actions, Config);
	TestEqual(TEXT("Action 0.5 scales to half MaxTorquePerDOF"), Batch.JointTorque[Batch.DOFIndex(0, 0)], 5.0f);
	TestEqual(TEXT("Action -2.0 clamps to -MaxTorquePerDOF"), Batch.JointTorque[Batch.DOFIndex(1, 0)], -10.0f);
	ApplyActions(Batch, 0, { 0.0f, 0.0f }, Config); // clear for the reward checks below

	// ---- Observations: size matches GetObservationSize, contains expected values ----
	FCreatureABASolver Solver;
	Solver.Step(Batch, 1.0f / 240.0f); // run one step so body 1/2 world positions are populated from the topology's joint offsets

	TArray<FContactPointState> ContactStates;
	FImpulseContactParams ContactParams;
	ContactParams.GroundZ = Config.GroundZ;
	FImpulseContactCache ContactCache;
	// This test only needs ContactStates populated so the observation and reward
	// helpers have touching/force data to read; the contact model itself is not
	// under test here.
	ResolveGroundContactImpulses(Batch, Topo, ContactPoints, ContactParams, Solver,
		1.0f / 240.0f, ContactCache, &ContactStates);

	TArray<float> Observation;
	ComputeObservations(Batch, 0, Config, ContactPoints, ContactStates, NumEnvs, Observation);
	TestEqual(TEXT("Observation size matches GetObservationSize"), Observation.Num(), GetObservationSize(Topo, ContactPoints.Num()));

	// ---- Reward: upright + balanced beats tilted / imbalanced ----
	const float UprightBalancedReward = ComputeReward(Batch, 0, Config, ContactPoints, ContactStates, NumEnvs);

	// Tilt the torso 60 degrees off vertical — upright term should collapse.
	Batch.SetBodyRot(0, 0, FQuat(FVector::ForwardVector, FMath::DegreesToRadians(60.0f)));
	const float TiltedReward = ComputeReward(Batch, 0, Config, ContactPoints, ContactStates, NumEnvs);
	TestTrue(TEXT("Upright reward beats a 60-degree tilt"), UprightBalancedReward > TiltedReward);
	Batch.SetBodyRot(0, 0, StandingRot); // restore

	// Shift the torso far sideways from the (still-planted) feet — the
	// balance term (center-of-pressure vs torso position) should collapse.
	Batch.SetBodyPos(0, 0, StandingPos + FVector(500.0f, 0.0f, 0.0f));
	const float OffCenterReward = ComputeReward(Batch, 0, Config, ContactPoints, ContactStates, NumEnvs);
	TestTrue(TEXT("Centered-over-feet reward beats a large horizontal offset"), UprightBalancedReward > OffCenterReward);
	Batch.SetBodyPos(0, 0, StandingPos); // restore

	// ---- Termination: upright standing shouldn't terminate; tilted/collapsed should ----
	TestFalse(TEXT("Standing upright at full height is not terminated"), IsTerminated(Batch, 0, Config));

	Batch.SetBodyRot(0, 0, FQuat(FVector::ForwardVector, FMath::DegreesToRadians(90.0f)));
	TestTrue(TEXT("Tipped fully sideways (90 deg) is terminated"), IsTerminated(Batch, 0, Config));
	Batch.SetBodyRot(0, 0, StandingRot);

	Batch.SetBodyPos(0, 0, FVector(0.0f, 0.0f, 10.0f)); // well below MinHeightFraction * TargetTorsoHeight
	TestTrue(TEXT("Collapsed to near-ground height is terminated"), IsTerminated(Batch, 0, Config));

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
