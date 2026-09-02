// Validates the imitation machinery in CreatureImitation.h: the pose retarget
// (component-space transforms -> the solver's own joint coordinates), the
// forward-kinematics round trip back out again, the revolute twist projection,
// and the reward terms' response to pose error.
//
// Built from synthetic topologies with NON-IDENTITY rest rotations and joint
// axes, deliberately: a rig whose bind pose is all-identity would pass the
// retarget even if BodyRestRotInParent were ignored entirely, which is exactly
// the class of error these tests exist to catch (see SOLVER_DEBUG_LOG.md
// entries 012 and 022 for two real bugs of precisely that shape).

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "AgentSolver/CreatureImitation.h"
#include "AgentSolver/CreatureRLEnvironment.h"
#include "PhysicsSolver/CreatureBatchSolver.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace CreatureImitationTest
{
	using namespace CreatureImitation;

	/**
	 * Torso (0) -> ball joint (1) -> revolute (2) -> revolute (3), a chain deep
	 * enough that a parent's error propagates to a grandchild if the frame math
	 * is wrong, and mixing both joint types so the two different FK conventions
	 * (post-multiplied ball, pre-multiplied revolute spin) are both exercised.
	 *
	 * Every rest rotation and joint axis below is deliberately off-axis and
	 * non-identity.
	 */
	static FCreatureTopology BuildChainTopology()
	{
		FCreatureTopology Topo;
		TArray<int32> BodyParent = { 0, 0, 1, 2 };
		TArray<int32> BodyDOFCount = { 0, 3, 1, 1 };
		TArray<int32> BodyLimbIndex = { INDEX_NONE, 0, 0, 0 };
		Topo.NumLimbs = 1;
		Topo.Build(BodyParent, BodyDOFCount, BodyLimbIndex);

		for (int32 Body = 0; Body < Topo.NumBodies; ++Body)
		{
			Topo.BodyMass[Body] = 1.0f;
			Topo.BodyInertiaDiagLocal[Body] = FVector(0.1f, 0.1f, 0.1f);
			Topo.BodyLocalCoMOffset[Body] = FVector::ZeroVector;
		}

		// Root's own bind rotation in component space -- the slot MutoTopology
		// repurposes for exactly this (see BodyRestRotInParent's comment).
		Topo.BodyRestRotInParent[0] = FQuat(FVector(0.0f, 0.0f, 1.0f), FMath::DegreesToRadians(35.0f));

		Topo.BodyRestRotInParent[1] = FQuat(FVector(0.3f, 0.5f, 0.81f).GetSafeNormal(), FMath::DegreesToRadians(27.0f));
		Topo.BodyJointOffsetInParent[1] = FVector(0.0f, 0.0f, -20.0f);
		Topo.BodyJointAxisLocal[1] = FVector::ZeroVector; // unused for ball joints

		Topo.BodyRestRotInParent[2] = FQuat(FVector(0.7f, 0.1f, 0.7f).GetSafeNormal(), FMath::DegreesToRadians(-41.0f));
		Topo.BodyJointOffsetInParent[2] = FVector(0.0f, 0.0f, -30.0f);
		// Axis lives in the PARENT's frame (see BodyJointAxisLocal's comment).
		Topo.BodyJointAxisLocal[2] = FVector(0.2f, 0.9f, 0.39f).GetSafeNormal();

		Topo.BodyRestRotInParent[3] = FQuat(FVector(0.1f, 0.8f, 0.59f).GetSafeNormal(), FMath::DegreesToRadians(18.0f));
		Topo.BodyJointOffsetInParent[3] = FVector(0.0f, 0.0f, -25.0f);
		Topo.BodyJointAxisLocal[3] = FVector(0.9f, 0.2f, 0.39f).GetSafeNormal();

		return Topo;
	}

	/**
	 * The rig's own REST pose in component space, built the same way
	 * ImitationBake::BuildRestComponentSpacePose builds it from a reference
	 * skeleton: compose each body's rest rotation/offset onto its parent's.
	 *
	 * This must be the pose that retargets to all-zero joint angles -- that
	 * invariant is the whole first test.
	 */
	static TArray<FTransform> BuildRestBodyTransforms(const FCreatureTopology& Topo)
	{
		TArray<FTransform> BodyCS;
		BodyCS.SetNum(Topo.NumBodies);
		BodyCS[0] = FTransform(Topo.BodyRestRotInParent[0], FVector(0.0f, 0.0f, 100.0f));
		for (int32 Body = 1; Body < Topo.NumBodies; ++Body)
		{
			const int32 Parent = Topo.BodyParent[Body];
			const FQuat ParentRot = BodyCS[Parent].GetRotation();
			BodyCS[Body] = FTransform(
				(ParentRot * Topo.BodyRestRotInParent[Body]).GetNormalized(),
				BodyCS[Parent].GetTranslation() + ParentRot.RotateVector(Topo.BodyJointOffsetInParent[Body]));
		}
		return BodyCS;
	}

	/** Component-space body transforms produced by the SOLVER's own Pass 1 from a batch's joint state -- the ground truth the retarget is checked against. */
	static TArray<FTransform> ReadBodyTransformsFromBatch(const FCreatureBatchState& Batch, int32 Env)
	{
		const FCreatureTopology& Topo = Batch.GetTopology();
		TArray<FTransform> BodyCS;
		BodyCS.SetNum(Topo.NumBodies);
		for (int32 Body = 0; Body < Topo.NumBodies; ++Body)
		{
			BodyCS[Body] = FTransform(Batch.GetBodyRot(Body, Env), Batch.GetBodyPos(Body, Env));
		}
		return BodyCS;
	}
}

// ============================================================================
// 1. Rest-pose invariant. The single most important check in this file: feeding
//    the rig's own rest pose through the retarget must produce zero joint
//    angles and identity ball rotations -- exactly the state
//    CreatureRLEnvironment::ResetEnv writes for a fresh episode. If the
//    retarget's frame conventions have drifted from CreatureBatchSolver.h's
//    Pass 1, this is where it shows, and nothing downstream is trustworthy
//    until it passes.
// ============================================================================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCreatureImitationRestPoseTest, "AgentSolver.Imitation.RestPose", EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FCreatureImitationRestPoseTest::RunTest(const FString& Parameters)
{
	using namespace CreatureImitationTest;
	using namespace CreatureImitation;

	const FCreatureTopology Topo = BuildChainTopology();
	const TArray<FTransform> RestCS = BuildRestBodyTransforms(Topo);

	FReferenceFrame Frame;
	float Residual = 0.0f;
	RetargetPoseToJointSpace(Topo, RestCS, TArray<int32>(), Frame, &Residual);

	for (int32 DOF = 0; DOF < Topo.NumDOF; ++DOF)
	{
		TestTrue(FString::Printf(TEXT("rest pose retargets DOF %d to zero (got %f)"), DOF, Frame.DOFPos[DOF]),
			FMath::IsNearlyZero(Frame.DOFPos[DOF], 1e-4f));
	}

	for (int32 Body = 1; Body < Topo.NumBodies; ++Body)
	{
		if (Topo.BodyDOFCount[Body] == 3)
		{
			TestTrue(FString::Printf(TEXT("rest pose retargets body %d's ball joint to identity"), Body),
				GeodesicAngleRad(Frame.BodyRelRot[Body], FQuat::Identity) < 1e-3f);
		}
	}

	// A rest pose is exactly representable, so nothing is discarded.
	TestTrue(TEXT("rest pose has no off-axis residual"), Residual < 1e-3f);

	// The root slot carries the torso's own component-space bind rotation.
	TestTrue(TEXT("rest pose recovers the root's bind rotation"),
		GeodesicAngleRad(Frame.RootRot, Topo.BodyRestRotInParent[0]) < 1e-3f);

	return true;
}

// ============================================================================
// 2. Forward-kinematics round trip. Retarget an arbitrary pose, write it into a
//    batch, run FK, and confirm the body transforms come back out. This is what
//    proves the retarget really is Pass 1's inverse rather than merely
//    self-consistent -- and it checks CreatureImitation's own single-env
//    RecomputeEnvKinematics against the solver's whole-batch
//    RecomputeKinematics at the same time, which is what would catch the two
//    drifting apart.
// ============================================================================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCreatureImitationRoundTripTest, "AgentSolver.Imitation.RoundTrip", EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FCreatureImitationRoundTripTest::RunTest(const FString& Parameters)
{
	using namespace CreatureImitationTest;
	using namespace CreatureImitation;

	const FCreatureTopology Topo = BuildChainTopology();

	// Build a genuinely articulated pose by driving the batch's joint state
	// directly and letting the SOLVER compute the body transforms -- so the
	// pose being retargeted is one the solver itself considers valid, not one
	// this test invented with the same math it is trying to verify.
	FCreatureBatchState Source;
	Source.Init(Topo, 1);
	Source.SetBodyPos(0, 0, FVector(10.0f, -5.0f, 120.0f));
	Source.SetBodyRot(0, 0, (FQuat(FVector(0.2f, 0.3f, 0.93f).GetSafeNormal(), 0.4f) * Topo.BodyRestRotInParent[0]).GetNormalized());

	const FQuat BallRot = FQuat(FVector(0.5f, 0.5f, 0.707f).GetSafeNormal(), FMath::DegreesToRadians(23.0f));
	Source.SetJointRelRot(1, 0, BallRot);
	const FVector BallRotVec = BallRot.ToRotationVector();
	Source.JointPos[Source.DOFIndex(0, 0)] = (float)BallRotVec.X;
	Source.JointPos[Source.DOFIndex(1, 0)] = (float)BallRotVec.Y;
	Source.JointPos[Source.DOFIndex(2, 0)] = (float)BallRotVec.Z;
	Source.JointPos[Source.DOFIndex(3, 0)] = 0.37f;  // revolute, body 2
	Source.JointPos[Source.DOFIndex(4, 0)] = -0.52f; // revolute, body 3

	FCreatureABASolver Solver;
	Solver.RecomputeKinematics(Source);

	const TArray<FTransform> PoseCS = ReadBodyTransformsFromBatch(Source, 0);

	// ---- Retarget it, then apply it to a DIFFERENT batch and re-run FK ----
	TArray<int32> EndEffectors = { 3 };
	FReferenceFrame Frame;
	float Residual = 0.0f;
	RetargetPoseToJointSpace(Topo, PoseCS, EndEffectors, Frame, &Residual);

	// A pose the solver itself generated sits exactly on every revolute's
	// 1-DOF manifold, so the twist projection discards nothing.
	TestTrue(FString::Printf(TEXT("a solver-generated pose has no off-axis residual (got %.4f rad)"), Residual), Residual < 1e-3f);

	FCreatureBatchState Target;
	Target.Init(Topo, 1);
	// ApplyReferenceFrameToEnv places the root from the supplied world position
	// plus the frame's rest-relative height, so hand it the source's own root
	// pose to make the two directly comparable.
	Frame.RootHeightAboveRest = 0.0f;
	ApplyReferenceFrameToEnv(Target, 0, Frame, Source.GetBodyPos(0, 0), Frame.RootRot);

	const TArray<FTransform> RoundTripCS = ReadBodyTransformsFromBatch(Target, 0);

	for (int32 Body = 0; Body < Topo.NumBodies; ++Body)
	{
		const float RotError = GeodesicAngleRad(PoseCS[Body].GetRotation(), RoundTripCS[Body].GetRotation());
		TestTrue(FString::Printf(TEXT("body %d round-trips its rotation (error %.5f rad)"), Body, RotError), RotError < 1e-3f);

		const float PosError = (float)FVector::Dist(PoseCS[Body].GetTranslation(), RoundTripCS[Body].GetTranslation());
		TestTrue(FString::Printf(TEXT("body %d round-trips its position (error %.4f)"), Body, PosError), PosError < 0.05f);
	}

	// ---- The single-env FK must agree with the solver's whole-batch one ----
	FCreatureBatchState SolverRefreshed;
	SolverRefreshed.Init(Topo, 1);
	ApplyReferenceFrameToEnv(SolverRefreshed, 0, Frame, Source.GetBodyPos(0, 0), Frame.RootRot);
	Solver.RecomputeKinematics(SolverRefreshed);
	for (int32 Body = 0; Body < Topo.NumBodies; ++Body)
	{
		const float RotError = GeodesicAngleRad(SolverRefreshed.GetBodyRot(Body, 0), RoundTripCS[Body].GetRotation());
		TestTrue(FString::Printf(TEXT("RecomputeEnvKinematics agrees with the solver's own FK for body %d"), Body), RotError < 1e-4f);
	}

	// ---- The pose reward must be exactly 1.0 at the reference ----
	// This is the in-test form of the first-frame check to run in PIE: with
	// bResetToReferencePose on, the pose reward should read ~1.0 before the
	// creature has done anything at all.
	FImitationConfig Imitation;
	const float PoseReward = ComputePoseReward(Target, 0, Frame, Imitation);
	TestTrue(FString::Printf(TEXT("pose reward is 1.0 at the reference (got %.6f)"), PoseReward), FMath::IsNearlyEqual(PoseReward, 1.0f, 1e-4f));

	const float EndEffReward = ComputeEndEffectorReward(Target, 0, Frame, EndEffectors, 100.0f, Imitation);
	TestTrue(FString::Printf(TEXT("end-effector reward is 1.0 at the reference (got %.6f)"), EndEffReward), FMath::IsNearlyEqual(EndEffReward, 1.0f, 1e-3f));

	return true;
}

// ============================================================================
// 3. Revolute twist projection. A rotation purely ABOUT the joint axis must be
//    recovered exactly with no residual; one partly off it must recover only
//    the on-axis component and report the rest as residual, rather than
//    silently folding the off-axis part into the joint angle.
// ============================================================================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCreatureImitationTwistTest, "AgentSolver.Imitation.TwistProjection", EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FCreatureImitationTwistTest::RunTest(const FString& Parameters)
{
	using namespace CreatureImitationTest;
	using namespace CreatureImitation;

	const FCreatureTopology Topo = BuildChainTopology();
	const TArray<FTransform> RestCS = BuildRestBodyTransforms(Topo);

	constexpr int32 RevoluteBody = 2;
	constexpr int32 ChildBody = 3; // RevoluteBody's own child -- see below
	const int32 RevoluteDOF = Topo.BodyDOFOffset[RevoluteBody];
	const float TargetAngle = FMath::DegreesToRadians(37.0f);

	// Pass 1 pre-multiplies the spin in world/component space, about the axis
	// expressed in the PARENT's frame.
	const FQuat ParentRot = RestCS[Topo.BodyParent[RevoluteBody]].GetRotation();
	const FVector AxisWorld = ParentRot.RotateVector(Topo.BodyJointAxisLocal[RevoluteBody]).GetSafeNormal();

	// Rotating a joint carries its DESCENDANTS with it. Rotating body 2's
	// transform alone would leave body 3 behind, which reads as a large
	// off-axis error at body 3 -- and since the residual this test checks is a
	// MAX across every revolute, that would swamp the on-axis case being
	// measured. (This is not hypothetical: the first version of this test did
	// exactly that and reported 0.617 rad of "residual" for a perfectly
	// representable twist.)
	auto RotateSubtree = [&](const FQuat& Delta)
	{
		TArray<FTransform> Posed = RestCS;
		Posed[RevoluteBody].SetRotation((Delta * RestCS[RevoluteBody].GetRotation()).GetNormalized());
		Posed[ChildBody].SetRotation((Delta * RestCS[ChildBody].GetRotation()).GetNormalized());
		return Posed;
	};

	{
		const TArray<FTransform> Twisted = RotateSubtree(FQuat(AxisWorld, TargetAngle));

		FReferenceFrame Frame;
		float Residual = 0.0f;
		RetargetPoseToJointSpace(Topo, Twisted, TArray<int32>(), Frame, &Residual);

		TestTrue(FString::Printf(TEXT("a pure on-axis twist recovers its angle exactly (want %.4f, got %.4f)"), TargetAngle, Frame.DOFPos[RevoluteDOF]),
			FMath::IsNearlyEqual(Frame.DOFPos[RevoluteDOF], TargetAngle, 1e-4f));
		TestTrue(FString::Printf(TEXT("a pure on-axis twist leaves no residual (got %.5f)"), Residual), Residual < 1e-3f);
		TestTrue(FString::Printf(TEXT("twisting one joint leaves its child's angle at zero (got %.5f)"), Frame.DOFPos[Topo.BodyDOFOffset[ChildBody]]),
			FMath::IsNearlyZero(Frame.DOFPos[Topo.BodyDOFOffset[ChildBody]], 1e-4f));
	}

	{
		// Rotate about an axis perpendicular to the joint's -- motion this
		// 1-DOF joint cannot represent at all.
		const FVector OffAxis = FVector::CrossProduct(AxisWorld, FVector(0.0f, 0.0f, 1.0f)).GetSafeNormal();
		const TArray<FTransform> Bent = RotateSubtree(FQuat(OffAxis, FMath::DegreesToRadians(30.0f)));

		FReferenceFrame Frame;
		float Residual = 0.0f;
		RetargetPoseToJointSpace(Topo, Bent, TArray<int32>(), Frame, &Residual);

		TestTrue(FString::Printf(TEXT("an off-axis rotation is REPORTED as residual, not absorbed into the angle (got %.4f rad)"), Residual),
			Residual > FMath::DegreesToRadians(10.0f));
	}

	return true;
}

// ============================================================================
// 4. Reward response. Relative assertions only ("closer to the reference beats
//    further from it"), matching CreatureRLEnvironmentTest's own style -- the
//    absolute values depend on tuning constants that are expected to move.
// ============================================================================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCreatureImitationRewardTest, "AgentSolver.Imitation.Reward", EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FCreatureImitationRewardTest::RunTest(const FString& Parameters)
{
	using namespace CreatureImitationTest;
	using namespace CreatureImitation;

	const FCreatureTopology Topo = BuildChainTopology();
	const TArray<FTransform> RestCS = BuildRestBodyTransforms(Topo);

	FReferenceFrame Frame;
	RetargetPoseToJointSpace(Topo, RestCS, TArray<int32>(), Frame, nullptr);

	FCreatureBatchState Batch;
	Batch.Init(Topo, 1);
	FRandomStream Stream(7);
	CreatureRLEnvironment::ResetEnv(Batch, 0, FVector(0.0f, 0.0f, 100.0f), Topo.BodyRestRotInParent[0], Stream, 0.0f, 0.0f);

	FImitationConfig Imitation;

	// ResetEnv writes exactly the rest pose, which is what the rest pose
	// retargets to -- so this must be a perfect match.
	const float RewardAtRest = ComputePoseReward(Batch, 0, Frame, Imitation);
	TestTrue(FString::Printf(TEXT("ResetEnv's rest state scores 1.0 against the rest reference (got %.6f)"), RewardAtRest),
		FMath::IsNearlyEqual(RewardAtRest, 1.0f, 1e-4f));

	const float ErrorAtRest = ComputeMeanPoseErrorRad(Batch, 0, Frame);
	TestTrue(TEXT("mean pose error is zero at the reference"), ErrorAtRest < 1e-4f);

	// ---- Perturb one revolute; reward must fall, monotonically ----
	const int32 RevoluteDOF = Topo.BodyDOFOffset[2];
	Batch.JointPos[Batch.DOFIndex(RevoluteDOF, 0)] = 0.2f;
	const float RewardSmallError = ComputePoseReward(Batch, 0, Frame, Imitation);

	Batch.JointPos[Batch.DOFIndex(RevoluteDOF, 0)] = 0.8f;
	const float RewardLargeError = ComputePoseReward(Batch, 0, Frame, Imitation);

	TestTrue(TEXT("a small pose error scores below a perfect match"), RewardSmallError < RewardAtRest);
	TestTrue(TEXT("a large pose error scores below a small one"), RewardLargeError < RewardSmallError);
	TestTrue(TEXT("the pose reward stays positive even when badly wrong"), RewardLargeError > 0.0f);

	// ---- A ball joint's error must register too, via the quaternion path ----
	Batch.JointPos[Batch.DOFIndex(RevoluteDOF, 0)] = 0.0f;
	Batch.SetJointRelRot(1, 0, FQuat(FVector(0.0f, 0.0f, 1.0f), 0.5f));
	const float RewardBallError = ComputePoseReward(Batch, 0, Frame, Imitation);
	TestTrue(TEXT("a ball joint's error lowers the pose reward"), RewardBallError < RewardAtRest);

	// ---- Pose-error termination fires only past its threshold ----
	CreatureRLEnvironment::FEnvConfig Config;
	Config.ObjectiveMode = CreatureRLEnvironment::EObjectiveMode::Imitation;
	Config.GroundZ = 0.0f;
	Config.TargetTorsoHeight = 100.0f;
	Config.LocalUpAxis = Topo.BodyRestRotInParent[0].UnrotateVector(FVector::UpVector);
	Config.Imitation.bTerminateOnUprightAndHeight = false;
	Config.Imitation.MaxPoseErrorRad = 0.05f;

	CreatureRLEnvironment::FImitationTarget Target;
	Target.Frame = &Frame;
	Target.RestTorsoHeight = 100.0f;

	TestTrue(TEXT("a large pose error terminates the episode"),
		CreatureRLEnvironment::IsTerminated(Batch, 0, Config, nullptr, 0, 1, &Target));

	Batch.SetJointRelRot(1, 0, FQuat::Identity);
	TestFalse(TEXT("matching the reference does not terminate the episode"),
		CreatureRLEnvironment::IsTerminated(Batch, 0, Config, nullptr, 0, 1, &Target));

	// Disabling the threshold must disable the check, not fall through to a
	// default -- 0 means "off", and an episode that is wildly off-pose should
	// then run to its natural end.
	Config.Imitation.MaxPoseErrorRad = 0.0f;
	Batch.SetJointRelRot(1, 0, FQuat(FVector(0.0f, 0.0f, 1.0f), 1.2f));
	TestFalse(TEXT("MaxPoseErrorRad=0 disables pose-error termination"),
		CreatureRLEnvironment::IsTerminated(Batch, 0, Config, nullptr, 0, 1, &Target));

	return true;
}

// ============================================================================
// 5. Phase sampling. A single-frame motion must ignore phase entirely; a
//    multi-frame one must interpolate and wrap continuously.
// ============================================================================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCreatureImitationPhaseTest, "AgentSolver.Imitation.PhaseSampling", EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FCreatureImitationPhaseTest::RunTest(const FString& Parameters)
{
	using namespace CreatureImitationTest;
	using namespace CreatureImitation;

	const FCreatureTopology Topo = BuildChainTopology();
	const int32 RevoluteDOF = Topo.BodyDOFOffset[2];

	FReferenceMotion Motion;
	Motion.bLooping = true;
	Motion.SampleRate = 4.0f;
	Motion.Duration = 1.0f;
	Motion.Frames.SetNum(4);
	for (int32 F = 0; F < 4; ++F)
	{
		Motion.Frames[F].Init(Topo, 0);
		Motion.Frames[F].DOFPos[RevoluteDOF] = (float)F * 0.1f;
	}

	FReferenceFrame Sampled;
	Motion.SampleByPhase(0.0f, Topo, Sampled);
	TestTrue(TEXT("phase 0 lands on frame 0"), FMath::IsNearlyEqual(Sampled.DOFPos[RevoluteDOF], 0.0f, 1e-4f));

	Motion.SampleByPhase(0.25f, Topo, Sampled);
	TestTrue(TEXT("phase 0.25 lands on frame 1"), FMath::IsNearlyEqual(Sampled.DOFPos[RevoluteDOF], 0.1f, 1e-4f));

	Motion.SampleByPhase(0.125f, Topo, Sampled);
	TestTrue(TEXT("phase 0.125 interpolates halfway between frames 0 and 1"), FMath::IsNearlyEqual(Sampled.DOFPos[RevoluteDOF], 0.05f, 1e-4f));

	// A phase past 1 wraps rather than clamping -- an env whose episode
	// outlives the clip must keep cycling, not freeze on the last frame.
	FReferenceFrame Wrapped;
	Motion.SampleByPhase(1.25f, Topo, Wrapped);
	TestTrue(TEXT("phase wraps past 1.0"), FMath::IsNearlyEqual(Wrapped.DOFPos[RevoluteDOF], 0.1f, 1e-4f));

	// Velocities are finite-differenced, so a clip with real motion must
	// produce non-zero reference velocity somewhere.
	FillVelocitiesCentralDifference(Topo, Motion);
	bool bAnyVelocity = false;
	for (const FReferenceFrame& F : Motion.Frames)
	{
		bAnyVelocity |= !FMath::IsNearlyZero(F.DOFVel[RevoluteDOF], 1e-4f);
	}
	TestTrue(TEXT("a moving clip finite-differences to non-zero reference velocity"), bAnyVelocity);

	// A single-frame reference is the phase-1 case: a held pose, with zero
	// velocity by definition, regardless of what phase is asked for.
	FReferenceMotion SinglePose;
	SinglePose.Frames.SetNum(1);
	SinglePose.Frames[0].Init(Topo, 0);
	SinglePose.Frames[0].DOFPos[RevoluteDOF] = 0.42f;
	TestTrue(TEXT("a single-frame motion reports itself as such"), SinglePose.IsSingleFrame());

	FillVelocitiesCentralDifference(Topo, SinglePose);
	TestTrue(TEXT("a single-frame motion keeps zero reference velocity"), FMath::IsNearlyZero(SinglePose.Frames[0].DOFVel[RevoluteDOF]));

	SinglePose.SampleByPhase(0.73f, Topo, Sampled);
	TestTrue(TEXT("a single-frame motion ignores phase"), FMath::IsNearlyEqual(Sampled.DOFPos[RevoluteDOF], 0.42f, 1e-4f));

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
