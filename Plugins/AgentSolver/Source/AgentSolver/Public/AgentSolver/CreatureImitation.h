#pragma once

// Imitation-learning glue: a reference pose/motion expressed in the SOLVER's
// OWN joint coordinates, plus the DeepMimic-style reward terms that score how
// closely FCreatureBatchState currently matches it.
//
// Deliberately free of any animation/UObject dependency -- everything here is
// plain C++ over FCreatureTopology/FCreatureBatchState and TArray<FTransform>,
// testable exactly the way CreatureRLEnvironment.h is (see
// Tests/CreatureImitationTest.cpp). Turning a UAnimSequence into an
// FReferenceMotion is a separate, WITH_EDITOR-only concern -- see
// ImitationBake.h.
//
// WHY JOINT-LOCAL, NOT WORLD SPACE
//
// The pose error is measured per joint, in the joint's own local frame, not by
// comparing world body orientations. That is the same choice DeepMimic makes,
// for the same reason: a world-space comparison double-counts error, because a
// torso tilted 10 degrees makes every one of its 40 descendants read as wrong
// even when every individual joint is perfect. Joint-local error attributes the
// mistake to the one joint that made it.
//
// It also happens to be free. The solver already stores exactly the two
// quantities this needs, per joint, with no derivation at all:
//   - revolute (BodyDOFCount == 1): Batch.JointPos[DOF], an angle in radians
//   - ball     (BodyDOFCount == 3): Batch.GetJointRelRot(Body, Env), the
//     joint-local relative quaternion (the DRIVING state; JointPos merely
//     mirrors its rotation vector -- see CreatureBatchSolver.h's Pass 3b)
// so the whole problem reduces to converting a reference pose into those same
// coordinates ONCE, offline, which is what RetargetPoseToJointSpace does.
//
// THE RETARGET, AND WHY IT IS AN INVERSE OF PASS 1 AND NOT A GUESS
//
// CreatureBatchSolver.h's Pass 1 defines forward kinematics as, with
// JointFrame = ParentRot * BodyRestRotInParent[b]:
//     ball:     BodyRot = JointFrame * RelRot
//     revolute: BodyRot = FQuat(AxisWorld, theta) * JointFrame
//               AxisWorld = ParentRot.RotateVector(BodyJointAxisLocal[b])
// Note the two conventions genuinely differ -- the ball joint's rotation is
// POST-multiplied (it lives in the joint's local frame) while the revolute's
// spin is PRE-multiplied (world/component frame, outermost). Inverting each
// gives exactly what RetargetPoseToJointSpace below computes.
//
// This yields a free correctness invariant worth stating explicitly, because
// it is what makes the whole file trustworthy: feeding the rig's own REST pose
// through the retarget must produce theta == 0 for every revolute and identity
// for every ball joint -- precisely the state CreatureRLEnvironment::ResetEnv
// already writes for a fresh episode. If that does not hold, the conventions
// have drifted and nothing downstream means anything. It is the first test in
// Tests/CreatureImitationTest.cpp.

#include "CoreMinimal.h"
#include "PhysicsSolver/CreatureBatchState.h"

namespace CreatureImitation
{
	/**
	 * One reference frame, already converted into the solver's joint
	 * coordinates. Everything here is directly comparable against
	 * FCreatureBatchState with no further transformation.
	 */
	struct FReferenceFrame
	{
		/** Body 0's pose in the SOURCE's component space -- see RootHeightAboveRest for why the raw Z is not what the reward compares. */
		FVector RootPos = FVector::ZeroVector;
		FQuat RootRot = FQuat::Identity;

		/**
		 * Body 0's height RELATIVE TO the same rig's rest pose, which is what
		 * the root reward actually compares. The raw component-space RootPos.Z
		 * is NOT comparable against the batch's world Z: the batch stands at
		 * Config.TargetTorsoHeight, derived from contact geometry by
		 * AMutoRLTrainingDriver::ComputeDefaultStandingHeight, while the
		 * animation's component-space Z comes from wherever the source skeleton
		 * happens to sit. Only the DEVIATION from each one's own rest height is
		 * meaningful across the two, so a crouch in the clip reads as "be 20cm
		 * below your nominal standing height" rather than as an absolute
		 * altitude the rig may have no way to reach.
		 */
		float RootHeightAboveRest = 0.0f;

		/**
		 * Root velocities in the ROOT'S OWN frame, not world/component space.
		 * Heading-invariant by construction, so a locomotion clip scores the
		 * same whichever way the episode happens to be facing -- without this
		 * the policy would be penalised for walking north when the clip was
		 * captured walking east.
		 */
		FVector RootLinVelLocal = FVector::ZeroVector;
		FVector RootAngVelLocal = FVector::ZeroVector;

		/** NumDOF. Same layout/meaning as Batch.JointPos: radians for revolutes, rotation-vector components for ball joints. */
		TArray<float> DOFPos;

		/** NumDOF. Same layout/meaning as Batch.JointVel: rad/s for revolutes, WORLD-frame joint angular velocity for ball joints (see Pass 1's JointAngVelWorld). */
		TArray<float> DOFVel;

		/** NumBodies. The ball joints' driving state; identity for the root and every revolute. This, not DOFPos, is what the pose reward compares for ball joints -- see ComputeMeanPoseErrorRad. */
		TArray<FQuat> BodyRelRot;

		/** NumBodies. Component-space body rotations, kept so velocities can be finite-differenced after the fact (see FillVelocitiesCentralDifference). Not read by any reward term. */
		TArray<FQuat> BodyRotCS;

		/** Parallel to FReferenceMotion::EndEffectorBodies. Stored ROOT-LOCAL (root rotation and translation removed) so the term is translation- and heading-invariant. */
		TArray<FVector> EndEffectorPosLocal;

		void Init(const FCreatureTopology& Topo, int32 NumEndEffectors)
		{
			DOFPos.Init(0.0f, Topo.NumDOF);
			DOFVel.Init(0.0f, Topo.NumDOF);
			BodyRelRot.Init(FQuat::Identity, Topo.NumBodies);
			BodyRotCS.Init(FQuat::Identity, Topo.NumBodies);
			EndEffectorPosLocal.Init(FVector::ZeroVector, NumEndEffectors);
		}

		bool IsCompatibleWith(const FCreatureTopology& Topo) const
		{
			return DOFPos.Num() == Topo.NumDOF
				&& DOFVel.Num() == Topo.NumDOF
				&& BodyRelRot.Num() == Topo.NumBodies;
		}
	};

	/** q and -q are the same rotation; ToRotationVector()/GetAngle() only behave on the W>=0 representative. Same canonicalization Pass 3b applies to JointRelRot. */
	inline FQuat Canonicalize(const FQuat& Q)
	{
		return (Q.W < 0.0f) ? FQuat(-Q.X, -Q.Y, -Q.Z, -Q.W) : Q;
	}

	/**
	 * Shortest-arc angle between two rotations, in [0, PI]. Robust where a
	 * rotation-vector difference is not -- see ClampJointLimits' 2026-08-12
	 * note on why per-axis rotation-vector math is unsound here.
	 *
	 * atan2(|v|, |w|) of the RELATIVE rotation, not the more obvious
	 * 2*acos(|dot(A,B)|). Those are equal in exact arithmetic, but acos is
	 * catastrophically ill-conditioned exactly where this function spends all
	 * its time: near a perfect match its derivative is unbounded, so a float32
	 * dot product of 1 - 1e-8 -- ordinary rounding noise for a quaternion
	 * composed through a few bodies -- comes back as ~2.8e-4 rad of entirely
	 * fictitious error. Since the pose reward is exp(-scale * mean SQUARED
	 * error) and the interesting region is error ~ 0, that noise floor is the
	 * one place accuracy actually matters. atan2 is well-conditioned there.
	 *
	 * Caught by the FK round-trip test, which compared two FK implementations
	 * that agreed to the last bit and still measured them 2.8e-4 rad apart.
	 */
	inline float GeodesicAngleRad(const FQuat& A, const FQuat& B)
	{
		const FQuat Delta = A.Inverse() * B;
		const float VectorLength = (float)FVector(Delta.X, Delta.Y, Delta.Z).Size();
		// |W|, not W: q and -q are the same rotation, and the absolute value is
		// what picks the short way round.
		return 2.0f * FMath::Atan2(VectorLength, FMath::Abs((float)Delta.W));
	}

	/**
	 * A baked reference pose (one frame) or motion (many), plus the metadata
	 * needed to index it by time.
	 */
	struct FReferenceMotion
	{
		TArray<FReferenceFrame> Frames;

		/** Body indices whose positions the end-effector reward tracks (feet/hands). */
		TArray<int32> EndEffectorBodies;

		float SampleRate = 30.0f;
		float Duration = 0.0f;
		bool bLooping = true;

		/**
		 * Worst-case angle, across every revolute joint and every frame, that
		 * the source pose asked a 1-DOF joint to rotate OFF its own axis --
		 * motion the retarget necessarily discards, since a revolute has
		 * nowhere to put it (see RetargetPoseToJointSpace's twist projection).
		 *
		 * Diagnostic, never consumed by a reward. It is the honest measure of
		 * "how well does this clip actually fit this rig": a large value means
		 * the reference is asking for motion the topology cannot represent, so
		 * a pose reward that plateaus below 1.0 is the RIG's limit, not the
		 * policy's failure. Logged by the bake rather than silently absorbed.
		 */
		float MaxRevoluteResidualRad = 0.0f;

		bool IsValid() const { return Frames.Num() > 0; }
		bool IsSingleFrame() const { return Frames.Num() == 1; }

		/** Phase in [0,1) for a given time, honouring bLooping. */
		float TimeToPhase(float Time) const
		{
			if (Duration <= KINDA_SMALL_NUMBER)
			{
				return 0.0f;
			}
			const float T = bLooping ? FMath::Fmod(FMath::Fmod(Time, Duration) + Duration, Duration) : FMath::Clamp(Time, 0.0f, Duration);
			return FMath::Clamp(T / Duration, 0.0f, 1.0f);
		}

		/**
		 * Samples the motion at a phase in [0,1), interpolating between the two
		 * bracketing frames. Rotations go through Slerp; DOFPos for BALL joints
		 * is then rebuilt from the slerped quaternion rather than lerped
		 * component-wise, because rotation-vector components do not interpolate
		 * independently (the same unsoundness ClampJointLimits ran into).
		 */
		void SampleByPhase(float Phase, const FCreatureTopology& Topo, FReferenceFrame& Out) const
		{
			check(IsValid());
			if (Frames.Num() == 1)
			{
				Out = Frames[0];
				return;
			}

			const float Wrapped = FMath::Fmod(FMath::Fmod(Phase, 1.0f) + 1.0f, 1.0f);
			// A looping clip's last frame blends back into its first; a
			// non-looping one clamps at the end instead of snapping home.
			const int32 Span = bLooping ? Frames.Num() : Frames.Num() - 1;
			const float Scaled = Wrapped * (float)Span;
			const int32 IndexA = FMath::Clamp((int32)FMath::FloorToInt(Scaled), 0, Frames.Num() - 1);
			const int32 IndexB = bLooping ? ((IndexA + 1) % Frames.Num()) : FMath::Min(IndexA + 1, Frames.Num() - 1);
			const float Alpha = FMath::Clamp(Scaled - (float)IndexA, 0.0f, 1.0f);

			const FReferenceFrame& A = Frames[IndexA];
			const FReferenceFrame& B = Frames[IndexB];

			Out.Init(Topo, EndEffectorBodies.Num());
			Out.RootPos = FMath::Lerp(A.RootPos, B.RootPos, Alpha);
			Out.RootRot = Canonicalize(FQuat::Slerp(A.RootRot, B.RootRot, Alpha).GetNormalized());
			Out.RootHeightAboveRest = FMath::Lerp(A.RootHeightAboveRest, B.RootHeightAboveRest, Alpha);
			Out.RootLinVelLocal = FMath::Lerp(A.RootLinVelLocal, B.RootLinVelLocal, Alpha);
			Out.RootAngVelLocal = FMath::Lerp(A.RootAngVelLocal, B.RootAngVelLocal, Alpha);

			for (int32 Body = 0; Body < Topo.NumBodies; ++Body)
			{
				Out.BodyRelRot[Body] = Canonicalize(FQuat::Slerp(A.BodyRelRot[Body], B.BodyRelRot[Body], Alpha).GetNormalized());
				Out.BodyRotCS[Body] = Canonicalize(FQuat::Slerp(A.BodyRotCS[Body], B.BodyRotCS[Body], Alpha).GetNormalized());
			}

			for (int32 DOF = 0; DOF < Topo.NumDOF; ++DOF)
			{
				Out.DOFVel[DOF] = FMath::Lerp(A.DOFVel[DOF], B.DOFVel[DOF], Alpha);
			}

			for (int32 Body = 1; Body < Topo.NumBodies; ++Body)
			{
				const int32 DOFOffset = Topo.BodyDOFOffset[Body];
				if (Topo.BodyDOFCount[Body] == 3)
				{
					const FVector RotVec = Out.BodyRelRot[Body].ToRotationVector();
					Out.DOFPos[DOFOffset + 0] = (float)RotVec.X;
					Out.DOFPos[DOFOffset + 1] = (float)RotVec.Y;
					Out.DOFPos[DOFOffset + 2] = (float)RotVec.Z;
				}
				else if (Topo.BodyDOFCount[Body] == 1)
				{
					// Interpolate the SHORT way round -- a joint sitting near
					// +/-PI would otherwise sweep the long way through zero.
					const float Delta = FMath::UnwindRadians(B.DOFPos[DOFOffset] - A.DOFPos[DOFOffset]);
					Out.DOFPos[DOFOffset] = FMath::UnwindRadians(A.DOFPos[DOFOffset] + Delta * Alpha);
				}
			}

			for (int32 EE = 0; EE < EndEffectorBodies.Num(); ++EE)
			{
				Out.EndEffectorPosLocal[EE] = FMath::Lerp(A.EndEffectorPosLocal[EE], B.EndEffectorPosLocal[EE], Alpha);
			}
		}
	};

	/**
	 * Converts a component-space pose (one FTransform per BODY -- the caller
	 * does any bone->body mapping, see ImitationBake.h) into the solver's joint
	 * coordinates. The exact inverse of CreatureBatchSolver.h's Pass 1; see
	 * this file's header comment for the derivation.
	 *
	 * Takes BODY transforms rather than bone transforms deliberately: it keeps
	 * this function free of any skeleton dependency, so it is exercisable
	 * against the same synthetic test rigs the rest of the plugin uses, where
	 * FCreatureTopology::BodyBoneIndex is INDEX_NONE throughout.
	 *
	 * EndEffectorPosLocal is stored root-local (root rotation AND translation
	 * removed) so the end-effector reward is translation- and heading-invariant.
	 */
	inline void RetargetPoseToJointSpace(
		const FCreatureTopology& Topo,
		const TArray<FTransform>& BodyCS,
		const TArray<int32>& EndEffectorBodies,
		FReferenceFrame& OutFrame,
		float* OutMaxRevoluteResidualRad = nullptr)
	{
		check(BodyCS.Num() == Topo.NumBodies);
		OutFrame.Init(Topo, EndEffectorBodies.Num());

		const FQuat RootRot = BodyCS[0].GetRotation().GetNormalized();
		OutFrame.RootPos = BodyCS[0].GetTranslation();
		OutFrame.RootRot = Canonicalize(RootRot);
		OutFrame.BodyRotCS[0] = OutFrame.RootRot;

		float MaxResidual = 0.0f;

		for (int32 Body = 1; Body < Topo.NumBodies; ++Body)
		{
			const int32 Parent = Topo.BodyParent[Body];
			const FQuat ParentRot = BodyCS[Parent].GetRotation().GetNormalized();
			const FQuat BodyRot = BodyCS[Body].GetRotation().GetNormalized();
			OutFrame.BodyRotCS[Body] = Canonicalize(BodyRot);

			// The frame the joint's own rotation lives in -- ParentRot alone is
			// NOT it. Getting this wrong by exactly the bind-pose rotation is
			// the same class of error SOLVER_DEBUG_LOG.md entry 012 records
			// against the ball-joint integrator.
			const FQuat JointFrame = (ParentRot * Topo.BodyRestRotInParent[Body]).GetNormalized();
			const int32 DOFOffset = Topo.BodyDOFOffset[Body];

			if (Topo.BodyDOFCount[Body] == 3)
			{
				// Pass 1: BodyRot = JointFrame * RelRot  ->  RelRot = JointFrame^-1 * BodyRot
				const FQuat RelRot = Canonicalize((JointFrame.Inverse() * BodyRot).GetNormalized());
				OutFrame.BodyRelRot[Body] = RelRot;
				const FVector RotVec = RelRot.ToRotationVector();
				OutFrame.DOFPos[DOFOffset + 0] = (float)RotVec.X;
				OutFrame.DOFPos[DOFOffset + 1] = (float)RotVec.Y;
				OutFrame.DOFPos[DOFOffset + 2] = (float)RotVec.Z;
			}
			else if (Topo.BodyDOFCount[Body] == 1)
			{
				// Pass 1: BodyRot = Spin * JointFrame  ->  Spin = BodyRot * JointFrame^-1,
				// with Spin ideally a pure rotation about AxisWorld. A real
				// animation pose will not sit exactly on that 1-DOF manifold, so
				// take the TWIST component about the axis (standard swing-twist
				// decomposition) and record the discarded swing as residual.
				const FVector AxisWorld = ParentRot.RotateVector(Topo.BodyJointAxisLocal[Body]).GetSafeNormal();
				const FQuat Spin = Canonicalize((BodyRot * JointFrame.Inverse()).GetNormalized());

				if (AxisWorld.IsNearlyZero())
				{
					// No authored axis (synthetic rigs, or a body whose axis was
					// never populated) -- nothing meaningful to project onto.
					OutFrame.DOFPos[DOFOffset] = 0.0f;
					continue;
				}

				const FVector SpinVec(Spin.X, Spin.Y, Spin.Z);
				const double Projection = FVector::DotProduct(SpinVec, AxisWorld);
				OutFrame.DOFPos[DOFOffset] = FMath::UnwindRadians(2.0f * (float)FMath::Atan2(Projection, (double)Spin.W));

				// Residual = the part of Spin that is NOT about the axis.
				FQuat Twist((float)(AxisWorld.X * Projection), (float)(AxisWorld.Y * Projection), (float)(AxisWorld.Z * Projection), Spin.W);
				if (Twist.SizeSquared() > SMALL_NUMBER)
				{
					Twist.Normalize();
					MaxResidual = FMath::Max(MaxResidual, GeodesicAngleRad(Spin, Twist));
				}
			}
		}

		for (int32 EE = 0; EE < EndEffectorBodies.Num(); ++EE)
		{
			const int32 Body = EndEffectorBodies[EE];
			if (BodyCS.IsValidIndex(Body))
			{
				OutFrame.EndEffectorPosLocal[EE] = RootRot.UnrotateVector(BodyCS[Body].GetTranslation() - OutFrame.RootPos);
			}
		}

		if (OutMaxRevoluteResidualRad)
		{
			*OutMaxRevoluteResidualRad = MaxResidual;
		}
	}

	/**
	 * Fills DOFVel/RootLinVelLocal/RootAngVelLocal across an already-retargeted
	 * frame sequence by central differences over BodyRotCS/RootPos. A
	 * single-frame motion is left at zero velocity throughout, which is the
	 * correct reference for "hold this pose".
	 *
	 * Ball-joint DOFVel is written in the WORLD frame, matching what Pass 1
	 * reads out of Batch.JointVel for a ball joint (JointAngVelWorld), and is
	 * the joint's OWN contribution -- body angular velocity minus the parent's
	 * -- not the body's total.
	 */
	inline void FillVelocitiesCentralDifference(const FCreatureTopology& Topo, FReferenceMotion& Motion)
	{
		const int32 NumFrames = Motion.Frames.Num();
		if (NumFrames < 2 || Motion.SampleRate <= KINDA_SMALL_NUMBER)
		{
			return;
		}

		TArray<FVector> BodyOmega;
		BodyOmega.SetNumZeroed(Topo.NumBodies);

		for (int32 F = 0; F < NumFrames; ++F)
		{
			// Central difference where both neighbours exist; one-sided at the
			// ends of a non-looping clip.
			const int32 Prev = Motion.bLooping ? ((F - 1 + NumFrames) % NumFrames) : FMath::Max(F - 1, 0);
			const int32 Next = Motion.bLooping ? ((F + 1) % NumFrames) : FMath::Min(F + 1, NumFrames - 1);
			const int32 NumIntervals = (Prev == F || Next == F) ? 1 : 2;
			const float Dt = (float)NumIntervals / Motion.SampleRate;
			if (Prev == Next)
			{
				continue;
			}

			const FReferenceFrame& A = Motion.Frames[Prev];
			const FReferenceFrame& B = Motion.Frames[Next];
			FReferenceFrame& Cur = Motion.Frames[F];

			for (int32 Body = 0; Body < Topo.NumBodies; ++Body)
			{
				// Rotations compose on the LEFT in world/component space, so the
				// delta that carries A to B is B * A^-1 and its rotation vector
				// is already a world-frame angular displacement.
				const FQuat Delta = Canonicalize((B.BodyRotCS[Body] * A.BodyRotCS[Body].Inverse()).GetNormalized());
				BodyOmega[Body] = Delta.ToRotationVector() / Dt;
			}

			Cur.RootAngVelLocal = Cur.RootRot.UnrotateVector(BodyOmega[0]);
			Cur.RootLinVelLocal = Cur.RootRot.UnrotateVector((B.RootPos - A.RootPos) / Dt);

			for (int32 Body = 1; Body < Topo.NumBodies; ++Body)
			{
				const int32 DOFOffset = Topo.BodyDOFOffset[Body];
				if (Topo.BodyDOFCount[Body] == 3)
				{
					const FVector JointOmega = BodyOmega[Body] - BodyOmega[Topo.BodyParent[Body]];
					Cur.DOFVel[DOFOffset + 0] = (float)JointOmega.X;
					Cur.DOFVel[DOFOffset + 1] = (float)JointOmega.Y;
					Cur.DOFVel[DOFOffset + 2] = (float)JointOmega.Z;
				}
				else if (Topo.BodyDOFCount[Body] == 1)
				{
					Cur.DOFVel[DOFOffset] = FMath::UnwindRadians(B.DOFPos[DOFOffset] - A.DOFPos[DOFOffset]) / Dt;
				}
			}
		}
	}

	/** Per-joint weights and exponential falloff rates for the imitation reward. Defaults follow DeepMimic's published weighting (0.65/0.10/0.15/0.10). */
	struct FImitationConfig
	{
		float PoseWeight = 0.65f;
		float VelocityWeight = 0.10f;
		float EndEffectorWeight = 0.15f;
		float RootWeight = 0.10f;

		// Every term below is exp(-Scale * meanSquaredError), so each lands in
		// (0,1] and reads directly on the control panel's line graphs without
		// further normalization. Errors are MEANS, not DeepMimic's sums, so
		// these scales stay meaningful if the rig's joint count changes.
		float PoseErrorScale = 2.0f;         // mean squared radians
		float VelocityErrorScale = 0.1f;     // mean squared rad/s
		float EndEffectorErrorScale = 40.0f; // mean squared distance / creature scale
		float RootErrorScale = 10.0f;        // height + orientation + linear velocity, all normalized

		/**
		 * Mean per-joint pose error (radians) above which the episode ends.
		 * 0 disables it. This is imitation's analogue of the standing
		 * objective's fallen-over check: once the pose has diverged far enough
		 * there is nothing left to learn from the rest of the episode, and
		 * ending it early is what keeps the replay buffer full of states the
		 * reference actually visits.
		 */
		float MaxPoseErrorRad = 0.0f;

		/**
		 * Whether imitation ALSO terminates on the standing objective's
		 * upright/height thresholds. On by default, since most reference
		 * motions are upright -- turn it off for a clip that legitimately puts
		 * the torso on the ground (a crawl, a roll, a getting-up motion), where
		 * those thresholds would end every episode before it starts.
		 */
		bool bTerminateOnUprightAndHeight = true;
	};

	/**
	 * Mean per-joint angular error, in radians, between the batch's current
	 * state and a reference frame. Shared by the pose reward and the
	 * pose-error termination check so the two can never disagree.
	 *
	 * Revolutes compare JointPos through UnwindRadians; ball joints compare
	 * the RelRot quaternions geodesically rather than differencing their
	 * rotation vectors -- rotation-vector components are not independent
	 * coordinates, and treating them as such is exactly the unsoundness
	 * ClampJointLimits had to be rewritten to avoid (see CreatureBatchSolver.h,
	 * 2026-08-12).
	 *
	 * Counts one error per JOINT (per body), not per DOF, so a 3-DOF ball joint
	 * does not outweigh three separate revolutes.
	 */
	inline float ComputeMeanPoseErrorRad(
		const FCreatureBatchState& Batch,
		int32 Env,
		const FReferenceFrame& Frame,
		float* OutMeanSquaredError = nullptr)
	{
		const FCreatureTopology& Topo = Batch.GetTopology();
		float SumAbs = 0.0f;
		float SumSquared = 0.0f;
		int32 Count = 0;

		for (int32 Body = 1; Body < Topo.NumBodies; ++Body)
		{
			const int32 DOFOffset = Topo.BodyDOFOffset[Body];
			float Error = 0.0f;

			if (Topo.BodyDOFCount[Body] == 3)
			{
				Error = GeodesicAngleRad(Batch.GetJointRelRot(Body, Env).GetNormalized(), Frame.BodyRelRot[Body]);
			}
			else if (Topo.BodyDOFCount[Body] == 1)
			{
				Error = FMath::Abs(FMath::UnwindRadians(Batch.JointPos[Batch.DOFIndex(DOFOffset, Env)] - Frame.DOFPos[DOFOffset]));
			}
			else
			{
				continue;
			}

			if (!FMath::IsFinite(Error))
			{
				// A blowup can reach here for the one step before IsTerminated
				// forces a reset -- same race CreatureRLEnvironment::ComputeReward
				// documents. Treat it as maximally wrong rather than poisoning
				// the mean with a NaN.
				Error = PI;
			}

			SumAbs += Error;
			SumSquared += Error * Error;
			++Count;
		}

		if (Count == 0)
		{
			if (OutMeanSquaredError) { *OutMeanSquaredError = 0.0f; }
			return 0.0f;
		}

		if (OutMeanSquaredError) { *OutMeanSquaredError = SumSquared / (float)Count; }
		return SumAbs / (float)Count;
	}

	/** exp(-PoseErrorScale * mean squared joint error). 1.0 exactly at the reference pose. */
	inline float ComputePoseReward(const FCreatureBatchState& Batch, int32 Env, const FReferenceFrame& Frame, const FImitationConfig& Imitation)
	{
		float MeanSquared = 0.0f;
		ComputeMeanPoseErrorRad(Batch, Env, Frame, &MeanSquared);
		return FMath::Exp(-Imitation.PoseErrorScale * MeanSquared);
	}

	/**
	 * exp(-VelocityErrorScale * mean squared joint velocity error). Compares
	 * per DOF (not per joint) since JointVel is a flat per-DOF channel in both
	 * conventions -- ball joints' three entries are a world-frame angular
	 * velocity, revolutes' single entry is a scalar rate.
	 */
	inline float ComputeVelocityReward(const FCreatureBatchState& Batch, int32 Env, const FReferenceFrame& Frame, const FImitationConfig& Imitation)
	{
		const FCreatureTopology& Topo = Batch.GetTopology();
		if (Topo.NumDOF == 0)
		{
			return 1.0f;
		}

		float SumSquared = 0.0f;
		for (int32 DOF = 0; DOF < Topo.NumDOF; ++DOF)
		{
			const float Error = Batch.JointVel[Batch.DOFIndex(DOF, Env)] - Frame.DOFVel[DOF];
			SumSquared += FMath::IsFinite(Error) ? Error * Error : 0.0f;
		}
		return FMath::Exp(-Imitation.VelocityErrorScale * (SumSquared / (float)Topo.NumDOF));
	}

	/**
	 * exp(-EndEffectorErrorScale * mean squared root-local end-effector
	 * displacement), with distances divided by CreatureScale so the falloff is
	 * tied to the rig's own size rather than to world units -- the same
	 * normalization CreatureRLEnvironment::ComputeReward's balance term already
	 * uses TargetTorsoHeight for.
	 *
	 * Root-local on both sides, so this measures limb placement RELATIVE TO the
	 * body and never double-counts a root error the root term already scores.
	 */
	inline float ComputeEndEffectorReward(
		const FCreatureBatchState& Batch,
		int32 Env,
		const FReferenceFrame& Frame,
		const TArray<int32>& EndEffectorBodies,
		float CreatureScale,
		const FImitationConfig& Imitation)
	{
		if (EndEffectorBodies.Num() == 0)
		{
			return 1.0f;
		}

		const float SafeScale = FMath::Max(CreatureScale, KINDA_SMALL_NUMBER);
		const FQuat RootRot = Batch.GetBodyRot(0, Env).GetNormalized();
		const FVector RootPos = Batch.GetBodyPos(0, Env);

		// Guard rather than check(): the frame and the body list come from the
		// same FReferenceMotion in every real path, but a caller that mixes a
		// frame from one motion with another's body list would otherwise index
		// out of bounds rather than simply scoring badly.
		const int32 NumEndEffectors = FMath::Min(EndEffectorBodies.Num(), Frame.EndEffectorPosLocal.Num());
		if (NumEndEffectors == 0)
		{
			return 1.0f;
		}

		float SumSquared = 0.0f;
		for (int32 EE = 0; EE < NumEndEffectors; ++EE)
		{
			const int32 Body = EndEffectorBodies[EE];
			const FVector ActualLocal = RootRot.UnrotateVector(Batch.GetBodyPos(Body, Env) - RootPos);
			const float Error = (float)FVector::Dist(ActualLocal, Frame.EndEffectorPosLocal[EE]) / SafeScale;
			SumSquared += FMath::IsFinite(Error) ? Error * Error : 1.0f;
		}
		return FMath::Exp(-Imitation.EndEffectorErrorScale * (SumSquared / (float)NumEndEffectors));
	}

	/**
	 * exp(-RootErrorScale * (height^2 + orientation^2 + linear velocity^2)),
	 * each normalized:
	 *  - height   : deviation-from-rest difference / CreatureScale (see
	 *               FReferenceFrame::RootHeightAboveRest for why absolute Z is
	 *               not comparable across the two coordinate systems)
	 *  - rotation : geodesic angle in radians
	 *  - velocity : root-local linear velocity difference / LinVelocityScale,
	 *               the same 200 cm/s reference ComputeObservations uses
	 *
	 * The velocity component is what gives a locomotion clip any pull toward
	 * actually travelling: the pose and end-effector terms are both
	 * heading-invariant and root-local, so on their own they are perfectly
	 * satisfied by a creature cycling its legs while standing still.
	 *
	 * Comparing Frame.RootRot (the SOURCE's component space) against the
	 * batch's WORLD root rotation is valid, and not an oversight: an env is
	 * reset with its torso at StandingTorsoRot == BodyRestRotInParent[0], which
	 * IS the skeleton's own component-space rest rotation (see
	 * AMutoRLTrainingDriver::StartTraining), so the two frames coincide by
	 * construction. Root POSITION does not enjoy that luxury, which is exactly
	 * why height goes through RootHeightAboveRest instead.
	 */
	inline float ComputeRootReward(
		const FCreatureBatchState& Batch,
		int32 Env,
		const FReferenceFrame& Frame,
		float GroundZ,
		float RestTorsoHeight,
		float CreatureScale,
		const FImitationConfig& Imitation)
	{
		constexpr float LinVelocityScale = 200.0f;
		const float SafeScale = FMath::Max(CreatureScale, KINDA_SMALL_NUMBER);

		const FQuat RootRot = Batch.GetBodyRot(0, Env).GetNormalized();
		const int32 RootIdx = Batch.BodyIndex(0, Env);

		const float ActualHeightAboveRest = (float)Batch.GetBodyPos(0, Env).Z - GroundZ - RestTorsoHeight;
		const float HeightError = (ActualHeightAboveRest - Frame.RootHeightAboveRest) / SafeScale;

		const float RotError = GeodesicAngleRad(RootRot, Frame.RootRot);

		const FVector ActualLinVelLocal = RootRot.UnrotateVector(
			FVector(Batch.LinVelX[RootIdx], Batch.LinVelY[RootIdx], Batch.LinVelZ[RootIdx]));
		const float VelError = (float)FVector::Dist(ActualLinVelLocal, Frame.RootLinVelLocal) / LinVelocityScale;

		const float SumSquared = HeightError * HeightError + RotError * RotError + VelError * VelError;
		return FMath::Exp(-Imitation.RootErrorScale * (FMath::IsFinite(SumSquared) ? SumSquared : 100.0f));
	}

	/**
	 * The four terms combined by their weights. Each Out* pointer is optional
	 * and lets the control panel's graphs read a component without recomputing
	 * it and risking drift from this function -- the same pattern
	 * CreatureRLEnvironment::ComputeReward already uses for its own terms.
	 */
	inline float ComputeImitationReward(
		const FCreatureBatchState& Batch,
		int32 Env,
		const FReferenceFrame& Frame,
		const TArray<int32>& EndEffectorBodies,
		float GroundZ,
		float RestTorsoHeight,
		float CreatureScale,
		const FImitationConfig& Imitation,
		float* OutPoseReward = nullptr,
		float* OutVelocityReward = nullptr,
		float* OutEndEffectorReward = nullptr,
		float* OutRootReward = nullptr)
	{
		const float PoseReward = ComputePoseReward(Batch, Env, Frame, Imitation);
		const float VelocityReward = ComputeVelocityReward(Batch, Env, Frame, Imitation);
		const float EndEffectorReward = ComputeEndEffectorReward(Batch, Env, Frame, EndEffectorBodies, CreatureScale, Imitation);
		const float RootReward = ComputeRootReward(Batch, Env, Frame, GroundZ, RestTorsoHeight, CreatureScale, Imitation);

		if (OutPoseReward) { *OutPoseReward = PoseReward; }
		if (OutVelocityReward) { *OutVelocityReward = VelocityReward; }
		if (OutEndEffectorReward) { *OutEndEffectorReward = EndEffectorReward; }
		if (OutRootReward) { *OutRootReward = RootReward; }

		return Imitation.PoseWeight * PoseReward
			+ Imitation.VelocityWeight * VelocityReward
			+ Imitation.EndEffectorWeight * EndEffectorReward
			+ Imitation.RootWeight * RootReward;
	}

	/**
	 * Forward kinematics for ONE env: body world pos/rot/velocities from the
	 * current root pose and joint state. Exactly CreatureBatchSolver.h's Pass 1
	 * restricted to a single env, with no integration and no joint-limit clamp.
	 *
	 * A deliberate duplicate of that pass, for the same reason
	 * CreatureRLEnvironment::ComputeMuscleStrengthAtCurrentAngle duplicates the
	 * solver's ComputeMuscleMultipliers: the solver's own
	 * FCreatureABASolver::RecomputeKinematics is whole-BATCH (a ParallelFor over
	 * every env), and RSI needs to refresh exactly one env, once per episode
	 * reset -- calling the batch version per reset would redo all 256 envs up to
	 * 256 times per training step. Reads nothing the solver keeps private.
	 *
	 * If Pass 1's conventions ever change, this must change with it; the FK
	 * round-trip test in Tests/CreatureImitationTest.cpp is what catches the
	 * drift, by checking this against the solver's own result.
	 */
	inline void RecomputeEnvKinematics(FCreatureBatchState& Batch, int32 Env)
	{
		const FCreatureTopology& Topo = Batch.GetTopology();
		for (int32 Body = 1; Body < Topo.NumBodies; ++Body)
		{
			const int32 Parent = Topo.BodyParent[Body];
			const int32 Idx = Batch.BodyIndex(Body, Env);
			const int32 ParentIdx = Batch.BodyIndex(Parent, Env);

			const FQuat ParentRot = Batch.GetBodyRot(Parent, Env);
			const FVector ParentPos = Batch.GetBodyPos(Parent, Env);
			const FVector ParentAngVel(Batch.AngVelX[ParentIdx], Batch.AngVelY[ParentIdx], Batch.AngVelZ[ParentIdx]);
			const FVector ParentLinVel(Batch.LinVelX[ParentIdx], Batch.LinVelY[ParentIdx], Batch.LinVelZ[ParentIdx]);

			const int32 DOFIdx = Batch.DOFIndex(Topo.BodyDOFOffset[Body], Env);
			const int32 Stride = Batch.GetPaddedNumEnvs();

			FQuat BodyRot;
			FVector JointAngVelWorld;
			if (Topo.BodyDOFCount[Body] == 3)
			{
				BodyRot = (ParentRot * Topo.BodyRestRotInParent[Body] * Batch.GetJointRelRot(Body, Env)).GetNormalized();
				JointAngVelWorld = FVector(Batch.JointVel[DOFIdx], Batch.JointVel[DOFIdx + Stride], Batch.JointVel[DOFIdx + 2 * Stride]);
			}
			else
			{
				const FVector AxisWorld = ParentRot.RotateVector(Topo.BodyJointAxisLocal[Body]);
				BodyRot = (FQuat(AxisWorld, Batch.JointPos[DOFIdx]) * ParentRot * Topo.BodyRestRotInParent[Body]).GetNormalized();
				JointAngVelWorld = AxisWorld * Batch.JointVel[DOFIdx];
			}

			const FVector BodyPos = ParentPos + ParentRot.RotateVector(Topo.BodyJointOffsetInParent[Body]);
			Batch.SetBodyPos(Body, Env, BodyPos);
			Batch.SetBodyRot(Body, Env, BodyRot);

			const FVector AngVel = ParentAngVel + JointAngVelWorld;
			const FVector LinVel = ParentLinVel + FVector::CrossProduct(ParentAngVel, BodyPos - ParentPos);
			Batch.AngVelX[Idx] = (float)AngVel.X; Batch.AngVelY[Idx] = (float)AngVel.Y; Batch.AngVelZ[Idx] = (float)AngVel.Z;
			Batch.LinVelX[Idx] = (float)LinVel.X; Batch.LinVelY[Idx] = (float)LinVel.Y; Batch.LinVelZ[Idx] = (float)LinVel.Z;
		}
	}

	/**
	 * Writes a reference frame into one env's joint state -- "reference state
	 * initialization" (RSI) -- and refreshes that env's body world transforms
	 * to match. Call AFTER CreatureRLEnvironment::ResetEnv, which zeroes
	 * everything and draws the episode's domain randomization.
	 *
	 * RSI is not a nicety. Starting every episode from the rest pose means the
	 * policy only ever sees the beginning of the reference, and has to master
	 * that before it experiences anything later -- for a cyclic motion that is
	 * the difference between learning and not. Starting from a RANDOM phase
	 * exposes the whole clip from the first iteration.
	 *
	 * Ball joints get BOTH JointRelRot (the driving state) and JointPos (the
	 * derived reporting value Pass 3b keeps in sync) -- writing only one leaves
	 * the observation disagreeing with the physics for a step.
	 *
	 * TorsoWorldPos/TorsoWorldRot place the root, and the caller supplies BOTH
	 * outright rather than this deriving them from the frame: the frame's own
	 * RootPos is in the source's component space and is not a world position
	 * (see FReferenceFrame::RootHeightAboveRest), and a caller may legitimately
	 * want to compose the frame's orientation with something else -- the
	 * training driver folds this episode's reset noise in, so that RSI does not
	 * silently discard PosNoiseStdDev/AngleNoiseRad. The frame's rest-relative
	 * height is applied on top of TorsoWorldPos.
	 */
	inline void ApplyReferenceFrameToEnv(
		FCreatureBatchState& Batch,
		int32 Env,
		const FReferenceFrame& Frame,
		const FVector& TorsoWorldPos,
		const FQuat& TorsoWorldRot)
	{
		const FCreatureTopology& Topo = Batch.GetTopology();
		check(Frame.IsCompatibleWith(Topo));

		Batch.SetBodyPos(0, Env, TorsoWorldPos + FVector(0.0f, 0.0f, Frame.RootHeightAboveRest));
		Batch.SetBodyRot(0, Env, TorsoWorldRot.GetNormalized());

		for (int32 Body = 1; Body < Topo.NumBodies; ++Body)
		{
			const int32 DOFOffset = Topo.BodyDOFOffset[Body];
			const int32 DOFCount = Topo.BodyDOFCount[Body];

			if (DOFCount == 3)
			{
				Batch.SetJointRelRot(Body, Env, Frame.BodyRelRot[Body].GetNormalized());
			}

			for (int32 k = 0; k < DOFCount; ++k)
			{
				const int32 DOFIdx = Batch.DOFIndex(DOFOffset + k, Env);
				Batch.JointPos[DOFIdx] = Frame.DOFPos[DOFOffset + k];
				Batch.JointVel[DOFIdx] = Frame.DOFVel[DOFOffset + k];
				Batch.JointTorque[DOFIdx] = 0.0f;
			}
		}

		// Joint angles alone are not a pose. Without this, the observation
		// gathered immediately after the reset -- and the end-effector reward
		// term, which reads body world positions -- would still describe the
		// rest pose for one full step.
		RecomputeEnvKinematics(Batch, Env);
	}
}
