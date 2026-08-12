#pragma once

#include "CoreMinimal.h"
#include "CreatureBatchState.h"
#include "SpatialAlgebra.h"
#include "SpatialAlgebraSIMD.h"
#include "Async/ParallelFor.h"
#include <atomic>

/**
 * Batched Featherstone Articulated Body Algorithm over an FCreatureBatchState.
 *
 * Scope of this version:
 *  - Body 0 (torso) is a free-floating 6-DOF base — solved via a direct 6x6
 *    linear solve of its fully-composited articulated inertia, since no joint
 *    reduction applies to it.
 *  - Other bodies are either 1-DOF revolute joints or 3-DOF ball joints
 *    (Topology.BodyDOFCount[Body] == 1 or 3, branched on per body). A ball
 *    joint's relative-to-parent orientation is a persistent quaternion
 *    (FCreatureBatchState::JointRelRot*), integrated via exp-map each step
 *    — the same technique Pass 3a uses for the free-floating root — with
 *    JointPos exposing a derived rotation-vector reporting value
 *    (RelRot.ToRotationVector()), not the driving state. In StepSIMD, ball
 *    joints' Pass 2/3b run lane-scalar (mirroring Pass 3a's precedent);
 *    only their Pass 1 kinematics is vectorized. Vectorizing their
 *    reduction/integration too is a follow-up, not covered here.
 *  - All spatial quantities are expressed in WORLD-ALIGNED axes, translated
 *    (never rotated) between reference points. This is valid because every
 *    body's velocity/acceleration/inertia is already stored in world frame,
 *    not body-local — so the Plucker transform between any two points on the
 *    kinematic tree reduces to the translation-only case (no rotation matrix
 *    bookkeeping needed). See SpatialAlgebra.h's TranslateMotion/TranslateForce.
 *  - KNOWN SIMPLIFICATION: Pass 3's acceleration propagation omits the
 *    velocity-product ("Coriolis") term a fully time-varying spatial
 *    transform would include. Fine for RL training (self-consistent dynamics
 *    is what matters, not an exact match to reality) but worth revisiting if
 *    you need this to track Chaos closely for the in-engine deployment path.
 *  - Integration is semi-implicit Euler, matching the rest of the pipeline.
 *
 * StepScalar() is the plain-nested-loop reference implementation — kept
 * permanently as the correctness oracle for StepSIMD() (see the automation
 * test in Tests/CreatureBatchSolverSIMDTest.cpp) and as a fallback for any
 * future platform without AVX2. StepSIMD() batches the env axis 8-wide
 * (FCreatureBatchState::SIMDWidth) using the AVX2 types in
 * SpatialAlgebraSIMD.h; the body-major/env-minor layout from
 * FCreatureBatchState is exactly what makes this possible with plain
 * contiguous loads — no gather/scatter needed. Step() calls StepSIMD().
 *
 * Pass 3a (the floating-base root solve) stays lane-scalar even inside
 * StepSIMD(): SolveSpatial6 is a branchy 6x6 Gaussian elimination with
 * partial pivoting, called once per env per step (vs. NumBodies-1 times
 * per env for the other passes) — not worth vectorizing the pivot search
 * for that little work.
 */
class FCreatureABASolver
{
public:
	void Step(FCreatureBatchState& Batch, float Dt, const FVector& Gravity = FVector(0.0f, 0.0f, -980.0f))
	{
		StepSIMD(Batch, Dt, Gravity);
	}

	void StepScalar(FCreatureBatchState& Batch, float Dt, const FVector& Gravity = FVector(0.0f, 0.0f, -980.0f))
	{
		const FCreatureTopology& Topo = Batch.GetTopology();
		const int32 NumBodies = Topo.NumBodies;
		const int32 NumEnvs = Batch.GetNumEnvs();
		const int32 PaddedNumEnvs = Batch.GetPaddedNumEnvs();
		const int32 BodySlots = NumBodies * PaddedNumEnvs;

		IAcc.SetNum(BodySlots, EAllowShrinking::No);
		PAcc.SetNum(BodySlots, EAllowShrinking::No);
		AAcc.SetNum(BodySlots, EAllowShrinking::No);
		UAcc.SetNum(BodySlots, EAllowShrinking::No);
		DAcc.SetNumZeroed(BodySlots, EAllowShrinking::No);
		UScalarAcc.SetNumZeroed(BodySlots, EAllowShrinking::No);
		WorldAxisAcc.SetNum(BodySlots, EAllowShrinking::No);
		BallU_u3Acc.SetNum(BodySlots, EAllowShrinking::No);
		ComputeMuscleMultipliers(Batch);

		// ---- Pass 1: forward kinematics + velocity propagation (base -> tip) ----
		// Parallelized across envs (see class comment): one task per env, body
		// order kept sequential WITHIN a task since that's where the base->tip
		// dependency actually lives. Different envs never touch the same
		// array slot (FCreatureBatchState::BodyIndex/DOFIndex always include
		// Env in the offset), so tasks need no synchronization.
		ParallelFor(NumEnvs, [&](int32 Env)
		{
			for (int32 Body = 1; Body < NumBodies; ++Body)
			{
				const int32 Parent = Topo.BodyParent[Body];
				const bool bIsBall = (Topo.BodyDOFCount[Body] == 3);

				const int32 Idx = Batch.BodyIndex(Body, Env);
				const int32 ParentIdx = Batch.BodyIndex(Parent, Env);

				const FQuat ParentRot = Batch.GetBodyRot(Parent, Env);
				const FVector ParentPos = Batch.GetBodyPos(Parent, Env);
				const FVector ParentAngVel(Batch.AngVelX[ParentIdx], Batch.AngVelY[ParentIdx], Batch.AngVelZ[ParentIdx]);
				const FVector ParentLinVel(Batch.LinVelX[ParentIdx], Batch.LinVelY[ParentIdx], Batch.LinVelZ[ParentIdx]);

				const int32 DOFIdx = Batch.DOFIndex(Topo.BodyDOFOffset[Body], Env);

				FQuat BodyRot;
				FVector JointAngVelWorld;
				if (bIsBall)
				{
					// Ball joint: relative orientation is a persistent quaternion
					// (not rebuilt from JointPos — see class comment), integrated
					// in Pass 3b.
					const FQuat RelRot = Batch.GetJointRelRot(Body, Env);
					BodyRot = (ParentRot * Topo.BodyRestRotInParent[Body] * RelRot).GetNormalized();
					JointAngVelWorld = FVector(Batch.JointVel[DOFIdx], Batch.JointVel[DOFIdx + PaddedNumEnvs], Batch.JointVel[DOFIdx + 2 * PaddedNumEnvs]);
					WorldAxisAcc[Idx] = FVector::ZeroVector; // unused for ball joints
				}
				else
				{
					const float JointAngle = Batch.JointPos[DOFIdx];
					const float JointRate = Batch.JointVel[DOFIdx];

					// Axis is invariant under rotation about itself, so rotating the
					// rest-pose axis by the PARENT's current rotation is valid even
					// though the joint angle hasn't been applied yet at this point.
					const FVector AxisWorld = ParentRot.RotateVector(Topo.BodyJointAxisLocal[Body]);
					WorldAxisAcc[Idx] = AxisWorld;
					// BodyRestRotInParent brings us to the body's correct bind-pose
					// world rotation at JointAngle==0; the JointAngle spin about
					// AxisWorld is then applied on top of that (world-space,
					// outermost) — matches this rig's authored bind pose instead of
					// implicitly assuming every body starts aligned with its parent.
					BodyRot = (FQuat(AxisWorld, JointAngle) * ParentRot * Topo.BodyRestRotInParent[Body]).GetNormalized();
					JointAngVelWorld = AxisWorld * JointRate;
				}

				const FVector JointOffsetWorld = ParentRot.RotateVector(Topo.BodyJointOffsetInParent[Body]);
				const FVector BodyPos = ParentPos + JointOffsetWorld;
				Batch.SetBodyPos(Body, Env, BodyPos);
				Batch.SetBodyRot(Body, Env, BodyRot);

				const FVector R = BodyPos - ParentPos; // parent -> body, world
				const FVector AngVel = ParentAngVel + JointAngVelWorld;
				const FVector LinVel = ParentLinVel + FVector::CrossProduct(ParentAngVel, R);

				Batch.AngVelX[Idx] = (float)AngVel.X; Batch.AngVelY[Idx] = (float)AngVel.Y; Batch.AngVelZ[Idx] = (float)AngVel.Z;
				Batch.LinVelX[Idx] = (float)LinVel.X; Batch.LinVelY[Idx] = (float)LinVel.Y; Batch.LinVelZ[Idx] = (float)LinVel.Z;
			}
		});

		// ---- Own rigid-body inertia + bias force, every body (order-independent) ----
		ParallelFor(NumEnvs, [&](int32 Env)
		{
			for (int32 Body = 0; Body < NumBodies; ++Body)
			{
				const int32 Idx = Batch.BodyIndex(Body, Env);
				const FQuat Rot = Batch.GetBodyRot(Body, Env);
				const FVector CoMOffset = Rot.RotateVector(Topo.BodyLocalCoMOffset[Body]);

				float Mass = Topo.BodyMass[Body];
				if (Body == 0)
				{
					// Simplification: carried mass added at the torso without
					// shifting CoM. Fine for now; revisit if off-center loads matter.
					Mass += Batch.CarriedMass[Env];
				}

				const FMat3 RotM = FMat3::FromRotation(Rot);
				const FMat3 IrotAboutCoM = RotM * FMat3::Diagonal(Topo.BodyInertiaDiagLocal[Body]) * RotM.Transpose();
				const FMat3 Rx = FMat3::Skew(CoMOffset);
				// Parallel-axis shift: inertia about CoM -> inertia about this body's own reference point (its joint origin / root origin)
				const FMat3 IrotAboutRef = IrotAboutCoM - (Rx * Rx) * Mass;
				const FMat3 HAboutRef = Rx * Mass;

				const FSpatialInertia Own = FSpatialInertia::FromRigidBody(Mass, IrotAboutRef, HAboutRef);

				const FVector AngVel(Batch.AngVelX[Idx], Batch.AngVelY[Idx], Batch.AngVelZ[Idx]);
				const FVector LinVel(Batch.LinVelX[Idx], Batch.LinVelY[Idx], Batch.LinVelZ[Idx]);
				const FSpatialVec V{ AngVel, LinVel };

				const FVector GravityForceWorld = Mass * Gravity;
				const FSpatialVec GravityWrench{ FVector::CrossProduct(CoMOffset, GravityForceWorld), GravityForceWorld };
				const FSpatialVec HitWrench{
					FVector(Batch.ExtTorqueX[Idx], Batch.ExtTorqueY[Idx], Batch.ExtTorqueZ[Idx]),
					FVector(Batch.ExtForceX[Idx], Batch.ExtForceY[Idx], Batch.ExtForceZ[Idx])
				};
				const FSpatialVec TotalExtForce = GravityWrench + HitWrench;

				// Gyroscopic (velocity-product) term minus applied external wrench.
				IAcc[Idx] = Own;
				PAcc[Idx] = SpatialCrossForce(V, Own.Apply(V)) - TotalExtForce;
			}
		});

		// ---- Pass 2: backward accumulation (tip -> base), reduce each joint (1-DOF revolute or 3-DOF ball) ----
		ParallelFor(NumEnvs, [&](int32 Env)
		{
			for (int32 Body = NumBodies - 1; Body >= 1; --Body)
			{
				const int32 Parent = Topo.BodyParent[Body];
				const bool bIsBall = (Topo.BodyDOFCount[Body] == 3);
				const int32 Idx = Batch.BodyIndex(Body, Env);
				const int32 ParentIdx = Batch.BodyIndex(Parent, Env);

				const int32 DOFIdx = Batch.DOFIndex(Topo.BodyDOFOffset[Body], Env);
				const int32 LimbIdx = Topo.BodyLimbIndex[Body];
				const float StrengthScale = (LimbIdx != INDEX_NONE) ? Batch.LimbStrengthScale[Batch.LimbIndex(LimbIdx, Env)] : 1.0f;
				const float ActiveMask = (LimbIdx != INDEX_NONE) ? (float)Batch.LimbActive[Batch.LimbIndex(LimbIdx, Env)] : 1.0f;

				FSpatialInertia Reduced;
				FSpatialVec ReducedBias;

				if (bIsBall)
				{
					// See class comment for the derivation: for a joint free in all
					// 3 rotational DOFs, D = Irot exactly, and the reduced inertia's
					// Irot/H blocks vanish (a spherical joint transmits no angular
					// coupling to the parent, only the reflected linear inertia).
					const FMat3 Irot = IAcc[Idx].Irot;
					const FMat3 H = IAcc[Idx].H;
					const FMat3 Dinv = Inverse3x3(Irot);

					const FVector Tau3(
						Batch.JointTorque[DOFIdx] * StrengthScale * ActiveMask * MuscleMultiplierAcc[DOFIdx],
						Batch.JointTorque[DOFIdx + PaddedNumEnvs] * StrengthScale * ActiveMask * MuscleMultiplierAcc[DOFIdx + PaddedNumEnvs],
						Batch.JointTorque[DOFIdx + 2 * PaddedNumEnvs] * StrengthScale * ActiveMask * MuscleMultiplierAcc[DOFIdx + 2 * PaddedNumEnvs]);
					const FVector U_u3 = Tau3 - PAcc[Idx].Ang;
					BallU_u3Acc[Idx] = U_u3;

					Reduced.Irot = FMat3::Zero();
					Reduced.H = FMat3::Zero();
					Reduced.MBlock = IAcc[Idx].MBlock - H.Transpose() * Dinv * H;

					ReducedBias = FSpatialVec{ Tau3, PAcc[Idx].Lin + H.Transpose() * (Dinv * U_u3) };
				}
				else
				{
					const FSpatialVec S{ WorldAxisAcc[Idx], FVector::ZeroVector };
					const FSpatialVec U = IAcc[Idx].Apply(S);
					const float D = FMath::Max(FSpatialVec::Dot(S, U), KINDA_SMALL_NUMBER);
					const float Tau = Batch.JointTorque[DOFIdx] * StrengthScale * ActiveMask * MuscleMultiplierAcc[DOFIdx];
					const float U_u = Tau - FSpatialVec::Dot(S, PAcc[Idx]);

					UAcc[Idx] = U;
					DAcc[Idx] = D;
					UScalarAcc[Idx] = U_u;

					const float InvD = 1.0f / D;
					Reduced.Irot = IAcc[Idx].Irot - FMat3::Outer(U.Ang, U.Ang) * InvD;
					Reduced.H = IAcc[Idx].H - FMat3::Outer(U.Ang, U.Lin) * InvD;
					Reduced.MBlock = IAcc[Idx].MBlock - FMat3::Outer(U.Lin, U.Lin) * InvD;

					ReducedBias = PAcc[Idx] + U * (U_u * InvD);
				}

				const FVector R = Batch.GetBodyPos(Body, Env) - Batch.GetBodyPos(Parent, Env);
				IAcc[ParentIdx] = IAcc[ParentIdx] + Reduced.TranslatedTo(R);
				PAcc[ParentIdx] = PAcc[ParentIdx] + TranslateForce(ReducedBias, R);
			}
		});

		// ---- Pass 3a: floating-base root solve ----
		ParallelFor(NumEnvs, [&](int32 Env)
		{
			const int32 RootIdx = Batch.BodyIndex(0, Env);
			const FSpatialVec A0 = SolveSpatial6(IAcc[RootIdx], PAcc[RootIdx] * -1.0f);
			AAcc[RootIdx] = A0;

			FVector RootLinVel(Batch.LinVelX[RootIdx], Batch.LinVelY[RootIdx], Batch.LinVelZ[RootIdx]);
			FVector RootAngVel(Batch.AngVelX[RootIdx], Batch.AngVelY[RootIdx], Batch.AngVelZ[RootIdx]);
			RootLinVel += A0.Lin * Dt;
			RootAngVel += A0.Ang * Dt;
			Batch.LinVelX[RootIdx] = (float)RootLinVel.X; Batch.LinVelY[RootIdx] = (float)RootLinVel.Y; Batch.LinVelZ[RootIdx] = (float)RootLinVel.Z;
			Batch.AngVelX[RootIdx] = (float)RootAngVel.X; Batch.AngVelY[RootIdx] = (float)RootAngVel.Y; Batch.AngVelZ[RootIdx] = (float)RootAngVel.Z;

			const FVector RootPos = Batch.GetBodyPos(0, Env) + RootLinVel * Dt;
			FQuat RootRot = Batch.GetBodyRot(0, Env);
			const FVector DTheta = RootAngVel * Dt;
			const float Angle = static_cast<float>(DTheta.Size());
			if (Angle > KINDA_SMALL_NUMBER)
			{
				RootRot = (FQuat(DTheta / Angle, Angle) * RootRot).GetNormalized();
			}
			Batch.SetBodyPos(0, Env, RootPos);
			Batch.SetBodyRot(0, Env, RootRot);
		});

		// ---- Pass 3b: forward acceleration + joint integration (base -> tip) ----
		ParallelFor(NumEnvs, [&](int32 Env)
		{
			for (int32 Body = 1; Body < NumBodies; ++Body)
			{
				const int32 Parent = Topo.BodyParent[Body];
				const bool bIsBall = (Topo.BodyDOFCount[Body] == 3);
				const int32 Idx = Batch.BodyIndex(Body, Env);
				const int32 ParentIdx = Batch.BodyIndex(Parent, Env);

				const FVector R = Batch.GetBodyPos(Body, Env) - Batch.GetBodyPos(Parent, Env);
				const FSpatialVec ParentAccAtBody = TranslateMotion(AAcc[ParentIdx], R); // see Coriolis-term note above

				const int32 DOFIdx = Batch.DOFIndex(Topo.BodyDOFOffset[Body], Env);

				if (bIsBall)
				{
					const FMat3 Irot = IAcc[Idx].Irot;
					const FMat3 H = IAcc[Idx].H;
					const FMat3 Dinv = Inverse3x3(Irot);
					const FVector RHS = BallU_u3Acc[Idx] - (Irot * ParentAccAtBody.Ang + H * ParentAccAtBody.Lin);
					const FVector QDDot3 = Dinv * RHS;

					AAcc[Idx] = ParentAccAtBody + FSpatialVec{ QDDot3, FVector::ZeroVector };

					FVector JointAngVelWorld(Batch.JointVel[DOFIdx], Batch.JointVel[DOFIdx + PaddedNumEnvs], Batch.JointVel[DOFIdx + 2 * PaddedNumEnvs]);
					JointAngVelWorld += QDDot3 * Dt;
					Batch.JointVel[DOFIdx] = (float)JointAngVelWorld.X;
					Batch.JointVel[DOFIdx + PaddedNumEnvs] = (float)JointAngVelWorld.Y;
					Batch.JointVel[DOFIdx + 2 * PaddedNumEnvs] = (float)JointAngVelWorld.Z;

					// RelRot is defined in the PARENT's frame, but JointAngVelWorld
					// (like everything else in this solver) is world-aligned — rotate
					// it into the parent's frame before composing the exp-map update,
					// same branch pattern as Pass 3a's root rotation integration.
					const FQuat ParentRot = Batch.GetBodyRot(Parent, Env);
					const FVector RelAngVelParentFrame = ParentRot.UnrotateVector(JointAngVelWorld);
					const FVector DTheta = RelAngVelParentFrame * Dt;
					const float Angle = static_cast<float>(DTheta.Size());
					FQuat RelRot = Batch.GetJointRelRot(Body, Env);
					if (Angle > KINDA_SMALL_NUMBER)
					{
						RelRot = (FQuat(DTheta / Angle, Angle) * RelRot).GetNormalized();
					}
					// Canonicalize to W>=0: q and -q represent the IDENTICAL
					// rotation, but ToRotationVector() only stays inside
					// [-180,180] deg per axis (the well-behaved region
					// ClampJointLimits' per-axis clamp assumes) when W>=0 —
					// otherwise the decomposed angle can exceed 180°, where
					// the axis-angle parameterization becomes ambiguous/
					// discontinuous. Confirmed as the real cause of a
					// reported "reaches max angle, fights, then teleports"
					// bug (2026-08-12): a throwaway diagnostic found ball
					// joints rotating 30-56° in a SINGLE 1/240s substep with
					// near-zero reported angular velocity — i.e. not real
					// motion, but ClampJointLimits' clamp-and-reconstruct
					// occasionally producing a wildly different quaternion
					// because the pre-clamp rotation vector had wrapped past
					// 180° without ever being renormalized back to the
					// canonical short-way-round representative.
					if (RelRot.W < 0.0f)
					{
						RelRot = FQuat(-RelRot.X, -RelRot.Y, -RelRot.Z, -RelRot.W);
					}
					Batch.SetJointRelRot(Body, Env, RelRot);

					// JointPos is not driving state for ball joints — it's a derived
					// rotation-vector reporting value for external readers (RL
					// observations, debug tools).
					const FVector ReportRotVec = RelRot.ToRotationVector();
					Batch.JointPos[DOFIdx] = (float)ReportRotVec.X;
					Batch.JointPos[DOFIdx + PaddedNumEnvs] = (float)ReportRotVec.Y;
					Batch.JointPos[DOFIdx + 2 * PaddedNumEnvs] = (float)ReportRotVec.Z;
				}
				else
				{
					const FSpatialVec S{ WorldAxisAcc[Idx], FVector::ZeroVector };
					const float QDDot = (UScalarAcc[Idx] - FSpatialVec::Dot(UAcc[Idx], ParentAccAtBody)) / DAcc[Idx];

					AAcc[Idx] = ParentAccAtBody + S * QDDot;

					Batch.JointVel[DOFIdx] += QDDot * Dt;
					Batch.JointPos[DOFIdx] += Batch.JointVel[DOFIdx] * Dt;
				}
			}
		});

		if (ClampJointLimits(Batch))
		{
			RecomputeKinematics(Batch); // see that function's comment — closes the one-step reporting lag a clamp would otherwise leave
		}

		// External forces are NOT cleared here — caller decides whether a hit
		// was a one-shot impulse (call Batch.ClearExternalForces(Env) after
		// Step()) or a sustained push that should persist across steps.
	}

	void StepSIMD(FCreatureBatchState& Batch, float Dt, const FVector& Gravity = FVector(0.0f, 0.0f, -980.0f))
	{
		const FCreatureTopology& Topo = Batch.GetTopology();
		const int32 NumBodies = Topo.NumBodies;
		const int32 NumEnvs = Batch.GetNumEnvs();
		const int32 PaddedNumEnvs = Batch.GetPaddedNumEnvs();
		const int32 BodySlots = NumBodies * PaddedNumEnvs;
		constexpr int32 SIMDWidth = FCreatureBatchState::SIMDWidth;
		check(PaddedNumEnvs % SIMDWidth == 0);

		IAcc8.SetNum(BodySlots, EAllowShrinking::No);
		PAcc8.SetNum(BodySlots, EAllowShrinking::No);
		AAcc8.SetNum(BodySlots, EAllowShrinking::No);
		UAcc8.SetNum(BodySlots, EAllowShrinking::No);
		DAcc.SetNumZeroed(BodySlots, EAllowShrinking::No);
		UScalarAcc.SetNumZeroed(BodySlots, EAllowShrinking::No);
		WorldAxisAcc8.SetNum(BodySlots, EAllowShrinking::No);
		BallU_u3Acc.SetNum(BodySlots, EAllowShrinking::No);
		ComputeMuscleMultipliers(Batch);

		const __m256 DtVec = _mm256_set1_ps(Dt);
		const FVec3x8 Gravity8 = FVec3x8::Broadcast(Gravity);
		const int32 NumChunks = PaddedNumEnvs / SIMDWidth;

		// ---- Pass 1: forward kinematics + velocity propagation (base -> tip) ----
		// Ball joints stay fully vectorized here — the kinematics are branch-free,
		// unlike Pass 2/3b's reduction/integration (see class comment).
		// Parallelized across SIMD-width env chunks: one task per chunk, body
		// order kept sequential WITHIN a task (that's where the base->tip
		// dependency lives). Different chunks never touch the same array
		// slot, so tasks need no synchronization — same reasoning as
		// StepScalar's per-env parallelization above.
		ParallelFor(NumChunks, [&](int32 ChunkIdx)
		{
			const int32 EnvChunk = ChunkIdx * SIMDWidth;
			for (int32 Body = 1; Body < NumBodies; ++Body)
			{
				const int32 Parent = Topo.BodyParent[Body];
				const bool bIsBall = (Topo.BodyDOFCount[Body] == 3);
				const FVec3x8 AxisLocal8 = FVec3x8::Broadcast(Topo.BodyJointAxisLocal[Body]);
				const FVec3x8 JointOffsetLocal8 = FVec3x8::Broadcast(Topo.BodyJointOffsetInParent[Body]);
				const FQuatx8 RestRotInParent8 = FQuatx8::Broadcast(Topo.BodyRestRotInParent[Body]);

				const int32 Idx = Batch.BodyIndex(Body, EnvChunk);
				const int32 ParentIdx = Batch.BodyIndex(Parent, EnvChunk);

				const FQuatx8 ParentRot = FQuatx8::Load(&Batch.RotX[ParentIdx], &Batch.RotY[ParentIdx], &Batch.RotZ[ParentIdx], &Batch.RotW[ParentIdx]);
				const FVec3x8 ParentPos = FVec3x8::Load(&Batch.PosX[ParentIdx], &Batch.PosY[ParentIdx], &Batch.PosZ[ParentIdx]);
				const FVec3x8 ParentAngVel = FVec3x8::Load(&Batch.AngVelX[ParentIdx], &Batch.AngVelY[ParentIdx], &Batch.AngVelZ[ParentIdx]);
				const FVec3x8 ParentLinVel = FVec3x8::Load(&Batch.LinVelX[ParentIdx], &Batch.LinVelY[ParentIdx], &Batch.LinVelZ[ParentIdx]);

				const int32 DOFIdx = Batch.DOFIndex(Topo.BodyDOFOffset[Body], EnvChunk);

				FQuatx8 BodyRot;
				FVec3x8 JointAngVelWorld;
				if (bIsBall)
				{
					const FQuatx8 RelRot = FQuatx8::Load(&Batch.JointRelRotX[Idx], &Batch.JointRelRotY[Idx], &Batch.JointRelRotZ[Idx], &Batch.JointRelRotW[Idx]);
					BodyRot = (ParentRot * RestRotInParent8 * RelRot).GetNormalized();
					JointAngVelWorld = FVec3x8::Load(&Batch.JointVel[DOFIdx], &Batch.JointVel[DOFIdx + PaddedNumEnvs], &Batch.JointVel[DOFIdx + 2 * PaddedNumEnvs]);
					WorldAxisAcc8.Store(Idx, FVec3x8::Zero()); // unused for ball joints
				}
				else
				{
					const __m256 JointAngle = _mm256_loadu_ps(&Batch.JointPos[DOFIdx]);
					const __m256 JointRate = _mm256_loadu_ps(&Batch.JointVel[DOFIdx]);

					const FVec3x8 AxisWorld = FQuatx8::RotateVector(ParentRot, AxisLocal8);
					WorldAxisAcc8.Store(Idx, AxisWorld);
					// See StepScalar's mirror of this for why RestRotInParent8 sits here.
					BodyRot = (FQuatx8::FromAxisAngle(AxisWorld, JointAngle) * ParentRot * RestRotInParent8).GetNormalized();
					JointAngVelWorld = AxisWorld * JointRate;
				}

				const FVec3x8 JointOffsetWorld = FQuatx8::RotateVector(ParentRot, JointOffsetLocal8);
				const FVec3x8 BodyPos = ParentPos + JointOffsetWorld;

				BodyPos.Store(&Batch.PosX[Idx], &Batch.PosY[Idx], &Batch.PosZ[Idx]);
				BodyRot.Store(&Batch.RotX[Idx], &Batch.RotY[Idx], &Batch.RotZ[Idx], &Batch.RotW[Idx]);

				const FVec3x8 R = BodyPos - ParentPos; // parent -> body, world
				const FVec3x8 AngVel = ParentAngVel + JointAngVelWorld;
				const FVec3x8 LinVel = ParentLinVel + FVec3x8::Cross(ParentAngVel, R);

				AngVel.Store(&Batch.AngVelX[Idx], &Batch.AngVelY[Idx], &Batch.AngVelZ[Idx]);
				LinVel.Store(&Batch.LinVelX[Idx], &Batch.LinVelY[Idx], &Batch.LinVelZ[Idx]);
			}
		});

		// ---- Own rigid-body inertia + bias force, every body (order-independent) ----
		ParallelFor(NumChunks, [&](int32 ChunkIdx)
		{
			const int32 EnvChunk = ChunkIdx * SIMDWidth;
			for (int32 Body = 0; Body < NumBodies; ++Body)
			{
				const FVec3x8 CoMOffsetLocal8 = FVec3x8::Broadcast(Topo.BodyLocalCoMOffset[Body]);
				const FVec3x8 InertiaDiagLocal8 = FVec3x8::Broadcast(Topo.BodyInertiaDiagLocal[Body]);
				const __m256 BaseMass8 = _mm256_set1_ps(Topo.BodyMass[Body]);
				const bool bIsRoot = (Body == 0);

				const int32 Idx = Batch.BodyIndex(Body, EnvChunk);
				const FQuatx8 Rot = FQuatx8::Load(&Batch.RotX[Idx], &Batch.RotY[Idx], &Batch.RotZ[Idx], &Batch.RotW[Idx]);
				const FVec3x8 CoMOffset = FQuatx8::RotateVector(Rot, CoMOffsetLocal8);

				__m256 Mass = BaseMass8;
				if (bIsRoot)
				{
					// Simplification: carried mass added at the torso without
					// shifting CoM. Fine for now; revisit if off-center loads matter.
					Mass = _mm256_add_ps(Mass, _mm256_loadu_ps(&Batch.CarriedMass[EnvChunk]));
				}

				const FMat3x8 RotM = FMat3x8::FromRotation(Rot);
				const FMat3x8 IrotAboutCoM = RotM * FMat3x8::Diagonal(InertiaDiagLocal8) * RotM.Transpose();
				const FMat3x8 Rx = FMat3x8::Skew(CoMOffset);
				// Parallel-axis shift: inertia about CoM -> inertia about this body's own reference point (its joint origin / root origin)
				const FMat3x8 IrotAboutRef = IrotAboutCoM - (Rx * Rx) * Mass;
				const FMat3x8 HAboutRef = Rx * Mass;

				const FSpatialInertiax8 Own = FSpatialInertiax8::FromRigidBody(Mass, IrotAboutRef, HAboutRef);

				const FVec3x8 AngVel = FVec3x8::Load(&Batch.AngVelX[Idx], &Batch.AngVelY[Idx], &Batch.AngVelZ[Idx]);
				const FVec3x8 LinVel = FVec3x8::Load(&Batch.LinVelX[Idx], &Batch.LinVelY[Idx], &Batch.LinVelZ[Idx]);
				const FSpatialVecx8 V{ AngVel, LinVel };

				const FVec3x8 GravityForceWorld = Gravity8 * Mass;
				const FSpatialVecx8 GravityWrench{ FVec3x8::Cross(CoMOffset, GravityForceWorld), GravityForceWorld };
				const FSpatialVecx8 HitWrench{
					FVec3x8::Load(&Batch.ExtTorqueX[Idx], &Batch.ExtTorqueY[Idx], &Batch.ExtTorqueZ[Idx]),
					FVec3x8::Load(&Batch.ExtForceX[Idx], &Batch.ExtForceY[Idx], &Batch.ExtForceZ[Idx])
				};
				const FSpatialVecx8 TotalExtForce = GravityWrench + HitWrench;

				// Gyroscopic (velocity-product) term minus applied external wrench.
				IAcc8.Store(Idx, Own);
				PAcc8.Store(Idx, SpatialCrossForce(V, Own.Apply(V)) - TotalExtForce);
			}
		});

		// ---- Pass 2: backward accumulation (tip -> base), reduce each joint (1-DOF revolute or 3-DOF ball) ----
		// Parallelized across SIMD-width env chunks; the ball-joint branch is
		// lane-scalar (see class comment), so its inner loop runs per-env
		// within [EnvChunk, EnvChunk+SIMDWidth) clamped to NumEnvs — same
		// bound the original per-body loop used, just narrowed to this task's
		// own chunk instead of the whole batch.
		ParallelFor(NumChunks, [&](int32 ChunkIdx)
		{
			const int32 EnvChunk = ChunkIdx * SIMDWidth;
			const int32 EnvBegin = EnvChunk;
			const int32 EnvEnd = FMath::Min(EnvChunk + SIMDWidth, NumEnvs);

			for (int32 Body = NumBodies - 1; Body >= 1; --Body)
			{
				const int32 Parent = Topo.BodyParent[Body];
				const int32 LimbIdx = Topo.BodyLimbIndex[Body];

				if (Topo.BodyDOFCount[Body] == 3)
				{
					// Lane-scalar — see class comment (mirrors Pass 3a's root solve).
					for (int32 Env = EnvBegin; Env < EnvEnd; ++Env)
					{
						const int32 Idx = Batch.BodyIndex(Body, Env);
						const int32 ParentIdx = Batch.BodyIndex(Parent, Env);
						const int32 DOFIdx = Batch.DOFIndex(Topo.BodyDOFOffset[Body], Env);

						const float StrengthScale = (LimbIdx != INDEX_NONE) ? Batch.LimbStrengthScale[Batch.LimbIndex(LimbIdx, Env)] : 1.0f;
						const float ActiveMask = (LimbIdx != INDEX_NONE) ? (float)Batch.LimbActive[Batch.LimbIndex(LimbIdx, Env)] : 1.0f;

						const FSpatialInertia IBody = IAcc8.LoadScalar(Idx);
						const FSpatialVec PBody = PAcc8.LoadScalar(Idx);

						const FMat3 Irot = IBody.Irot;
						const FMat3 H = IBody.H;
						const FMat3 Dinv = Inverse3x3(Irot);

						const FVector Tau3(
							Batch.JointTorque[DOFIdx] * StrengthScale * ActiveMask * MuscleMultiplierAcc[DOFIdx],
							Batch.JointTorque[DOFIdx + PaddedNumEnvs] * StrengthScale * ActiveMask * MuscleMultiplierAcc[DOFIdx + PaddedNumEnvs],
							Batch.JointTorque[DOFIdx + 2 * PaddedNumEnvs] * StrengthScale * ActiveMask * MuscleMultiplierAcc[DOFIdx + 2 * PaddedNumEnvs]);
						const FVector U_u3 = Tau3 - PBody.Ang;
						BallU_u3Acc[Idx] = U_u3;

						FSpatialInertia Reduced;
						Reduced.Irot = FMat3::Zero();
						Reduced.H = FMat3::Zero();
						Reduced.MBlock = IBody.MBlock - H.Transpose() * Dinv * H;

						const FSpatialVec ReducedBias{ Tau3, PBody.Lin + H.Transpose() * (Dinv * U_u3) };

						const FVector R = Batch.GetBodyPos(Body, Env) - Batch.GetBodyPos(Parent, Env);
						IAcc8.AccumulateScalar(ParentIdx, Reduced.TranslatedTo(R));
						PAcc8.AccumulateScalar(ParentIdx, TranslateForce(ReducedBias, R));
					}
					continue;
				}

				{
					const int32 Idx = Batch.BodyIndex(Body, EnvChunk);
					const int32 ParentIdx = Batch.BodyIndex(Parent, EnvChunk);

					const FVec3x8 Axis = WorldAxisAcc8.Load(Idx);
				const FSpatialVecx8 S{ Axis, FVec3x8::Zero() };
				const FSpatialInertiax8 IBody = IAcc8.Load(Idx);
				const FSpatialVecx8 U = IBody.Apply(S);
				const __m256 D = _mm256_max_ps(FSpatialVecx8::Dot(S, U), _mm256_set1_ps(KINDA_SMALL_NUMBER));

				const int32 DOFIdx = Batch.DOFIndex(Topo.BodyDOFOffset[Body], EnvChunk);
				__m256 StrengthScale = _mm256_set1_ps(1.0f);
				__m256 ActiveMask = _mm256_set1_ps(1.0f);
				if (LimbIdx != INDEX_NONE)
				{
					const int32 LimbSlotIdx = Batch.LimbIndex(LimbIdx, EnvChunk);
					StrengthScale = _mm256_loadu_ps(&Batch.LimbStrengthScale[LimbSlotIdx]);
					ActiveMask = LoadUint8AsFloat8(&Batch.LimbActive[LimbSlotIdx]);
				}
				const __m256 MuscleMul = _mm256_loadu_ps(&MuscleMultiplierAcc[DOFIdx]);
				const __m256 Tau = _mm256_mul_ps(_mm256_mul_ps(_mm256_mul_ps(_mm256_loadu_ps(&Batch.JointTorque[DOFIdx]), StrengthScale), ActiveMask), MuscleMul);

				const FSpatialVecx8 PBody = PAcc8.Load(Idx);
				const __m256 U_u = _mm256_sub_ps(Tau, FSpatialVecx8::Dot(S, PBody));

				UAcc8.Store(Idx, U);
				_mm256_storeu_ps(&DAcc[Idx], D);
				_mm256_storeu_ps(&UScalarAcc[Idx], U_u);

				const __m256 InvD = _mm256_div_ps(_mm256_set1_ps(1.0f), D);
				FSpatialInertiax8 Reduced;
				Reduced.Irot = IBody.Irot - FMat3x8::Outer(U.Ang, U.Ang) * InvD;
				Reduced.H = IBody.H - FMat3x8::Outer(U.Ang, U.Lin) * InvD;
				Reduced.MBlock = IBody.MBlock - FMat3x8::Outer(U.Lin, U.Lin) * InvD;

				const FSpatialVecx8 ReducedBias = PBody + U * _mm256_mul_ps(U_u, InvD);

				const FVec3x8 BodyPos = FVec3x8::Load(&Batch.PosX[Idx], &Batch.PosY[Idx], &Batch.PosZ[Idx]);
				const FVec3x8 ParentPos = FVec3x8::Load(&Batch.PosX[ParentIdx], &Batch.PosY[ParentIdx], &Batch.PosZ[ParentIdx]);
				const FVec3x8 R = BodyPos - ParentPos;

				IAcc8.Accumulate(ParentIdx, Reduced.TranslatedTo(R));
				PAcc8.Accumulate(ParentIdx, TranslateForce(ReducedBias, R));
				}
			}
		});

		// ---- Pass 3a: floating-base root solve (lane-scalar — see class comment) ----
		// Parallelized at the same SIMD-chunk granularity as the other passes
		// (not because this pass needs 8-wide grouping — it's lane-scalar —
		// but to keep one consistent chunk partitioning across the whole
		// Step(), and because SolveSpatial6's branchy pivot search is exactly
		// the kind of per-env work this class comment already calls out as
		// not worth vectorizing, so it stays a plain per-env loop here too.
		ParallelFor(NumChunks, [&](int32 ChunkIdx)
		{
			const int32 EnvBegin = ChunkIdx * SIMDWidth;
			const int32 EnvEnd = FMath::Min(EnvBegin + SIMDWidth, NumEnvs);
			for (int32 Env = EnvBegin; Env < EnvEnd; ++Env)
			{
			const int32 RootIdx = Batch.BodyIndex(0, Env);
			const FSpatialInertia I0 = IAcc8.LoadScalar(RootIdx);
			const FSpatialVec P0 = PAcc8.LoadScalar(RootIdx);
			const FSpatialVec A0 = SolveSpatial6(I0, P0 * -1.0f);
			AAcc8.StoreScalar(RootIdx, A0);

			FVector RootLinVel(Batch.LinVelX[RootIdx], Batch.LinVelY[RootIdx], Batch.LinVelZ[RootIdx]);
			FVector RootAngVel(Batch.AngVelX[RootIdx], Batch.AngVelY[RootIdx], Batch.AngVelZ[RootIdx]);
			RootLinVel += A0.Lin * Dt;
			RootAngVel += A0.Ang * Dt;
			Batch.LinVelX[RootIdx] = (float)RootLinVel.X; Batch.LinVelY[RootIdx] = (float)RootLinVel.Y; Batch.LinVelZ[RootIdx] = (float)RootLinVel.Z;
			Batch.AngVelX[RootIdx] = (float)RootAngVel.X; Batch.AngVelY[RootIdx] = (float)RootAngVel.Y; Batch.AngVelZ[RootIdx] = (float)RootAngVel.Z;

			const FVector RootPos = Batch.GetBodyPos(0, Env) + RootLinVel * Dt;
			FQuat RootRot = Batch.GetBodyRot(0, Env);
			const FVector DTheta = RootAngVel * Dt;
			const float Angle = static_cast<float>(DTheta.Size());
			if (Angle > KINDA_SMALL_NUMBER)
			{
				RootRot = (FQuat(DTheta / Angle, Angle) * RootRot).GetNormalized();
			}
			Batch.SetBodyPos(0, Env, RootPos);
			Batch.SetBodyRot(0, Env, RootRot);
			}
		});

		// ---- Pass 3b: forward acceleration + joint integration (base -> tip) ----
		// Parallelized the same way as Pass 2: one task per SIMD chunk, ball
		// joints running lane-scalar over this task's own [EnvBegin,EnvEnd).
		ParallelFor(NumChunks, [&](int32 ChunkIdx)
		{
			const int32 EnvChunk = ChunkIdx * SIMDWidth;
			const int32 EnvBegin = EnvChunk;
			const int32 EnvEnd = FMath::Min(EnvChunk + SIMDWidth, NumEnvs);

			for (int32 Body = 1; Body < NumBodies; ++Body)
			{
				const int32 Parent = Topo.BodyParent[Body];

				if (Topo.BodyDOFCount[Body] == 3)
				{
				// Lane-scalar — see class comment (mirrors Pass 3a's root solve).
				for (int32 Env = EnvBegin; Env < EnvEnd; ++Env)
				{
					const int32 Idx = Batch.BodyIndex(Body, Env);
					const int32 ParentIdx = Batch.BodyIndex(Parent, Env);
					const int32 DOFIdx = Batch.DOFIndex(Topo.BodyDOFOffset[Body], Env);

					const FVector R = Batch.GetBodyPos(Body, Env) - Batch.GetBodyPos(Parent, Env);
					const FSpatialVec ParentAccAtBody = TranslateMotion(AAcc8.LoadScalar(ParentIdx), R); // see Coriolis-term note above

					const FSpatialInertia IBody = IAcc8.LoadScalar(Idx);
					const FMat3 Dinv = Inverse3x3(IBody.Irot);
					const FVector RHS = BallU_u3Acc[Idx] - (IBody.Irot * ParentAccAtBody.Ang + IBody.H * ParentAccAtBody.Lin);
					const FVector QDDot3 = Dinv * RHS;

					AAcc8.StoreScalar(Idx, ParentAccAtBody + FSpatialVec{ QDDot3, FVector::ZeroVector });

					FVector JointAngVelWorld(Batch.JointVel[DOFIdx], Batch.JointVel[DOFIdx + PaddedNumEnvs], Batch.JointVel[DOFIdx + 2 * PaddedNumEnvs]);
					JointAngVelWorld += QDDot3 * Dt;
					Batch.JointVel[DOFIdx] = (float)JointAngVelWorld.X;
					Batch.JointVel[DOFIdx + PaddedNumEnvs] = (float)JointAngVelWorld.Y;
					Batch.JointVel[DOFIdx + 2 * PaddedNumEnvs] = (float)JointAngVelWorld.Z;

					const FQuat ParentRot = Batch.GetBodyRot(Parent, Env);
					const FVector RelAngVelParentFrame = ParentRot.UnrotateVector(JointAngVelWorld);
					const FVector DTheta = RelAngVelParentFrame * Dt;
					const float Angle = static_cast<float>(DTheta.Size());
					FQuat RelRot = Batch.GetJointRelRot(Body, Env);
					if (Angle > KINDA_SMALL_NUMBER)
					{
						RelRot = (FQuat(DTheta / Angle, Angle) * RelRot).GetNormalized();
					}
					// Canonicalize to W>=0 — see StepScalar's identical line for why.
					if (RelRot.W < 0.0f)
					{
						RelRot = FQuat(-RelRot.X, -RelRot.Y, -RelRot.Z, -RelRot.W);
					}
					Batch.SetJointRelRot(Body, Env, RelRot);

					const FVector ReportRotVec = RelRot.ToRotationVector();
					Batch.JointPos[DOFIdx] = (float)ReportRotVec.X;
					Batch.JointPos[DOFIdx + PaddedNumEnvs] = (float)ReportRotVec.Y;
					Batch.JointPos[DOFIdx + 2 * PaddedNumEnvs] = (float)ReportRotVec.Z;
				}
				continue;
				}

				const int32 Idx = Batch.BodyIndex(Body, EnvChunk);
				const int32 ParentIdx = Batch.BodyIndex(Parent, EnvChunk);

				const FVec3x8 BodyPos = FVec3x8::Load(&Batch.PosX[Idx], &Batch.PosY[Idx], &Batch.PosZ[Idx]);
				const FVec3x8 ParentPos = FVec3x8::Load(&Batch.PosX[ParentIdx], &Batch.PosY[ParentIdx], &Batch.PosZ[ParentIdx]);
				const FVec3x8 R = BodyPos - ParentPos;
				const FSpatialVecx8 ParentAccAtBody = TranslateMotion(AAcc8.Load(ParentIdx), R); // see Coriolis-term note above

				const FVec3x8 Axis = WorldAxisAcc8.Load(Idx);
				const FSpatialVecx8 S{ Axis, FVec3x8::Zero() };
				const FSpatialVecx8 UBody = UAcc8.Load(Idx);
				const __m256 UScalar = _mm256_loadu_ps(&UScalarAcc[Idx]);
				const __m256 D = _mm256_loadu_ps(&DAcc[Idx]);
				const __m256 QDDot = _mm256_div_ps(_mm256_sub_ps(UScalar, FSpatialVecx8::Dot(UBody, ParentAccAtBody)), D);

				AAcc8.Store(Idx, ParentAccAtBody + S * QDDot);

				const int32 DOFIdx = Batch.DOFIndex(Topo.BodyDOFOffset[Body], EnvChunk);
				const __m256 JointVel = _mm256_add_ps(_mm256_loadu_ps(&Batch.JointVel[DOFIdx]), _mm256_mul_ps(QDDot, DtVec));
				_mm256_storeu_ps(&Batch.JointVel[DOFIdx], JointVel);
				const __m256 JointPos = _mm256_add_ps(_mm256_loadu_ps(&Batch.JointPos[DOFIdx]), _mm256_mul_ps(JointVel, DtVec));
				_mm256_storeu_ps(&Batch.JointPos[DOFIdx], JointPos);
			}
		});

		if (ClampJointLimits(Batch))
		{
			RecomputeKinematics(Batch); // see that function's comment — closes the one-step reporting lag a clamp would otherwise leave
		}

		// External forces are NOT cleared here — caller decides whether a hit
		// was a one-shot impulse (call Batch.ClearExternalForces(Env) after
		// Step()) or a sustained push that should persist across steps.
	}

private:
	// StepScalar's scratch (AoS) — kept separate from StepSIMD's so the
	// scalar reference implementation stays byte-for-byte unchanged.
	TArray<FSpatialInertia> IAcc;
	TArray<FSpatialVec> PAcc;
	TArray<FSpatialVec> AAcc;
	TArray<FSpatialVec> UAcc;
	TArray<float> DAcc;
	TArray<float> UScalarAcc;
	TArray<FVector> WorldAxisAcc;

	// StepSIMD's scratch (SoA, see SpatialAlgebraSIMD.h). DAcc/UScalarAcc
	// are already flat float arrays so StepSIMD reuses the same members
	// (resized identically either way) rather than duplicating them.
	FSpatialInertiaSoA IAcc8;
	FSpatialVecSoA PAcc8;
	FSpatialVecSoA AAcc8;
	FSpatialVecSoA UAcc8;
	FVec3SoA WorldAxisAcc8;

	// Ball joints' generalized bias (u = tau - P.Ang), cached between Pass 2
	// and Pass 3b. Shared by StepScalar and StepSIMD since ball-joint math
	// is lane-scalar in both (see class comment) — same reasoning as
	// DAcc/UScalarAcc above.
	TArray<FVector> BallU_u3Acc;

	// Per-DOF-per-env muscle strength multiplier (DOF-major/env-minor, same
	// layout as Batch.JointTorque), recomputed once per Step() and folded
	// into JointTorque in Pass 2 alongside StrengthScale/ActiveMask. Shared
	// by StepScalar and StepSIMD — see ComputeMuscleMultipliers()'s comment
	// for why: this is inherently scalar (FRichCurve::Eval branches), so
	// there's no SIMD path to keep independent for cross-validation, unlike
	// the rest of the ABA math those two functions duplicate on purpose.
	TArray<float> MuscleMultiplierAcc;

	/**
	 * Fills MuscleMultiplierAcc[DOFIdx] for every DOF x env from
	 * Topo.DOFExtensionCurve/DOFFlexionCurve, evaluated at the DOF's current
	 * angle (Batch.JointPos, converted from radians and normalized into the
	 * muscle's authored [MinRange,MaxRangeUnwrapped] range as a 0-1 curve X
	 * value) and picking Extension vs Flexion by the SIGN of the commanded
	 * JointTorque for that DOF — i.e. "how strong can this muscle currently
	 * pull in the direction it's being asked to pull". DOFs with no authored
	 * curve (DOFHasMuscleCurve==false, e.g. every DOF in the synthetic test
	 * topologies, or Muto's still-unarticulated spine) get multiplier 1: no
	 * behavior change for anything that doesn't have this data.
	 */
	void ComputeMuscleMultipliers(const FCreatureBatchState& Batch)
	{
		const FCreatureTopology& Topo = Batch.GetTopology();
		const int32 NumDOF = Topo.NumDOF;
		const int32 NumEnvs = Batch.GetNumEnvs();
		MuscleMultiplierAcc.SetNum(NumDOF * Batch.GetPaddedNumEnvs(), EAllowShrinking::No);

		// Each DOF's row is fully independent (no cross-DOF or cross-env
		// state), so this parallelizes trivially by DOF — no chunking needed.
		ParallelFor(NumDOF, [&](int32 DOF)
		{
			if (!Topo.DOFHasMuscleCurve[DOF])
			{
				for (int32 Env = 0; Env < NumEnvs; ++Env)
				{
					MuscleMultiplierAcc[Batch.DOFIndex(DOF, Env)] = 1.0f;
				}
				return;
			}

			const FRichCurve* ExtCurve = Topo.DOFExtensionCurve[DOF].GetRichCurveConst();
			const FRichCurve* FlexCurve = Topo.DOFFlexionCurve[DOF].GetRichCurveConst();
			const float MinDeg = Topo.DOFRangeMinDeg[DOF];
			const float MaxDeg = Topo.DOFRangeMaxDeg[DOF]; // already unwrapped > MinDeg

			for (int32 Env = 0; Env < NumEnvs; ++Env)
			{
				const int32 Idx = Batch.DOFIndex(DOF, Env);
				const float AngleDeg = FMath::RadiansToDegrees(Batch.JointPos[Idx]);

				// Wrap AngleDeg into [MinDeg, MinDeg+360) so it lines up with
				// MaxDeg the same way the muscle range was authored (values
				// like 315.8 mean -44.2 modulo 360 — see DOFRangeMaxDeg's
				// unwrapping above).
				float Wrapped = FMath::Fmod(AngleDeg - MinDeg, 360.0f);
				if (Wrapped < 0.0f) Wrapped += 360.0f;
				const float T = (MaxDeg > MinDeg) ? FMath::Clamp(Wrapped / (MaxDeg - MinDeg), 0.0f, 1.0f) : 0.0f;

				const float Torque = Batch.JointTorque[Idx];
				const float CurveVal = (Torque >= 0.0f)
					? (ExtCurve ? ExtCurve->Eval(T) : 1.0f)
					: (FlexCurve ? FlexCurve->Eval(T) : 1.0f);
				MuscleMultiplierAcc[Idx] = FMath::Max(CurveVal, 0.0f);
			}
		});
	}

	/**
	 * Hard joint-angle limit enforcement for 1-DOF revolutes: clamps
	 * Batch.JointPos into [DOFRangeMinDeg, DOFRangeMaxDeg] (same wrapped
	 * range ComputeMuscleMultipliers reads — ranges crossing the 0/360
	 * boundary are authored with Min > Max, see DOFRangeMaxDeg's comment)
	 * and zeroes JointVel at the clamp, like an inelastic stop, so the joint
	 * doesn't immediately re-violate the limit or bounce against it forever.
	 *
	 * Added because nothing previously stopped a revolute from physically
	 * rotating past its authored anatomical range at all — DOFRangeMinDeg/
	 * MaxDeg were ONLY ever read by ComputeMuscleMultipliers to scale how
	 * much TORQUE a muscle can currently apply, never to bound POSITION;
	 * with enough momentum/external force a joint could keep going straight
	 * through its limit with no resistance whatsoever. Confirmed as the real
	 * cause of a reported bug (Knee2_R visibly rotating to roughly -80°,
	 * well past its authored MinRange=330.16°/-29.84°) — the user's own
	 * hypothesis (that MinRange needed unwrapping below MaxRange, e.g.
	 * -29.84 instead of 330.16) turned out NOT to be it: MinDeg's absolute
	 * representation is provably irrelevant to ComputeMuscleMultipliers'
	 * T calculation (it's all Fmod-relative arithmetic, invariant to adding
	 * or subtracting 360 from MinDeg) — the actual bug was simply that no
	 * limit enforcement of any kind existed anywhere in the solver.
	 *
	 * Only applied to DOFs with real authored range data
	 * (DOFHasMuscleCurve[DOF]==true) — every DOF in the synthetic test
	 * topologies, and any real DOF lacking curve data (Min==Max==0 by
	 * default, which would otherwise be a degenerate always-clamped range),
	 * is a no-op.
	 *
	 * Ball joints (3-DOF, the limb MOUNT bones) get a CONE limit — added
	 * 2026-08-12 after they turned out to be exactly the gap this comment
	 * used to flag (strong enough contact torque could spin a shoulder/hip
	 * to any orientation with nothing to stop it, visibly "folding" limbs
	 * toward the torso), then revised the SAME day after an initial
	 * per-axis-clamp attempt introduced a worse, genuinely discontinuous
	 * bug (see the ball-joint branch's own comment below for the full
	 * story — independently clamping RelRot's rotation-vector X/Y/Z
	 * components and reconstructing a quaternion from them is NOT a small
	 * correction in general, since those components don't behave like
	 * independent Euler angles). The final approach clamps the rotation
	 * vector's MAGNITUDE only (provably continuous, preserves direction),
	 * which only supports a single uniform cone angle per joint rather
	 * than true independent Roll/Yaw/Pitch ranges — a deliberate precision
	 * trade for robustness. A real swing-twist decomposition would recover
	 * that precision if it's ever needed; not attempted here.
	 *
	 * Runs as its own scalar pass (like ComputeMuscleMultipliers, for the
	 * same reason: branchy modular arithmetic isn't SIMD-friendly) — both
	 * StepScalar/StepSIMD call this exact same function, once, right after
	 * Pass 3b has integrated JointPos/JointVel, so their outputs stay
	 * identical for AgentSolver.CreatureBatchSolverSIMD's parity check.
	 *
	 * Returns true if it clamped ANY joint — callers use this to decide
	 * whether to also call RecomputeKinematics (see that function's
	 * comment for why: this clamp alone leaves BodyPos/BodyRot one step
	 * stale for whatever it just corrected).
	 */
	bool ClampJointLimits(FCreatureBatchState& Batch)
	{
		const FCreatureTopology& Topo = Batch.GetTopology();
		const int32 NumBodies = Topo.NumBodies;
		const int32 NumEnvs = Batch.GetNumEnvs();
		const int32 PaddedNumEnvs = Batch.GetPaddedNumEnvs();
		std::atomic<bool> bAnyClampedGlobal{false};

		// Shared by both branches below: wraps AngleDeg relative to MinDeg
		// into [0,360), and if it's outside [0,Width] — with a small
		// epsilon on BOTH sides of the wrap, since a joint sitting EXACTLY
		// at its own limit is valid but FMath::RadiansToDegrees(FMath::
		// DegreesToRadians(X)) isn't an exact float32 round trip, and
		// landing a hair on the wrong side purely from that noise must not
		// trigger a spurious clamp (confirmed via AgentSolver.MuscleCurve,
		// which starts joints exactly at MinDeg/MaxDeg and briefly
		// regressed without this margin) — returns the clamped angle in
		// the SAME lap as the input (continuous with it, not snapped to a
		// possibly very different-looking but numerically equivalent one).
		auto ClampAngleDeg = [](float AngleDeg, float MinDeg, float Width, bool& bOutClamped) -> float
		{
			float Wrapped = FMath::Fmod(AngleDeg - MinDeg, 360.0f);
			if (Wrapped < 0.0f) Wrapped += 360.0f;

			constexpr float Epsilon = 0.01f;
			bOutClamped = (Wrapped > Width + Epsilon) && (Wrapped < 360.0f - Epsilon);
			if (!bOutClamped)
			{
				return AngleDeg;
			}
			const float ClampedWrapped = (Wrapped - Width < 360.0f - Wrapped) ? Width : 0.0f; // whichever boundary is circularly nearer
			const float LapBase = AngleDeg - Wrapped;
			return LapBase + ClampedWrapped;
		};

		ParallelFor(NumBodies, [&](int32 Body)
		{
			if (Body == 0)
			{
				return;
			}

			if (Topo.BodyDOFCount[Body] == 1)
			{
				const int32 DOF = Topo.BodyDOFOffset[Body];
				if (!Topo.DOFHasMuscleCurve[DOF])
				{
					return;
				}
				const float MinDeg = Topo.DOFRangeMinDeg[DOF];
				const float Width = Topo.DOFRangeMaxDeg[DOF] - MinDeg;
				if (Width <= 0.0f)
				{
					return;
				}

				for (int32 Env = 0; Env < NumEnvs; ++Env)
				{
					const int32 DOFIdx = Batch.DOFIndex(DOF, Env);
					const float AngleDeg = FMath::RadiansToDegrees(Batch.JointPos[DOFIdx]);
					bool bClamped = false;
					const float ClampedDeg = ClampAngleDeg(AngleDeg, MinDeg, Width, bClamped);
					if (bClamped)
					{
						Batch.JointPos[DOFIdx] = FMath::DegreesToRadians(ClampedDeg);
						Batch.JointVel[DOFIdx] = 0.0f;
						bAnyClampedGlobal.store(true, std::memory_order_relaxed);
					}
				}
			}
			else if (Topo.BodyDOFCount[Body] == 3)
			{
				// SUPERSEDES an earlier (2026-08-12) attempt at independent
				// per-axis clamping + iterative reconstruction: clamping
				// rotation-vector X/Y/Z components independently and
				// rebuilding a quaternion via MakeFromRotationVector is NOT
				// a small, continuous correction in general — rotation-
				// vector components don't behave like independent Euler
				// angles (changing one changes the reconstructed rotation's
				// ENTIRE axis and angle together), so even a modest
				// per-axis correction can snap the joint to a wildly
				// different orientation. Confirmed via a throwaway
				// diagnostic: ball joints were "rotating" 30-56° in a
				// SINGLE 1/240s substep with near-zero reported angular
				// velocity — not real motion, an artifact of that
				// reconstruction — and adding more refinement iterations
				// (tried first) didn't help, because the problem isn't
				// convergence, it's that the whole per-axis approach is
				// mathematically unsound for a multi-axis correction.
				//
				// Fix: clamp the rotation VECTOR'S MAGNITUDE only, scaling
				// it down toward zero while preserving its axis/direction
				// exactly. This is provably continuous (a smooth rescale,
				// never a discontinuous jump) at the cost of precision —
				// it enforces a single CONE limit (uniform in every
				// direction) per ball joint instead of true independent
				// Roll/Yaw/Pitch ranges. The cone's half-angle is the
				// LARGEST of the joint's 3 authored per-axis widths, so it
				// can reach at least as far as its most permissive
				// authored axis in any direction — intentionally a looser,
				// safer approximation rather than a tighter, riskier one.
				const int32 DOFOffset = Topo.BodyDOFOffset[Body];
				float MaxWidthDeg = 0.0f;
				bool bHasAnyRange = false;
				for (int32 k = 0; k < 3; ++k)
				{
					const int32 DOF = DOFOffset + k;
					if (Topo.DOFHasMuscleCurve[DOF])
					{
						bHasAnyRange = true;
						MaxWidthDeg = FMath::Max(MaxWidthDeg, Topo.DOFRangeMaxDeg[DOF] - Topo.DOFRangeMinDeg[DOF]);
					}
				}
				if (!bHasAnyRange || MaxWidthDeg <= 0.0f)
				{
					return;
				}
				const float MaxAngleRad = FMath::DegreesToRadians(MaxWidthDeg);

				for (int32 Env = 0; Env < NumEnvs; ++Env)
				{
					const FVector RotVec = Batch.GetJointRelRot(Body, Env).ToRotationVector();
					const float Angle = static_cast<float>(RotVec.Size());
					if (Angle > MaxAngleRad)
					{
						bAnyClampedGlobal.store(true, std::memory_order_relaxed);
						const FVector ClampedRotVec = RotVec * (MaxAngleRad / Angle);
						Batch.SetJointRelRot(Body, Env, FQuat::MakeFromRotationVector(ClampedRotVec));
						const int32 DOFIdx = Batch.DOFIndex(DOFOffset, Env);
						Batch.JointPos[DOFIdx] = (float)ClampedRotVec.X;
						Batch.JointPos[DOFIdx + PaddedNumEnvs] = (float)ClampedRotVec.Y;
						Batch.JointPos[DOFIdx + 2 * PaddedNumEnvs] = (float)ClampedRotVec.Z;
						// Full stop, all 3 axes — same inelastic-limit-stop
						// choice as the revolute case.
						Batch.JointVel[DOFIdx] = 0.0f;
						Batch.JointVel[DOFIdx + PaddedNumEnvs] = 0.0f;
						Batch.JointVel[DOFIdx + 2 * PaddedNumEnvs] = 0.0f;
					}
				}
			}
		});

		return bAnyClampedGlobal.load(std::memory_order_relaxed);
	}

	/**
	 * Forward-kinematics-only refresh: recomputes BodyRot/BodyPos/AngVel/
	 * LinVel for every body from the CURRENT JointPos/JointVel/JointRelRot
	 * state — i.e. exactly Pass 1, without touching JointPos/JointVel
	 * themselves or running Pass 2/3.
	 *
	 * Exists to close a real one-step lag: ClampJointLimits runs at the END
	 * of Step()/StepScalar()/StepSIMD(), AFTER Pass 1 already computed THIS
	 * step's BodyPos/BodyRot from the PRE-clamp JointPos — so a correction
	 * ClampJointLimits makes this step isn't reflected in BodyPos/BodyRot
	 * until the FOLLOWING step's own Pass 1 runs. Added while investigating
	 * a user-reported "reaches max angle, fights, then teleports somewhere
	 * else, generally up" bug (2026-08-12) — this lag turned out NOT to be
	 * that bug's actual cause (a throwaway diagnostic confirmed identical
	 * behavior with and without this fix; the real cause was the ball-joint
	 * clamp's per-axis reconstruction, see that branch's comment), but the
	 * lag itself is real and worth closing regardless: without this, ANY
	 * clamp correction is invisible to BodyPos/BodyRot for one full
	 * substep, which is still a genuine (if usually small) staleness bug
	 * for anything sampling positions every step. Called automatically by
	 * Step()/StepScalar()/StepSIMD() whenever ClampJointLimits returns
	 * true, closing the gap the same step it happens instead of one step
	 * late.
	 */
	void RecomputeKinematics(FCreatureBatchState& Batch)
	{
		const FCreatureTopology& Topo = Batch.GetTopology();
		const int32 NumBodies = Topo.NumBodies;
		const int32 NumEnvs = Batch.GetNumEnvs();

		ParallelFor(NumEnvs, [&](int32 Env)
		{
			for (int32 Body = 1; Body < NumBodies; ++Body)
			{
				const int32 Parent = Topo.BodyParent[Body];
				const bool bIsBall = (Topo.BodyDOFCount[Body] == 3);

				const int32 Idx = Batch.BodyIndex(Body, Env);
				const int32 ParentIdx = Batch.BodyIndex(Parent, Env);

				const FQuat ParentRot = Batch.GetBodyRot(Parent, Env);
				const FVector ParentPos = Batch.GetBodyPos(Parent, Env);
				const FVector ParentAngVel(Batch.AngVelX[ParentIdx], Batch.AngVelY[ParentIdx], Batch.AngVelZ[ParentIdx]);
				const FVector ParentLinVel(Batch.LinVelX[ParentIdx], Batch.LinVelY[ParentIdx], Batch.LinVelZ[ParentIdx]);

				const int32 DOFIdx = Batch.DOFIndex(Topo.BodyDOFOffset[Body], Env);

				FQuat BodyRot;
				FVector JointAngVelWorld;
				if (bIsBall)
				{
					const FQuat RelRot = Batch.GetJointRelRot(Body, Env);
					BodyRot = (ParentRot * Topo.BodyRestRotInParent[Body] * RelRot).GetNormalized();
					JointAngVelWorld = FVector(Batch.JointVel[DOFIdx], Batch.JointVel[DOFIdx + Batch.GetPaddedNumEnvs()], Batch.JointVel[DOFIdx + 2 * Batch.GetPaddedNumEnvs()]);
				}
				else
				{
					const float JointAngle = Batch.JointPos[DOFIdx];
					const float JointRate = Batch.JointVel[DOFIdx];
					const FVector AxisWorld = ParentRot.RotateVector(Topo.BodyJointAxisLocal[Body]);
					BodyRot = (FQuat(AxisWorld, JointAngle) * ParentRot * Topo.BodyRestRotInParent[Body]).GetNormalized();
					JointAngVelWorld = AxisWorld * JointRate;
				}

				const FVector JointOffsetWorld = ParentRot.RotateVector(Topo.BodyJointOffsetInParent[Body]);
				const FVector BodyPos = ParentPos + JointOffsetWorld;
				Batch.SetBodyPos(Body, Env, BodyPos);
				Batch.SetBodyRot(Body, Env, BodyRot);

				const FVector R = BodyPos - ParentPos;
				const FVector AngVel = ParentAngVel + JointAngVelWorld;
				const FVector LinVel = ParentLinVel + FVector::CrossProduct(ParentAngVel, R);

				Batch.AngVelX[Idx] = (float)AngVel.X; Batch.AngVelY[Idx] = (float)AngVel.Y; Batch.AngVelZ[Idx] = (float)AngVel.Z;
				Batch.LinVelX[Idx] = (float)LinVel.X; Batch.LinVelY[Idx] = (float)LinVel.Y; Batch.LinVelZ[Idx] = (float)LinVel.Z;
			}
		});
	}

public:
	/**
	 * Caps how fast any joint can rotate — a direct "slow the muscles down"
	 * knob, independent of MaxTorquePerDOF (limits FORCE, not speed) or the
	 * strength curves (scale force by angle, not speed). Optional: callers
	 * that don't want a speed limit (the original behavior) simply don't
	 * call this — unlike ClampJointLimits/ComputeMuscleMultipliers, this is
	 * NOT called automatically from Step()/StepScalar()/StepSIMD(), since
	 * MaxSpeedRad is a per-driver tuning knob (see AMutoRLTrainingDriver::
	 * MaxJointSpeedDegPerSec), not authored topology data.
	 *
	 * For 1-DOF revolutes, clamps JointVel to [-MaxSpeedRad, MaxSpeedRad]
	 * directly. For 3-DOF ball joints, JointVel stores actual world-space
	 * angular velocity (NOT rotation-vector components — unlike JointPos,
	 * see Pass 3's comment), so it's clamped as a single vector magnitude
	 * (uniformly rescaled if too fast, preserving direction) rather than
	 * per-axis.
	 */
	void ClampJointSpeed(FCreatureBatchState& Batch, float MaxSpeedRad)
	{
		if (MaxSpeedRad <= 0.0f)
		{
			return;
		}

		const FCreatureTopology& Topo = Batch.GetTopology();
		const int32 NumBodies = Topo.NumBodies;
		const int32 NumEnvs = Batch.GetNumEnvs();
		const int32 PaddedNumEnvs = Batch.GetPaddedNumEnvs();

		ParallelFor(NumBodies, [&](int32 Body)
		{
			if (Body == 0)
			{
				return;
			}

			if (Topo.BodyDOFCount[Body] == 1)
			{
				const int32 DOF = Topo.BodyDOFOffset[Body];
				for (int32 Env = 0; Env < NumEnvs; ++Env)
				{
					const int32 DOFIdx = Batch.DOFIndex(DOF, Env);
					Batch.JointVel[DOFIdx] = FMath::Clamp(Batch.JointVel[DOFIdx], -MaxSpeedRad, MaxSpeedRad);
				}
			}
			else if (Topo.BodyDOFCount[Body] == 3)
			{
				const int32 DOFOffset = Topo.BodyDOFOffset[Body];
				for (int32 Env = 0; Env < NumEnvs; ++Env)
				{
					const int32 DOFIdx = Batch.DOFIndex(DOFOffset, Env);
					FVector AngVel(Batch.JointVel[DOFIdx], Batch.JointVel[DOFIdx + PaddedNumEnvs], Batch.JointVel[DOFIdx + 2 * PaddedNumEnvs]);
					const float Speed = static_cast<float>(AngVel.Size());
					if (Speed > MaxSpeedRad)
					{
						AngVel *= MaxSpeedRad / Speed;
						Batch.JointVel[DOFIdx] = (float)AngVel.X;
						Batch.JointVel[DOFIdx + PaddedNumEnvs] = (float)AngVel.Y;
						Batch.JointVel[DOFIdx + 2 * PaddedNumEnvs] = (float)AngVel.Z;
					}
				}
			}
		});
	}
};
