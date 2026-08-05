#pragma once

#include "CoreMinimal.h"
#include "CreatureBatchState.h"
#include "SpatialAlgebra.h"

/**
 * Batched Featherstone Articulated Body Algorithm over an FCreatureBatchState.
 *
 * Scope of this version:
 *  - Body 0 (torso) is a free-floating 6-DOF base — solved via a direct 6x6
 *    linear solve of its fully-composited articulated inertia, since no joint
 *    reduction applies to it.
 *  - All other bodies are 1-DOF revolute joints. 3-DOF ball joints (if you
 *    end up wanting them at hips/shoulders) need a block matrix inversion
 *    instead of the scalar U/d reduction below — a follow-up, not covered here.
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
 * Per-body-per-env loops below are written as plain nested loops rather than
 * explicit SIMD — correctness first. The body-major/env-minor layout from
 * FCreatureBatchState is exactly what makes the inner (env) loop vectorizable
 * later; converting it is a follow-up once this is validated.
 */
class FCreatureABASolver
{
public:
	void Step(FCreatureBatchState& Batch, float Dt, const FVector& Gravity = FVector(0.0f, 0.0f, -980.0f))
	{
		const FCreatureTopology& Topo = Batch.GetTopology();
		const int32 NumBodies = Topo.NumBodies;
		const int32 NumEnvs = Batch.GetNumEnvs();
		const int32 BodySlots = NumBodies * Batch.GetPaddedNumEnvs();

		IAcc.SetNum(BodySlots, EAllowShrinking::No);
		PAcc.SetNum(BodySlots, EAllowShrinking::No);
		AAcc.SetNum(BodySlots, EAllowShrinking::No);
		UAcc.SetNum(BodySlots, EAllowShrinking::No);
		DAcc.SetNumZeroed(BodySlots, EAllowShrinking::No);
		UScalarAcc.SetNumZeroed(BodySlots, EAllowShrinking::No);
		WorldAxisAcc.SetNum(BodySlots, EAllowShrinking::No);

		// ---- Pass 1: forward kinematics + velocity propagation (base -> tip) ----
		for (int32 Body = 1; Body < NumBodies; ++Body)
		{
			const int32 Parent = Topo.BodyParent[Body];
			for (int32 Env = 0; Env < NumEnvs; ++Env)
			{
				const int32 Idx = Batch.BodyIndex(Body, Env);
				const int32 ParentIdx = Batch.BodyIndex(Parent, Env);

				const FQuat ParentRot = Batch.GetBodyRot(Parent, Env);
				const FVector ParentPos = Batch.GetBodyPos(Parent, Env);
				const FVector ParentAngVel(Batch.AngVelX[ParentIdx], Batch.AngVelY[ParentIdx], Batch.AngVelZ[ParentIdx]);
				const FVector ParentLinVel(Batch.LinVelX[ParentIdx], Batch.LinVelY[ParentIdx], Batch.LinVelZ[ParentIdx]);

				const int32 DOFIdx = Batch.DOFIndex(Topo.BodyDOFOffset[Body], Env);
				const float JointAngle = Batch.JointPos[DOFIdx];
				const float JointRate = Batch.JointVel[DOFIdx];

				// Axis is invariant under rotation about itself, so rotating the
				// rest-pose axis by the PARENT's current rotation is valid even
				// though the joint angle hasn't been applied yet at this point.
				const FVector AxisWorld = ParentRot.RotateVector(Topo.BodyJointAxisLocal[Body]);
				WorldAxisAcc[Idx] = AxisWorld;

				const FVector JointOffsetWorld = ParentRot.RotateVector(Topo.BodyJointOffsetInParent[Body]);
				const FVector BodyPos = ParentPos + JointOffsetWorld;
				const FQuat BodyRot = (FQuat(AxisWorld, JointAngle) * ParentRot).GetNormalized();
				Batch.SetBodyPos(Body, Env, BodyPos);
				Batch.SetBodyRot(Body, Env, BodyRot);

				const FVector R = BodyPos - ParentPos; // parent -> body, world
				const FVector AngVel = ParentAngVel + AxisWorld * JointRate;
				const FVector LinVel = ParentLinVel + FVector::CrossProduct(ParentAngVel, R);

				Batch.AngVelX[Idx] = (float)AngVel.X; Batch.AngVelY[Idx] = (float)AngVel.Y; Batch.AngVelZ[Idx] = (float)AngVel.Z;
				Batch.LinVelX[Idx] = (float)LinVel.X; Batch.LinVelY[Idx] = (float)LinVel.Y; Batch.LinVelZ[Idx] = (float)LinVel.Z;
			}
		}

		// ---- Own rigid-body inertia + bias force, every body (order-independent) ----
		for (int32 Body = 0; Body < NumBodies; ++Body)
		{
			for (int32 Env = 0; Env < NumEnvs; ++Env)
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
		}

		// ---- Pass 2: backward accumulation (tip -> base), reduce each 1-DOF joint ----
		for (int32 Body = NumBodies - 1; Body >= 1; --Body)
		{
			const int32 Parent = Topo.BodyParent[Body];
			for (int32 Env = 0; Env < NumEnvs; ++Env)
			{
				const int32 Idx = Batch.BodyIndex(Body, Env);
				const int32 ParentIdx = Batch.BodyIndex(Parent, Env);

				const FSpatialVec S{ WorldAxisAcc[Idx], FVector::ZeroVector };
				const FSpatialVec U = IAcc[Idx].Apply(S);
				const float D = FMath::Max(FSpatialVec::Dot(S, U), KINDA_SMALL_NUMBER);

				const int32 DOFIdx = Batch.DOFIndex(Topo.BodyDOFOffset[Body], Env);
				const int32 LimbIdx = Topo.BodyLimbIndex[Body];
				const float StrengthScale = (LimbIdx != INDEX_NONE) ? Batch.LimbStrengthScale[Batch.LimbIndex(LimbIdx, Env)] : 1.0f;
				const float ActiveMask = (LimbIdx != INDEX_NONE) ? (float)Batch.LimbActive[Batch.LimbIndex(LimbIdx, Env)] : 1.0f;
				const float Tau = Batch.JointTorque[DOFIdx] * StrengthScale * ActiveMask;

				const float U_u = Tau - FSpatialVec::Dot(S, PAcc[Idx]);

				UAcc[Idx] = U;
				DAcc[Idx] = D;
				UScalarAcc[Idx] = U_u;

				FSpatialInertia Reduced;
				const float InvD = 1.0f / D;
				Reduced.Irot = IAcc[Idx].Irot - FMat3::Outer(U.Ang, U.Ang) * InvD;
				Reduced.H = IAcc[Idx].H - FMat3::Outer(U.Ang, U.Lin) * InvD;
				Reduced.MBlock = IAcc[Idx].MBlock - FMat3::Outer(U.Lin, U.Lin) * InvD;

				const FSpatialVec ReducedBias = PAcc[Idx] + U * (U_u * InvD);

				const FVector R = Batch.GetBodyPos(Body, Env) - Batch.GetBodyPos(Parent, Env);
				IAcc[ParentIdx] = IAcc[ParentIdx] + Reduced.TranslatedTo(R);
				PAcc[ParentIdx] = PAcc[ParentIdx] + TranslateForce(ReducedBias, R);
			}
		}

		// ---- Pass 3a: floating-base root solve ----
		for (int32 Env = 0; Env < NumEnvs; ++Env)
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
		}

		// ---- Pass 3b: forward acceleration + joint integration (base -> tip) ----
		for (int32 Body = 1; Body < NumBodies; ++Body)
		{
			const int32 Parent = Topo.BodyParent[Body];
			for (int32 Env = 0; Env < NumEnvs; ++Env)
			{
				const int32 Idx = Batch.BodyIndex(Body, Env);
				const int32 ParentIdx = Batch.BodyIndex(Parent, Env);

				const FVector R = Batch.GetBodyPos(Body, Env) - Batch.GetBodyPos(Parent, Env);
				const FSpatialVec ParentAccAtBody = TranslateMotion(AAcc[ParentIdx], R); // see Coriolis-term note above

				const FSpatialVec S{ WorldAxisAcc[Idx], FVector::ZeroVector };
				const float QDDot = (UScalarAcc[Idx] - FSpatialVec::Dot(UAcc[Idx], ParentAccAtBody)) / DAcc[Idx];

				AAcc[Idx] = ParentAccAtBody + S * QDDot;

				const int32 DOFIdx = Batch.DOFIndex(Topo.BodyDOFOffset[Body], Env);
				Batch.JointVel[DOFIdx] += QDDot * Dt;
				Batch.JointPos[DOFIdx] += Batch.JointVel[DOFIdx] * Dt;
			}
		}

		// External forces are NOT cleared here — caller decides whether a hit
		// was a one-shot impulse (call Batch.ClearExternalForces(Env) after
		// Step()) or a sustained push that should persist across steps.
	}

private:
	TArray<FSpatialInertia> IAcc;
	TArray<FSpatialVec> PAcc;
	TArray<FSpatialVec> AAcc;
	TArray<FSpatialVec> UAcc;
	TArray<float> DAcc;
	TArray<float> UScalarAcc;
	TArray<FVector> WorldAxisAcc;
};