#pragma once

#include "CoreMinimal.h"
#include "PhysicsSolver/CreatureBatchState.h"
#include "PhysicsSolver/SpatialAlgebra.h"
#include "PhysicsSolver/SpatialAlgebraSIMD.h"
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
	/**
	 * Per-joint PASSIVE parameters (armature + damping), resolved from topology
	 * once per body so that every code path that touches D agrees exactly.
	 *
	 * There are FIVE such paths and they must not drift: StepScalar's Pass 2 and
	 * Pass 3b, StepSIMD's Pass 2 and Pass 3b, plus the impulse pair
	 * ComputeArticulatedInertias / SolveImpulseResponse. If the force pass and
	 * the impulse pass disagree about D, the contact solver sizes its impulses
	 * against a different creature than the one being integrated — which is
	 * exactly the failure mode SOLVER_DEBUG_LOG.md entry 024 hit when locking
	 * joints in the factorization alone turned out to be a silent no-op because
	 * SolveImpulseResponse redid the reduction itself.
	 *
	 * THE ALGEBRA, spelled out, because the ball-joint case is not obvious.
	 *
	 * Armature adds A to the JOINT inertia D, not to the body inertia I^A. For a
	 * revolute that is scalar: D' = S^T I^A S + A, and taking A = r * (S^T I^A S)
	 * makes it simply D' = D * (1+r) — every downstream expression already
	 * divides by D, so nothing else changes.
	 *
	 * For a ball joint S = [I3; 0], so U = [Irot; H^T] and D = Irot. The current
	 * code exploits D == Irot to conclude that the reduced inertia's Irot and H
	 * blocks vanish identically. With armature that is no longer true, and the
	 * general form
	 *     I^a = I^A - U D^-1 U^T
	 * has to be used. Written naively that reintroduces catastrophic
	 * cancellation (Irot - Irot*D^-1*Irot on quantities of order 1e10 in
	 * float32), so it is written via the exact identities instead. With
	 * D = Irot*(1+r) and A = Irot*r:
	 *
	 *     I - Irot*D^-1  = A*D^-1                    (since Irot = D - A)
	 *     Irot - Irot*D^-1*Irot = Irot*D^-1*A = Irot * r/(1+r)
	 *     H    - Irot*D^-1*H    = A*D^-1*H    = H    * r/(1+r)
	 *     Irot*D^-1*u           = u - A*D^-1*u = u   * 1/(1+r)
	 *
	 * Every one of those is a clean scalar multiple of `ArmatureFrac = r/(1+r)`
	 * with NO subtraction of like-magnitude terms, and every one is EXACTLY zero
	 * at r = 0. So the default path is bit-identical to the pre-armature solver
	 * rather than merely close — which matters, because
	 * AgentSolver.CreatureBatchSolverSIMD compares scalar against SIMD to a
	 * tolerance, and AgentSolver.PendulumEnergyConservation to 1e-6.
	 *
	 * Isotropy for ball joints is structural, not laziness: JointVel at a ball
	 * joint stores world-space angular velocity (see FCreatureBatchState's
	 * JointRelRotX comment), so a per-axis coefficient would be a coefficient on
	 * world axes and would change meaning as the creature turned. Slot 0 of the
	 * body's three DOF entries is authoritative.
	 */
	struct FJointPassiveFactors
	{
		/** D_effective = D_articulated * ArmatureScale. 1 = no armature. */
		float ArmatureScale = 1.0f;
		/** r/(1+r) — the ball-joint reduced-block coefficient derived above. 0 = no armature. */
		float ArmatureFrac = 0.0f;
		/**
		 * Fraction of the joint's own velocity that damping removes this substep,
		 * = clamp(Dt / DampingTimeConstant, 0, 1). Consumed by ApplyJointDamping,
		 * NOT by Pass 2 — damping is a velocity-level impulse here, not a torque,
		 * for the reason set out at length on that function. The clamp at 1 is
		 * what makes it unconditionally stable: damping can remove all of qd and
		 * never reverse it.
		 */
		float DampFrac = 0.0f;

		FORCEINLINE bool HasDamping() const { return DampFrac > 0.0f; }
	};

	/**
	 * Dt <= 0 disables damping entirely rather than dividing by it. Several
	 * diagnostics deliberately call Step() with Dt == 0 purely to refresh
	 * kinematics (MutoScalarSIMDParityTest does), and a passive force that
	 * depends on 1/Dt has no meaning there.
	 */
	static FJointPassiveFactors GetJointPassiveFactors(const FCreatureTopology& Topo, int32 Body, float Dt)
	{
		FJointPassiveFactors F;
		const int32 DOF = Topo.BodyDOFOffset[Body];
		if (!Topo.DOFArmatureRatio.IsValidIndex(DOF))
		{
			return F; // topology predates these arrays (or NumDOF == 0) — neutral
		}

		const float Ratio = FMath::Max(0.0f, Topo.DOFArmatureRatio[DOF]);
		if (Ratio > 0.0f)
		{
			F.ArmatureScale = 1.0f + Ratio;
			F.ArmatureFrac = Ratio / F.ArmatureScale;
		}

		const float TimeConst = Topo.DOFDampingTimeConstant[DOF];
		if (TimeConst > 0.0f && Dt > 0.0f)
		{
			F.DampFrac = FMath::Min(Dt / TimeConst, 1.0f);
		}
		return F;
	}

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
		// Zeroed, not merely sized: body 0 never writes its own C (Pass 1 starts
		// at Body 1) and Pass 2/3b read it, so it must be a reliable zero.
		CAcc.SetNumZeroed(BodySlots, EAllowShrinking::No);
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

				// Velocity-product bias — see CAcc's derivation.
				CAcc[Idx] = FSpatialVec{
					bIsBall ? FVector::ZeroVector : FVector::CrossProduct(ParentAngVel, JointAngVelWorld),
					FVector::CrossProduct(LinVel, JointAngVelWorld)
				};
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

				const FVector GravityForceWorld = Mass * Topo.BodyGravityScale[Body] * Gravity;
				const FSpatialVec GravityWrench{ FVector::CrossProduct(CoMOffset, GravityForceWorld), GravityForceWorld };

				// Gyroscopic (velocity-product) term minus the applied gravity wrench.
				IAcc[Idx] = Own;
				PAcc[Idx] = SpatialCrossForce(V, Own.Apply(V)) - GravityWrench;
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

				// WHERE THE VELOCITY-PRODUCT TERM GOES (Featherstone Table 7.1):
				//     u   = tau - S^T p^A            <- p^A, NOT p^A + I c
				//     p^a = p^A + I^a c + U D^-1 u   <- I^a, the REDUCED inertia
				// The generalized bias u is computed from the plain p^A; the c
				// term enters ONLY the parent accumulation, and it is multiplied
				// by the reduced inertia I^a, not the articulated I^A.
				// Getting either of those wrong destabilizes the solver rather
				// than fixing it — both mistakes were made and measured before
				// this landed (SOLVER_DEBUG_LOG.md entry 005).
				// Armature and damping — see FJointPassiveFactors for the algebra
				// and for why both are exact no-ops at their zero defaults.
				const FJointPassiveFactors Passive = GetJointPassiveFactors(Topo, Body, Dt);

				if (bIsBall)
				{
					// With no armature, D == Irot exactly and the reduced inertia's
					// Irot/H blocks vanish (a spherical joint transmits no angular
					// coupling to the parent, only the reflected linear inertia).
					// Armature breaks that identity; Passive.ArmatureFrac carries the
					// general form without cancellation. See FJointPassiveFactors.
					const FMat3 Irot = IAcc[Idx].Irot;
					const FMat3 H = IAcc[Idx].H;
					// D = Irot * ArmatureScale, inverted as the scaled inverse so the
					// ArmatureScale == 1 case is bit-identical to Inverse3x3(Irot).
					const FMat3 Dinv = (Passive.ArmatureScale == 1.0f)
						? Inverse3x3(Irot)
						: Inverse3x3(Irot) * (1.0f / Passive.ArmatureScale);

					// Damping is NOT applied here — see ApplyJointDamping for why a
					// torque folded into tau cannot deliver a predictable amount of
					// damping in an articulated tree.
					const FVector Tau3(
						Batch.JointTorque[DOFIdx] * StrengthScale * ActiveMask * MuscleMultiplierAcc[DOFIdx],
						Batch.JointTorque[DOFIdx + PaddedNumEnvs] * StrengthScale * ActiveMask * MuscleMultiplierAcc[DOFIdx + PaddedNumEnvs],
						Batch.JointTorque[DOFIdx + 2 * PaddedNumEnvs] * StrengthScale * ActiveMask * MuscleMultiplierAcc[DOFIdx + 2 * PaddedNumEnvs]);
					const FVector U_u3 = Tau3 - PAcc[Idx].Ang;
					BallU_u3Acc[Idx] = U_u3;

					// Irot*D^-1*A and A*D^-1*H, both == Irot/H scaled by r/(1+r).
					Reduced.Irot = Irot * Passive.ArmatureFrac;
					Reduced.H = H * Passive.ArmatureFrac;
					Reduced.MBlock = IAcc[Idx].MBlock - H.Transpose() * Dinv * H;

					// + I^a c. With no armature Reduced.Irot/H are zero, so this
					// contributes only Reduced.MBlock * C.Lin to the linear part.
					//
					// Angular part is p^A.Ang + Irot*D^-1*u, which equals Tau3 exactly
					// when there is no armature (Irot*D^-1 == identity, so the term
					// collapses to u = Tau3 - p^A.Ang) and Tau3 - u*r/(1+r) otherwise.
					ReducedBias = FSpatialVec{ Tau3 - U_u3 * Passive.ArmatureFrac,
						PAcc[Idx].Lin + H.Transpose() * (Dinv * U_u3) }
						+ Reduced.Apply(CAcc[Idx]);
				}
				else
				{
					const FSpatialVec S{ WorldAxisAcc[Idx], FVector::ZeroVector };
					const FSpatialVec U = IAcc[Idx].Apply(S);
					// Armature enters as D * (1+r); every expression below already
					// divides by D, so this is the whole of it for a revolute.
					const float D = FMath::Max(FSpatialVec::Dot(S, U), KINDA_SMALL_NUMBER) * Passive.ArmatureScale;
					// Damping is NOT applied here — see ApplyJointDamping.
					const float Tau = Batch.JointTorque[DOFIdx] * StrengthScale * ActiveMask * MuscleMultiplierAcc[DOFIdx];
					const float U_u = Tau - FSpatialVec::Dot(S, PAcc[Idx]);

					UAcc[Idx] = U;
					DAcc[Idx] = D;
					UScalarAcc[Idx] = U_u;

					const float InvD = 1.0f / D;
					Reduced.Irot = IAcc[Idx].Irot - FMat3::Outer(U.Ang, U.Ang) * InvD;
					Reduced.H = IAcc[Idx].H - FMat3::Outer(U.Ang, U.Lin) * InvD;
					Reduced.MBlock = IAcc[Idx].MBlock - FMat3::Outer(U.Lin, U.Lin) * InvD;

					ReducedBias = PAcc[Idx] + Reduced.Apply(CAcc[Idx]) + U * (U_u * InvD);
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
			// SPATIAL -> CLASSICAL. The equation assembled above is Featherstone's
			// f = I a + v x* (I v), in which `a` is the SPATIAL acceleration: its
			// linear part is the rate of change of the velocity field sampled at a
			// point FIXED in space, not the acceleration of the body's own moving
			// material reference point. The two differ by w x v:
			//     d(u_material)/dt = A.Lin + w x u
			// Integrating A.Lin directly (what this used to do) therefore injects a
			// spurious -w x u drift, perpendicular to the velocity, which curves a
			// straight free-flight path and grows with w.
			//
			// Sanity check on the simplest case: a free body with its reference AT
			// the CoM and no external force gives A.Lin = -w x u exactly, so the
			// corrected update yields du/dt = 0 — a constant CoM velocity, i.e.
			// Newton's first law. Before this fix that same case drifted from
			// (10,0,0) to (6.3,9.1,-5.7) in 5 s, and the drift did NOT shrink with
			// a finer timestep (see SOLVER_DEBUG_LOG.md entry 005), which is the
			// signature of a modelling error rather than an integration error.
			//
			// Uses the PRE-update angular velocity deliberately: both terms are
			// evaluated at the start of the step, matching the explicit ordering
			// the rest of this pass already uses.
			RootLinVel += (A0.Lin + FVector::CrossProduct(RootAngVel, RootLinVel)) * Dt;
			RootAngVel += A0.Ang * Dt;
			Batch.LinVelX[RootIdx] = (float)RootLinVel.X; Batch.LinVelY[RootIdx] = (float)RootLinVel.Y; Batch.LinVelZ[RootIdx] = (float)RootLinVel.Z;
			Batch.AngVelX[RootIdx] = (float)RootAngVel.X; Batch.AngVelY[RootIdx] = (float)RootAngVel.Y; Batch.AngVelZ[RootIdx] = (float)RootAngVel.Z;

			const FVector RootPos = Batch.GetBodyPos(0, Env) + RootLinVel * Dt;
			FQuat RootRot = Batch.GetBodyRot(0, Env);
			const FVector DTheta = RootAngVel * Dt;
			const float Angle = static_cast<float>(DTheta.Size());
			// SMALL_NUMBER (1e-8), not KINDA_SMALL_NUMBER (1e-4). This guard only
			// exists to avoid normalizing a zero axis; using the coarse tolerance
			// silently DISCARDED every rotation smaller than 1e-4 rad per step.
			// At w = 0.3 rad/s that threshold is crossed at ~3000 Hz, so at any
			// fine substep rate the orientation simply froze while the angular
			// velocity kept integrating — the finer the timestep, the more
			// rotation was thrown away, which is why momentum drift got WORSE as
			// dt shrank instead of converging. Measured: a closed 2-body chain
			// drifted 6e-7 at 1000 Hz but 7.6e-4 at 4000 Hz purely from this.
			// See SOLVER_DEBUG_LOG.md entry 012.
			if (Angle > SMALL_NUMBER)
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
				// TranslateMotion gives only a_p + alpha_p x R; the velocity-product
				// remainder is CAcc (see its derivation).
				const FSpatialVec ParentAccAtBody = TranslateMotion(AAcc[ParentIdx], R) + CAcc[Idx];

				const int32 DOFIdx = Batch.DOFIndex(Topo.BodyDOFOffset[Body], Env);

				if (bIsBall)
				{
					const FMat3 Irot = IAcc[Idx].Irot;
					const FMat3 H = IAcc[Idx].H;
					// MUST match Pass 2's Dinv exactly — the reduced inertia handed to
					// the parent and the acceleration recovered here are two halves of
					// one solve. Same scaled-inverse form, for the same bit-identity
					// reason (see FJointPassiveFactors).
					const float ArmScale = GetJointPassiveFactors(Topo, Body, Dt).ArmatureScale;
					const FMat3 Dinv = (ArmScale == 1.0f)
						? Inverse3x3(Irot)
						: Inverse3x3(Irot) * (1.0f / ArmScale);
					// U^T a_parent uses the ARTICULATED Irot/H, not D: armature is
					// inertia of the joint itself, not of its coupling to the parent.
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
					// RelRot is defined by BodyRot = ParentRot * RestRot * RelRot, so
					// the frame it lives in is (ParentRot * RestRot) — NOT ParentRot
					// alone. Requiring BodyRot' = exp(w_i dt) * BodyRot and expanding
					// with w_i = w_p + w_J gives
					//     RelRot' = exp( (ParentRot*RestRot)^-1 w_J dt ) * RelRot
					// Conjugating by ParentRot only (what this used to do) rotates the
					// joint velocity into the wrong frame whenever RestRot is not
					// identity — which it never is on a real rig. Measured: every rig
					// containing a ball joint failed momentum conservation by 0.2-20%,
					// dt-invariantly, while otherwise-identical all-revolute rigs were
					// clean to 1e-6. See SOLVER_DEBUG_LOG.md entry 012.
					const FQuat ParentRot = Batch.GetBodyRot(Parent, Env);
					const FQuat JointFrame = ParentRot * Topo.BodyRestRotInParent[Body];
					const FVector RelAngVelParentFrame = JointFrame.UnrotateVector(JointAngVelWorld);
					const FVector DTheta = RelAngVelParentFrame * Dt;
					const float Angle = static_cast<float>(DTheta.Size());
					FQuat RelRot = Batch.GetJointRelRot(Body, Env);
					// SMALL_NUMBER (1e-8), not KINDA_SMALL_NUMBER (1e-4). This guard only
			// exists to avoid normalizing a zero axis; using the coarse tolerance
			// silently DISCARDED every rotation smaller than 1e-4 rad per step.
			// At w = 0.3 rad/s that threshold is crossed at ~3000 Hz, so at any
			// fine substep rate the orientation simply froze while the angular
			// velocity kept integrating — the finer the timestep, the more
			// rotation was thrown away, which is why momentum drift got WORSE as
			// dt shrank instead of converging. Measured: a closed 2-body chain
			// drifted 6e-7 at 1000 Hz but 7.6e-4 at 4000 Hz purely from this.
			// See SOLVER_DEBUG_LOG.md entry 012.
			if (Angle > SMALL_NUMBER)
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

		ClampJointLimits(Batch);

		// ALWAYS refresh, not just after a clamp. Pass 1 computed every non-root
		// body's BodyPos/BodyRot/velocity from the PRE-step root transform and
		// PRE-step joint angles; Pass 3a then moved the root and Pass 3b
		// integrated the joints. So on exit from Step(), every non-root body is
		// one full step STALE — and that is precisely the state external callers
		// sample.
		//
		// It matters most for ground contact, which reads BodyPos/BodyRot to
		// place its contact points and BodyLinVel/BodyAngVel to evaluate its
		// damper. Feeding a penalty force one-step-old positions and velocities
		// makes it act with a phase lag, and a lagged damper is NEGATIVE damping
		// — it pumps energy in rather than removing it. That is consistent with
		// the measured contact behaviour: bounded but growing energy (E/E0 ~ 6.5
		// on the 2-leg rig) with no corresponding external work.
		//
		// Cost is one extra forward-kinematics pass per step, which is cheap
		// next to Pass 2/3's reduction, and it makes the state Step() leaves
		// behind self-consistent for ANY caller rather than only for the
		// solver's own next iteration.
		RecomputeKinematics(Batch);

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
		CAcc8.SetNum(BodySlots, EAllowShrinking::No);
		// Body 0's slice is never written by Pass 1 (which starts at Body 1) but
		// IS read by Pass 2/3b, so zero it explicitly — SetNum alone would leave
		// whatever the previous Step() left behind. Only body 0 needs this;
		// every other body overwrites its own slice in Pass 1.
		{
			const FSpatialVecx8 Zero8{ FVec3x8::Zero(), FVec3x8::Zero() };
			for (int32 Env = 0; Env < PaddedNumEnvs; Env += SIMDWidth)
			{
				CAcc8.Store(Batch.BodyIndex(0, Env), Zero8);
			}
		}
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

				// Velocity-product bias — mirrors StepScalar; see CAcc's derivation.
				CAcc8.Store(Idx, FSpatialVecx8{
					bIsBall ? FVec3x8::Zero() : FVec3x8::Cross(ParentAngVel, JointAngVelWorld),
					FVec3x8::Cross(LinVel, JointAngVelWorld)
				});
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
				const __m256 GravityScale8 = _mm256_set1_ps(Topo.BodyGravityScale[Body]);
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

				const FVec3x8 GravityForceWorld = Gravity8 * _mm256_mul_ps(Mass, GravityScale8);
				const FSpatialVecx8 GravityWrench{ FVec3x8::Cross(CoMOffset, GravityForceWorld), GravityForceWorld };

				// Gyroscopic (velocity-product) term minus the applied gravity wrench.
				IAcc8.Store(Idx, Own);
				PAcc8.Store(Idx, SpatialCrossForce(V, Own.Apply(V)) - GravityWrench);
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
				// Armature/damping — identical derivation to StepScalar's Pass 2, and
				// it has to stay identical or AgentSolver.CreatureBatchSolverSIMD's
				// scalar-vs-SIMD parity check fails. See FJointPassiveFactors.
				const FJointPassiveFactors Passive = GetJointPassiveFactors(Topo, Body, Dt);

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
						const FMat3 Dinv = (Passive.ArmatureScale == 1.0f)
							? Inverse3x3(Irot)
							: Inverse3x3(Irot) * (1.0f / Passive.ArmatureScale);

						const FVector Tau3(
							Batch.JointTorque[DOFIdx] * StrengthScale * ActiveMask * MuscleMultiplierAcc[DOFIdx],
							Batch.JointTorque[DOFIdx + PaddedNumEnvs] * StrengthScale * ActiveMask * MuscleMultiplierAcc[DOFIdx + PaddedNumEnvs],
							Batch.JointTorque[DOFIdx + 2 * PaddedNumEnvs] * StrengthScale * ActiveMask * MuscleMultiplierAcc[DOFIdx + 2 * PaddedNumEnvs]);
						const FVector U_u3 = Tau3 - PBody.Ang;
						BallU_u3Acc[Idx] = U_u3;

						FSpatialInertia Reduced;
						Reduced.Irot = Irot * Passive.ArmatureFrac;
						Reduced.H = H * Passive.ArmatureFrac;
						Reduced.MBlock = IBody.MBlock - H.Transpose() * Dinv * H;

						// + I^a c — see StepScalar's mirror of this.
						const FSpatialVec ReducedBias = FSpatialVec{ Tau3 - U_u3 * Passive.ArmatureFrac,
							PBody.Lin + H.Transpose() * (Dinv * U_u3) }
							+ Reduced.Apply(CAcc8.LoadScalar(Idx));

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
				// Armature: D * (1+r). Mirrors StepScalar exactly, including applying
				// the scale AFTER the KINDA_SMALL_NUMBER floor rather than before.
				__m256 D = _mm256_max_ps(FSpatialVecx8::Dot(S, U), _mm256_set1_ps(KINDA_SMALL_NUMBER));
				if (Passive.ArmatureScale != 1.0f)
				{
					D = _mm256_mul_ps(D, _mm256_set1_ps(Passive.ArmatureScale));
				}

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
				// Damping is NOT applied here — see ApplyJointDamping.
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

				// + I^a c — see StepScalar's mirror of this.
				const FSpatialVecx8 ReducedBias = PBody + Reduced.Apply(CAcc8.Load(Idx)) + U * _mm256_mul_ps(U_u, InvD);

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
			// SPATIAL -> CLASSICAL. The equation assembled above is Featherstone's
			// f = I a + v x* (I v), in which `a` is the SPATIAL acceleration: its
			// linear part is the rate of change of the velocity field sampled at a
			// point FIXED in space, not the acceleration of the body's own moving
			// material reference point. The two differ by w x v:
			//     d(u_material)/dt = A.Lin + w x u
			// Integrating A.Lin directly (what this used to do) therefore injects a
			// spurious -w x u drift, perpendicular to the velocity, which curves a
			// straight free-flight path and grows with w.
			//
			// Sanity check on the simplest case: a free body with its reference AT
			// the CoM and no external force gives A.Lin = -w x u exactly, so the
			// corrected update yields du/dt = 0 — a constant CoM velocity, i.e.
			// Newton's first law. Before this fix that same case drifted from
			// (10,0,0) to (6.3,9.1,-5.7) in 5 s, and the drift did NOT shrink with
			// a finer timestep (see SOLVER_DEBUG_LOG.md entry 005), which is the
			// signature of a modelling error rather than an integration error.
			//
			// Uses the PRE-update angular velocity deliberately: both terms are
			// evaluated at the start of the step, matching the explicit ordering
			// the rest of this pass already uses.
			RootLinVel += (A0.Lin + FVector::CrossProduct(RootAngVel, RootLinVel)) * Dt;
			RootAngVel += A0.Ang * Dt;
			Batch.LinVelX[RootIdx] = (float)RootLinVel.X; Batch.LinVelY[RootIdx] = (float)RootLinVel.Y; Batch.LinVelZ[RootIdx] = (float)RootLinVel.Z;
			Batch.AngVelX[RootIdx] = (float)RootAngVel.X; Batch.AngVelY[RootIdx] = (float)RootAngVel.Y; Batch.AngVelZ[RootIdx] = (float)RootAngVel.Z;

			const FVector RootPos = Batch.GetBodyPos(0, Env) + RootLinVel * Dt;
			FQuat RootRot = Batch.GetBodyRot(0, Env);
			const FVector DTheta = RootAngVel * Dt;
			const float Angle = static_cast<float>(DTheta.Size());
			// SMALL_NUMBER (1e-8), not KINDA_SMALL_NUMBER (1e-4). This guard only
			// exists to avoid normalizing a zero axis; using the coarse tolerance
			// silently DISCARDED every rotation smaller than 1e-4 rad per step.
			// At w = 0.3 rad/s that threshold is crossed at ~3000 Hz, so at any
			// fine substep rate the orientation simply froze while the angular
			// velocity kept integrating — the finer the timestep, the more
			// rotation was thrown away, which is why momentum drift got WORSE as
			// dt shrank instead of converging. Measured: a closed 2-body chain
			// drifted 6e-7 at 1000 Hz but 7.6e-4 at 4000 Hz purely from this.
			// See SOLVER_DEBUG_LOG.md entry 012.
			if (Angle > SMALL_NUMBER)
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
					// TranslateMotion gives only a_p + alpha_p x R; CAcc supplies the
					// velocity-product remainder (see its derivation).
					const FSpatialVec ParentAccAtBody = TranslateMotion(AAcc8.LoadScalar(ParentIdx), R) + CAcc8.LoadScalar(Idx);

					const FSpatialInertia IBody = IAcc8.LoadScalar(Idx);
					// MUST match this pass's own Pass 2 Dinv — see StepScalar's mirror.
					const float ArmScale = GetJointPassiveFactors(Topo, Body, Dt).ArmatureScale;
					const FMat3 Dinv = (ArmScale == 1.0f)
						? Inverse3x3(IBody.Irot)
						: Inverse3x3(IBody.Irot) * (1.0f / ArmScale);
					const FVector RHS = BallU_u3Acc[Idx] - (IBody.Irot * ParentAccAtBody.Ang + IBody.H * ParentAccAtBody.Lin);
					const FVector QDDot3 = Dinv * RHS;

					AAcc8.StoreScalar(Idx, ParentAccAtBody + FSpatialVec{ QDDot3, FVector::ZeroVector });

					FVector JointAngVelWorld(Batch.JointVel[DOFIdx], Batch.JointVel[DOFIdx + PaddedNumEnvs], Batch.JointVel[DOFIdx + 2 * PaddedNumEnvs]);
					JointAngVelWorld += QDDot3 * Dt;
					Batch.JointVel[DOFIdx] = (float)JointAngVelWorld.X;
					Batch.JointVel[DOFIdx + PaddedNumEnvs] = (float)JointAngVelWorld.Y;
					Batch.JointVel[DOFIdx + 2 * PaddedNumEnvs] = (float)JointAngVelWorld.Z;

					// RelRot is defined by BodyRot = ParentRot * RestRot * RelRot, so
					// the frame it lives in is (ParentRot * RestRot) — NOT ParentRot
					// alone. Requiring BodyRot' = exp(w_i dt) * BodyRot and expanding
					// with w_i = w_p + w_J gives
					//     RelRot' = exp( (ParentRot*RestRot)^-1 w_J dt ) * RelRot
					// Conjugating by ParentRot only (what this used to do) rotates the
					// joint velocity into the wrong frame whenever RestRot is not
					// identity — which it never is on a real rig. Measured: every rig
					// containing a ball joint failed momentum conservation by 0.2-20%,
					// dt-invariantly, while otherwise-identical all-revolute rigs were
					// clean to 1e-6. See SOLVER_DEBUG_LOG.md entry 012.
					const FQuat ParentRot = Batch.GetBodyRot(Parent, Env);
					const FQuat JointFrame = ParentRot * Topo.BodyRestRotInParent[Body];
					const FVector RelAngVelParentFrame = JointFrame.UnrotateVector(JointAngVelWorld);
					const FVector DTheta = RelAngVelParentFrame * Dt;
					const float Angle = static_cast<float>(DTheta.Size());
					FQuat RelRot = Batch.GetJointRelRot(Body, Env);
					// SMALL_NUMBER (1e-8), not KINDA_SMALL_NUMBER (1e-4). This guard only
			// exists to avoid normalizing a zero axis; using the coarse tolerance
			// silently DISCARDED every rotation smaller than 1e-4 rad per step.
			// At w = 0.3 rad/s that threshold is crossed at ~3000 Hz, so at any
			// fine substep rate the orientation simply froze while the angular
			// velocity kept integrating — the finer the timestep, the more
			// rotation was thrown away, which is why momentum drift got WORSE as
			// dt shrank instead of converging. Measured: a closed 2-body chain
			// drifted 6e-7 at 1000 Hz but 7.6e-4 at 4000 Hz purely from this.
			// See SOLVER_DEBUG_LOG.md entry 012.
			if (Angle > SMALL_NUMBER)
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
				// TranslateMotion gives only a_p + alpha_p x R; CAcc supplies the
				// velocity-product remainder (see its derivation).
				const FSpatialVecx8 ParentAccAtBody = TranslateMotion(AAcc8.Load(ParentIdx), R) + CAcc8.Load(Idx);

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

		ClampJointLimits(Batch);

		// ALWAYS refresh, not just after a clamp. Pass 1 computed every non-root
		// body's BodyPos/BodyRot/velocity from the PRE-step root transform and
		// PRE-step joint angles; Pass 3a then moved the root and Pass 3b
		// integrated the joints. So on exit from Step(), every non-root body is
		// one full step STALE — and that is precisely the state external callers
		// sample.
		//
		// It matters most for ground contact, which reads BodyPos/BodyRot to
		// place its contact points and BodyLinVel/BodyAngVel to evaluate its
		// damper. Feeding a penalty force one-step-old positions and velocities
		// makes it act with a phase lag, and a lagged damper is NEGATIVE damping
		// — it pumps energy in rather than removing it. That is consistent with
		// the measured contact behaviour: bounded but growing energy (E/E0 ~ 6.5
		// on the 2-leg rig) with no corresponding external work.
		//
		// Cost is one extra forward-kinematics pass per step, which is cheap
		// next to Pass 2/3's reduction, and it makes the state Step() leaves
		// behind self-consistent for ANY caller rather than only for the
		// solver's own next iteration.
		RecomputeKinematics(Batch);

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

	/**
	 * Velocity-product ("Coriolis"/centripetal) bias C per body, computed in
	 * Pass 1 and consumed by Pass 2 and Pass 3b. Featherstone's c_i.
	 *
	 * DERIVATION (world-aligned axes; each body's reference point is its own
	 * joint origin; R = pos(body) - pos(parent), fixed in the PARENT's frame so
	 * dR/dt = w_p x R; u denotes a body's stored LinVel).
	 *
	 * CRITICAL: this solver's acceleration is SPATIAL, not classical — Pass 3a
	 * establishes du/dt = A.Lin + w x u. The transport bias must therefore be
	 * derived in spatial terms. Doing it in classical terms instead yields
	 * w_p x (w_p x R), which is WRONG and measurably destabilizing: that term
	 * cancels identically in the derivation below. (Tried, measured, reverted —
	 * SOLVER_DEBUG_LOG.md entry 005.)
	 *
	 * Exact kinematics:
	 *   u_i = u_p + w_p x R          w_i = w_p + w_J
	 *   du_i/dt = du_p/dt + alpha_p x R + w_p x (w_p x R)
	 *
	 * Convert both sides to spatial (A.Lin = du/dt - w x u) and substitute
	 * u_p = u_i - w_p x R. The w_p x (w_p x R) terms cancel, leaving
	 *
	 *   A_i.Lin - TranslateMotion(A_p,R).Lin = (w_p - w_i) x u_i = u_i x w_J
	 *
	 * so the LINEAR part is, for BOTH joint types:
	 *   C.Lin = u_i x w_J
	 *
	 * This matches Featherstone's c = v x (S qdot) — its linear component with
	 * S qdot = (w_J, 0) is exactly u_i x w_J.
	 *
	 *   alpha_i = alpha_p + d(w_J)/dt
	 *
	 * The ANGULAR part depends on how w_J is parameterized, and the two joint
	 * types differ — getting this wrong is easy, so it is spelled out:
	 *
	 *  - REVOLUTE: w_J = axisWorld * qdot, and axisWorld is carried by the
	 *    parent, so d(axisWorld)/dt = w_p x axisWorld. Hence
	 *      d(w_J)/dt = w_p x w_J + axisWorld * qddot
	 *    and the first term is bias:
	 *      C.Ang = w_p x w_J
	 *    (equivalently w_i x w_J, since w_J x w_J = 0 — so this agrees with
	 *    Featherstone's v x (S qdot) angular component too.)
	 *
	 *  - BALL: JointVel stores w_J directly in WORLD components, and Pass 3b
	 *    integrates JointVel += QDDot3 * Dt, i.e. QDDot3 IS d(w_J)/dt in world
	 *    already. The motion subspace S = [I3; 0] is constant in that frame, so
	 *    there is no Sdot term and no axis-carrying term:
	 *      C.Ang = 0
	 *
	 * Body 0 (floating base) has no parent and keeps C = 0.
	 */
	TArray<FSpatialVec> CAcc;

	// StepSIMD's scratch (SoA, see SpatialAlgebraSIMD.h). DAcc/UScalarAcc
	// are already flat float arrays so StepSIMD reuses the same members
	// (resized identically either way) rather than duplicating them.
	FSpatialInertiaSoA IAcc8;
	FSpatialVecSoA PAcc8;
	FSpatialVecSoA AAcc8;
	FSpatialVecSoA UAcc8;
	FVec3SoA WorldAxisAcc8;
	FSpatialVecSoA CAcc8; // StepSIMD's velocity-product bias — see CAcc's derivation

	// Ball joints' generalized bias (u = tau - P.Ang), cached between Pass 2
	// and Pass 3b. Shared by StepScalar and StepSIMD since ball-joint math
	// is lane-scalar in both (see class comment) — same reasoning as
	// DAcc/UScalarAcc above.
	TArray<FVector> BallU_u3Acc;

	// ---- Impulse-dynamics scratch (see the velocity-level contact section) ----
	// ImpI is body-major/env-minor, sized once per ComputeArticulatedInertias
	// call and safe to read concurrently across envs (each env only ever
	// reads/writes its own slice, and it is never resized during a parallel
	// section) -- it does NOT need thread-local treatment.
	//
	// ImpP/ImpU/ImpUVec/DVScratch/DQScratch/JointImpScratch are different: true
	// per-call SCRATCH that SolveImpulseResponse (and its wrapper functions
	// below -- ImpulseResponseAtPoint, JointImpulseResponse, ApplyJointImpulse,
	// BallJointImpulseResponse, ApplyBallJointImpulse, and ApplyJointDamping's
	// own SolveScaleAndApply) overwrite on every call. Until 2026-08-25 these
	// were plain instance members, sized for exactly ONE env and reused
	// sequentially across a caller's own `for (Env)` loop -- which is
	// EXACTLY what forced both ApplyJointDamping's and (per this class's own
	// comment on the global contact solve, CreatureGroundContact.h) the
	// ground-contact solver's per-env loops to stay single-threaded: two envs
	// running concurrently would both scribble into the same arrays.
	// `static thread_local` (not per-instance batch-sizing like ImpI's) is the
	// fix -- each worker thread gets its own private copy, so two envs
	// processed concurrently on different threads never collide, while a
	// single thread's own sequential calls (across multiple envs, or across
	// multiple SEPARATE FCreatureABASolver instances, e.g. the Live driver's
	// vs. a Ragdoll/Visualizer actor's own -- all sharing the same worker
	// threads) keep reusing the same already-correctly-sized buffer exactly
	// as before, since every call already does its own `.SetNum()` before use.
	// `static` means these can no longer be `mutable` (mutable only applies to
	// non-static data members) -- unnecessary anyway, since a const member
	// function can always touch a class's static state.
	TArray<FSpatialInertia> ImpI;
	static inline thread_local TArray<FSpatialVec> ImpP;    // impulse bias per body
	static inline thread_local TArray<FSpatialVec> ImpU;    // ball: u3 in .Ang | revolute: (Uu, D, 0) in .Ang
	static inline thread_local TArray<FSpatialVec> ImpUVec; // revolute: the full U spatial vector
	static inline thread_local TArray<FSpatialVec> DVScratch;
	static inline thread_local TArray<float> DQScratch;
	// Per-DOF joint-space impulse input for SolveImpulseResponse. Kept as a
	// persistent (now thread-local) buffer rather than a local so the
	// joint-limit rows, which call this once per DOF per iteration, don't
	// allocate in the solve loop.
	static inline thread_local TArray<float> JointImpScratch;
	// Lock set of the CURRENT factorization, set by ComputeArticulatedInertias
	// and read by SolveImpulseResponse so the two agree about which joints are
	// welded. Non-owning; valid only until the next ComputeArticulatedInertias.
	//
	// ImpLockedStride == 0 means one byte per BODY, shared by every env (the
	// original signature, and correct only when NumEnvs == 1 -- which is all the
	// diagnostics that predate this ever used). A positive stride means
	// body-major/env-minor, indexed [Body * stride + Env], which is what a real
	// batched run needs: whether a joint is saturated is per-env state, so a
	// single per-body array would weld env 0's saturated joints in all 256 envs.
	mutable const uint8* ImpLockedBody = nullptr;
	mutable int32 ImpLockedStride = 0;

	FORCEINLINE bool IsJointLocked(int32 Body, int32 Env) const
	{
		return ImpLockedBody && ImpLockedBody[Body * ImpLockedStride + Env] != 0;
	}

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
	 * Topo.DOFExtensionCurve/DOFFlexionCurve scaled by the matching
	 * Topo.DOFExtensionStrength/DOFFlexionStrength, evaluated at the DOF's current
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
			// Scalar strength x curve shape -- see FCreatureTopology::
			// DOFExtensionStrength. Negative authored values are clamped away here
			// rather than at load: a negative multiplier would invert the sign of
			// the commanded torque, i.e. a muscle that pulls the way it was told
			// NOT to, which is never what the author meant by "strength".
			const float ExtStrength = FMath::Max(Topo.DOFExtensionStrength[DOF], 0.0f);
			const float FlexStrength = FMath::Max(Topo.DOFFlexionStrength[DOF], 0.0f);
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
					? ExtStrength * (ExtCurve ? ExtCurve->Eval(T) : 1.0f)
					: FlexStrength * (FlexCurve ? FlexCurve->Eval(T) : 1.0f);
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

			// BUG FOUND 2026-08-17: this used to be `(AngleDeg - Wrapped) +
			// ClampedWrapped`, i.e. AngleDeg + (ClampedWrapped - Wrapped). That's
			// fine when Wrapped and ClampedWrapped are on the same side of the
			// wrap, but when a joint settles right at MinDeg and dips a hair
			// BELOW it each substep (completely normal once resting at a stop),
			// Wrapped computes to ~359.94 (correct: Fmod of a tiny negative
			// residual wraps almost all the way around) and the nearest-boundary
			// snap correctly picks ClampedWrapped=0 -- but 0 - 359.94 = -359.94,
			// a SPURIOUS extra full lap instead of the intended ~0.06 correction.
			// Confirmed via a throwaway diagnostic on the real rig's Knee2_L:
			// JointPos jumped by EXACTLY -360.00 deg every single substep once
			// settled at its lower stop (Solver.JointImpulseResponse stayed
			// perfectly smooth throughout -- not a numerical-conditioning issue,
			// purely this arithmetic bug), eventually destabilizing the whole
			// tree. Fix: always take the SHORTEST-path correction relative to
			// AngleDeg, never a spurious whole lap -- mathematically identical
			// to the old formula whenever |Delta|<=180 (the normal, legitimate
			// clamp case), and only differs where before it was actually wrong.
			float Delta = ClampedWrapped - Wrapped;
			if (Delta > 180.0f) Delta -= 360.0f;
			else if (Delta < -180.0f) Delta += 360.0f;
			return AngleDeg + Delta;
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
	 *
	 * Public so diagnostics can exercise forward kinematics in isolation, with
	 * no integration, no timestep and no joint-limit clamping in the way (see
	 * MutoJointAxisAuditTest.cpp). It only recomputes BodyPos/BodyRot from the
	 * current JointPos/root pose, so calling it is always safe.
	 */
public:
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
	// ================= VELOCITY-LEVEL IMPULSE DYNAMICS =================
	//
	// The impulse equations are the SAME recursion as the force pass with
	// acceleration -> velocity change and force -> impulse, and with every bias
	// term dropped (an impulse acts over zero time, so there is no gyroscopic
	// term and no velocity-product term). That is why this mirrors Pass 2/3
	// structurally but is much shorter.
	//
	// Why this exists: penalty contact computes a FORCE from penetration and
	// integrates it explicitly, which is only conditionally stable and, past the
	// limit, returns more energy than it absorbs. A velocity-level contact
	// instead asks "what impulse makes the contact point stop approaching?" and
	// applies exactly that — unconditionally stable, with no stiffness to tune
	// and no dependence on the timestep. See SOLVER_DEBUG_LOG.md entry 014.
	//
	// These run per-env and scalar. Contacts are a small fraction of bodies and
	// correctness matters more than throughput here; the batched force path is
	// untouched.

	/**
	 * A revolute joint's world axis, recomputed from the parent's current
	 * rotation exactly as Pass 1 defines it.
	 *
	 * Deliberately NOT read from WorldAxisAcc: that array is filled only by
	 * StepScalar, while Step() dispatches to StepSIMD (which fills the SoA
	 * WorldAxisAcc8 instead), so reading it here indexed out of an empty array
	 * whenever the normal Step() path was used. Recomputing costs one quaternion
	 * rotate and makes these entry points independent of which Step variant ran.
	 */
	static FORCEINLINE FVector RevoluteAxisWorld(const FCreatureBatchState& Batch,
	                                             const FCreatureTopology& Topo, int32 Body, int32 Env)
	{
		return Batch.GetBodyRot(Topo.BodyParent[Body], Env).RotateVector(Topo.BodyJointAxisLocal[Body]);
	}

	/**
	 * Recompute articulated inertias for the CURRENT configuration, with no bias
	 * terms. Must be called after the configuration changes and before any
	 * impulse query below. Kept separate from Step() so a caller can resolve
	 * several contacts against one factorization.
	 */
	/**
	 * `LockedBody`, if given, is one byte per body: non-zero means treat that
	 * body's joint as WELDED for this factorization -- skip the `U U^T / D`
	 * reduction and hand the child's full articulated inertia to the parent.
	 *
	 * Why this exists: the articulated effective mass at a contact is computed
	 * as though every joint above it can rotate freely, which is correct for an
	 * instantaneous impulse on a free chain and badly wrong for a leg folded
	 * onto its stops. Measured on the real rig: 14 kg at a foot under a 6170 kg
	 * creature, rising to only 17 kg once six leg joints were saturated
	 * (SOLVER_DEBUG_LOG.md entry 023). Locking the saturated joints here makes
	 * the same query return the rigid-strut answer instead.
	 *
	 * It is an APPROXIMATION and knowingly so: a joint at a limit is
	 * unilaterally constrained (it can still leave the stop in one direction),
	 * and welding it removes that direction too. Whether that trade is worth it
	 * is a measurement, not an opinion -- see entry 024.
	 */
	void ComputeArticulatedInertias(const FCreatureBatchState& Batch, const uint8* LockedBody = nullptr, int32 LockedStride = 0)
	{
		// Remembered for SolveImpulseResponse: the lock set is a property of
		// THIS factorization, and the recursion that reads ImpI has to skip the
		// same joints or the two disagree. (They did, at first: locking only
		// here changed nothing at all, because SolveImpulseResponse redoes the
		// reduction itself -- a no-op that reported a clean 1.0x ratio.)
		ImpLockedBody = LockedBody;
		ImpLockedStride = LockedStride;
		const FCreatureTopology& Topo = Batch.GetTopology();
		const int32 NumBodies = Topo.NumBodies;
		const int32 NumEnvs = Batch.GetNumEnvs();
		const int32 BodySlots = NumBodies * Batch.GetPaddedNumEnvs();
		ImpI.SetNum(BodySlots, EAllowShrinking::No);

		ParallelFor(NumEnvs, [&](int32 Env)
		{
			// Own rigid-body inertia about each body's own reference point.
			for (int32 Body = 0; Body < NumBodies; ++Body)
			{
				const int32 Idx = Batch.BodyIndex(Body, Env);
				const FQuat Rot = Batch.GetBodyRot(Body, Env);
				const FVector CoMOffset = Rot.RotateVector(Topo.BodyLocalCoMOffset[Body]);
				float Mass = Topo.BodyMass[Body];
				if (Body == 0) Mass += Batch.CarriedMass[Env];

				const FMat3 RotM = FMat3::FromRotation(Rot);
				const FMat3 IrotAboutCoM = RotM * FMat3::Diagonal(Topo.BodyInertiaDiagLocal[Body]) * RotM.Transpose();
				const FMat3 Rx = FMat3::Skew(CoMOffset);
				ImpI[Idx] = FSpatialInertia::FromRigidBody(Mass, IrotAboutCoM - (Rx * Rx) * Mass, Rx * Mass);
			}

			// Reduce tip -> base. Identical to Pass 2's inertia half; the bias
			// half is impulse-specific and lives in SolveImpulseResponse.
			for (int32 Body = NumBodies - 1; Body >= 1; --Body)
			{
				const int32 Parent = Topo.BodyParent[Body];
				const int32 Idx = Batch.BodyIndex(Body, Env);
				const int32 ParentIdx = Batch.BodyIndex(Parent, Env);

				// ARMATURE, and only armature: an impulse acts over zero time, so
				// every bias term is dropped (see the section comment) and joint
				// DAMPING — a force — has no place here. Armature is inertia, so it
				// must appear identically in both passes or the contact solver sizes
				// its impulses against a different creature than the one being
				// integrated. Dt is irrelevant to the armature half, hence the 0.
				const FJointPassiveFactors Passive = GetJointPassiveFactors(Topo, Body, 0.0f);

				FSpatialInertia Reduced;
				if (LockedBody && LockedBody[Body * LockedStride + Env])
				{
					// Welded: the joint contributes no compliance, so the child's
					// whole articulated inertia passes straight to the parent.
					// Armature is irrelevant here by construction — a welded joint has
					// no D to add it to.
					Reduced = ImpI[Idx];
				}
				else if (Topo.BodyDOFCount[Body] == 3)
				{
					const FMat3 H = ImpI[Idx].H;
					const FMat3 Irot = ImpI[Idx].Irot;
					const FMat3 Dinv = (Passive.ArmatureScale == 1.0f)
						? Inverse3x3(Irot)
						: Inverse3x3(Irot) * (1.0f / Passive.ArmatureScale);
					Reduced.Irot = Irot * Passive.ArmatureFrac;
					Reduced.H = H * Passive.ArmatureFrac;
					Reduced.MBlock = ImpI[Idx].MBlock - H.Transpose() * Dinv * H;
				}
				else
				{
					const FSpatialVec S{ RevoluteAxisWorld(Batch, Topo, Body, Env), FVector::ZeroVector };
					const FSpatialVec U = ImpI[Idx].Apply(S);
					const float D = FMath::Max(FSpatialVec::Dot(S, U), KINDA_SMALL_NUMBER) * Passive.ArmatureScale;
					const float InvD = 1.0f / D;
					Reduced.Irot = ImpI[Idx].Irot - FMat3::Outer(U.Ang, U.Ang) * InvD;
					Reduced.H = ImpI[Idx].H - FMat3::Outer(U.Ang, U.Lin) * InvD;
					Reduced.MBlock = ImpI[Idx].MBlock - FMat3::Outer(U.Lin, U.Lin) * InvD;
				}

				const FVector R = Batch.GetBodyPos(Body, Env) - Batch.GetBodyPos(Parent, Env);
				ImpI[ParentIdx] = ImpI[ParentIdx] + Reduced.TranslatedTo(R);
			}
		});
	}

	/**
	 * Velocity change throughout one env's tree produced by a single spatial
	 * impulse on one body (about that body's own reference point).
	 * OutDeltaV is indexed by body, OutDeltaQd by DOF — both sized by the caller.
	 * Requires ComputeArticulatedInertias() for the current configuration.
	 *
	 * WorldAxisAcc must hold the current revolute axes; Step() fills it, and
	 * ComputeArticulatedInertias reads the same values, so calling this straight
	 * after a Step() is valid.
	 */
	void SolveImpulseResponse(const FCreatureBatchState& Batch, int32 Env,
	                          int32 ImpulseBody, const FSpatialVec& Impulse,
	                          TArray<FSpatialVec>& OutDeltaV, TArray<float>& OutDeltaQd,
	                          const float* JointImpulsePerDOF = nullptr) const
	{
		const FCreatureTopology& Topo = Batch.GetTopology();
		const int32 NumBodies = Topo.NumBodies;

		// Impulse bias, mirroring PAcc but with the applied impulse negated and
		// no gyroscopic/velocity-product terms.
		ImpP.SetNum(NumBodies, EAllowShrinking::No);
		ImpU.SetNum(NumBodies, EAllowShrinking::No);
		for (int32 b = 0; b < NumBodies; ++b) ImpP[b] = FSpatialVec{ FVector::ZeroVector, FVector::ZeroVector };
		// ImpulseBody == INDEX_NONE means a pure JOINT-space impulse with no
		// wrench on any body -- which is what a joint-limit constraint applies.
		if (ImpulseBody != INDEX_NONE)
		{
			ImpP[ImpulseBody] = FSpatialVec{ Impulse.Ang * -1.0f, Impulse.Lin * -1.0f };
		}

		// ---- backward: reduce impulse bias to the root ----
		for (int32 Body = NumBodies - 1; Body >= 1; --Body)
		{
			const int32 Parent = Topo.BodyParent[Body];
			const int32 Idx = Batch.BodyIndex(Body, Env);
			FSpatialVec ReducedBias;

			const int32 BiasDOFOffset = Topo.BodyDOFOffset[Body];
			// Armature only — must match ComputeArticulatedInertias exactly.
			const FJointPassiveFactors Passive = GetJointPassiveFactors(Topo, Body, 0.0f);

			if (IsJointLocked(Body, Env))
			{
				// Welded for this factorization: no joint DOF to reduce out, so
				// the bias passes through untouched. Must match the identical
				// branch in ComputeArticulatedInertias.
				ImpU[Body] = FSpatialVec{ FVector::ZeroVector, FVector::ZeroVector };
				ReducedBias = ImpP[Body];
			}
			else if (Topo.BodyDOFCount[Body] == 3)
			{
				const FMat3 H = ImpI[Idx].H;
				const FMat3 Irot = ImpI[Idx].Irot;
				const FMat3 Dinv = (Passive.ArmatureScale == 1.0f)
					? Inverse3x3(Irot)
					: Inverse3x3(Irot) * (1.0f / Passive.ArmatureScale);
				// Joint-space impulse enters exactly where the force pass's tau
				// does: added to u before the reduction. Zero recovers the
				// original body-impulse-only behaviour.
				const FVector JointImp3 = JointImpulsePerDOF
					? FVector(JointImpulsePerDOF[BiasDOFOffset + 0], JointImpulsePerDOF[BiasDOFOffset + 1], JointImpulsePerDOF[BiasDOFOffset + 2])
					: FVector::ZeroVector;
				const FVector U3 = JointImp3 - ImpP[Body].Ang;
				ImpU[Body] = FSpatialVec{ U3, FVector::ZeroVector };
				// BUG FOUND 2026-08-17 (while adding armature): the angular part of the
				// reduced bias used to be hardcoded to ZeroVector. Featherstone's
				// p^a = p^A + U D^-1 u with U = [Irot; H^T] gives
				//     p^a.Ang = ImpP.Ang + Irot*D^-1*u
				// which, with no armature (Irot*D^-1 == identity), collapses to
				// ImpP.Ang + u = JointImp3 -- NOT zero. Zero is correct only when
				// JointImp3 == 0, i.e. only for the body-impulse-only case this
				// function originally supported. When ball-joint cone limit rows were
				// added (2026-08-16) they began passing a non-zero JointImpulsePerDOF
				// at ball joints through this exact branch, and the angular reaction
				// that a hip or spine limit should transmit to its parent was silently
				// discarded. That defeats the whole stated purpose of the joint-space
				// query -- "whole-tree, not local: pushing on one joint moves every
				// body ... that is the entire reason a joint limit solved this way can
				// brace a chain" (see JointImpulseResponse) -- so hip and spine cone
				// limits were braced against nothing but their own child subtree.
				// The revolute branch below was always correct (it adds the full
				// spatial U * (Uu/D)); only the ball branch was wrong.
				// Written as JointImp3 - A*D^-1*u so it is exactly JointImp3 at zero
				// armature rather than the cancellation-prone Irot*D^-1*u.
				ReducedBias = FSpatialVec{ JointImp3 - U3 * Passive.ArmatureFrac,
					ImpP[Body].Lin + H.Transpose() * (Dinv * U3) };
			}
			else
			{
				const FSpatialVec S{ RevoluteAxisWorld(Batch, Topo, Body, Env), FVector::ZeroVector };
				const FSpatialVec U = ImpI[Idx].Apply(S);
				const float D = FMath::Max(FSpatialVec::Dot(S, U), KINDA_SMALL_NUMBER) * Passive.ArmatureScale;
				const float JointImp1 = JointImpulsePerDOF ? JointImpulsePerDOF[BiasDOFOffset] : 0.0f;
				const float Uu = JointImp1 - FSpatialVec::Dot(S, ImpP[Body]);
				ImpU[Body] = FSpatialVec{ FVector(Uu, D, 0.0f), U.Ang }; // stash Uu/D + U for the forward pass
				ImpUVec.SetNum(NumBodies, EAllowShrinking::No);
				ImpUVec[Body] = U;
				ReducedBias = ImpP[Body] + U * (Uu / D);
			}

			const FVector R = Batch.GetBodyPos(Body, Env) - Batch.GetBodyPos(Topo.BodyParent[Body], Env);
			ImpP[Parent] = ImpP[Parent] + TranslateForce(ReducedBias, R);
		}

		// ---- root: floating base velocity change ----
		const int32 RootIdx = Batch.BodyIndex(0, Env);
		OutDeltaV[0] = SolveSpatial6(ImpI[RootIdx], ImpP[0] * -1.0f);

		// ---- forward: propagate down, recovering each joint's velocity change ----
		for (int32 Body = 1; Body < NumBodies; ++Body)
		{
			const int32 Parent = Topo.BodyParent[Body];
			const int32 Idx = Batch.BodyIndex(Body, Env);
			const FVector R = Batch.GetBodyPos(Body, Env) - Batch.GetBodyPos(Parent, Env);
			const FSpatialVec ParentDV = TranslateMotion(OutDeltaV[Parent], R); // no c term for impulses
			const int32 DOFOffset = Topo.BodyDOFOffset[Body];

			if (IsJointLocked(Body, Env))
			{
				// Welded: moves exactly with its parent, no joint velocity.
				OutDeltaV[Body] = ParentDV;
				for (int32 k = 0; k < Topo.BodyDOFCount[Body]; ++k)
				{
					OutDeltaQd[DOFOffset + k] = 0.0f;
				}
			}
			else if (Topo.BodyDOFCount[Body] == 3)
			{
				// Same armature-scaled Dinv as the backward pass above.
				const float ArmScale = GetJointPassiveFactors(Topo, Body, 0.0f).ArmatureScale;
				const FMat3 Dinv = (ArmScale == 1.0f)
					? Inverse3x3(ImpI[Idx].Irot)
					: Inverse3x3(ImpI[Idx].Irot) * (1.0f / ArmScale);
				const FVector RHS = ImpU[Body].Ang - (ImpI[Idx].Irot * ParentDV.Ang + ImpI[Idx].H * ParentDV.Lin);
				const FVector DQ = Dinv * RHS;
				OutDeltaV[Body] = ParentDV + FSpatialVec{ DQ, FVector::ZeroVector };
				OutDeltaQd[DOFOffset + 0] = (float)DQ.X;
				OutDeltaQd[DOFOffset + 1] = (float)DQ.Y;
				OutDeltaQd[DOFOffset + 2] = (float)DQ.Z;
			}
			else
			{
				const FSpatialVec S{ RevoluteAxisWorld(Batch, Topo, Body, Env), FVector::ZeroVector };
				const float Uu = (float)ImpU[Body].Ang.X;
				const float D = (float)ImpU[Body].Ang.Y;
				const float DQ = (Uu - FSpatialVec::Dot(ImpUVec[Body], ParentDV)) / D;
				OutDeltaV[Body] = ParentDV + S * DQ;
				OutDeltaQd[DOFOffset] = DQ;
			}
		}
	}

	/**
	 * Fill OutLocked with 1 for every joint that is currently AT a stop AND still
	 * being driven further into it, for every env. Layout is body-major /
	 * env-minor with stride NumEnvs, ready to hand straight to
	 * ComputeArticulatedInertias(Batch, OutLocked.GetData(), NumEnvs).
	 *
	 * This is the production caller the welding path never had. Entry 024
	 * measured what it buys -- articulated effective mass at a foot going 18 kg
	 * -> 269 kg (14.8x) at the exact substep the heels were being dragged under
	 * -- and then only ever exercised it from a diagnostic, so the measurement
	 * stood while the mechanism sat unreachable from any real run.
	 *
	 * THE "STILL BEING DRIVEN INTO IT" CONDITION MATTERS. Welding is a
	 * one-sided approximation applied to a one-sided constraint: a joint at its
	 * stop can still leave in one direction, and a weld removes that direction
	 * too. Requiring the joint to be moving INTO the stop (not merely resting at
	 * it, and certainly not leaving it) keeps the approximation confined to the
	 * case it is actually right for -- a chain being compressed, which is when
	 * the rigid-strut effective mass is the correct answer. A joint that is
	 * unloading is left free and reports its compliant mass, which is also
	 * correct.
	 *
	 * Uses exactly the wrapped-range convention ClampJointLimits and the
	 * joint-limit constraint rows use; a joint with no authored range is never
	 * locked, matching their no-op behaviour.
	 */
	void BuildSaturatedJointLocks(const FCreatureBatchState& Batch, float MarginDeg, TArray<uint8>& OutLocked) const
	{
		const FCreatureTopology& Topo = Batch.GetTopology();
		const int32 NumBodies = Topo.NumBodies;
		const int32 NumEnvs = Batch.GetNumEnvs();
		OutLocked.SetNumZeroed(FMath::Max(1, NumBodies * NumEnvs), EAllowShrinking::No);
		for (int32 i = 0; i < OutLocked.Num(); ++i)
		{
			OutLocked[i] = 0;
		}
		if (Topo.NumDOF == 0)
		{
			return;
		}

		const float MarginRad = FMath::DegreesToRadians(FMath::Max(0.0f, MarginDeg));

		for (int32 Body = 1; Body < NumBodies; ++Body)
		{
			const int32 DOFOffset = Topo.BodyDOFOffset[Body];

			if (Topo.BodyDOFCount[Body] == 1)
			{
				if (!Topo.DOFHasMuscleCurve[DOFOffset])
				{
					continue;
				}
				const float MinDeg = Topo.DOFRangeMinDeg[DOFOffset];
				const float WidthDeg = Topo.DOFRangeMaxDeg[DOFOffset] - MinDeg;
				if (WidthDeg <= 0.0f)
				{
					continue;
				}
				for (int32 Env = 0; Env < NumEnvs; ++Env)
				{
					const int32 DOFIdx = Batch.DOFIndex(DOFOffset, Env);
					const float AngleDeg = FMath::RadiansToDegrees(Batch.JointPos[DOFIdx]);
					const float Qd = Batch.JointVel[DOFIdx];

					// Same [0,360) wrap relative to the range minimum the clamp uses --
					// this rig's ranges are authored wrapped (Knee1 is [283.2, 403.6]),
					// so a bind-pose joint reads 0 deg and is in range only once you
					// know 0 is 360.
					float Wrapped = FMath::Fmod(AngleDeg - MinDeg, 360.0f);
					if (Wrapped < 0.0f) Wrapped += 360.0f;

					float GapLoRad, GapHiRad;
					if (Wrapped <= WidthDeg)
					{
						GapLoRad = FMath::DegreesToRadians(Wrapped);
						GapHiRad = FMath::DegreesToRadians(WidthDeg - Wrapped);
					}
					else
					{
						// Inside the forbidden arc: the nearer stop is already violated.
						const float OverMax = Wrapped - WidthDeg;
						const float UnderMin = 360.0f - Wrapped;
						GapHiRad = FMath::DegreesToRadians(OverMax <= UnderMin ? -OverMax : 1.0e6f);
						GapLoRad = FMath::DegreesToRadians(OverMax <= UnderMin ? 1.0e6f : -UnderMin);
					}

					// Lower stop closes as qd goes negative, upper as it goes positive.
					const bool bIntoLo = (GapLoRad < MarginRad) && (Qd < 0.0f);
					const bool bIntoHi = (GapHiRad < MarginRad) && (Qd > 0.0f);
					if (bIntoLo || bIntoHi)
					{
						OutLocked[Body * NumEnvs + Env] = 1;
					}
				}
			}
			else if (Topo.BodyDOFCount[Body] == 3)
			{
				// Cone limit, same "largest authored per-axis width" half-angle the
				// clamp and the cone rows both use (see ClampJointLimits).
				float MaxWidthDeg = 0.0f;
				bool bHasAnyRange = false;
				for (int32 k = 0; k < 3; ++k)
				{
					if (Topo.DOFHasMuscleCurve[DOFOffset + k])
					{
						bHasAnyRange = true;
						MaxWidthDeg = FMath::Max(MaxWidthDeg, Topo.DOFRangeMaxDeg[DOFOffset + k] - Topo.DOFRangeMinDeg[DOFOffset + k]);
					}
				}
				if (!bHasAnyRange || MaxWidthDeg <= 0.0f)
				{
					continue;
				}
				const float MaxAngleRad = FMath::DegreesToRadians(MaxWidthDeg);

				for (int32 Env = 0; Env < NumEnvs; ++Env)
				{
					const FVector RotVec = Batch.GetJointRelRot(Body, Env).ToRotationVector();
					const float Angle = static_cast<float>(RotVec.Size());
					if (Angle <= UE_SMALL_NUMBER || (MaxAngleRad - Angle) >= MarginRad)
					{
						continue;
					}
					// Still opening the cone? d|RotVec|/dt > 0 along its own direction.
					const int32 DOFIdx = Batch.DOFIndex(DOFOffset, Env);
					const FVector JointVelVec(
						Batch.JointVel[DOFIdx],
						Batch.JointVel[DOFIdx + Batch.GetPaddedNumEnvs()],
						Batch.JointVel[DOFIdx + 2 * Batch.GetPaddedNumEnvs()]);
					if (FVector::DotProduct(RotVec / Angle, JointVelVec) > 0.0f)
					{
						OutLocked[Body * NumEnvs + Env] = 1;
					}
				}
			}
		}
	}

	/**
	 * Velocity change of WorldPoint on Body caused by a linear impulse there —
	 * i.e. one column of the contact's inverse effective-mass operator.
	 * Dividing 1 by the component along a direction gives the articulated
	 * effective mass in that direction, which is the number a velocity-level
	 * contact needs and which no hand-tuned constant can supply correctly.
	 */
	FVector ImpulseResponseAtPoint(const FCreatureBatchState& Batch, int32 Env, int32 Body,
	                               const FVector& WorldPoint, const FVector& LinearImpulse)
	{
		const FCreatureTopology& Topo = Batch.GetTopology();
		DVScratch.SetNum(Topo.NumBodies, EAllowShrinking::No);
		DQScratch.SetNum(FMath::Max(1, Topo.NumDOF), EAllowShrinking::No);
		for (int32 d = 0; d < DQScratch.Num(); ++d) DQScratch[d] = 0.0f;

		const FVector Arm = WorldPoint - Batch.GetBodyPos(Body, Env);
		const FSpatialVec Wrench{ FVector::CrossProduct(Arm, LinearImpulse), LinearImpulse };
		SolveImpulseResponse(Batch, Env, Body, Wrench, DVScratch, DQScratch);

		// Velocity change of the material point at WorldPoint.
		return DVScratch[Body].Lin + FVector::CrossProduct(DVScratch[Body].Ang, Arm);
	}

	/**
	 * Change in DOF's own joint velocity per unit joint-space impulse applied at
	 * that same DOF -- the articulated INVERSE inertia of one joint, i.e. the
	 * diagonal a joint-limit constraint row divides by. The analogue of
	 * ImpulseResponseAtPoint for joint space.
	 *
	 * Whole-tree, not local: pushing on one joint moves every body, and the
	 * response includes all of it. That is the entire reason a joint limit
	 * solved this way can brace a chain, while a position clamp cannot.
	 *
	 * Requires ComputeArticulatedInertias() for the current configuration.
	 */
	float JointImpulseResponse(const FCreatureBatchState& Batch, int32 Env, int32 DOF)
	{
		const FCreatureTopology& Topo = Batch.GetTopology();
		DVScratch.SetNum(Topo.NumBodies, EAllowShrinking::No);
		DQScratch.SetNum(FMath::Max(1, Topo.NumDOF), EAllowShrinking::No);
		JointImpScratch.SetNumZeroed(FMath::Max(1, Topo.NumDOF), EAllowShrinking::No);
		for (int32 d = 0; d < DQScratch.Num(); ++d) { DQScratch[d] = 0.0f; JointImpScratch[d] = 0.0f; }

		JointImpScratch[DOF] = 1.0f;
		SolveImpulseResponse(Batch, Env, INDEX_NONE, FSpatialVec{ FVector::ZeroVector, FVector::ZeroVector },
			DVScratch, DQScratch, JointImpScratch.GetData());
		JointImpScratch[DOF] = 0.0f;

		return DQScratch[DOF];
	}

	/** Apply a joint-space impulse at DOF, updating every body and joint velocity. */
	void ApplyJointImpulse(FCreatureBatchState& Batch, int32 Env, int32 DOF, float Impulse)
	{
		const FCreatureTopology& Topo = Batch.GetTopology();
		DVScratch.SetNum(Topo.NumBodies, EAllowShrinking::No);
		DQScratch.SetNum(FMath::Max(1, Topo.NumDOF), EAllowShrinking::No);
		JointImpScratch.SetNumZeroed(FMath::Max(1, Topo.NumDOF), EAllowShrinking::No);
		for (int32 d = 0; d < DQScratch.Num(); ++d) { DQScratch[d] = 0.0f; JointImpScratch[d] = 0.0f; }

		JointImpScratch[DOF] = Impulse;
		SolveImpulseResponse(Batch, Env, INDEX_NONE, FSpatialVec{ FVector::ZeroVector, FVector::ZeroVector },
			DVScratch, DQScratch, JointImpScratch.GetData());
		JointImpScratch[DOF] = 0.0f;

		for (int32 b = 0; b < Topo.NumBodies; ++b)
		{
			const int32 Idx = Batch.BodyIndex(b, Env);
			Batch.AngVelX[Idx] += (float)DVScratch[b].Ang.X;
			Batch.AngVelY[Idx] += (float)DVScratch[b].Ang.Y;
			Batch.AngVelZ[Idx] += (float)DVScratch[b].Ang.Z;
			Batch.LinVelX[Idx] += (float)DVScratch[b].Lin.X;
			Batch.LinVelY[Idx] += (float)DVScratch[b].Lin.Y;
			Batch.LinVelZ[Idx] += (float)DVScratch[b].Lin.Z;
		}
		for (int32 d = 0; d < Topo.NumDOF; ++d)
		{
			Batch.JointVel[Batch.DOFIndex(d, Env)] += DQScratch[d];
		}
	}

	/**
	 * Ball-joint analogue of JointImpulseResponse: change in the rate of
	 * change of the joint's OWN rotation-vector magnitude (d|RotVec|/dt) per
	 * unit joint-space impulse applied along Dir across all 3 of the ball
	 * joint's DOF slots at once, where Dir is normally the joint's current
	 * RotVec direction (or its negation) -- i.e. the 1D effective inverse
	 * mass along whichever direction a cone-limit constraint pushes. Same
	 * whole-tree articulated-inertia machinery as JointImpulseResponse,
	 * generalized from a single scalar DOF to a 3-DOF direction; SolveImpulseResponse
	 * already accepts a full 3-component JointImpulsePerDOF at a ball joint
	 * (see its BodyDOFCount==3 branch), so this needs no new solver math.
	 */
	float BallJointImpulseResponse(const FCreatureBatchState& Batch, int32 Env, int32 DOFOffset, const FVector& Dir)
	{
		const FCreatureTopology& Topo = Batch.GetTopology();
		DVScratch.SetNum(Topo.NumBodies, EAllowShrinking::No);
		DQScratch.SetNum(FMath::Max(1, Topo.NumDOF), EAllowShrinking::No);
		JointImpScratch.SetNumZeroed(FMath::Max(1, Topo.NumDOF), EAllowShrinking::No);
		for (int32 d = 0; d < DQScratch.Num(); ++d) { DQScratch[d] = 0.0f; JointImpScratch[d] = 0.0f; }

		JointImpScratch[DOFOffset + 0] = Dir.X;
		JointImpScratch[DOFOffset + 1] = Dir.Y;
		JointImpScratch[DOFOffset + 2] = Dir.Z;
		SolveImpulseResponse(Batch, Env, INDEX_NONE, FSpatialVec{ FVector::ZeroVector, FVector::ZeroVector },
			DVScratch, DQScratch, JointImpScratch.GetData());
		JointImpScratch[DOFOffset + 0] = 0.0f; JointImpScratch[DOFOffset + 1] = 0.0f; JointImpScratch[DOFOffset + 2] = 0.0f;

		return (float)Dir.X * DQScratch[DOFOffset + 0] + (float)Dir.Y * DQScratch[DOFOffset + 1] + (float)Dir.Z * DQScratch[DOFOffset + 2];
	}

	/** Apply a joint-space impulse of magnitude Impulse along Dir across a ball joint's 3 DOF slots at once -- the ball-joint analogue of ApplyJointImpulse. */
	void ApplyBallJointImpulse(FCreatureBatchState& Batch, int32 Env, int32 DOFOffset, const FVector& Dir, float Impulse)
	{
		const FCreatureTopology& Topo = Batch.GetTopology();
		DVScratch.SetNum(Topo.NumBodies, EAllowShrinking::No);
		DQScratch.SetNum(FMath::Max(1, Topo.NumDOF), EAllowShrinking::No);
		JointImpScratch.SetNumZeroed(FMath::Max(1, Topo.NumDOF), EAllowShrinking::No);
		for (int32 d = 0; d < DQScratch.Num(); ++d) { DQScratch[d] = 0.0f; JointImpScratch[d] = 0.0f; }

		JointImpScratch[DOFOffset + 0] = Dir.X * Impulse;
		JointImpScratch[DOFOffset + 1] = Dir.Y * Impulse;
		JointImpScratch[DOFOffset + 2] = Dir.Z * Impulse;
		SolveImpulseResponse(Batch, Env, INDEX_NONE, FSpatialVec{ FVector::ZeroVector, FVector::ZeroVector },
			DVScratch, DQScratch, JointImpScratch.GetData());
		JointImpScratch[DOFOffset + 0] = 0.0f; JointImpScratch[DOFOffset + 1] = 0.0f; JointImpScratch[DOFOffset + 2] = 0.0f;

		for (int32 b = 0; b < Topo.NumBodies; ++b)
		{
			const int32 Idx = Batch.BodyIndex(b, Env);
			Batch.AngVelX[Idx] += (float)DVScratch[b].Ang.X;
			Batch.AngVelY[Idx] += (float)DVScratch[b].Ang.Y;
			Batch.AngVelZ[Idx] += (float)DVScratch[b].Ang.Z;
			Batch.LinVelX[Idx] += (float)DVScratch[b].Lin.X;
			Batch.LinVelY[Idx] += (float)DVScratch[b].Lin.Y;
			Batch.LinVelZ[Idx] += (float)DVScratch[b].Lin.Z;
		}
		for (int32 d = 0; d < Topo.NumDOF; ++d)
		{
			Batch.JointVel[Batch.DOFIndex(d, Env)] += DQScratch[d];
		}
	}

	/** Read-only view of one body's articulated inertia, for conditioning audits. */
	FSpatialInertia DebugArticulatedInertia(const FCreatureBatchState& Batch, int32 Body, int32 Env) const
	{
		return ImpI[Batch.BodyIndex(Body, Env)];
	}

	/** Apply a linear impulse at a world point, updating every body and joint velocity. */
	void ApplyImpulseAtPoint(FCreatureBatchState& Batch, int32 Env, int32 Body,
	                         const FVector& WorldPoint, const FVector& LinearImpulse)
	{
		const FCreatureTopology& Topo = Batch.GetTopology();
		DVScratch.SetNum(Topo.NumBodies, EAllowShrinking::No);
		DQScratch.SetNum(FMath::Max(1, Topo.NumDOF), EAllowShrinking::No);
		for (int32 d = 0; d < DQScratch.Num(); ++d) DQScratch[d] = 0.0f;

		const FVector Arm = WorldPoint - Batch.GetBodyPos(Body, Env);
		const FSpatialVec Wrench{ FVector::CrossProduct(Arm, LinearImpulse), LinearImpulse };
		SolveImpulseResponse(Batch, Env, Body, Wrench, DVScratch, DQScratch);

		for (int32 b = 0; b < Topo.NumBodies; ++b)
		{
			const int32 Idx = Batch.BodyIndex(b, Env);
			Batch.AngVelX[Idx] += (float)DVScratch[b].Ang.X;
			Batch.AngVelY[Idx] += (float)DVScratch[b].Ang.Y;
			Batch.AngVelZ[Idx] += (float)DVScratch[b].Ang.Z;
			Batch.LinVelX[Idx] += (float)DVScratch[b].Lin.X;
			Batch.LinVelY[Idx] += (float)DVScratch[b].Lin.Y;
			Batch.LinVelZ[Idx] += (float)DVScratch[b].Lin.Z;
		}
		for (int32 d = 0; d < Topo.NumDOF; ++d)
		{
			Batch.JointVel[Batch.DOFIndex(d, Env)] += DQScratch[d];
		}
	}

	/**
	 * Two-body point contact within the SAME articulated tree -- e.g. one
	 * limb touching another. There is no separate two-body solve: the whole
	 * creature is one Featherstone tree, so this is resolved by
	 * SUPERPOSITION of two single-body impulse queries (SolveImpulseResponse
	 * is linear in its applied wrench) -- the response to "+Impulse at PtA
	 * on BodyA" plus the response to "-Impulse at PtB on BodyB", summed.
	 * Each call touches every body in the tree (that's how an impulse on one
	 * limb can be felt by another, through the shared torso), so both
	 * contributions matter at BOTH contact points.
	 *
	 * Dotting the result with a unit direction gives 1/m_eff for that
	 * direction, in exactly the shape ImpulseResponseAtPoint already returns
	 * for a body-vs-ground contact -- callers that build a contact row don't
	 * need to know this is two bodies instead of one.
	 *
	 * Requires ComputeArticulatedInertias() for the current configuration.
	 */
	FVector PairImpulseResponseAtPoint(const FCreatureBatchState& Batch, int32 Env,
	                                   int32 BodyA, const FVector& PtA,
	                                   int32 BodyB, const FVector& PtB,
	                                   const FVector& Impulse)
	{
		const FCreatureTopology& Topo = Batch.GetTopology();
		const int32 NumBodies = Topo.NumBodies;
		const int32 NumDOF = FMath::Max(1, Topo.NumDOF);

		// Two independent output sets -- unlike DVScratch/DQScratch (shared
		// mutable members meant for one query at a time), both calls' results
		// are needed simultaneously to sum them, so local arrays are used here
		// instead.
		TArray<FSpatialVec> DV_A, DV_B;
		TArray<float> DQ_A, DQ_B;
		DV_A.SetNum(NumBodies);
		DV_B.SetNum(NumBodies);
		DQ_A.SetNumZeroed(NumDOF);
		DQ_B.SetNumZeroed(NumDOF);

		const FVector ArmA = PtA - Batch.GetBodyPos(BodyA, Env);
		const FSpatialVec WrenchA{ FVector::CrossProduct(ArmA, Impulse), Impulse };
		SolveImpulseResponse(Batch, Env, BodyA, WrenchA, DV_A, DQ_A);

		const FVector ArmB = PtB - Batch.GetBodyPos(BodyB, Env);
		const FSpatialVec WrenchB{ FVector::CrossProduct(ArmB, -Impulse), -Impulse };
		SolveImpulseResponse(Batch, Env, BodyB, WrenchB, DV_B, DQ_B);

		// Velocity of the material point at PtA under BOTH impulses (BodyB's
		// impulse reaches PtA through the shared tree, same as BodyA's own).
		const FVector VA = (DV_A[BodyA].Lin + FVector::CrossProduct(DV_A[BodyA].Ang, ArmA))
		                 + (DV_B[BodyA].Lin + FVector::CrossProduct(DV_B[BodyA].Ang, ArmA));
		// Velocity of the material point at PtB under BOTH impulses.
		const FVector VB = (DV_A[BodyB].Lin + FVector::CrossProduct(DV_A[BodyB].Ang, ArmB))
		                 + (DV_B[BodyB].Lin + FVector::CrossProduct(DV_B[BodyB].Ang, ArmB));

		return VA - VB;
	}

	/**
	 * Apply an impulse pair (+Impulse at PtA on BodyA, -Impulse at PtB on
	 * BodyB) to every body and joint velocity -- the two-body analogue of
	 * ApplyImpulseAtPoint, via the same superposition PairImpulseResponseAtPoint
	 * uses (see that function's comment).
	 */
	void ApplyPairImpulseAtPoints(FCreatureBatchState& Batch, int32 Env,
	                              int32 BodyA, const FVector& PtA,
	                              int32 BodyB, const FVector& PtB,
	                              const FVector& Impulse)
	{
		const FCreatureTopology& Topo = Batch.GetTopology();
		const int32 NumBodies = Topo.NumBodies;
		const int32 NumDOF = FMath::Max(1, Topo.NumDOF);

		TArray<FSpatialVec> DV_A, DV_B;
		TArray<float> DQ_A, DQ_B;
		DV_A.SetNum(NumBodies);
		DV_B.SetNum(NumBodies);
		DQ_A.SetNumZeroed(NumDOF);
		DQ_B.SetNumZeroed(NumDOF);

		const FVector ArmA = PtA - Batch.GetBodyPos(BodyA, Env);
		const FSpatialVec WrenchA{ FVector::CrossProduct(ArmA, Impulse), Impulse };
		SolveImpulseResponse(Batch, Env, BodyA, WrenchA, DV_A, DQ_A);

		const FVector ArmB = PtB - Batch.GetBodyPos(BodyB, Env);
		const FSpatialVec WrenchB{ FVector::CrossProduct(ArmB, -Impulse), -Impulse };
		SolveImpulseResponse(Batch, Env, BodyB, WrenchB, DV_B, DQ_B);

		for (int32 b = 0; b < NumBodies; ++b)
		{
			const int32 Idx = Batch.BodyIndex(b, Env);
			const FSpatialVec DV = DV_A[b] + DV_B[b];
			Batch.AngVelX[Idx] += (float)DV.Ang.X;
			Batch.AngVelY[Idx] += (float)DV.Ang.Y;
			Batch.AngVelZ[Idx] += (float)DV.Ang.Z;
			Batch.LinVelX[Idx] += (float)DV.Lin.X;
			Batch.LinVelY[Idx] += (float)DV.Lin.Y;
			Batch.LinVelZ[Idx] += (float)DV.Lin.Z;
		}
		for (int32 d = 0; d < Topo.NumDOF; ++d)
		{
			Batch.JointVel[Batch.DOFIndex(d, Env)] += DQ_A[d] + DQ_B[d];
		}
	}

	/**
	 * PASSIVE JOINT DAMPING, applied as an exact velocity-level impulse.
	 * Call once per substep AFTER Step() and BEFORE contact resolution.
	 *
	 * WHY THIS IS NOT A TORQUE IN PASS 2, which is where it started and where the
	 * obvious implementation puts it.
	 *
	 * MuJoCo's `damping` is a plain force, `qfrc_passive = -d * qvel`, added to
	 * the RHS before forward dynamics. That was tried here first, with the
	 * coefficient derived from the joint's own articulated inertia
	 * (d = D / TimeConstant) so it would stay unit-free, and with the claim that
	 * an isolated joint therefore loses exactly (dt / TimeConstant) of its
	 * velocity per step.
	 *
	 * THE CLAIM WAS FALSE, and a test caught it. In an articulated tree the joint
	 * acceleration is not tau/D — it is (u - U^T a_parent)/D, and a_parent itself
	 * responds to the same torque. The parent recoils, which AMPLIFIES the
	 * relative acceleration by a factor that depends on the whole subtree's mass
	 * distribution and is unbounded in general (it grows without limit as the
	 * parent gets light). Measured on a 2-body chain: a damping torque sized for a
	 * 10% velocity reduction removed 197% instead, i.e. it REVERSED the joint and
	 * nearly doubled its speed in the opposite direction. At TimeConstant == dt it
	 * overshot by 19x. The stability cap keyed on D was worse than useless: 1/D is
	 * a LOWER bound on the response gain, so capping with it is anti-conservative
	 * precisely when it matters.
	 *
	 * No cheap analytic upper bound on that gain exists — it depends on the
	 * parent's articulated inertia, which Pass 2 has not finished accumulating at
	 * the moment the child's torque is formed.
	 *
	 * SO THE GAIN IS MEASURED INSTEAD OF ASSUMED. `SolveImpulseResponse` already
	 * computes the exact whole-tree response to a joint-space impulse -- that is
	 * the machinery the joint-limit rows are built on. One query gives
	 * g = d(qd)/d(impulse) for this joint including every reaction in the tree, so
	 * the impulse that produces a WANTED velocity change is just Delta/g. That is:
	 *
	 *   - EXACT, not approximate: the fraction removed is the fraction requested.
	 *   - UNCONDITIONALLY STABLE, genuinely this time: DampFrac is clamped to 1, so
	 *     damping can remove all of a joint's velocity and never reverse it. There
	 *     is no d*dt/D < 2 bound to respect, which is the bound MuJoCo's
	 *     `implicitfast` integrator exists to remove. This gets the same property
	 *     without an implicit integrator.
	 *   - MOMENTUM CONSERVING: a joint impulse is internal, and
	 *     SolveImpulseResponse propagates its reaction through the whole tree, so
	 *     the parent takes its share. A post-hoc scaling of JointVel -- the other
	 *     obvious cheap fix -- would NOT do that, and would quietly violate
	 *     conservation of angular momentum on every substep.
	 *
	 * COST, stated plainly: one whole-tree impulse pass per damped joint per
	 * substep (per env), plus one articulated-inertia factorization. On the real
	 * rig that is 26 revolutes + 14 ball joints = 40 passes, against roughly 200
	 * for the contact solve, so it is a real but not dominant addition. It is
	 * skipped entirely for joints whose time constant is 0, and the whole function
	 * returns immediately if no joint has damping at all.
	 *
	 * SEQUENCING: joints are processed in order, and each application perturbs the
	 * others, so the fraction removed is exact only for the joint being processed
	 * at the moment it is processed. That is Gauss-Seidel, and it is harmless here
	 * in a way it is not for contact: every application is strictly dissipative,
	 * so the worst a sweep ordering can do is damp slightly less than asked. It
	 * can never inject energy.
	 */
	void ApplyJointDamping(FCreatureBatchState& Batch, float Dt)
	{
		if (Dt <= 0.0f)
		{
			return;
		}
		const FCreatureTopology& Topo = Batch.GetTopology();
		if (Topo.NumDOF == 0 || Topo.DOFDampingTimeConstant.Num() == 0)
		{
			return;
		}

		// Cheap early-out so an undamped rig pays nothing beyond this scan.
		bool bAnyDamping = false;
		for (int32 Body = 1; Body < Topo.NumBodies && !bAnyDamping; ++Body)
		{
			const int32 DOF = Topo.BodyDOFOffset[Body];
			bAnyDamping = Topo.DOFDampingTimeConstant.IsValidIndex(DOF) && Topo.DOFDampingTimeConstant[DOF] > 0.0f;
		}
		if (!bAnyDamping)
		{
			return;
		}

		// Damping needs its own factorization: it runs before contact, which
		// factorizes again with its own weld set. Deliberately unlocked here --
		// welding is about saturated limits, not about damping.
		ComputeArticulatedInertias(Batch);

		const int32 NumBodies = Topo.NumBodies;
		const int32 NumDOF = FMath::Max(1, Topo.NumDOF);
		const int32 NumEnvs = Batch.GetNumEnvs();

		// Solve once for a unit joint-space impulse, then scale that same response
		// by whatever magnitude achieves the requested velocity change and apply
		// it. One tree pass, not the two that JointImpulseResponse followed by
		// ApplyJointImpulse would cost.
		//
		// Sizing DVScratch/DQScratch/JointImpScratch happens HERE, per call,
		// rather than once before the env loop the way an earlier version of
		// this function did -- now that the env loop below is a ParallelFor
		// (see its own comment) and these three are `static thread_local` (see
		// their declaration), a one-time size-before-the-loop would only have
		// sized the CALLING thread's copy; every OTHER worker thread that
		// picks up an env task needs its own copy sized before first use too.
		// Cheap: EAllowShrinking::No makes this a no-op once a given thread's
		// copy is already the right size, which is every call after its first.
		auto SolveScaleAndApply = [&](int32 Env, int32 DOFOffset, int32 NumComponents,
		                              const FVector& Dir, float TargetDelta) -> bool
		{
			DVScratch.SetNum(NumBodies, EAllowShrinking::No);
			DQScratch.SetNum(NumDOF, EAllowShrinking::No);
			JointImpScratch.SetNumZeroed(NumDOF, EAllowShrinking::No);
			for (int32 d = 0; d < NumDOF; ++d) { DQScratch[d] = 0.0f; JointImpScratch[d] = 0.0f; }
			if (NumComponents == 1)
			{
				JointImpScratch[DOFOffset] = 1.0f;
			}
			else
			{
				JointImpScratch[DOFOffset + 0] = (float)Dir.X;
				JointImpScratch[DOFOffset + 1] = (float)Dir.Y;
				JointImpScratch[DOFOffset + 2] = (float)Dir.Z;
			}

			SolveImpulseResponse(Batch, Env, INDEX_NONE, FSpatialVec{ FVector::ZeroVector, FVector::ZeroVector },
				DVScratch, DQScratch, JointImpScratch.GetData());

			// Gain along the constrained direction, whole-tree.
			const float Gain = (NumComponents == 1)
				? DQScratch[DOFOffset]
				: (float)(Dir.X * DQScratch[DOFOffset + 0] + Dir.Y * DQScratch[DOFOffset + 1] + Dir.Z * DQScratch[DOFOffset + 2]);
			if (!(Gain > UE_SMALL_NUMBER) || !FMath::IsFinite(Gain))
			{
				return false; // no compliance along this direction, or degenerate
			}

			const float Impulse = TargetDelta / Gain;
			if (!FMath::IsFinite(Impulse))
			{
				return false;
			}

			for (int32 b = 0; b < NumBodies; ++b)
			{
				const int32 Idx = Batch.BodyIndex(b, Env);
				Batch.AngVelX[Idx] += (float)(DVScratch[b].Ang.X * Impulse);
				Batch.AngVelY[Idx] += (float)(DVScratch[b].Ang.Y * Impulse);
				Batch.AngVelZ[Idx] += (float)(DVScratch[b].Ang.Z * Impulse);
				Batch.LinVelX[Idx] += (float)(DVScratch[b].Lin.X * Impulse);
				Batch.LinVelY[Idx] += (float)(DVScratch[b].Lin.Y * Impulse);
				Batch.LinVelZ[Idx] += (float)(DVScratch[b].Lin.Z * Impulse);
			}
			for (int32 d = 0; d < Topo.NumDOF; ++d)
			{
				Batch.JointVel[Batch.DOFIndex(d, Env)] += DQScratch[d] * Impulse;
			}
			return true;
		};

		// Parallelized across envs, same idiom as Step()'s own passes (see its
		// class comment): one task per env, no synchronization needed because
		// SolveScaleAndApply only ever touches Batch slots this Env owns
		// (Batch.BodyIndex/DOFIndex always fold Env into the offset) and its
		// own scratch (DVScratch/DQScratch/JointImpScratch, and transitively
		// ImpP/ImpU/ImpUVec inside SolveImpulseResponse) is now
		// `static thread_local` -- see those declarations' comment for why
		// this was UNSAFE before 2026-08-25 (this loop used to be sequential
		// specifically because of this) and confirmed safe now. Measured
		// before this change: ~220-260ms for this one function at 256 envs,
		// vs. ~7-8ms for Step()'s own 5-pass ABA integration on the same
		// batch -- this loop was doing whole-tree impulse-response solves
		// (SolveImpulseResponse: one backward + one forward tree pass per
		// damped joint per env) fully single-threaded.
		ParallelFor(NumEnvs, [&](int32 Env)
		{
			for (int32 Body = 1; Body < NumBodies; ++Body)
			{
				const FJointPassiveFactors Passive = GetJointPassiveFactors(Topo, Body, Dt);
				if (!Passive.HasDamping())
				{
					continue;
				}
				const int32 DOFOffset = Topo.BodyDOFOffset[Body];

				if (Topo.BodyDOFCount[Body] == 1)
				{
					const float Qd = Batch.JointVel[Batch.DOFIndex(DOFOffset, Env)];
					if (FMath::Abs(Qd) <= UE_SMALL_NUMBER)
					{
						continue;
					}
					SolveScaleAndApply(Env, DOFOffset, 1, FVector::ZeroVector, -Passive.DampFrac * Qd);
				}
				else if (Topo.BodyDOFCount[Body] == 3)
				{
					// Isotropic, and structurally so: a ball joint's JointVel is world
					// angular velocity, so a per-axis coefficient would be a
					// coefficient on world axes and would change meaning as the
					// creature turned. Damping along the joint's own current spin
					// direction is the frame-invariant choice.
					const FVector W(
						Batch.JointVel[Batch.DOFIndex(DOFOffset + 0, Env)],
						Batch.JointVel[Batch.DOFIndex(DOFOffset + 1, Env)],
						Batch.JointVel[Batch.DOFIndex(DOFOffset + 2, Env)]);
					const float Speed = (float)W.Size();
					if (Speed <= UE_SMALL_NUMBER)
					{
						continue;
					}
					SolveScaleAndApply(Env, DOFOffset, 3, W / Speed, -Passive.DampFrac * Speed);
				}
			}
		});
	}

};
