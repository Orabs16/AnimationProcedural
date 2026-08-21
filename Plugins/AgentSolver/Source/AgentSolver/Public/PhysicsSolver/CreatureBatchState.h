#pragma once

#include "CoreMinimal.h"
#include "Curves/CurveFloat.h"

/**
 * Static description of a single creature's kinematic tree.
 * Shared across all N parallel training environments — the topology
 * never changes per-env; only state and per-env parameters do.
 *
 * Fixed-size topology (with per-limb "active" flags rather than removing
 * bodies) is deliberate: a variable-size tree per env would break the
 * uniform SoA layout below and force per-env branching in the solver.
 */
struct FCreatureTopology
{
    // Body 0 is always the root (torso). BodyParent[i] < i for all i > 0,
    // so a single forward pass over indices visits parents before children.
    TArray<int32> BodyParent;

    // Joint DOF count per body (0 for the root; 1-3 per limb joint)
    TArray<int32> BodyDOFCount;

    // First DOF index into the flattened joint-space arrays, per body
    TArray<int32> BodyDOFOffset;

    // Which limb (0..NumLimbs-1) a body belongs to; INDEX_NONE for the torso
    TArray<int32> BodyLimbIndex;

    int32 NumBodies = 0;
    int32 NumDOF = 0;
    int32 NumLimbs = 6;

	// Per-body physical properties (index 0 = root/torso, unused for joint fields).
	// Populate these after calling Build(). All "local" vectors are in the
	// PARENT's rest frame for joint fields, and in the body's OWN rest frame
	// for BodyLocalCoMOffset/inertia — rotating the axis by the parent's
	// current world rotation, rather than the child's, is deliberate: a
	// revolute axis is invariant under rotation about itself, so either
	// convention works, but using the parent avoids a chicken-and-egg
	// dependency on the child's not-yet-computed world rotation.
	// Revolute axis, expressed in the PARENT's frame (it is consumed as
	// ParentRot.RotateVector(axis)). If your rig authors the axis in the
	// child bone's own frame -- as Muto's muscles do -- rotate it by
	// BodyRestRotInParent before storing it here, or the joint turns about the
	// wrong axis by exactly the bind-pose rotation. See MutoTopology.h's
	// JointAxisLocal.Add() and SOLVER_DEBUG_LOG.md entry 022.
	TArray<FVector> BodyJointAxisLocal;
	TArray<FVector> BodyJointOffsetInParent; // joint location, in parent's rest frame
	// Body's bind-pose rotation relative to its parent's rest frame — the same
	// thing a "Make Relative (Parent to Child)" node would give you. Without
	// this, the forward kinematics in CreatureBatchSolver.h would implicitly
	// treat every body's rest orientation as identical to its parent's (since
	// only the offset TRANSLATION was ever composed in, never the rest
	// ROTATION), collapsing the whole skeleton to "every bone points wherever
	// the torso points" at JointPos==0 regardless of how the rig is actually
	// authored. Index 0 (root) is NOT read by the FK pass (Body 0 has no
	// parent, the FK loop starts at Body 1) — callers are free to repurpose
	// it, e.g. Muto's topology stores the torso's own bind-pose rotation in
	// COMPONENT space there, since "standing" reset needs that and it isn't
	// used for anything else.
	TArray<FQuat> BodyRestRotInParent;
	TArray<FVector> BodyLocalCoMOffset;      // CoM offset from joint origin, body's own rest frame
	TArray<float>   BodyMass;
	TArray<FVector> BodyInertiaDiagLocal;    // diagonal inertia about CoM, body's own rest frame

	// Rest-pose offset (body's own rest frame) from this body's origin to an
	// unarticulated "fused" child bone hanging past it, if any — ZeroVector
	// for every body without one. Added for Muto's Tip bones
	// (FTip/BTip/MTip/FeetTip, see MutoTopology.h's 2026-08-11 Bone/Child
	// semantics correction): those are no longer their own ABA body (no
	// muscle ever drives them independently), but their real mesh geometry
	// still hangs past their parent (Feet/Hand)'s own joint origin, which
	// ground-contact placement needs to know about — without this, a
	// contact point placed (and ground-aligned) at Feet's own origin has no
	// way to account for the fact that the visual foot/hand extends further
	// out, and the unaccounted-for Tip geometry visibly pokes through the
	// ground. Purely a placement hint for CreatureGroundContact.h; the
	// solver itself never reads this.
	TArray<FVector> BodyFusedTipOffset;

	// Per-bone collision/contact radius, authored manually (see
	// FMassMuscleDataMass::Radius) — a lightweight stand-in for real
	// PhysicsAsset collision volumes, which can't be read from this module
	// (Chaos/AVX2 conflict, see AgentSolver.Build.cs). Zero for every body
	// unless a caller populates it (MutoTopology.h does, from MassAsset).
	// Consumed by CreatureGroundContact.h to place ground-contact points at
	// a bone's true visual surface instead of its joint origin.
	TArray<float> BodyRadius;

	// Per-bone capsule half-height (see FMassMuscleDataMass::
	// CapsuleHalfHeight) — pulls the collision capsule's start cap back from
	// BodyFusedTipOffset along the bone->tip axis. Zero (default) collapses
	// the capsule down to the old single sphere-at-the-tip. Consumed by
	// CreatureGroundContact.h alongside BodyRadius/BodyFusedTipOffset.
	TArray<float> BodyCapsuleHalfHeight;

	// Per-body multiplier on the gravity wrench (1 = normal, 0 = this body
	// is weightless). MuJoCo's `gravcomp` in ratio form. The solver applies
	// gravity as Mass * Gravity * BodyGravityScale[Body] in the bias-force
	// pass of both step variants; everything else about the body is
	// unchanged, so a 0-gravity body still has full mass and inertia and
	// still reacts to whatever its joints do to it.
	//
	// This exists so a rig can model a FIXED MOUNT, which this solver has no
	// other way to express: the root is always a free-floating 6-DOF base, so
	// an "anchor" body given overwhelming mass still free-falls at exactly g
	// like everything else (the equivalence principle does not care how heavy
	// it is), and a pendulum hung off it never swings because the whole
	// assembly falls together with no internal stress. Setting the anchor's
	// scale to 0 holds it up. See CreaturePendulumEnergyTest.
	TArray<float> BodyGravityScale;

	// This body's index in the source REFERENCE SKELETON, or INDEX_NONE for any
	// topology not built from one (every synthetic test rig, and the extracted
	// sub-chains MutoIsolatedLimbTest builds). Populated by BuildMutoTopology.
	//
	// Exists so consumers can look authored per-bone data up by index instead of
	// by name -- see UMassMuscleProfileAssetMass::FindBoneByIndex. Purely a
	// lookup key for callers; the solver itself never reads it.
	TArray<int32> BodyBoneIndex;

	// Muscle strength-vs-angle curves, one entry per DOF (same flattened
	// indexing as JointPos/JointTorque — DOFHasMuscleCurve[d]==false means no
	// authored data for that DOF, and the solver treats it as multiplier 1,
	// i.e. no behavior change for topologies that never populate these, like
	// the synthetic test rigs). DOFRangeMinDeg/MaxDeg is the muscle's
	// authored [MinRange,MaxRange] in degrees with MaxDeg already unwrapped
	// to be > MinDeg (matching FMassMuscleViewportClient's arc-drawing
	// convention for ranges that cross the 0/360 boundary) — the curves'
	// own X axis is normalized 0-1 across that range, not raw degrees.
	TArray<FRuntimeFloatCurve> DOFExtensionCurve;
	TArray<FRuntimeFloatCurve> DOFFlexionCurve;

	// The muscle's authored SCALAR strengths (FMassMuscleDataMuscle::
	// ExtensionStrength / FlexionStrength), which multiply the curve value for
	// the matching direction. Curve and scalar answer different questions and
	// both are authored per muscle: the curve is the SHAPE of strength across
	// the joint's range (normalized 0-1 on X, and typically peaking at 1), the
	// scalar is HOW STRONG that muscle is overall relative to its neighbors.
	// Multiplying them is the only reading under which editing either one in
	// the MassMuscleProfile tool does what the tool's UI implies.
	//
	// Default 1 (not 0) so a DOF with a curve but no meaningful scalar, and
	// every synthetic test topology, behaves exactly as before.
	TArray<float> DOFExtensionStrength;
	TArray<float> DOFFlexionStrength;
	TArray<float> DOFRangeMinDeg;
	TArray<float> DOFRangeMaxDeg;
	TArray<uint8> DOFHasMuscleCurve;

	/**
	 * ARMATURE — added rotor inertia on the joint, MuJoCo's `armature`.
	 *
	 * Stored as a DIMENSIONLESS RATIO rather than an absolute inertia:
	 * the solver uses D_effective = D_articulated * (1 + ratio). MuJoCo's own
	 * `armature` is absolute (kg m^2), and that is deliberately NOT copied here.
	 * This solver runs in cm-kg-s while every published armature value assumes
	 * SI metres, and an absolute constant transplanted across that boundary is
	 * exactly the class of bug that cost this project two entire sessions
	 * (SOLVER_DEBUG_LOG.md entries 001 and 017: a torque limit ~7 million times
	 * too small, a contact spring 143x too weak). A ratio cannot be wrong by
	 * six orders of magnitude, and it is additionally invariant to the 3000x
	 * spread in this rig's own per-body inertias — one value means the same
	 * thing at the torso and at a fingertip, which an absolute coefficient
	 * emphatically does not.
	 *
	 * Why it matters: D is the denominator of every joint acceleration
	 * (qddot = (u - U^T a_parent) / D) and of every articulated effective mass
	 * the contact solver queries. Raising it lowers the stiffness of the whole
	 * coupled system, which is the one lever entry 024's iteration sweep did
	 * NOT rule out — that sweep refuted convergence RATE as the missing
	 * ingredient, not system conditioning. It is also the standard reason
	 * MuJoCo locomotion rigs are well-behaved at a few hundred Hz with ~10
	 * solver iterations.
	 *
	 * Ball joints (3-DOF) are ISOTROPIC by construction: a per-axis coefficient
	 * would have to live in world components here (JointVel for a ball joint is
	 * world angular velocity, see JointRelRotX's comment) and would therefore
	 * not be frame-invariant. Slot 0 of the body's three DOFs is authoritative;
	 * BuildMutoTopology writes all three the same.
	 *
	 * 0 (default) reproduces the pre-armature solver exactly — not merely
	 * closely: every armature term is an exact scalar multiple of the ratio, so
	 * at 0 they vanish identically rather than leaving float residue. See
	 * FCreatureABASolver's FJointPassiveFactors for the algebra.
	 */
	TArray<float> DOFArmatureRatio;

	/**
	 * JOINT DAMPING — passive viscous resistance, MuJoCo's `damping`.
	 *
	 * Stored as a TIME CONSTANT in seconds, not a coefficient: an isolated joint
	 * loses the fraction (dt / DampingTimeConstant) of its own velocity per
	 * substep. 0 disables it.
	 *
	 * Applied by FCreatureABASolver::ApplyJointDamping as a velocity-level
	 * IMPULSE, not as a torque folded into tau. That distinction is not a
	 * refinement — the torque form was implemented first, and it does not work.
	 * See ApplyJointDamping's comment for the measurement that killed it: in an
	 * articulated tree the parent recoils from the damping torque and amplifies
	 * the joint's response by an unbounded, configuration-dependent factor, so a
	 * torque sized for a 10% reduction was measured removing 197% — reversing the
	 * joint. The impulse form measures that gain instead of assuming it.
	 *
	 * Same unit-safety argument as DOFArmatureRatio: a time constant cannot be
	 * wrong by six orders of magnitude the way an absolute coefficient
	 * transplanted from an SI-metre MuJoCo model can.
	 *
	 * The trade, stated plainly: a fixed timescale is NOT physically identical
	 * to a fixed coefficient. Real viscous damping has a constant coefficient,
	 * so its timescale varies as the articulated inertia changes with pose.
	 * This damps every joint on a consistent timescale instead. For an RL rig
	 * spanning 1 kg tips and a 3282 kg torso that is the more useful behaviour,
	 * but it is a modelling choice, not the exact MuJoCo model.
	 *
	 * Why it matters: between its stops a ball joint here is a perfectly
	 * frictionless pendulum with zero dissipation anywhere in its range. A
	 * measured passive drop showed Head2/LowerMouth climbing from ~0 to 97 and
	 * 77 rad/s in 200 ms before the rig diverged. Ball-joint cone limit rows
	 * were added to absorb that AT THE STOP; damping absorbs it across the
	 * whole range, which is what MuJoCo relies on and is the more direct fix.
	 *
	 * Ball joints are isotropic, same reasoning and same slot-0 convention as
	 * DOFArmatureRatio.
	 */
	TArray<float> DOFDampingTimeConstant;

    void Build(const TArray<int32> &InBodyParent, const TArray<int32> &InBodyDOFCount,
               const TArray<int32> &InBodyLimbIndex)
    {
        check(InBodyParent.Num() == InBodyDOFCount.Num());
        check(InBodyParent.Num() == InBodyLimbIndex.Num());

        BodyParent = InBodyParent;
        BodyDOFCount = InBodyDOFCount;
        BodyLimbIndex = InBodyLimbIndex;
        NumBodies = BodyParent.Num();

        BodyDOFOffset.SetNumUninitialized(NumBodies);
        int32 Offset = 0;
        for (int32 i = 0; i < NumBodies; ++i)
        {
            BodyDOFOffset[i] = Offset;
            Offset += BodyDOFCount[i];
        }
        NumDOF = Offset;

		BodyJointAxisLocal.Init(FVector::UpVector, NumBodies);
		BodyJointOffsetInParent.SetNumZeroed(NumBodies);
		BodyRestRotInParent.Init(FQuat::Identity, NumBodies);
		BodyLocalCoMOffset.SetNumZeroed(NumBodies);
		BodyMass.SetNumZeroed(NumBodies);
		BodyInertiaDiagLocal.SetNumZeroed(NumBodies);
		BodyFusedTipOffset.SetNumZeroed(NumBodies);
		BodyRadius.SetNumZeroed(NumBodies);
		BodyCapsuleHalfHeight.SetNumZeroed(NumBodies);
		BodyGravityScale.Init(1.0f, NumBodies);
		BodyBoneIndex.Init(INDEX_NONE, NumBodies);

		DOFExtensionCurve.SetNum(NumDOF);
		DOFFlexionCurve.SetNum(NumDOF);
		DOFExtensionStrength.Init(1.0f, NumDOF);
		DOFFlexionStrength.Init(1.0f, NumDOF);
		DOFRangeMinDeg.SetNumZeroed(NumDOF);
		DOFRangeMaxDeg.SetNumZeroed(NumDOF);
		DOFHasMuscleCurve.SetNumZeroed(NumDOF); // defaults false — multiplier 1 unless populated
		// Both zeroed, i.e. OFF by default: an untouched topology (every
		// synthetic test rig) reproduces the pre-armature, pre-damping solver
		// exactly. Callers opt in — MutoTopology.h fills these from the
		// driver-supplied uniform defaults.
		DOFArmatureRatio.SetNumZeroed(NumDOF);
		DOFDampingTimeConstant.SetNumZeroed(NumDOF);
		// Caller fills these in per-body after Build() — Body 0 (root/torso)
		// only needs BodyMass / BodyInertiaDiagLocal / BodyLocalCoMOffset;
		// its joint fields are unused since it's solved as a floating base.
    }
};

/**
 * Structure-of-Arrays state for N parallel creature instances.
 *
 * Layout: body-major / env-minor, Index(Body, Env) = Body * PaddedNumEnvs + Env.
 * Position/rotation are split into scalar component arrays (PosX/PosY/PosZ,
 * RotX/RotY/RotZ/RotW) rather than arrays of FVector/FQuat, so that a per-body
 * slice across PaddedNumEnvs environments is a run of contiguous floats the
 * solver can load straight into wide SIMD registers with no gather/shuffle.
 *
 * NumEnvs is padded up to SIMDWidth so every per-body slice divides evenly
 * into whole vector registers — no remainder loop in the hot path.
 */
class FCreatureBatchState
{
public:
    static constexpr int32 SIMDWidth = 8; // AVX2 float lanes; use 4 for SSE/NEON

    void Init(const FCreatureTopology &InTopology, int32 InNumEnvs)
    {
        Topology = InTopology;
        NumEnvs = InNumEnvs;
        PaddedNumEnvs = Align(InNumEnvs, SIMDWidth);

        const int32 BodySlots = Topology.NumBodies * PaddedNumEnvs;
        const int32 DOFSlots = Topology.NumDOF * PaddedNumEnvs;
        const int32 LimbSlots = Topology.NumLimbs * PaddedNumEnvs;

        PosX.SetNumZeroed(BodySlots);
        PosY.SetNumZeroed(BodySlots);
        PosZ.SetNumZeroed(BodySlots);

        RotX.SetNumZeroed(BodySlots);
        RotY.SetNumZeroed(BodySlots);
        RotZ.SetNumZeroed(BodySlots);
        RotW.Init(1.0f, BodySlots); // identity quaternion

        // Ball joints only (BodyDOFCount[Body] == 3): persistent child-relative-to-parent
        // orientation, integrated via exp-map each step (see CreatureBatchSolver.h). Left
        // allocated-but-unused for revolute/root bodies — negligible memory, keeps addressing
        // uniform via BodyIndex().
        JointRelRotX.SetNumZeroed(BodySlots);
        JointRelRotY.SetNumZeroed(BodySlots);
        JointRelRotZ.SetNumZeroed(BodySlots);
        JointRelRotW.Init(1.0f, BodySlots); // identity quaternion

        LinVelX.SetNumZeroed(BodySlots);
        LinVelY.SetNumZeroed(BodySlots);
        LinVelZ.SetNumZeroed(BodySlots);
        AngVelX.SetNumZeroed(BodySlots);
        AngVelY.SetNumZeroed(BodySlots);
        AngVelZ.SetNumZeroed(BodySlots);

        JointPos.SetNumZeroed(DOFSlots);
        JointVel.SetNumZeroed(DOFSlots);
        JointTorque.SetNumZeroed(DOFSlots); // policy actuation input


        LimbStrengthScale.Init(1.0f, LimbSlots);
        LimbActive.Init(1, LimbSlots);
        CarriedMass.SetNumZeroed(PaddedNumEnvs);
    }

    FORCEINLINE int32 BodyIndex(int32 BodyIdx, int32 EnvIdx) const { return BodyIdx * PaddedNumEnvs + EnvIdx; }
    FORCEINLINE int32 DOFIndex(int32 DOFIdx, int32 EnvIdx) const { return DOFIdx * PaddedNumEnvs + EnvIdx; }
    FORCEINLINE int32 LimbIndex(int32 LimbIdx, int32 EnvIdx) const { return LimbIdx * PaddedNumEnvs + EnvIdx; }

    /** Convenience gather — for call sites outside the hot solver loop. */
    FORCEINLINE FVector GetBodyPos(int32 BodyIdx, int32 EnvIdx) const
    {
        const int32 Idx = BodyIndex(BodyIdx, EnvIdx);
        return FVector(PosX[Idx], PosY[Idx], PosZ[Idx]);
    }

    FORCEINLINE void SetBodyPos(int32 BodyIdx, int32 EnvIdx, const FVector &P)
    {
        const int32 Idx = BodyIndex(BodyIdx, EnvIdx);
        PosX[Idx] = static_cast<float>(P.X);
        PosY[Idx] = static_cast<float>(P.Y);
        PosZ[Idx] = static_cast<float>(P.Z);
    }

    FORCEINLINE FQuat GetBodyRot(int32 BodyIdx, int32 EnvIdx) const
    {
        const int32 Idx = BodyIndex(BodyIdx, EnvIdx);
        return FQuat(RotX[Idx], RotY[Idx], RotZ[Idx], RotW[Idx]);
    }

    FORCEINLINE void SetBodyRot(int32 BodyIdx, int32 EnvIdx, const FQuat &Q)
    {
        const int32 Idx = BodyIndex(BodyIdx, EnvIdx);
        RotX[Idx] = static_cast<float>(Q.X);
        RotY[Idx] = static_cast<float>(Q.Y);
        RotZ[Idx] = static_cast<float>(Q.Z);
        RotW[Idx] = static_cast<float>(Q.W);
    }

    /** Ball joints only — child orientation relative to parent. See JointRelRotX's comment in Init(). */
    FORCEINLINE FQuat GetJointRelRot(int32 BodyIdx, int32 EnvIdx) const
    {
        const int32 Idx = BodyIndex(BodyIdx, EnvIdx);
        return FQuat(JointRelRotX[Idx], JointRelRotY[Idx], JointRelRotZ[Idx], JointRelRotW[Idx]);
    }

    FORCEINLINE void SetJointRelRot(int32 BodyIdx, int32 EnvIdx, const FQuat &Q)
    {
        const int32 Idx = BodyIndex(BodyIdx, EnvIdx);
        JointRelRotX[Idx] = static_cast<float>(Q.X);
        JointRelRotY[Idx] = static_cast<float>(Q.Y);
        JointRelRotZ[Idx] = static_cast<float>(Q.Z);
        JointRelRotW[Idx] = static_cast<float>(Q.W);
    }

    /**
     * Domain randomization for one env, applied at episode reset (see
     * CreatureRLEnvironment::ResetEnv, which is the only caller outside tests).
     *
     * All three outputs are OVERWRITTEN, never accumulated, so calling this
     * with neutral arguments (1, 1, 0, 0) is the documented way to RESTORE an
     * env to un-randomized values -- which matters, because LimbStrengthScale/
     * LimbActive/CarriedMass persist across episodes otherwise, and an env that
     * drew a weak or missing limb would keep it forever once randomization was
     * switched off mid-run.
     *
     * LimbLossChance is the probability a limb is DEACTIVATED. 0 means never:
     * the comparison is strictly `< LimbLossChance` and short-circuits at 0, so
     * FRand()'s half-open [0,1) range cannot knock a limb out by drawing exactly
     * zero (which the previous `FRand() > LimbLossChance` form could).
     */
    void RandomizeEnv(int32 EnvIdx, const FRandomStream &Stream, float MinStrength, float MaxStrength,
                      float LimbLossChance, float MaxCarriedMass)
    {
        for (int32 Limb = 0; Limb < Topology.NumLimbs; ++Limb)
        {
            const int32 Idx = LimbIndex(Limb, EnvIdx);
            LimbStrengthScale[Idx] = Stream.FRandRange(MinStrength, MaxStrength);
            LimbActive[Idx] = (LimbLossChance > 0.0f && Stream.FRand() < LimbLossChance) ? 0 : 1;
        }
        CarriedMass[EnvIdx] = Stream.FRandRange(0.0f, MaxCarriedMass);
    }

    const FCreatureTopology &GetTopology() const { return Topology; }
    int32 GetNumEnvs() const { return NumEnvs; }
    int32 GetPaddedNumEnvs() const { return PaddedNumEnvs; }

    // Body-major, env-minor — scalar component arrays for SIMD-friendly access
    TArray<float> PosX, PosY, PosZ;
    TArray<float> RotX, RotY, RotZ, RotW;
    // Ball joints only (Topology.BodyDOFCount[Body] == 3) — see JointRelRotX's comment in Init().
    TArray<float> JointRelRotX, JointRelRotY, JointRelRotZ, JointRelRotW;
    TArray<float> LinVelX, LinVelY, LinVelZ;
    TArray<float> AngVelX, AngVelY, AngVelZ;

    // DOF-major, env-minor (generalized coordinates)
    TArray<float> JointPos;
    TArray<float> JointVel;
    TArray<float> JointTorque;

    // Limb-major / env-major per-env domain randomization
    TArray<float> LimbStrengthScale;
    TArray<uint8> LimbActive;
    TArray<float> CarriedMass; // per-env, not per-limb

private:
    FCreatureTopology Topology;
    int32 NumEnvs = 0;
    int32 PaddedNumEnvs = 0;
};