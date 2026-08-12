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
	TArray<FVector> BodyJointAxisLocal;      // revolute axis, in parent's rest frame
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
	TArray<float> DOFRangeMinDeg;
	TArray<float> DOFRangeMaxDeg;
	TArray<uint8> DOFHasMuscleCurve;

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

		DOFExtensionCurve.SetNum(NumDOF);
		DOFFlexionCurve.SetNum(NumDOF);
		DOFRangeMinDeg.SetNumZeroed(NumDOF);
		DOFRangeMaxDeg.SetNumZeroed(NumDOF);
		DOFHasMuscleCurve.SetNumZeroed(NumDOF); // defaults false — multiplier 1 unless populated
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

        ExtForceX.SetNumZeroed(BodySlots);
        ExtForceY.SetNumZeroed(BodySlots);
        ExtForceZ.SetNumZeroed(BodySlots);
        ExtTorqueX.SetNumZeroed(BodySlots);
        ExtTorqueY.SetNumZeroed(BodySlots);
        ExtTorqueZ.SetNumZeroed(BodySlots);

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
     * Accumulate a world-space force at a world-space point on one body in
     * one env — e.g. a rocket impact or a hitscan shot. This does NOT touch
     * velocity directly: it's picked up by the solver's existing bias-force
     * pass next step, so it propagates through the joint chain correctly
     * instead of yanking one link out of sync with its neighbors.
     * Call once per hit; call ClearExternalForces() after the solver
     * consumes it (or let it persist across steps for a multi-frame push).
     */
    FORCEINLINE void ApplyForceAtPoint(int32 BodyIdx, int32 EnvIdx, const FVector &Force, const FVector &WorldPoint)
    {
        const int32 Idx = BodyIndex(BodyIdx, EnvIdx);
        ExtForceX[Idx] += static_cast<float>(Force.X);
        ExtForceY[Idx] += static_cast<float>(Force.Y);
        ExtForceZ[Idx] += static_cast<float>(Force.Z);

        const FVector BodyPos = GetBodyPos(BodyIdx, EnvIdx);
        const FVector Torque = FVector::CrossProduct(WorldPoint - BodyPos, Force);
        ExtTorqueX[Idx] += static_cast<float>(Torque.X);
        ExtTorqueY[Idx] += static_cast<float>(Torque.Y);
        ExtTorqueZ[Idx] += static_cast<float>(Torque.Z);
    }

    /** Zero the accumulated external wrench for one env after the solver has consumed it. */
    void ClearExternalForces(int32 EnvIdx)
    {
        for (int32 BodyIdx = 0; BodyIdx < Topology.NumBodies; ++BodyIdx)
        {
            const int32 Idx = BodyIndex(BodyIdx, EnvIdx);
            ExtForceX[Idx] = ExtForceY[Idx] = ExtForceZ[Idx] = 0.0f;
            ExtTorqueX[Idx] = ExtTorqueY[Idx] = ExtTorqueZ[Idx] = 0.0f;
        }
    }

    /** Domain randomization for one env, applied at episode reset. */
    void RandomizeEnv(int32 EnvIdx, const FRandomStream &Stream, float MinStrength, float MaxStrength,
                      float LimbLossChance, float MaxCarriedMass)
    {
        for (int32 Limb = 0; Limb < Topology.NumLimbs; ++Limb)
        {
            const int32 Idx = LimbIndex(Limb, EnvIdx);
            LimbStrengthScale[Idx] = Stream.FRandRange(MinStrength, MaxStrength);
            LimbActive[Idx] = Stream.FRand() > LimbLossChance ? 1 : 0;
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

    // Body-major, env-minor — accumulated external wrench (hits, pushes, explosions)
    TArray<float> ExtForceX, ExtForceY, ExtForceZ;
    TArray<float> ExtTorqueX, ExtTorqueY, ExtTorqueZ;

    // Limb-major / env-major per-env domain randomization
    TArray<float> LimbStrengthScale;
    TArray<uint8> LimbActive;
    TArray<float> CarriedMass; // per-env, not per-limb

private:
    FCreatureTopology Topology;
    int32 NumEnvs = 0;
    int32 PaddedNumEnvs = 0;
};