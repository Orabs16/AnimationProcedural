#pragma once

// RL environment glue over FCreatureBatchState: observations, actions,
// reward, termination, and reset. Topology-agnostic (no Muto-specific
// assumptions) — the caller supplies ground-contact points (e.g. from
// CreatureGroundContact::BuildMutoContactPoints) and per-env config.
//
// First trainable objective is STANDING / BALANCE, not locomotion (see
// project roadmap/memory: the original "six-limbed" handoff mentioned
// forward-locomotion reward shaping, but that's a later milestone —
// standing validates the observation/action/reward/reset plumbing with a
// much smaller reward-design surface first). There is no forward-velocity
// term here.
//
// Deliberately decoupled from the Learning Agents plugin itself — no
// ULearningAgentsInteractor/Trainer here, just plain C++ functions over
// FCreatureBatchState, testable the same way the rest of AgentSolver is
// (see Tests/CreatureRLEnvironmentTest.cpp). Wiring this into an actual
// ULearningAgentsManager/Interactor/Trainer (Blueprint/editor-asset setup,
// Python trainer process config) is a separate follow-up task — that's
// UObject/editor-asset work best done with the user driving, not something
// to guess at here.

#include "CoreMinimal.h"
#include "CreatureBatchState.h"
#include "CreatureGroundContact.h"

namespace CreatureRLEnvironment
{
	using CreatureGroundContact::FContactPointDef;
	using CreatureGroundContact::FContactPointState;

	struct FEnvConfig
	{
		float GroundZ = 0.0f;

		// Z distance above GroundZ the torso sits at in the rest/standing
		// pose. Depends on leg geometry, so the caller supplies it (e.g.
		// derived once from the topology's rest pose) rather than this file
		// guessing at Muto-specific leg lengths.
		float TargetTorsoHeight = 100.0f;

		// The torso-LOCAL direction that counts as "up" for the upright-ness
		// check below — defaults to local +Z, which is only correct if the
		// rig's own bone convention happens to have local +Z pointing the
		// same way the character's own up direction does. NOT a safe
		// assumption in general (confirmed false for Muto's Pelvis bone, via
		// direct measurement: the correct standing pose measured
		// dot(TorsoLocalZ, WorldUp) == -0.675, i.e. actively wrong, not just
		// imprecise) — the caller should derive this from the same rest
		// rotation used for "standing" (e.g.
		// StandingTorsoRot.UnrotateVector(FVector::UpVector), the local axis
		// that rotates TO world-up at the known-correct standing pose)
		// rather than assuming identity.
		FVector LocalUpAxis = FVector::UpVector;

		// Scales a normalized [-1,1] policy action into Batch.JointTorque.
		// A single uniform scale for now — per-DOF/per-limb torque limits
		// are a reasonable follow-up once real per-joint strength data
		// exists (see MassProfile_Muto's placeholder-mass note in memory).
		// Kept small deliberately: MassProfile_Muto's per-bone masses are
		// still placeholder ~1.0 with correspondingly small inertia
		// (~0.1-0.3, see the synthetic test topologies), and this same class
		// of solver already hit a real torque-vs-inertia numerical blowup
		// at torques as "small" as 50 during earlier SIMD/ball-joint testing
		// (fixed there by dropping to +/-2.0) — this default leaves some
		// headroom above that but nowhere near the physically-nonsensical
		// values a torque limit tuned for real (much heavier) creature mass
		// would imply.
		// RESCALED 2026-08-13. The old 5.0 was chosen when every body was a 1.0 kg
		// placeholder, and the reasoning above is now obsolete: with the authored
		// masses the single worst body (BElbow1_L) needs tau = 3.44e7 just to hold
		// ITSELF against gravity, so 5.0 was ~6.9 MILLION times too weak — the
		// creature had, in effect, no muscles at all and could only ever collapse.
		// 5e7 gives ~1.5x margin over that worst holding torque.
		//
		// This is a torque LIMIT, not a commanded torque: the policy emits [-1,1]
		// and the muscle curves scale it, so a larger limit widens the reachable
		// range rather than making the creature thrash. The reward's torque
		// penalty is normalized by this same value, so its meaning is unchanged.
		// See SOLVER_DEBUG_LOG.md entry 017.
		float MaxTorquePerDOF = 5.0e7f;

		// "Fallen over" termination thresholds.
		float MinUprightDot = 0.5f;    // dot(TorsoUp, WorldUp) below this ends the episode
		float MinHeightFraction = 0.5f; // torso height below this fraction of TargetTorsoHeight ends the episode

		// Reward weights.
		float AliveBonus = 1.0f;
		float UprightWeight = 1.0f;
		float BalanceWeight = 0.5f;
		// TorquePenalty itself is a mean-of-normalized-squared-torque term in
		// [0,1] (see ComputeReward) — this weight is directly comparable to
		// AliveBonus/UprightWeight/BalanceWeight's own ~1-ish scale, not a
		// tiny fraction of a huge raw sum-of-squared-torques like it used to be.
		float TorquePenaltyWeight = 0.1f;
	};

	/** Observation layout: [TorsoUp.XYZ, TorsoLinVel.XYZ, TorsoAngVel.XYZ, TorsoHeight-Target, (JointPos,JointVel) x NumDOF, (bTouching,NormalForce) x NumContactPoints]. */
	inline int32 GetObservationSize(const FCreatureTopology& Topo, int32 NumContactPoints)
	{
		return 10 + 2 * Topo.NumDOF + 2 * NumContactPoints;
	}

	inline void ComputeObservations(
		const FCreatureBatchState& Batch,
		int32 Env,
		const FEnvConfig& Config,
		const TArray<FContactPointDef>& ContactPoints,
		const TArray<FContactPointState>& ContactStates, // ContactPoints.Num() x NumEnvsForContacts, see ApplyGroundContactForces's OutState layout
		int32 NumEnvsForContacts,
		TArray<float>& OutObservation)
	{
		const FCreatureTopology& Topo = Batch.GetTopology();
		OutObservation.Reset(GetObservationSize(Topo, ContactPoints.Num()));

		const FQuat TorsoRot = Batch.GetBodyRot(0, Env);
		const FVector TorsoUp = TorsoRot.RotateVector(Config.LocalUpAxis);
		const int32 TorsoIdx = Batch.BodyIndex(0, Env);

		OutObservation.Add((float)TorsoUp.X);
		OutObservation.Add((float)TorsoUp.Y);
		OutObservation.Add((float)TorsoUp.Z);
		OutObservation.Add(Batch.LinVelX[TorsoIdx]);
		OutObservation.Add(Batch.LinVelY[TorsoIdx]);
		OutObservation.Add(Batch.LinVelZ[TorsoIdx]);
		OutObservation.Add(Batch.AngVelX[TorsoIdx]);
		OutObservation.Add(Batch.AngVelY[TorsoIdx]);
		OutObservation.Add(Batch.AngVelZ[TorsoIdx]);
		const float HeightAboveGround = (float)Batch.GetBodyPos(0, Env).Z - Config.GroundZ;
		OutObservation.Add(HeightAboveGround - Config.TargetTorsoHeight);

		for (int32 DOF = 0; DOF < Topo.NumDOF; ++DOF)
		{
			const int32 DOFIdx = Batch.DOFIndex(DOF, Env);
			OutObservation.Add(Batch.JointPos[DOFIdx]);
			OutObservation.Add(Batch.JointVel[DOFIdx]);
		}

		for (int32 PointIdx = 0; PointIdx < ContactPoints.Num(); ++PointIdx)
		{
			const FContactPointState& State = ContactStates[PointIdx * NumEnvsForContacts + Env];
			OutObservation.Add(State.bTouching ? 1.0f : 0.0f);
			OutObservation.Add(State.NormalForce);
		}
	}

	/** Normalized [-1,1] action per DOF -> Batch.JointTorque, scaled by Config.MaxTorquePerDOF. */
	inline void ApplyActions(FCreatureBatchState& Batch, int32 Env, const TArray<float>& NormalizedActions, const FEnvConfig& Config)
	{
		const FCreatureTopology& Topo = Batch.GetTopology();
		check(NormalizedActions.Num() == Topo.NumDOF);
		for (int32 DOF = 0; DOF < Topo.NumDOF; ++DOF)
		{
			const int32 DOFIdx = Batch.DOFIndex(DOF, Env);
			Batch.JointTorque[DOFIdx] = FMath::Clamp(NormalizedActions[DOF], -1.0f, 1.0f) * Config.MaxTorquePerDOF;
		}
	}

	/**
	 * Standing/balance reward: alive bonus + how upright the torso is +
	 * how close the contact points' force-weighted centroid (center of
	 * pressure) sits under the torso horizontally (a standard postural
	 * balance proxy — supersedes the vaguer "bias weight toward the ankle"
	 * phrasing from the original handoff, which predates the ground contact
	 * model's CanTouchGround-driven redesign and no longer maps onto a
	 * single distinguished "ankle" point per limb) - torque-squared penalty
	 * to discourage needless effort.
	 */
	inline float ComputeReward(
		const FCreatureBatchState& Batch,
		int32 Env,
		const FEnvConfig& Config,
		const TArray<FContactPointDef>& ContactPoints,
		const TArray<FContactPointState>& ContactStates,
		int32 NumEnvsForContacts)
	{
		const FCreatureTopology& Topo = Batch.GetTopology();
		const FQuat TorsoRot = Batch.GetBodyRot(0, Env);
		const FVector TorsoUp = TorsoRot.RotateVector(Config.LocalUpAxis);
		const float UprightDot = FMath::Max(0.0f, (float)FVector::DotProduct(TorsoUp, FVector::UpVector));

		FVector WeightedContactSum = FVector::ZeroVector;
		float TotalForce = 0.0f;
		for (int32 PointIdx = 0; PointIdx < ContactPoints.Num(); ++PointIdx)
		{
			const FContactPointState& State = ContactStates[PointIdx * NumEnvsForContacts + Env];
			if (State.bTouching)
			{
				const FContactPointDef& Point = ContactPoints[PointIdx];
				// Use the SAME world-surface derivation the contact solver used to
				// produce State.NormalForce, rather than re-deriving it here.
				// This used to be `BodyPos + BodyRot.RotateVector(Point.LocalOffset)`
				// with no radius offset and no capsule-end handling — so for every
				// STRUCTURAL point (LocalOffset == ZeroVector, and there are 25 of
				// those against 10 authored ones once bAllBodiesCollideWithGround is
				// on) the centre of pressure was placed at the body's JOINT ORIGIN
				// instead of anywhere near the ground it was pressing on. The balance
				// term is the second-largest positive term in this reward, so that
				// mis-placement was shaping the policy directly.
				//
				// Averaging the two capsule ends is the honest reduction of a capsule
				// to one point: FContactPointState aggregates both ends' impulses into
				// a single per-point force (see ResolveGroundContactImpulses' OutState
				// accumulation, which does `+=` across ends), so the position paired
				// with that force should be the midpoint of the ends that generated it.
				FVector Surfaces[2];
				const int32 NumEnds = CreatureGroundContact::GetContactPointWorldSurfaces(
					Batch, Point, Env, FVector::UpVector, Surfaces);
				const FVector WorldPoint = (NumEnds == 2) ? (Surfaces[0] + Surfaces[1]) * 0.5f : Surfaces[0];
				WeightedContactSum += WorldPoint * State.NormalForce;
				TotalForce += State.NormalForce;
			}
		}

		float BalanceTerm = 0.0f;
		if (TotalForce > KINDA_SMALL_NUMBER)
		{
			const FVector CenterOfPressure = WeightedContactSum / TotalForce;
			const FVector TorsoPos = Batch.GetBodyPos(0, Env);
			const float HorizOffset = (float)FVector::Dist2D(CenterOfPressure, TorsoPos);
			// 1 when CoP is directly under the torso, decaying with offset
			// scaled by the body's own size (TargetTorsoHeight as a proxy
			// for creature scale) so the falloff isn't tied to arbitrary
			// world units.
			BalanceTerm = FMath::Exp(-HorizOffset / FMath::Max(Config.TargetTorsoHeight, KINDA_SMALL_NUMBER));
		}

		// Mean of (Tau/MaxTorquePerDOF)^2 across DOFs, i.e. always in [0,1]
		// regardless of NumDOF or MaxTorquePerDOF — keeps TorquePenaltyWeight's
		// meaning portable across topologies/torque limits instead of scaling
		// with joint count (a flat per-DOF sum here made this term's actual
		// magnitude wildly exceed the ~2.5-max positive terms once NumDOF
		// and MaxTorquePerDOF were both nontrivial, e.g. ~50 DOF x 50 torque
		// -> a worst-case penalty two orders of magnitude larger than
		// anything else in the reward, dominating training).
		float TorquePenalty = 0.0f;
		if (Topo.NumDOF > 0 && Config.MaxTorquePerDOF > KINDA_SMALL_NUMBER)
		{
			for (int32 DOF = 0; DOF < Topo.NumDOF; ++DOF)
			{
				const float NormalizedTau = Batch.JointTorque[Batch.DOFIndex(DOF, Env)] / Config.MaxTorquePerDOF;
				TorquePenalty += NormalizedTau * NormalizedTau;
			}
			TorquePenalty /= Topo.NumDOF;
		}

		const float Reward = Config.AliveBonus
			+ Config.UprightWeight * UprightDot
			+ Config.BalanceWeight * BalanceTerm
			- Config.TorquePenaltyWeight * TorquePenalty;

		// A numerical blowup upstream (see IsBodyStateValid) can still reach
		// here for the one step where the corrupted state's reward/completion
		// are gathered together, BEFORE IsTerminated below has a chance to
		// force the reset — never hand the trainer a NaN/Inf reward sample
		// for that step. Falls back to 0 (in-distribution: normal reward
		// here is roughly [-0.1, 2.5]), not a large negative "punishment"
		// value — a blowup is a numerical artifact of THIS simulation, not
		// something the agent chose, and (with bUseGradNormMaxClipping now
		// on, see AMutoRLTrainingDriver's constructor) an earlier, larger
		// fallback here was a real suspect for repeatedly injecting
		// out-of-distribution reward outliers into PPO's advantage/critic
		// statistics across however many of the (256, by default) parallel
		// envs blow up at any given time — plausibly contributing to the
		// exact kind of rare gradient-explosion event that produces NaN
		// network weights after an otherwise-fine training run.
		return FMath::IsFinite(Reward) ? Reward : 0.0f;
	}

	/**
	 * True if this env's body state (any body — a blowup anywhere in the
	 * tree reaches the torso within the same solver step, since Pass 2/3 of
	 * the ABA backward/forward passes fold every body's contribution into
	 * every other, but checking all bodies is cheap and doesn't rely on that)
	 * OR any DOF's JointPos/JointVel contains any NaN/Inf. A physics blowup
	 * (large torque + a near-singular configuration, numerical resonance,
	 * etc.) is a real, expected occurrence during early/exploratory RL
	 * training — without this check it silently escapes IsTerminated below
	 * (NaN/Inf compared against MinUprightDot/MinHeightFraction is always
	 * false, so a blown-up env was NEVER being terminated) and gets encoded
	 * straight into the next observation, which is a hard UE assert
	 * (LearningArray.h's Array::Check) inside Learning Agents, crashing the
	 * whole training process rather than just failing one episode. Confirmed
	 * this is exactly what was happening: a real training run crashed with
	 * "Invalid value nan found at flat array index 0" (TorsoUp.X) after
	 * ~2 minutes of otherwise-normal (if ragdoll-y) early training.
	 *
	 * The DOF-array check is NOT redundant with the body-array one above,
	 * despite Pass 1 folding JointVel into a body's AngVel via
	 * JointAngVelWorld = AxisWorld*JointRate (CreatureBatchSolver.h): that
	 * fold-in happens in Pass 1, using whatever JointVel existed BEFORE this
	 * step's own Pass 3 integration runs. If a DOF's JointVel blows up
	 * during Pass 3 of the LAST substep of a StepPhysicsSubstepped() call,
	 * there is no subsequent Pass 1 within that same call to propagate it
	 * into any body's AngVel — so the body-level check alone can miss it for
	 * exactly one step, and GatherAgentObservation reads Batch.JointPos/
	 * JointVel directly (not derived from body state) into the observation
	 * vector regardless. Confirmed exactly this gap in practice: a real run
	 * crashed with "-inf found at flat array index 11" (JointVel[0], per the
	 * observation layout above) while every body-level value was still
	 * finite, in AMutoRLVisualizerActor::Tick() specifically (RunInference
	 * reads the DOF arrays fresh at the START of the very next tick, before
	 * that tick's own Pass 1 has run at all).
	 */
	inline bool IsBodyStateValid(const FCreatureBatchState& Batch, int32 Env)
	{
		const FCreatureTopology& Topo = Batch.GetTopology();
		for (int32 Body = 0; Body < Topo.NumBodies; ++Body)
		{
			const FQuat Rot = Batch.GetBodyRot(Body, Env);
			const FVector Pos = Batch.GetBodyPos(Body, Env);
			const int32 Idx = Batch.BodyIndex(Body, Env);
			const bool bFinite =
				FMath::IsFinite(Rot.X) && FMath::IsFinite(Rot.Y) && FMath::IsFinite(Rot.Z) && FMath::IsFinite(Rot.W) &&
				FMath::IsFinite(Pos.X) && FMath::IsFinite(Pos.Y) && FMath::IsFinite(Pos.Z) &&
				FMath::IsFinite(Batch.LinVelX[Idx]) && FMath::IsFinite(Batch.LinVelY[Idx]) && FMath::IsFinite(Batch.LinVelZ[Idx]) &&
				FMath::IsFinite(Batch.AngVelX[Idx]) && FMath::IsFinite(Batch.AngVelY[Idx]) && FMath::IsFinite(Batch.AngVelZ[Idx]);
			if (!bFinite)
			{
				return false;
			}
		}
		for (int32 DOF = 0; DOF < Topo.NumDOF; ++DOF)
		{
			const int32 DOFIdx = Batch.DOFIndex(DOF, Env);
			if (!FMath::IsFinite(Batch.JointPos[DOFIdx]) || !FMath::IsFinite(Batch.JointVel[DOFIdx]))
			{
				return false;
			}
		}
		return true;
	}

	/**
	 * True iff every ContactPointState::NormalForce for this env is finite.
	 * NOT covered by IsBodyStateValid (that checks Batch only -- ContactStates
	 * is a separate array the caller owns, populated by
	 * CreatureGroundContact::ResolveGroundContactImpulses). Needed for the
	 * exact same reason IsBodyStateValid checks the DOF arrays separately
	 * from the body arrays (see that function's comment): a value written
	 * during the LAST substep of a StepPhysicsSubstepped() call can be
	 * non-finite even when the body/DOF state that same substep produced is
	 * itself still finite (or gets caught and reset before the NEXT read),
	 * because nothing between "ResolveGroundContactImpulses writes
	 * ContactStates" and "the next tick's RunInference reads it" ever
	 * revisits that array -- ResetEnv does not touch it. Confirmed exactly
	 * this gap in practice (2026-08-16): a real crash reached
	 * AMutoRLVisualizerActor::Tick()'s RunInference with a non-finite
	 * observation entry even after IsBodyStateValid-driven resets were
	 * already firing regularly.
	 */
	inline bool IsContactStateValid(const TArray<FContactPointState>& ContactStates, int32 NumContactPoints, int32 Env, int32 NumEnvsForContacts)
	{
		for (int32 PointIdx = 0; PointIdx < NumContactPoints; ++PointIdx)
		{
			const int32 Idx = PointIdx * NumEnvsForContacts + Env;
			if (!ContactStates.IsValidIndex(Idx) || !FMath::IsFinite(ContactStates[Idx].NormalForce))
			{
				return false;
			}
		}
		return true;
	}

	/**
	 * Zeroes this env's own slots in ContactStates (bTouching=false,
	 * NormalForce=0) -- the ContactPointState analogue of what ResetEnv does
	 * for Batch. Call alongside ResetEnv whenever a reset is triggered by
	 * IsTerminated, since ResetEnv itself does not touch ContactStates (it
	 * only knows about Batch) -- without this, a non-finite NormalForce that
	 * caused the reset would otherwise survive it untouched and immediately
	 * fail the very next tick's IsContactStateValid check again. Safe for a
	 * multi-env batch: only this env's strided slots are touched, so other
	 * envs sharing the same ContactStates array are unaffected.
	 */
	inline void ClearContactStatesForEnv(TArray<FContactPointState>& ContactStates, int32 NumContactPoints, int32 Env, int32 NumEnvsForContacts)
	{
		for (int32 PointIdx = 0; PointIdx < NumContactPoints; ++PointIdx)
		{
			const int32 Idx = PointIdx * NumEnvsForContacts + Env;
			if (ContactStates.IsValidIndex(Idx))
			{
				ContactStates[Idx] = FContactPointState();
			}
		}
	}

	/**
	 * True once the creature has fallen over (too tilted or too low) or its
	 * state has numerically blown up — caller resets the env when this is
	 * true. ContactStates/NumContactPoints/NumEnvsForContacts are optional
	 * (defaulted for source compatibility with every existing caller/test);
	 * pass them to also catch the ContactPointState leak IsBodyStateValid
	 * alone cannot see (see IsContactStateValid's comment) — every real
	 * caller (the training driver's completion callback, the RL visualizer's
	 * Tick) should pass them.
	 */
	inline bool IsTerminated(const FCreatureBatchState& Batch, int32 Env, const FEnvConfig& Config,
		const TArray<FContactPointState>* ContactStates = nullptr, int32 NumContactPoints = 0, int32 NumEnvsForContacts = 1)
	{
		if (!IsBodyStateValid(Batch, Env))
		{
			return true;
		}
		if (ContactStates && !IsContactStateValid(*ContactStates, NumContactPoints, Env, NumEnvsForContacts))
		{
			return true;
		}
		const FQuat TorsoRot = Batch.GetBodyRot(0, Env);
		const float UprightDot = (float)FVector::DotProduct(TorsoRot.RotateVector(Config.LocalUpAxis), FVector::UpVector);
		const float HeightAboveGround = (float)Batch.GetBodyPos(0, Env).Z - Config.GroundZ;
		return UprightDot < Config.MinUprightDot || HeightAboveGround < Config.MinHeightFraction * Config.TargetTorsoHeight;
	}

	/**
	 * Resets one env to a standing pose (StandingTorsoPos/Rot, all joints at
	 * their rest/zero position, zero velocities, cleared external forces),
	 * with small domain-randomization noise on the torso's horizontal
	 * position and orientation. Joint "rest" here means JointPos=0 for
	 * revolutes and identity JointRelRot for ball joints — matching how
	 * MutoTopology's rest-pose offsets were derived, i.e. this is the
	 * skeleton's authored bind pose, not an arbitrary zero.
	 */
	inline void ResetEnv(
		FCreatureBatchState& Batch,
		int32 Env,
		const FVector& StandingTorsoPos,
		const FQuat& StandingTorsoRot,
		FRandomStream& Stream,
		float PosNoiseStdDev,
		float AngleNoiseRad)
	{
		const FCreatureTopology& Topo = Batch.GetTopology();

		const FVector NoisyPos = StandingTorsoPos + FVector(Stream.FRandRange(-PosNoiseStdDev, PosNoiseStdDev), Stream.FRandRange(-PosNoiseStdDev, PosNoiseStdDev), 0.0f);
		Batch.SetBodyPos(0, Env, NoisyPos);
		const FQuat NoiseRot(Stream.VRand(), Stream.FRandRange(-AngleNoiseRad, AngleNoiseRad));
		Batch.SetBodyRot(0, Env, (NoiseRot * StandingTorsoRot).GetNormalized());

		const int32 TorsoIdx = Batch.BodyIndex(0, Env);
		Batch.LinVelX[TorsoIdx] = Batch.LinVelY[TorsoIdx] = Batch.LinVelZ[TorsoIdx] = 0.0f;
		Batch.AngVelX[TorsoIdx] = Batch.AngVelY[TorsoIdx] = Batch.AngVelZ[TorsoIdx] = 0.0f;

		for (int32 Body = 1; Body < Topo.NumBodies; ++Body)
		{
			Batch.SetJointRelRot(Body, Env, FQuat::Identity); // ball joints only; harmless no-op storage for revolutes
			const int32 DOFOffset = Topo.BodyDOFOffset[Body];
			const int32 DOFCount = Topo.BodyDOFCount[Body];
			for (int32 k = 0; k < DOFCount; ++k)
			{
				const int32 DOFIdx = Batch.DOFIndex(DOFOffset + k, Env);
				Batch.JointPos[DOFIdx] = 0.0f;
				Batch.JointVel[DOFIdx] = 0.0f;
				Batch.JointTorque[DOFIdx] = 0.0f;
			}
			// Body world pos/rot themselves are recomputed by the solver's
			// Pass 1 from JointPos/parent on the next Step() — no need to
			// set them directly here.
		}

		Batch.ClearExternalForces(Env);
	}
}
