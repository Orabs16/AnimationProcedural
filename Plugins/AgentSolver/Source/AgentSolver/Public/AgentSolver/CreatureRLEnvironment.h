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
#include "PhysicsSolver/CreatureBatchState.h"
#include "PhysicsSolver/CreatureGroundContact.h"
#include "AgentSolver/CreatureImitation.h"

namespace CreatureRLEnvironment
{
	using CreatureGroundContact::FContactPointDef;
	using CreatureGroundContact::FContactPointState;

	/**
	 * What the agent is being trained to do. Deliberately an exclusive MODE
	 * rather than a set of weights that can all be on at once: the standing
	 * objective's uprightness/balance/torso-height terms and an imitation
	 * target are two different opinions about where the body should be, and
	 * leaving both live means they quietly pull against each other whenever
	 * the reference pose is anything but a symmetric upright stand. Alive
	 * bonus, torque penalty and the energy/muscle maluses are shared by both
	 * modes -- those are costs, not objectives, and are meaningful either way.
	 */
	enum class EObjectiveMode : uint8
	{
		Standing = 0,
		Imitation = 1,
	};

	/**
	 * The imitation reward's per-step inputs. Passed by pointer to
	 * ComputeReward so the standing path costs nothing and so every existing
	 * caller (and every existing test) keeps compiling untouched.
	 *
	 * Frame is sampled PER ENV, not once per step: reference state
	 * initialization gives each episode its own starting phase, so at any
	 * instant the 256 parallel envs are spread across the whole clip.
	 */
	struct FImitationTarget
	{
		const CreatureImitation::FReferenceFrame* Frame = nullptr;
		const TArray<int32>* EndEffectorBodies = nullptr;

		/** Height the batch's torso stands at in the rest pose -- the baseline FReferenceFrame::RootHeightAboveRest is applied on top of. */
		float RestTorsoHeight = 100.0f;
	};

	/** The four imitation terms, for the control panel's per-component graphs. Same purpose as ComputeReward's existing Out* pointers. */
	struct FImitationBreakdown
	{
		float PoseReward = 0.0f;
		float VelocityReward = 0.0f;
		float EndEffectorReward = 0.0f;
		float RootReward = 0.0f;
	};

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
		//
		// NOT the ceiling on delivered torque, as of 2026-08-21. This clamps the
		// COMMANDED torque; the solver then multiplies it by the muscle multiplier
		// (curve shape x authored scalar strength, see FCreatureTopology::
		// DOFExtensionStrength), which is no longer bounded by 1. MuscleProfile_Muto
		// authors scalars over [0.5, 5.0] across 44 of its 67 curve-bearing DOFs, so
		// the strongest joints now deliver up to 5x this figure and the weakest half
		// of it. That is what the tool's strength field was always supposed to mean;
		// it just never reached the solver before. If this needs to be a hard ceiling
		// again, clamp the multiplier in ComputeMuscleMultipliers rather than dropping
		// this value, which would squash the weak joints along with the strong ones.
		float MaxTorquePerDOF = 5.0e7f;

		// Uniform curriculum knob on top of every DOF's authored
		// MuscleActivationThreshold (see ApplyActions below): the EFFECTIVE
		// threshold used each step is DOFMuscleActivationThreshold * this
		// multiplier, not the authored value directly. 1.0 (default)
		// reproduces the authored thresholds exactly. Added because a fresh
		// PPO policy's actions start clustered near 0, so the full authored
		// 0.2 threshold applied from step one can gate off every DOF's small
		// corrective torques before the policy ever has a chance to learn to
		// push past it -- set this to 0 at the start of training (no gating
		// at all, so early exploration can find a working balance policy
		// unobstructed) and ramp it up toward 1.0 over the course of
		// training, once the policy already knows how to move, to
		// reintroduce the modeled recruitment threshold. Live-tunable: see
		// UMutoRLTrainingEnvironment::GatherAgentReward_Implementation's
		// per-step Config refresh.
		float MuscleActivationThresholdMultiplier = 1.0f;

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

		// Reward Settings pane (see SAgentSolverControlPanel.h) -- see
		// ComputeTorsoHeightBonus/ComputeEnergyConsumptionMalus/
		// ComputeMusclesUseMalus below for the exact formulas these feed.
		float RewardHeightTarget = 100.0f;
		float RewardHeightMultiplier = 1.0f;
		float RewardEnergyConsumptionMultiplier = 1.0f;
		float RewardMusclesUseMultiplier = 1.0f;

		// Final multiplier applied to the fully-summed reward (see
		// ComputeReward's own return statement) -- AFTER every term above,
		// not another term alongside them. Lets the whole reward's scale be
		// rescaled in one place (e.g. to compensate for a change elsewhere,
		// or to bring it in line with what feels responsive during PPO
		// training) without re-deriving every individual weight above.
		float GlobalRewardScale = 1.0f;

		// Added AFTER GlobalRewardScale (Reward*Scale + Offset, the standard
		// linear rescale order) -- shifts the whole reward's baseline, e.g.
		// to make "doing nothing" net to a specific target value rather than
		// whatever the summed terms happen to land on.
		float GlobalRewardOffset = 0.0f;

		// ----- Imitation (see CreatureImitation.h) -----

		/** Standing (the original objective) or Imitation. See EObjectiveMode. */
		EObjectiveMode ObjectiveMode = EObjectiveMode::Standing;

		/** Weights and falloff rates for the four imitation terms; only read when ObjectiveMode == Imitation. */
		CreatureImitation::FImitationConfig Imitation;

		/**
		 * Appends [sin(2*pi*phase), cos(2*pi*phase)] to the observation, so
		 * the policy knows WHERE IN THE CLIP it is. Required for imitating an
		 * animation and pointless for a single pose (whose target never
		 * changes, leaving the phase a constant the network would have to
		 * learn to ignore).
		 *
		 * Sine/cosine rather than the raw phase because the raw value jumps
		 * from 1 back to 0 at the loop point, and a discontinuity in an
		 * observation is a discontinuity the policy has to model around;
		 * the (sin, cos) pair is continuous through the wrap.
		 *
		 * CHANGES GetObservationSize, and therefore the network's input shape
		 * -- a policy trained with this off cannot be loaded with it on, or
		 * vice versa. That is why it is a separate flag from ObjectiveMode
		 * rather than implied by it: pose imitation (phase 1) keeps the
		 * existing layout and stays compatible with already-saved networks.
		 */
		bool bAppendPhaseObservation = false;
	};

	/** Observation layout: [TorsoUp.XYZ, TorsoLinVel.XYZ, TorsoAngVel.XYZ, TorsoHeight-Target, (JointPos,JointVel) x NumDOF, (bTouching,NormalForce) x NumContactPoints, and — only when bAppendPhase — (sin,cos) of the reference motion's phase]. Every component below is normalized to roughly [-1,1] before being packed -- see ComputeObservations' own comment for why and how. */
	inline int32 GetObservationSize(const FCreatureTopology& Topo, int32 NumContactPoints, bool bAppendPhase = false)
	{
		return 10 + 2 * Topo.NumDOF + 2 * NumContactPoints + (bAppendPhase ? 2 : 0);
	}

	/**
	 * ULearningAgentsObservations::SpecifyContinuousObservation (see
	 * UMutoRLInteractor::SpecifyAgentObservation_Implementation) is called
	 * with its default Scale=1.0, applied UNIFORMLY across this whole flat
	 * vector -- it cannot rescale individual components differently. Below,
	 * TorsoUp.XYZ is already a unit vector (~[-1,1]) but everything else was
	 * being packed in RAW physical units: linear/angular velocity in cm/s
	 * and rad/s (often tens to hundreds), a height delta in the same raw cm
	 * units as TargetTorsoHeight (~400 in this project), and contact
	 * NormalForce at whatever magnitude this rig's mass/torque regime
	 * produces -- all sitting next to near-unit-scale components in the same
	 * vector, with nothing to bring them onto a comparable scale. Badly-
	 * conditioned inputs like this are a well-known source of unstable
	 * gradients/NaN weights in NN training, independent of anything reward-
	 * related (see AMutoRLTrainingDriver's constructor for the OTHER,
	 * already-diagnosed NaN-collapse mechanism, ActionEntropyWeight/
	 * ActionRegularizationWeight/GradNormMax -- this is a separate
	 * contributor, not a duplicate fix for the same one).
	 *
	 * Each raw component below is now divided by a reference scale chosen to
	 * bring TYPICAL values roughly into [-1,1] (an occasional larger swing,
	 * e.g. during a fall, is fine -- exactly what a network with normalized-
	 * but-unclamped inputs is expected to handle):
	 *  - LinVelocityScale (200 cm/s = 2 m/s) matches Epic's own
	 *    ULearningAgentsObservations::SpecifyVelocityObservation's default
	 *    VelocityScale, chosen for the exact same raw-cm/s-is-too-large
	 *    reason.
	 *  - AngVelocityScale (2*PI rad/s = one full rotation/second) has no
	 *    engine-provided default to match (Learning Agents has no typed
	 *    angular-velocity observation helper) -- picked as a "fast but not
	 *    yet a blowup" reference for this rig.
	 *  - The height delta is divided by Config.TargetTorsoHeight itself,
	 *    not a magic constant -- this rig's own standing height is the
	 *    natural reference scale for "how far off the ground is too far",
	 *    and stays correct if TargetTorsoHeight is ever retuned or this code
	 *    is reused for a differently-sized rig.
	 *  - JointPos (already radians) is divided by PI, the natural reference
	 *    for an angle in radians.
	 *  - JointVel reuses AngVelocityScale (same units as TorsoAngVel).
	 *  - NormalForce is divided by Config.MaxTorquePerDOF -- not a literal
	 *    force/torque unit conversion, but the same reasoning ComputeReward's
	 *    own TorquePenalty/EnergyConsumptionMalus/MusclesUseMalus terms
	 *    already use it for: it's this rig's own characteristic strength
	 *    scale, available without introducing yet another magic number.
	 */
	inline void ComputeObservations(
		const FCreatureBatchState& Batch,
		int32 Env,
		const FEnvConfig& Config,
		const TArray<FContactPointDef>& ContactPoints,
		const TArray<FContactPointState>& ContactStates, // ContactPoints.Num() x NumEnvsForContacts, see ApplyGroundContactForces's OutState layout
		int32 NumEnvsForContacts,
		TArray<float>& OutObservation,
		float Phase = 0.0f)
	{
		constexpr float LinVelocityScale = 200.0f;
		constexpr float AngVelocityScale = 2.0f * PI;

		const FCreatureTopology& Topo = Batch.GetTopology();
		OutObservation.Reset(GetObservationSize(Topo, ContactPoints.Num(), Config.bAppendPhaseObservation));

		const FQuat TorsoRot = Batch.GetBodyRot(0, Env);
		const FVector TorsoUp = TorsoRot.RotateVector(Config.LocalUpAxis);
		const int32 TorsoIdx = Batch.BodyIndex(0, Env);
		const float SafeTargetTorsoHeight = FMath::Max(Config.TargetTorsoHeight, KINDA_SMALL_NUMBER);
		const float SafeMaxTorquePerDOF = FMath::Max(Config.MaxTorquePerDOF, KINDA_SMALL_NUMBER);

		OutObservation.Add((float)TorsoUp.X);
		OutObservation.Add((float)TorsoUp.Y);
		OutObservation.Add((float)TorsoUp.Z);
		OutObservation.Add(Batch.LinVelX[TorsoIdx] / LinVelocityScale);
		OutObservation.Add(Batch.LinVelY[TorsoIdx] / LinVelocityScale);
		OutObservation.Add(Batch.LinVelZ[TorsoIdx] / LinVelocityScale);
		OutObservation.Add(Batch.AngVelX[TorsoIdx] / AngVelocityScale);
		OutObservation.Add(Batch.AngVelY[TorsoIdx] / AngVelocityScale);
		OutObservation.Add(Batch.AngVelZ[TorsoIdx] / AngVelocityScale);
		const float HeightAboveGround = (float)Batch.GetBodyPos(0, Env).Z - Config.GroundZ;
		OutObservation.Add((HeightAboveGround - Config.TargetTorsoHeight) / SafeTargetTorsoHeight);

		for (int32 DOF = 0; DOF < Topo.NumDOF; ++DOF)
		{
			const int32 DOFIdx = Batch.DOFIndex(DOF, Env);
			OutObservation.Add(Batch.JointPos[DOFIdx] / PI);
			OutObservation.Add(Batch.JointVel[DOFIdx] / AngVelocityScale);
		}

		for (int32 PointIdx = 0; PointIdx < ContactPoints.Num(); ++PointIdx)
		{
			const FContactPointState& State = ContactStates[PointIdx * NumEnvsForContacts + Env];
			OutObservation.Add(State.bTouching ? 1.0f : 0.0f);
			OutObservation.Add(State.NormalForce / SafeMaxTorquePerDOF);
		}

		// Where in the reference clip this env currently is. Already in
		// [-1,1] by construction, so it needs none of the normalization the
		// components above do. See FEnvConfig::bAppendPhaseObservation for
		// why this is a (sin, cos) pair and not the raw phase.
		if (Config.bAppendPhaseObservation)
		{
			OutObservation.Add(FMath::Sin(2.0f * PI * Phase));
			OutObservation.Add(FMath::Cos(2.0f * PI * Phase));
		}

		// Same reasoning as ComputeReward's own IsFinite fallback, and the
		// same race: IsTerminated (via IsBodyStateValid) catches a physics
		// blowup and marks the agent Terminated, but Learning Agents' own
		// ULearningAgentsPPOTrainer::ProcessExperience then gathers ONE
		// terminal observation of that SAME corrupted state for the replay
		// buffer (Interactor->GatherObservations, called on the just-completed
		// instances) BEFORE Manager->ResetAgents() actually overwrites it a
		// few lines later in the same function -- so a blowup can still reach
		// here even though IsTerminated already flagged it. Unlike the reward
		// path (a single scalar), a blown-up state can leave ANY subset of
		// these floats non-finite, not just one -- sanitize the whole vector
		// in one pass rather than trying to guard each field individually.
		// ObservationBlowupClamp: the IsFinite sanitize above only catches
		// NaN/Inf -- the same blowup-reaches-here-for-one-step race can just
		// as easily hand a field a huge but still-FINITE value (e.g.
		// NormalForce mid-resonance, before IsBodyStateValid's own NaN/Inf
		// check trips), which sails through untouched otherwise. Every field
		// here is normalized to roughly [-1,1] in normal operation (see this
		// function's own comment above) -- +-10 is generous headroom for any
		// legitimate value while still bounding one bad step from feeding the
		// encoder/critic an outlier several orders of magnitude out of
		// distribution.
		constexpr float ObservationBlowupClamp = 10.0f;
		for (float& Value : OutObservation)
		{
			if (!FMath::IsFinite(Value))
			{
				Value = 0.0f;
			}
			else
			{
				Value = FMath::Clamp(Value, -ObservationBlowupClamp, ObservationBlowupClamp);
			}
		}
	}

	/**
	 * Normalized [-1,1] action per DOF -> Batch.JointTorque, scaled by
	 * Config.MaxTorquePerDOF. A muscle whose |normalized action| falls below
	 * its authored Topo.DOFMuscleActivationThreshold (FMassMuscleDataMuscle::
	 * MuscleActivationThreshold, default 0.2 — see that field's comment)
	 * produces zero torque instead, same idea as a biological motor-unit
	 * recruitment threshold. DOFs with no authored muscle default the
	 * threshold to 0, so they activate at any nonzero command exactly as
	 * before this was added.
	 *
	 * Above the threshold, the remaining action range [threshold,1] is
	 * remapped to [0,1] before scaling by MaxTorquePerDOF, rather than using
	 * the raw action directly -- a muscle crossing its threshold ramps UP
	 * from zero torque, instead of instantly jumping to
	 * threshold*MaxTorquePerDOF. Still a dead zone below the threshold (zero
	 * torque, zero gradient, by design), but continuous at the boundary
	 * itself instead of having a discontinuous jump there.
	 */
	inline void ApplyActions(FCreatureBatchState& Batch, int32 Env, const TArray<float>& NormalizedActions, const FEnvConfig& Config)
	{
		const FCreatureTopology& Topo = Batch.GetTopology();
		check(NormalizedActions.Num() == Topo.NumDOF);
		for (int32 DOF = 0; DOF < Topo.NumDOF; ++DOF)
		{
			const int32 DOFIdx = Batch.DOFIndex(DOF, Env);
			const float ClampedAction = FMath::Clamp(NormalizedActions[DOF], -1.0f, 1.0f);
			const float Threshold = FMath::Max(Topo.DOFMuscleActivationThreshold[DOF] * Config.MuscleActivationThresholdMultiplier, 0.0f);
			const float Magnitude = FMath::Abs(ClampedAction);
			float Torque = 0.0f;
			if (Magnitude >= Threshold && Threshold < 1.0f)
			{
				const float Remapped = (Magnitude - Threshold) / (1.0f - Threshold);
				Torque = FMath::Sign(ClampedAction) * Remapped * Config.MaxTorquePerDOF;
			}
			Batch.JointTorque[DOFIdx] = Torque;
		}
	}

	/**
	 * "How strong can this muscle currently pull, given the DOF's present
	 * angle" -- i.e. the authored Extension/Flexion strength scalar times the
	 * matching curve's value at the current angle, picking Extension vs
	 * Flexion by the SIGN of the commanded torque, exactly the same
	 * computation FCreatureABASolver::ComputeMuscleMultipliers() does
	 * internally to fold into JointTorque during Step() (that one is private
	 * and only reachable from inside the solver, hence this standalone
	 * duplicate -- both read only Batch/Topo, no solver-internal-only state,
	 * so there's nothing it needs that isn't already available here). DOFs
	 * with no authored curve (DOFHasMuscleCurve==false) return 1.0 -- no
	 * angle-dependent effect for anything that doesn't have this data.
	 */
	inline float ComputeMuscleStrengthAtCurrentAngle(const FCreatureTopology& Topo, const FCreatureBatchState& Batch, int32 DOF, int32 DOFIdx)
	{
		if (!Topo.DOFHasMuscleCurve[DOF])
		{
			return 1.0f;
		}

		const FRichCurve* ExtCurve = Topo.DOFExtensionCurve[DOF].GetRichCurveConst();
		const FRichCurve* FlexCurve = Topo.DOFFlexionCurve[DOF].GetRichCurveConst();
		const float ExtStrength = FMath::Max(Topo.DOFExtensionStrength[DOF], 0.0f);
		const float FlexStrength = FMath::Max(Topo.DOFFlexionStrength[DOF], 0.0f);
		const float MinDeg = Topo.DOFRangeMinDeg[DOF];
		const float MaxDeg = Topo.DOFRangeMaxDeg[DOF]; // already unwrapped > MinDeg

		const float AngleDeg = FMath::RadiansToDegrees(Batch.JointPos[DOFIdx]);
		float Wrapped = FMath::Fmod(AngleDeg - MinDeg, 360.0f);
		if (Wrapped < 0.0f) Wrapped += 360.0f;
		const float T = (MaxDeg > MinDeg) ? FMath::Clamp(Wrapped / (MaxDeg - MinDeg), 0.0f, 1.0f) : 0.0f;

		const float Torque = Batch.JointTorque[DOFIdx];
		const float CurveVal = (Torque >= 0.0f)
			? ExtStrength * (ExtCurve ? ExtCurve->Eval(T) : 1.0f)
			: FlexStrength * (FlexCurve ? FlexCurve->Eval(T) : 1.0f);
		return FMath::Max(CurveVal, 0.0f);
	}

	/**
	 * Reward-Settings-pane "Torso Height" bonus (see
	 * AMutoRLTrainingDriver::RewardHeightTarget/RewardHeightMultiplier) --
	 * exponential falloff, always positive, maxed out (== RewardMultiplier)
	 * exactly at TargetHeight: RewardMultiplier * exp(-|Height-Target|/Target).
	 * Same falloff shape ComputeReward's own BalanceTerm already uses.
	 * Height is Batch.GetBodyPos(0,Env).Z directly (not offset by
	 * Config.GroundZ) -- consistent with GroundZ always being 0.0f in
	 * practice (see AMutoRLTrainingDriver::StartTraining).
	 */
	inline float ComputeTorsoHeightBonus(const FCreatureBatchState& Batch, int32 Env, float TargetHeight, float RewardMultiplier)
	{
		const float CurrentHeight = (float)Batch.GetBodyPos(0, Env).Z;
		const float SafeTarget = FMath::Max(TargetHeight, KINDA_SMALL_NUMBER);
		return RewardMultiplier * FMath::Exp(-FMath::Abs(CurrentHeight - TargetHeight) / SafeTarget);
	}

	/**
	 * Reward-Settings-pane "Energy Consumption" malus (see
	 * AMutoRLTrainingDriver::RewardEnergyConsumptionMultiplier): sum, across
	 * DOFs, of the commanded torque's magnitude as a FRACTION of
	 * MaxTorquePerDOF ("the forces applied by the muscles" -- Batch.JointTorque
	 * is this solver's per-DOF torque/force channel), times
	 * EnergyConsumptionMultiplier -- same [0, NumDOF] scale as
	 * ComputeMusclesUseMalus below, and normalized against MaxTorquePerDOF for
	 * the exact same reason TorquePenalty is in ComputeReward (see that
	 * term's comment): this used to divide the RAW torque sum by a fixed
	 * /10000, which was in-range back when MaxTorquePerDOF was the 5.0
	 * placeholder value but silently became ~10,000,000x too small once
	 * MaxTorquePerDOF was rescaled to 5.0e7 (see MaxTorquePerDOF's own
	 * comment) -- e.g. a rig using ~65% of its torque budget across ~68 DOF
	 * produced a malus around 220,000, versus every other reward term's
	 * combined ~[-1,2.5] range, making this single term the entire reward
	 * signal and drowning out any actual balance/uprightness gradient.
	 */
	inline float ComputeEnergyConsumptionMalus(const FCreatureBatchState& Batch, int32 Env, float MaxTorquePerDOF, float EnergyConsumptionMultiplier)
	{
		const FCreatureTopology& Topo = Batch.GetTopology();
		float TotalNormalizedForce = 0.0f;
		if (MaxTorquePerDOF > KINDA_SMALL_NUMBER)
		{
			for (int32 DOF = 0; DOF < Topo.NumDOF; ++DOF)
			{
				TotalNormalizedForce += FMath::Abs(Batch.JointTorque[Batch.DOFIndex(DOF, Env)]) / MaxTorquePerDOF;
			}
		}
		return TotalNormalizedForce * EnergyConsumptionMultiplier;
	}

	/**
	 * Reward-Settings-pane "Muscles Use" malus (see
	 * AMutoRLTrainingDriver::RewardMusclesUseMultiplier). Formula as
	 * specified: for each muscle, "how much it's activated in % of its max
	 * power AT THE ANGLE it's currently at" -- commanded torque magnitude
	 * divided by (MaxTorquePerDOF * the angle-dependent strength multiplier
	 * from ComputeMuscleStrengthAtCurrentAngle above, i.e. the actual torque
	 * ceiling right now, not the topology-wide MaxTorquePerDOF alone) --
	 * summed across DOFs, times MusclesUseMultiplier.
	 */
	inline float ComputeMusclesUseMalus(const FCreatureBatchState& Batch, int32 Env, float MaxTorquePerDOF, float MusclesUseMultiplier)
	{
		const FCreatureTopology& Topo = Batch.GetTopology();
		float TotalActivation = 0.0f;
		for (int32 DOF = 0; DOF < Topo.NumDOF; ++DOF)
		{
			const int32 DOFIdx = Batch.DOFIndex(DOF, Env);
			const float MaxPowerAtAngle = MaxTorquePerDOF * ComputeMuscleStrengthAtCurrentAngle(Topo, Batch, DOF, DOFIdx);
			if (MaxPowerAtAngle > KINDA_SMALL_NUMBER)
			{
				TotalActivation += FMath::Abs(Batch.JointTorque[DOFIdx]) / MaxPowerAtAngle;
			}
		}
		return TotalActivation * MusclesUseMultiplier;
	}

	/**
	 * Standing/balance reward: alive bonus + how upright the torso is +
	 * how close the contact points' force-weighted centroid (center of
	 * pressure) sits under the torso horizontally (a standard postural
	 * balance proxy — supersedes the vaguer "bias weight toward the ankle"
	 * phrasing from the original handoff, which predates the ground contact
	 * model's CanTouchGround-driven redesign and no longer maps onto a
	 * single distinguished "ankle" point per limb) - torque-squared penalty
	 * to discourage needless effort, plus the Reward-Settings-pane's torso-
	 * height bonus and muscle-penalty (energy/muscles-use) malus terms (see
	 * ComputeTorsoHeightBonus/ComputeEnergyConsumptionMalus/
	 * ComputeMusclesUseMalus above). The three optional Out* pointers let a
	 * caller (SAgentSolverControlPanel's visualization graphs, via
	 * AMutoRLTrainingDriver::LastTorsoHeightBonus/LastEnergyConsumptionMalus/
	 * LastMusclesUseMalus) read the individual components without
	 * recomputing them separately and risking drift from this function's own
	 * math.
	 */
	inline float ComputeReward(
		const FCreatureBatchState& Batch,
		int32 Env,
		const FEnvConfig& Config,
		const TArray<FContactPointDef>& ContactPoints,
		const TArray<FContactPointState>& ContactStates,
		int32 NumEnvsForContacts,
		float* OutTorsoHeightBonus = nullptr,
		float* OutEnergyConsumptionMalus = nullptr,
		float* OutMusclesUseMalus = nullptr,
		const FImitationTarget* ImitationTarget = nullptr,
		FImitationBreakdown* OutImitation = nullptr)
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

		const float TorsoHeightBonus = ComputeTorsoHeightBonus(Batch, Env, Config.RewardHeightTarget, Config.RewardHeightMultiplier);
		const float EnergyConsumptionMalus = ComputeEnergyConsumptionMalus(Batch, Env, Config.MaxTorquePerDOF, Config.RewardEnergyConsumptionMultiplier);
		const float MusclesUseMalus = ComputeMusclesUseMalus(Batch, Env, Config.MaxTorquePerDOF, Config.RewardMusclesUseMultiplier);
		if (OutTorsoHeightBonus) { *OutTorsoHeightBonus = TorsoHeightBonus; }
		if (OutEnergyConsumptionMalus) { *OutEnergyConsumptionMalus = EnergyConsumptionMalus; }
		if (OutMusclesUseMalus) { *OutMusclesUseMalus = MusclesUseMalus; }

		// The two objectives share their COSTS (alive bonus, torque penalty,
		// energy/muscle maluses) and differ only in the task term. See
		// EObjectiveMode for why this is exclusive rather than additive.
		//
		// Falls back to the standing terms if imitation is selected but no
		// reference was supplied -- a missing/failed bake should degrade to
		// the objective that has always worked, not to a reward of zero that
		// looks like a broken policy.
		float TaskTerm = 0.0f;
		const bool bImitating = (Config.ObjectiveMode == EObjectiveMode::Imitation)
			&& ImitationTarget != nullptr && ImitationTarget->Frame != nullptr;

		if (bImitating)
		{
			static const TArray<int32> EmptyEndEffectors;
			FImitationBreakdown Breakdown;
			TaskTerm = CreatureImitation::ComputeImitationReward(
				Batch, Env, *ImitationTarget->Frame,
				ImitationTarget->EndEffectorBodies ? *ImitationTarget->EndEffectorBodies : EmptyEndEffectors,
				Config.GroundZ, ImitationTarget->RestTorsoHeight, Config.TargetTorsoHeight,
				Config.Imitation,
				&Breakdown.PoseReward, &Breakdown.VelocityReward,
				&Breakdown.EndEffectorReward, &Breakdown.RootReward);
			if (OutImitation) { *OutImitation = Breakdown; }
		}
		else
		{
			TaskTerm = Config.UprightWeight * UprightDot
				+ Config.BalanceWeight * BalanceTerm
				+ TorsoHeightBonus;
		}

		const float RawReward = Config.AliveBonus
			+ TaskTerm
			- Config.TorquePenaltyWeight * TorquePenalty
			- EnergyConsumptionMalus
			- MusclesUseMalus;

		// A numerical blowup upstream (see IsBodyStateValid) can still reach
		// here for the one step where the corrupted state's reward/completion
		// are gathered together, BEFORE IsTerminated below has a chance to
		// force the reset — never hand the trainer a NaN/Inf reward sample
		// for that step, NOR a finite-but-astronomical one: TorquePenalty/
		// EnergyConsumptionMalus/MusclesUseMalus are unbounded sums over
		// Batch.JointTorque, so a blowup can hand them a huge but still-finite
		// value that the IsFinite check alone doesn't catch (confirmed via
		// TensorBoard: a one-frame ~-5e6 return spike, finite the whole way,
		// landing right when grads/critic and grads/policy also spike).
		// Clamped on this RAW, pre-Scale/Offset sum (not the final Reward
		// below) so the bound stays meaningful regardless of the user's own
		// GlobalRewardScale/Offset tuning — normal RawReward here is roughly
		// [-0.1, 2.5], so +-10 is generous headroom that never clips a real
		// reward. Falls back to 0 for non-finite (in-distribution), not a
		// large negative "punishment" value — a blowup is a numerical
		// artifact of THIS simulation, not something the agent chose, and
		// (with bUseGradNormMaxClipping now on, see AMutoRLTrainingDriver's
		// constructor) an earlier, larger fallback here was a real suspect for
		// repeatedly injecting out-of-distribution reward outliers into PPO's
		// advantage/critic statistics across however many of the (256, by
		// default) parallel envs blow up at any given time — plausibly
		// contributing to the exact kind of rare gradient-explosion event
		// that produces NaN network weights after an otherwise-fine training
		// run.
		constexpr float RawRewardBlowupClamp = 10.0f;
		const float ClampedRawReward = FMath::IsFinite(RawReward) ? FMath::Clamp(RawReward, -RawRewardBlowupClamp, RawRewardBlowupClamp) : 0.0f;

		const float Reward = Config.GlobalRewardScale * ClampedRawReward + Config.GlobalRewardOffset;
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
		const TArray<FContactPointState>* ContactStates = nullptr, int32 NumContactPoints = 0, int32 NumEnvsForContacts = 1,
		const FImitationTarget* ImitationTarget = nullptr)
	{
		// Blowup guards are unconditional in BOTH objectives -- they are not
		// task logic, they are the only thing standing between a NaN and a
		// hard assert inside Learning Agents (see IsBodyStateValid).
		if (!IsBodyStateValid(Batch, Env))
		{
			return true;
		}
		if (ContactStates && !IsContactStateValid(*ContactStates, NumContactPoints, Env, NumEnvsForContacts))
		{
			return true;
		}

		if (Config.ObjectiveMode == EObjectiveMode::Imitation)
		{
			// Imitation's analogue of "fallen over": once the pose has drifted
			// far enough from the reference there is nothing further to learn
			// from this episode, and ending it is what keeps the replay buffer
			// populated with states the reference motion actually visits.
			if (Config.Imitation.MaxPoseErrorRad > 0.0f && ImitationTarget && ImitationTarget->Frame)
			{
				if (CreatureImitation::ComputeMeanPoseErrorRad(Batch, Env, *ImitationTarget->Frame) > Config.Imitation.MaxPoseErrorRad)
				{
					return true;
				}
			}
			// A reference motion is not necessarily an upright stand -- a
			// crawl, a roll or a get-up would trip these thresholds on frame
			// one, ending every episode before the policy sees any of it.
			if (!Config.Imitation.bTerminateOnUprightAndHeight)
			{
				return false;
			}
		}

		const FQuat TorsoRot = Batch.GetBodyRot(0, Env);
		const float UprightDot = (float)FVector::DotProduct(TorsoRot.RotateVector(Config.LocalUpAxis), FVector::UpVector);
		const float HeightAboveGround = (float)Batch.GetBodyPos(0, Env).Z - Config.GroundZ;
		return UprightDot < Config.MinUprightDot || HeightAboveGround < Config.MinHeightFraction * Config.TargetTorsoHeight;
	}

	/**
	 * Per-episode domain randomization, drawn fresh by ResetEnv every time an
	 * env restarts. Randomizes the three things FCreatureBatchState already
	 * carries per-env and the solver already consumes in Pass 2 of both step
	 * variants: per-limb muscle strength, per-limb loss (a limb whose muscles
	 * produce no torque at all -- the joint stays intact and still swings
	 * passively, it just goes limp), and extra mass carried at the torso.
	 *
	 * DEFAULTS ARE NEUTRAL AND bEnabled IS FALSE, so a caller that does not opt
	 * in gets exactly the previous behavior: every env identical, strength 1,
	 * all limbs active, no carried mass. When disabled ResetEnv still writes
	 * those neutral values rather than skipping the call, because the three
	 * arrays persist across episodes -- see FCreatureBatchState::RandomizeEnv.
	 *
	 * Ranges are deliberately expressed as scale factors, not absolute units,
	 * with the single exception of MaxCarriedMass (kg, matching BodyMass).
	 */
	struct FDomainRandomization
	{
		bool bEnabled = false;

		/** Per-limb muscle strength multiplier, drawn uniformly per limb per episode. */
		float MinLimbStrengthScale = 1.0f;
		float MaxLimbStrengthScale = 1.0f;

		/** Probability that any given limb goes limp for the whole episode. 0 = never. */
		float LimbLossChance = 0.0f;

		/** Upper bound of the uniform draw for extra torso-carried mass, in kg. */
		float MaxCarriedMass = 0.0f;
	};

	/**
	 * Resets one env to a standing pose (StandingTorsoPos/Rot, all joints at
	 * their rest/zero position, zero velocities), with small noise on the
	 * torso's horizontal position and orientation, and a fresh draw of the
	 * per-episode domain randomization (see FDomainRandomization -- limb
	 * strength, limb loss, carried mass). Joint "rest" here means JointPos=0 for
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
		float AngleNoiseRad,
		const FDomainRandomization& Randomization = FDomainRandomization())
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

		// Drawn AFTER the pose noise above so both come from the same stream in a
		// fixed order -- reseeding ResetStream reproduces an entire run. Called
		// unconditionally: with randomization off these are the neutral values,
		// which RESTORES any env that was randomized under an earlier setting
		// (the per-limb arrays are episode-persistent state, not per-step inputs).
		Batch.RandomizeEnv(Env, Stream,
			Randomization.bEnabled ? Randomization.MinLimbStrengthScale : 1.0f,
			Randomization.bEnabled ? Randomization.MaxLimbStrengthScale : 1.0f,
			Randomization.bEnabled ? Randomization.LimbLossChance : 0.0f,
			Randomization.bEnabled ? Randomization.MaxCarriedMass : 0.0f);
	}
}
