#pragma once

// Ground contact for AgentSolver's ABA creature. No new solver math: contact
// is a penalty force (spring-damper + clamped Coulomb friction) applied
// through FCreatureBatchState::ApplyForceAtPoint, which already does exactly
// "accumulate an external wrench at a world point on a body" -- the solver's
// bias pass folds it in every step like any other external force. This
// matches the project's existing "self-consistent dynamics over exact
// physical accuracy" stance (see CreatureBatchSolver.h's Coriolis-omission
// note) rather than building a full LCP/constraint contact solver.
//
// Ground is an infinite flat plane (GroundZ + GroundNormal in FContactParams)
// -- a heightfield/uneven terrain is a possible future extension, not needed
// yet.

#include "CoreMinimal.h"
#include "CreatureBatchState.h"
#include "CreatureBatchSolver.h" // velocity-level contact needs the impulse machinery

#if WITH_EDITOR
// Only needs the mass-profile asset type now -- BuildMutoContactPoints reads
// CanTouchGround/LimbIndex straight from the already-built FCreatureTopology,
// no MutoTopology::-namespace helpers required anymore.
#include "UMassMuscleProfileAsset.h"
#endif

namespace CreatureGroundContact
{
	/** One contact point: a fixed offset (in BodyIndex's own rest frame) tracked every step. */
	struct FContactPointDef
	{
		int32 BodyIndex = INDEX_NONE;
		FVector LocalOffset = FVector::ZeroVector; // sphere CENTER, in BodyIndex's own rest frame -- see Radius
		FName DebugName;
		int32 LimbIndex = INDEX_NONE; // which physical limb this belongs to (0..7 for Muto) -- for future reward code to group by limb

		// Collision sphere radius at LocalOffset (see FMassMuscleDataMass::
		// Radius) -- the actual ground-contact surface is this far below the
		// sphere's center along the ground normal, not the center itself.
		// Zero reproduces the old "point contact at LocalOffset exactly"
		// behavior (e.g. every synthetic test's hand-built points).
		float Radius = 0.0f;

		// Capsule half-height (see FMassMuscleDataMass::CapsuleHalfHeight):
		// LocalOffset is the capsule's fixed END cap (the tip); the START cap
		// sits CapsuleHalfHeight*2 back toward the body's own origin, along
		// the BodyIndex->LocalOffset axis. Zero (default) collapses the
		// capsule to a single sphere at LocalOffset -- the original behavior,
		// and what every synthetic test's hand-built point still gets.
		float CapsuleHalfHeight = 0.0f;
	};

	/**
	 * One candidate collision pair between two bodies with authored collision
	 * geometry -- limb-vs-limb, limb-vs-spine, or spine-vs-spine, all
	 * uniformly (2026-08-16: the spine/head/jaw chain became independently
	 * articulated, each link its own real ABA body with its own BodyRadius,
	 * so it no longer needs the separate fused-torso-segment machinery this
	 * struct used to carry -- see BuildMutoLimbCollisionPairs for the
	 * exclusion rules that keep this from producing nonsense pairs). Built
	 * once from topology, not per-step -- per-step broadphase (are these two
	 * bodies' capsules actually overlapping right now) is done fresh every
	 * call in ResolveGroundContactImpulses, since poses change every
	 * substep.
	 */
	struct FLimbPairDef
	{
		int32 BodyA = INDEX_NONE;
		int32 BodyB = INDEX_NONE;
	};

	/**
	 * Per-point-per-env contact output, consumed by the RL observation and reward
	 * code (see CreatureRLEnvironment.h). NormalForce is reported as an
	 * equivalent average force even though the solver works in impulses, so
	 * downstream code reads the same units it always did.
	 */
	struct FContactPointState
	{
		float NormalForce = 0.0f;
		bool bTouching = false;
	};

	/**
	 * VELOCITY-LEVEL (IMPULSE) GROUND CONTACT. Call AFTER Solver.Step().
	 *
	 * This is the only contact model. A penalty spring-damper lived here until
	 * 2026-08-14; it computed a FORCE from penetration and let the integrator
	 * apply it over the step. That is only conditionally stable
	 * (DamperK*Dt/m_eff < 2), it needs a stiffness tuned per rig, and past the
	 * limit it hands back more energy than it absorbs -- which is what remained
	 * after every structural solver bug was fixed (SOLVER_DEBUG_LOG.md 013).
	 *
	 * This instead asks a question with an exact answer: what impulse makes the
	 * contact point stop approaching the ground? For an articulated body that
	 * requires the ARTICULATED effective mass at the contact -- the whole chain's
	 * reflected inertia, not the foot's own -- which
	 * FCreatureABASolver::ImpulseResponseAtPoint supplies by construction. No
	 * stiffness, no damping ratio, and no dependence on the timestep.
	 *
	 * Penetration is corrected by feeding back a small fraction (Beta) of the
	 * depth as a target separating velocity, the standard Baumgarte term. Slop
	 * leaves a little penetration uncorrected so resting contacts do not jitter.
	 *
	 * Friction is a second impulse along the tangential direction, solved with
	 * its own effective mass and clamped to the Coulomb cone of the normal
	 * impulse just applied.
	 *
	 * Iterations resolve contacts sequentially (Gauss-Seidel): with several
	 * points sharing one chain, each impulse changes the others' velocities, so
	 * a few passes let them agree. 4-8 is usually plenty.
	 */
	struct FImpulseContactParams
	{
		float GroundZ = 0.0f;
		FVector GroundNormal = FVector::UpVector;

		/**
		 * SOFT CONSTRAINT parameters, replacing a raw Baumgarte Beta.
		 *
		 * A contact is specified as a damped harmonic oscillator -- how fast it
		 * should push overlap out (Hertz) and how it should approach that
		 * (DampingRatio) -- instead of by a raw stiffness that has to be re-derived
		 * every time mass changes. That indirection is the point: Hertz and
		 * DampingRatio are invariant to the creature's mass and to the timestep,
		 * whereas SpringK had to be rescaled by 143x when the masses were
		 * authored (SOLVER_DEBUG_LOG.md entry 017) and silently produced a
		 * 6047-unit sag until it was.
		 *
		 * Beyond the bias rate, the soft formulation adds an impulse-RELAXATION
		 * term, which is the regularization a pure Baumgarte scheme lacks: the
		 * accumulated impulse is scaled back each iteration rather than being
		 * free to grow, so the constraint yields slightly instead of fighting to
		 * the death. This is Box2D v3's "soft step" and is the same idea as
		 * MuJoCo's diagonal regularizer R > 0 (the limit R -> 0 recovers a hard
		 * constraint, and MuJoCo's docs note the system goes "unstable and noisy"
		 * as stiffness rises toward it).
		 *
		 * ContactHertz is clamped internally to a quarter of the step rate -- a
		 * constraint cannot usefully be stiffer than the timestep can resolve,
		 * and asking for it is how the old formulation blew up.
		 */
		float ContactHertz = 30.0f;
		float DampingRatio = 10.0f;

		/**
		 * CFM-style regularization (2026-08-17): added directly to the
		 * effective INVERSE mass denominator in the Delta formula (Delta =
		 * -MassScale*(VN+Bias)/(InvMassN+Cfm) - ...), NOT derived from
		 * Hertz/DampingRatio. This is distinct from the existing
		 * ImpulseScale regularization above (which the codebase already
		 * documents as MuJoCo-regularizer-equivalent but which only touches
		 * the bias/position-error term) -- Cfm instead directly bounds how
		 * large a SINGLE row's corrective impulse can get, independent of
		 * penetration depth or approach velocity, exactly like Bullet's CFM
		 * added to the diagonal of J*M^-1*J^T. 0 = off, matches all
		 * pre-existing behavior exactly. Added specifically for the case
		 * where a body's ground-contact row and its OWN joint-limit row
		 * (FJointLimitParams::Cfm) are both strongly active at once (e.g. an
		 * ankle/elbow folded onto its limit while also bearing ground load)
		 * -- two independently-regularized-by-Hertz rows can still diverge
		 * against each other since neither's static effective-mass snapshot
		 * accounts for the other; a floor on each row's own max response is
		 * what actually bounds that.
		 */
		float Cfm = 0.0f;

		/**
		 * SOR (successive over-relaxation) factor for the NORMAL row's
		 * accumulated-impulse update only (2026-08-17) -- NewTotal =
		 * Accumulated + Relaxation*Delta instead of Accumulated + Delta.
		 * 1.0 = full correction each iteration, matching all pre-existing
		 * behavior exactly. Values < 1 (0.3-0.5 is the usual starting
		 * range) apply only a FRACTION of each iteration's computed
		 * correction, directly damping the alternating-overcorrection
		 * oscillation that two strongly-coupled rows (e.g. this row and
		 * FJointLimitParams's, both acting on one folded-and-grounded
		 * joint) produce under naive PGS -- complementary to Cfm above, not
		 * a substitute: Cfm bounds how BIG one row's response can get, SOR
		 * bounds how FAST the coupled system can swing between two rows'
		 * competing corrections. Deliberately NOT applied to friction
		 * (Cache.TangentImpulse1/2) -- not implicated in this instability,
		 * and friction is already Coulomb-cone-bounded by the (now also
		 * relaxed) normal impulse.
		 */
		float Relaxation = 1.0f;

		/** Penetration left uncorrected, so resting contacts settle instead of buzzing. */
		float Slop = 0.5f;
		/** 0 = fully inelastic. Locomotion wants ~0. */
		float Restitution = 0.0f;
		float FrictionCoefficient = 0.8f;
		int32 Iterations = 6;
		/**
		 * Extra passes run with the stabilization term OFF, after the biased ones.
		 *
		 * DEFAULT 0, and that is deliberate — it depends on step ORDERING.
		 * Box2D's relax phase runs AFTER position integration: the bias velocity
		 * has already pushed the body out of penetration by the time relaxation
		 * removes it, so it does useful work and only its leftover energy is
		 * taken back. Our contact resolves after Step(), which has ALREADY
		 * integrated position, so a relax pass strips the bias velocity while it
		 * has moved nothing at all — the correction is created and destroyed in
		 * the same breath, and penetration never recovers.
		 *
		 * Measured on the single-body drop test: relax=2 settles 8.08 units deep
		 * and stays there; relax=0 settles at the expected ~1 unit. See
		 * SOLVER_DEBUG_LOG.md entry 021.
		 *
		 * Set this above zero only if contact is moved to resolve BETWEEN the
		 * velocity and position integrations, which is the ordering it assumes.
		 */
		int32 RelaxIterations = 0;
		/** Upper bound on the per-step normal impulse (0 = none). A safety net, not a tuning knob. */
		float MaxNormalImpulse = 0.0f;

		/**
		 * Ceiling on the Baumgarte separating velocity, in cm/s. REQUIRED, not
		 * optional: the bias term is (Beta/Dt) * penetration, so a point that has
		 * ended up deep produces an enormous target velocity -- at Beta=0.2,
		 * Dt=1/60 and 600 units of depth that is 7188 cm/s, and the solver
		 * faithfully delivers the impulse to achieve it, firing the limb away.
		 * Clamping makes deep contacts recover firmly but at a bounded rate,
		 * which is what every production impulse solver does (Box2D's
		 * maxLinearCorrection, Bullet's ERP cap).
		 */
		float MaxBiasVelocity = 100.0f;

		/**
		 * Solve every constraint against the ASSEMBLED inverse constraint inertia
		 * A = J M^-1 J^T + R (MuJoCo's formulation) instead of relaxing each row
		 * against the bodies one at a time. See FGlobalRow's comment for the full
		 * reasoning; the short version is that SOLVER_DEBUG_LOG.md entry 024's
		 * iteration sweep refuted convergence rate as the missing ingredient,
		 * which indicts the solver class rather than its tuning.
		 *
		 * DEFAULT ON. The per-row path is kept and selectable so the two can be
		 * compared rather than argued about -- the same choice entry 024 made for
		 * bSolveJointLimitsAsConstraints.
		 */
		bool bUseGlobalSolve = true;

		/**
		 * Sweeps for the global solve. Separate from `Iterations` because the two
		 * costs are not comparable: a global sweep is O(rows^2) of plain arithmetic
		 * with NO tree traversals, where a per-row sweep costs one full
		 * backward+root+forward pass over every body per row. Roughly an order of
		 * magnitude cheaper per sweep at this rig's size, so the budget entry 024
		 * could not afford to raise is affordable here.
		 *
		 * <= 0 falls back to `Iterations`, so a caller that only knows about the
		 * older field keeps working.
		 */
		int32 GlobalIterations = 64;
	};

	/**
	 * Accumulated impulses persisted ACROSS steps, for warm starting.
	 *
	 * Slot = (PointIdx * 2 + EndIdx) * NumEnvs + Env. Our contact points are
	 * fixed local offsets on named bodies rather than the output of a collision
	 * narrowphase, so that index IS a stable persistent contact ID -- no
	 * feature matching or proximity heuristic needed, which is the fiddly part
	 * of warm starting in a general engine.
	 *
	 * Owned by the caller so it survives between steps.
	 */
	/**
	 * JOINT LIMITS AS CONSTRAINT ROWS, solved in the same sequential-impulse
	 * loop as contact -- the way MuJoCo, PhysX articulations and Box2D v3 all
	 * do it, and the fix for SOLVER_DEBUG_LOG.md entry 023.
	 *
	 * What it replaces: `ClampJointLimits` is a post-integration POSITION clamp
	 * that snaps JointPos to the boundary and zeroes JointVel. Nothing in the
	 * impulse path knows a joint is limited, so a leg folded solid onto its
	 * stops is still modelled as freely folding. Measured consequence: the
	 * articulated effective mass at a foot went 14 kg -> 17 kg when six leg
	 * joints locked, under a 6170 kg creature. The contact sized its impulse
	 * for 17 kg and the feet went through the floor.
	 *
	 * Solving the limit as a row instead means the contact impulse and the
	 * limit impulse see each other every iteration: contact pushes the foot up,
	 * the leg tries to fold, the limit row resists, and the load propagates to
	 * the torso through the same coupled solve. The per-row effective mass is
	 * still the free-joint one -- what changes is that iteration now converges
	 * toward the braced answer instead of toward the folded one.
	 *
	 * The clamp is deliberately LEFT IN PLACE as a position-level backstop, the
	 * same role Slop plays for contact. If it turns out to fight these rows
	 * (it still zeroes JointVel), that is a separate change with its own
	 * measurement.
	 *
	 * Ball joints are NOT handled here yet: their limit is a cone on the
	 * rotation vector's magnitude rather than a per-DOF range, so it is a
	 * different row type. They keep the existing clamp. Recorded rather than
	 * silently skipped -- the hips are ball joints, and they carry load.
	 */
	struct FJointLimitParams
	{
		bool bEnabled = true;

		/** Stiffness of the limit stop. Higher than contact by default: hitting the end of a joint should feel harder than landing on the ground, and unlike ground there is no penetration to tolerate. */
		float Hertz = 60.0f;

		/** Overdamped, like contact -- a limit that rings is worse than one that is slightly soft. */
		float DampingRatio = 5.0f;

		/** CFM-style regularization on this row's own effective-mass denominator. See FImpulseContactParams::Cfm's comment for the full reasoning -- same mechanism, applied to InvMass instead of InvMassN. 0 = off. Shared by both the revolute (FLimitRow) and ball-joint (FBallLimitRow) rows below. */
		float Cfm = 0.0f;

		/** SOR under-relaxation on this row's accumulated-impulse update. See FImpulseContactParams::Relaxation's comment. 1.0 = off. Shared by both the revolute and ball-joint rows below. */
		float Relaxation = 1.0f;

		/** Overshoot tolerated before the row pushes back, in degrees. The joint-space analogue of contact Slop: without it a resting joint at its stop buzzes. */
		float SlopDeg = 0.25f;

		/**
		 * Engage the row this far BEFORE the limit, in degrees.
		 *
		 * With the speculative term below, this greatly reduces tunnelling but
		 * does NOT eliminate it: measured 1522 lap skips with the position
		 * clamp alone, 571 with these rows (entry 024). Two known holes, both
		 * real rather than tuning issues:
		 *   - rows are built ONCE per substep from the joint speed at that
		 *     moment, so an impulse applied later in the iteration loop can
		 *     leave a joint moving faster than the row was sized for;
		 *   - ball joints (the hips, which carry load) have no row type here at
		 *     all and still rely on the clamp.
		 * Do not describe this as tunnel-proof.
		 */
		float MarginDeg = 3.0f;

		/** Cap on the push-out velocity for an already-violated limit, in deg/s. Same role as FImpulseContactParams::MaxBiasVelocity, and added for the same reason: an unbounded Baumgarte term was a real divergence source here before. */
		float MaxBiasVelocityDeg = 720.0f;
	};

	/**
	 * Tunables for limb-vs-limb collision, resolved in the SAME Gauss-Seidel
	 * loop as ground contact and joint limits (see
	 * ResolveGroundContactImpulses) -- a limb braced against the ground AND
	 * against another limb needs both constraints to see each other's effect
	 * every iteration, the same reason joint limits were folded into that
	 * loop rather than solved as a separate pass.
	 *
	 * Deliberately its own tunables rather than reusing
	 * FImpulseContactParams: limb-limb stiffness is not expected to need the
	 * same values as ground stiffness, same reasoning FJointLimitParams
	 * already applies by not reusing ContactHertz.
	 */
	struct FLimbCollisionParams
	{
		bool bEnabled = true;
		float Hertz = 30.0f;
		float DampingRatio = 10.0f;
		/** CFM-style regularization on this row's own effective-mass denominator. See FImpulseContactParams::Cfm's comment. 0 = off. */
		float Cfm = 0.0f;
		/** SOR under-relaxation on this row's accumulated-impulse update. See FImpulseContactParams::Relaxation's comment. 1.0 = off. */
		float Relaxation = 1.0f;
		/** Penetration left uncorrected, so two resting limbs settle instead of buzzing. */
		float Slop = 0.5f;
		/** 0 = fully inelastic. */
		float Restitution = 0.0f;
		float FrictionCoefficient = 0.8f;
		/** Upper bound on the per-step normal impulse (0 = none). */
		float MaxNormalImpulse = 0.0f;
		/** Ceiling on the Baumgarte separating velocity, in cm/s -- same role as FImpulseContactParams::MaxBiasVelocity. */
		float MaxBiasVelocity = 100.0f;
	};

	/**
	 * ============ GLOBAL CONSTRAINT SOLVE (MuJoCo's formulation) ============
	 *
	 * Assembles the full inverse constraint inertia
	 *
	 *     A = J M^-1 J^T + R
	 *
	 * over EVERY active row in one env -- ground contacts, their friction,
	 * revolute joint limits, ball-joint cone limits, limb-vs-limb pairs and their
	 * friction, all in one matrix -- and solves the resulting box/cone-constrained
	 * problem on that matrix. This is what MuJoCo does, and it is the change
	 * SOLVER_DEBUG_LOG.md entry 024 named as the correct fix and deferred as "a
	 * rewrite".
	 *
	 * WHY, precisely. The per-row path (still present below, still selectable)
	 * recovers only each row's own DIAGONAL, one whole-tree impulse query at a
	 * time, and never computes the off-diagonal coupling at all. It discovers
	 * coupling only implicitly, by applying one row's impulse to the bodies and
	 * letting the next row observe the changed velocity -- which is the
	 * definition of Gauss-Seidel and is the only solver that data layout can
	 * support. Entry 024's iteration sweep is what that costs: 8/16/32/64/128
	 * iterations gave peak penetrations of 71871, 7324, 5474414, 70351, 1288 and
	 * divergence at substeps 383, 383, 535, 487, 495 -- everything diverging,
	 * non-monotonic across a 16x range. That is the canonical signature of
	 * Gauss-Seidel on a strongly coupled system: each sweep order lands on a
	 * different fixed-point trajectory and none of them is the solution. The
	 * entry's conclusion is the one to keep: convergence RATE is not the missing
	 * ingredient, so "more iterations" and every variant of it is refuted.
	 *
	 * WHAT THIS BUYS, concretely, and it is not just "a better algorithm":
	 *
	 *  1. ITERATIONS BECOME FREE BY COMPARISON. Once A is assembled the sweep is
	 *     pure O(rows^2) arithmetic with ZERO tree traversals -- the residual is
	 *     maintained in constraint space (Cdot = Cdot0 + A*lambda), so Batch is
	 *     not touched until the very end. The per-row path pays one full
	 *     backward+root+forward pass over every body PER ROW PER ITERATION. At
	 *     this rig's size a dense sweep is roughly an order of magnitude cheaper
	 *     than a tree sweep, so the iteration budget that entry 024 could not
	 *     afford to raise is now affordable by default.
	 *  2. ASSEMBLY IS ~COST-NEUTRAL. Each row needs exactly one response column,
	 *     and the per-row path already spent one tree pass per row at gather time
	 *     to extract that column's diagonal and then threw the rest away. This
	 *     keeps it. `SolveImpulseResponse` is linear in its applied wrench --
	 *     `PairImpulseResponseAtPoint` already relies on that by superposing two
	 *     queries -- so one unit-impulse call per row yields that row's ENTIRE
	 *     column of J M^-1 J^T, not merely its diagonal.
	 *  3. IT MAKES THE RESIDUAL OBSERVABLE. `FIterationDebugLog::
	 *     GlobalResidualPerIteration` reports max_i |Cdot_i + Bias_i| per sweep.
	 *     The per-row path cannot report this because it never forms a residual,
	 *     which is a large part of why "is it converging or oscillating?" stayed
	 *     an open question through entries 016-024.
	 *
	 * WHAT THIS IS NOT. The sweep implemented here is projected SOR on the
	 * assembled matrix, not MuJoCo's Newton solver on the convex dual. That is a
	 * deliberate stopping point: assembling A is the structural change and the
	 * prerequisite for anything better, whereas swapping SOR for Newton
	 * afterwards is a self-contained change to this one loop that can be made and
	 * measured on its own. Solving the RIGHT system approximately is a different
	 * situation from solving the wrong one exactly, and it is the one worth
	 * getting to first.
	 *
	 * WARM STARTING is handled differently from the per-row path, and better: the
	 * cached impulses seed `lambda` directly instead of being applied to the
	 * bodies up front. The residual formula accounts for them exactly, so there
	 * is no "apply, then measure, then correct" round trip and no risk of
	 * double-application.
	 *
	 * PER ENV. Constraint rows in different envs never couple, so A is block
	 * diagonal by env and assembling one matrix over all 256 would be a 5.9e9
	 * entry matrix with 256 nonzero blocks. This solves one env's block at a
	 * time. That structure is also what finally makes the contact path
	 * parallelizable over envs -- each block is fully independent now, and the
	 * only thing still preventing a ParallelFor is `SolveImpulseResponse`'s
	 * single-env mutable scratch on the solver.
	 */
	struct FGlobalRow
	{
		enum class EKind : uint8
		{
			GroundNormal,
			GroundTangent,   // T1; its T2 partner is always the next row
			GroundTangent2,
			LimitRevolute,
			LimitBall,
			PairNormal,
			PairTangent,
			PairTangent2,
		};

		EKind Kind = EKind::GroundNormal;
		/** Index into Active / LimitRows / BallLimitRows / ActivePairs, by Kind. */
		int32 SrcIdx = INDEX_NONE;
		/** For a tangent row, the row index of its own contact's normal row (for the Coulomb cone). */
		int32 NormalRow = INDEX_NONE;
		/** World direction for contact/pair rows; constraint direction for ball limits. Unused for revolute. */
		FVector Dir = FVector::ZeroVector;

		/** Biased-pass softness triple, precomputed at build time. */
		float Bias = 0.0f;
		float MassScale = 1.0f;
		float ImpulseScale = 0.0f;
		/** Regularization added to this row's diagonal, and its SOR factor. */
		float Cfm = 0.0f;
		float Relaxation = 1.0f;
		/** Coulomb coefficient for a tangent row; upper bound for a normal row (0 = unbounded). */
		float Friction = 0.0f;
		float MaxImpulse = 0.0f;

		/** Slot in FImpulseContactCache this row's accumulated impulse persists in. */
		int32 CacheSlot = INDEX_NONE;
		/** Accumulated impulse, warm-started from the cache. */
		float Lambda = 0.0f;
		/** Constraint velocity before ANY impulse is applied this step. */
		float Cdot0 = 0.0f;
		/** 1 / (A[i][i] + Cfm), or 0 for a row dropped as degenerate. */
		float InvDiag = 0.0f;
	};

	/** Reusable scratch for the global solve, owned by the caller so it is not reallocated per substep. */
	struct FGlobalSolveScratch
	{
		TArray<FGlobalRow> Rows;
		/** Response columns, row-major: RespDV[Row * NumBodies + Body], RespDQ[Row * NumDOF + DOF]. */
		TArray<FSpatialVec> RespDV;
		TArray<float> RespDQ;
		/** Dense A, row-major [i * NumRows + j]. */
		TArray<float> A;
		/** Maintained constraint velocities, Cdot[i] = Cdot0[i] + sum_j A[i][j] * Lambda[j]. */
		TArray<float> Cdot;
		/** Current-state velocity vector the initial residual is measured from. */
		TArray<FSpatialVec> StateDV;
		TArray<float> StateDQ;
		/** Per-call scratch for one SolveImpulseResponse query. */
		TArray<FSpatialVec> QueryDV;
		TArray<float> QueryDQ;
		TArray<float> QueryJointImp;
	};

	/** Accumulated impulses persisted ACROSS steps, for warm starting, plus the global solve's pooled buffers. */
	struct FImpulseContactCache
	{
		TArray<float> NormalImpulse;
		TArray<float> TangentImpulse1;
		TArray<float> TangentImpulse2;
		/** Accumulated joint-limit impulses, 2 rows per DOF (lower, upper) per env -- warm started exactly like the contact ones. */
		TArray<float> JointLimitImpulse;

		/**
		 * Accumulated ball-joint cone-limit impulses, one row per BODY (not
		 * per DOF -- a ball joint's cone is a single one-sided constraint on
		 * its rotation vector's magnitude, not a Lo/Hi pair like a revolute's
		 * per-axis range) per env, warm started the same way.
		 */
		TArray<float> BallJointLimitImpulse;

		/** Warm-start impulses for limb-pair contacts -- one slot per LimbPairs entry per env, parallel to the ground arrays above. */
		TArray<float> PairNormalImpulse;
		TArray<float> PairTangentImpulse1;
		TArray<float> PairTangentImpulse2;

		/**
		 * Working buffers for the global solve (FImpulseContactParams::
		 * bUseGlobalSolve). Lives here rather than as a local so the dense matrix
		 * and the response columns are not reallocated every substep -- at ~200
		 * rows on the real rig that is a few hundred KB of churn per substep per
		 * env otherwise. Contains no state that must survive between calls; it is
		 * pooled memory, not persisted solver state (the persisted state is the
		 * accumulated-impulse arrays above, which the global path writes back at
		 * the end of each solve so warm starting works identically).
		 */
		FGlobalSolveScratch GlobalScratch;

		void EnsureSized(int32 NumPoints, int32 NumEnvs)
		{
			const int32 N = FMath::Max(1, NumPoints * 2 * NumEnvs);
			if (NormalImpulse.Num() != N)
			{
				NormalImpulse.SetNumZeroed(N);
				TangentImpulse1.SetNumZeroed(N);
				TangentImpulse2.SetNumZeroed(N);
			}
		}

		void EnsureLimitsSized(int32 NumDOF, int32 NumEnvs)
		{
			const int32 N = FMath::Max(1, NumDOF * 2 * NumEnvs);
			if (JointLimitImpulse.Num() != N)
			{
				JointLimitImpulse.SetNumZeroed(N);
			}
		}

		void EnsureBallLimitsSized(int32 NumBodies, int32 NumEnvs)
		{
			const int32 N = FMath::Max(1, NumBodies * NumEnvs);
			if (BallJointLimitImpulse.Num() != N)
			{
				BallJointLimitImpulse.SetNumZeroed(N);
			}
		}

		void EnsurePairsSized(int32 NumPairs, int32 NumEnvs)
		{
			const int32 N = FMath::Max(1, NumPairs * NumEnvs);
			if (PairNormalImpulse.Num() != N)
			{
				PairNormalImpulse.SetNumZeroed(N);
				PairTangentImpulse1.SetNumZeroed(N);
				PairTangentImpulse2.SetNumZeroed(N);
			}
		}
	};

	/**
	 * Soft-constraint coefficients for one timestep, from a natural frequency and
	 * damping ratio (Box2D v3's formulation).
	 *
	 *   omega = 2*pi*hertz
	 *   a1 = 2*zeta + h*omega
	 *   a2 = h*omega*a1
	 *   a3 = 1/(1+a2)
	 *   biasRate = omega/a1        massScale = a2*a3        impulseScale = a3
	 *
	 * `impulseScale` is the regularization: each iteration removes that fraction
	 * of the accumulated impulse, so the constraint is genuinely compliant rather
	 * than a hard constraint with a bias hack. As hertz -> infinity, a2 -> inf,
	 * impulseScale -> 0 and massScale -> 1, recovering the hard constraint --
	 * which is exactly the regime that was unstable.
	 */
	struct FContactSoftness
	{
		float BiasRate = 0.0f;
		float MassScale = 1.0f;
		float ImpulseScale = 0.0f;
	};

	inline FContactSoftness MakeContactSoftness(float Hertz, float Zeta, float H)
	{
		FContactSoftness S;
		if (Hertz <= 0.0f || H <= 0.0f)
		{
			return S; // hard constraint: no bias, no relaxation
		}
		const float Omega = 2.0f * PI * Hertz;
		const float A1 = 2.0f * Zeta + H * Omega;
		const float A2 = H * Omega * A1;
		const float A3 = 1.0f / (1.0f + A2);
		S.BiasRate = Omega / A1;
		S.MassScale = A2 * A3;
		S.ImpulseScale = A3;
		return S;
	}

	/** Deterministic tangent basis for a normal -- must be stable step to step or warm starting is meaningless. */
	inline void BuildTangentBasis(const FVector& N, FVector& OutT1, FVector& OutT2)
	{
		OutT1 = (FMath::Abs(N.X) >= 0.57f)
			? FVector::CrossProduct(N, FVector(0.0f, 1.0f, 0.0f)).GetSafeNormal()
			: FVector::CrossProduct(N, FVector(1.0f, 0.0f, 0.0f)).GetSafeNormal();
		OutT2 = FVector::CrossProduct(N, OutT1).GetSafeNormal();
	}

	/**
	 * Closest points between two line segments (P1,Q1) and (P2,Q2), clamped
	 * to the segments -- standard algorithm (Ericson, "Real-Time Collision
	 * Detection" 5.1.9), new to this codebase. Needed because limb-vs-limb
	 * bodies are capsules (a segment + radius): ground contact could get
	 * away with treating each capsule end as an independent sphere because
	 * the other side of that test is a flat plane, but two arbitrary
	 * capsules need a genuine segment-segment distance.
	 */
	inline void ClosestPointsBetweenSegments(const FVector& P1, const FVector& Q1, const FVector& P2, const FVector& Q2, FVector& OutC1, FVector& OutC2)
	{
		const FVector D1 = Q1 - P1;
		const FVector D2 = Q2 - P2;
		const FVector R = P1 - P2;
		const float A = (float)FVector::DotProduct(D1, D1);
		const float E = (float)FVector::DotProduct(D2, D2);
		const float F = (float)FVector::DotProduct(D2, R);

		float S, T;
		if (A <= UE_SMALL_NUMBER && E <= UE_SMALL_NUMBER)
		{
			// Both segments degenerate to points -- e.g. a body with no
			// fused-tip offset and no capsule half-height, the same
			// "collapses to a single sphere at the body origin" case ground
			// contact already treats as valid.
			S = 0.0f;
			T = 0.0f;
		}
		else if (A <= UE_SMALL_NUMBER)
		{
			S = 0.0f;
			T = FMath::Clamp(F / E, 0.0f, 1.0f);
		}
		else
		{
			const float C = (float)FVector::DotProduct(D1, R);
			if (E <= UE_SMALL_NUMBER)
			{
				T = 0.0f;
				S = FMath::Clamp(-C / A, 0.0f, 1.0f);
			}
			else
			{
				const float B = (float)FVector::DotProduct(D1, D2);
				const float Denom = A * E - B * B;
				S = (Denom > UE_SMALL_NUMBER) ? FMath::Clamp((B * F - C * E) / Denom, 0.0f, 1.0f) : 0.0f;
				T = (B * S + F) / E;

				if (T < 0.0f)
				{
					T = 0.0f;
					S = FMath::Clamp(-C / A, 0.0f, 1.0f);
				}
				else if (T > 1.0f)
				{
					T = 1.0f;
					S = FMath::Clamp((B - C) / A, 0.0f, 1.0f);
				}
			}
		}

		OutC1 = P1 + D1 * S;
		OutC2 = P2 + D2 * T;
	}

	/**
	 * THE one derivation of a capsule's two local end caps from an authored
	 * (tip offset, half height) pair. Everything that needs capsule ends calls
	 * this: GetBodyWorldCapsule for limb-vs-limb, the ground gather in
	 * ResolveGroundContactImpulses, AMutoRLTrainingDriver::ComputeDefaultStandingHeight,
	 * and CreatureRLEnvironment's centre-of-pressure term.
	 *
	 * IT EXISTS BECAUSE THOSE FOUR HAD DRIFTED, and one of them was wrong.
	 *
	 * `TipOffset` is the fixed end cap. When it is non-zero the axis is its own
	 * direction and the base cap is pulled back along it. When it is ZERO --
	 * which is the normal, correct state for an INTERIOR chain body whose child
	 * is a real articulated body rather than a fused Tip -- there is no direction
	 * to derive, and `TipOffset.GetSafeNormal()` returns the ZERO VECTOR.
	 *
	 * GetBodyWorldCapsule already handled that, falling back to local +X (the
	 * rig's authored joint-orient convention is "+X toward the child bone",
	 * confirmed against real data: FElbow2_L's offset in FElbow_L's own rest
	 * frame is exactly (247.9, 0, 0)). The GROUND path did not. It computed
	 *     LocalEnds[1] = LocalOffset - LocalOffset.GetSafeNormal() * 2h
	 * which for a zero LocalOffset is `0 - 0 * 2h` == LocalEnds[0], while still
	 * setting NumEnds = 2 -- producing TWO INDEPENDENT CONTACT ROWS AT EXACTLY
	 * THE SAME WORLD POINT, each with its own accumulated impulse and its own
	 * warm-start slot. That is precisely the "two rows fighting over the same
	 * load" case BuildMutoContactPoints goes out of its way to avoid when it
	 * refuses to double up authored points, and that accumulated-impulse
	 * clamping handles worst. It fired on every authored contact point sitting
	 * on an interior body -- FElbow3_* and BElbow3_* are flagged CanTouchGround
	 * and were measured at leverArm = 0.00 (SOLVER_DEBUG_LOG.md entry 021), i.e.
	 * exactly this case, on four bodies, every substep.
	 *
	 * Returns the number of DISTINCT ends (1 or 2). Callers must honour it
	 * rather than assuming 2 whenever a half height is authored: a coincidence
	 * check is the backstop that makes a future degenerate input harmless
	 * instead of silently doubling a row again.
	 */
	inline int32 GetCapsuleLocalEnds(const FVector& TipOffset, float CapsuleHalfHeight, FVector& OutTip, FVector& OutBase)
	{
		if (!TipOffset.IsNearlyZero())
		{
			OutTip = TipOffset;
			OutBase = (CapsuleHalfHeight > 0.0f)
				? TipOffset - TipOffset.GetSafeNormal() * (CapsuleHalfHeight * 2.0f)
				: TipOffset;
		}
		else
		{
			// Interior body: run from this body's own origin out along local +X
			// (toward its real child) so the capsule spans the visible bone
			// segment instead of collapsing onto the proximal pivot.
			OutBase = FVector::ZeroVector;
			OutTip = FVector::ForwardVector * (CapsuleHalfHeight * 2.0f);
		}
		// A zero-length capsule is a sphere, and a sphere is ONE row.
		return FVector::DistSquared(OutTip, OutBase) > UE_SMALL_NUMBER ? 2 : 1;
	}

	/**
	 * A body's collision capsule in world space.
	 *
	 * Two cases, because BodyFusedTipOffset means something different on each
	 * side of the rig's own Bone/Child split (see MutoTopology.h's 2026-08-11
	 * correction):
	 *
	 *  - LEAF bodies whose child was folded into a rigid fused "Tip" (Hand,
	 *    Feet -- no muscle ever drives the Tip independently): BodyFusedTipOffset
	 *    is the real, nonzero offset to that fused geometry, and IS the tip axis.
	 *    Same "tip end + pulled-back base end" convention ResolveGroundContactImpulses'
	 *    ground-point gather already uses.
	 *  - INTERIOR chain bodies whose child is a REAL, independently-articulated
	 *    body (e.g. FElbow -> FElbow2): BodyFusedTipOffset is correctly zero
	 *    (there is no fused geometry -- the child moves on its own), but these
	 *    bodies still carry a real, large authored CapsuleHalfHeight (confirmed
	 *    against the real rig: FElbow_L's CapsuleHalfHeight*2 = 241.4, essentially
	 *    equal to its 247.9-unit bone length to FElbow2_L). Deriving the capsule
	 *    axis from BodyFusedTipOffset.GetSafeNormal() here silently returns the
	 *    ZERO vector (GetSafeNormal on a zero-length input), collapsing BOTH
	 *    capsule ends onto the body's own joint origin regardless of
	 *    CapsuleHalfHeight -- i.e. every interior limb bone's "capsule" was
	 *    actually a point-sphere sitting at its proximal pivot, nowhere near
	 *    covering its visible length. This is why FElbow crossing BElbow2 mid-bone
	 *    never registered contact even after limb collision was wired up and
	 *    independently confirmed correct on Hand/Feet pairs. Fixed by using local
	 *    +X as the axis instead: the rig's authored joint-orient convention is
	 *    "+X toward the child bone" (see MutoTopology.h / project memory),
	 *    confirmed directly against real data (FElbow2_L's JointOffsetInParent,
	 *    i.e. its offset in FElbow_L's own rest frame, is exactly (247.9, 0, 0)).
	 */
	inline void GetBodyWorldCapsule(const FCreatureBatchState& Batch, const FCreatureTopology& Topo, int32 Body, int32 Env, FVector& OutP0, FVector& OutP1)
	{
		const FQuat Rot = Batch.GetBodyRot(Body, Env);
		const FVector BodyPos = Batch.GetBodyPos(Body, Env);

		// Shared with the ground path — see GetCapsuleLocalEnds.
		FVector LocalTip, LocalBase;
		GetCapsuleLocalEnds(Topo.BodyFusedTipOffset[Body], Topo.BodyCapsuleHalfHeight[Body], LocalTip, LocalBase);

		OutP0 = BodyPos + Rot.RotateVector(LocalBase);
		OutP1 = BodyPos + Rot.RotateVector(LocalTip);
	}

	/**
	 * A contact point's world-space collision surface positions, one per distinct
	 * capsule end, already pushed out to the SURFACE along -Normal by Radius.
	 * Returns the number written (1 or 2).
	 *
	 * The single place that turns an FContactPointDef into world positions.
	 * ResolveGroundContactImpulses' gather, the reward's centre-of-pressure term
	 * and the driver's standing-height derivation all used to do this inline and
	 * inconsistently -- the gather applied the radius offset and the reward did
	 * not, which put the balance term's centre of pressure at joint origins
	 * rather than contact patches for the 25 structural points that have a zero
	 * LocalOffset.
	 */
	inline int32 GetContactPointWorldSurfaces(const FCreatureBatchState& Batch, const FContactPointDef& Point, int32 Env,
	                                          const FVector& GroundNormal, FVector OutWorld[2])
	{
		FVector LocalTip, LocalBase;
		const int32 NumEnds = GetCapsuleLocalEnds(Point.LocalOffset, Point.CapsuleHalfHeight, LocalTip, LocalBase);

		const FQuat Rot = Batch.GetBodyRot(Point.BodyIndex, Env);
		const FVector BodyPos = Batch.GetBodyPos(Point.BodyIndex, Env);
		const FVector SurfaceOffset = Point.Radius * GroundNormal;

		OutWorld[0] = BodyPos + Rot.RotateVector(LocalTip) - SurfaceOffset;
		OutWorld[1] = (NumEnds == 2) ? (BodyPos + Rot.RotateVector(LocalBase) - SurfaceOffset) : OutWorld[0];
		return NumEnds;
	}

	/**
	 * Optional per-iteration diagnostic hook (2026-08-17): captures one
	 * watched ground-contact body's accumulated normal impulse and one
	 * watched joint-limit DOF's accumulated impulse, once per PGS
	 * iteration -- built specifically to measure whether a coupled-row
	 * instability (a body simultaneously touching ground AND sitting at its
	 * own joint limit, e.g. an ankle folded onto its stop) is actually
	 * converging or oscillating, iteration by iteration, rather than only
	 * seeing the post-substep result. Purely additive: leave both Watch*
	 * fields at INDEX_NONE (or pass nullptr for DebugLog) for zero
	 * behavioral or performance change -- nothing is logged or checked in
	 * the hot path unless a caller explicitly asks.
	 */
	struct FIterationDebugLog
	{
		int32 WatchBody = INDEX_NONE;
		int32 WatchDOF = INDEX_NONE;
		TArray<float> ContactNormalImpulsePerIteration;
		TArray<float> JointLimitImpulsePerIteration;
		/**
		 * Global-solve only: per-iteration infinity norm of the constraint
		 * residual max_i |Cdot_i + Bias_i| over all rows in the watched env. This
		 * is the number that actually answers "is it converging?", and it is only
		 * available once the constraint system is assembled — the per-row path has
		 * no residual to report because it never forms one.
		 */
		TArray<float> GlobalResidualPerIteration;
		/** Global-solve only: row count and matrix dimension actually assembled, for the watched env. */
		int32 GlobalNumRows = 0;
	};

	// ---- Gathered constraint rows -------------------------------------------
	//
	// At namespace scope (they used to be local to ResolveGroundContactImpulses)
	// so the global solver can consume the same gather rather than duplicating
	// 200 lines of row construction. The gather is the part that has been
	// measured and debugged repeatedly; only the SOLVE differs between the two
	// paths, and that is exactly the seam to put the switch on.

	/** One active ground-contact end: a body point overlapping the ground plane. */
	struct FActive
	{
		int32 PointIdx, Body, Slot, Env;
		FVector WorldPoint;
		float Penetration;
		float InvMassN, InvMassT1, InvMassT2;
	};

	/** One active limb-vs-limb pair. Normal points from A to B. */
	struct FActivePair
	{
		int32 PairIdx, BodyA, BodyB, Env;
		FVector WorldPtA, WorldPtB, Normal;
		float Penetration;
		float InvMassN, InvMassT1, InvMassT2;
	};

	/** One active revolute joint-limit row. Sign*qd is d(separation)/dt either way. */
	struct FLimitRow
	{
		int32 DOF, Env, Slot;
		float Sign;       // +1 lower stop, -1 upper stop
		float Separation; // radians; >0 = gap remaining, <0 = already past the stop
		float InvMass;    // d(qd)/d(joint impulse) for this DOF, whole-tree
	};

	/** One active ball-joint cone-limit row. */
	struct FBallLimitRow
	{
		int32 Body, DOFOffset, Env, Slot;
		FVector ConstraintDir; // unit; +impulse along this pushes the joint back toward its rest cone
		float Separation;      // radians; >0 = gap remaining to the cone surface, <0 = already past it
		float InvMass;         // d(d|RotVec|/dt)/d(joint impulse along ConstraintDir), whole-tree
	};


	/**
	 * Evaluate one row's constraint velocity against a (DeltaV, DeltaQd) pair.
	 * Used for both the initial residual (state velocities) and for assembly
	 * (response columns), which is exactly why it takes them as arguments: the
	 * Jacobian must be identical in both, or A and Cdot0 describe different
	 * constraints.
	 */
	inline float EvalGlobalRow(const FGlobalRow& Row,
	                           const TArray<FActive>& Active, const TArray<FActivePair>& ActivePairs,
	                           const TArray<FLimitRow>& LimitRows, const TArray<FBallLimitRow>& BallLimitRows,
	                           const FCreatureBatchState& Batch, const FCreatureTopology& Topo,
	                           const FSpatialVec* DV, const float* DQ)
	{
		switch (Row.Kind)
		{
		case FGlobalRow::EKind::GroundNormal:
		case FGlobalRow::EKind::GroundTangent:
		case FGlobalRow::EKind::GroundTangent2:
		{
			const FActive& Act = Active[Row.SrcIdx];
			const FVector Arm = Act.WorldPoint - Batch.GetBodyPos(Act.Body, Act.Env);
			const FSpatialVec& V = DV[Act.Body];
			return (float)FVector::DotProduct(V.Lin + FVector::CrossProduct(V.Ang, Arm), Row.Dir);
		}
		case FGlobalRow::EKind::PairNormal:
		case FGlobalRow::EKind::PairTangent:
		case FGlobalRow::EKind::PairTangent2:
		{
			const FActivePair& AP = ActivePairs[Row.SrcIdx];
			const FVector ArmA = AP.WorldPtA - Batch.GetBodyPos(AP.BodyA, AP.Env);
			const FVector ArmB = AP.WorldPtB - Batch.GetBodyPos(AP.BodyB, AP.Env);
			const FVector VA = DV[AP.BodyA].Lin + FVector::CrossProduct(DV[AP.BodyA].Ang, ArmA);
			const FVector VB = DV[AP.BodyB].Lin + FVector::CrossProduct(DV[AP.BodyB].Ang, ArmB);
			// VN > 0 = separating, matching the per-row path's convention.
			return (float)FVector::DotProduct(VB - VA, Row.Dir);
		}
		case FGlobalRow::EKind::LimitRevolute:
		{
			const FLimitRow& R = LimitRows[Row.SrcIdx];
			return R.Sign * DQ[R.DOF];
		}
		case FGlobalRow::EKind::LimitBall:
		{
			const FBallLimitRow& R = BallLimitRows[Row.SrcIdx];
			return (float)(Row.Dir.X * DQ[R.DOFOffset + 0] + Row.Dir.Y * DQ[R.DOFOffset + 1] + Row.Dir.Z * DQ[R.DOFOffset + 2]);
		}
		default:
			return 0.0f;
		}
	}

	/**
	 * Compute one row's response column: the whole-tree velocity change produced
	 * by a UNIT impulse along that row's own constraint direction. One
	 * SolveImpulseResponse call (two, superposed, for a limb pair).
	 */
	inline void BuildGlobalRowResponse(const FGlobalRow& Row,
	                                   const TArray<FActive>& Active, const TArray<FActivePair>& ActivePairs,
	                                   const TArray<FLimitRow>& LimitRows, const TArray<FBallLimitRow>& BallLimitRows,
	                                   FCreatureBatchState& Batch, const FCreatureTopology& Topo,
	                                   FCreatureABASolver& Solver, FGlobalSolveScratch& S,
	                                   FSpatialVec* OutDV, float* OutDQ)
	{
		const int32 NumBodies = Topo.NumBodies;
		const int32 NumDOF = FMath::Max(1, Topo.NumDOF);
		S.QueryDV.SetNum(NumBodies, EAllowShrinking::No);
		S.QueryDQ.SetNumZeroed(NumDOF, EAllowShrinking::No);
		S.QueryJointImp.SetNumZeroed(NumDOF, EAllowShrinking::No);
		for (int32 d = 0; d < NumDOF; ++d) { S.QueryDQ[d] = 0.0f; S.QueryJointImp[d] = 0.0f; }

		auto Accumulate = [&](float Scale)
		{
			for (int32 b = 0; b < NumBodies; ++b)
			{
				OutDV[b] = OutDV[b] + S.QueryDV[b] * Scale;
			}
			for (int32 d = 0; d < Topo.NumDOF; ++d)
			{
				OutDQ[d] += S.QueryDQ[d] * Scale;
			}
		};

		for (int32 b = 0; b < NumBodies; ++b) { OutDV[b] = FSpatialVec{ FVector::ZeroVector, FVector::ZeroVector }; }
		for (int32 d = 0; d < Topo.NumDOF; ++d) { OutDQ[d] = 0.0f; }

		switch (Row.Kind)
		{
		case FGlobalRow::EKind::GroundNormal:
		case FGlobalRow::EKind::GroundTangent:
		case FGlobalRow::EKind::GroundTangent2:
		{
			const FActive& Act = Active[Row.SrcIdx];
			const FVector Arm = Act.WorldPoint - Batch.GetBodyPos(Act.Body, Act.Env);
			const FSpatialVec Wrench{ FVector::CrossProduct(Arm, Row.Dir), Row.Dir };
			Solver.SolveImpulseResponse(Batch, Act.Env, Act.Body, Wrench, S.QueryDV, S.QueryDQ);
			Accumulate(1.0f);
			break;
		}
		case FGlobalRow::EKind::PairNormal:
		case FGlobalRow::EKind::PairTangent:
		case FGlobalRow::EKind::PairTangent2:
		{
			// Superposition, same convention as PairImpulseResponseAtPoint:
			// +impulse on B, -impulse on A, so a positive lambda separates them.
			const FActivePair& AP = ActivePairs[Row.SrcIdx];
			const FVector ArmB = AP.WorldPtB - Batch.GetBodyPos(AP.BodyB, AP.Env);
			const FSpatialVec WrenchB{ FVector::CrossProduct(ArmB, Row.Dir), Row.Dir };
			Solver.SolveImpulseResponse(Batch, AP.Env, AP.BodyB, WrenchB, S.QueryDV, S.QueryDQ);
			Accumulate(1.0f);

			for (int32 d = 0; d < NumDOF; ++d) { S.QueryDQ[d] = 0.0f; }
			const FVector ArmA = AP.WorldPtA - Batch.GetBodyPos(AP.BodyA, AP.Env);
			const FSpatialVec WrenchA{ FVector::CrossProduct(ArmA, -Row.Dir), -Row.Dir };
			Solver.SolveImpulseResponse(Batch, AP.Env, AP.BodyA, WrenchA, S.QueryDV, S.QueryDQ);
			Accumulate(1.0f);
			break;
		}
		case FGlobalRow::EKind::LimitRevolute:
		{
			const FLimitRow& R = LimitRows[Row.SrcIdx];
			S.QueryJointImp[R.DOF] = R.Sign;
			Solver.SolveImpulseResponse(Batch, R.Env, INDEX_NONE, FSpatialVec{ FVector::ZeroVector, FVector::ZeroVector },
				S.QueryDV, S.QueryDQ, S.QueryJointImp.GetData());
			S.QueryJointImp[R.DOF] = 0.0f;
			Accumulate(1.0f);
			break;
		}
		case FGlobalRow::EKind::LimitBall:
		{
			const FBallLimitRow& R = BallLimitRows[Row.SrcIdx];
			S.QueryJointImp[R.DOFOffset + 0] = (float)Row.Dir.X;
			S.QueryJointImp[R.DOFOffset + 1] = (float)Row.Dir.Y;
			S.QueryJointImp[R.DOFOffset + 2] = (float)Row.Dir.Z;
			Solver.SolveImpulseResponse(Batch, R.Env, INDEX_NONE, FSpatialVec{ FVector::ZeroVector, FVector::ZeroVector },
				S.QueryDV, S.QueryDQ, S.QueryJointImp.GetData());
			S.QueryJointImp[R.DOFOffset + 0] = 0.0f;
			S.QueryJointImp[R.DOFOffset + 1] = 0.0f;
			S.QueryJointImp[R.DOFOffset + 2] = 0.0f;
			Accumulate(1.0f);
			break;
		}
		}
	}

	/** The cache slot a row's accumulated impulse persists in, by kind. */
	inline float& GlobalRowCacheEntry(const FGlobalRow& Row, FImpulseContactCache& Cache)
	{
		switch (Row.Kind)
		{
		case FGlobalRow::EKind::GroundNormal:   return Cache.NormalImpulse[Row.CacheSlot];
		case FGlobalRow::EKind::GroundTangent:  return Cache.TangentImpulse1[Row.CacheSlot];
		case FGlobalRow::EKind::GroundTangent2: return Cache.TangentImpulse2[Row.CacheSlot];
		case FGlobalRow::EKind::LimitRevolute:  return Cache.JointLimitImpulse[Row.CacheSlot];
		case FGlobalRow::EKind::LimitBall:      return Cache.BallJointLimitImpulse[Row.CacheSlot];
		case FGlobalRow::EKind::PairNormal:     return Cache.PairNormalImpulse[Row.CacheSlot];
		case FGlobalRow::EKind::PairTangent:    return Cache.PairTangentImpulse1[Row.CacheSlot];
		default:                                return Cache.PairTangentImpulse2[Row.CacheSlot];
		}
	}

	/**
	 * Assemble and solve one env's constraint block. See FGlobalRow's comment for
	 * the formulation and for why this exists.
	 */
	inline void SolveConstraintsGlobalForEnv(
		int32 Env,
		FCreatureBatchState& Batch,
		const FCreatureTopology& Topo,
		const TArray<FActive>& Active,
		const TArray<FActivePair>& ActivePairs,
		const TArray<FLimitRow>& LimitRows,
		const TArray<FBallLimitRow>& BallLimitRows,
		const FImpulseContactParams& Params,
		const FJointLimitParams* JointLimits,
		const FLimbCollisionParams* LimbCollision,
		const FContactSoftness& Soft,
		const FContactSoftness& LimitSoft,
		const FContactSoftness& PairSoft,
		const FVector& T1, const FVector& T2,
		float Dt,
		FCreatureABASolver& Solver,
		FImpulseContactCache& Cache,
		FGlobalSolveScratch& S,
		FIterationDebugLog* DebugLog)
	{
		const int32 NumBodies = Topo.NumBodies;
		const int32 NumDOF = FMath::Max(1, Topo.NumDOF);
		const int32 NumEnvs = Batch.GetNumEnvs();

		// ---- 1. Build this env's row list ----------------------------------
		S.Rows.Reset();

		auto AddContactTriple = [&](int32 SrcIdx, const FActive& Act)
		{
			const int32 NormalRow = S.Rows.Num();
			const float Separation = -FMath::Max(0.0f, Act.Penetration - Params.Slop);

			FGlobalRow N;
			N.Kind = FGlobalRow::EKind::GroundNormal;
			N.SrcIdx = SrcIdx;
			N.Dir = Params.GroundNormal;
			N.Cfm = Params.Cfm;
			N.Relaxation = Params.Relaxation;
			N.MaxImpulse = Params.MaxNormalImpulse;
			N.CacheSlot = Act.Slot;
			if (Separation < 0.0f)
			{
				N.Bias = FMath::Max(Soft.BiasRate * Separation,
					Params.MaxBiasVelocity > 0.0f ? -Params.MaxBiasVelocity : -BIG_NUMBER);
				N.MassScale = Soft.MassScale;
				N.ImpulseScale = Soft.ImpulseScale;
			}
			S.Rows.Add(N);

			FGlobalRow Tg;
			Tg.Kind = FGlobalRow::EKind::GroundTangent;
			Tg.SrcIdx = SrcIdx;
			Tg.Dir = T1;
			Tg.NormalRow = NormalRow;
			Tg.Friction = Params.FrictionCoefficient;
			Tg.CacheSlot = Act.Slot;
			S.Rows.Add(Tg);

			Tg.Kind = FGlobalRow::EKind::GroundTangent2;
			Tg.Dir = T2;
			S.Rows.Add(Tg);
		};

		for (int32 i = 0; i < Active.Num(); ++i)
		{
			if (Active[i].Env == Env) { AddContactTriple(i, Active[i]); }
		}

		for (int32 i = 0; i < LimitRows.Num(); ++i)
		{
			const FLimitRow& R = LimitRows[i];
			if (R.Env != Env) { continue; }
			FGlobalRow Row;
			Row.Kind = FGlobalRow::EKind::LimitRevolute;
			Row.SrcIdx = i;
			Row.Cfm = JointLimits->Cfm;
			Row.Relaxation = JointLimits->Relaxation;
			Row.CacheSlot = R.Slot;
			if (R.Separation > 0.0f)
			{
				// SPECULATIVE: the joint has not reached the stop, so the row must
				// only refuse to let it travel further than the remaining gap this
				// step. No softness on a speculative row — same as the per-row path.
				Row.Bias = R.Separation / Dt;
			}
			else
			{
				const float MaxBias = FMath::DegreesToRadians(JointLimits->MaxBiasVelocityDeg);
				Row.Bias = FMath::Max(LimitSoft.BiasRate * R.Separation, MaxBias > 0.0f ? -MaxBias : -BIG_NUMBER);
				Row.MassScale = LimitSoft.MassScale;
				Row.ImpulseScale = LimitSoft.ImpulseScale;
			}
			S.Rows.Add(Row);
		}

		for (int32 i = 0; i < BallLimitRows.Num(); ++i)
		{
			const FBallLimitRow& R = BallLimitRows[i];
			if (R.Env != Env) { continue; }
			FGlobalRow Row;
			Row.Kind = FGlobalRow::EKind::LimitBall;
			Row.SrcIdx = i;
			Row.Dir = R.ConstraintDir;
			Row.Cfm = JointLimits->Cfm;
			Row.Relaxation = JointLimits->Relaxation;
			Row.CacheSlot = R.Slot;
			if (R.Separation > 0.0f)
			{
				Row.Bias = R.Separation / Dt;
			}
			else
			{
				const float MaxBias = FMath::DegreesToRadians(JointLimits->MaxBiasVelocityDeg);
				Row.Bias = FMath::Max(LimitSoft.BiasRate * R.Separation, MaxBias > 0.0f ? -MaxBias : -BIG_NUMBER);
				Row.MassScale = LimitSoft.MassScale;
				Row.ImpulseScale = LimitSoft.ImpulseScale;
			}
			S.Rows.Add(Row);
		}

		for (int32 i = 0; i < ActivePairs.Num(); ++i)
		{
			const FActivePair& AP = ActivePairs[i];
			if (AP.Env != Env) { continue; }
			const int32 Slot = AP.PairIdx * NumEnvs + AP.Env;
			const int32 NormalRow = S.Rows.Num();
			const float Separation = -FMath::Max(0.0f, AP.Penetration - LimbCollision->Slop);

			FVector PT1, PT2;
			BuildTangentBasis(AP.Normal, PT1, PT2);

			FGlobalRow N;
			N.Kind = FGlobalRow::EKind::PairNormal;
			N.SrcIdx = i;
			N.Dir = AP.Normal;
			N.Cfm = LimbCollision->Cfm;
			N.Relaxation = LimbCollision->Relaxation;
			N.MaxImpulse = LimbCollision->MaxNormalImpulse;
			N.CacheSlot = Slot;
			if (Separation < 0.0f)
			{
				N.Bias = FMath::Max(PairSoft.BiasRate * Separation,
					LimbCollision->MaxBiasVelocity > 0.0f ? -LimbCollision->MaxBiasVelocity : -BIG_NUMBER);
				N.MassScale = PairSoft.MassScale;
				N.ImpulseScale = PairSoft.ImpulseScale;
			}
			S.Rows.Add(N);

			FGlobalRow Tg;
			Tg.Kind = FGlobalRow::EKind::PairTangent;
			Tg.SrcIdx = i;
			Tg.Dir = PT1;
			Tg.NormalRow = NormalRow;
			Tg.Friction = LimbCollision->FrictionCoefficient;
			Tg.CacheSlot = Slot;
			S.Rows.Add(Tg);

			Tg.Kind = FGlobalRow::EKind::PairTangent2;
			Tg.Dir = PT2;
			S.Rows.Add(Tg);
		}

		const int32 NumRows = S.Rows.Num();
		if (NumRows == 0)
		{
			return;
		}

		// ---- 2. Snapshot the current velocities, for Cdot0 ------------------
		S.StateDV.SetNum(NumBodies, EAllowShrinking::No);
		S.StateDQ.SetNumZeroed(NumDOF, EAllowShrinking::No);
		for (int32 b = 0; b < NumBodies; ++b)
		{
			const int32 Idx = Batch.BodyIndex(b, Env);
			S.StateDV[b] = FSpatialVec{
				FVector(Batch.AngVelX[Idx], Batch.AngVelY[Idx], Batch.AngVelZ[Idx]),
				FVector(Batch.LinVelX[Idx], Batch.LinVelY[Idx], Batch.LinVelZ[Idx]) };
		}
		for (int32 d = 0; d < Topo.NumDOF; ++d)
		{
			S.StateDQ[d] = Batch.JointVel[Batch.DOFIndex(d, Env)];
		}

		// ---- 3. Response columns, one whole-tree query per row --------------
		S.RespDV.SetNum(NumRows * NumBodies, EAllowShrinking::No);
		S.RespDQ.SetNumZeroed(NumRows * NumDOF, EAllowShrinking::No);
		for (int32 i = 0; i < NumRows; ++i)
		{
			BuildGlobalRowResponse(S.Rows[i], Active, ActivePairs, LimitRows, BallLimitRows,
				Batch, Topo, Solver, S,
				S.RespDV.GetData() + i * NumBodies, S.RespDQ.GetData() + i * NumDOF);
		}

		// ---- 4. A = J M^-1 J^T (+ Cfm on the diagonal) ----------------------
		S.A.SetNumZeroed(NumRows * NumRows, EAllowShrinking::No);
		for (int32 i = 0; i < NumRows; ++i)
		{
			for (int32 j = 0; j < NumRows; ++j)
			{
				S.A[i * NumRows + j] = EvalGlobalRow(S.Rows[i], Active, ActivePairs, LimitRows, BallLimitRows,
					Batch, Topo, S.RespDV.GetData() + j * NumBodies, S.RespDQ.GetData() + j * NumDOF);
			}
		}

		// Warm start seeds lambda directly; the residual below accounts for it
		// exactly, so nothing is applied to the bodies until step 6.
		for (int32 i = 0; i < NumRows; ++i)
		{
			FGlobalRow& Row = S.Rows[i];
			const float Diag = S.A[i * NumRows + i] + Row.Cfm;
			// Same reasoning as the per-row path's InvMassN guard: this is 1/m_eff,
			// so a coarse epsilon would silently drop the STIFFEST rows, which are
			// exactly the ones carrying load. UE_SMALL_NUMBER permits effective
			// masses to 1e8 kg.
			Row.InvDiag = (Diag > UE_SMALL_NUMBER) ? (1.0f / Diag) : 0.0f;
			Row.Lambda = (Row.InvDiag > 0.0f) ? GlobalRowCacheEntry(Row, Cache) : 0.0f;
			Row.Cdot0 = EvalGlobalRow(Row, Active, ActivePairs, LimitRows, BallLimitRows,
				Batch, Topo, S.StateDV.GetData(), S.StateDQ.GetData());
		}

		// Restitution folded in at build time — it depends only on the approach
		// velocity before any impulse, which is exactly Cdot0.
		for (int32 i = 0; i < NumRows; ++i)
		{
			FGlobalRow& Row = S.Rows[i];
			if (Row.Kind == FGlobalRow::EKind::GroundNormal && Params.Restitution > 0.0f)
			{
				Row.Bias += Params.Restitution * FMath::Min(0.0f, Row.Cdot0);
			}
			else if (Row.Kind == FGlobalRow::EKind::PairNormal && LimbCollision && LimbCollision->Restitution > 0.0f)
			{
				Row.Bias += LimbCollision->Restitution * FMath::Min(0.0f, Row.Cdot0);
			}
		}

		// ---- 5. Sweep in CONSTRAINT space — no tree traversals -------------
		S.Cdot.SetNumZeroed(NumRows, EAllowShrinking::No);
		for (int32 i = 0; i < NumRows; ++i)
		{
			float V = S.Rows[i].Cdot0;
			for (int32 j = 0; j < NumRows; ++j)
			{
				V += S.A[i * NumRows + j] * S.Rows[j].Lambda;
			}
			S.Cdot[i] = V;
		}

		// Applying a delta to row j updates every row's velocity through column j.
		auto ApplyDelta = [&](int32 j, float DeltaLambda)
		{
			if (DeltaLambda == 0.0f) { return; }
			S.Rows[j].Lambda += DeltaLambda;
			for (int32 i = 0; i < NumRows; ++i)
			{
				S.Cdot[i] += S.A[i * NumRows + j] * DeltaLambda;
			}
		};

		// Infinity norm of the constraint residual, max_i |Cdot_i + Bias_i|.
		auto Residual = [&](bool bRelax) -> float
		{
			float Worst = 0.0f;
			for (int32 i = 0; i < NumRows; ++i)
			{
				if (S.Rows[i].InvDiag <= 0.0f) { continue; }
				Worst = FMath::Max(Worst, FMath::Abs(S.Cdot[i] + (bRelax ? 0.0f : S.Rows[i].Bias)));
			}
			return Worst;
		};

		if (DebugLog)
		{
			// Record the residual BEFORE any sweep, so the series reads
			// [initial, after sweep 1, after sweep 2, ...]. Without this the first
			// recorded value is already post-solve and there is nothing to measure
			// the reduction against -- a well-conditioned system reaches its fixed
			// point in a single sweep, so every recorded value would be identical
			// and the trace would look like a solver that had done nothing.
			DebugLog->GlobalResidualPerIteration.Add(Residual(false));
			DebugLog->GlobalNumRows = NumRows;
		}

		const int32 RelaxIters = FMath::Max(0, Params.RelaxIterations);
		const int32 BiasedIters = FMath::Max(1, Params.GlobalIterations > 0 ? Params.GlobalIterations : Params.Iterations);
		const int32 TotalIters = BiasedIters + RelaxIters;

		for (int32 Iter = 0; Iter < TotalIters; ++Iter)
		{
			const bool bRelax = (Iter >= BiasedIters);

			for (int32 i = 0; i < NumRows; ++i)
			{
				FGlobalRow& Row = S.Rows[i];
				if (Row.InvDiag <= 0.0f) { continue; }

				// T2 is solved together with its T1 partner, below.
				if (Row.Kind == FGlobalRow::EKind::GroundTangent2 || Row.Kind == FGlobalRow::EKind::PairTangent2)
				{
					continue;
				}

				const bool bTangent = (Row.Kind == FGlobalRow::EKind::GroundTangent || Row.Kind == FGlobalRow::EKind::PairTangent);
				if (!bTangent)
				{
					const float Bias = bRelax ? 0.0f : Row.Bias;
					const float MassScale = bRelax ? 1.0f : Row.MassScale;
					const float ImpulseScale = bRelax ? 0.0f : Row.ImpulseScale;

					const float Delta = -MassScale * (S.Cdot[i] + Bias) * Row.InvDiag - ImpulseScale * Row.Lambda;
					float NewTotal = FMath::Max(0.0f, Row.Lambda + Row.Relaxation * Delta);
					if (Row.MaxImpulse > 0.0f)
					{
						NewTotal = FMath::Min(NewTotal, Row.MaxImpulse);
					}
					ApplyDelta(i, NewTotal - Row.Lambda);
					continue;
				}

				// Friction pair: solve both tangents, then project onto the Coulomb
				// cone of the CURRENT normal impulse. Frictionless directions get no
				// bias and no softness — they only ever remove sliding velocity.
				const int32 i2 = i + 1;
				if (i2 >= NumRows) { continue; }
				FGlobalRow& Row2 = S.Rows[i2];
				const float MaxT = Row.Friction * S.Rows[Row.NormalRow].Lambda;

				float NewT1 = Row.Lambda + (Row.InvDiag > 0.0f ? -S.Cdot[i] * Row.InvDiag : 0.0f);
				float NewT2 = Row2.Lambda + (Row2.InvDiag > 0.0f ? -S.Cdot[i2] * Row2.InvDiag : 0.0f);
				const float TMag = FMath::Sqrt(NewT1 * NewT1 + NewT2 * NewT2);
				if (TMag > MaxT && TMag > KINDA_SMALL_NUMBER)
				{
					const float Scale = MaxT / TMag;
					NewT1 *= Scale;
					NewT2 *= Scale;
				}
				ApplyDelta(i, NewT1 - Row.Lambda);
				ApplyDelta(i2, NewT2 - Row2.Lambda);
			}

			if (DebugLog)
			{
				// The observable the per-row path cannot provide, because it never
				// forms a residual at all.
				DebugLog->GlobalResidualPerIteration.Add(Residual(bRelax));
			}
		}

		// ---- 6. Apply once, from the stored response columns ----------------
		// No further tree solves: the response to every row is already in hand, so
		// the total velocity change is a plain linear combination.
		for (int32 i = 0; i < NumRows; ++i)
		{
			const FGlobalRow& Row = S.Rows[i];
			GlobalRowCacheEntry(const_cast<FGlobalRow&>(Row), Cache) = Row.Lambda;

			const float L = Row.Lambda;
			if (L == 0.0f) { continue; }
			const FSpatialVec* DV = S.RespDV.GetData() + i * NumBodies;
			const float* DQ = S.RespDQ.GetData() + i * NumDOF;
			for (int32 b = 0; b < NumBodies; ++b)
			{
				const int32 Idx = Batch.BodyIndex(b, Env);
				Batch.AngVelX[Idx] += (float)(DV[b].Ang.X * L);
				Batch.AngVelY[Idx] += (float)(DV[b].Ang.Y * L);
				Batch.AngVelZ[Idx] += (float)(DV[b].Ang.Z * L);
				Batch.LinVelX[Idx] += (float)(DV[b].Lin.X * L);
				Batch.LinVelY[Idx] += (float)(DV[b].Lin.Y * L);
				Batch.LinVelZ[Idx] += (float)(DV[b].Lin.Z * L);
			}
			for (int32 d = 0; d < Topo.NumDOF; ++d)
			{
				Batch.JointVel[Batch.DOFIndex(d, Env)] += DQ[d] * L;
			}
		}

		if (DebugLog)
		{
			for (int32 i = 0; i < NumRows; ++i)
			{
				const FGlobalRow& Row = S.Rows[i];
				if (Row.Kind == FGlobalRow::EKind::GroundNormal && DebugLog->WatchBody != INDEX_NONE
					&& Active[Row.SrcIdx].Body == DebugLog->WatchBody)
				{
					DebugLog->ContactNormalImpulsePerIteration.Add(Row.Lambda);
				}
				else if (Row.Kind == FGlobalRow::EKind::LimitRevolute && DebugLog->WatchDOF != INDEX_NONE
					&& LimitRows[Row.SrcIdx].DOF == DebugLog->WatchDOF)
				{
					DebugLog->JointLimitImpulsePerIteration.Add(Row.Lambda);
				}
			}
		}
	}

	/**
	 * Velocity-level contact resolution, sequential-impulse style.
	 * Call AFTER Solver.Step().
	 *
	 * Three things here follow production practice (Box2D / Bullet) rather than
	 * the naive formulation, each for a measured reason:
	 *
	 *  1. ACCUMULATED clamping. The running TOTAL impulse per contact is clamped
	 *     non-negative and the DIFFERENCE is applied -- not each increment
	 *     independently. Clamping increments stops a contact ever undoing an
	 *     earlier over-correction within the step, which is what makes
	 *     Gauss-Seidel oscillate once several contacts share a chain.
	 *  2. WARM STARTING. Last step's accumulated impulses are reapplied before
	 *     iterating, so resting contacts start from the answer instead of from
	 *     zero. Biggest single convergence win for a creature standing still.
	 *  3. RELAXATION PASS. After the biased iterations, iterate again with the
	 *     Baumgarte term OFF. Stabilization deliberately injects energy to push
	 *     penetration out; the relax pass removes whatever of it is left in the
	 *     velocities, which is the documented cure for bias-injected energy
	 *     rather than merely bounding it.
	 *
	 * Effective masses are computed ONCE per contact per step (they depend on
	 * configuration, not velocity), instead of twice per contact per iteration.
	 */
	inline void ResolveGroundContactImpulses(
		FCreatureBatchState& Batch,
		const FCreatureTopology& Topo,
		const TArray<FContactPointDef>& ContactPoints,
		const FImpulseContactParams& Params,
		FCreatureABASolver& Solver,
		float Dt,
		FImpulseContactCache& Cache,
		TArray<FContactPointState>* OutState = nullptr,
		const FJointLimitParams* JointLimits = nullptr,
		const TArray<FLimbPairDef>& LimbPairs = TArray<FLimbPairDef>(),
		const FLimbCollisionParams* LimbCollision = nullptr,
		FIterationDebugLog* DebugLog = nullptr,
		// Optional weld set for the factorization -- see
		// FCreatureABASolver::ComputeArticulatedInertias and BuildSaturatedJointLocks.
		// Stride 0 means one byte per body shared by every env (valid only at
		// NumEnvs == 1); a positive stride means [Body * stride + Env].
		const uint8* LockedBody = nullptr,
		int32 LockedStride = 0)
	{
		const int32 NumEnvs = Batch.GetNumEnvs();
		const int32 NumPoints = ContactPoints.Num();
		if (OutState)
		{
			// Init, not SetNumZeroed: TArray::SetNumZeroed only zero-initializes
			// NEWLY GROWN elements -- when the array is already the right size
			// (every call after the first, since NumPoints*NumEnvs is constant),
			// it is a no-op and every element keeps whatever it held last call.
			// A contact point that stops touching is then never revisited by the
			// gather loop below (which only writes bTouching/NormalForce for
			// ACTIVE points), so its LAST value -- including a one-time NaN/Inf
			// from a physics blowup -- survives forever, silently defeating the
			// entire IsBodyStateValid/IsTerminated/ResetEnv recovery mechanism
			// this project built specifically to make blowups recoverable
			// (CreatureRLEnvironment.h): body/DOF state resets cleanly, but a
			// poisoned ContactPointState::NormalForce is not covered by that
			// check at all and keeps reaching ComputeObservations every tick
			// after, corrupting every subsequent observation until the process
			// crashes on Learning Agents' hard non-finite assert. Confirmed via
			// a throwaway random-torque stress diagnostic (2026-08-16): body
			// state already recovered by IsTerminated's reset while NormalForce
			// stayed non-finite from several ticks earlier. Init unconditionally
			// overwrites every slot every call, closing the gap.
			OutState->Init(FContactPointState(), NumPoints * NumEnvs);
		}
		const bool bSolveLimits = JointLimits && JointLimits->bEnabled && Topo.NumDOF > 0;
		const bool bSolvePairs = LimbCollision && LimbCollision->bEnabled && LimbPairs.Num() > 0;
		if (Dt <= 0.0f || (NumPoints == 0 && !bSolveLimits && !bSolvePairs))
		{
			return;
		}
		Cache.EnsureSized(NumPoints, NumEnvs);
		if (bSolvePairs)
		{
			Cache.EnsurePairsSized(LimbPairs.Num(), NumEnvs);
		}

		FVector T1, T2;
		BuildTangentBasis(Params.GroundNormal, T1, T2);

		// A constraint cannot be stiffer than the step can resolve. Box2D clamps
		// contact hertz to a quarter of the step rate for exactly this reason,
		// and asking for more is how the previous formulation destabilized.
		const float ClampedHertz = FMath::Min(Params.ContactHertz, 0.25f / Dt);
		const FContactSoftness Soft = MakeContactSoftness(ClampedHertz, Params.DampingRatio, Dt);

		// One factorization serves every contact and every iteration: the
		// configuration does not change in here, only velocities do.
		//
		// LockedBody welds saturated joints into this factorization, which is what
		// makes a leg folded onto its stops present its rigid-strut effective mass
		// instead of its compliant one (measured 18 kg -> 269 kg, entry 024). It had
		// no production caller until 2026-08-17.
		Solver.ComputeArticulatedInertias(Batch, LockedBody, LockedStride);

		TArray<FActive> Active;
		Active.Reserve(NumPoints * 2);

		// ---- gather active contacts + their effective masses (once) ----
		for (int32 Env = 0; Env < NumEnvs; ++Env)
		{
			for (int32 PointIdx = 0; PointIdx < NumPoints; ++PointIdx)
			{
				const FContactPointDef& Point = ContactPoints[PointIdx];

				// Shared derivation, and it returns 1 rather than 2 whenever the two
				// ends would coincide — which is what stops an interior body's
				// zero-length "capsule" becoming two rows at one point. See
				// GetCapsuleLocalEnds for the full account of that bug.
				FVector WorldEnds[2];
				const int32 NumEnds = GetContactPointWorldSurfaces(Batch, Point, Env, Params.GroundNormal, WorldEnds);

				for (int32 EndIdx = 0; EndIdx < 2; ++EndIdx)
				{
					const int32 Slot = (PointIdx * 2 + EndIdx) * NumEnvs + Env;
					if (EndIdx >= NumEnds)
					{
						Cache.NormalImpulse[Slot] = 0.0f;
						Cache.TangentImpulse1[Slot] = 0.0f;
						Cache.TangentImpulse2[Slot] = 0.0f;
						continue;
					}

					const FVector WorldPoint = WorldEnds[EndIdx];
					const float Penetration = Params.GroundZ - static_cast<float>(WorldPoint.Z);

					if (Penetration <= 0.0f)
					{
						// Separated: forget its history, or warm starting would
						// reapply a stale impulse when it next touches.
						Cache.NormalImpulse[Slot] = 0.0f;
						Cache.TangentImpulse1[Slot] = 0.0f;
						Cache.TangentImpulse2[Slot] = 0.0f;
						continue;
					}

					FActive A;
					A.PointIdx = PointIdx; A.Body = Point.BodyIndex; A.Slot = Slot; A.Env = Env;
					A.WorldPoint = WorldPoint; A.Penetration = Penetration;
					A.InvMassN = (float)FVector::DotProduct(
						Solver.ImpulseResponseAtPoint(Batch, Env, A.Body, WorldPoint, Params.GroundNormal), Params.GroundNormal);
					A.InvMassT1 = (float)FVector::DotProduct(
						Solver.ImpulseResponseAtPoint(Batch, Env, A.Body, WorldPoint, T1), T1);
					A.InvMassT2 = (float)FVector::DotProduct(
						Solver.ImpulseResponseAtPoint(Batch, Env, A.Body, WorldPoint, T2), T2);
					// The guard exists only to avoid dividing by a zero inverse
					// mass. KINDA_SMALL_NUMBER (1e-4) is far too coarse for that:
					// InvMass is 1/m_eff, so it silently DROPS any contact whose
					// articulated effective mass exceeds 10,000 kg -- and a braced
					// contact on this 6170 kg rig was measured at 4390 kg, a margin
					// of only 2.3x (SOLVER_DEBUG_LOG.md entry 020). The contacts
					// most able to carry load are exactly the ones with the highest
					// effective mass, so this would discard them first.
					// UE_SMALL_NUMBER (1e-8) still guards the division while
					// allowing effective masses up to 1e8 kg.
					if (A.InvMassN <= UE_SMALL_NUMBER)
					{
						continue; // no compliance along the normal -- nothing to solve
					}
					Active.Add(A);
				}
			}
		}

		// ---- gather active limb-vs-limb pairs + their effective masses (once) ----
		//
		// LimbPairs is a static, all-different-limb candidate list (see
		// BuildMutoLimbCollisionPairs) -- the broadphase here is just "are
		// these two specific capsules overlapping RIGHT NOW", freshly tested
		// every call since poses change every substep. Same "separated ->
		// forget warm-start history" rule as the ground gather above.
		TArray<FActivePair> ActivePairs;
		// Computed once here (like Soft/LimitSoft above), not per-iteration --
		// only meaningful/used when bSolvePairs, exactly like LimitSoft is
		// only meaningful when bSolveLimits.
		FContactSoftness PairSoft;
		if (bSolvePairs)
		{
			const float ClampedPairHertz = FMath::Min(LimbCollision->Hertz, 0.25f / Dt);
			PairSoft = MakeContactSoftness(ClampedPairHertz, LimbCollision->DampingRatio, Dt);

			ActivePairs.Reserve(LimbPairs.Num());
			for (int32 Env = 0; Env < NumEnvs; ++Env)
			{
				for (int32 PairIdx = 0; PairIdx < LimbPairs.Num(); ++PairIdx)
				{
					const FLimbPairDef& Pair = LimbPairs[PairIdx];
					const int32 Slot = PairIdx * NumEnvs + Env;

					FVector SegA0, SegA1, SegB0, SegB1;
					GetBodyWorldCapsule(Batch, Topo, Pair.BodyA, Env, SegA0, SegA1);
					GetBodyWorldCapsule(Batch, Topo, Pair.BodyB, Env, SegB0, SegB1);
					const float RadiusA = Topo.BodyRadius[Pair.BodyA];
					const float RadiusB = Topo.BodyRadius[Pair.BodyB];

					FVector PtA, PtB;
					ClosestPointsBetweenSegments(SegA0, SegA1, SegB0, SegB1, PtA, PtB);

					const float RadiusSum = RadiusA + RadiusB;
					const FVector Delta = PtB - PtA;
					const float Distance = (float)Delta.Size();
					const float Penetration = RadiusSum - Distance;

					if (Penetration <= 0.0f)
					{
						Cache.PairNormalImpulse[Slot] = 0.0f;
						Cache.PairTangentImpulse1[Slot] = 0.0f;
						Cache.PairTangentImpulse2[Slot] = 0.0f;
						continue;
					}

					// Nearly-coincident closest points (capsule axes crossing
					// exactly) have no well-defined separation direction --
					// fall back to a fixed axis rather than normalizing noise.
					const FVector Normal = Distance > UE_SMALL_NUMBER ? (Delta / Distance) : FVector::UpVector;

					// Contact points on each capsule's SURFACE, not its axis --
					// matches the ground gather's own `- Point.Radius *
					// GroundNormal` surface correction.
					const FVector SurfPtA = PtA + Normal * RadiusA;
					const FVector SurfPtB = PtB - Normal * RadiusB;

					FActivePair AP;
					AP.PairIdx = PairIdx; AP.BodyA = Pair.BodyA; AP.BodyB = Pair.BodyB; AP.Env = Env;
					AP.WorldPtA = SurfPtA; AP.WorldPtB = SurfPtB; AP.Normal = Normal;
					AP.Penetration = Penetration;

					// Sign convention, matched by the resolve loop below: VN is
					// defined as (velocity of B - velocity of A) . Normal, so
					// VN > 0 means separating (B moving further from A along
					// the A->B direction) -- the same "positive = separating"
					// convention the ground contact's VN already uses. A
					// separating impulse is therefore +impulse on B, -impulse
					// on A, which is exactly PairImpulseResponseAtPoint's
					// (BodyX=B, BodyY=A) convention: "+Impulse at X, -Impulse
					// at Y". Passing (BodyA, BodyB) here instead would measure
					// the response to pushing them TOGETHER, not apart.
					FVector PT1, PT2;
					BuildTangentBasis(Normal, PT1, PT2);
					AP.InvMassN = (float)FVector::DotProduct(
						Solver.PairImpulseResponseAtPoint(Batch, Env, AP.BodyB, SurfPtB, AP.BodyA, SurfPtA, Normal), Normal);
					AP.InvMassT1 = (float)FVector::DotProduct(
						Solver.PairImpulseResponseAtPoint(Batch, Env, AP.BodyB, SurfPtB, AP.BodyA, SurfPtA, PT1), PT1);
					AP.InvMassT2 = (float)FVector::DotProduct(
						Solver.PairImpulseResponseAtPoint(Batch, Env, AP.BodyB, SurfPtB, AP.BodyA, SurfPtA, PT2), PT2);

					// Same reasoning as the ground gather's InvMassN guard --
					// UE_SMALL_NUMBER, not KINDA_SMALL_NUMBER, so a well-braced
					// pair isn't discarded for having a large effective mass.
					if (AP.InvMassN <= UE_SMALL_NUMBER)
					{
						continue;
					}
					ActivePairs.Add(AP);
				}
			}
		}

		// ---- gather active joint-limit rows + their effective masses (once) ----
		//
		// A row exists when the joint is inside MarginDeg of a stop, OR when its
		// current speed would carry it across the stop within this step. The
		// second half is what makes tunnelling impossible: the row is created
		// BEFORE the crossing, and its speculative bias below bounds the
		// approach to exactly what closes the remaining gap.
		TArray<FLimitRow> LimitRows;

		// Ball-joint cone-limit rows (2026-08-16). Ball joints (every spine
		// body and every limb's mount body) previously had NO velocity-level
		// joint-limit row at all -- CreatureBatchSolver.h's ClampJointLimits
		// is a hard, undamped position+velocity deadstop, but between hitting
		// that stop a ball joint was a perfectly frictionless pendulum with
		// zero damping anywhere. A long serial chain of these (the spine,
		// once independently articulated) can build up real resonant energy
		// under repeated ground-contact shocks with nothing to dissipate it,
		// confirmed via a throwaway diagnostic: a passive ragdoll drop with
		// zero torque showed Head2/LowerMouth's angular speed climbing from
		// ~0 to 97/77 rad/s between t=1.0-1.2s before the whole rig diverged.
		// This row closes that gap with the same soft, damped speculative-
		// contact treatment revolute joint limits already get, generalized
		// from a 1D per-DOF range to a 1D constraint along the ball joint's
		// current rotation-vector direction (the ONE direction its cone
		// limit can be violated in) -- everything else (Hertz/DampingRatio
		// softness, speculative margin, accumulated-impulse clamp, warm
		// start) is identical in spirit to the revolute rows just below.
		TArray<FBallLimitRow> BallLimitRows;

		FContactSoftness LimitSoft = Soft;
		if (bSolveLimits)
		{
			Cache.EnsureLimitsSized(Topo.NumDOF, NumEnvs);
			Cache.EnsureBallLimitsSized(Topo.NumBodies, NumEnvs);
			const float LimitHertz = FMath::Min(JointLimits->Hertz, 0.25f / Dt); // same step-rate ceiling as contact
			LimitSoft = MakeContactSoftness(LimitHertz, JointLimits->DampingRatio, Dt);

			const float MarginRad = FMath::DegreesToRadians(JointLimits->MarginDeg);
			const float SlopRad = FMath::DegreesToRadians(JointLimits->SlopDeg);

			for (int32 Env = 0; Env < NumEnvs; ++Env)
			{
				for (int32 Body = 1; Body < Topo.NumBodies; ++Body)
				{
					// Ball-joint cone-limit row: a single one-sided constraint
					// on the rotation vector's magnitude, along whichever
					// direction it's currently pointing (the only direction
					// the cone can be violated in) -- see FBallLimitRow's
					// comment for why this exists.
					if (Topo.BodyDOFCount[Body] == 3)
					{
						const int32 BallDOFOffset = Topo.BodyDOFOffset[Body];
						float MaxWidthDeg = 0.0f;
						bool bHasAnyRange = false;
						for (int32 k = 0; k < 3; ++k)
						{
							const int32 AxisDOF = BallDOFOffset + k;
							if (Topo.DOFHasMuscleCurve[AxisDOF])
							{
								bHasAnyRange = true;
								MaxWidthDeg = FMath::Max(MaxWidthDeg, Topo.DOFRangeMaxDeg[AxisDOF] - Topo.DOFRangeMinDeg[AxisDOF]);
							}
						}
						if (!bHasAnyRange || MaxWidthDeg <= 0.0f)
						{
							continue; // no authored range at all -- matches ClampJointLimits' own no-op in this case
						}
						const float MaxAngleRad = FMath::DegreesToRadians(MaxWidthDeg);

						const FVector RotVec = Batch.GetJointRelRot(Body, Env).ToRotationVector();
						const float Angle = static_cast<float>(RotVec.Size());
						if (Angle <= UE_SMALL_NUMBER)
						{
							continue; // no direction to constrain along, and nowhere near a MaxAngleRad>0 cone at the origin
						}
						const FVector ConstraintDir = -(RotVec / Angle); // +impulse here pushes the joint back toward its rest cone

						const FVector JointVelVec(
							Batch.JointVel[Batch.DOFIndex(BallDOFOffset + 0, Env)],
							Batch.JointVel[Batch.DOFIndex(BallDOFOffset + 1, Env)],
							Batch.JointVel[Batch.DOFIndex(BallDOFOffset + 2, Env)]);
						const float Cdot = (float)FVector::DotProduct(ConstraintDir, JointVelVec); // d(Separation)/dt

						const float SeparationRaw = MaxAngleRad - Angle;
						const float BallInvMass = Solver.BallJointImpulseResponse(Batch, Env, BallDOFOffset, ConstraintDir);
						if (BallInvMass <= UE_SMALL_NUMBER)
						{
							continue;
						}

						const bool bBallActive = SeparationRaw < MarginRad + FMath::Max(0.0f, -Cdot) * Dt;
						const int32 BallSlot = Body * NumEnvs + Env;
						if (bBallActive)
						{
							BallLimitRows.Add({ Body, BallDOFOffset, Env, BallSlot, ConstraintDir, SeparationRaw - SlopRad, BallInvMass });
						}
						else
						{
							Cache.BallJointLimitImpulse[BallSlot] = 0.0f;
						}
						continue;
					}
					if (Topo.BodyDOFCount[Body] != 1)
					{
						continue;
					}
					const int32 DOF = Topo.BodyDOFOffset[Body];
					if (!Topo.DOFHasMuscleCurve[DOF])
					{
						continue;
					}
					const float WidthDeg = Topo.DOFRangeMaxDeg[DOF] - Topo.DOFRangeMinDeg[DOF];
					if (WidthDeg <= 0.0f)
					{
						continue;
					}

					const int32 DOFIdx = Batch.DOFIndex(DOF, Env);
					const float AngleDeg = FMath::RadiansToDegrees(Batch.JointPos[DOFIdx]);
					const float Qd = Batch.JointVel[DOFIdx]; // rad/s

					// Wrapped into [0,360) relative to the range minimum -- the
					// SAME convention ClampJointLimits uses, and mandatory here:
					// this rig's authored ranges are things like [283.2, 403.6],
					// so a joint at bind pose reads 0 deg and is in range only
					// once you know 0 is 360.
					float Wrapped = FMath::Fmod(AngleDeg - Topo.DOFRangeMinDeg[DOF], 360.0f);
					if (Wrapped < 0.0f)
					{
						Wrapped += 360.0f;
					}

					// Signed gap to each stop. Inside the range both are >= 0;
					// inside the FORBIDDEN arc the nearer stop goes negative and
					// the far one is suppressed, so a violated joint is pushed
					// back the short way rather than driven the long way round.
					constexpr float FarAway = 1.0e6f;
					float GapLoDeg, GapHiDeg;
					if (Wrapped <= WidthDeg)
					{
						GapLoDeg = Wrapped;
						GapHiDeg = WidthDeg - Wrapped;
					}
					else
					{
						const float OverMax = Wrapped - WidthDeg;
						const float UnderMin = 360.0f - Wrapped;
						if (OverMax <= UnderMin) { GapHiDeg = -OverMax;  GapLoDeg = FarAway; }
						else                     { GapLoDeg = -UnderMin; GapHiDeg = FarAway; }
					}

					const float InvMass = Solver.JointImpulseResponse(Batch, Env, DOF);
					// Same reasoning as the contact normal guard: this is 1/m_eff,
					// so a coarse epsilon would silently drop the STIFFEST joints,
					// which are exactly the ones that need to carry load.
					if (InvMass <= UE_SMALL_NUMBER)
					{
						continue;
					}

					const float GapLoRad = FMath::DegreesToRadians(GapLoDeg);
					const float GapHiRad = FMath::DegreesToRadians(GapHiDeg);

					// A stop engages when the joint is within the margin of it,
					// or when its current speed would carry it past within this
					// step. Lower stop closes as qd goes negative, upper as it
					// goes positive.
					const bool bLoActive = GapLoRad < MarginRad + FMath::Max(0.0f, -Qd) * Dt;
					const bool bHiActive = GapHiRad < MarginRad + FMath::Max(0.0f, +Qd) * Dt;

					const int32 LoSlot = (DOF * 2 + 0) * NumEnvs + Env;
					const int32 HiSlot = (DOF * 2 + 1) * NumEnvs + Env;

					if (bLoActive)
					{
						LimitRows.Add({ DOF, Env, LoSlot, +1.0f, GapLoRad - SlopRad, InvMass });
					}
					else
					{
						// Inactive rows forget their history, or warm starting
						// reapplies a stale impulse the next time they engage --
						// the same rule the separated contacts above follow.
						Cache.JointLimitImpulse[LoSlot] = 0.0f;
					}

					if (bHiActive)
					{
						LimitRows.Add({ DOF, Env, HiSlot, -1.0f, GapHiRad - SlopRad, InvMass });
					}
					else
					{
						Cache.JointLimitImpulse[HiSlot] = 0.0f;
					}
				}
			}
		}

		// ================= SOLVE =================
		//
		// The gather above is shared; only the solve differs. That is deliberately
		// where the switch sits: the gather is the part that has been measured and
		// corrected repeatedly across entries 013-025, and none of that work is
		// specific to how the rows are subsequently relaxed.
		if (Params.bUseGlobalSolve)
		{
			// One block per env — rows in different envs never couple, so a single
			// matrix over all of them would be block diagonal with 255 wasted
			// blocks. See FGlobalRow's comment.
			for (int32 Env = 0; Env < NumEnvs; ++Env)
			{
				SolveConstraintsGlobalForEnv(Env, Batch, Topo, Active, ActivePairs, LimitRows, BallLimitRows,
					Params, JointLimits, LimbCollision, Soft, LimitSoft, PairSoft, T1, T2, Dt,
					Solver, Cache, Cache.GlobalScratch,
					// Only the watched env gets a trace, or the residual series from
					// 256 envs would be interleaved into one unreadable array.
					(DebugLog && Env == 0) ? DebugLog : nullptr);
			}
		}
		else
		{

		auto PointVelocity = [&Batch](int32 Body, int32 Env, const FVector& WorldPoint)
		{
			const int32 Idx = Batch.BodyIndex(Body, Env);
			const FVector W(Batch.AngVelX[Idx], Batch.AngVelY[Idx], Batch.AngVelZ[Idx]);
			const FVector V(Batch.LinVelX[Idx], Batch.LinVelY[Idx], Batch.LinVelZ[Idx]);
			return V + FVector::CrossProduct(W, WorldPoint - Batch.GetBodyPos(Body, Env));
		};

		// ---- 2. warm start: replay last step's converged impulses ----
		for (const FActive& A : Active)
		{
			const FVector Warm = Cache.NormalImpulse[A.Slot] * Params.GroundNormal
				+ Cache.TangentImpulse1[A.Slot] * T1
				+ Cache.TangentImpulse2[A.Slot] * T2;
			if (!Warm.IsNearlyZero())
			{
				Solver.ApplyImpulseAtPoint(Batch, A.Env, A.Body, A.WorldPoint, Warm);
			}
		}
		for (const FLimitRow& R : LimitRows)
		{
			const float Warm = Cache.JointLimitImpulse[R.Slot];
			if (FMath::Abs(Warm) > UE_SMALL_NUMBER)
			{
				Solver.ApplyJointImpulse(Batch, R.Env, R.DOF, R.Sign * Warm);
			}
		}
		for (const FBallLimitRow& R : BallLimitRows)
		{
			const float Warm = Cache.BallJointLimitImpulse[R.Slot];
			if (FMath::Abs(Warm) > UE_SMALL_NUMBER)
			{
				Solver.ApplyBallJointImpulse(Batch, R.Env, R.DOFOffset, R.ConstraintDir, Warm);
			}
		}
		for (const FActivePair& AP : ActivePairs)
		{
			const int32 Slot = AP.PairIdx * NumEnvs + AP.Env;
			FVector PT1, PT2;
			BuildTangentBasis(AP.Normal, PT1, PT2);
			const FVector Warm = Cache.PairNormalImpulse[Slot] * AP.Normal
				+ Cache.PairTangentImpulse1[Slot] * PT1
				+ Cache.PairTangentImpulse2[Slot] * PT2;
			if (!Warm.IsNearlyZero())
			{
				// Same convention as the gather phase: +Impulse on B, -Impulse on A.
				Solver.ApplyPairImpulseAtPoints(Batch, AP.Env, AP.BodyB, AP.WorldPtB, AP.BodyA, AP.WorldPtA, Warm);
			}
		}

		// ---- 1./3. iterate: biased passes, then relaxation passes ----
		// Honour 0 — it used to be clamped to at least 1, which forced a relax
		// pass on callers that had asked for none. See RelaxIterations' comment.
		const int32 RelaxIters = FMath::Max(0, Params.RelaxIterations);
		const int32 TotalIters = FMath::Max(1, Params.Iterations) + RelaxIters;

		for (int32 Iter = 0; Iter < TotalIters; ++Iter)
		{
			const bool bRelax = (Iter >= Params.Iterations); // stabilization off

			for (const FActive& A : Active)
			{
				const FVector PV = PointVelocity(A.Body, A.Env, A.WorldPoint);
				const float VN = (float)FVector::DotProduct(PV, Params.GroundNormal);

				// Separation: negative when overlapping, matching the sign
				// convention the softness math is written in. Slop leaves a little
				// overlap uncorrected so resting contacts settle instead of buzzing.
				const float Separation = -FMath::Max(0.0f, A.Penetration - Params.Slop);

				// The relax passes run the SAME solve with softness disabled -- no
				// bias, no relaxation, unit mass scale -- which is what strips the
				// energy the biased passes deliberately injected.
				float Bias = 0.0f;
				float MassScale = 1.0f;
				float ImpulseScale = 0.0f;
				if (!bRelax && Separation < 0.0f)
				{
					Bias = Soft.BiasRate * Separation;
					if (Params.MaxBiasVelocity > 0.0f)
					{
						Bias = FMath::Max(Bias, -Params.MaxBiasVelocity);
					}
					MassScale = Soft.MassScale;
					ImpulseScale = Soft.ImpulseScale;
				}
				const float RestitutionTerm = Params.Restitution * FMath::Min(0.0f, VN);

				// Soft-constraint impulse. The trailing -ImpulseScale*accumulated
				// is the regularization; without it this reduces to the hard
				// constraint the previous version solved. Params.Cfm is a
				// SECOND, independent regularization added directly to the
				// InvMassN denominator (see FImpulseContactParams::Cfm) --
				// bounds this row's own max response regardless of Bias.
				const float Delta =
					-MassScale * (VN + Bias + RestitutionTerm) / (A.InvMassN + Params.Cfm)
					- ImpulseScale * Cache.NormalImpulse[A.Slot];

				// ACCUMULATED clamp: form the new total, clamp THAT, apply the difference.
				// Params.Relaxation (SOR) scales only the INCREMENT, never the
				// already-converged part of the accumulator -- a fraction < 1
				// damps how fast this row can swing per iteration, which is
				// what actually stops it and the joint-limit row from
				// alternately overshooting each other's previous correction.
				float NewTotal = FMath::Max(0.0f, Cache.NormalImpulse[A.Slot] + Params.Relaxation * Delta);
				if (Params.MaxNormalImpulse > 0.0f)
				{
					NewTotal = FMath::Min(NewTotal, Params.MaxNormalImpulse);
				}
				const float AppliedN = NewTotal - Cache.NormalImpulse[A.Slot];
				Cache.NormalImpulse[A.Slot] = NewTotal;
				if (DebugLog && A.Body == DebugLog->WatchBody)
				{
					DebugLog->ContactNormalImpulsePerIteration.Add(NewTotal);
				}

				// Friction on the same accumulated basis, clamped to the Coulomb
				// cone of the CURRENT accumulated normal impulse.
				const FVector PV2 = PointVelocity(A.Body, A.Env, A.WorldPoint);
				const float VT1 = (float)FVector::DotProduct(PV2, T1);
				const float VT2 = (float)FVector::DotProduct(PV2, T2);
				// Same reasoning as the normal guard above: clamping at 1e-4 caps
				// the tangential effective mass at 10,000 kg and would understate
				// friction on exactly the best-braced contacts.
				float NewT1 = Cache.TangentImpulse1[A.Slot] + (-VT1 / FMath::Max(A.InvMassT1, UE_SMALL_NUMBER));
				float NewT2 = Cache.TangentImpulse2[A.Slot] + (-VT2 / FMath::Max(A.InvMassT2, UE_SMALL_NUMBER));
				const float MaxT = Params.FrictionCoefficient * NewTotal;
				const float TMag = FMath::Sqrt(NewT1 * NewT1 + NewT2 * NewT2);
				if (TMag > MaxT && TMag > KINDA_SMALL_NUMBER)
				{
					const float Scale = MaxT / TMag;
					NewT1 *= Scale;
					NewT2 *= Scale;
				}
				const float AppliedT1 = NewT1 - Cache.TangentImpulse1[A.Slot];
				const float AppliedT2 = NewT2 - Cache.TangentImpulse2[A.Slot];
				Cache.TangentImpulse1[A.Slot] = NewT1;
				Cache.TangentImpulse2[A.Slot] = NewT2;

				const FVector Applied = AppliedN * Params.GroundNormal + AppliedT1 * T1 + AppliedT2 * T2;
				if (!Applied.IsNearlyZero())
				{
					Solver.ApplyImpulseAtPoint(Batch, A.Env, A.Body, A.WorldPoint, Applied);
				}
			}

			// ---- joint-limit rows, in the SAME iteration as contact ----
			//
			// This interleaving is the whole point. Solving limits in a separate
			// pass (or, as before, as a position clamp after the fact) leaves
			// contact sizing its impulse against a leg it believes can still
			// fold. Here, each iteration lets the contact impulse and the limit
			// impulse see each other's effect, so the pair converges toward
			// "braced strut transmitting load to the torso" instead of
			// "compliant leg folding under a 17 kg foot".
			for (const FLimitRow& R : LimitRows)
			{
				const int32 DOFIdx = Batch.DOFIndex(R.DOF, R.Env);
				// d(separation)/dt: negative means closing on the stop.
				const float Cdot = R.Sign * Batch.JointVel[DOFIdx];

				float Bias = 0.0f;
				float MassScale = 1.0f;
				float ImpulseScale = 0.0f;
				if (!bRelax)
				{
					if (R.Separation > 0.0f)
					{
						// SPECULATIVE. The joint has not reached the stop yet, so
						// the row must not push -- it must only refuse to let the
						// joint travel further than the remaining gap this step.
						// With this term the row fires exactly when the joint
						// would otherwise cross, which is why it cannot be
						// tunnelled through the way the position clamp was.
						// Deliberately different from the contact rows, which
						// have no speculative branch.
						Bias = R.Separation / Dt;
					}
					else
					{
						Bias = LimitSoft.BiasRate * R.Separation;
						const float MaxBias = FMath::DegreesToRadians(JointLimits->MaxBiasVelocityDeg);
						if (MaxBias > 0.0f)
						{
							Bias = FMath::Max(Bias, -MaxBias);
						}
						MassScale = LimitSoft.MassScale;
						ImpulseScale = LimitSoft.ImpulseScale;
					}
				}

				// JointLimits->Cfm is the same independent regularization as
				// FImpulseContactParams::Cfm above, added directly to this
				// row's own InvMass -- see that field's comment for why this
				// is a distinct mechanism from the existing MassScale/
				// ImpulseScale regularization, specifically aimed at bounding
				// this row when it's fighting an active ground-contact row
				// on the SAME body (e.g. an ankle folded onto its limit).
				const float Delta = -MassScale * (Cdot + Bias) / (R.InvMass + JointLimits->Cfm)
					- ImpulseScale * Cache.JointLimitImpulse[R.Slot];

				// Accumulated clamp, non-negative: a limit can only push the
				// joint back into range, never pull it toward the stop.
				// JointLimits->Relaxation (SOR) -- see FImpulseContactParams::
				// Relaxation's comment; same mechanism, this row's increment.
				const float NewTotal = FMath::Max(0.0f, Cache.JointLimitImpulse[R.Slot] + JointLimits->Relaxation * Delta);
				const float AppliedL = NewTotal - Cache.JointLimitImpulse[R.Slot];
				Cache.JointLimitImpulse[R.Slot] = NewTotal;
				if (DebugLog && R.DOF == DebugLog->WatchDOF)
				{
					DebugLog->JointLimitImpulsePerIteration.Add(NewTotal);
				}

				if (FMath::Abs(AppliedL) > UE_SMALL_NUMBER)
				{
					Solver.ApplyJointImpulse(Batch, R.Env, R.DOF, R.Sign * AppliedL);
				}
			}

			// ---- ball-joint cone-limit rows, same iteration, same reasoning ----
			//
			// See FBallLimitRow's comment: this is what gives ball joints (the
			// spine, and every limb's mount) the same damped, speculative
			// soft-stop behavior revolute joint limits already had -- closing
			// the "perfectly frictionless pendulum between hits" gap that let
			// the spine/jaw chain build up a resonant oscillation.
			for (const FBallLimitRow& R : BallLimitRows)
			{
				const FVector JointVelVec(
					Batch.JointVel[Batch.DOFIndex(R.DOFOffset + 0, R.Env)],
					Batch.JointVel[Batch.DOFIndex(R.DOFOffset + 1, R.Env)],
					Batch.JointVel[Batch.DOFIndex(R.DOFOffset + 2, R.Env)]);
				// d(separation)/dt: negative means closing on the cone.
				const float Cdot = (float)FVector::DotProduct(R.ConstraintDir, JointVelVec);

				float Bias = 0.0f;
				float MassScale = 1.0f;
				float ImpulseScale = 0.0f;
				if (!bRelax)
				{
					if (R.Separation > 0.0f)
					{
						// SPECULATIVE, same reasoning as the revolute row above:
						// the joint hasn't reached the cone yet, so this only
						// refuses to let it travel further than the remaining
						// gap this step.
						Bias = R.Separation / Dt;
					}
					else
					{
						Bias = LimitSoft.BiasRate * R.Separation;
						const float MaxBias = FMath::DegreesToRadians(JointLimits->MaxBiasVelocityDeg);
						if (MaxBias > 0.0f)
						{
							Bias = FMath::Max(Bias, -MaxBias);
						}
						MassScale = LimitSoft.MassScale;
						ImpulseScale = LimitSoft.ImpulseScale;
					}
				}

				// Same Cfm regularization as the revolute row above.
				const float Delta = -MassScale * (Cdot + Bias) / (R.InvMass + JointLimits->Cfm)
					- ImpulseScale * Cache.BallJointLimitImpulse[R.Slot];

				// Accumulated clamp, non-negative: a limit can only push the
				// joint back toward its rest cone, never pull it further out.
				// Same SOR relaxation as the revolute row above.
				const float NewTotal = FMath::Max(0.0f, Cache.BallJointLimitImpulse[R.Slot] + JointLimits->Relaxation * Delta);
				const float AppliedL = NewTotal - Cache.BallJointLimitImpulse[R.Slot];
				Cache.BallJointLimitImpulse[R.Slot] = NewTotal;

				if (FMath::Abs(AppliedL) > UE_SMALL_NUMBER)
				{
					Solver.ApplyBallJointImpulse(Batch, R.Env, R.DOFOffset, R.ConstraintDir, AppliedL);
				}
			}

			// ---- limb-vs-limb pairs, in the SAME iteration as ground + joint limits ----
			//
			// Same coupling reasoning as the joint-limit rows just above: a limb
			// simultaneously resting on the ground AND pressed against another
			// limb needs both constraints to see each other's velocity change
			// every iteration, or one gets sized against a load path that
			// doesn't yet include the other. Mechanically identical to the
			// ground Active loop above, just body-pair shaped -- same
			// accumulated clamp, same warm-started tangent basis, same
			// Coulomb cone.
			for (const FActivePair& AP : ActivePairs)
			{
				const int32 Slot = AP.PairIdx * NumEnvs + AP.Env;
				FVector PT1, PT2;
				BuildTangentBasis(AP.Normal, PT1, PT2);

				// VN > 0 = separating (B moving away from A along Normal) --
				// see the gather phase's comment for why this pairs with
				// PairImpulseResponseAtPoint(BodyB, BodyA, ...) rather than
				// (BodyA, BodyB, ...).
				const FVector PVB = PointVelocity(AP.BodyB, AP.Env, AP.WorldPtB);
				const FVector PVA = PointVelocity(AP.BodyA, AP.Env, AP.WorldPtA);
				const float VN = (float)FVector::DotProduct(PVB - PVA, AP.Normal);

				const float Separation = -FMath::Max(0.0f, AP.Penetration - LimbCollision->Slop);

				float Bias = 0.0f;
				float MassScale = 1.0f;
				float ImpulseScale = 0.0f;
				if (!bRelax && Separation < 0.0f)
				{
					Bias = PairSoft.BiasRate * Separation;
					if (LimbCollision->MaxBiasVelocity > 0.0f)
					{
						Bias = FMath::Max(Bias, -LimbCollision->MaxBiasVelocity);
					}
					MassScale = PairSoft.MassScale;
					ImpulseScale = PairSoft.ImpulseScale;
				}
				const float RestitutionTerm = LimbCollision->Restitution * FMath::Min(0.0f, VN);

				// Same Cfm regularization as ground contact above.
				const float Delta =
					-MassScale * (VN + Bias + RestitutionTerm) / (AP.InvMassN + LimbCollision->Cfm)
					- ImpulseScale * Cache.PairNormalImpulse[Slot];

				// Same SOR relaxation as ground contact above.
				float NewTotal = FMath::Max(0.0f, Cache.PairNormalImpulse[Slot] + LimbCollision->Relaxation * Delta);
				if (LimbCollision->MaxNormalImpulse > 0.0f)
				{
					NewTotal = FMath::Min(NewTotal, LimbCollision->MaxNormalImpulse);
				}
				const float AppliedN = NewTotal - Cache.PairNormalImpulse[Slot];
				Cache.PairNormalImpulse[Slot] = NewTotal;

				const FVector PVB2 = PointVelocity(AP.BodyB, AP.Env, AP.WorldPtB);
				const FVector PVA2 = PointVelocity(AP.BodyA, AP.Env, AP.WorldPtA);
				const FVector RelVel2 = PVB2 - PVA2;
				const float VT1 = (float)FVector::DotProduct(RelVel2, PT1);
				const float VT2 = (float)FVector::DotProduct(RelVel2, PT2);
				float NewT1 = Cache.PairTangentImpulse1[Slot] + (-VT1 / FMath::Max(AP.InvMassT1, UE_SMALL_NUMBER));
				float NewT2 = Cache.PairTangentImpulse2[Slot] + (-VT2 / FMath::Max(AP.InvMassT2, UE_SMALL_NUMBER));
				const float MaxT = LimbCollision->FrictionCoefficient * NewTotal;
				const float TMag = FMath::Sqrt(NewT1 * NewT1 + NewT2 * NewT2);
				if (TMag > MaxT && TMag > KINDA_SMALL_NUMBER)
				{
					const float Scale = MaxT / TMag;
					NewT1 *= Scale;
					NewT2 *= Scale;
				}
				const float AppliedT1 = NewT1 - Cache.PairTangentImpulse1[Slot];
				const float AppliedT2 = NewT2 - Cache.PairTangentImpulse2[Slot];
				Cache.PairTangentImpulse1[Slot] = NewT1;
				Cache.PairTangentImpulse2[Slot] = NewT2;

				const FVector Applied = AppliedN * AP.Normal + AppliedT1 * PT1 + AppliedT2 * PT2;
				if (!Applied.IsNearlyZero())
				{
					// +Applied on B, -Applied on A -- matches the gather/warm-start convention.
					Solver.ApplyPairImpulseAtPoints(Batch, AP.Env, AP.BodyB, AP.WorldPtB, AP.BodyA, AP.WorldPtA, Applied);
				}
			}
		}

		} // end of the per-row solve

		// SHARED by both solve paths: the global solver writes its converged
		// lambdas back into the same accumulated-impulse cache slots, so the force
		// reported downstream is derived identically either way and the RL
		// observation/reward code does not need to know which solver ran.
		if (OutState)
		{
			for (const FActive& A : Active)
			{
				FContactPointState& St = (*OutState)[A.PointIdx * NumEnvs + A.Env];
				St.bTouching = true;
				// Reported as an equivalent average force, so observation code
				// reads the same units it did under the penalty model.
				St.NormalForce += Cache.NormalImpulse[A.Slot] / Dt;
			}
		}
	}

#if WITH_EDITOR
	/**
	 * Derives ground-contact points for Muto straight from the authored
	 * MassProfile_Muto CanTouchGround flag, one point (at the body's own
	 * origin) per body whose bone is flagged -- no structural guess about
	 * "the last N bones of the chain are the foot". This matters because the
	 * real data doesn't follow a uniform pattern: legs are touchable on
	 * {Feet, FeetTip} but NOT the shin-equivalent Knee2, while arms are
	 * touchable on {Elbow3, Hand} but NOT the Tip past the hand -- a fixed
	 * "last 3 bones" rule (the previous approach here) gets legs wrong.
	 * M limbs come out excluded automatically since every M bone has
	 * CanTouchGround=false in the authored data (matches the established
	 * fact that M limbs never touch the ground).
	 *
	 * Requires BodyDebugNames (MutoTopology::BuildMutoTopology's optional
	 * OutBodyDebugNames out-param) to map body index -> bone name for the
	 * MassAsset lookup. Per-point LimbIndex comes straight from
	 * Topo.BodyLimbIndex -- one slot per archetype x side (0..7), matching
	 * NumLimbs = Archetypes.Num() * 2 (see MutoTopology.h's LimbIndex
	 * assignment; this used to have to re-derive its own parallel counter
	 * here because that assignment only distinguished archetype, not L/R
	 * side, but that's fixed now).
	 */
	inline TArray<FContactPointDef> BuildMutoContactPoints(
		const FCreatureTopology& Topo,
		UMassMuscleProfileAssetMass* MassAsset,
		const TArray<FName>& BodyDebugNames,
		bool bAllBodiesCollideWithGround = false,
		float StructuralRadiusFallback = 10.0f)
	{
		TArray<FContactPointDef> Points;
		if (!MassAsset)
		{
			return Points;
		}

		for (int32 Body = 1; Body < Topo.NumBodies; ++Body)
		{
			if (!BodyDebugNames.IsValidIndex(Body))
			{
				continue;
			}
			const int32 MassIdx = MassAsset->FindBoneByName(BodyDebugNames[Body]);
			if (MassIdx != INDEX_NONE && MassAsset->Mass[MassIdx].CanTouchGround)
			{
				// LocalOffset = the fused Tip's rest offset if this body has
				// one (ZeroVector otherwise) -- Feet/Hand's own joint origin
				// isn't the visual extent of the foot/hand, the fused,
				// unarticulated Tip bone hanging past it is (see
				// FCreatureTopology::BodyFusedTipOffset's comment). Radius =
				// the authored per-bone collision radius (see
				// FMassMuscleDataMass::Radius) -- REPLACES the old computed
				// "ground-alignment correction" heuristic entirely (see
				// project memory/roadmap for that saga): rather than
				// forcing every point to reach an artificially-equalized
				// height, each point now uses real, author-provided
				// geometry (LocalOffset reaches to the bone's visual end,
				// Radius accounts for its thickness) -- if that data is
				// authored accurately, every touching limb naturally lands
				// at the same height because that's what the real mesh
				// does, not because the code forced it to. CapsuleHalfHeight
				// (see FMassMuscleDataMass::CapsuleHalfHeight) extends this
				// into a real capsule back up the limb instead of a single
				// point at the tip.
				Points.Add({ Body, Topo.BodyFusedTipOffset[Body], BodyDebugNames[Body], Topo.BodyLimbIndex[Body], Topo.BodyRadius[Body], Topo.BodyCapsuleHalfHeight[Body] });
			}
		}

		// ---- Structural ground collision on every remaining body ----
		//
		// GROUND ONLY. There is no self-collision here and none is implied:
		// ResolveGroundContactImpulses tests against a plane at GroundZ and
		// nothing else, so adding points cannot make limbs collide with each
		// other. Limb-vs-limb needs a broadphase and pairwise tests that do not
		// exist in this file.
		//
		// Why: the authored CanTouchGround flags cover feet, hands and elbows
		// only. Everything else -- crucially the TORSO, which is body 0 and
		// carries 3282 kg of the creature's 6170 -- passes through the floor
		// completely unopposed. Once the legs were fixed enough to stop being
		// first through the ground, the pelvis simply became first instead, and
		// dragged the whole rig down behind it with no contact anywhere to
		// object (user-observed, 2026-08-14). A solver cannot brace against
		// geometry it was never told about.
		//
		// One sphere at each body's own ORIGIN, i.e. at the joint. That is
		// literally what was asked for ("collision for all joints"), and it is
		// the right primitive for this failure: it is joints being dragged
		// under, not bone midpoints. A capsule down each bone would cover more
		// but needs a per-bone extent that is only authored for the limbs that
		// already have points.
		//
		// Bodies that ALREADY have an authored point are skipped rather than
		// doubled up. Their geometry was tuned by hand, and stacking a second
		// row at nearly the same place is a good way to make a resting contact
		// jitter -- two rows fighting over the same load is exactly what
		// accumulated-impulse clamping is bad at.
		if (bAllBodiesCollideWithGround)
		{
			TArray<bool> HasAuthoredPoint;
			HasAuthoredPoint.Init(false, Topo.NumBodies);
			for (const FContactPointDef& P : Points)
			{
				HasAuthoredPoint[P.BodyIndex] = true;
			}

			for (int32 Body = 0; Body < Topo.NumBodies; ++Body)
			{
				if (HasAuthoredPoint[Body])
				{
					continue;
				}
				// Authored radius where the asset has one; otherwise a fallback,
				// because a zero-radius point sits exactly on the joint origin
				// and lets the visible mesh around it sink half its thickness
				// before anything reacts. Authoring real radii is strictly
				// better than tuning this number.
				const float Radius = Topo.BodyRadius[Body] > 0.0f ? Topo.BodyRadius[Body] : StructuralRadiusFallback;

				// Body 0's debug name is the synthetic "Torso" label, not a
				// bone -- name the point for what it actually is so a contact
				// trace stays readable.
				const FString BaseName = (Body == 0)
					? FString(TEXT("Torso"))
					: (BodyDebugNames.IsValidIndex(Body) ? BodyDebugNames[Body].ToString() : FString::Printf(TEXT("body%d"), Body));
				const FName Name(*(BaseName + TEXT("_structural")));

				Points.Add({ Body, FVector::ZeroVector, Name, Topo.BodyLimbIndex[Body], Radius, 0.0f });
			}
		}

		return Points;
	}

	/**
	 * Candidate limb-vs-limb collision pairs for Muto: every pair of
	 * DIFFERENT limbs' bodies that both have authored collision geometry
	 * (Topo.BodyRadius > 0). Same-limb pairs are never generated (same
	 * BodyLimbIndex is skipped) -- a knee never tests against its own leg's
	 * other knee.
	 *
	 * LIMB-VS-LIMB ONLY, deliberately -- the spine/head/jaw chain
	 * (Topo.BodyLimbIndex == INDEX_NONE) is entirely excluded from
	 * candidacy, not just from same-limb-style adjacency. This used to be
	 * broader (2026-08-16: briefly generalized to also pair limbs against
	 * the newly-independently-articulated spine, and spine bodies against
	 * each other, once the fused-torso special case went away) but was
	 * reverted the same day: pairing every limb against the spine took the
	 * candidate count from ~503 to 701, all sharing the same Gauss-Seidel
	 * budget as ground contact + joint limits, and two of the new spine
	 * bodies (UpperMouth, Chin) turned out to have NO authored mass/inertia
	 * either (silently defaulting to a placeholder ~800x lighter than their
	 * real neighbors) -- that combination reproducibly blew up a passive,
	 * zero-torque ragdoll drop within 3 substeps (confirmed via a throwaway
	 * diagnostic matching a real user-reported crash to within 0.01% on the
	 * exact body/velocity/force numbers).
	 *
	 * The revert alone was NOT sufficient, though: even with the candidate
	 * count back at ~503 (limb-vs-limb only, no spine), the same drop still
	 * blew up later (t=0.92s instead of t=0.0125s, body FHand_R) at the
	 * then-default 8 Gauss-Seidel iterations -- a genuine convergence-
	 * starvation problem, not a bug in this pairing/exclusion logic or the
	 * impulse math (both independently verified correct). An iteration
	 * sweep on the same reproduction (AgentSolver.TEMP.LimbCollisionRevertVerify)
	 * showed 8 iterations blows up but 16/32/64 all ran the full 1s drop
	 * clean, so AMutoRLTrainingDriver::ContactIterations' default was raised
	 * 8 -> 16 the same day. Revisit broadening spine-vs-limb pairing again
	 * only once UpperMouth/Chin get real mass/inertia authored -- the
	 * iteration budget alone does not make that combination safe.
	 *
	 * Mid-limb segments without authored radius (shoulders, upper arm/leg
	 * links) are deliberately NOT given a fallback sphere here the way
	 * BuildMutoContactPoints' bAllBodiesCollideWithGround option does for
	 * ground -- a smaller, cheaper candidate list was chosen over full
	 * coverage.
	 *
	 * Static, topology-only -- call once (e.g. alongside BuildMutoContactPoints
	 * in StartTraining()), not per-step. Per-step broadphase (are two bodies'
	 * capsules actually overlapping right now) happens inside
	 * ResolveGroundContactImpulses, since poses change every substep.
	 */
	inline TArray<FLimbPairDef> BuildMutoLimbCollisionPairs(const FCreatureTopology& Topo)
	{
		TArray<FLimbPairDef> Pairs;
		TArray<int32> Candidates;
		for (int32 Body = 1; Body < Topo.NumBodies; ++Body)
		{
			if (Topo.BodyRadius[Body] > 0.0f && Topo.BodyLimbIndex[Body] != INDEX_NONE)
			{
				Candidates.Add(Body);
			}
		}
		for (int32 i = 0; i < Candidates.Num(); ++i)
		{
			for (int32 j = i + 1; j < Candidates.Num(); ++j)
			{
				const int32 A = Candidates[i];
				const int32 B = Candidates[j];
				if (Topo.BodyLimbIndex[A] != Topo.BodyLimbIndex[B])
				{
					Pairs.Add({ A, B });
				}
			}
		}
		return Pairs;
	}
#endif // WITH_EDITOR
}
