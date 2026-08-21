# AgentSolver — Open Items Register

Companion to `SOLVER_DEBUG_LOG.md`. That file records *hypotheses tested*; this one
records *what is currently left*: code that is built but never called, features wired
but switched off, data authored but never read, and problems measured but not resolved.

Items are addressable by ID (`U-01`, `O-04`, …) so they can be cited from commits and
from log entries. **Once an item's ID is used, it is never reused** — resolved items
stay in the table below rather than being deleted, because the reasoning is the value.

- **First audit:** 2026-08-17, by reading the full `AgentSolver` module (45 files),
  the `MassMuscleProfile` data structures, and `SOLVER_DEBUG_LOG.md` entries 001–025.
- **Revised:** 2026-08-21, after entry 026 landed and after the UAsset network
  persistence was added to the driver. Every still-open item was re-verified against
  the current source.
- **Revised again:** 2026-08-21, after entry 027 closed `U-07`, `U-08`, `U-10`, `U-11`,
  `D-02` and `R-05` and opened `X-08`. Verified by build + a full automation run:
  **22 tests, 0 failures** (24 before — the two removed tested the deleted
  external-wrench path). The real-rig passive drop reproduced entry 026's table
  digit-for-digit, so none of that moved the physics that was measured.
- **Revised again:** 2026-08-21, after entry 028 replaced the thin-rod inertia
  placeholder with a capsule tensor derived from the authored collision radius,
  narrowing `A-01` and half-closing `A-03`. 22 tests, 0 failures. **This one did move
  the physics** — in the good direction (resting penetration −27%, max joint speed
  −51%, marginal configurations surviving 2× longer).
- **Revised again:** 2026-08-21, after entry 029 re-fitted the contact constants and
  closed `O-07`. `ContactHertz` 15 → 45. Opened `O-13` and `O-14`, made `O-06`
  deliberately worse, and quantified `X-08`. 13/13 affected tests pass.

Provenance tags: `code` read directly in source · `log` from the debug log's own
measurements · `unverified` depends on `.uasset` data or runtime behaviour not checked.

A presentation copy with the same IDs is at
https://claude.ai/code/artifact/e7caac2b-2809-490d-8de5-57e856960c09

---

## Current state, in three sentences

The 41-body rig survives 50 seconds of sustained ground contact with the torso drifting
0.2 cm over the last 40 — the first time this has been true for *this* rig, since entry
025's stability result was measured on the 35-body rig that predates the spine
articulation. Stability required **armature + damping + welding together**; no single
addition achieves it, and welding alone is worse than baseline.

The largest remaining gap is not in the solver: **nothing has ever tested whether the
creature can stand** (`O-08`). Every stability number on record is a passive drop with
zero actuation, which is a worst case and not the training case.

That gap widened on 2026-08-21. Entry 027 connected the authored per-muscle strengths to
the solver and found them to be anything but neutral — 44 of 67 DOFs now deliver between
0.5× and 5× the configured torque limit (`X-08`). The passive drop cannot see this at
all, because it commands zero torque for the multiplier to scale.

Entry 028 then replaced the inertia placeholder with a capsule tensor derived from the
authored collision radius, and entry 029 re-fitted the contact constants on top of it.

That re-fit produced the sharpest methodological finding in the register: **a passive
drop cannot fit contact constants at all.** All 24 candidate pairs survive it. Only
under actuation do they separate — and there the shipped pair of the last three months
turns out to diverge at 35.55 s. Every contact sweep in this project's history used the
regime that cannot tell them apart.

---

## Resolved

| ID | What it was | What closed it |
|---|---|---|
| R-01 | Interior bodies got two coincident ground contact rows at one point, because a zero `LocalOffset` normalises to the zero vector. Fired on `FElbow3_*`/`BElbow3_*`, every substep | `GetCapsuleLocalEnds` is now the one derivation shared by all four call sites, and returns 1 end when the two coincide |
| R-02 | The balance reward placed centre of pressure at joint origins for the 25 structural points, ignoring radius and capsule geometry | Uses the same world-surface derivation that produced the force it weights by |
| U-01 | `FIterationDebugLog` had zero callers, including tests | Records the per-iteration constraint residual; driver exposes `WatchContactBody`, `WatchJointLimitDOF`, `LogSolverResidualEverySubsteps` |
| U-02 | Joint welding — entry 024's 14.8× measurement — was only ever called from a diagnostic | `BuildSaturatedJointLocks` is the production caller; lock array is now per-body-per-env, and only joints being *driven into* a stop are welded |
| U-03 | `Cfm` on all three row types, no caller, no editor property | Exposed on the driver, default 1e-8 |
| U-04 | `Relaxation` (SOR), same | Exposed on the driver, default 1.0 |
| U-05 | `MaxNormalImpulse` unreachable; `MaxBiasVelocity` reachable only as a hardcoded default | Both exposed |
| D-01 | Limb collision complete, tested, and off — while `ContactIterations` had already been doubled to pay for it | Default **on** |
| O-04 | Per-row Gauss-Seidel refuted by entry 024's non-monotonic sweep, with no structural response | Global constraint solve: `A = J M⁻¹ Jᵀ + R` assembled over every active row, projected SOR in constraint space. **See the attribution note below — this is not what fixed the divergence** |
| P-01 | Contact cost unmeasured since entry 025 flagged it | 2.161 ms/substep single-env at shipped defaults, vs 1.964 baseline and 3.391 for per-row-with-everything |
| P-02 | Every row's diagonal recovered by its own whole-tree solve, discarding the rest | Rows now contribute whole response columns; the off-diagonal coupling is kept |
| N-01 | The debug log stopped at entry 025 and was wrong on four counts | Entry 026 |
| N-02 | Test counts stale | Now **22 tests, 11 permanent and 11 `TEMP`**, all passing. (Also corrects this register's own first pass, which said 23/12 — it wrongly counted `Muto.JointAxisAudit` as TEMP when entry 022 deliberately promoted it out) |

### Closed by entry 027 (2026-08-21)

| ID | What it was | What closed it |
|---|---|---|
| U-07 | Domain randomization complete, consumed by both step variants, and never invoked — all 256 envs trained on an identical, undamaged, unloaded creature | `FDomainRandomization` drawn by `ResetEnv` every episode; driver properties under `Muto RL|Reset|Domain Randomization`. Off by default. Also fixed a latent bug: `FRand() > LimbLossChance` could disable a limb at chance 0, since `FRand()` is `[0,1)` |
| U-08 | External-wrench path loaded by both step variants every step, cleared by the driver every substep, written only by tests | Deleted: `ApplyForceAtPoint`, `ClearExternalForces`, six per-body-per-env arrays, both `HitWrench` loads, the clear loop. **The pendulum energy test was using it structurally**, so `FCreatureTopology::BodyGravityScale` replaces it — one per-body float for six per-body-per-env arrays, free in the SIMD inner loop |
| U-10 | Muscle scalar strengths editable, mirrored, saved — and ignored | Multiplied into the muscle multiplier alongside the curve. **Not a no-op: 44 of 67 DOFs are non-unit, range [0.5, 5.0]** — see the new `X-08` |
| U-11 | `FMassMuscleDataMass::BoneIndex` write-only, free to go stale after a mesh re-import with no symptom | Now the lookup key. `SyncBoneIndices()` repairs it in `PostLoad`; `FindBoneByIndex` added; `FCreatureTopology::BodyBoneIndex` threaded through `BuildMutoTopology` and `BuildMutoContactPoints`; `AgentSolver.MutoTopology` asserts the index and the debug name agree |
| D-02 | Joint speed limiting off, ordering never validated | Deleted rather than fixed — it clipped the velocities the constraint solve had just converged on |
| R-05 | The above, stated as a risk: enabling `D-02` would silently override the constraint solve | Gone with `D-02` |

### Closed by entry 029 (2026-08-21)

| ID | What it was | What closed it |
|---|---|---|
| O-07 | Contact constants fitted to a rig with wrong joint axes, no torso collision and thin-rod inertia — and, it turned out, fitted on a regime that cannot discriminate them at all | Re-fit: `ContactHertz` 15 → **45**, `ContactDampingRatio` unchanged at **10**. Chosen over 24 pairs × 2 regimes by a rule fixed before the numbers existed, gated on 50 s survival in both. The old pair **diverges at 35.55 s** under sustained actuation; the new one survives. Resting penetration 1.90 → 0.83 cm. Cost: peak penetration 18.89 → 47.95 (`O-06`), and two new items, `O-13` and `O-14` |

### Two real bugs found while implementing the above

Both were caught by tests asserting a physical law rather than a tolerance, which is
why they are worth remembering as a technique.

**Ball-joint reduced bias was wrong.** `SolveImpulseResponse` hardcoded the reduced
bias's angular part to `ZeroVector`. Featherstone gives
`p^a.Ang = ImpP.Ang + Irot·D⁻¹·u`, which collapses to `JointImp3` — zero only when the
joint impulse is zero, i.e. only for the body-impulse-only case the function originally
supported. Once ball-joint cone rows started passing a real joint impulse through that
branch (2026-08-16), every hip and spine limit silently discarded the reaction it owed
its parent, so those limits braced against their own child subtree only. Caught by
asserting that an impulse *internal* to a floating-base tree cannot change total angular
momentum; now conserved to 1.4e-08.

**Damping cannot be a torque in an articulated tree.** The first implementation followed
MuJoCo literally — `-d·qd` folded into tau — and was measured *reversing* joints: a
requested 10% velocity reduction removed 197%, and `T = dt` overshot by 19×. The parent
recoils from the torque and amplifies the joint's response by a factor that depends on
the whole subtree and is unbounded as the parent gets lighter. Worse, the stability cap
keyed on `D` was actively anti-conservative, since `1/D` is a *lower* bound on the gain.
Reimplemented as an impulse that *measures* the gain via `SolveImpulseResponse`. The
same trap invalidates the naive expectation that `qddot` scales as `1/(1+armature)` on a
floating base — measured 20×, not 2×; that law holds only against a near-fixed base.

### Attribution note on O-04

The global solve is **not** what fixed the divergence. The per-row path with the same
other additions (`ALL row`) also survives. What the global solve buys, measured against
that: peak penetration 24.83 → 15.34 (1.6× better), torso held at 108.68 instead of
74.52 (47% higher, i.e. far less collapsed), and **1.57× cheaper** while running 64
sweeps against the per-row path's 16 — because a dense sweep costs no tree traversals.

It is also projected SOR, not MuJoCo's Newton on the convex dual. Assembling `A` was the
structural change; swapping the sweep for Newton is now a self-contained change to one
loop.

---

## U — Dead code: implemented, reachable, never called

### U-06 · Restitution is structurally present but never non-zero · `code`

Both contact row types compute a `RestitutionTerm`. Default 0, and **re-verified
2026-08-21: zero assignments anywhere in the module**. Correct for locomotion, but it
means that branch of the impulse formula has never executed in any run or test. Untested
code if ever enabled.

`Public/CreatureGroundContact.h` — `FImpulseContactParams::Restitution`,
`FLimbCollisionParams::Restitution`.

### U-09 · DebugArticulatedInertia read by one TEMP test · `code`

Public solver API kept alive for the entry-020 conditioning audit, whose question the log
records as answered. Re-verified: still no non-test caller.

---

## D — Dormant: wired end-to-end, off by default

### D-03 · The relaxation pass is correctly disabled, so one of entry 016's three techniques contributes nothing · `code` `log`

`ContactRelaxIterations = 0` is right for the current ordering — Box2D relaxes *after*
position integration, contact here resolves after `Step()` has already integrated
position, so a relax pass strips the bias velocity while it has moved nothing (relax=2 →
8.08 cm deep and stays there; relax=0 → 0.58). Of accumulated clamping / warm starting /
relaxation, the third is structurally unavailable. The ordering change it needs — resolve
contact between the velocity and position integrations — has not been attempted.

### D-04 · Resume-from-snapshot exists but is off · `code`

`bLoadSnapshotOnStart = false` while `bAutoSaveOnEndPlay = true` and the 300 s periodic
autosave are on — so runs are saved but never resumed, which quietly discards progress
across sessions.

### D-05 · Intentional fallbacks, no action · `code`

`TargetTorsoHeightOverride = -1` (auto-derive), `bUseSocketCommunicator = false`.

### D-06 · UAsset network persistence is manual-only · `code` — NEW 2026-08-21

`SaveTrainedNetworksToAssets` / `LoadTrainedNetworksFromAssets` plus
`EncoderNetworkAsset` / `PolicyNetworkAsset` / `DecoderNetworkAsset` were added to the
driver, giving versionable, content-browser-visible weight assets instead of loose files
under `Saved/`. Good, and it addresses half of `X-06`.

But all three **automatic** persistence paths still use the raw snapshot files:
`bLoadSnapshotOnStart` (`MutoRLTrainingDriver.cpp:458`), the periodic autosave
(`:657`) and `bAutoSaveOnEndPlay` (`:861`). The asset path is reachable only from the two
`CallInEditor` buttons. So an unattended run still persists only to loose files, and the
hard-crash safety net that motivated the periodic autosave does not benefit from the new
assets at all.

---

## A — Placeholder data with no real source

Mass was authored for real (43 kg → 6170 kg, entry 017). Almost nothing else was. **None
of this group has changed.**

### A-01 · Inertia is derived, not authored · `code` — NARROWED 2026-08-21

**Entry 028 removed the magic number.** `BodyInertiaDiagLocal` is now a solid-capsule
tensor built from three authored quantities — the bone's mass, its collision `Radius`,
and its rest-pose length — rather than a thin rod whose zero axial term was faked as
`0.05 × IPerp`. That fudge implicitly claimed every bone was `L/sqrt(120)` = 9.1% as
thick as it was long; the authored radii disagree by up to 9× in both directions, and
axial inertia moved by 77.5× on `Back3` and 0.35× on `Feet_L`.

**What is still open:** nobody has authored an inertia *tensor*. The shape is still an
assumption — a capsule, uniform density — and the mass asset has no inertia or density
field. This still propagates into the ABA `D` term, every articulated effective mass and
therefore every contact impulse, so armature remains a global scale layered on top of a
derived quantity. It is no longer a *placeholder*, which is the difference.

The one body that still takes the legacy path is `Pelvis`, which has no authored radius
because it has no ground-contact geometry. `BuildMutoTopology` now warns when that
happens instead of substituting silently.

### A-02 · CoM is half the bone length by assumption · `code`

`OwnCoM = 0.5 × LengthVec`. Reasonable for a rod; not a measurement, not authorable.

### A-03 · Inertia and contact use different lever arms on the same body · `log`

Inertia is built from `BodyLength` (offset to the body's own child); the contact point sits
at `BodyFusedTipOffset`. On Feet: 106.4 cm vs ~70 cm, ratio 0.44. Entry 021 recorded this
as a modelling inconsistency, explicitly not a unit error. Never addressed.

**Half-closed 2026-08-21.** The two models now agree about a body's *thickness* — entry
028 made inertia read the same authored `Radius` collision reads. They still disagree
about its *length*, and deliberately: `BodyCapsuleHalfHeight` defaults to 0, collapsing
the collision capsule to a sphere at the tip, while a bone's mass genuinely spans its
whole segment. Making inertia use the collision length would give most bones a point-like
mass distribution — worse than the rod it replaced. The real fix is authoring
`CapsuleHalfHeight` (`A-06`), after which the two lengths converge on their own.

### A-04 · Off-diagonal inertia discarded at authoring time · `code`

`FCreatureTopology` stores inertia as an `FVector` diagonal, so the Tip-fusion
parallel-axis shift computes a real tensor and keeps only its diagonal. The stated
justification — off-diagonal terms stay small while the limb axis is roughly aligned with
the offsets — is never checked, and degrades exactly where a Tip hangs off-axis.

### A-05 · UpperMouth and Chin still have no authored mass — this blocks O-09 · `code`

Both are recorded in-source as having no authored mass or inertia, defaulting to a
placeholder ~800× lighter than their real neighbours. They are no longer independent ABA
bodies, so the acute failure is gone, but the mass they contribute to Head2 and LowerMouth
is still that default. `BuildMutoLimbCollisionPairs` names this as the explicit
precondition for re-enabling spine-vs-limb pairing. Unmet.

### A-06 · Collision geometry is hand-authored, coverage unknown · `code` `unverified`

`Radius` (default 15) and `CapsuleHalfHeight` (default 0) stand in for a real
PhysicsAsset, which cannot be read from this module at all (the AVX2 minspec conflicts
with Chaos). Which bones have tuned values lives in the `.uasset` and was not inspected.
`StructuralContactRadius = 10` backfills for ground contact; limb collision has no
fallback (`O-11`). **Probably the highest-leverage non-code change available** — contact
height, penetration depth and limb-collision coverage all derive from this geometry.

**The stakes went up on 2026-08-21.** Entry 028 made `Radius` drive INERTIA as well. A
wrong radius used to produce a visibly wrong contact point; it now also silently produces
a wrong mass matrix. The radii were partially inspected for the first time in the process
and one thing stands out: `Back3` is authored at r/L = 0.85 and `Back1` at 0.685, i.e.
those spine bodies are spheres rather than bones. Whether that is the intended collision
volume or an artefact of authoring purely for contact has never been checked — and it now
decides how hard the torso is to rotate.

### A-07 · Body 0's `BodyRestRotInParent` slot means something different from every other index · `code`

Index 0 holds Pelvis's own bind-pose rotation in *component* space, because the FK pass
starts at body 1 and never reads it. Three call sites depend on it. Documented in three
places, which is the right mitigation, but still a trap for generic code iterating from 0.

---

## O — Open physics problems

### O-01 · Joint wind-up is still only bounded modulo 360° · `code` `log`

`ClampAngleDeg` does `Fmod(AngleDeg − MinDeg, 360)`, so a joint that spins a full lap past
its limit re-enters the "valid" window and is accepted. The clamp bounds the residue, never
the accumulated rotation; entry 023 measured joints wound past −8000°. The 2026-08-17 fix
corrected a spurious −360° correction, not the underlying gap, and the velocity-level limit
rows wrap the same way. Entry 024 measured 571 lap skips *after* the rows landed, down from
1522, and said it plainly: "Do not describe this as tunnel-proof."

MuJoCo does not have this class of bug at all — its hinge coordinates are unbounded reals
and limits apply to the raw value.

### O-02 · Ball-joint limits are one uniform cone sized by the widest authored axis · `code`

`ClampJointLimits` clamps the rotation vector's *magnitude* only, with the cone half-angle
taken as the **largest** of the three authored per-axis widths — so a hip authored with a
20° roll range and a 90° pitch range is permitted 90° in every direction, roll included.
The velocity-level cone rows inherit the same approximation.

**Not a gap versus MuJoCo:** its ball joint takes a single max-angle range too, and the
standard workaround there is to model the joint as three hinges when independent swing
ranges are needed.

### O-03 · Two joint-limit mechanisms with opposite semantics run every substep · `code` `log`

The constraint rows transmit a limit's load up the chain; `ClampJointLimits` zeroes
`JointVel`, destroying that momentum. **Re-verified 2026-08-21: both still run every
substep on the same DOFs** (the clamp is still called from both `StepScalar` and
`StepSIMD`, and still zeroes velocity in both the revolute and ball branches). Entry 024
flagged the conflict and asked for a measurement; none was made. `U-01` is now wired, so
the instrument to settle it exists.

### O-05 · Worse as the timestep shrinks; the leading explanation was never checked · `log`

At matched work: 120 Hz × 16 → 1.417 s, 240 × 8 → 2.946 s, 480 × 4 → 1.035 s,
1920 × 1 → 0.938 s. An interior optimum with monotonic degradation above it; a convergent
method cannot degrade under refinement. Entry 020 refuted the epsilon hypothesis and
proposed a better one — soft-constraint `impulseScale` reaches 0.50 at 1920 Hz versus 0.11
at 240 Hz, so each iteration discards half the accumulated impulse at fine steps while the
iteration count never rises to compensate — and flagged it as "the next thing to test."
Never run. **Worth re-testing now**, because the global solve's sweep count is cheap enough
to compensate, which would confirm or kill the explanation.

### O-06 · Peak penetration at impact is still large · `log`

**47.95 cm** at the shipped defaults after entry 029's re-fit — up from 18.89 (entry 028),
15.34 (entry 026) and 20.69 (entry 025, on a different rig).

**This is the first time a shipped change has made this item worse on purpose.** Entry
029 traded peak penetration for stability under load: the old constants diverged at
35.55 s of sustained actuation, the new ones survive 50 s and more than halve resting
penetration (1.90 → 0.83 cm). A transient the rig recovers from beats a divergence it
does not — but the transient is real, and it is now the largest it has ever been.

See also `O-14`: the peak is no longer even set at impact.

Note the direction: entry 028 made the peak *worse* (+23%) while making resting
penetration better (2.60 → 1.90) and peak speed better (−28%). More inertia means the
impact carries more momentum into the first substep for the same closing speed. This is
the clearest case yet for MuJoCo's `solimp` depth-dependent impedance — stiffening with
penetration depth attacks exactly the transient without touching the resting behaviour
that is already good. Still not implemented.

### O-08 · "Can it stand up" has never been answered on this rig · `log`

Every stability result for the current code is a *passive* drop with zero actuation. The
last actuated results are entries 017/018 — a PD stand diverging at 1.03–1.59 s — and those
predate the joint-axis fix, torso collision, ball cone rows, spine articulation, and
everything in entry 026. **The project's actual objective is unmeasured.** Note entry 018
also measured a PD controller diverging on its own at Kp = 5e8 with contact disabled, so a
new actuated test needs its controller validated first, or it will measure the harness.

### O-09 · Limbs pass through the torso and spine · `code`

`BuildMutoLimbCollisionPairs` excludes every body with `BodyLimbIndex == INDEX_NONE` — the
whole spine, head and jaw chain. Pairing was broadened on 2026-08-16 and reverted the same
day: candidates 503 → 701 on an unchanged budget, plus `A-05`'s unauthored masses, blew up
a zero-torque drop within 3 substeps. Both preconditions still open.

### O-10 · A limb can fold through itself · `code`

Pairs are generated only where `BodyLimbIndex[A] != BodyLimbIndex[B]`, so no two bodies
within one limb ever test against each other.

### O-11 · Mid-limb segments have no limb-collision geometry · `code`

Candidacy requires `BodyRadius > 0` with no fallback — a deliberate choice of a smaller,
cheaper candidate list over full coverage. Shoulders and the upper arm and leg links are
absent by construction, unlike ground contact which backfills via
`StructuralRadiusFallback`.

### O-12 · Limb collision is the most fragile switch in the system · `log` — NEW 2026-08-21

Entry 026 measured it making things **worse** in isolation: `glob+pas+lim` diverges at
2.35 s against `glob+passive`'s 2.68 s, and `row+limbcol` reaches only 1.25 s against a
1.03 s baseline. It is only safe once welding is present. It is now on by default, which is
the right call given it survives in the full configuration, but it should be the first
thing switched off when diagnosing a new instability.
### O-13 · The joint-stop stiffness gap has nearly closed, and cannot be reopened · `code` `log` — NEW 2026-08-21

`ContactHertz` is clamped internally to `0.25/SubstepDt`, which at the shipped
`PhysicsSubstepDt = 1/240` is exactly **60 Hz**. `JointLimitHertz` defaults to 60 — it has
been sitting *on* that ceiling all along, which nothing recorded and no test checks.

Its comment says joint stops are "higher than ContactHertz on purpose: a joint's end of
travel is harder than the ground". That was a 4× gap when contact was 15 Hz. Entry 029's
re-fit to 45 makes it **1.33×**, and there is no headroom left — the limit cannot be
raised to restore the ordering, because it is already clamped.

The ordering still holds, so nothing is broken today. What is gone is the margin: any
future increase to `ContactHertz` inverts it silently. Buying more requires a smaller
`PhysicsSubstepDt`, which costs substeps everywhere.

### O-14 · A late transient appeared in the passive drop, and is not explained · `log` — NEW 2026-08-21

At the re-fit constants the 50 s passive confirmation no longer sets its peak penetration
at impact. It reads 23.72 cm through t = 20 s, then **47.95 cm by t = 30 s**, and the torso
finishes higher (95.15) than it sat for most of the run. At the old constants it was flat
at 18.89 throughout.

It recovers, it never diverges, and it survives the full 50 s — but a passive rig under
gravity alone should get monotonically quieter, not louder at t = 25 s. Something is
slowly accumulating and then releasing. Candidates not yet distinguished: a slow slide
into a new contact set, a joint walking to a stop and welding, or friction drift. Nothing
has been instrumented to tell them apart.

---

## R — Risks found by reading, not in the debug log

### R-03 · Observations span ~8 orders of magnitude with no normalization here · `code` `unverified`

The observation vector interleaves `TorsoUp` components in [−1, 1], a height error in cm,
joint angles in radians that `O-01` permits to reach ±140, and raw `NormalForce` values
entry 007 measured in the 1e6–1e7 range. No scaling is applied in `ComputeObservations`.
Whether Learning Agents' encoder normalizes internally was not verified. If it does not,
the force channels dominate the input scale entirely and the upright signal the reward is
built on is numerically invisible to the policy. **Cheap to check and potentially decisive
for training** — arguably the highest-value item in this whole register that nobody has
looked at.

### R-04 · Contact resolution is serial across envs, and cannot be parallelized as written · `code`

**Re-verified 2026-08-21: still no `ParallelFor` in `CreatureGroundContact.h`** (the only
occurrence of the word is a comment noting this). The force path is `ParallelFor`-ed over
8-wide env chunks in all four passes; the contact path throws all of that away.

It also cannot simply be wrapped: `SolveImpulseResponse` writes to solver members sized for
one env (`ImpP`, `ImpU`, `ImpUVec`) and every entry point above it shares `DVScratch`,
`DQScratch`, `JointImpScratch`. Two envs solving concurrently would corrupt each other.

The global solve made this *closer*: constraint rows are now gathered and solved per env in
independent blocks, so the only remaining blocker is that shared scratch. Giving
`SolveImpulseResponse` per-env scratch would unlock a `ParallelFor` over the dominant cost
in the training loop.

### R-06 · Five TEMP call sites still run the configuration entry 021 called wrong · `code`

Entry 021 established `RelaxIterations > 0` is wrong for this step ordering and set the
default to 0. **Four sites still set it explicitly** — `MutoImpulseContactTest.cpp:223`
and `:256`, `MutoIsolatedLimbTest.cpp:326`, `MutoThresholdAuditTest.cpp:102`. Any number
those diagnostics report about resting penetration or settling is measured under a
known-bad relaxation setting.

Was five; the fifth went away with `MutoWrenchPropagationTest.cpp`, which entry 027
deleted along with the external-wrench path it tested. That is attrition, not a fix.

### R-07 · Float32 throughout, on quantities spanning 1 to 1e10 · `code` `log`

Everything is `float`: state arrays, `FMat3`, `FSpatialInertia`, the 6×6 root solve. Entry
020 measured `min |Det(Irot)| = 4.63e10`, and body masses range from ~1 kg at the tips to
3282 kg at the torso. Single precision carries ~7 significant digits, and Pass 2 repeatedly
forms `I − U Uᵀ/D` — accumulate-and-subtract on quantities of very different magnitude,
which is where float32 cancellation lives. Entry 020 audited *thresholds* and correctly
refuted the epsilon hypothesis; it did not audit *precision*. MuJoCo's reference
implementation is float64 for exactly this class of reason.

Partially mitigated by armature, which raises `D` and improves conditioning — but that is a
side effect, not a fix, and the ball-joint reduction was deliberately written via
cancellation-free identities precisely because this regime is tight.

---

## P — Cost and structure

### P-03 · The limb-pair path heap-allocates four arrays per call, inside the iteration loop · `code`

`PairImpulseResponseAtPoint` and `ApplyPairImpulseAtPoints` each declare four local
`TArray`s per call, because the shared scratch is single-query and both results are needed
simultaneously to superpose them. Re-verified: still there. Note the global solve path does
*not* hit these (it builds pair response columns directly), so this now costs only when
`bUseGlobalConstraintSolve` is off.

### P-04 · A third of the tree runs lane-scalar inside `StepSIMD` · `code`

Ball joints' Pass 2 reduction and Pass 3b integration both fall back to a per-env scalar
loop; only their Pass 1 kinematics is vectorized. Muto has 14 ball joints of 41 bodies and
18 of 68 DOF, each ball body costing a 3×3 inverse per pass. Flagged as a follow-up in the
class comment; the spine articulation made it substantially more expensive without
revisiting it, and armature added two more 3×3 products to that same branch.

### P-05 · Limb collision is an O(n²) static list, fully re-tested every substep · `code`

~503 candidate pairs, each getting a full segment-segment closest-point computation every
substep with no AABB pre-pass and no spatial partitioning. Now that limb collision is on by
default (`D-01`), this cost is being paid in every run.

### P-06 · The force path has a correctness oracle; the impulse path has none · `code`

`StepScalar` is a full duplicate of the ABA math kept permanently as the SIMD reference, and
it has caught real bugs. The impulse machinery has no equivalent second implementation.
Partially improved: `AgentSolver.SolverUpgrade` now asserts momentum conservation through
the joint-space impulse path, which is what caught the ball-joint reduced-bias bug — a
physical-law check is a weaker but much cheaper oracle than a second implementation.

---

## X — Reinforcement-learning gaps

### X-01 · The reward is standing and balance only · `code`

Alive bonus + uprightness + centre-of-pressure proximity − normalized torque penalty. No
forward-velocity term, no gait shaping, no locomotion. A deliberate first milestone,
recorded here because locomotion is the actual goal of the project.

### X-02 · There is no actuator model — torque is applied instantly · `code`

`ApplyActions` writes `clamp(action, −1, 1) × MaxTorquePerDOF` straight into `JointTorque`,
and Pass 2 multiplies by the authored strength curve. That curve is force-versus-*length*
only: no activation dynamics, no force–velocity relation, no tendon routing — so the policy
can reverse a joint's full torque in a single 1/240 s substep, which no muscle can. MuJoCo's
`muscle` actuator (Hill-type FLV with activation dynamics) is the direct analogue and is the
single closest match to what this project's authoring tool is *trying* to express.

### X-03 · One torque limit for all 68 DOF · `code`

`MaxTorquePerDOF = 5e7`, sized at ~1.5× the worst single-body holding torque (`BElbow1_L`
needs 3.44e7 to hold itself). Every other joint, including the smallest tips, gets the same
ceiling.

**Partially answered 2026-08-21 by `U-10`:** the authored per-muscle scalar strengths now
reach the solver, so delivered torque varies 10× across DOFs even though the limit does
not. This is per-MUSCLE variation, not a per-DOF limit — it scales the commanded torque
rather than bounding it, which is a different thing and brings its own problem (`X-08`).

### X-04 · The observation sees only the last substep's contact state · `code`

`ResolveGroundContactImpulses` calls `OutState->Init(...)` at entry, unconditionally
overwriting every slot — a well-documented fix for a NaN that used to survive resets. The
consequence is that after four substeps `ContactStates` holds only the fourth substep's
forces, so a transient impact in substeps 1–3 is invisible to both the policy and the
reward.

### X-05 · The NaN-weights crash has an upstream root cause and four local workarounds · `code`

`bUseGradNormMaxClipping = true`, `ActionEntropyWeight = 0.01`,
`ActionRegularizationWeight = 0.01`, each added after the previous failed. The diagnosis is
good — the engine's own `ppo.py` clips `log_std` on the upper side only, so a
drifting-negative `log_std` can send the log-probability gradient to infinity in float32.
That root cause is engine source and judged unpatchable from this project; the third
mitigation's effect is unconfirmed, and the comment already names `LearningRatePolicy` as
the next lever.

### X-06 · Trained weights have no topology/layout versioning · `code` — REVISED 2026-08-21

**Partially addressed.** UAsset-backed persistence now exists
(`SaveTrainedNetworksToAssets` / `LoadTrainedNetworksFromAssets` + three
`ULearningAgentsNeuralNetwork` slots), giving versionable, content-browser-visible weight
assets instead of loose files under `Saved/`. See `D-06` for the caveat that it is
manual-only.

**Still open:** nothing records *which topology or observation layout* a set of weights was
trained against. A snapshot silently becomes invalid the next time `NumDOF` or the
contact-point count changes — which has happened at least three times in this project's
history (35 → 43 → 41 bodies, 50 → 74 → 68 DOF), and the observation vector is
`10 + 2·NumDOF + 2·NumContactPoints` wide. Loading a mismatched snapshot is a silent
correctness failure, not a load error.

### X-08 · The configured torque limit is no longer the delivered ceiling · `code` `log` — NEW 2026-08-21

`U-10` connected the authored per-muscle scalar strengths to the solver, and the authored
data turned out not to be neutral: **44 of 67 curve-bearing DOFs carry a non-unit
strength, spanning [0.5, 5.0]** (measured by a new permanent assertion in
`AgentSolver.MutoTopology`).

`MaxTorquePerDOF` clamps the COMMANDED torque; the muscle multiplier is applied after it
and used to be bounded by the curve, which peaks at 1. It now reaches 5. So the strongest
joints deliver up to 5× the configured limit and the weakest half of it.

That is what the tool's strength field was always supposed to mean, and it partially
answers `X-03` ("one torque limit for all 68 DOF") — the limit is uniform but the
delivered torque is no longer. What is open is that **entry 017's sizing was never
re-swept against it**: 5e7 was chosen as ~1.5× the worst single-body holding torque with
the multiplier capped at 1, and on the 44 affected DOFs that margin is now somewhere
between 0.75× and 7.5×.

**Measured 2026-08-21 (entry 029), and downgraded.** The contact re-fit's ceiling ladder
ran the rig under torque babble at three amplitudes, each with the authored strengths and
again with them pinned to 1:

| amplitude | authored peak pen | pinned to 1 | ratio |
|---|---|---|---|
| 30% | 29.45 | 11.18 | 2.6× |
| 60% | 41.01 | 26.66 | 1.5× |
| 100% | 62.29 | 11.34 | **5.5×** |

At the re-fit constants the 5× spread **does not destabilise** — every rung survives. It
does cost real contact quality, by up to 5.5× in peak penetration at full load. So this
moves from a suspected stability risk to a measured quality cost, which is a much weaker
argument for clamping.

Still untested against a TRAINED policy — babble is a stress input, not behaviour.

If the 5× destabilises training, clamp the multiplier in `ComputeMuscleMultipliers` —
one line — rather than lowering `MaxTorquePerDOF`, which would squash the weak joints
along with the strong ones.

### X-07 · One of three editor-crash paths on the training thread is unmitigated · `code`

All three of `ULearningAgentsPPOTrainer`'s training-end paths reach `RevertGameSettings()`
→ `ApplySettings()` → `FlushRenderingCommands()` → `check(IsInGameThread())`, and the loop
runs off the game thread by design because a PPO iteration blocks for minutes. Path 1
(receive timeout) is defused by the 900 s timeout; path 3 (iterations exhausted) by never
reaching 1,000,000. **Path 2 — a hard shared-memory or socket write failure — is
unmitigated**, and there is no interception point since the call happens inside
`RunTraining()`.

---

## N — Record-keeping debt

### N-03 · Twelve TEMP diagnostics are permanent residents · `code`

`EnergyTrace`, `ImpulseContact`, `IsolatedLimb`, `JointLimitContact`, `MassAuthoringDump`,
`RigUpgradeCheck`, `Rung1SingleBody`, `ScalarSIMDParity`, `SkeletonAudit`,
`ThresholdAudit`, `ContactSweep` — more than half the suite.

`ContactSweep` (added by entry 029) is the newest and the most expensive: ~9 minutes,
because its finalist round re-runs every 12 s survivor for a full 50 s in two regimes.
That cost is the point — shorter gates picked winners that died at 12.33 s and 28.36 s —
but it makes the test unsuitable for routine runs and it should be deleted, not kept. (`OffsetWrench` and `WrenchPropagation` were
the twelfth and thirteenth until entry 027 deleted them with their subject.) Entry 022 set the right
precedent by promoting `JointAxisAudit` out of `TEMP` and keeping it, on the grounds that it
guards an invariant no other test can see.

`RigUpgradeCheck` is explicitly marked for deletion once entry 026's numbers are considered
recorded (they now are, in the log). **It is now actively misleading:** its rows track the
shipped defaults by design, so entries 028 and 029 both moved every number in its table,
and it no longer reproduces the entry it was written to defend. Either delete it or
re-baseline it — leaving it as-is invites reading today's output against entry 026's text. `ScalarSIMDParity` and `Rung1SingleBody` deserve
promotion. Most of the rest deserve deletion with their findings left in the log — which is
what entry 021 did for four answered diagnostics. Four of them additionally carry `R-06`'s
bad relaxation setting, and `MutoJointLimitContactTest` alone is 901 lines.

### N-04 · Comments still describe removed code · `code` — MOSTLY RESOLVED

Two of the four fixed. One remains:

- **Still wrong:** four references to the deleted `ApplyGroundContactForces` remain, in
  `ComputeObservations`' parameter comment and in `ComputeDefaultStandingHeight`'s
  reasoning about "the spring engaging from the first step". There is no spring.
- **Fixed:** `BuildMutoContactPoints` no longer claims limb-vs-limb tests "do not exist in
  this file".
- **Fixed (entry 027):** `CreatureGroundContact.h`'s file header no longer describes
  contact as a penalty force staged through `ApplyForceAtPoint`. It had been the most
  misleading comment in the module — the first thing a reader saw in the contact file,
  describing the opposite of the truth. Removing the external-wrench path forced the
  issue, which is the argument for deleting dead code rather than annotating it.

### N-05 · `PhysicsSubstepDt` carries a stale narrative about constants that no longer exist · `code`

The comment block still walks through the 1/960 escalation and the `SpringK = 3000` /
`SpringK = 11000` sweep. Both `ContactSpringK` and `ContactDamperK` were removed in entry
021. The reasoning is worth keeping — in the log, not on a live field whose current value
was chosen for unrelated reasons.

### N-06 · `NumLimbs` still defaults to 6 · `code`

`FCreatureTopology::NumLimbs = 6` (re-verified at `CreatureBatchState.h:32`), a leftover
from the original "six-limbed creature" handoff. `BuildMutoTopology` overwrites it with 8.
Harmless for Muto; silently wrong for any future topology builder that forgets to set it.

---

# Part Two — Measured against MuJoCo

MuJoCo behaviour described from its computation and modelling documentation, not verified
against a running instance. **Revised 2026-08-21:** four of the original eight
recommendations have been adopted; the tables reflect the current state.

## 1. Where the two designs agree

Entry 015 was right that the architecture is not the problem. Five choices now match
MuJoCo closely enough that no change is indicated.

**Reduced coordinates over a kinematic tree.** Both represent state as a floating root pose
plus per-joint scalars, not 41 free bodies with 40 constraints holding them together. The
single most important decision, and it is correct.

**Soft constraints, not hard ones.** `MakeContactSoftness` implements Box2D v3's soft-step
reparameterization, and its `impulseScale · accumulated` term is functionally MuJoCo's
diagonal regularizer *R*. Both codebases document the same conclusion: as stiffness rises
toward hard, the system becomes unstable and noisy.

**Constraint rows for joint limits, solved with contact.** Entry 023's diagnosis and entry
024's fix are exactly MuJoCo's model.

**Warm starting.** MuJoCo warm-starts `qacc`; this warm-starts accumulated impulses per row.
This project got the easier version for free — contact points are fixed local offsets on
named bodies, so `(PointIdx, EndIdx, Env)` is already a stable contact ID.

**An assembled constraint system.** New as of entry 026. Both now form
`A = J M⁻¹ Jᵀ + R` over all active rows and solve that, rather than relaxing rows against
the bodies one at a time.

## 2. The formulation fork, and what remains of it

This solver uses the **Articulated Body Algorithm** — O(n), no matrix assembled. MuJoCo
computes bias forces with recursive Newton–Euler, builds the joint-space mass matrix `M`
explicitly with the Composite Rigid Body algorithm, and factorizes it (sparse *LᵗDL*).

The reason MuJoCo pays for that factorization is the constraint solve: it needs
`J M⁻¹ Jᵀ`, and with `M` factorized that is a batch of back-substitutions.

**This is now a narrower difference than it was.** Entry 026 showed that `A` can be
assembled without ever forming `M`, by exploiting the fact that `SolveImpulseResponse` is
linear in its applied wrench: one unit-impulse query per row yields that row's entire
column of `J M⁻¹ Jᵀ`. Assembly costs one tree pass per row — which the per-row path was
*already spending* to extract the diagonal alone, and then discarding. Measured 1.57×
cheaper overall than the per-row path, because iterations afterwards are pure arithmetic.

What remains genuinely different: MuJoCo's factorization is reusable for other purposes
(inverse dynamics, sensors), where this assembly serves the constraint solve only.

## 3. Solver class

MuJoCo ships **PGS**, **CG**, and **Newton** (its default: exact Newton on the convex dual,
Cholesky factorization of the Hessian, line search, typically converging in a handful of
iterations regardless of coupling).

This project is now at "projected SOR on the assembled system" — past PGS-on-the-tree,
short of Newton. That is a deliberate stopping point: assembling `A` was the structural
prerequisite, and swapping SOR for Newton is a self-contained change to one loop that can
be measured on its own. The residual trace needed to evaluate such a change now exists.

## 4. Subsystem matrix

### Core dynamics

| Subsystem | AgentSolver | MuJoCo | Gap |
|---|---|---|---|
| Forward dynamics | Featherstone ABA, O(n), no matrix formed | RNE bias + CRB mass matrix + sparse LᵗDL | Different by design; narrower since entry 026 |
| Coordinates | Reduced — root pose + per-joint scalars | Reduced — same | None |
| Topology | Strict tree, `BodyParent[i] < i` | Tree + equality constraints for closed loops | No closed loops possible here; architectural |
| **Per-DOF armature** | **`DOFArmatureRatio`, dimensionless ratio on `D`** | `armature`, absolute | **Adopted.** Ratio form is deliberate — see §7 on constants |
| **Per-DOF damping** | **`DOFDampingTimeConstant`, exact velocity-level impulse** | `damping`, a passive force | **Adopted, and deliberately not the same mechanism** — the force form was measured reversing joints here |
| Per-DOF friction loss | None | `frictionloss`, a constraint row | Absent |
| Passive joint spring | None | `stiffness` + `springref` | Absent; would be a natural pose prior for RL |
| Integrator | Semi-implicit Euler only | Euler, RK4, implicit, implicitfast | Less pressing now — the damping impulse is unconditionally stable by construction, which is the main thing `implicitfast` buys |
| Precision | float32 | float64 reference; float32 in MJX | See `R-07` |
| Units | cm–kg–s | Unit-agnostic; all defaults calibrated for SI m | Borrowed constants need rescaling — entries 001, 017 |
| Inverse dynamics | None | `mj_inverse` | Absent |
| Parallel envs | AVX2 8-wide, threaded — force path only | MJX / MuJoCo Warp, whole pipeline | Contact path still serial (`R-04`) |

### Constraints and contact

| Subsystem | AgentSolver | MuJoCo | Gap |
|---|---|---|---|
| Constraint system | **Assembled `A = J M⁻¹ Jᵀ + R` over all rows** | Same | **Closed** |
| Solver | Projected SOR on `A` (per-row PGS retained as a switch) | PGS, CG, or Newton (default) | Newton remains the upgrade |
| Softness | Hertz + damping ratio | `solref` = (timeconst, dampratio) | Equivalent |
| Regularizer | `impulseScale` + `Cfm`, both live | `R` from `solimp` | Closed |
| Depth-dependent impedance | None — `MaxBiasVelocity` caps push-out instead | `solimp` = (d0, d1, width, midpoint, power) | **Open**, and the principled fix for `O-06` |
| Friction cone | Normal, then tangents, then radial rescale to μN | Pyramidal or elliptic, inside the convex program | Decoupled projection here |
| Friction dimensions | 2 tangential only | `condim` 1 / 3 / 4 / 6 | No torsional friction: a resting foot pivots freely |
| Normal:friction impedance | Not modelled | `impratio` | Absent |
| Speculative contacts | Limit rows only, via `MarginDeg` | `margin` / `gap` on all geoms | Ground contacts engage only once penetrating |

### Joints, actuation, collision

| Subsystem | AgentSolver | MuJoCo | Gap |
|---|---|---|---|
| Hinge coordinate | Wrapped mod 360 in clamp and rows | Unbounded real; limits on the raw value | `O-01` does not exist in MuJoCo by construction |
| Ball-joint limit | Single cone on rotation-vector magnitude | Single max-angle limit | *Same limitation* — see `O-02` |
| Actuation | Direct torque × force-vs-length curve | Gear/gain/bias, tendons, `muscle` with Hill FLV + activation | No velocity dependence, no activation lag (`X-02`) |
| Broadphase | Static O(n²) list, ~503 pairs | Sweep-and-prune AABB + contype/conaffinity | `P-05` |
| Narrowphase | Plane–sphere, segment–segment capsules | Analytic primitives, convex meshes, heightfields, SDFs | Ground is one infinite plane; no terrain |
| Self-collision | Limb-vs-limb, different limbs only | Full pairwise with exclusion masks | `O-09`, `O-10`, `O-11`, `O-12` |
| Sensors | Hand-built observation vector | Declarative sensor set, noise-capable | Absent; low priority |

## 5. What is left to steal, re-ranked

Items 1, 2, 3 and 4 of the original list (armature, damping, turning on the regularizers,
assembling the constraint matrix) are done. What remains, in order of expected value:

**1. Depth-dependent impedance (`solimp`).** `MaxBiasVelocity = 100 cm/s` is a hard ceiling
on push-out — it works, and it is what production engines do, but it is a cliff. MuJoCo
varies constraint impedance *continuously* with violation depth: soft at first touch,
progressively stiffer as penetration grows. That is a principled attack on `O-06`'s 15 cm
of impact penetration, where a shallow resting contact stays compliant and quiet while a
deep one firms up smoothly. *Moderate effort — extend `MakeContactSoftness` to take
separation.*

**2. Torsional friction (`condim = 4`).** One extra row resisting rotation about the contact
normal. A foot flat on the ground currently resists sliding but can spin freely about its
own vertical axis — for a creature whose first objective is *standing*, that lets the policy
accumulate yaw drift the uprightness reward never penalizes. Cheap: one row type using
existing machinery, with the angular effective mass from the same impulse-response query.

**3. Newton on the convex dual.** Replaces the SOR sweep now that `A` is assembled. Self-
contained, and the residual trace exists to evaluate it. Worth doing *after* the tuning
sweeps in `O-07`, since a better solver on badly-tuned constants is a smaller win than the
reverse.

**4. Unwrapped joint coordinates.** Deletes `O-01` rather than mitigating it. Not free: the
authored ranges are genuinely wrapped (`Knee1` is [283.2, 403.6]), so this means resolving
each range to a canonical unwrapped interval at topology-build time and defining a single
permitted lap. Best done as its own change with its own test — it touches the function that
has already produced two distinct bugs.

**5. Elliptic friction inside the same solve.** Replaces the post-hoc cone projection. Now
much more tractable than before, since the assembled system is the right place for it.

**6. A Hill-type actuator with activation dynamics.** The direct analogue of `X-02`, and the
closest match to what the MassMuscleProfile tool is already trying to express. Larger, and
it changes what the policy is learning, so it belongs after `O-08` is answered.

## 6. What this solver has that MuJoCo does not

**An exact articulated effective-mass query as first-class API.** `ImpulseResponseAtPoint`
returns the velocity change of a world point from a linear impulse there, validated to
machine precision against the closed form. MuJoCo gets the same information from
`J M⁻¹ Jᵀ`, but not as a directly callable question about an arbitrary point. Entry 026
leaned on this hard: it is what makes assembling `A` possible without forming `M`, and what
makes exact damping possible without an implicit integrator.

**Curve-authored anatomical muscle strength.** MuJoCo's muscle model is parametric. This
project has a bespoke editor where per-joint extension and flexion strength are drawn as
curves against the joint's own authored range, with L/R mirroring — a better authoring story
for a creature nobody has biomechanical data for. Still only half-connected (`X-02`,
`U-10`).

**Native to Unreal, with a deployment path.** Builds a topology from a `USkeletalMesh`'s
reference skeleton, poses a `UPoseableMeshComponent`, runs in PIE against real assets.
`MutoTopology.h` has no MuJoCo counterpart — and it is where a large share of this project's
real bugs have lived (entries 008, 009, 022 are all topology or authoring bugs).

**A written record of negative results**, including retracted claims and three occasions
where an instrument produced a confident wrong answer. Entry 026 added a fourth.

## 7. Where the comparison stops being useful

**The cost profiles are not comparable.** MuJoCo's reference implementation is float64 and
simulates one model per call. This project runs 256 environments 8-wide in float32 inside a
game engine. Adopting the *algorithms* is right; assuming the cost profile transfers is not.

**Every borrowed constant needs a unit rescale.** MuJoCo is nominally unit-agnostic but its
defaults — `solref` timeconst 0.02, gravity −9.81, published armature and damping
magnitudes — are calibrated for SI metres, and this solver runs in centimetres. That exact
mismatch produced entry 001's scale inconsistency and entry 017's two off-by-millions
constants. It is why armature is stored as a dimensionless ratio and damping as a time
constant rather than as MuJoCo's absolute coefficients.

**Adopt the mechanism, not necessarily the formulation.** Entry 026's damping is the worked
example: copying MuJoCo's passive-force implementation literally produced joints that
reversed under damping, because the recoil of a floating parent amplifies a joint torque by
an unbounded factor. The *idea* — dissipate energy across the whole range of motion rather
than only at the stops — transferred; the implementation had to be rebuilt around this
solver's own impulse machinery.

**MJX is the more honest comparison than MuJoCo C.** For batched RL the relevant reference
is MJX or MuJoCo Warp — float32, GPU, fixed iteration counts chosen so the solver stays
compilable, CG or Newton rather than PGS. Much closer to this project's constraints, and it
makes the same trade: it accepts float32 and a fixed iteration budget, but not row-by-row
Gauss-Seidel.
