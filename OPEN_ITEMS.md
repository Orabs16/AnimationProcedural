# AgentSolver — Open Items Register

Companion to `SOLVER_DEBUG_LOG.md`. That file records *hypotheses tested*; this one
records *what is currently left*: code that is built but never called, features wired
but switched off, data authored but never read, and problems measured but not resolved.

**Produced 2026-08-17** by reading the full `AgentSolver` module (45 files, 13,594
lines), the `MassMuscleProfile` data structures and editor model, and all 2,193 lines
of `SOLVER_DEBUG_LOG.md` (entries 001–025).

**Nothing was compiled, no automation test was run, and no simulation was executed.**
Every item carries a provenance tag:

- `code` — read directly in source, with file:line.
- `log` — taken from `SOLVER_DEBUG_LOG.md`'s own measurements.
- `unverified` — depends on `.uasset` data or runtime behaviour that was not checked.

Items are addressable by ID (`U-01`, `O-04`, …) so they can be cited from commits and
from future log entries. A presentation copy of this register with the same IDs is
published at https://claude.ai/code/artifact/e7caac2b-2809-490d-8de5-57e856960c09

---

## STATUS UPDATE — 2026-08-17, later the same day

A first pass of fixes landed after this register was written. Full write-up with
measurements is `SOLVER_DEBUG_LOG.md` **entry 026**; everything below was
verified by build + automation run (24 tests pass, 0 failures).

**RESOLVED**

| ID | What |
|---|---|
| R-01 | Capsule end caps unified in `GetCapsuleLocalEnds`; the duplicate coincident ground row is gone, and a degenerate capsule now reports 1 end instead of 2 |
| R-02 | Centre of pressure uses the same world-surface derivation the solver used to produce the force |
| U-01 | `FIterationDebugLog` wired: records the per-iteration constraint residual, plus driver watch properties |
| U-02 | Welding has a production caller (`BuildSaturatedJointLocks`); lock array is now per-body-per-env |
| U-03 | `Cfm` exposed on all three row types, default 1e-8 |
| U-04 | `Relaxation` exposed on all three row types |
| U-05 | `MaxNormalImpulse` exposed; `MaxBiasVelocity` / `MaxBiasVelocityDeg` now editor-reachable |
| D-01 | `bEnableLimbCollision` now defaults **on** |
| O-04 | Global constraint solve implemented (`A = J M⁻¹ Jᵀ + R`, projected SOR). See the caveat below — it is not what fixed the divergence |
| P-01 | Cost measured: 2.161 ms/substep single-env for the shipped defaults, vs 1.964 baseline and 3.391 for per-row-with-everything |
| P-02 | Superseded for the solve path: rows now contribute whole response columns, not just diagonals |
| N-01 | Log brought current — entry 026 |
| N-02 | Corrected below (22 tests at audit time, not 23; now 24) |

**NEW, found while implementing** — both real, both fixed, both now guarded by
`AgentSolver.SolverUpgrade`:

- **Ball-joint reduced bias was wrong.** `SolveImpulseResponse` hardcoded the
  angular part to zero, which is correct only for a zero joint impulse. Since
  ball-joint cone rows landed (2026-08-16) every hip and spine limit discarded the
  reaction it should have transmitted to its parent. Caught by asserting that an
  internal impulse cannot change total angular momentum; now conserved to 1.4e-08.
- **Damping cannot be a torque.** The first implementation followed MuJoCo
  literally and was measured *reversing* joints (a requested 10% reduction removed
  197%) because the parent's recoil amplifies a joint torque by an unbounded
  factor. Reimplemented as an exact impulse that measures the whole-tree gain.

**IMPORTANT CORRECTION to O-07 / entry 025.** Entry 025's "50 s stable" was
measured on the **35-body** rig, before the spine articulation. On the current
41-body rig that same configuration diverges at 1.03 s — so the rig was already
broken and nothing recorded it. The shipped defaults now survive 50 s with the
torso drifting 0.2 cm over the last 40.

**HONEST ATTRIBUTION.** The global solve is *not* what fixed the divergence:
the per-row path with the same other additions survives too. Survival needed
armature + damping + welding together. What the global solve buys is 1.6× lower
peak penetration, a torso held 47% higher, and 1.57× lower cost while running 4×
the sweeps.

**Still open, unchanged:** O-01, O-02, O-03, O-05, O-06, O-07, O-08, O-09, O-10,
O-11, A-01 through A-07, R-03, R-04, R-05, R-06, R-07, P-03, P-04, P-05, P-06,
U-06 through U-11, D-02 through D-05, all of X.

---

## Read this first

Two items dominate.

**O-04** — entry 024 swept Gauss-Seidel iterations to 128 over a 6-second window and
every configuration still diverged, non-monotonically. That refutes the whole
"add iterations / block-solve / reorder the sweep" family. No code change since has
altered the solver class.

**N-01** — this log stops at entry 025 while the code contains four substantial
undocumented work items dated 2026-08-16/17, including limb-vs-limb collision, which
entry 025 explicitly states does not exist.

Separately, **U-02**, **U-03** and **U-04** mean three mechanisms built specifically to
attack the coupled-row instability — joint welding in the factorization, CFM
regularization, SOR under-relaxation — are all present, complete, and switched off with
no caller anywhere. The instrument built to measure whether they help (**U-01**) has no
callers either.

---

## U — Dead code: implemented, reachable, never called

### U-01 · The PGS convergence instrument has zero callers · `code`

`FIterationDebugLog` captures a watched contact body's and a watched joint-limit DOF's
accumulated impulse per Gauss-Seidel iteration, and is plumbed through as the final
parameter of `ResolveGroundContactImpulses`. Nothing calls it — not the driver, not
either visualizer, not one of the 23 tests. It was built to answer the question entry
024 left standing (is a body simultaneously grounded and against its own stop
converging, or alternately overshooting?). That question is still open.

`Public/CreatureGroundContact.h:593` definition, `:637` parameter. Callers: none.

### U-02 · Joint welding — entry 024's 14.8x result — is not in the live path · `code` `log`

`ComputeArticulatedInertias(Batch, LockedBody)` takes a per-body byte array and, where
set, skips the `U Uᵀ / D` reduction so a saturated joint hands its parent the child's
full articulated inertia. Matching branches exist in `SolveImpulseResponse` (backward
bias and forward recovery), agreeing via `ImpLockedBody`.

Production calls the single-argument overload. The only caller passing a lock set is a
TEMP diagnostic. So the mechanism measured to raise a foot's articulated effective mass
from 18 kg to 269 kg — at the exact substep the heels were being dragged under — is
unreachable from any real run.

`Public/CreatureBatchSolver.h:1489` param, `:1529 :1599 :1651` branches ·
`Public/CreatureGroundContact.h:687` production call, no lock ·
`Tests/MutoJointLimitContactTest.cpp:748` only lock-passing caller.

### U-03 · CFM regularization exists on all three row types, set by nobody · `code`

`Cfm` is added directly to each row's inverse-mass denominator, bounding a single row's
corrective impulse independent of penetration depth — Bullet's diagonal regularizer. On
`FImpulseContactParams`, `FJointLimitParams`, `FLimbCollisionParams`. Its comment names
what it was added for: a body whose ground row and own joint-limit row are both strongly
active, which the Hertz-based regularization cannot bound because neither row's static
effective-mass snapshot accounts for the other.

Default 0 = off. Not a `UPROPERTY`, so it cannot be enabled from the editor. Never
measured.

`Public/CreatureGroundContact.h:166 :282 :329` decls, `:1178 :1281 :1345 :1404` uses.

### U-04 · SOR under-relaxation, same story, same target · `code`

`Relaxation` scales only each iteration's increment, damping how fast two coupled rows
swing between competing corrections. Documented as complementary to `Cfm`: Cfm bounds
how big one row's response gets, SOR bounds how fast the pair oscillates. Default 1.0 =
off. No caller, no `UPROPERTY`, no measurement. With U-03, a matched pair of fixes for
the same diagnosed problem, both fully written and both inert.

`Public/CreatureGroundContact.h:186 :285 :331`, `:1187 :1288 :1351 :1408`.

### U-05 · Per-step normal-impulse ceiling never engaged · `code`

`MaxNormalImpulse`, described in-source as "a safety net, not a tuning knob". Default 0,
no caller. Its penalty-model ancestor (`MaxNormalForce`, entry 007) was worth +45%
survival at the time.

`Public/CreatureGroundContact.h:215 :338`.

### U-06 · Restitution is structurally present but never non-zero · `code`

Both contact row types compute a `RestitutionTerm`. Default 0, no caller — correct for
locomotion, but that branch has never executed in any run or test. Untested code if
ever enabled.

`Public/CreatureGroundContact.h:191 :335`, `:1169 :1400`.

### U-07 · Domain randomization is complete and never invoked · `code`

`FCreatureBatchState::RandomizeEnv` randomizes per-limb strength scale, per-limb loss,
and per-env carried mass. All three are genuinely consumed: `LimbStrengthScale` and
`LimbActive` multiply into tau in Pass 2 of both step variants, and `CarriedMass` is
added to body 0's mass in both variants and in `ComputeArticulatedInertias`.

`RandomizeEnv` is called from exactly one place: a SIMD parity test. The driver never
calls it and exposes no property for strength range, limb-loss chance, or carried mass.
All 256 training envs run with identical neutral values (1.0 strength, all limbs active,
zero carried mass). The randomization the architecture was designed around is not
happening.

`Public/CreatureBatchState.h:307` def, `:341-343` arrays ·
`Public/CreatureBatchSolver.h:172 :214-215 :604 :660-661 :709-710 :1512` consumers ·
`Tests/CreatureBatchSolverSIMDTest.cpp:49` only caller.

### U-08 · External-wrench path read every step, written only by tests · `code`

`ApplyForceAtPoint` and `ExtForce*`/`ExtTorque*` are the documented "rocket impact or
hitscan shot" feature — a world force at a world point, folded in by the bias pass so it
propagates through the chain. Both step variants load these arrays every step, and the
driver calls `ClearExternalForces` for every env at the top of every substep. Nothing
writes them outside tests, so the clear is clearing zeros ×256 envs ×4 substeps ×41
bodies per decision.

`Public/CreatureBatchState.h:281` · `Private/MutoRLTrainingDriver.cpp:696` the clear.

### U-09 · DebugArticulatedInertia read by one TEMP test · `code`

Public solver API kept alive for the entry-020 conditioning audit, whose question the log
records as answered.

`Public/CreatureBatchSolver.h:1826` · `Tests/MutoThresholdAuditTest.cpp:178`.

### U-10 · The muscle profile's scalar strengths are editable, mirrored, saved — and ignored · `code`

`FMassMuscleDataMuscle::ExtensionStrength` / `FlexionStrength` have full editor support:
spinboxes, change handlers, dirty marking, L/R mirror propagation. `BuildMutoTopology`
reads only `ExtensionStrengthCurve` / `FlexionStrengthCurve`. The two scalars never reach
the solver, so anyone authoring in that tool is adjusting a number with no effect on
training. Either fold them into the multiplier alongside the curve, or remove them from
the UI.

`MassMuscleProfile/Public/FMassMuscleData.h:42 :45` · `SMassMuscleDetailsPannel.h:81-84`
UI · `FMassMuscleEditorModel.h:145-146` mirroring · consumers in AgentSolver: none.

### U-11 · FMassMuscleDataMass::BoneIndex unused by the solver · `code`

Every AgentSolver lookup goes through `FindBoneByName`. The stored index is never
consulted, so it is free to be stale relative to the skeleton unnoticed.

`MassMuscleProfile/Public/FMassMuscleData.h:65`.

---

## D — Dormant: wired end-to-end, off by default

### D-01 · Limb collision is complete, tested, and disabled — while the config still pays for it · `code`

`bEnableLimbCollision = false`. Behind the flag: `FLimbPairDef`,
`BuildMutoLimbCollisionPairs`, `ClosestPointsBetweenSegments` (Ericson 5.1.9),
`GetBodyWorldCapsule`, `PairImpulseResponseAtPoint`, `ApplyPairImpulseAtPoints`, a full
row type interleaved into the Gauss-Seidel loop with its own warm-start cache, and three
permanent tests.

The cost was left switched on. `ContactIterations` was raised 8 → 16 *because* limb
collision at 8 starved and blew up a passive drop at t=0.92 s. Every training run now
pays double the solver iterations for a feature that is not running. Either enable it or
return the iteration count to what the enabled features need.

`Public/MutoRLTrainingDriver.h:401` flag, `:324` iteration count raised for it ·
`Public/CreatureGroundContact.h:1639` · `Tests/MutoLimbCollisionTest.cpp` three tests.

### D-02 · Joint speed limiting disabled, and its ordering never validated · `code`

`MaxJointSpeedDegPerSec = 0.0` disables `ClampJointSpeed`. If enabled it runs *after*
`ResolveGroundContactImpulses`, clipping the joint velocities the limit and contact rows
just solved for, with no re-solve. See R-05.

`Public/MutoRLTrainingDriver.h:247` · `Private/MutoRLTrainingDriver.cpp:712-715`.

### D-03 · The relaxation pass is correctly disabled, so one of entry 016's three techniques contributes nothing · `code` `log`

`ContactRelaxIterations = 0` is right for the current ordering, and entry 021 measured
why: Box2D relaxes *after* position integration so the bias velocity has already done its
work, whereas contact here resolves after `Step()` has already integrated position, so
relaxation strips the bias while it has moved nothing (relax=2 → 8.08 units deep,
relax=0 → 0.58). Of accumulated clamping / warm starting / relaxation, the third is
structurally unavailable. The ordering change it needs — resolve contact between the
velocity and position integrations — has not been attempted.

`Public/CreatureGroundContact.h:213` and its comment · log entry 021.

### D-04 · Resume-from-snapshot exists but is off · `code`

`bLoadSnapshotOnStart = false`, so every PIE session starts from fresh weights unless
someone flips it. `bAutoSaveOnEndPlay` and the 300 s autosave are on — runs are saved
but not resumed, which quietly discards progress across sessions.

`Public/MutoRLTrainingDriver.h:488`.

### D-05 · Intentional fallbacks, no action · `code`

`TargetTorsoHeightOverride = -1` (auto-derive), `bUseSocketCommunicator = false`.

---

## A — Placeholder data with no real source

Mass was authored for real (43 kg → 6170 kg, entry 017). Almost nothing else was.

### A-01 · Inertia has no authored source anywhere · `code`

Every `BodyInertiaDiagLocal` is a thin-uniform-rod approximation, `IPerp = (1/12) m L²`
with the long axis hardcoded at `0.05 × IPerp`, L being the distance to the body's own
child. Body 0 uses Pelvis→Back1 as its proxy L. The mass asset has no inertia, density,
or shape field to derive from. This propagates into the ABA `D` term, every articulated
effective mass, and therefore every contact impulse.

`Public/MutoTopology.h:562-564` per-body, `:528-535` body 0.

### A-02 · CoM is half the bone length by assumption · `code`

`OwnCoM = 0.5 × LengthVec`. Reasonable for a rod; not a measurement, not authorable.

`Public/MutoTopology.h:559-560`.

### A-03 · Inertia and contact use different lever arms on the same body · `log`

Inertia is built from `BodyLength` (offset to the body's own child); the contact point
sits at `BodyFusedTipOffset`. On Feet: 106.4 cm vs ~70 cm, ratio 0.44. Recorded in entry
021 as a modelling inconsistency, explicitly not a unit error, never addressed.

### A-04 · Off-diagonal inertia discarded at authoring time · `code`

`FCreatureTopology` stores inertia as an `FVector` diagonal. The Tip-fusion parallel-axis
shift computes a real tensor and keeps only its diagonal, justified in-source on the
grounds that off-diagonal terms stay small while the limb axis is roughly aligned with
the offsets. That assumption is never checked, and degrades exactly where a Tip hangs
off-axis.

`Public/CreatureBatchState.h:65` · `Public/MutoTopology.h:593-604`.

### A-05 · UpperMouth and Chin still have no authored mass — this blocks O-09 · `code`

Both are recorded in-source as having no authored mass or inertia, silently defaulting to
a placeholder ~800x lighter than their real neighbours. They are no longer independent
ABA bodies (they were briefly, and free-spun to ~50 rad/s under gravity alone), so the
acute failure is gone — but the mass they contribute to Head2 and LowerMouth is still
that default.

`BuildMutoLimbCollisionPairs` names this as the explicit precondition for re-enabling
spine-vs-limb pairing: "Revisit broadening spine-vs-limb pairing again only once
UpperMouth/Chin get real mass/inertia authored — the iteration budget alone does not make
that combination safe." Unmet.

`Public/CreatureGroundContact.h:1607-1626` · `Public/MutoTopology.h:90-93 :313-321`.

### A-06 · Collision geometry is hand-authored, coverage unknown · `code` `unverified`

`Radius` (default 15) and `CapsuleHalfHeight` (default 0) stand in for a real
PhysicsAsset, which cannot be read from this module at all — the AVX2 minspec conflicts
with Chaos. Which bones have tuned values lives in the `.uasset` and was not inspected.
`StructuralContactRadius = 10` backfills for ground contact; limb collision has no
fallback (O-11).

`MassMuscleProfile/Public/FMassMuscleData.h:82 :93` · `Public/MutoTopology.h:210-221` ·
`AgentSolver.Build.cs:15`.

### A-07 · Body 0's BodyRestRotInParent slot means something different from every other index · `code`

Index 0 holds Pelvis's own bind-pose rotation in *component* space, because the FK pass
starts at body 1 and never reads it. Three call sites depend on this
(`StandingTorsoRot = Topo.BodyRestRotInParent[0]` in the driver and both visualizers).
Documented in three places, which is the right mitigation — but a latent trap for any
generic code that iterates the array from 0.

`Public/CreatureBatchState.h:50-62` · `Public/MutoTopology.h:187-202 :520` ·
`Private/MutoRLTrainingDriver.cpp:371`.

---

## O — Open physics problems

### O-01 · Joint wind-up is still only bounded modulo 360° · `code` `log`

Entry 007 identified the shape: `ClampAngleDeg` does `Fmod(AngleDeg − MinDeg, 360)`, so a
joint that spins a full lap past its limit re-enters the valid window on the other side
and is accepted. The clamp bounds the residue, never the accumulated rotation. Entry 023
measured joints wound past −8000°.

The 2026-08-17 fix is real — it removed a spurious −360° correction that fired every
substep once a joint settled at its lower stop — but it fixes the arithmetic of the
correction, not the inability to bound the lap. The velocity-level limit rows wrap the
same way. Entry 024 measured 571 lap skips *after* the rows landed, down from 1522, and
said it explicitly: "Do not describe this as tunnel-proof."

`Public/CreatureBatchSolver.h:1199-1231` clamp ·
`Public/CreatureGroundContact.h:1012-1035` rows, same wrap · log entries 007, 023, 024.

### O-02 · Ball-joint limits are one uniform cone sized by the widest authored axis · `code`

`ClampJointLimits` clamps the rotation vector's *magnitude* only, preserving direction —
provably continuous, which is why it superseded a per-axis attempt that produced 30-56°
jumps in a single 1/240 s substep. The cost is stated in-source: one cone half-angle per
joint, taken as the **largest** of the three authored per-axis widths, deliberately loose
rather than tight. A hip authored with a 20° roll range and a 90° pitch range is
permitted 90° in every direction, roll included. The 2026-08-16 cone rows inherit the same
approximation. A real swing-twist decomposition was never attempted.

Note: this is *also* MuJoCo's ball-joint model — see Part Two §7.

`Public/CreatureBatchSolver.h:1269-1337` · `Public/CreatureGroundContact.h:935-987`.

### O-03 · Two joint-limit mechanisms with opposite semantics run every substep · `code` `log`

The constraint rows transmit a limit's load up the chain. `ClampJointLimits` zeroes
`JointVel`, destroying that momentum. Both run every substep on the same DOFs. Entry 024
left the clamp in deliberately as a position-level backstop and flagged the conflict:
"If it turns out to fight these rows (it still zeroes `JointVel`), that is a separate
change with its own measurement." No such measurement was made. The instrument that
would answer it is U-01.

`Public/CreatureBatchSolver.h:924` and `:449` clamp calls ·
`Public/CreatureGroundContact.h:1238-1300` rows.

### O-04 · Gauss-Seidel does not converge on this system, and more iterations is refuted · `log`

**The central open result.** Entry 024's 1440-substep (6 s) sweep, after first nearly
reporting a 400-substep window as a fix:

```
iters |  pen@150 | pen@250 |   maxPen | diverged
    8 |     7.25 |   17.77 |    71871 | 383
   16 |     7.32 |   13.50 |     7324 | 383
   32 |     7.54 |   27.61 |  5474414 | 535
   64 |     7.52 |   39.28 |    70351 | 487
  128 |     7.52 |   39.42 |     1288 | 495
```

Everything diverges; not monotonic. Entry 017's earlier 4/8/16/32/64 sweep was equally
flat and non-monotonic. Conclusion drawn, and load-bearing: **convergence rate is not the
missing ingredient**, which rules out "add iterations / block-solve the chain / reorder
the sweep" entirely.

Entry 024 ranked the alternatives, ending with "full LCP over all constraints (MuJoCo's
actual approach): correct, but a rewrite, and there is no evidence yet that the cheaper
fix is insufficient." Nothing since has changed the solver class — cone rows, limb
collision, Cfm and SOR are all row-level interventions inside the same PGS loop the sweep
indicted. See Part Two §3.

### O-05 · Worse as the timestep shrinks; the leading explanation was never checked · `log`

At matched work: 120 Hz × 16 → 1.417 s, 240 × 8 → 2.946 s, 480 × 4 → 1.035 s,
960 × 2 → 1.052 s, 1920 × 1 → 0.938 s. An interior optimum with monotonic degradation
above it; a convergent method cannot degrade under refinement.

Entry 019 proposed another absolute-epsilon defect; entry 020 measured every epsilon and
cleanly refuted that, then proposed a better explanation and flagged it as "the next
thing to test": soft-constraint `impulseScale` reaches 0.50 at 1920 Hz versus 0.11 at
240 Hz, so each iteration discards half the accumulated impulse at fine steps and
iteration count never rises to compensate. Never run.

log entries 019, 020 · `Public/CreatureGroundContact.h:428-443` `MakeContactSoftness`.

### O-06 · 20.7 cm of peak penetration at impact on the stable passive rig · `log`

Entry 025's 50-second run is genuinely stable — torso drifts 0.01 cm over the last 40 s,
peak penetration and speed both reached at initial impact and never approached again.
The entry is equally clear it is "stable, not correct": the rig visibly sinks before
contact catches it.

### O-07 · The shipped contact constants come from a sweep the log retracted twice · `log` `code`

`ContactHertz = 15` and `ContactDampingRatio = 10` come from entry 018's softness sweep.
Entry 022 then established that every contact number in entries 013-021 was measured on a
rig whose 26 revolutes rotated about wrong axes; entry 025 added that the same runs had no
torso collision, so the creature could not be held up by construction. Both entries say
the sweeps should be re-run against the now-stable baseline. They have not been, and the
comment on `ContactHertz` still asserts "measured best around 15 on the full rig".

`Public/MutoRLTrainingDriver.h:309 :313` · log entries 018, 022, 025.

### O-08 · "Can it stand up" has never been answered on the current rig · `log`

Every stability result for the current code is a *passive* drop. Entry 025: "It settles
as a collapsed heap, not standing. There is no actuation in a passive drop." The last
actuated results are entries 017/018 — a PD stand diverging at 1.03-1.59 s — and those
predate the joint-axis fix, torso collision, cone rows, spine articulation, and the
iteration change. The project's actual objective is unmeasured.

### O-09 · Limbs pass through the torso and spine · `code`

`BuildMutoLimbCollisionPairs` excludes every body with `BodyLimbIndex == INDEX_NONE` —
the whole spine, head and jaw chain — from candidacy. Pairing was broadened on 2026-08-16
and reverted the same day: candidates 503 → 701 on an unchanged budget, and A-05's
unauthored masses reproducibly blew up a zero-torque drop within 3 substeps. Revert
reasoning is sound; both preconditions (real masses, iteration budget) are still open.

`Public/CreatureGroundContact.h:1592-1663` · `Tests/MutoLimbCollisionTest.cpp:63`.

### O-10 · A limb can fold through itself · `code`

Pairs are generated only where `BodyLimbIndex[A] != BodyLimbIndex[B]`, so no two bodies
within one limb ever test against each other. Sensible for adjacent links; it also means a
four-segment leg can fold its foot through its own thigh.

`Public/CreatureGroundContact.h:1656`.

### O-11 · Mid-limb segments have no limb-collision geometry · `code`

Candidacy requires `BodyRadius > 0` with no fallback — stated in-source as a deliberate
choice of "a smaller, cheaper candidate list over full coverage". Shoulders and the upper
arm and leg links are absent by construction, unlike ground contact which backfills via
`StructuralRadiusFallback`.

`Public/CreatureGroundContact.h:1629-1632 :1645`.

---

## R — Risks found reading the code, not in the log

None of these appears in `SOLVER_DEBUG_LOG.md`.

### R-01 · Interior bodies can get two coincident contact rows at the same point · `code` `unverified`

The ground gather derives a capsule's second end as
`LocalOffset − LocalOffset.GetSafeNormal() * (CapsuleHalfHeight * 2)`. For an authored
contact point on an *interior* body, `LocalOffset == BodyFusedTipOffset == ZeroVector` —
entry 021 measured exactly this, recording `FElbow3_*` and `BElbow3_*` at
`leverArm = 0.00`. `GetSafeNormal()` on a zero vector returns zero, so
`LocalEnds[1] == LocalEnds[0]` while `NumEnds` is still set to 2.

Result: two independent rows at identical world positions, each with its own accumulated
impulse and warm-start slot — precisely the "two rows fighting over the same load" case
that `BuildMutoContactPoints` avoids when it skips doubling up authored points, and that
the same comment calls out as what accumulated-impulse clamping handles badly.

`GetBodyWorldCapsule` was fixed for this exact bug in the limb-collision path (it falls
back to local `+X` because zero-normalizing collapsed both capsule ends onto the joint
origin). The ground path never got that fix.

**Fires only if FElbow3 / BElbow3 carry a non-zero authored `CapsuleHalfHeight`** — asset
data, not inspected. The sibling comment noting `FElbow_L`'s half-height at 241.4 makes it
likely they do.

`Public/CreatureGroundContact.h:709-714` derivation, `:550-578` where the same bug was
fixed, `:1550-1554` the doubling-up warning · log entry 021 for leverArm = 0.00.

### R-02 · The balance reward computes centre of pressure at joint origins · `code`

`ComputeReward` reconstructs each touching point as
`BodyPos + BodyRot.RotateVector(Point.LocalOffset)`. The gather loop that produced the
force it weights by used `... − Point.Radius * GroundNormal` and selected between two
capsule end-caps. Neither correction is applied here.

With `bAllBodiesCollideWithGround` on, structural points have `LocalOffset == ZeroVector`
and outnumber authored points 25 to 10 — so most of the CoP sum is placed at joint
origins, offset from the real contact patch by the bone's radius and, for authored points,
by up to the full fused-tip length. The balance term is the second-largest positive term
in the reward.

`Public/CreatureRLEnvironment.h:190-195` · compare `Public/CreatureGroundContact.h:727-731`.

### R-03 · Observations span ~8 orders of magnitude with no normalization here · `code` `unverified`

The observation vector interleaves `TorsoUp` components in [−1, 1], a height error in cm,
joint angles in radians that O-01 permits to reach ±140, and raw `NormalForce` values
entry 007 measured in the 1e6-1e7 range. No scaling is applied in `ComputeObservations`.

Whether Learning Agents' encoder normalizes internally was not verified. If it does not,
the force channels dominate the input scale entirely and every other channel — including
the upright signal the reward is built on — is numerically invisible to the policy.

`Public/CreatureRLEnvironment.h:105-146`.

### R-04 · Contact resolution is serial across envs, and cannot be parallelized as written · `code`

The force path is `ParallelFor`-ed over 8-wide env chunks in all four passes.
`ResolveGroundContactImpulses` contains no `ParallelFor` at all — every gather and every
iteration is a plain `for (Env ...)`.

It also cannot simply be wrapped: `SolveImpulseResponse` writes to solver members sized
for one env (`ImpP`, `ImpU`, `ImpUVec`) and every entry point above it shares
`DVScratch`, `DQScratch`, `JointImpScratch`. Two envs solving concurrently would corrupt
each other. A concrete, nameable blocker on the throughput of the whole training loop,
recorded nowhere.

`Public/CreatureBatchSolver.h:1038-1047` shared scratch ·
`Public/CreatureGroundContact.h:700 :796 :926` serial loops.

### R-05 · If joint speed limiting is enabled it silently overrides the constraint solve · `code`

`ClampJointSpeed` runs after `ResolveGroundContactImpulses`, so it would clip the joint
velocities the limit and contact rows had just converged on, with no re-solve and no
report. Harmless only because D-02 leaves it off.

`Private/MutoRLTrainingDriver.cpp:707-715`.

### R-06 · Half the TEMP diagnostics run the configuration entry 021 called wrong · `code`

Entry 021 established `RelaxIterations > 0` is wrong for this step ordering and set the
default to 0. Six sites still set it explicitly. Any number those diagnostics report
about resting penetration or settling is measured under a known-bad relaxation setting.

`Tests/MutoImpulseContactTest.cpp:223 :256` · `MutoIsolatedLimbTest.cpp:326` ·
`MutoThresholdAuditTest.cpp:102` · `MutoWrenchPropagationTest.cpp:249`.

### R-07 · Float32 throughout, on quantities spanning 1 to 1e10 · `code` `log`

Everything is `float`: state arrays, `FMat3`, `FSpatialInertia`, the 6×6 root solve.
Entry 020 measured `min |Det(Irot)| = 4.63e10` across all bodies, and body masses range
from ~1 kg at the tips to 3282 kg at the torso.

Single precision carries ~7 significant digits. Pass 2 repeatedly forms `I − U Uᵀ/D` and
translates composite inertias between reference points — accumulate-and-subtract on
quantities of very different magnitude, which is where float32 cancellation lives. Entry
020 audited *thresholds* and correctly refuted the epsilon hypothesis; it did not audit
*precision*. MuJoCo's reference implementation is float64 for exactly this class of
reason.

`Public/SpatialAlgebra.h:8` `float M[3][3]`, `:282-319` `SolveSpatial6` · log entry 020.

---

## P — Cost and structure

Entry 025 closed with "cost went up ... not measured; it will matter for training
throughput with many envs." Still not measured.

### P-01 · Contact cost per decision, from the shipped defaults · `code`

Per substep, per env: 35 contact points × up to 2 ends × 3 `ImpulseResponseAtPoint` calls
at gather, each a full backward + root-solve + forward pass over 41 bodies. Then one
`JointImpulseResponse` per active revolute limit row and one `BallJointImpulseResponse`
per active cone row, each also a full tree pass. Then 16 iterations, each applying one
full tree pass per active row.

×4 substeps per decision at `FixedDt = 1/60` and `PhysicsSubstepDt = 1/240`, ×256 envs —
all serial (R-04) while the force path runs 8-wide and threaded. Severe asymmetry, no
number on it.

### P-02 · Every row's diagonal is recovered by its own whole-tree solve · `code`

`JointImpulseResponse` sets one entry of a joint-impulse vector to 1, runs the full
recursion, and reads back one scalar. `BallJointImpulseResponse` does the same for a
direction. Each is O(bodies) for one number, and the numbers are the diagonal of a matrix
the solver never forms. See Part Two §4 for what forming it would buy.

`Public/CreatureBatchSolver.h:1717 :1775`.

### P-03 · The limb-pair path heap-allocates four arrays per call, inside the iteration loop · `code`

`PairImpulseResponseAtPoint` and `ApplyPairImpulseAtPoints` each declare four local
`TArray`s and size them per call, because the shared scratch is single-query and both
results are needed simultaneously to superpose them. Reasoning documented and correct;
consequence is four allocations per active pair per iteration, 16 iterations deep, in the
substep loop. A second scratch set on the solver removes it.

`Public/CreatureBatchSolver.h:1891-1896 :1932-1937`.

### P-04 · A third of the tree runs lane-scalar inside StepSIMD · `code`

Ball joints' Pass 2 reduction and Pass 3b integration both fall back to a per-env scalar
loop; only their Pass 1 kinematics is vectorized. Muto now has 14 ball joints of 41 bodies
(6 spine + 8 limb mounts) and 18 of 68 DOF, each ball body costing a 3×3 inverse per pass.
Flagged as a follow-up in the class comment; the 2026-08-16 spine articulation made it
substantially more expensive without revisiting it.

`Public/CreatureBatchSolver.h:22-26` caveat, `:651-691 :824-895` scalar branches.

### P-05 · Limb collision is an O(n²) static list, fully re-tested every substep · `code`

~503 candidate pairs, each getting a full segment-segment closest-point computation every
substep with no AABB pre-pass and no spatial partitioning.

`Public/CreatureGroundContact.h:796-870`.

### P-06 · The force path has a correctness oracle; the impulse path has none · `code`

`StepScalar` is a full duplicate of the ABA math kept permanently as the SIMD reference,
and it has caught real bugs. The impulse machinery has no equivalent second
implementation. Its only correctness anchor is entry 014's closed-form GATE on two
effective-mass cases — exact but narrow: it validates `ImpulseResponseAtPoint` on a
single body, not the whole-tree recursion, the welded branch (U-02), or the joint-space
entry points.

`Public/CreatureBatchSolver.h:40-47 :63` · log entry 014 GATE.

---

## X — Reinforcement-learning gaps

### X-01 · The reward is standing and balance only · `code`

Alive bonus + uprightness + CoP proximity − normalized torque penalty. No forward-velocity
term, no gait shaping, no locomotion. A deliberate first milestone (smaller reward-design
surface), recorded here because locomotion is the actual goal.

`Public/CreatureRLEnvironment.h:8-13 :170-251`.

### X-02 · There is no actuator model — torque is applied instantly · `code`

`ApplyActions` writes `clamp(action, −1, 1) * MaxTorquePerDOF` straight into
`JointTorque`, and Pass 2 multiplies by the authored strength curve. That curve is
force-versus-*length* only. No activation dynamics, no force-velocity relation, no tendon
routing — so the policy can reverse a joint's full torque in a single 1/240 s substep,
which no muscle can. Compare MuJoCo's `muscle` actuator, Part Two §5 item 1's neighbours.

`Public/CreatureRLEnvironment.h:149-158` · `Public/CreatureBatchSolver.h:1074-1119`.

### X-03 · One torque limit for all 68 DOF · `code`

`MaxTorquePerDOF = 5e7`, sized at ~1.5x the single worst holding torque in the rig
(`BElbow1_L` at 3.44e7). Every other joint including the smallest tips gets that same
ceiling. Per-DOF limits are noted in-source as "a reasonable follow-up once real per-joint
strength data exists". Entry 018 separately measured a PD controller at `Kp = 5e8`
diverging on its own at 240 Hz, so the headroom is not free.

`Public/CreatureRLEnvironment.h:58-82` · log entries 017, 018.

### X-04 · The observation sees only the last substep's contact state · `code`

`ResolveGroundContactImpulses` calls `OutState->Init(...)` at entry, unconditionally
overwriting every slot — a genuine, well-documented fix for a NaN that used to survive
resets. The consequence is that after four substeps `ContactStates` holds only the fourth
substep's forces. A transient impact in substeps 1-3 is invisible to the policy and the
reward.

`Public/CreatureGroundContact.h:641-663` · `Private/MutoRLTrainingDriver.cpp:692-716`.

### X-05 · The NaN-weights crash has an upstream root cause and four local workarounds · `code`

The driver constructor carries three settings added in sequence after each previous one
failed: `bUseGradNormMaxClipping = true`, `ActionEntropyWeight = 0.01`,
`ActionRegularizationWeight = 0.01` (10x the plugin default). The diagnosis is unusually
good — the engine's own `ppo.py` clips `log_std` on the upper side only, so a
drifting-negative `log_std` can send the log-probability gradient to infinity in float32
well before `std` reaches zero.

That root cause is engine source and explicitly judged unpatchable from this project. The
third mitigation's effect is unconfirmed, and the comment already names the next lever
(`LearningRatePolicy`). An open loop.

`Private/MutoRLTrainingDriver.cpp:151-215`.

### X-06 · Trained weights live only in three raw snapshot files · `code`

`MakePolicy` is always given nullptr network assets, so weights exist only in a transient
in-memory Policy plus whatever `SaveTrainedNetworks` wrote to `Saved/MutoRL/Snapshots`.
No UAsset-backed checkpoint, no versioning, and no record of which topology or observation
layout a snapshot was trained against — so a snapshot silently becomes invalid the next
time `NumDOF` or the contact-point count changes, which has happened at least three times
(35 → 43 → 41 bodies).

`Private/MutoRLTrainingDriver.cpp:436-448 :794-836`.

### X-07 · One of three editor-crash paths on the training thread is unmitigated · `code`

All three of `ULearningAgentsPPOTrainer`'s training-end paths reach `RevertGameSettings()`
→ `ApplySettings()` → `FlushRenderingCommands()` → `check(IsInGameThread())`, and the
loop runs off the game thread by design because a PPO iteration blocks for minutes. Path 1
(receive timeout) is defused by the 900 s timeout; path 3 (iterations exhausted) by never
reaching 1,000,000. **Path 2 — a hard shared-memory or socket write failure — is
documented as unmitigated.** No interception point exists; the call happens inside
`RunTraining()`.

`Private/MutoRLTrainingDriver.cpp:719-765`.

---

## N — Record-keeping debt

### N-01 · This log stops at entry 025 and is now wrong on four counts · `code` `log`

Four substantial pieces of work exist only as code comments, all after the last entry:

- **Spine / head / jaw articulation** (2026-08-16) — the chain became independently
  articulated as six 3-DOF ball bodies, taking the rig from 35 bodies / 50 DOF to
  41 / 68, and moving F/B/M limb mounts off the pelvis onto their real anatomical spine
  parents. This changes every body count in the log.
- **Ball-joint cone limit rows** (2026-08-16) — velocity-level cone constraints, closing
  the "frictionless pendulum between hits" gap. This is entry 024's own recommended next
  step, implemented and never written up.
- **Limb-vs-limb collision** (2026-08-16) — entry 025 states flatly that "self-collision
  still does not exist" and "limbs still pass through each other". It exists.
- **Cfm, SOR relaxation, and the iteration debug log** (2026-08-17) — U-01 through U-04.

Anyone treating the log as current state — including a future session that reads it
first, which is what it is for — will be wrong about the rig's size, its constraint set,
and its collision capabilities.

Contradicted by `Public/MutoTopology.h:117-149`,
`Public/CreatureGroundContact.h:888-913 :1592-1663 :146-186`.

### N-02 · Test counts no longer match the suite · `code` — CORRECTED, then RESOLVED

Entry 025 closes "Suite: 19 tests pass, 0 failures".

*Correction to this register's own first pass:* there were **22** declared at audit
time, not 23 — 11 permanent and 11 under `AgentSolver.TEMP.*`. The original count
of 12 TEMP mistakenly included `AgentSolver.Muto.JointAxisAudit`, which entry 022
deliberately promoted out of `TEMP`.

*Now:* 24 declared, and **all 24 verified passing** by build + automation run on
2026-08-17 (`AgentSolver.SolverUpgrade` and `AgentSolver.TEMP.RigUpgradeCheck`
added). So this item is closed, and the "nothing was run for this document"
caveat no longer applies to the suite.

### N-03 · Eleven TEMP diagnostics are permanent residents · `code`

`EnergyTrace`, `ImpulseContact`, `IsolatedLimb`, `JointLimitContact`,
`MassAuthoringDump`, `OffsetWrench`, `Rung1SingleBody`, `ScalarSIMDParity`,
`SkeletonAudit`, `ThresholdAudit`, `WrenchPropagation` — plus
`RigUpgradeCheck`, added 2026-08-17 and explicitly marked for deletion once entry
026's numbers are considered recorded. Entry 022 set the
right precedent by promoting `JointAxisAudit` out of TEMP and keeping it, on the grounds
that it guards an invariant no other test can see. The rest run on every suite
invocation, several answering questions the log records as closed, several under R-06's
bad relaxation setting; `MutoJointLimitContactTest` alone is 901 lines.

Some deserve promotion (`ScalarSIMDParity`, `Rung1SingleBody`); most deserve deletion
with their findings left in the log, which is what entry 021 did for four penalty-only
diagnostics.

### N-04 · Four in-source comments describe removed code · `code`

- `CreatureGroundContact.h`'s file header still opens "contact is a penalty force
  (spring-damper + clamped Coulomb friction) applied through `ApplyForceAtPoint` ...
  rather than building a full LCP/constraint contact solver." The penalty model was
  deleted in entry 021 and the file now contains a constraint solver.
- `ComputeObservations`' parameter comment references "`ApplyGroundContactForces`'s
  OutState layout". That function no longer exists.
- `ComputeDefaultStandingHeight` reasons at length about "`ApplyGroundContactForces`'
  Penetration>0.0f check" and picking a margin so "the spring engages from the very first
  step". There is no spring.
- `BuildMutoContactPoints` asserts "Limb-vs-limb needs a broadphase and pairwise tests
  that do not exist in this file." They are in that file, 90 lines below.

`Public/CreatureGroundContact.h:3-10 :1528-1532` · `Public/CreatureRLEnvironment.h:110` ·
`Private/MutoRLTrainingDriver.cpp:281-293`.

### N-05 · PhysicsSubstepDt carries 40 lines of history about constants that no longer exist · `code`

The comment walks through the 1/240 → 1/960 escalation, the `SpringK = 3000` /
`SpringK = 11000` sweep, and the warning not to expect finer substeps to buy stiffness —
all about `ContactSpringK` / `ContactDamperK`, removed in entry 021. The reasoning is
worth keeping; it belongs in the log, not on a live field whose current value was chosen
for unrelated reasons.

`Public/MutoRLTrainingDriver.h:180-223`.

### N-06 · NumLimbs still defaults to 6 · `code`

`FCreatureTopology::NumLimbs = 6`, a leftover from the original "six-limbed creature"
handoff. `BuildMutoTopology` overwrites it with 8. Harmless for Muto; misleading as
documentation, and silently wrong for any future topology builder that forgets to set it.

`Public/CreatureBatchState.h:32` · `Public/MutoTopology.h:503`.

---

# Part Two — Measured against MuJoCo

MuJoCo behaviour below is from its computation and modelling documentation, not verified
against a running instance.

## 1. Where the two designs already agree

Entry 015 was right that the architecture is not the problem. Four choices match MuJoCo
closely enough that no change is indicated.

**Reduced coordinates over a kinematic tree.** Both represent state as a floating root
pose plus per-joint scalars, not 41 free bodies with 40 constraints holding them
together. PhysX articulations do the same. The single most important decision, and it is
correct: it removes joint drift by construction and is why a 68-DOF creature is tractable
at all.

**Soft constraints, not hard ones.** `MakeContactSoftness` implements Box2D v3's soft-step
reparameterization — natural frequency and damping ratio rather than raw stiffness — and
its trailing `impulseScale * accumulated` term is functionally MuJoCo's diagonal
regularizer *R*. Both codebases document the same conclusion: as stiffness rises toward
hard, the system becomes unstable and noisy. MuJoCo, the standard tool for RL locomotion,
deliberately does not use hard contact either.

**Constraint rows for joint limits, solved with contact.** Entry 023's diagnosis (a leg
folded onto its stops is still modelled as freely folding, so contact sizes its impulse
against a compliant chain) and entry 024's fix are exactly MuJoCo's model, where limits,
contacts, friction loss and equality constraints are all rows of one system.

**Warm starting.** Both persist solver state across steps — MuJoCo warm-starts `qacc`,
this code warm-starts accumulated impulses per row. This project got the easier version
for free: contact points are fixed local offsets on named bodies, so
`(PointIdx, EndIdx, Env)` is already a stable contact ID with none of the feature-matching
a general collision engine needs.

## 2. The formulation fork: O(n) forward dynamics vs an explicit mass matrix

This is the deepest difference. It is not obviously a mistake, but it reaches every open
item in section O.

This solver uses the **Articulated Body Algorithm**: a backward pass reducing each joint
out of its parent's inertia, a 6×6 root solve, a forward pass recovering accelerations.
O(n) in bodies, no matrix assembled, minimal memory. The fastest way to answer "given
these torques, what are the accelerations?"

MuJoCo does something different on purpose: recursive Newton-Euler for bias forces, then
**builds the joint-space mass matrix M explicitly** with the Composite Rigid Body
algorithm and factorizes it (sparse *LᵗDL*). Forward dynamics is then a
back-substitution. More work than ABA for forward dynamics alone.

The reason is the constraint solve. MuJoCo needs `A = J M⁻¹ Jᵀ + R` — the full inverse
constraint inertia, every row against every other row — and with M factorized that is a
batch of back-substitutions against the rows of J. Having paid for the factorization
once, coupling between all constraints is cheap.

ABA never forms M, so this codebase cannot form that matrix, and doesn't. It recovers
**one diagonal entry at a time**: `ImpulseResponseAtPoint` for a contact direction,
`JointImpulseResponse` for a revolute limit, `BallJointImpulseResponse` for a cone — each
a full tree traversal returning one scalar (P-02). The off-diagonal coupling between rows
is never computed. It is only ever discovered *implicitly*, by applying one row's impulse
and letting the next observe the changed velocity.

That is the definition of Gauss-Seidel, and it is why O-04 happened. Sequential row-by-row
relaxation is the only solver this data layout can support. Everything added since entry
024 — cone rows, limb pairs, Cfm, SOR — is an attempt to make Gauss-Seidel behave on a
system whose sweep result says it will not.

## 3. Solver class: why O-04 is a symptom, not a bug

MuJoCo ships three constraint solvers and defaults to the third.

- **PGS** — projected Gauss-Seidel, row by row. This is what
  `ResolveGroundContactImpulses` is. MuJoCo keeps it mainly for comparison.
- **CG** — conjugate gradient on the convex dual objective, with line search. Global
  rather than row-local; sees the whole coupled system each step.
- **Newton** — the default. Exact Newton on the convex dual, Cholesky factorization of
  the Hessian, line search. Typically reaches high accuracy in a handful of iterations
  regardless of coupling strength, because it is not relaxing rows against each other —
  it is solving the system.

Set entry 024's sweep against that. Peak penetrations of 71,871 · 7,324 · 5,474,414 ·
70,351 · 1,288 at 8/16/32/64/128 iterations, divergence at substeps 383 · 383 · 535 ·
487 · 495. Non-monotonic across a 16x range, everything diverging. That is the canonical
signature of Gauss-Seidel on a strongly coupled system: each sweep order produces a
different fixed-point trajectory and none is the solution. Adding sweeps changes which
wrong answer you get.

Entry 024 reached the right conclusion and named the right remedy: "full LCP over all
constraints (MuJoCo's actual approach): correct, but a rewrite, and there is no evidence
yet that the cheaper fix is insufficient." The cheaper fixes have since been tried — cone
rows landed, limb pairs landed, Cfm and SOR were written. Two of the four are still off
and unmeasured (U-03, U-04), so the evidence question is technically open. But the sweep
is the strongest single piece of evidence in the entire log and it points at the solver
class.

The ingredients already exist. `SolveImpulseResponse` is linear in its applied wrench —
`PairImpulseResponseAtPoint` already relies on that, superposing two independent queries.
Running it once per row against a unit impulse gives that row's *entire column* of
`J M⁻¹ Jᵀ`, not just its diagonal. The matrix is rows × rows, and active rows on this rig
number in the tens. Assembly costs one tree pass per row — which the code **already
spends** at gather time to get the diagonal alone.

## 4. Subsystem matrix

### Core dynamics

| Subsystem | AgentSolver | MuJoCo | Gap |
|---|---|---|---|
| Forward dynamics | Featherstone ABA, O(n), no matrix formed | RNE bias + CRB mass matrix + sparse LᵗDL | Different by design; drives everything below |
| Coordinates | Reduced — root pose + per-joint scalars | Reduced — same | None |
| Topology | Strict tree, `BodyParent[i] < i` | Tree + equality constraints (connect/weld) for loops | No closed loops possible here; architectural |
| Integrator | Semi-implicit Euler only | Euler, RK4, implicit, implicitfast | No implicit option — matters once damping exists |
| Precision | float32 throughout | float64 reference (`mjtNum`); float32 in MJX | See R-07 |
| Units | cm–kg–s (UE world units) | Unit-agnostic; all defaults calibrated for SI m–kg–s | Borrowed constants need rescaling — source of entries 001, 017 |
| Inverse dynamics | None | `mj_inverse` | Absent; useful for reference-motion tracking |
| Parallel envs | AVX2 8-wide over envs, threaded — force path only | MJX (JAX/GPU) and MuJoCo Warp, whole pipeline | Contact path serial here (R-04) — the real gap |

### Constraints and contact

| Subsystem | AgentSolver | MuJoCo | Gap |
|---|---|---|---|
| Constraint system | Independent rows, diagonal recovered per row | One stacked Jacobian J; `A = J M⁻¹ Jᵀ + R` assembled | No off-diagonal coupling computed here |
| Solver | PGS with warm start and accumulated clamping | PGS, CG, or Newton (default) on the convex dual | The central gap — see O-04 |
| Softness | Hertz + damping ratio → bias rate / mass scale / impulse scale | `solref` = (timeconst, dampratio) | Equivalent |
| Regularizer | `impulseScale`, plus an unused `Cfm` (U-03) | `R` from `solimp`, always active | Cfm is dead code; MuJoCo's is load-bearing |
| Depth-dependent impedance | None — `MaxBiasVelocity` caps push-out instead | `solimp` = (d0, d1, width, midpoint, power) | Missing; relevant to O-06 |
| Friction cone | Normal, then tangents, then radial rescale to μ·N | Pyramidal or elliptic, inside the convex program | Decoupled projection here; systematically lags |
| Friction dimensions | 2 tangential only | `condim` 1 / 3 / 4 / 6 — adds torsional, then rolling | No torsional friction: a resting foot pivots freely |
| Normal:friction impedance | Not modelled | `impratio` | Missing |
| Speculative contacts | Limit rows only, via `MarginDeg` | `margin` / `gap` on all geoms | Ground contacts engage only once penetrating |

### Joints, actuation, collision

| Subsystem | AgentSolver | MuJoCo | Gap |
|---|---|---|---|
| Per-DOF armature | **None** | `armature` — added to M's diagonal | The single highest-value omission |
| Per-DOF damping | **None** | `damping` | Absent; drove the 2026-08-16 spine resonance |
| Per-DOF friction loss | None | `frictionloss`, solved as a constraint row | Absent |
| Passive joint spring | None | `stiffness` + `springref` | Absent; a natural pose prior for RL |
| Hinge coordinate | Wrapped mod 360 at the clamp and in the rows | Unbounded real; limits apply to the raw value | O-01 does not exist in MuJoCo by construction |
| Ball-joint limit | Single cone on rotation-vector magnitude | Single max-angle limit on the joint quaternion | *Same limitation.* MuJoCo's workaround is three hinges |
| Actuation | Direct joint torque × authored force-vs-length curve | Gear/gain/bias actuators, tendons, `muscle` with Hill FLV + activation | No velocity dependence, no activation lag (X-02) |
| Broadphase | Static O(n²) list, ~503 pairs, no pre-pass | Sweep-and-prune AABB + contype/conaffinity masks | See P-05 |
| Narrowphase | Plane–sphere, segment–segment for capsules | Analytic primitives, convex meshes (MPR), heightfields, SDFs | Ground is one infinite plane; no terrain |
| Self-collision | Limb-vs-limb, different limbs only, off by default | Full pairwise with explicit exclusion masks | See D-01, O-09, O-10, O-11 |
| Sensors | Hand-built observation vector | Declarative sensor set, noise-capable | Absent; low priority |

## 5. What to steal, ranked by payoff over effort

**1. Per-DOF armature.** One scalar per DOF added to the joint-space inertia before the
reduction: `D = Max(dot(S,U), eps) + Armature[DOF]` in Pass 2 of both step variants and
the identical line in `ComputeArticulatedInertias`; for ball joints, added to `Irot`'s
diagonal before `Inverse3x3`. This is the standard stabilizer for RL rigs and why MuJoCo
humanoids are well-behaved at 500 Hz with ~10 iterations. It raises every joint's
effective inertia, directly reducing the stiffness of the coupled system — the thing entry
024's sweep says is out of reach of iteration count. It also conditions a chain of 1 kg
tips under a 3282 kg torso far better, adjacent to R-07. Not free: armature is fictitious
inertia and damps real dynamics, so keep it small relative to each DOF's own `D` and
expose it per-DOF rather than as one global number.
*Effort: ~4 line-equivalents plus a topology field and a driver property. Targets O-04,
O-05, R-07.*

**2. Per-DOF joint damping.** A linear `−d·qd` term folded into the bias force `p`
alongside the gyroscopic term. Between its stops a ball joint here is a perfectly
frictionless pendulum with zero dissipation anywhere — the code says so, and a diagnostic
measured Head2 and LowerMouth climbing from ~0 to 97 and 77 rad/s in 200 ms before
divergence. The response was to add cone limit rows, which damp the joint *at its stop*.
Damping is what MuJoCo relies on instead, and it dissipates across the whole range of
motion. Pair with item 7 — explicit damping has its own bound (`d·dt/I < 2`), which is
why MuJoCo's default integrator is `implicitfast`.
*Effort: small, plus a topology field. Targets the spine resonance, O-04 indirectly.*

**3. Turn on Cfm and SOR, and instrument them.** Zero new code. U-03 and U-04 are
written and off; U-01 is the instrument built to evaluate them. Expose all three on the
driver, set the debug log to watch a foot simultaneously grounded and against its ankle
stop, and read the per-iteration traces. Worth doing *before* item 4 — not because it is
likely sufficient (the entry-024 sweep argues it isn't) but because it is the only way to
close the evidence question entry 024 left open, and because the traces distinguish
"oscillation between two rows" from "genuinely unreachable fixed point", which point at
different remedies.
*Effort: three `UPROPERTY` declarations and a test harness. Targets U-01→U-04, O-03, and
the evidence gate on item 4.*

**4. Assemble the constraint matrix and solve it globally.** The real fix for O-04, and
the one entry 024 named. Instead of querying each row's diagonal in isolation, run
`SolveImpulseResponse` once per active row against a unit impulse and keep the *whole
response*, giving that row's full column of `A = J M⁻¹ Jᵀ`. Add the regularizer on the
diagonal. Then solve the small dense system properly — projected conjugate gradient, or
dense Newton with Cholesky and line search, following MuJoCo's dual formulation.

Three things make this cheaper here than it sounds: `SolveImpulseResponse` is already
linear in its applied wrench and `PairImpulseResponseAtPoint` already exploits that; the
code already spends one tree pass per row at gather time, so assembly adds storage not
traversals; and active rows number in the tens, where a dense factorization is trivial
next to 16 Gauss-Seidel sweeps each costing a tree pass per row. Still the largest item
here, and it needs P-01 measured first — a per-substep dense solve × 256 envs × 4
substeps must be budgeted, not assumed. R-04 may be a prerequisite rather than an
optimization.
*Effort: large. Targets O-04, O-05, O-06, and the tuning invalidation in O-07.*

**5. Depth-dependent impedance in place of a bias-velocity cap.**
`MaxBiasVelocity = 100` cm/s is a hard ceiling on push-out, added in entry 014 after an
unbounded Baumgarte term asked for 7188 cm/s at 600 units of depth and fired a limb away.
It works, and it is what production engines do, but it is a cliff. MuJoCo's `solimp`
varies the constraint's impedance *continuously* with violation depth: soft at first
touch, progressively stiffer as penetration grows. A principled attack on O-06 — a
shallow resting contact stays compliant and quiet, a deep one firms up smoothly rather
than saturating against a constant.
*Effort: moderate — extend `MakeContactSoftness` to take separation. Targets O-06.*

**6. Torsional friction on ground contacts.** `condim = 4` in MuJoCo terms: one extra row
resisting rotation about the contact normal. Currently a foot flat on the ground resists
sliding but can spin about its own vertical axis with nothing but the Coulomb tangent rows
opposing it — and for a single contact point those oppose nothing at all. For a creature
whose first objective is *standing*, a freely pivoting foot is directly counterproductive:
it lets the policy accumulate yaw drift the uprightness reward never penalizes. Cheap —
one more row per contact using existing machinery, with the angular effective mass from
the same impulse-response query.
*Effort: small–moderate, one new row type. Targets O-08, X-01.*

**7. Implicit treatment of the damping terms.** MuJoCo's `implicitfast` treats joint
damping implicitly in velocity, making it unconditionally stable rather than bounded by
`d·dt/I < 2`. Only worth doing once items 1 and 2 exist — it is what lets you choose
damping for physical realism instead of for the timestep. Entry 013 already implemented
exactly this idea once, for the penalty contact damper
(`F = (kp − cv)/(1 + c·dt/m)`), so the technique is not unfamiliar here.
*Effort: moderate. Makes items 1–2 safe at 240 Hz.*

**8. Stop wrapping joint coordinates.** MuJoCo hinge angles are unbounded reals and limits
apply to the raw value, which is why the lap-skip class of bug does not exist there. Here
both `ClampAngleDeg` and the limit rows wrap mod 360, so a joint crossing its entire
forbidden arc in one substep re-enters the valid window and is accepted — 571 skips even
after the rows landed. The change is to bound the raw accumulated angle rather than its
residue. Not free: the authored ranges are genuinely wrapped (`Knee1` is [283.2, 403.6],
`Feet` is [291.3, 405.0]), so unwrapping means resolving each range to a canonical
unwrapped interval at topology-build time and defining a single permitted lap. Best done
as its own change with its own test, since it touches the one function that has already
produced two distinct bugs.
*Effort: moderate, historically bug-prone function. Targets O-01 — deletes it rather than
mitigating it.*

## 6. What this solver has that MuJoCo does not

**An exact articulated effective-mass query as first-class API.**
`ImpulseResponseAtPoint` returns the velocity change of a world point from a linear
impulse there, validated to machine precision against the closed form
`1/m + (r × n)ᵗ I⁻¹ (r × n)` — entry 014's GATE reported `relErr = 0` on both cases.
MuJoCo obtains the same information from `J M⁻¹ Jᵀ`, but not as a direct callable question
about an arbitrary point. This primitive is what makes the impulse contact model possible
with no tuned stiffness at all.

**Curve-authored anatomical muscle strength.** MuJoCo's muscle model is parametric — a
Hill-type FLV curve with fitted constants. This project has a bespoke editor tool where
per-joint extension and flexion strength are drawn as curves against the joint's own
authored range of motion, with L/R mirroring. A different and arguably better authoring
story for a creature nobody has real biomechanical data for. Also only half-connected
(X-02 has no velocity term, U-10 has two ignored scalars).

**Native to Unreal, with a deployment path.** Builds a topology directly from a
`USkeletalMesh`'s reference skeleton, poses a `UPoseableMeshComponent`, runs inside PIE
against real assets. MuJoCo has none of that, and bridging it would mean maintaining a
model export pipeline and a state round-trip. `MutoTopology.h` has no MuJoCo counterpart —
and it is where a large share of this project's real bugs have lived (entries 008, 009,
022 are all topology or authoring bugs, not solver bugs).

**A written record of negative results.** Rarer than any solver feature.
`SOLVER_DEBUG_LOG.md` records refuted hypotheses, retracted claims, and three separate
occasions where an instrument produced a confident wrong answer and was caught by
disagreeing with a direct observation. That discipline is why this document could be
written at all — and N-01 is the one place it has slipped.

## 7. Where the comparison stops being useful

**The cost profiles are not comparable.** MuJoCo's reference implementation is float64 and
simulates one model per call; its Newton solver is budgeted against that. This project
exists to run 256 environments 8-wide in float32 inside a game engine. Adopting MuJoCo's
*algorithms* is right; assuming its cost profile transfers is not. Item 4 needs P-01
measured before it is committed to, and R-04 may have to land first.

**Every borrowed constant needs a unit rescale.** MuJoCo is nominally unit-agnostic but
its defaults — `solref` timeconst 0.02, gravity −9.81, published armature and damping
magnitudes — are calibrated for SI metres. This solver runs in centimetres. That exact
mismatch produced entry 001's mass-versus-length scale inconsistency and entry 017's two
off-by-millions constants, which between them cost most of a session. Any number lifted
from a MuJoCo model or paper needs converting deliberately and documenting at the field.

**MJX is the more honest comparison than MuJoCo C.** For batched RL the relevant reference
is MJX or MuJoCo Warp — float32, GPU, fixed iteration counts chosen so the solver stays
compilable, and CG or Newton rather than PGS. Much closer to this project's constraints,
and it makes the same trade: it accepts float32 and a fixed iteration budget, but it does
*not* accept row-by-row Gauss-Seidel. Which is the whole argument of §3.

**Two open items are shared limitations, not gaps.** O-02's single-cone ball-joint limit
is also MuJoCo's model — its ball joint takes one max-angle range too, and the standard
workaround when independent swing ranges are needed is to model the joint as three hinges.
And neither engine claims energy conservation under contact. Copying MuJoCo would fix
neither.
