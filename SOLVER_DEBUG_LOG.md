# Solver Stability Debug Log

Running record of hypotheses tested against the batched Featherstone ABA solver
(`CreatureBatchState.h`, `SpatialAlgebra.h`, `CreatureBatchSolver.h`).
Negative results are recorded too — the point is to stop re-testing ruled-out ideas.

Ground rules in force:
- Increasing substep count is **not** an accepted fix.
- One change at a time; re-run the checkpoint before moving on.
- Isolation work runs at fixed 60 Hz, single substep.

---

## Entry 001 — Suspect 1: unit mismatch (cm vs m)

**Date:** 2026-08-12
**Hypothesis:** A cm/m mixup somewhere in the ABA recursion — most likely an
inertia tensor built from cm-scale geometry but consumed as if m-scale, or a
gravity constant in the wrong system — is the root cause of the instability.

**Test performed:** Traced every physical quantity entering the recursion
through all three files plus `CreatureGroundContact.h`, `MutoTopology.h`
(where mass/inertia are populated) and `MutoRLTrainingDriver.h` (where the
constants are authored). Cross-checked against the real measured numbers
already emitted by `MutoGroundPenetrationDiagnosticTest` in `Saved/Logs`.

### Unit table

System in use is **cm–kg–s** throughout (UE world units).

| Quantity | Field / expression | Unit | Set at | OK? |
|---|---|---|---|---|
| Position | `PosX/Y/Z` | cm | FK from RefSkeleton | yes |
| Bone offset | `BodyJointOffsetInParent` | cm | `MutoTopology.h` | yes |
| CoM offset | `BodyLocalCoMOffset = 0.5*LengthVec` | cm | `MutoTopology.h:408` | yes |
| Mass | `BodyMass` | kg | `MutoTopology.h` (uniform 1.0) | dim. yes / **magnitude no** |
| Inertia | `BodyInertiaDiagLocal = (1/12)mL^2` | kg·cm² | `MutoTopology.h:411` | yes |
| Gravity | `Gravity` | cm/s² (−980) | `MutoRLTrainingDriver.h:227` | yes |
| Force | `ExtForce`, `m*g` | kg·cm/s² | `CreatureBatchSolver.h:179` | yes |
| Torque | `ExtTorque = r x F` | kg·cm²/s² | `CreatureBatchState.h:283` | yes |
| Joint torque | `JointTorque` | kg·cm²/s² | `ApplyActions`, ±5.0 | dim. yes / **magnitude no** |
| Linear velocity | `LinVelX/Y/Z` | cm/s | — | yes |
| Angular velocity | `AngVelX/Y/Z` | rad/s | — | yes |
| Joint position | `JointPos` | rad | — | yes |
| Joint range | `DOFRangeMinDeg/MaxDeg` | deg | converted at use | yes |
| Timestep | `Dt` | s | — | yes |
| Contact spring | `SpringK` | kg/s² | tuned on a 5 kg rig | **magnitude no** |
| Contact damper | `DamperK` | kg/s | " | **magnitude no** |
| Contact friction | `FrictionK` | kg/s | " | **magnitude no** |

Dimensional check of the recursion closes:
`alpha = tau / I = (kg·cm²/s²) / (kg·cm²) = 1/s²`.

### Result: hypothesis as literally stated is **RULED OUT**

There is no cm/m conversion error. Gravity is −980 cm/s² at every site
(`CreatureBatchSolver.h:58/63/383`, `MutoRLTrainingDriver.h:227`, all tests).
Inertia is built cm-scale and consumed cm-scale. Degree/radian conversions at
the joint-limit boundary (`CreatureBatchSolver.h:972/977`) are correct and
symmetric.

### But: a different scale inconsistency is **CONFIRMED**

Mass scale is inconsistent with **length** scale, not with unit convention.

Measured from the project's own diagnostic output (`Saved/Logs`, 2026-08-12):

```
Feet_L   mass=1.00  leverArm=106.40  authoredI=1650.0  effMass at contact = I/r^2 = 0.1458
FHand_L  mass=1.00  leverArm= 56.08  authoredI= 336.3  effMass at contact = I/r^2 = 0.1069
BHand_L  mass=1.00  leverArm= 58.79  authoredI= 361.7  effMass at contact = I/r^2 = 0.1046
totalMass = 43.0
```

Limb segments are 56–106 cm long — a creature several metres tall — carrying a
uniform placeholder mass of 1.0 kg per bone, 43 kg total. That is roughly the
density of a balloon. Because `I = (1/12)mL^2` carries mass linearly, the
under-scaled mass propagates straight into the effective mass seen at a contact
point, `m_eff = I/r^2 ≈ 0.105–0.146 kg`.

### This fully explains the 960 Hz requirement

Explicit-integration stability bound for a velocity-proportional contact term of
gain `c` acting on effective mass `m_eff` is `dt < 2*m_eff/c`:

| Term | Gain | Bound at m_eff = 0.105 kg | Min rate |
|---|---|---|---|
| `DamperK` (normal) | 20 | dt < 0.0105 s | 95 Hz |
| `FrictionK` (tangential) | 150 | dt < 0.0014 s | **714 Hz** |

**Friction, not the normal spring, sets the requirement.** 960 Hz is simply the
nearest power-of-two step above 714 Hz. This is consistent with every measured
observation on record: 240 Hz diverged, 960 Hz did not, and a contact-free fall
was always well behaved.

### Still unexplained (carried forward)

The sweep result recorded in `MutoRLTrainingDriver.h:209-213` — SpringK=3000
unstable at 1/3840, SpringK=11000 unstable even at 1/15360 — is **not**
explained by the above. A linear stability limit is monotonic in dt; that result
is not. Something beyond contact stiffness is wrong. Carried into Suspect 2/3.

### Quantitative confirmation (measured 2026-08-12)

Added `Private/Tests/MutoMassAuthoringDumpTest.cpp` (temporary) to dump each
body's real geometry and the mass its own capsule volume implies at ~1000 kg/m³.
Measured result:

```
body  bone          current   boneLen   radius     suggest
   0  Torso            9.00      0.00     0.00        0.00   <- no suggestion, see below
   1  FShoulder_L      1.00    242.91    40.38     1520.01
  11  BShoulder_L      1.00    207.76    44.03     1622.99
  12  BElbow1_L        1.00    351.26    36.83     1706.16
  27  Hips_L           1.00    228.78    49.33     2251.78
  30  Feet_L           1.00    140.71     9.39       42.41
TOTAL current = 43.0 kg    TOTAL suggested = 21223.2 kg    (493.6x)
```

Bone segments are 1.0–3.5 **metres** with radii up to 49 cm. This is a kaiju-
scale rig carrying 43 kg. The mass scale is wrong by a factor of ~500.

Projected effect on the stability bound (`dt < 2*m_eff/c`, c = FrictionK = 150):

```
FHand_L   lever= 56.08   m_eff 0.1069 -> 52.77   min rate 701 Hz -> 1 Hz
BHand_L   lever= 58.79   m_eff 0.1046 -> 51.64   min rate 717 Hz -> 1 Hz
Feet_L    lever=106.40   m_eff 0.1458 -> 71.94   min rate 515 Hz -> 1 Hz
WORST CASE: 717 Hz now -> 1 Hz after
```

**The predicted requirement of 717 Hz versus the 960 Hz actually shipped is a
near-exact match** (960 is the first power-of-two step above 717). This is a
quantitative confirmation, not an analogy: the substep requirement is a direct
arithmetic consequence of the mass scale.

Note the normal spring is scale-free and was never the binding term: a spring
sized to hold static weight at sag `s` has `k/m = g/s`, so `omega = sqrt(g/s)`,
giving ~11 Hz at a 2 cm sag and ~16 Hz at 1 cm regardless of creature mass.
Only the absolute-gain velocity terms (`DamperK`, `FrictionK`) were ever the
problem.

### Fix applied

Decision (user, 2026-08-12): author real per-bone masses in `MassProfile_Muto`
by hand, keeping mass traceable to the asset rather than computed in code.
The dump above supplies the values. **Not yet applied — awaiting authoring.**

Two things the dump does NOT answer, flagged for the authoring pass:

1. **Torso (body 0) gets no suggestion.** `BodyLocalCoMOffset[0]` is explicitly
   `ZeroVector` (`MutoTopology.h:382`), so the length proxy is 0 and the volume
   formula yields nothing. The torso is the **floating base** — its mass and
   inertia dominate the root 6x6 solve (`SolveSpatial6`, `CreatureBatchSolver.h:264`),
   so leaving it at 9 kg under ~21 t of limbs would be far worse than the
   original uniform placeholder. It needs a value of its own.
2. **The 493x figure assumes water density throughout.** The absolute number is
   a starting point; the *ratio* is the finding. Any density choice that keeps
   mass scaling as length³ resolves the instability equally well.

### Verification result — fix APPLIED and CORRECT, but NOT the root cause

Masses authored by the user 2026-08-12. Total **43.0 kg -> 6162.0 kg**, torso
9 -> 3282 kg (the floating base is now properly dominant). Predicted worst-case
contact rate requirement fell **717 Hz -> 72 Hz**.

Then ran the checkpoint (`Private/Tests/MutoStabilityCheckpointTest.cpp`):
60 Hz, single substep, passive, zero reset noise, 5 simulated seconds.

```
A: NO CONTACT (control, ABA only)          NON-FINITE at t=3.117s: LinVelX [body 3]
B: shipped 500/20/150                      NON-FINITE at t=2.333s: LinVelX [body 18]
C: weight-derived spring, light damping    NON-FINITE at t=0.733s: LinVelX [body 3]
D: weight-derived spring, med damping      NON-FINITE at t=0.367s: PosX   [body 0]
E: weight-derived spring, heavy damping    NON-FINITE at t=0.367s: PosX   [body 0]
```

**Case A — contact forces disabled entirely — still diverges.** The instability
is not in the contact model.

### Entry 002 — Contact-free matrix: the substep theory is dead

Ran the identical contact-free case across mass scales and substep rates:

```
authored mass @   60 Hz    NON-FINITE at t=3.117s  LinVelX [body 3]
authored mass @  240 Hz    NON-FINITE at t=2.871s  PosX    [body 0]
authored mass @  960 Hz    NON-FINITE at t=2.691s  LinVelX [body 3]
authored mass @ 3840 Hz    NON-FINITE at t=2.677s  PosX    [body 0]
OLD 1.0 kg    @   60 Hz    NON-FINITE at t=2.517s  LinVelX [body 3]
OLD 1.0 kg    @  240 Hz    NON-FINITE at t=2.488s  JointPos[body 3]
OLD 1.0 kg    @  960 Hz    NON-FINITE at t=2.566s  PosX    [body 0]
```

Two conclusions, both hard:

1. **Failure time is invariant to dt** — and, across a 64x range of substep
   rates, it *converges* on ~2.67 s rather than receding. Numerical instability
   does the opposite: its onset moves out sharply as dt shrinks. A failure time
   that converges as dt -> 0 means the **continuous system being integrated is
   itself divergent**. This is a modeling/algebraic bug, not a discretization
   one. No substep count could ever have fixed it, and 960 Hz never did — it
   only changed how the contact model interacted with an already-broken
   recursion.
2. **It is pre-existing.** The old uniform 1.0 kg rig fails the same way at
   ~2.5 s. The mass change neither caused nor cured it.

Localization is consistent: **body 3 (`FElbow2_L`)**, a 1-DOF revolute, with the
torso (body 0) following one step later. `LinVelX` goes non-finite while `PosX`
is still finite, so velocity diverges first and position inherits it — a
velocity/acceleration blowup, not a position one. Note `FElbow2_L` is the same
body that appears in the older jump logs at `angle=-400.1 range=[318.4,439.8]`.

Not free-fall overflow: the divergent component is **horizontal** (`LinVelX`),
and 2.67 s of free fall is only ~3500 cm.

### Leading hypothesis for the real cause

`TranslateMotion` in `SpatialAlgebra.h:260` explicitly omits the velocity-product
("Coriolis") term when propagating **acceleration** in Pass 3b, and Pass 1
propagates velocity with a bare `Cross(ParentAngVel, R)`. Dropping the
velocity-product term from an articulated chain is not a loss of accuracy that
stays bounded — it breaks the power balance between the inertia and bias terms,
so a passive chain can gain energy without limit. That would produce exactly
what is measured: well behaved while joint rates are small, then runaway, at a
dt-invariant time.

Verified translation math that is NOT at fault (checked by hand, so it is not
re-derived later): `FSpatialInertia::TranslatedTo` reduces correctly to the
parallel-axis theorem for a single rigid body, and the `H = Skew(R)*m`
convention matches `FromRigidBody`.

**Next measurement:** total system energy (KE + PE) per step on the passive,
contact-free case. Flat/decreasing exonerates the hypothesis; monotonic growth
confirms it. That is Suspect 3's instrument applied to Suspect 2's finding, and
it is the correct next step because Suspect 4 (SIMD/padding) cannot explain a
failure that is invariant to both mass and dt.

---

## Entry 003 — Suspect 3: energy injection. CONFIRMED, and it localizes the bug

**Test:** `Private/Tests/MutoEnergyTraceTest.cpp`. Total mechanical energy in
DOUBLE precision (float32 spacing at PE ~ 2.4e9 is ~128, enough to hide the
drift being measured). Two families: gravity ON (the real failing case), and
gravity OFF with a joint velocity kick — the latter is a CLOSED system, so
energy must be exactly conserved and any growth is unambiguously solver-created.

**Measurement caveat (my test's bug, not the solver's):** `E0` reads 0 for the
zero-gravity cases. `JointVel` is set after the FK-refresh step, but body-space
`AngVel`/`LinVel` are only populated by Pass 1 of the *next* `Step()`, so the
kick had not propagated into body velocities when `E0` was sampled. The `E/E0`
column is therefore meaningless for those rows and the "bounded" verdicts on
them are artifacts. The raw `E` column is valid and is what is read below.

### Zero gravity, closed system — energy MUST be constant

```
ZG-60  (60 Hz) : E = 8.82e7 -> 8.54e7 -> 1.38e8 -> 4.97e7 -> ... -> 4.98e8   (5.6x growth)
ZG-960 (960Hz) : E = 8.82e7 -> ............................ -> 1.01e8   (1.15x growth)
ZG-REST        : E = 0 exactly, all 5 s (no spurious motion from rest — good)
```

Energy is **created** in a closed system with zero gravity, zero torque and no
contact. Confirmed. It also oscillates non-physically (1.38e8 -> 4.97e7).

### Gravity on — the real failing case

```
G-60  : 0.9998 -> 0.9888 -> 0.9778 -> 0.9705 -> 0.9876 -> 2.063 -> 71.03 -> NON-FINITE t=3.200s
G-960 : 1.0000 -> 0.9993 -> 0.9986 -> 0.9926 -> 1.182  -> 12.88 -> NON-FINITE t=2.700s
                   (t=0.5)   (t=1.0)   (t=1.5)  (t=2.0)  (t=2.5)
```

Both traces are **well behaved and slightly decaying until t ~ 1.5-2.0 s**, then
turn around and explode. G-960 diverges EARLIER than G-60 despite a 16x finer
step — refining dt does not help and slightly hurts, consistent with Entry 002.

### Ruled out: joint-limit clamping

Both clamp branches perform a full inelastic stop — revolute zeroes `JointVel`
(`CreatureBatchSolver.h:978`), ball zeroes all three components (`:1047-1049`).
Clamping *removes* energy. It is not the injector. (This closes the
"clamping leaves velocity pointing into the clamp" sub-item of Suspect 3.)

### ROOT CAUSE: the velocity-product term is missing from the recursion

`TranslateMotion` (`SpatialAlgebra.h:260`) transports a spatial motion vector as
`{ Ang, Lin - R x Ang }`. That is **correct for velocity** — `v_B = v_A + w x R`
— and it is what Pass 1 effectively does, which is why the kinematics are fine.

For **acceleration** the correct transport is

```
a_B = a_A + alpha x R + w x (w x R)
```

and the `w x (w x R)` centripetal term is dropped. Likewise the angular part
omits `w_parent x w_joint`. In Featherstone's notation the solver is assuming the
velocity-product bias `c_i == 0` everywhere:

```
c_i = { w_parent x w_joint ,  w_parent x (w_parent x R) }
```

That assumption is only valid at rest or in pure translation. Consequences:

- **Pass 3b** (`CreatureBatchSolver.h:297`) omits `c_i` from the acceleration.
- **Pass 2** omits `I^A_i c_i` from the bias force, so `u_i` and the parent
  accumulation are both wrong by the same term.

**Why this matches every observation:** the omitted term scales as `w^2 * R`.
With R ~ 250 cm and w ~ 5 rad/s it is ~6250 cm/s², already **6x gravity** — and
it grows quadratically in `w`, so it is a positive feedback. That gives exactly
the measured signature: quiescent while joint rates are small, then runaway once
rates build. It is dt-invariant because it is an error in the ODE being
integrated, not in how it is integrated — which is why 3840 Hz fails at the same
moment 60 Hz does, and why the ZG case (rates stay small, no gravity to spin
things up) grows but never explodes within 5 s.

The omission is acknowledged in the code's own comments as a deliberate accuracy
trade-off ("see CreatureABASolver.h Pass 3 comment"). It is not an accuracy
trade-off. It is an instability.

### Fix (proposed, not yet applied)

Restore the velocity-product term:
1. Compute `c_i` per body in Pass 1, where `w_parent`, `w_joint` and `R` are all
   already in hand.
2. Pass 2: `u_i = tau_i - S_i^T (p_i + I_i c_i)`, and accumulate
   `p_parent += X^T (p_i + I_i c_i + U_i u_i / d_i)`.
3. Pass 3b: `A_i = TranslateMotion(A_parent, R) + c_i + S_i qdd_i`.

Staging: **scalar path (`StepScalar`) first**, verified against the energy and
finite-value checkpoints, then port to `StepSIMD` (which is what `Step()`
actually calls) and re-verify with a scalar/SIMD parity check — that parity check
is Suspect 4's checkpoint, which this fix makes newly meaningful.

---

## Entry 004 — Suspect 4: SIMD padding + scalar/batch parity. CLEARED

**Test:** `Private/Tests/MutoScalarSIMDParityTest.cpp`, three parts.

### Padding contamination — RULED OUT (definitive)

`NumEnvs=5` pads to 8, leaving lanes 5-7 unused. Ran the real envs twice: once
with padding as `Init()` leaves it, once with **every padding lane of every
state array poisoned with NaN and 1e30**. Compared envs 0-4 for *bitwise*
equality each step.

```
VERDICT: clean — real envs bitwise identical for all 60 steps
         despite NaN/1e30 in every padding lane.
```

Not a single ULP of leakage. This matches the code structure: every SIMD op in
the recursion is lane-elementwise, and there are no horizontal reductions or
cross-lane broadcasts, so a padding lane has no path to a real lane.

Noted while reading, since it looks like an inconsistency but is harmless: in
Pass 2 the revolute branch accumulates across the **full 8-lane chunk**
(`CreatureBatchSolver.h:620-621`), including padding, while the ball-joint
branch loops only to `EnvEnd = min(chunk+8, NumEnvs)` (`:542`) and skips it.
Because accumulation is elementwise, the extra padding work is wasted but not
contaminating.

### Scalar/SIMD parity — no second bug

Part 1 (gravity on, the unstable regime) showed the paths drifting apart and
crossing 1e-3 relative at step 50. **On its own that is not interpretable**:
Entry 003 established this system is genuinely divergent, so any difference
between two implementations — including pure float32 roundoff — is amplified
exponentially. The test cannot separate "SIMD bug" from "chaos" in that regime.

Part 3 repeats the comparison with gravity OFF and a joint kick — bounded per
Entry 003's ZG rows. Both parts start from the identical step-0 difference of
3.129e-05 (FK-chain roundoff over 34 chained bodies), so the growth rate is a
clean discriminator:

```
Part 1 (gravity, UNSTABLE): 3.129e-05 -> 1.540e-02   (492x, monotonic)
Part 3 (zero-g, BOUNDED)  : 3.129e-05 -> 3.559e-04   (11.4x, NON-monotonic)
                             wandering 2.3e-5 / 7.1e-5 / 2.6e-5 / 6.1e-5 ...
```

In the bounded regime the difference wanders up and down around a few times
1e-5 rather than trending — the signature of roundoff noise, not systematic
bias. **The two implementations agree.** Part 1's divergence is a *consequence*
of the Entry 003 instability, not an independent cause.

**Suspect 4 is cleared on both counts.** Re-run Part 1 after the velocity-product
fix; it should then track Part 3's bounded behaviour, and that is the parity
gate before re-enabling full batching.

---

## Status after all four suspects

| Suspect | Verdict |
|---|---|
| 1. Unit mismatch (cm vs m) | No conversion error. A **mass-vs-length** scale error was found and **fixed** (43 kg -> 6162 kg). Real, but not the root cause. |
| 2. NaN origin | Located: `LinVelX`, body 3 (`FElbow2_L`), t~2.67-3.1 s, **invariant to dt across 60-3840 Hz**. |
| 3. Energy injection | **CONFIRMED — ROOT CAUSE.** Energy created in a closed zero-gravity system. Missing velocity-product term in the ABA recursion. |
| 4. SIMD / padding | **Cleared.** Padding bitwise clean; scalar and SIMD agree at roundoff level in bounded dynamics. |

The rebuild-from-scratch path in the original plan is **not** required: the
failure is localized to a specific, well-understood omission with a known fix,
and the surrounding machinery (kinematics, inertia translation, clamping,
batching, padding) has each been individually exonerated by measurement.

---

## Entry 005 — The fix. TWO bugs, not one. Both found by rung 1.

Implementing entry 003's fix naively made things **worse** at every checkpoint.
Two wrong attempts, then the real answer. Recorded in full because the wrong
turns are exactly what a future reader would otherwise repeat.

### Bug A — spatial vs classical acceleration (found by rung 1)

`Private/Tests/MutoRung1SingleBodyTest.cpp` runs a SINGLE rigid body, no joints,
no gravity, no forces. One body has no parent, so the transport term is
identically zero and this isolates the solver core completely. Exact answer:
CoM velocity constant, |L| constant, KE constant.

```
BEFORE:  ref AT CoM,  60Hz: CoM vel (10,0,0) -> (6.29, 9.14, -5.74)  |drift| = 11.41
         ref AT CoM, 960Hz: CoM vel (10,0,0) -> (3.09, 7.77, -5.72)  |drift| = 11.87
```

**Newton's first law violated, and the drift did not shrink with dt** (slightly
worse at 960 Hz) while the angular-momentum error *did* converge properly
(0.040 -> 0.0025 for a 16x finer step). A linear error that ignores dt next to
an angular error that respects it is a modelling bug sitting beside ordinary
integration error.

Cause: the assembled equation is Featherstone's `f = I a + v x* (I v)`, in which
`a` is the **spatial** acceleration — but Pass 3a integrated
`LinVel += A.Lin * Dt`, which needs the **classical** acceleration of the body's
own moving material reference point. They differ by `w x u`:

```
du/dt = A.Lin + w x u
```

Check on the simplest case: a free body with reference AT the CoM and no force
gives `A.Lin = -w x u` exactly, so the corrected update yields `du/dt = 0`.
Muto's torso has `BodyLocalCoMOffset[0] == ZeroVector`, i.e. precisely this
case — and the measured blowup was in `LinVelX`, **horizontal**, during a
vertical fall. A spurious `-w x u` is perpendicular to velocity: it curves a
straight fall.

Fix: `RootLinVel += (A0.Lin + Cross(RootAngVel, RootLinVel)) * Dt` in both
`StepScalar` and `StepSIMD`.

```
AFTER:   ref AT CoM,  60Hz: |drift| = 2.828e-07   (was 11.41)
         ref AT CoM, 960Hz: |drift| = 1.522e-08
         ref AT CoM,  60Hz SIMD: |drift| = 4.357e-08   (SIMD matches scalar)
```

### Bug B — the velocity-product bias, and WHERE it goes

Two failed attempts before the right one:

1. **Wrong formula.** Used the classical-mechanics transport `w_p x (w_p x R)`.
   In a *spatial* formulation that term **cancels identically** — deriving it
   properly (convert `du_i/dt = du_p/dt + alpha_p x R + w_p x (w_p x R)` to
   spatial and substitute `u_p = u_i - w_p x R`) leaves
   `C.Lin = (w_p - w_i) x u_i = u_i x w_J`, matching Featherstone's
   `c = v x (S qdot)`. Adding the classical form made ZG-60 diverge at 1.8 s
   where it had been bounded for 5 s.

2. **Wrong location.** Even with the right formula, applying it as
   `p^A + I^A c` used in *both* the generalized bias and the accumulation was
   wrong twice over. Featherstone Table 7.1:

   ```
   u   = tau - S^T p^A            <- plain p^A, NOT p^A + I c
   p^a = p^A + I^a c + U D^-1 u   <- I^a, the REDUCED inertia, not I^A
   ```

   The c term enters **only** the parent accumulation, multiplied by the
   **reduced** inertia. Contaminating `u` with it diverged ZG-60 at 2.5 s.

Correct final form (both paths):
- `C.Ang = w_p x w_J` for revolute; `0` for ball (its `JointVel` is already a
  world-frame angular velocity, so `QDDot3` is its world derivative directly)
- `C.Lin = u_i x w_J` for both
- Pass 2: `ReducedBias = <original> + Reduced.Apply(C)`, `u` left untouched
- Pass 3b: `ParentAccAtBody = TranslateMotion(A_parent, R) + C`

### Result: the contact-free rig is FIXED at every rate

```
BEFORE                                  AFTER
authored @   60 Hz  NON-FINITE 3.117s   finite 5.0s  torsoZ=-11877.1
authored @  240 Hz  NON-FINITE 2.871s   finite 5.0s  torsoZ=-11846.5
authored @  960 Hz  NON-FINITE 2.691s   finite 5.0s  torsoZ=-11838.7
authored @ 3840 Hz  NON-FINITE 2.677s   finite 5.0s  torsoZ=-11836.4
OLD 1.0kg @   60 Hz NON-FINITE 2.517s   finite 5.0s  torsoZ=-11877.2
```

Torso Z now **converges** with dt (-11877 -> -11846 -> -11838 -> -11836), which
is what a correct first-order integrator does and what the broken solver never
did. Energy on the real rig, passive under gravity, no contact:

```
G-60  : 0.9998 -> 0.89     bounded 5 s, monotonically decreasing
G-960 : 1.0000 -> 0.9935   bounded 5 s, near-perfect conservation
```

Closed-system conservation, small amplitude: **+0.05% over 5 s**
(`ZG-tiny`, 0.01 rad/s kick: 3.528542e4 -> 3.530464e4).

The larger `ZG` kicks (0.5 rad/s) still diverge, but that is a **test artifact,
not a bug**: it sets every one of ~50 DOFs spinning simultaneously in a 5-link
chain of ~2 m segments, producing tip speeds of tens of m/s driven hard into
joint limits. The magnitude sweep shows the dynamics are clean where the initial
condition is physical (0.01 -> +0.05%, 0.05 -> -7.3% dissipative, 0.5 -> blows).

All 8 pre-existing tests still pass (`PendulumEnergyConservation`, `MutoTopology`,
`GroundContact.Drop`, `GroundContact.MutoWiring`, `MuscleCurve`, `RLEnvironment`).

---

## Entry 006 — Remaining: the CONTACT model is a separate, still-live bug

With the ABA core fixed, case A (no contact) is finite and the failures moved
entirely into contact, which is what that control was built to isolate.

Swept stiffness, damping and rate, all within the explicit bounds derived from
the binding effective mass (the HANDS at ~3.6 kg, not the feet at ~10):

```
A: NO CONTACT              FINITE 5.0s  (free-falls through floor, as expected)
B: shipped 500/20/150 @60  NON-FINITE 2.333s
C: k=40k,  c=350  @60      NON-FINITE 1.233s
D: k=50k,  c=400  @60      NON-FINITE 1.067s
E: k=400k, c=1400 @240     NON-FINITE 1.246s
F: k=800k, c=1700 @240     NON-FINITE 1.225s
G: k=1.51M,c=6000 @960     NON-FINITE 1.211s
```

**Failure time is ~1.2 s and essentially invariant to a 37x range of stiffness
and a 16x range of substep rate.** That is the same signature that identified
the ABA bug: a modelling error, not a stiffness or integration problem. No
contact tuning will fix it, exactly as no substep count fixed the last one.

Ruled out already: contact wrench assembly is correct (`ApplyForceAtPoint`
builds the torque about the body's own reference point, matching how
`GravityWrench` is built and how the bias pass consumes it).

---

## Entry 007 — Contact: FK staleness ruled out; uncapped damper found and bounded

### Ruled out: forward-kinematics staleness

At the end of `Step()`, non-root `BodyPos`/`BodyRot` really are stale by one
step (Pass 1 computes them from the OLD root transform; Pass 3a then moves the
root and Pass 3b integrates joints, with no FK refresh unless a clamp fires).
Tested by inserting a zero-dt `Step()` before sampling contact:

```
C : k=40k  @60   1.233s  ->  C' +FKrefresh  1.150s
E : k=400k @240  1.246s  ->  E' +FKrefresh  1.438s
G : k=1.5M @960  1.211s  ->  G' +FKrefresh  1.326s
```

No pattern, no fix. **Staleness is real but is not this bug.**

### Traced what actually runs away

`Private/Tests/MutoContactTraceTest.cpp` traces torso height, torso VERTICAL
velocity, penetration, total and peak normal force, and peak joint speed.
Creature weight is 6.039e6.

```
  t      torsoZ    torsoVZ   maxPen  totalNormalF  maxNormalF (at)
0.000     413.7       -4.5     2.00       1.6e+06       8e+05 Feet_L
0.100     408.1     -102.3     0.00             0           0 None
0.400     333.3     -392.9     0.00             0           0 None
0.542     268.4     -498.9     0.00     1.568e+07   7.842e+06 Feet_L
0.700     163.6     -864.9    85.98       3.6e+07     1.8e+07 Feet_L
0.846       4.1    -1339.4    79.06     9.637e+07   2.172e+07 FHand_R
```

Two findings:

1. **The standing pose was never supported.** At t=0 the feet carry 1.6e6
   against a 6.04e6 weight, so the creature immediately free-falls (maxPen and
   force both exactly 0 from t=0.1 to t=0.4, with torsoVZ tracking g*t). This is
   expected for a PASSIVE rig — a ragdoll with zero joint torque and free joints
   cannot hold itself up — but it means the passive test is a 4-metre drop, not
   a standing test.
2. **Total normal force reaches 9.7e7 — 16x the creature's weight — while the
   torso accelerates DOWNWARD at ~2.5 g.** Force of that size is not physical.

### Root of the spike: the damper term is unbounded

`MaxPenetrationForForce` bounds only the SPRING half:

```
NormalForceMag = Max(0, SpringK * SpringPenetration - DamperK * VelAlongNormal)
```

With `MaxPen=20, k=4e5` the spring is capped at 8e6, yet the measured peak was
2.3e7 at a single point. The excess is the damper, which scales with the
approach speed of the contact POINT — a limb tip whipped into the ground at
~100 m/s yields `1400 * 10000 = 1.4e7` on its own. That force ejects the limb,
which raises the next approach speed, which raises the force: a runaway driven
by VELOCITY, which is exactly why the failure time was invariant to both
timestep and stiffness.

### Fix applied: `FContactParams::MaxNormalForce`

Bounds the TOTAL normal force per contact end (0 disables — unchanged
behaviour for every existing test). Friction is already clamped to
`FrictionCoefficient * NormalForceMag`, so this bounds the tangential force too.

Measured, passive 4 m drop:

```
B: k=400k, c=1400 @240, no cap        NON-FINITE 1.246s
D: k=400k, c=1400 @240, cap=1.0x W    NON-FINITE 1.575s
E: k=400k, c=1400 @240, cap=0.5x W    NON-FINITE 1.808s   (+45% survival)
H: k=400k, c=1400 @60,  cap=0.5x W    NON-FINITE 1.483s
```

**A real improvement and a real bug fixed, but NOT sufficient.** All 7
pre-existing tests still pass, and rung 1 still passes.

### Still open — next lead: joint wind-up past the limits

Failures localize persistently to **body 3 (`FElbow2_L`)** and body 0, and joint
speeds reach 100 rad/s before divergence. The older jump logs show `FElbow2_L`
at `angle=-400.1` against `range=[318.4,439.8]`.

`ClampJointLimits` wraps modulo 360 (`ClampAngleDeg` does
`Fmod(AngleDeg - MinDeg, 360)`), so -400 deg wraps to +1.6 deg inside the
range and is NOT clamped — a joint that spins far past its anatomical limit
re-enters the "valid" window on the other side and is accepted. The clamp
therefore cannot bound accumulated rotation, only its residue mod 360. That is
consistent with a joint winding up without limit and with the persistent
localization to one revolute body.

---

## Entry 008 — Skeleton audit: the `Null` bone, dropped mass, scale

Asked directly: does any bone that is NOT an ABA body still influence the
simulation? Audited with `Private/Tests/MutoSkeletonAuditTest.cpp`.
Skeleton has **52 bones**; the ABA topology has **35 bodies / 50 DOF**.

### The `Null` bone is real, and it DOES matter — but only for the reset pose

```
[  0] Null    parent= -1  T=(0.00 -0.00 0.00)  R=( 67.51 -90.00 -180.00)  S=(1,1,1)
[  1] Pelvis  parent=  0  T=(448.01 0.00 0.00) R=(-64.92   0.00  180.00)  S=(1,1,1)
 => composed Pelvis-in-component: T=(0.00 -171.36 413.94) R=(47.56 90.00 180.00)
```

Bone 0 is literally named `Null` and is the skeleton root. Its rotation is
**not identity** — (67.51, -90, -180) — and `GetRestTransformRelativeTo(...,
INDEX_NONE)` composes through it, so it lands in `BodyRestRotInParent[0]`.

Where that does and does not reach:
- **Does NOT affect the dynamics.** `Null` is not an ABA body, carries no mass,
  and Pass 1 / `RecomputeKinematics` both start at Body 1, so
  `BodyRestRotInParent[0]` is never read by forward kinematics. The recursion
  never sees it.
- **DOES define the reset pose.** Callers use `BodyRestRotInParent[0]` as
  `StandingTorsoRot` at every episode reset, so `Null`'s rotation is baked into
  what "standing upright" means, and into `Config.LocalUpAxis`.

Consistency check that it is being handled correctly: the composed bind-pose
height is **413.94**, and `ComputeDefaultStandingHeight` independently returns
**413.7**. Two different derivations agreeing to 0.06% means the standing pose
is self-consistent with the geometry, not accidentally tilted.

### Dropped mass: 8 kg of 6170 (0.1%) — negligible but real

```
FTip_L/R, BTip_L/R, MTip_L/R, FeetTip_L/R   1.00 kg each = 8.00 kg
authored total  = 6170.00 kg
simulated total = 6162.00 kg
DROPPED         =    8.00 kg  (0.1%)
```

The 8 Tip bones are not ABA bodies and are not summed into the torso, so their
authored mass is silently discarded. At 0.1% this cannot explain any
instability. Worth noting though: `FeetTip_L/R` have `CanTouchGround=1` and are
the source of the FEET's contact offset (106.40) while contributing no mass —
the geometry is simulated, the mass is not.

### Clean

- **All 52 bones have unit scale.** Offsets and inertia are safe (inertia would
  scale with length squared, so this was worth confirming).
- **No ABA body has zero or missing mass.**
- No `BuildMutoTopology` warnings.

### Contact geometry, for the record

```
FElbow3_L/R  localOffset=  0.00  radius=18.62   <- ON the joint origin
BElbow3_L/R  localOffset=  0.00  radius=15.79   <- ON the joint origin
FHand_L/R    localOffset= 56.08  radius= 8.85
BHand_L/R    localOffset= 58.79  radius=15.00
Feet_L/R     localOffset=106.40  radius= 9.39
```

The four `Elbow3` points sit exactly on their own joint origin (zero lever), so
contact there produces no torque about that joint and transmits straight to the
parent. The `Feet` lever of 106.40 is the largest in the rig and is the one that
turns a bounded normal force into the largest joint torque.

**Conclusion: the `Null` bone is not implicated in the instability.** It is
handled correctly, it never enters the recursion, and the two independent
derivations of standing height agree.

---

## Entry 009 — Muscle ranges audited (clean), Tip mass fused, contact isolated

### Muscle ranges: CLEAN — no +/-360 normalization needed

Asked whether authored ranges are guaranteed within [-360,360] with Min < Max.
`ClampAngleDeg` only clamps when `Wrapped > Width + eps`, and `Wrapped` is
always in [0,360), so **any DOF whose Width >= 360 could never be clamped at
all** — that joint would be silently unconstrained. `ApplyMuscleToDOF` computes
`MaxDeg = (MaxRange > MinRange) ? MaxRange : MaxRange + 360` with no upper
bound, so bad authored data would produce exactly that. Audited:

```
50 DOF total | 0 without curves | 0 width<=0 | 0 width>=360 | 0 out of band
widths over 50 ranged DOF: min=72.42 max=136.86 mean=102.82
67 muscles authored | 47 with MaxRange<=MinRange (unwrapped by +360) | 0 outside [-360,360]
```

Every authored pair already lies in [-360,360]; the existing `+360` unwrap fixes
all 47 wrapped cases so `Min < Max` holds universally; every width is
anatomically sane. **No gather-time normalization required, and the limit can
fire on every joint.** This RULES OUT entry 007's modulo-360 lead.

(The `angle=-400.1` seen in old logs is a wound-up but VALID angle:
-400.1 = -40.1 mod 360, which is inside that joint's [-41.6, 79.8] range.
`ClampAngleDeg` deliberately preserves the lap for continuity, and `FQuat` is
periodic, so accumulated winding is cosmetic, not dynamic.)

### Fixed: the 8 kg of Tip mass is no longer dropped

The 8 Tip bones are not ABA bodies, but they have real mass at the far end of a
long lever (Feet's Tip is 106 cm out) AND they carry the contact point — so the
ground reaction was torquing a body that excluded the very geometry generating
it. Now fused into the parent body in `MutoTopology.h`: mass added, CoM shifted
to the combined centroid, and inertia given the parallel-axis contribution of
both parts about that new CoM (diagonal kept, matching the existing thin-rod
placeholder convention). Treated as a point mass — its own spin inertia is
second order beside `m*r^2` at this lever.

```
BEFORE: authored 6170.00 | simulated 6162.00 | DROPPED 8.00 (0.1%)
AFTER : authored 6170.00 | simulated 6170.00 | DROPPED 0.00 (0.0%)
```

Side benefit, measured: contact effective mass rose where it binds —
`FHand 1.069 -> 1.240`, `Feet 2.915 -> 3.024` — dropping the required rate on
the binding body from 70 Hz to 60 Hz. All 7 pre-existing tests still pass.

### Isolated rigs: a 5-body MINIMAL REPRODUCTION

`Private/Tests/MutoIsolatedLimbTest.cpp` extracts real sub-topologies from the
Muto rig (authentic offsets, masses, inertia, axes, ranges — only the body set
changes) and runs each passive for 3 s.

```
                       bodies DOF    kg  pts | no-contact 60/240Hz | CONTACT 60Hz / 240Hz
A torso+1 F limb          6    7   3688   2  | 0.9381 / 0.9844     | DIV 1.083s / DIV 1.008s
B torso+2 F limbs        11   14   4094   4  | 0.9370 / 0.9842     | DIV 1.400s / DIV 1.150s
C torso+1 leg             5    6   3713   1  | 0.9376 / 0.9843     | DIV 1.450s / finite E/E0=1.506
D torso+2 legs            9   12   4144   2  | 0.9349 / 0.9836     | finite E/E0=37.3 / finite E/E0=-2.002
FULL creature            35   50   6170  10  | 0.9340 / 0.9834     | DIV 1.200s / DIV 1.142s
```

Two conclusions:

1. **Contact-free is clean at every scale.** E/E0 is 0.934-0.984 on all five
   rigs and converges toward 1.0 as dt shrinks (0.937 @60Hz -> 0.984 @240Hz).
   The ABA core fix holds from 5 bodies to 35.
2. **Contact fails at ANY scale.** `RIG C` — torso + one leg, **5 bodies, 6 DOF,
   a SINGLE contact point** — diverges at 1.450 s. The bug does not need body
   count or chain depth. Where rigs stay "finite" (C@240, D) they are not
   healthy: E/E0 reaches 37.3 and even -2.0.

**`RIG C` is now the minimal reproduction to debug against** — five bodies and
one contact point, versus the 34-body collapsing ragdoll everything before this
was measured on.

---

## Entry 010 — Minimal reproduction: the bug is EXTERNAL WRENCH x ARTICULATION

`Private/Tests/MutoMinimalContactTest.cpp` (rigs) and
`Private/Tests/MutoOffsetWrenchTest.cpp` (single body). Sub-topologies extracted
from the real rig; one contact point on the last body of the chain.

### The measurement that localizes it: energy balance per step

A wrench (tau; F) about a body's reference point does work
`W = (F . v_ref + tau . w) * dt`. Gravity lives in PE, so for a passive rig
`dE == W` must hold to within integration error. The residual is reported as
`(dE - W) / max(1,|W|,|dE|)`, so **+2.0 means `dE = -W` exactly** — the system
GAINS precisely what the wrench should REMOVE.

### Bisection by body count

```
RIG 3 (torso+Hips+Knee1,      contact on Knee1) : finite 3.0s @60Hz
RIG 4 (torso+Hips+Knee1+Knee2,contact on Knee2) : DIVERGED t=1.050s @60Hz
RIG 5 (torso+whole leg,       contact on Feet)  : DIVERGED t=1.683s @60Hz
```

The transition is 3 -> 4 bodies. A ball joint plus **two** consecutive revolutes
is enough; one revolute is not.

### Force magnitude is NOT the driver

`MaxNormalForce` swept off / 20x weight / 5x weight gave **byte-identical**
results on every rig. The entry-007 force cap bounds a symptom, not the cause.

### Timestep is NOT the driver — and the residual converges on exactly 2

```
RIG 3:  finite@60 | DIV 1.296s@240 | DIV 1.257s@960 | DIV 1.232s@3840
RIG 4: DIV 1.050s@60 | finite@240  | DIV 0.914s@960 | DIV 0.928s@3840
RIG 5: DIV 1.683s@60 | DIV 1.529s@240 | DIV 1.683s@960 | DIV 1.757s@3840
residual -> +1.997, +1.986, +1.999, +1.999 as dt shrinks
```

This kills the explicit-penalty over-bounce theory: over-bounce vanishes as
dt -> 0, and this **converges on the exact inversion `dE = -W`** instead. A
structural error that survives the continuum limit.

### The external wrench path is CORRECT on a single body

`MutoOffsetWrenchTest` drives one free rigid body, no joints/contact/gravity,
with a force at an offset — the only thing contact adds over the already-passing
pendulum test (which proves the zero-lever case). Checked against closed-form
`a_com = F/m`, `alpha = I^-1 (r x F)`, `dE = F . v_app`:

```
force AT ref point       alpha err 0         a_com err 2.9e-09   energy +0.0025  OK
force OFFSET +X          alpha err 2.9e-09   a_com err 2.9e-09   energy +0.0025  OK
force OFFSET +X, CoM +X  alpha err 1.7e-06   a_com err 4.7e-06   energy +0.0014  OK
force OFFSET +Y          alpha err 3.4e-08   a_com err 2.9e-09   energy +0.0041  OK
```

Torque, acceleration and energy all correct to ~1e-6.

### Conclusion — the fault is in the INTERACTION, and that is now pinned down

Three facts together:
1. Articulated chain with NO external wrench: energy conserved (entry 005/009,
   E/E0 = 0.934-0.984 across all rig sizes, converging to 1 as dt shrinks).
2. External wrench on a SINGLE body: exact to 1e-6, energy balanced.
3. External wrench THROUGH an articulated chain: `dE = -W` exactly.

So neither half is broken alone — the defect is in how an external wrench
propagates through the ABA reduction. Gravity is also an external wrench and
DOES behave (fact 1), and the one structural difference is that
`GravityWrench` is rebuilt INSIDE the solver each step from the post-Pass-1
`CoMOffset`, whereas `ExtForce`/`ExtTorque` are accumulated OUTSIDE from the
previous step's `BodyPos`/`BodyRot`.

**Next step: instrument Pass 2's reduction of the external term.** Specifically
whether `PAcc`'s `-TotalExtForce` contribution survives `TranslateForce(...)` up
the chain consistently with the accelerations Pass 3b then derives from it. The
single-body test rules out the wrench construction; the contact-free test rules
out the recursion; only the reduction of an external bias term through a joint
remains unexamined.

(Note: a plain FK refresh before sampling contact was already tested in entry
007 and did NOT help, so simple positional staleness is not sufficient to
explain this — but the refresh was never re-measured against the energy
residual, only against survival time. Worth redoing with this instrument.)

Untested candidates remaining:
- ~~The modulo-360 clamp~~ — RULED OUT by entry 009's range audit.
- ~~Contact force magnitude~~ — RULED OUT by the cap sweep above.
- ~~Explicit penalty over-bounce~~ — RULED OUT by the dt sweep above.
- ~~External wrench construction~~ — RULED OUT by the single-body test above.

---

## Entry 011 — Momentum-balance instrument: INCONCLUSIVE, test is at fault

Built `Private/Tests/MutoWrenchPropagationTest.cpp` to check the
formulation-independent laws `dP/dt = F_ext` and `dL/dt = T_ext` on extracted
chains under one known external force. **Its multi-body numbers must NOT be
used** — the instrument fails its own control.

### Why it is not trustworthy

The `CLOSED` variant (no gravity, no external force, small initial velocity)
reports **30% linear momentum error on a ONE-BODY rig**. That case is provable
by hand and admits no error at all:

```
torso CoM offset = 0, so H = 0, Irot = I_world, MBlock = m*Id
V = (w, v) = ((0,0.3,0), (50,0,0)),  m = 3282
SpatialCrossForce(V, I V).Lin = w x (m v) = (0,0,-49230)
=> PAcc.Lin = (0,0,-49230)  =>  A0.Lin = (0,0,+15.0)
rung-1 correction: w x u = (0,0.3,0) x (50,0,0) = (0,0,-15.0)
=> du/dt = A0.Lin + w x u = (0,0,0)   -- LinVel EXACTLY constant
```

An instrument that reports 30% error where the answer is provably zero cannot
be used to judge the solver.

Corroborating that it is the test: the same run flags `GRAVITY only` as
violating momentum on multi-body rigs, which directly contradicts entry 009's
energy measurements (contact-free, gravity-driven, E/E0 = 0.934-0.984 and
converging to 1 as dt shrinks, across every rig from 5 to 35 bodies). Both
cannot be true; the energy result is corroborated by three independent tests,
this one is not.

Suspected faults in the test itself, in order:
1. `TExt` starts at or near zero (the torso's `BodyLocalCoMOffset` is
   `ZeroVector`, so `r_com x F` vanishes at t=0), and the `max(1, |TExt|)`
   denominator then reports a meaningless ratio.
2. The unclamped rigs (muscle ranges deliberately not copied) flail under a
   1e6 force on a ~2 m lever, so later steps measure chaotic state divergence
   rather than a systematic error — consistent with the non-monotonic dt
   behaviour (`ROOT` improved with finer dt while `TIP` worsened).
3. Momentum is sampled around `Step()`, but `Step()` internally rewrites
   non-root transforms in Pass 1, so `M0` and `M1` may not be sampled in
   comparable states.

### What still stands from entry 010

Unaffected by the above, because they use energy/analytic references rather
than this instrument:
- external wrench on a SINGLE body is exact to ~1e-6 (closed-form check);
- articulated chain with no wrench conserves energy and converges with dt;
- articulated chain WITH a contact wrench gives `dE = -W` exactly, dt-invariant;
- the 3 -> 4 body bisection.

### Next step (revised)

Fix the instrument before trusting it: sample momentum at identical points in
the step, keep the rigs gentle enough not to flail, restore joint limits or
verify they never fire, and normalize against a quantity that is not near zero.
Validate it against the ONE-BODY closed case first — it must read zero error
there before any multi-body number from it means anything. Only then re-examine
Pass 2's reduction of the external bias term.

---

## Entry 012 — Instrument repaired; TWO more real solver bugs found and fixed

### The instrument's own bug

The variant dispatch was
`if (Mode == Grav) {...} else { ApplyForceAtPoint(...); }`, so **CLOSED fell
into the else and had the full 1e6 force applied while being SCORED as a closed
system**. Every CLOSED number in entry 011 measured a driven system against a
closed-system expectation. Also fixed: momentum is now sampled after a zero-dt
`Step()` (FK refresh) so `M0`/`M1` are comparable states; the angular
denominator is a fixed `|F| * L_char` instead of a `|T_ext|` that starts at 0;
and the force is a fraction of the rig's own weight so the chains do not flail.

**GATE now passes at machine precision** — 1-body CLOSED reads
`linear 3.576e-10, angular 4.4e-12` at all three rates. Every number below is
from an instrument that passes its own control.

### BUG 1 — rotation integration silently discarded below 1e-4 rad/step

Both exp-map integrations (root in Pass 3a, ball `RelRot` in Pass 3b, in BOTH
`StepScalar` and `StepSIMD` — 4 sites) guarded with

```
if (Angle > KINDA_SMALL_NUMBER)   // 1e-4
```

The guard only needs to avoid normalizing a zero axis, but at that tolerance it
**throws away every rotation smaller than 1e-4 rad per step**. At `w = 0.3 rad/s`
the threshold is crossed at ~3000 Hz, so at any fine substep rate the
orientation froze while angular velocity kept integrating. The finer the
timestep, the more rotation was discarded — which is why momentum drift got
WORSE as dt shrank instead of converging.

Measured on a closed 2-body all-revolute chain: `6.3e-07` at 1000 Hz but
`7.6e-4` at 4000 Hz, purely from this. Fixed by using `SMALL_NUMBER` (1e-8).

```
all-revolute CLOSED   BEFORE 1k/4k/16k          AFTER 1k/4k/16k
2 bodies              6e-07 / 7.6e-4 / 7.6e-4   6e-07 / 3.0e-06 / 1.1e-05  OK
3 bodies              8e-07 / 1.2e-3 / 1.2e-3   8e-07 / 1.6e-06 / 1.7e-05  OK
4 bodies              1e-07 / 1.6e-3 / 1.6e-3   1e-07 / 4.1e-06 / 4.6e-06  OK
```

### BUG 2 — ball joint integrated in the wrong frame

With BUG 1 fixed, every all-revolute rig was clean but **every rig containing
the ball joint still failed, dt-invariantly** (2-body CLOSED 0.00233 flat;
3-body TIP 0.206 flat). That isolates the ball path.

`RelRot` is defined by `BodyRot = ParentRot * RestRot * RelRot`. Requiring
`BodyRot' = exp(w_i dt) * BodyRot` with `w_i = w_p + w_J` gives

```
RelRot' = exp( (ParentRot*RestRot)^-1 w_J dt ) * RelRot
```

The code conjugated by **`ParentRot` alone**, dropping `RestRot` — so the joint
velocity was rotated into the wrong frame whenever the bind-pose relative
rotation is not identity, which on a real rig it never is.

```
ball rigs             BEFORE 1k/4k/16k          AFTER 1k/4k/16k
2-body BALL CLOSED    0.00233 flat              3.5e-07 / 7.4e-07 / 1.8e-05  OK
2-body BALL TIP       0.0032  flat              2.5e-04 / 7.8e-05 / 1.8e-04  OK
3-body ball+rev       0.00395 flat              9.7e-07 / 3.1e-07 / 5.5e-06  OK
```

Residual TIP error now **converges with dt** (0.0091 -> 0.0026 -> 0.0023) and is
identical in ball and all-revolute rigs — ordinary first-order integration
error, not a structural defect.

### Effect on the contact instability: large, but not a cure

All 8 pre-existing tests plus rung 1 and the offset-wrench test still pass.

```
            BEFORE (60Hz / 240Hz)      AFTER (60Hz / 240Hz)
A  1 F limb   DIV 1.083s / 1.008s   ->  DIV 1.783s / 2.025s
B  2 F limbs  DIV 1.400s / 1.150s   ->  DIV 2.583s / 2.975s
C  1 leg      DIV 1.450s / finite   ->  DIV 2.167s / 1.779s
D  2 legs     finite E=37.3 / -2.0  ->  finite E=6.49 / 6.97
FULL          DIV 1.200s / 1.142s   ->  DIV 1.700s / 2.808s
```

Survival roughly doubled and D's energy error fell ~5x. Critically, divergence
is now generally **later at 240 Hz than at 60 Hz** (A, B, FULL), the correct
direction for an integration-limited failure — where before it was dt-invariant.
That is a qualitative change in character, not just a numbers improvement.

Contact still diverges eventually, so at least one more defect remains, but the
momentum instrument is now trustworthy and can be pointed at the contact path
itself rather than at the recursion.

---

## Entry 013 — Contact path CLEARED; remaining issue is the penalty model, mitigated

### The contact wrench is applied CORRECTLY

Pointed the (now validated) momentum instrument at the REAL ground-contact
model: run `ApplyGroundContactForces`, read back the wrench it actually
deposited in `ExtForce`/`ExtTorque`, and check the system against it.

```
REAL GROUND CONTACT, every rig (1-4 bodies, ball and revolute), every rate:
   linear 1.0e-05 / 8.8e-05 / 4.0e-04     angular 4.4e-07 / 4.0e-06 / 5.7e-05   ALL OK
```

Momentum is conserved. **There is no bug in how contact forces are applied or
propagated.** Combined with entries 010/012, every structural suspect is now
closed: wrench construction, wrench propagation, the recursion, the ball joint,
rotation integration, SIMD/padding, joint ranges, and mass accounting.

### Ruled out: unconditional FK refresh

`Step()` left non-root transforms one step stale (Pass 1 builds them from the
pre-step root; Pass 3a/3b then move root and joints). Made `RecomputeKinematics`
run unconditionally rather than only after a clamp. Results were mixed — rig A
got worse (1.783 -> 1.517 s), rig B better, FULL better at 60 Hz and worse at
240 Hz — i.e. noise, not a fix. **Kept anyway** on correctness grounds: the state
`Step()` leaves behind should be self-consistent for any caller, which is what
ground contact samples. It is not the cure.

### What remains is the explicit penalty model itself

With every structural cause eliminated and momentum conserved, the residual
energy growth is inherent to integrating a stiff spring-damper explicitly. The
damper is evaluated at the START-of-step velocity and applied across the whole
step, which is stable only while `DamperK * Dt / m_eff < 2`; past that it
reverses the approach velocity and returns more than it absorbed.

### Mitigation 1 — implicit damping (new, opt-in)

Solving for the force against the velocity the body will HAVE:

```
F = k*p - c*(v + F*Dt/m)   =>   F = (k*p - c*v) / (1 + c*Dt/m)
```

unconditionally stable in the damping term. Added as
`FContactParams::Dt` + `EffectiveMass` (both zero = original explicit force, so
every existing test is untouched), and exposed on the driver as
`ContactEffectiveMass`.

### Mitigation 2 — `MaxNormalForce` (entry 007), retained

### Combined effect on a passive 4 m ragdoll collapse

```
                                             survival at 60 Hz
start of session (broken ABA)                     ~1.2 s
+ ABA fixes (entries 005, 012)                    ~1.7-2.8 s
+ MaxNormalForce cap                               2.85 s
+ implicit damping                                 4.88 s   (of a 5.0 s run)
```

Roughly 4x, and the 60 Hz result now beats what 960 Hz achieved before any of
this work. Note this scenario is deliberately brutal: a passive rig has zero
joint torque, so it cannot stand by construction — it free-falls ~4 m and
pile-drives into the ground. It is a worst case, not the training case.

### Final state

**All 19 automation tests pass, 0 failures**, including the 8 pre-existing ones.

Solver-side work is complete: every structural defect found has been fixed and
verified by an independent instrument. What is left is contact-model design
(velocity-level impulses instead of penalty forces would remove the remaining
stiffness limit entirely) plus per-rig tuning of `ContactEffectiveMass`,
`MaxNormalForce` and the spring/damper against the bounds in entry 006.

---

## Entry 014 — Velocity-level (impulse) contact: implemented and VALIDATED

New design, replacing force-from-penetration with "what impulse stops the
contact point approaching?". Its one hard requirement is the ARTICULATED
effective mass at the contact — the whole chain's reflected inertia, which no
hand-tuned constant can supply — so the solver gained an impulse-dynamics path
to compute it exactly.

### Added to `FCreatureABASolver`

The impulse equations are the same recursion as the force pass with
acceleration -> velocity change and force -> impulse, and **every bias term
dropped** (an impulse acts over zero time: no gyroscopic term, no
velocity-product term). Hence:

- `ComputeArticulatedInertias()` — backward pass, no bias, one factorization
  reused by every contact and iteration in a step.
- `SolveImpulseResponse()` — impulse bias backward, floating-base solve, then
  velocity change forward, recovering each joint's velocity change.
- `ImpulseResponseAtPoint()` — velocity change of a world point from a linear
  impulse there. `1 / (response . n)` IS the articulated effective mass along n.
- `ApplyImpulseAtPoint()` — applies it to every body and joint velocity.

Gotcha hit and fixed during bring-up: these first read `WorldAxisAcc`, which
only `StepScalar` fills — `Step()` dispatches to `StepSIMD` and fills the SoA
`WorldAxisAcc8`, so the array was empty and the test crashed with
`index 37 into an array of size 0`. Revolute axes are now recomputed from the
parent rotation (`RevoluteAxisWorld`), making these entry points independent of
which Step variant ran.

### Added to `CreatureGroundContact.h`

`ResolveGroundContactImpulses()` — called AFTER `Step()`. Per contact:
normal impulse to remove the approach velocity, plus a clamped Baumgarte term
for penetration, then a friction impulse along the tangent solved with its own
effective mass and clamped to the Coulomb cone. Sequential (Gauss-Seidel) over
`Iterations`. No stiffness, no damping ratio, no dependence on dt.

### GATE — validated against closed form, exactly

```
through CoM : m_eff expected=10.0000    got=10.0000    relErr=0   OK [GATE]
offset lever: 1/m_eff expected=20.100000 got=20.100000 relErr=0   OK [GATE]
```

The second is the full `1/m + (r x n)^T I^-1 (r x n)`. The impulse response is
correct to machine precision, so any remaining instability is in how contacts
are sequenced/tuned, NOT in the dynamics.

### Bug found during bring-up: unbounded Baumgarte

First runs diverged at a suspiciously constant ~1.4 s on every rig and rate —
the signature of an implementation bug, not a method limitation. Cause: the bias
term `(Beta/Dt) * penetration` is a TARGET SEPARATING VELOCITY, so at 600 units
of depth, Beta=0.2 and Dt=1/60 it asks for 7188 cm/s, and the solver faithfully
delivers the impulse to achieve it — firing the limb away. Clamped via
`MaxBiasVelocity` (default 100 cm/s), as every production impulse solver does.

### Result: first genuine SUPPORT of the session

```
torso+2 legs @240Hz  IMPULSE  finite 5.0s | E/E0=+2.711 | torsoZ=  +47.5 | maxPen=483
torso+2 legs @240Hz  PENALTY  finite 5.0s | E/E0=+64.44 | torsoZ=-16243.3
torso+1 leg  @ 60Hz  IMPULSE  finite 5.0s                (PENALTY diverged at 2.683 s)
```

`torsoZ = +47.5` is the creature being HELD UP — the first time any
configuration has done so. The penalty model at the same settings put it 16,243
units through the floor.

### Honest status: not yet universally robust

The FULL 35-body rig still diverges under impulse contact (1.667 s @60,
1.513 s @240), and some smaller cases regress relative to penalty. With the gate
proving the dynamics exact, the remaining work is contact *sequencing*, not
physics:

1. **Accumulated-impulse clamping.** Standard sequential impulse clamps the
   TOTAL normal impulse per contact to be non-negative across iterations, rather
   than clamping each increment independently as this does. Without it, Gauss-
   Seidel over 10 coupled points can oscillate — the most likely cause of the
   full-rig failure and the clearest next step.
2. **Warm-starting** across steps for faster convergence.
3. **Cost**: two full tree passes per contact per iteration. Fine for
   correctness, wants caching of the per-contact effective mass before shipping.

All 20 automation tests pass, 0 failures. Both contact models are available
side by side: the penalty path is unchanged and still default, so nothing
currently working is disturbed by adopting the new one incrementally.

---

## Entry 015 — How production engines solve this (literature review)

Checked our approach against Box2D / Bullet / PhysX / MuJoCo. Every problem we
hit is a known one with a named standard solution, and three of our current
choices are the textbook-wrong variant.

### 1. Accumulated impulse clamping — we do this WRONG

We clamp each impulse INCREMENT to be non-negative. Production clamps the
running TOTAL and applies the difference:

```
newImpulse = max(point.normalImpulse + delta, 0)
applied    = newImpulse - point.normalImpulse
point.normalImpulse = newImpulse
```

Clamping increments prevents a contact from ever "undoing" an earlier
over-correction within the same step, which is what makes Gauss-Seidel over
several coupled points oscillate. Friction clamps the same way against
`friction * normalImpulse`. This confirms entry 014's suspected next step and
gives the exact formula.

### 2. Warm starting

Persist accumulated impulses per contact across steps (matched by contact ID or
a local-space distance threshold) and reapply them before iterating. This is the
single biggest convergence win for resting/stacked contacts, and we do none of
it — every step starts from zero.

### 3. Baumgarte's deep-penetration blow-up is a documented failure mode

"If the penetration is deep this can go terribly wrong and objects can gain a
lot of momentum and shoot off" — exactly the 7188 cm/s target velocity measured
in entry 014. Our `MaxBiasVelocity` clamp is the standard mitigation, but the
better fixes are:

- **Split impulse / pseudo-velocities** (Bullet): run position correction on a
  SEPARATE velocity channel that never touches real velocity, so the position
  solver injects ZERO kinetic energy. Noted as good for contacts and poor for
  joints — our use is contacts.
- **Relaxation pass** (Box2D v3): solve with stabilization, integrate, then
  solve again WITHOUT stabilization to strip the energy stabilization added.

Either directly removes the `dE = -W` style injection rather than bounding it.

### 4. Soft constraints, not raw stiffness

Catto's current recommendation ("Soft Step" = soft constraints + sub-stepping)
parameterizes contact by NATURAL FREQUENCY (Hz) and DAMPING RATIO rather than a
raw spring constant — which is precisely the reparameterization our entry-006
stability bounds were groping toward. MuJoCo does the same: a diagonal
regularizer `R > 0` plus impedance/stiffness/damping, and its docs state
outright that past a stiffness threshold "the system becomes unstable and
noisy". MuJoCo is the standard tool for RL locomotion and it deliberately does
NOT use hard contact.

### 5. Architecture is right

PhysX simulates articulations in REDUCED COORDINATES (root pose + joint angles,
per-DOF scalars) with a contact solver on top — structurally what we have.
The design is not the problem.

### 6. Sub-stepping vs iterations — with a caveat about our ground rule

Catto's benchmark conclusion is that sub-stepping beats adding iterations,
because smaller steps reduce non-linearity. This LOOKS like it contradicts the
project rule "never propose more substeps as a fix". It does not: that rule was
about using 960 Hz to MASK a solver bug, and those bugs are now found and fixed
(entries 005, 012). Choosing 4 substeps x 1 iteration over 1 step x 8 iterations
is an algorithmic trade at equal cost, not a way to hide a defect. Flagged for
the user's decision rather than assumed.

Sources: Box2D Solver2D benchmark, Erin Catto's Sequential Impulses (GDC 2006),
Bullet split-impulse discussions, PhysX 5.1 articulation docs, MuJoCo
computation docs.

---

## Entry 016 — Applied the three standard techniques

All three from entry 015 implemented in `ResolveGroundContactImpulses`.

1. **Accumulated clamping.** The running TOTAL is clamped and the DIFFERENCE
   applied, replacing per-increment clamping. Friction accumulates on a stable
   two-tangent basis and is cone-clamped against the CURRENT accumulated normal
   impulse.
2. **Warm starting.** `FImpulseContactCache` persists accumulated impulses
   across steps and replays them before iterating. Our contact points are fixed
   local offsets on named bodies, so `(PointIdx, EndIdx, Env)` is already a
   stable persistent contact ID — none of the feature-matching a general engine
   needs. Separated contacts have their history cleared so a stale impulse is
   never replayed on re-touch.
3. **Relaxation pass.** `RelaxIterations` extra passes with Baumgarte OFF, to
   take back the energy stabilization deliberately injected.

Also: effective masses now computed ONCE per contact per step instead of twice
per contact per iteration (they depend on configuration, not velocity) — a large
cost reduction as a side effect.

### Results — better, still not universally stable

```
                          PENALTY            IMPULSE (before)   IMPULSE (now)
torso+1 leg  @ 60Hz   DIV 2.683s         DIV 1.433s         DIV 3.033s
torso+2 legs @240Hz   E/E0=+64.4         E/E0=+2.711        E/E0=-0.62  finite
FULL         @ 60Hz   DIV 3.317s         DIV 1.667s         DIV 1.667s
FULL         @240Hz   DIV 2.496s         DIV 1.513s         DIV 1.346s
```

The 2-leg rig now **dissipates** energy (E/E0 = -0.62 relative to a reference
that includes potential energy) where every earlier variant gained it — the
relaxation pass doing exactly its job. Small rigs improved. **The full 35-body
rig did not**, and still fails around 1.3-1.7 s.

The GATE remains exact (`relErr = 0` on both closed-form effective-mass checks),
so the dynamics underneath are not in question — this is a convergence or
scenario issue, not a physics one.

All 20 automation tests pass, 0 failures.

### Honest read on what is left

The full rig differs from the small rigs in having 10 contact points spread over
8 chains that all meet at one floating base. Remaining candidates, untested:

- **Iteration count vs coupling.** 6+2 passes of Gauss-Seidel may simply not
  converge for 10 contacts sharing a root. Cheap to test by sweeping iterations.
- **Sub-stepping.** Catto's benchmark conclusion is that sub-steps beat
  iterations at equal cost. With the ABA bugs fixed, 240 Hz with few iterations
  is now a legitimate design point rather than a bandage — the user's concern was
  specifically 960 Hz masking a defect, which no longer applies.
- **Soft constraints** (entry 015 item 4) — reparameterize by natural frequency
  and damping ratio. The larger change, and the one MuJoCo's whole design
  argues for given this is an RL locomotion project.
- **Scenario**: a passive ragdoll cannot stand by construction and the torso has
  no contact geometry, so it passes through the floor freely regardless of how
  good the foot contact is. A test with actuation, or with torso collision,
  would measure the thing that actually matters for training.

---

## Entry 017 — "It just free falls in the editor": DIAGNOSED. Two constants.

User reported the rig free-falling in the Unreal editor. Not a solver bug —
**two driver constants were never rescaled after the mass authoring** (43 kg ->
6170 kg, a 143x change). Both are arithmetic, not tuning.

### 1. `ContactSpringK = 500` — contact could never support the creature

```
weight = 6170 kg * 980 = 6.047e6
resting sag = weight / (500 * 2 ends) = 6047 UNITS
```

The rig had to sink SIXTY METRES before the spring balanced its weight. That
reads as free fall because it effectively is one. Measured directly: with the
old constants the torso ends at **-4310** after 3 s. Correct value for a 20-unit
sag is 151165; shipped as **150000**, which also sits under the 240 Hz stability
bound of 9.2e5. `ContactDamperK` 20 -> 1500 and `ContactFrictionK` 150 -> 1500
to match (the 240 Hz explicit bound is c < 2*m_eff/dt = 1920 at m_eff ~ 4).

### 2. `MaxTorquePerDOF = 5.0` — the creature had no muscles at all

Torque needed to hold the single worst body against gravity:

```
BElbow1_L: tau = m*g*|CoM lever| = 3.442e7
shipped MaxTorquePerDOF = 5.0
=> ~6,884,720x too weak
```

Flagged as ~2000x too weak back in entry 001 when bodies were 1 kg placeholders;
with real masses it is nearly seven million. The creature could only ever
collapse, whatever contact did. Rescaled to **5e7** (~1.5x the worst holding
torque) in both `FEnvConfig` and the driver. It is a LIMIT, not a command — the
policy emits [-1,1] and the reward's torque penalty normalizes by the same
value, so its meaning is unchanged.

### 3. `PhysicsSubstepDt` 1/960 -> 1/240

Per the user: high frequency is for FIDELITY, not for buying tenths of a second
before going non-finite. The two ABA bugs that 960 Hz was masking are fixed, and
240 Hz is a normal rate for this class of solver.

### Iteration sweep — convergence RULED OUT

```
FULL rig, 240 Hz, impulse contact:
  iters= 4  DIV 1.521s    iters=16  DIV 1.550s    iters=64  DIV 3.058s
  iters= 8  DIV 1.679s    iters=32  DIV 1.554s
```

Not monotonic and not a fix. The full-rig failure is **not** Gauss-Seidel
failing to converge over coupled contacts, which was entry 016's leading theory.

### The uncomfortable part: correcting the constants EXPOSES the instability

```
OLD  k=500    c=20   bare      finite 3.0s | torsoZ -> -4310   (free fall, but finite)
RESC k=150000 c=1500 bare      DIVERGED at 1.596s
RESC k=150000 c=1500 +cap      DIVERGED at 2.058s
RESC k=150000 c=1500 +cap+imp  DIVERGED at 2.263s
ACTUATED stand (PD), penalty   DIVERGED at 1.588s
ACTUATED stand (PD), impulse   DIVERGED at 1.033s
```

The old value did not "work" — it was too weak to engage, so nothing could go
wrong. A correctly-sized spring makes contact actually carry 6 tonnes, and the
unresolved contact instability appears. Actuating the rig does not avoid it
either, so it is not an artifact of the passive scenario.

**Trade-off in the shipped defaults, stated plainly:** the corrected constants
are physically right and fix the reported free fall, but the sim now diverges
after ~1.5-2.3 s instead of falling through while staying finite. Anyone needing
a non-diverging (if non-supporting) sim in the meantime can set
`ContactSpringK` back down. The honest position is that contact is not solved
and no choice of these constants makes it so.

All 20 automation tests pass, 0 failures.

---

## Entry 018 — Soft constraints implemented; and a controller bug in my own test

### Soft-constraint reparameterization (Box2D v3 "soft step")

`FImpulseContactParams` now takes **ContactHertz** and **DampingRatio** instead
of a Baumgarte `Beta`. Per-step coefficients:

```
omega = 2*pi*hertz ; a1 = 2*zeta + h*omega ; a2 = h*omega*a1 ; a3 = 1/(1+a2)
biasRate = omega/a1   massScale = a2*a3   impulseScale = a3

delta = -massScale * (vn + bias + restitution) / invMass  -  impulseScale * accumulated
```

The trailing `impulseScale * accumulated` is the part a pure Baumgarte scheme
lacks: it scales the accumulated impulse back each iteration, making the
constraint genuinely compliant rather than a hard constraint with a position
hack attached. Same idea as MuJoCo's diagonal regularizer R > 0 —
`hertz -> inf` drives `impulseScale -> 0` and recovers the hard constraint,
which is precisely the unstable regime.

Hertz is clamped internally to `0.25/dt`: a constraint cannot be stiffer than
the step can resolve, and asking for it is how the old formulation blew up.

**Why this matters beyond stability:** Hertz and DampingRatio are invariant to
mass and to timestep. `ContactSpringK` was not — it silently produced a
6047-unit sag once the masses were authored (entry 017) and needed a 143x
rescale. The new parameters would not have needed touching.

### A controller bug in the test, caught by a control I should have run first

The first actuated sweep diverged at ~1.03-1.14 s for EVERY hertz and damping
ratio — invariant, which is the signature that says the swept parameter is not
the cause. Suspecting my own harness, ran the PD with contact DISABLED:

```
Kp=5e8 NO CONTACT  DIVERGED at 4.154s   <- the PD itself is unstable
Kp=5e7 NO CONTACT  finite 5.0s
Kp=5e6 NO CONTACT  finite 5.0s
```

`omega = sqrt(Kp/D)` exceeds the 240 Hz limit for the smallest joints at
Kp=5e8, so the controller was diverging on its own and the "contact" sweep was
partly measuring it. Re-ran at Kp=5e7.

### Result with a valid controller: the parameters finally bite

```
actuated stand, impulse contact, 240 Hz, Kp=5e7:
  hz=15 zeta=10   DIVERGED 2.946s   <- best
  hz=30 zeta=10   DIVERGED 1.929s
  hz=60 zeta=10   DIVERGED 1.579s
  hz=30 zeta= 2   DIVERGED 1.200s
  hz=30 zeta=30   DIVERGED 1.800s
  hz= 8 zeta=10   DIVERGED 1.571s   <- too soft to support; sinks instead
```

Real, ordered variation — softer helps to a point, with an optimum near hz=15
that nearly doubles survival over hz=60, and a floor below which contact is too
compliant to carry the load. **Every previous sweep in this log was flat**,
which is what said each swept parameter was innocent. This one is not flat, so
these parameters are genuinely coupled to the failure for the first time.

Still diverges at ~2.9 s. Not solved, but no longer inert.

All 20 automation tests pass, 0 failures.

### Standing recommendation

Best known configuration: impulse contact, `ContactHertz = 15`,
`DampingRatio = 10`, 8 iterations + 2 relax, 240 Hz, PD/actuation gains at or
below Kp = 5e7. The remaining ~2.9 s ceiling is the open item.

---

## Entry 019 — Driver wired; sub-stepping does NOT help, and that is a lead

### Driver wiring

`AMutoRLTrainingDriver::StepPhysicsSubstepped` now supports both models behind
`bUseImpulseContact` (default **false** — the penalty path is the one exercised
in the editor). The two sit on opposite sides of `Step()`: a penalty force must
be staged BEFORE integration so the bias pass sees it; an impulse corrects the
velocities integration just produced, so it runs AFTER. Resolving inside the
substep loop means contact is solved once per SUBSTEP.

`FImpulseContactCache` is now a driver member, so accumulated impulses survive
between calls — without that, warm starting does nothing.

Exposed: `ContactHertz` (15), `ContactDampingRatio` (10), `ContactIterations`
(8), `ContactRelaxIterations` (2), `ContactSlop` (0.5).

### Sub-steps vs iterations at MATCHED work — Catto's result does NOT reproduce

```
actuated stand, impulse contact, hz=15 zeta=10, work held ~constant:
   120 Hz x 16 iters  DIVERGED 1.417s
   240 Hz x  8 iters  DIVERGED 2.946s   <- optimum
   480 Hz x  4 iters  DIVERGED 1.035s
   960 Hz x  2 iters  DIVERGED 1.052s
  1920 Hz x  1 iters  DIVERGED 0.938s
```

Box2D's benchmark reports sub-steps beating iterations at equal cost. Here the
curve has an interior optimum at 240 Hz and gets **monotonically worse above
it**. Sub-stepping is not the missing half of "soft step" for this solver.

### Why that is diagnostically interesting

"Worse as the timestep shrinks" is the exact signature that led to the
rotation-integration bug in entry 012 — where a `KINDA_SMALL_NUMBER` threshold
silently discarded per-step rotations below 1e-4 rad, so finer steps threw away
more motion. A convergent method cannot degrade with refinement; when it does,
something in the per-step path is scale-sensitive rather than approximate.

That makes **another dt-sensitive defect of the same class the leading
hypothesis**, in preference to any further contact tuning. Places worth auditing
for hard thresholds or absolute epsilons compared against per-step quantities:

- `KINDA_SMALL_NUMBER` guards on `D` in the ABA reductions (`FMath::Max(D, ...)`)
  and on `Inverse3x3`'s determinant — both compare an absolute epsilon against a
  quantity that scales with mass and inertia, and this rig's inertias are ~1e6-1e8.
- `IsNearlyZero` / `GetSafeNormal` calls on per-step deltas in the contact and
  clamp paths.
- `SolveSpatial6`'s pivot threshold, likewise absolute.

The entry-012 fix changed one such threshold and cured a whole class of
symptoms; the same audit has not been done for the rest.

All 20 automation tests pass, 0 failures.

---

## Entry 020 — Threshold audit: hypothesis REFUTED, one latent bug closed

Enumerated every absolute epsilon in `CreatureBatchSolver.h`, `SpatialAlgebra.h`
and `CreatureGroundContact.h`, then MEASURED what each is compared against
during a real simulation (`Private/Tests/MutoThresholdAuditTest.cpp`) rather
than reasoning about it.

### The hypothesis is wrong

```
@ 240 Hz: sampled 16312 contacts | min(1/m_eff)=2.278e-4 -> max m_eff=4390 kg | SKIPPED=0 | T-clamped=0
@ 960 Hz: sampled 32976 contacts | min(1/m_eff)=4.171e-3 -> max m_eff= 240 kg | SKIPPED=0 | T-clamped=0
@1920 Hz: sampled 55680 contacts | min(1/m_eff)=4.171e-3 -> max m_eff= 240 kg | SKIPPED=0 | T-clamped=0

min |Det(Irot)| over all bodies = 4.63e+10   (threshold 1e-4 -> margin 4.63e14 x)
```

**No threshold ever fires.** `Inverse3x3`'s determinant guard and
`SolveSpatial6`'s pivot guard have a 4.6e14x margin — they are compared against
quantities scaling as mass*length^2 (~1e10 here), so they are nowhere near
relevant. `D = FMath::Max(dot(S,U), KINDA_SMALL_NUMBER)` is in the same regime.

Entry 019's leading hypothesis — that another dt-sensitive absolute epsilon
explains the "worse with finer dt" behaviour — is **refuted**. That is a clean
negative result: the remaining contact failure is not an epsilon problem.

### One latent bug found and fixed anyway

`InvMassN <= KINDA_SMALL_NUMBER` silently **drops** a contact, and
`FMath::Max(InvMassT, KINDA_SMALL_NUMBER)` caps the tangential effective mass.
Since InvMass = 1/m_eff, a 1e-4 threshold means any contact whose articulated
effective mass exceeds **10,000 kg** is discarded or clamped. Measured closest
approach on this 6170 kg rig: **4390 kg — a margin of only 2.3x.**

It never fired in these runs, so it is not the active fault, but the contacts
with the highest effective mass are precisely the best-braced ones — the guard
would discard the most load-bearing contacts first, and only once the creature
started actually supporting itself. Changed to `UE_SMALL_NUMBER` (1e-8), which
still guards the division while permitting effective masses to 1e8 kg.

### Incidental observation worth keeping

`max m_eff` differs by rate (4390 kg at 240 Hz vs 240 kg at 960/1920 Hz).
Effective mass depends only on configuration, so this is trajectory divergence,
not a solver difference — but it says something useful: at the higher rates the
creature never reaches a BRACED pose at all. Its contacts stay an order of
magnitude more compliant, which is consistent with the earlier finding that
high substep rates leave the soft constraint too relaxed to build up impulse
(entry 019's `impulseScale` reaches 0.50 at 1920 Hz versus 0.11 at 240 Hz, so
each iteration discards half the accumulated impulse).

That interaction — softness relaxation versus iteration count at high substep
rates — is a better explanation of "worse with finer dt" than any threshold, and
is the next thing to test.

All 21 automation tests pass, 0 failures.

---

## Entry 021 — Penalty contact REMOVED; impulse is the only model

Carrying two contact models was itself a defect: they sit on **opposite sides of
`Step()`** (a penalty force must be staged before integration; an impulse
corrects velocities after it), so the codebase contained two step orderings at
once, and every measurement had to state which one it used. Removed.

### What went

- `FContactParams` and `ApplyGroundContactForces` — deleted from
  `CreatureGroundContact.h`.
- Driver properties `ContactSpringK`, `ContactDamperK`, `ContactFrictionK`,
  `ContactMaxNormalForce`, `ContactEffectiveMass`, and the `bUseImpulseContact`
  toggle. Impulse contact is now unconditional.
- Four penalty-only diagnostics whose questions are answered and recorded:
  `MutoGroundPenetrationDiagnosticTest`, `MutoContactTraceTest`,
  `MutoStabilityCheckpointTest`, `MutoMinimalContactTest`.
- The `REAL GROUND CONTACT` variant of `MutoWrenchPropagationTest`, which read
  back the wrench the penalty model deposited in `ExtForce`/`ExtTorque`. The
  impulse path never writes those, so there is nothing to read back. It had
  already answered its question (entry 013).
- `MutoImpulseContactTest` rewritten: about half of it was head-to-head
  comparison plus a one-off diagnosis of the stale driver constants (entry 017),
  neither of which needs to run again. What remains is the closed-form GATE, the
  PD control, and the softness and substep sweeps.

`FContactPointState` survives — the impulse path reports an equivalent average
force through it, so the RL observation and reward code is untouched.

### Ported, and what porting revealed

`CreatureGroundContactTest` (the permanent test) moved to impulse contact and
immediately FAILED, settling **8.08 units deep** instead of ~1 and staying there.
That is a real ordering bug, not a tolerance problem:

> Box2D's relax phase runs AFTER position integration, so the bias velocity has
> already pushed the body out of penetration before relaxation removes it. Our
> contact resolves after `Step()`, which has ALREADY integrated position — so the
> relax pass strips the bias velocity while it has moved nothing at all. The
> correction is created and destroyed in the same breath and penetration never
> recovers.

Worse, `RelaxIterations` was clamped with `FMath::Max(1, ...)`, so a caller
asking for zero relax passes silently got one anyway.

```
drop test resting penetration:  relax=2  ->  8.08 units (fails)
                                relax=0  ->  0.58 units (passes)
```

`RelaxIterations` now defaults to **0** and the clamp honours it, with the
ordering dependency documented on the field. It should only be raised if contact
is moved to resolve BETWEEN the velocity and position integrations, which is the
ordering it assumes.

This also retires entry 018's claim that the relax pass improved things — that
was measured on an already-diverging run and does not survive this. Re-measured
with relax disabled, the full-rig contact numbers are **unchanged** (2.946 s
optimum at hz=15, 240 Hz x 8 iters), so relaxation was never the deciding factor
there either way. It only ever mattered for resting penetration.

### State

**17 automation tests pass, 0 failures** — 17 rather than 21 because four
penalty-only diagnostics were deleted, not because anything regressed.

The remaining open problem is unchanged: sustained ground contact diverges after
roughly 1.5-2.9 s. Removing the penalty model does not fix that, and was not
expected to — it removes an alternative that had already been measured as no
better, and with it a standing source of ambiguity about which orderings and
constants any given result referred to.

### Still open (carried forward, not addressed by this entry)

- Whether a 20 kg foot transmitting ~3e6 of load at a 106 cm lever arm is
  representable at all by a penalty contact, given a 3282 kg torso on 20 kg
  feet — the authored mass distribution may need the extremities heavier.
- Rung 4 (single limb, passive, one contact point) to isolate contact from a
  34-body collapsing creature.
- A contact test that is not a 4 m ragdoll drop: hold the torso, or apply
  muscle torques, so contact is exercised near equilibrium rather than at
  impact.

### Incidental observations (not acted on, logged so they are not re-derived)

- Revolute joint-limit clamping **does** zero `JointVel` (`CreatureBatchSolver.h:978`),
  so it is not an energy-injection source. Relevant to Suspect 3.
- Inertia is built from `BodyLength` (offset to the body's own child) but the
  contact force is applied at `BodyFusedTipOffset`, a different and longer lever
  (Feet: 106.4 cm vs the ~70 cm implied by its inertia). Modeling inconsistency,
  ratio ~0.44; not a unit error.
- `BElbow3_*` / `FElbow3_*` have `leverArm = 0.00` — their contact point sits
  exactly on the joint origin, so contact there generates zero torque about that
  joint and transmits directly to the parent.
- ~~`MaxTorquePerDOF = 5.0`~~ **RESOLVED in entry 017** (now 5.0e7). Kept for the
  reasoning: it was ~2000x too small to hold a 1 kg limb with a 10 cm
  CoM lever against gravity (`m*g*r = 1*980*10 = 9800`). The creature cannot
  resist gravity with actuation at all; it collapses limply into the floor every
  episode. Same root cause: the constant was chosen on SI intuition, the solver
  runs in cm–kg–s.

## Entry 022 — Every revolute rotated about the WRONG AXIS. User-spotted, measured, fixed.

**Hypothesis.** User observation: `Knee1_R` has only a Yaw muscle authored, so it
is built as a 1-DOF revolute and is structurally incapable of rolling about its
own long axis — yet in the editor it visibly rolls.

The visualizer was ruled out by inspection first, before writing anything:
`MutoRLVisualizer.cpp:233-234` writes `Batch.GetBodyRot(Body,0)` straight into
`SetBoneTransformByName` in world space with no remapping. So the roll on screen
is roll the solver's forward kinematics produced.

That left the axis itself. `CreatureBatchState.h` declared
`BodyJointAxisLocal` as living in the **parent's** frame, and Pass 1 consumes it
as `ParentRot.RotateVector(AxisLocal)`. But `BuildMutoTopology` stored a bare
`(0,-1,0)` for every chain joint — and the muscle that drives that joint is
authored as `<ThisBone>_muscle_Yaw_*` with `BoneName == ThisBone`, so "Yaw"
names an axis of **this bone**, not of its parent. The two frames differ by
exactly `BodyRestRotInParent`.

### Test performed

New instrument, `MutoJointAxisAuditTest.cpp`. Pure kinematics — no dynamics, no
contact, no timestep, so nothing it reports can be blamed on integration.

- **A. Axis audit.** For every 1-DOF body, express the axis the solver actually
  rotates about in that bone's OWN frame (`Rest.UnrotateVector(AxisLocal)`). A
  correct Yaw revolute must read `(0,-1,0)`. The X component is roll leakage, in
  radians of roll per radian of commanded yaw.
- **B. Functional FK check.** Bind pose, command `Knee1_R`'s single DOF to
  +30°, nothing else touched. Resolve the resulting world rotation delta in the
  bone's own frame. Same quantity as A, reached through the real FK path — if
  they disagreed the bug would be in FK rather than in the data.

Both agreed, which localizes the defect to the axis data.

### Result — CONFIRMED, and far wider than the one bone reported

```
Knee1_R  rest rotation 234.98deg
  axis in bone frame:  roll(X)=+0.9929  yaw(Y)=+0.1114  pitch(Z)=-0.0405
  commanded  +30.000 deg of YAW
  delivered  roll +29.788 deg   yaw +3.343 deg   pitch -1.216 deg

ALL 26 revolutes leaked, 2.5% to 99.3%:
  Knee1_L/R  99.3%     MElbow_L/R 60.6%    BElbow2_L/R 58.7%
  FElbow_L/R 58.0%     FElbow2    44.0%    MHand      43.5%
  ...  FHand_L/R 2.5% (the least-affected)
```

`Knee1` is essentially a **pure roll joint**: 99.3% of every commanded yaw came
out as roll. The user's read of the viewport was exactly right.

Why this was never caught: the joints that looked fine looked fine by accident.
A rest rotation that happens to be nearly a pure rotation about Y leaves Y
invariant, so those bones' two frames coincide. `FHand` (2.5%) has a 301° rest
rotation that is very nearly about Y; `Knee1`'s 235° is about something else.

### Fix applied

One line in `MutoTopology.h` — rotate the axis into the frame the solver
actually consumes it in:

```cpp
JointAxisLocal.Add(RestRelToParentBody.GetRotation().RotateVector(FVector(0.0f, -1.0f, 0.0f)));
```

Chosen over changing the solver to post-multiply, because it changes DATA only:
the solver's stated contract ("axis in the parent's frame") stays literally
true, and the fix cannot affect the ball-joint path or the SIMD path.

Corroboration that this is the right frame, not just a frame that passes the
test: **the ball-joint path was already doing this**. It builds
`JointFrame = ParentRot * BodyRestRotInParent` before interpreting its DOFs
(`CreatureBatchSolver.h:387`). The revolute path was the odd one out.

### Verification

```
axis audit, all 26 revolutes:  roll(X)=0.0000  yaw(Y)=-1.0000  pitch(Z)=0.0000
  -> 0 of 26 leak beyond 0.010 (was 26 of 26)
Knee1_R functional FK:  commanded +30 deg yaw
  -> roll +0.000  yaw -30.000  pitch -0.000
```

(The delivered yaw reads −30 against +30 commanded because the axis is −Y by
design — see `MutoTopology.h`'s comment on why. Rotating +30° about −Y *is*
−30° about +Y. Not a sign bug; the test decomposes onto +Y.)

**Full suite: 18 tests pass, 0 failures** (17 + this one). No regressions.

### This does NOT fix contact, and it moved the numbers the WRONG way

Re-running the contact sweeps on the corrected rig:

```
                       before axis fix    after axis fix
  hz=15  240Hz x 8         2.946 s          1.367 s
  120 Hz x 16 iters          --             1.525 s  (new best)
  1920 Hz x 1 iter           --             0.776 s
```

Divergence is **sooner**, not later. Reported as measured, with no attempt to
present a correctness fix as a stability win. The plausible reading is that a
rig bending about wrong axes was collapsing along different — and evidently
slower — paths than one that bends where knees actually bend; but that is a
story about the number, not evidence, and it is not being claimed as a finding.

**The more important consequence: every contact-stability number in entries 013
through 021 was measured on a rig whose 26 revolutes rotated about wrong axes.**
That includes the softness sweep, the iteration sweep, the substep sweep, and
the 2.946 s "best known configuration". Those tunings were fitted to a rig that
did not exist. They are not evidence any more and should be re-run before any of
them is treated as a constraint on the design. The qualitative conclusions that
did NOT depend on the rig's articulation — contact-free stability, momentum
conservation, single-body wrench exactness, SIMD parity — are unaffected.

### Housekeeping

`MutoJointAxisAuditTest` was promoted out of the `TEMP` namespace to
`AgentSolver.Muto.JointAxisAudit` and kept. It guards an invariant that is
invisible to every other test in the directory: a rig with all its revolute axes
wrong still animates, still conserves momentum, still passes everything else. It
took a human watching the viewport to catch this one, which is exactly the
failure mode a permanent test should close.

`FCreatureABASolver::RecomputeKinematics` was made public (visibility only, no
behaviour change) so kinematics can be exercised with no integration and no
joint-limit clamping in the way.

## Entry 023 — Joint limits are not constraints. User-observed, instrument-corrected, CONFIRMED.

**User observation** (watching the passive ragdoll at 0.25x, entry 022's new
actor), with a substep timeline: feet land at ~1, arms at 30-60, **heels
perforate at ~130**, tips follow at ~150, knees under by 180, hips by 200,
divergence at ~450. Their reading:

> "The legs bend perfectly as expected, but when they both reach their maximum
> flex angle on all of their bones they don't have any way of still slowing down
> the fall. And this is right at this time that the heels perforate the ground."

Critically, the **arms stay fine until the whole model has already collapsed** —
and the arms are the limbs that never saturate their limits, because they carry
almost no weight. That splits the failure along a line no previous test had:
contact works on limbs inside their range and fails on limbs at the end of it.

**Hypothesis.** Joint limits are not constraints. `ClampJointLimits` is a
post-integration POSITION clamp that snaps `JointPos` to the boundary and zeroes
`JointVel` (`CreatureBatchSolver.h:1170`); nothing in the impulse path knows a
joint is limited. `ComputeArticulatedInertias` reduces every revolute out as
freely rotating, so a leg folded solid against its stops is still modelled as
compliant, and the contact under it computes its impulse against an articulated
effective mass that assumes the leg can keep folding.

### FIRST ATTEMPT: my own instrument was wrong, and it said the opposite

The first version of the trace reported **0 DOFs at a limit throughout the entire
collapse**, which reads as a clean refutation — and I was one step from reporting
it as one.

It was a bug in the helper, not a result. `RevoluteLimitSide` compared raw
degrees against `DOFRangeMin/MaxDeg` directly. This rig's authored ranges are
wrapped — `Knee1` is `[283.2, 403.6]`, `Knee2` `[330.2, 448.6]`, `Feet`
`[291.3, 405.0]` — so a joint at the bind pose reads 0 deg, which is inside
`[283.2, 403.6]` only once you know 0 is 360. `ClampJointLimits` itself wraps
correctly; my helper did not. It reported "nothing at a limit" for joints that
had wound past **-8000 degrees**.

Recorded rather than quietly fixed: this is the second time an instrument of
mine has produced a confident wrong answer (see entry 011), and the tell was the
same both times — a number that disagreed with a direct observation. The user
watched the joints stop at maximum flex. The instrument said no joint was ever
at a limit. One of them had to be wrong, and it was not the user.

### Corrected result — hypothesis CONFIRMED, and the timeline matches

```
 substep | atLimit | outOfRange | lapSkips | worstPen | effMass | worst contact
       1 |       0 |          0 |        0 |     0.02 |      14 | Feet_L
      60 |       0 |          0 |        0 |     0.82 |      13 | Feet_R
     120 |       0 |          0 |        0 |     1.21 |      70 | FElbow3_L
     130 |       2 |          0 |        0 |     5.78 |      15 | Feet_R   <- limits engage
     150 |       4 |          0 |        0 |    19.60 |      16 | Feet_R
     152 |       4 |          0 |        2 |    19.46 |      17 | Feet_R   <- first TUNNEL
     153 |       6 |          0 |        2 |    19.54 |      17 | Feet_L
     170 |       6 |          0 |        2 |    73.65 |      18 | Feet_L
     186 |       8 |          0 |        4 |   106.07 |      30 | Feet_L
```

- **6+ revolutes at a limit from substep 153**, first two at **130** — the exact
  substep the user reported the heels perforating. The observation was right.
- **Articulated effective mass at the foot: 14 kg**, under a **6170 kg**
  creature.
- **Effective mass barely responds to saturation: 14 kg free -> 17 kg with six
  joints locked (1.2x).** This is the hypothesis, measured. A leg folded onto its
  stops is physically a rigid strut that should present a large fraction of the
  creature's mass at the foot; the solver sees 17 kg.
- **Limits tunnel: 1522 lap skips, first at substep 152.** `ClampJointLimits`
  keeps a clamped joint in its current lap, so a changed lap index means the
  joint crossed its entire forbidden arc within one substep and the clamp never
  saw it out of range. Knee1's forbidden arc is 360-120.4 = 239.6 deg, which at
  240 Hz needs ~57,500 deg/s — reachable once the collapse is underway.
- **The clamp fails outright (state left out of range) at substep 371**, shortly
  before divergence at 374.

Note the 14 kg is NOT itself a bug. For an instantaneous impulse at the foot of a
freely articulated leg, a small effective mass is correct — you can flick a foot
without moving the torso. The defect is that the model has no way to stop being
that leg. Contact cancels the foot's approach velocity, gravity re-creates it by
folding the leg one more notch, and the fold never runs out because the limit
that should end it does not exist as far as the dynamics are concerned.

### Sequence, as measured

1. Substeps 1-120: feet resting, penetration creeping 0.02 -> 1.21 cm. Contact
   is cancelling velocity but never supporting weight; the leg absorbs the
   descent by folding. This looks correct on screen and is the "bend perfectly
   as expected" phase.
2. ~130: leg joints reach their stops. Physically the leg becomes a strut; in
   the solver nothing changes, effective mass stays ~15 kg. Penetration jumps
   1.21 -> 5.78 cm in ten substeps.
3. 152 onward: joints start tunnelling through their limits entirely.
4. 153-370: runaway. Feet dragged under, chain follows, clamp finally fails
   outright, non-finite at 374.

### Status

**CONFIRMED, no fix applied.** The fix is architectural — joint limits have to
become constraint rows solved in the same system as contact, which is what
MuJoCo, PhysX articulations and Box2D v3 all do — and that is a large enough
change to be the user's call, not something to slip in under a diagnostic.

Three defects are in scope for it, all the same root cause:
1. Articulated effective mass ignores limited joints (measured: 14 -> 17 kg
   where it should be thousands).
2. Limits tunnel at high joint speed (measured: 1522 lap skips).
3. The clamp zeroes `JointVel`, discarding that momentum instead of
   transmitting it up the chain.

Also worth noting for whoever does it: entry 022 established that every contact
tuning number from entries 013-021 was measured on a rig with wrong joint axes.
Those tunings should not be treated as constraints on the design of this fix.

## Entry 024 — Joint limits as constraint rows: IMPLEMENTED. Improves, does not fix.

Entry 023's fix, built. Joint limits are now constraint rows solved in the same
sequential-impulse loop as contact, interleaved per iteration.

### What was added

- `SolveImpulseResponse` takes an optional per-DOF JOINT-space impulse, entering
  exactly where the force pass's tau does (`Uu = tau - S.p`). `ImpulseBody ==
  INDEX_NONE` means a pure joint impulse with no body wrench.
- `JointImpulseResponse(Batch, Env, DOF)` -> d(qd)/d(joint impulse), whole-tree.
  The joint-space analogue of `ImpulseResponseAtPoint`, and the diagonal a limit
  row divides by.
- `ApplyJointImpulse(Batch, Env, DOF, J)`.
- `FJointLimitParams` + limit rows in `ResolveGroundContactImpulses`, warm
  started and accumulated-clamped exactly like contact rows, solved in the SAME
  iteration so contact and limits see each other's effect.
- Rows use a SPECULATIVE bias when the joint has not yet reached the stop
  (`bias = separation/dt`), which contact rows do not. That is the anti-tunnel
  mechanism: the row engages on approach and bounds the approach to what closes
  the remaining gap in one step.
- Driver switch `bSolveJointLimitsAsConstraints` (default on) so the two
  behaviours can be compared rather than argued about.

`ClampJointLimits` is deliberately left in as a position-level backstop.

### Result — real improvement, mid-fall

```
                              position clamp    constraint rows
  penetration @ substep 150            19.60              7.25
  penetration @ substep 250           462.70             17.77
  total lap skips                       1522               571
  first tunnelled limit                  152               162
  diverged at substep                    374               383
```

Penetration at substep 250 improves **26x**. This is the phase the user
described as the feet being dragged under, and it is substantially fixed.

### And a claim I have to retract, from my own run

The iteration sweep first ran to 400 substeps and reported:

```
  iters | maxPen | diverged
     32 |    321 | no
     64 |    179 | no
    128 |    193 | no
```

I was one step from reporting "32 iterations fixes it". 400 substeps is 1.67 s —
barely past where the old configuration already failed, so "survived the trace"
was a statement about the trace, not about the solver. Re-run at 1440 substeps
(6 s):

```
  iters | pen@150 | pen@250 |  maxPen | diverged
      8 |    7.25 |   17.77 |   71871 | 383
     16 |    7.32 |   13.50 |    7324 | 383
     32 |    7.54 |   27.61 | 5474414 | 535
     64 |    7.52 |   39.28 |   70351 | 487
    128 |    7.52 |   39.42 |    1288 | 495
```

Everything still diverges. More iterations buys ~100-150 substeps and is not
monotonic. **Convergence rate is not the missing ingredient**, which also means
the "just add iterations / block-solve the chain" family of fixes is not the
answer. Third time an instrument window has flattered a result here (see 011,
023); the pattern is always a bound chosen before knowing what the answer looked
like.

### Why the foot still reports ~17 kg, measured

A sequential-impulse row's effective mass is `n.(J M^-1 J^T).n` for the
UNCONSTRAINED system, by construction — adding limit rows cannot change it. What
was supposed to produce rigid-strut behaviour was convergence of the coupled
iteration, and the sweep above shows that does not get there.

So the structural alternative was measured directly: weld every saturated joint
inside the factorization itself (`ComputeArticulatedInertias(Batch, LockedBody)`
plus the matching branches in `SolveImpulseResponse`, which is the part that
matters — locking only the factorization was a silent no-op that reported a
clean 1.0x and nearly got written up as "welding changes nothing").

```
  substep | locked | effMass free | effMass welded | ratio
      140 |      4 |           63 |             63 |  1.0x
      160 |     10 |           18 |            269 | 14.8x
      180 |      8 |           24 |            168 |  7.1x
      200 |     14 |           40 |            121 |  3.0x
      220 |     14 |           73 |             89 |  1.2x
```

**14.8x at the moment it matters** (substep 160, ten joints saturated, right
where the heels are being dragged under). Real, and far short of the 1000-3000 kg
a rigid strut under this creature should present.

The gap is diagnosable: **only revolutes are being locked.** Each leg is
`Hips (3-DOF ball) -> Knee1 -> Knee2 -> Feet`, and the hip ball joint has no
limit row and no lock, because its limit is a cone on the rotation vector's
magnitude rather than a per-DOF range. Welding the three revolutes below a joint
that is still free leaves the leg able to fold at the top, so the chain never
becomes a strut no matter how rigid its lower half is. 269 kg is what you get
with three of four joints locked.

### Recommended next step

Ball-joint cone limits as constraint rows, with a lock in the factorization —
same treatment the revolutes just got. That is the one change the measurement
above actually points at, and it is testable against the same 14.8x number: if
the hips lock too, the foot should report something in the 1000s.

Everything else on the shortlist is now less well supported:
- more iterations: refuted above,
- better sweep ordering / block Gauss-Seidel: same family as iterations,
- full LCP over all constraints (MuJoCo's actual approach): correct, but a
  rewrite, and there is no evidence yet that the cheaper fix is insufficient.

**Suite: 19 tests pass, 0 failures.**

## Entry 025 — Ground collision on every body. THE PASSIVE RIG IS STABLE.

**User observation**, after entry 024 stopped the legs being first through the
floor: the PELVIS became first instead. It has no contact point at all, so
nothing objected while it went under and dragged the whole rig behind it.

That is not a solver bug. The solver cannot brace against geometry it was never
told about. The authored `CanTouchGround` flags cover feet, hands and elbows --
10 points -- and nothing else. Body 0 carries roughly 3282 kg of the creature's
6170 and had no collision whatsoever.

### Change

`BuildMutoContactPoints` gains `bAllBodiesCollideWithGround` (driver default on)
plus a `StructuralRadiusFallback`. Every body without an authored point gets one
sphere at its own joint origin, using the authored per-bone radius where the
mass asset has one. 10 points -> 35.

**GROUND ONLY, and structurally so.** `ResolveGroundContactImpulses` tests
against the plane at `GroundZ` and nothing else, so no number of points can
produce limb-vs-limb collision -- that would need a broadphase and pairwise
tests which do not exist in this codebase. Limbs still pass through each other.

Bodies that already have an authored point are skipped rather than doubled up:
their geometry was hand-tuned, and stacking a second row at nearly the same
place is how a resting contact starts to jitter.

### Result

```
                          authored only    every body
  [8 iters] pen@250             17.77          1.57
  [8 iters] maxPen              71871            21
  [8 iters] torsoZ end   1.3e12              63.2
  [8 iters] diverged              383         never

  [32 iters] maxPen           5474414            33
  [32 iters] diverged             535         never
```

50-second confirmation run (12000 substeps at 240 Hz), because a 400-substep
window already flattered one result in entry 024 and "survived the trace" is a
claim about the trace until the trace is boring:

```
  t=10.0 s   torsoZ=63.23   maxPen 20.69   maxSpeed 948.3
  t=20.0 s   torsoZ=63.23   maxPen 20.69   maxSpeed 948.3
  t=30.0 s   torsoZ=63.23   maxPen 20.69   maxSpeed 948.3
  t=40.0 s   torsoZ=63.22   maxPen 20.69   maxSpeed 948.3
  t=50.0 s   torsoZ=63.22   maxPen 20.69   maxSpeed 948.3
  -> survived, finite throughout over 50 s
```

**The passive rig lands, settles, and stays settled.** Torso drifts 0.01 cm over
the last 40 seconds. Peak penetration and peak speed are both reached during the
initial impact and never approached again. This is the first time anything in
this project has survived sustained ground contact -- every configuration in
entries 013-024 went non-finite between 1.4 s and 2.9 s.

### What this does NOT mean

- **It is stable, not correct.** 20.69 cm of peak penetration at impact is a lot;
  the rig visibly sinks before contact catches it. That is a quality problem now,
  not a stability one.
- **It settles as a collapsed heap, not standing.** There is no actuation in a
  passive drop. "Can it stand up" is an untested and separate question.
- **Self-collision still does not exist.** Limbs pass through each other and
  through the torso. The user scoped this deliberately ("only with the ground not
  with the other limbs for now").
- **Ball-joint limits are still not constraint rows** (entry 024's recommended
  next step). The hips remain on the old position clamp, and the welded-inertia
  measurement that motivated that work still stands at 14.8x rather than the
  1000-3000 kg a fully locked strut would give.
- **Cost went up.** 35 points instead of 10, each costing three
  `ImpulseResponseAtPoint` tree passes at gather time plus one
  `ApplyImpulseAtPoint` per iteration. Not measured; it will matter for training
  throughput with many envs.

### Contact tuning is now re-measurable

Every contact number in entries 013-021 was taken on a rig with wrong joint axes
(entry 022) and no torso collision (this entry). Those tunings were fitted to a
creature that could not be held up by construction. The softness, iteration and
substep sweeps are all worth re-running now that there is a stable baseline to
compare against -- for the first time, "better" can mean something other than
"diverged later".

**Suite: 19 tests pass, 0 failures.**

## Entry 026 — Armature, damping, welding and a global constraint solve. THE 41-BODY RIG IS STABLE.

Acts on `OPEN_ITEMS.md` (new companion register, written the same day — items are
cited by its IDs below). Four things landed: the capsule end-cap fix, the dormant
regularizers turned on, MuJoCo's `armature` and `damping` adopted, and the
assembled-constraint solve entry 024 named and deferred.

### First: entry 025's stability result does not apply to the current rig

Entry 025's "survives 50 s, torso drifts 0.01 cm" was measured on the **35-body**
rig, before the 2026-08-16 spine articulation took it to 41 bodies / 68 DOF.
Re-running that exact configuration on the CURRENT rig:

```
BASELINE (entry 025's config, 41-body rig)   DIVERGED at substep 246 (1.03 s)
```

So the rig was already broken and the log did not say so — the log stops at 025
while four undocumented changes landed after it (`OPEN_ITEMS.md` N-01). Every
number below is on the current rig.

### What was added

- **Capsule end caps** (R-01). `GetCapsuleLocalEnds` is now the single derivation,
  shared by limb collision, the ground gather, the reward's centre of pressure and
  the driver's standing height. The ground gather previously computed the second
  cap as `LocalOffset - LocalOffset.GetSafeNormal()*2h`, which for an interior
  body's ZERO offset returns the zero vector — yielding two coincident contact
  rows at one point while still reporting two distinct ends. `FElbow3_*` and
  `BElbow3_*` are flagged CanTouchGround and were measured at `leverArm = 0.00` in
  entry 021, i.e. exactly this case, on four bodies, every substep. The helper
  returns 1 when the ends coincide.
- **Centre of pressure** (R-02). `ComputeReward` re-derived contact positions
  without the radius offset or capsule handling, so for the 25 structural points
  (of 35) it placed the CoP at the body's joint origin. Now uses the same
  derivation the solver used to produce the force it weights by.
- **Cfm, SOR relaxation, MaxBiasVelocity, MaxNormalImpulse** (U-03, U-04, U-05):
  had no caller and no editor property, so the two mechanisms written specifically
  for the coupled ground-row-vs-joint-limit-row instability had never once run.
  Exposed, and defaulted non-zero where appropriate.
- **Saturated-joint welding** (U-02): entry 024 measured 14.8x and then only ever
  called it from a diagnostic. `BuildSaturatedJointLocks` is the production
  caller; the lock array is now per-body-per-env (it was per-body, which would
  have welded env 0's saturated joints in all 256). Only joints at a stop AND
  still being driven into it are welded, which confines the one-sided
  approximation to the case it is right for.
- **Armature** (MuJoCo's `armature`), as a dimensionless RATIO: `D *= (1+r)`. Not
  absolute, deliberately — every published MuJoCo value assumes SI metres and this
  solver runs in cm, which is the exact boundary that produced entries 001 and
  017. The ball-joint reduction needed the general `I^a = I^A - U D^-1 U^T` form;
  written via the identities `Irot*D^-1*A = Irot*r/(1+r)` and
  `A*D^-1*H = H*r/(1+r)` it has no cancellation and vanishes identically at r = 0.
- **Joint damping** (MuJoCo's `damping`), as a TIME CONSTANT — see the retraction
  below for why it ended up an impulse and not a torque.
- **Global constraint solve**: `A = J M^-1 J^T + R` assembled over every active row
  (ground, friction, revolute limits, ball cone limits, limb pairs), solved with
  projected SOR in constraint space. One response column per row via
  `SolveImpulseResponse`, which is linear in its applied wrench. Batch is not
  touched until the end; the residual is maintained as `Cdot = Cdot0 + A*lambda`.

### A claim I had to retract mid-implementation: damping cannot be a torque

Damping went in first as MuJoCo does it — a force `-d*qd` folded into tau, with
`d = D/T` so the parameter stayed unit-free, and with the claim that an isolated
joint therefore loses exactly `dt/T` of its velocity per step.

**The claim was false and a test caught it.** In an articulated tree the joint
acceleration is `(u - U^T a_parent)/D`, and `a_parent` responds to the same
torque: the parent recoils, amplifying the relative acceleration by a factor that
depends on the whole subtree and is unbounded as the parent gets lighter.
Measured on a 2-body chain:

```
requested 10% velocity reduction (T = 10 dt)  ->  removed 197%   (joint REVERSED)
requested full removal           (T =    dt)  ->  overshot 19x
```

The stability cap keyed on `D` was worse than useless: `1/D` is a LOWER bound on
the response gain, so capping with it is anti-conservative exactly when it
matters. No cheap analytic upper bound exists — it needs the parent's articulated
inertia, which Pass 2 has not finished accumulating when the child's torque is
formed.

**Fix: measure the gain instead of assuming it.** `SolveImpulseResponse` already
returns the exact whole-tree response to a joint-space impulse, so
`ApplyJointDamping` queries `g = d(qd)/d(impulse)` and applies `Delta/g`. That is
exact, unconditionally stable (the fraction is clamped at 1, so damping can remove
all of a joint's velocity and never reverse it — the property MuJoCo's
`implicitfast` integrator exists to provide), and momentum-conserving, because a
joint impulse is internal and its reaction propagates to the parent. Verified:

```
T = 10 dt  keeps 0.90000 of qd       T = 4 dt    keeps 0.75000
T =    dt  keeps 0.00000             T = dt/100  keeps 0.00000  (clamped, no reversal)
ball joint T = 5 dt  keeps 0.80002 of |w|, spin direction preserved to 1-1e-3
angular-momentum drift from one damping application:  7.8e-08
```

Cost: one tree pass per damped joint per substep (~40 on this rig, against ~200
for contact). The cheap alternative — scaling `JointVel` in place — was rejected
because it does not transmit the reaction and so violates conservation of angular
momentum on every substep.

### A real bug found while doing the armature work

`SolveImpulseResponse`'s ball-joint branch hardcoded the reduced bias's ANGULAR
part to `ZeroVector`. Featherstone gives `p^a.Ang = ImpP.Ang + Irot*D^-1*u`, which
with no armature collapses to `JointImp3` — zero only when the joint impulse is
zero, i.e. only for the body-impulse-only case the function originally supported.
When ball-joint cone limit rows started passing a real joint impulse through that
branch (2026-08-16), the angular reaction a hip or spine limit should transmit to
its parent was **silently discarded**, so those limits braced against their own
child subtree only. That defeats the stated purpose of the joint-space query
("whole-tree, not local ... that is the entire reason a joint limit solved this way
can brace a chain"). The revolute branch was always correct.

Caught by asserting the one law it breaks: an impulse internal to a floating-base
tree cannot change total angular momentum. After the fix,
`|L| 9314.8 -> 9314.8`, relative change **1.44e-08**.

### Results — one change at a time

Passive drop, zero torque, real 41-body rig, 10 s at 240 Hz. `ms/substep` is
single-env on a Development editor build, so it is a relative comparison between
configurations, not a throughput figure.

```
  config       | maxPen    | pen@end | torsoZ end | maxJointSpeed | ms/substep | diverged
  BASELINE     |    245.00 |    0.00 |       0.00 |        4.7e28 |      1.964 | 1.03 s
  row+passive  |   2857.08 |    0.00 |       0.00 |        8.1e18 |      2.661 | 2.54 s
  row+weld     |    1.8e12 |  1.8e12 |       0.00 |        1.0e24 |      1.884 | 1.01 s
  row+limbcol  |    463.05 |    0.00 |       0.00 |        3.2e24 |      2.755 | 1.25 s
  global only  | 217787.23 |217787.2 |       0.00 |        2.9e13 |      1.124 | 1.04 s
  glob+passive |    578.37 |    0.00 |       0.00 |        6.4e13 |      1.594 | 2.68 s
  glob+pas+wel |     55.31 |    5.25 |      64.35 |          45.5 |      1.745 | never
  glob+pas+lim |    4.25e6 |  4.25e6 |       0.00 |        2.2e13 |      2.040 | 2.35 s
  ALL row      |     24.83 |    4.41 |      74.52 |          57.1 |      3.391 | never
  ALL          |     15.34 |    2.60 |     108.68 |          59.1 |      2.161 | never
```

Read carefully, because the headline is not the one I expected:

- **No single addition fixes it.** Baseline fails at 1.03 s; armature+damping alone
  reaches 2.54-2.68 s, welding alone is WORSE (1.01 s), limb collision alone
  reaches 1.25 s, the global solve alone reaches 1.04 s.
- **Survival needs armature+damping AND welding together.** Welding alone
  destabilizes, which makes sense: it raises the effective mass sharply and
  per-row Gauss-Seidel then over-corrects against a much stiffer system. Armature
  softens `D` and damping dissipates; the combination is what holds.
- **The global solve is NOT what fixed the divergence.** `ALL row` — the per-row
  solve with every other addition — also survives. Crediting the mass matrix for
  this would be wrong.
- **What the global solve buys is quality and cost.** Against `ALL row`: peak
  penetration 24.83 -> 15.34 (1.6x better), the torso is held at 108.68 instead of
  74.52 (47% higher, i.e. far less collapsed), and it is **1.57x CHEAPER** (2.161
  vs 3.391 ms/substep) while running 64 sweeps against the per-row path's 16. That
  is the predicted trade, measured: a dense sweep costs no tree traversals.
- Limb collision remains the fragile one: `glob+pas+lim` (2.35 s) is worse than
  `glob+passive` (2.68 s), and it is only safe once welding is present.

### 50-second confirmation, shipped defaults

Because a window chosen before knowing what the answer looks like has flattered a
result here at least three times (entries 011, 023, 024):

```
  t= 10.0 s  torsoZ=  108.68  maxPen=  15.34  maxSpeed= 3247.5
  t= 20.0 s  torsoZ=  108.10  maxPen=  15.34  maxSpeed= 3247.5
  t= 30.0 s  torsoZ=  108.20  maxPen=  15.34  maxSpeed= 3247.5
  t= 40.0 s  torsoZ=  108.21  maxPen=  15.34  maxSpeed= 3247.5
  t= 50.0 s  torsoZ=  108.48  maxPen=  15.34  maxSpeed= 3247.5
```

Torso drifts 0.2 cm over the last 40 s. Peak penetration and peak speed are both
reached during the initial impact and never approached again.

### What this does NOT mean

- **Stable, not correct.** 15.34 cm of peak penetration and 3247 cm/s of peak body
  speed are both impact transients, and both are large.
- **Still a collapsed heap, not standing.** Passive drop, zero actuation. "Can it
  stand" (`OPEN_ITEMS.md` O-08) remains untested on this rig.
- **The contact tunings are still fitted to a rig that did not exist** (O-07).
  `ContactHertz = 15` comes from entry 018's sweep, which entries 022 and 025 both
  invalidated. Now that there is a stable baseline again, and a residual trace to
  read, those sweeps are finally worth re-running.
- **The global solve is projected SOR, not Newton.** Assembling `A` was the
  structural change; swapping the sweep for Newton on the convex dual is now a
  self-contained change to one loop.
- **Contact is still serial across envs** (R-04). The per-env block structure makes
  it parallelizable, but `SolveImpulseResponse`'s single-env mutable scratch still
  prevents it.

### Instrumentation

`FIterationDebugLog` (U-01) had no caller at all; it now records the per-iteration
constraint residual, which only the assembled system can provide. Driver
properties `WatchContactBody`, `WatchJointLimitDOF`,
`LogSolverResidualEverySubsteps`. Measured on a 4-coplanar-point body (a
rank-deficient system, so it is allowed to converge slowly): residual
217.23 -> 37.97, an 82.5% reduction. On a full-rank single-point system:
217.23 -> 12.54 in one sweep, then stationary. That floor is nonzero **by
construction** — a soft constraint's fixed point is
`(Cdot+Bias) = -ImpulseScale*lambda/(MassScale*InvDiag)`, and that residual IS the
compliance, the same regularization MuJoCo's R > 0 provides.

### Suite

**24 tests pass, 0 failures**, up from 22. New: `AgentSolver.SolverUpgrade`
(permanent, ten parts covering the capsule ends, the armature identities on both
joint types, momentum conservation, the exact damping fractions, welding, the lock
builder's one-sided rule, and global-vs-per-row agreement) and
`AgentSolver.TEMP.RigUpgradeCheck` (the table above — delete once these numbers
are considered recorded, the way entry 021 deleted four answered diagnostics).

---

## Entry 027 — Dead-code removal, and two write-only fields that turned out not to be cosmetic

**Date:** 2026-08-21
**Trigger:** direct request against the `OPEN_ITEMS.md` register — delete `U-08` and
`D-02`, invoke the never-invoked domain randomization (`U-07`), and start reading the two
authored fields nothing read (`U-10`, `U-11`).
**Status:** all five done. Build clean, 22/22 automation tests pass (24 before; the two
deleted are `TEMP.OffsetWrench` and `TEMP.WrenchPropagation`, whose subject no longer
exists). Real-rig behaviour unchanged in the passive drop — `TEMP.RigUpgradeCheck`
reproduces entry 026's table digit-for-digit, `ALL` still 15.34 peak penetration, torso
108.68, 50 s with no divergence.

### The headline: `U-10` is not a cleanup, it is a physics change

`FMassMuscleDataMuscle::ExtensionStrength` / `FlexionStrength` were described in the
register as "editable, mirrored, saved — and ignored". Wiring them through was expected to
be a no-op on the shipped rig, on the assumption that nobody had moved them off 1.0.

That assumption was wrong, and the measurement is now a permanent assertion in
`AgentSolver.MutoTopology`:

    Authored muscle strengths: 67/68 curve DOFs, 44 differ from 1.0, range [0.500, 5.000]

**44 of 67 curve-bearing DOFs carry a non-unit authored strength, spanning a 10x spread.**
So this change makes the strongest joints deliver 5x the torque they delivered yesterday
and the weakest half of it. Consequences that follow directly:

- `FEnvConfig::MaxTorquePerDOF` is **no longer a ceiling on delivered torque**. It clamps
  the COMMANDED torque; the multiplier that used to be bounded by the curve (which peaks
  at 1) is now curve x scalar and reaches 5. The comment on that field says so now.
- Any policy weights trained before today were trained against a uniformly-strong
  creature and are not comparable to weights trained after. This is exactly the
  topology/observation-provenance gap `X-06` describes, arriving in a new form: nothing
  records which muscle calibration a snapshot was trained under.
- Entry 017's torque rescale reasoning ("5e7 gives ~1.5x margin over the worst holding
  torque") was derived with the multiplier capped at 1. On the 44 affected DOFs that
  margin is now anywhere from 0.75x to 7.5x. **This has not been re-swept.**

Not changed here, deliberately: no clamp was added on the multiplier. Clamping it would
restore the ceiling but would also flatten the authored variation back out, which is the
thing the request was about. The clamp is a one-line change in
`ComputeMuscleMultipliers` if the 5x turns out to destabilise training.

### `U-08` — external wrench removed, and what it was actually holding up

Removed `ApplyForceAtPoint`, `ClearExternalForces`, the six `ExtForce*`/`ExtTorque*`
per-body-per-env arrays, the `HitWrench` load in both step variants, and the driver's
per-substep clear loop (256 envs x 4 substeps x 41 bodies of zeroing zeros).

The register called this "written only by tests". That was accurate but incomplete: one of
those tests, `AgentSolver.PendulumEnergyConservation`, was using it **structurally**, and
no substitute existed. The anchor trick needs a per-body force, because:

> A huge mass does not resist gravity. Gravity accelerates every mass equally regardless
> of size, so a 1e5 kg anchor free-falls at exactly -980 like the rod hanging off it, the
> assembly develops no internal stress, and the pendulum does not swing at all.

Pinning the anchor's pose after each `Step()` does not fix this either — the joint
acceleration was already computed inside a free-falling (i.e. weightless) system, so the
pendulum is dead before the pin runs. Verified by reasoning before implementing, not
after a failed attempt.

Replaced with `FCreatureTopology::BodyGravityScale` — a per-body multiplier on the gravity
wrench, MuJoCo's `gravcomp` in ratio form, defaulting to 1. It is per-BODY topology data,
not per-env state, so in the SIMD path it broadcasts once per body alongside `BaseMass8`
and costs zero additional memory traffic in the inner loop. Six per-body-per-env float
arrays became one per-body float. The pendulum's anchor now sets it to 0 and the test
passes unchanged (energy spread 2.45% of scale, anchor displacement 1.7 cm over 8.3 s).

`TEMP.OffsetWrench` and `TEMP.WrenchPropagation` were deleted outright — both existed to
diagnose the wrench path (entries 010/011/012), and both questions are recorded as
answered.

### `D-02` — joint speed limiting removed

`ClampJointSpeed` and `MaxJointSpeedDegPerSec` are gone. It was off by default, and
`R-05` recorded that its ordering had never been validated: it ran AFTER
`ResolveGroundContactImpulses`, clipping the very joint velocities the limit and contact
rows had just solved for, with no re-solve. Deleting it removes that hazard rather than
leaving a knob that silently corrupts the constraint solution when anyone turns it on.

### `U-07` — domain randomization now actually runs

`CreatureRLEnvironment::FDomainRandomization` (limb strength range, limb-loss chance,
carried mass), drawn by `ResetEnv` on every episode reset, exposed on the driver under
`Muto RL|Reset|Domain Randomization`. **Off by default** — enabling it changes what the
policy is being asked to solve, so it is an explicit decision.

Two details that are easy to get wrong and are handled:

1. `RandomizeEnv` is called even when randomization is DISABLED, with neutral arguments.
   The three arrays are episode-persistent state, not per-step inputs, so an env that drew
   a weak or missing limb would otherwise keep it forever after the feature was switched
   back off.
2. The limb-loss test was `FRand() > LimbLossChance`, and `FRand()` is half-open `[0,1)`.
   A draw of exactly 0.0 with chance 0 disabled a limb. Now `LimbLossChance > 0 &&
   FRand() < LimbLossChance`, which short-circuits at zero.

### `U-11` — `BoneIndex` is now the lookup key

`FMassMuscleDataMass::BoneIndex` was write-only: `InitializeFromSkeletalMesh` stamped it
and nothing read it, so a mesh re-import that added, removed or reordered bones left every
stored index silently wrong with no symptom.

- `UMassMuscleProfileAssetMass::PostLoad` now calls a new `SyncBoneIndices()`, which
  repairs every index from its name against the current skeleton.
- `FindBoneByIndex` added; `FindBoneByName` kept as the fallback for callers with no
  reference skeleton (the extracted sub-chains `TEMP.IsolatedLimb` builds).
- `FCreatureTopology::BodyBoneIndex` added and filled by `BuildMutoTopology`, whose
  mass/radius/capsule lookups are now index-keyed. Every call site already had the index
  in hand — it had resolved the name to walk the skeleton — so this costs nothing.
- `BuildMutoContactPoints` prefers the index, falling back to the name.

The point is not the FName-vs-int32 comparison. It is that **a field that is read is a
field that gets maintained**, and `AgentSolver.MutoTopology` now asserts that the index
names the same bone the debug name does. An off-by-one there would have given every body
its neighbour's authored mass, radius and capsule — a plausible-looking rig rather than an
obviously broken one.

### What this does NOT mean

- Nothing here was measured on a DRIVEN rig. `TEMP.RigUpgradeCheck` is a passive drop at
  zero torque, so it cannot see `U-10` at all — the multiplier scales `JointTorque`, which
  is zero throughout. The 5x is unexercised by any test that currently runs.
- Domain randomization is implemented and reachable, not validated. Nobody has yet
  trained with it on, and the limb-loss knob in particular is harsh for a creature that
  has not learned to stand on all its limbs (`O-08`, still never tested).
- `BodyGravityScale` is exercised only by the pendulum anchor (scale 0) and defaults
  everywhere else. The intermediate values are untested.

---

## Entry 028 — Inertia derived from the authored collision radius. The magic 0.05 is gone.

**Date:** 2026-08-21
**Trigger:** direct request — reuse the capsule radius the collision model already reads,
instead of the 5% fudge, for inertia.
**Status:** done. Build clean, 22/22 pass. The rig is **more robust**, not just different:
every configuration that survived still survives, and every marginal one survives longer.

### What the 0.05 was actually claiming

Every body's inertia was a thin uniform rod about its own bone length:

    IPerp  = (1/12) * m * L^2
    IAxial = 0.05 * IPerp          <- "~0 (not exactly, for numerical safety)"

A rod has no thickness, so its axial term is identically zero, and 0.05 was a placeholder
for a shape nobody had measured. But a placeholder still *asserts* something. Solving it
against a real solid reveals what:

    0.5 * m * r^2 == 0.05 * (1/12) * m * L^2   =>   r = L / sqrt(120) = 0.0913 * L

**The old model claimed every bone on the creature is exactly 9.1% as thick as it is
long.** The authored collision radii — which the ground and limb-collision paths have been
reading all along — disagree, in both directions:

| bone | authored r | L | r/L | r the fudge implied |
|---|---|---|---|---|
| Back3 | 82.91 | 97.52 | **0.850** | 8.90 |
| Back1 | 61.92 | 90.44 | **0.685** | 8.26 |
| Hips_L | 49.33 | 228.78 | 0.216 | 20.88 |
| FShoulder_L | 40.38 | 242.91 | 0.166 | 22.17 |
| FHand_L | 8.85 | 79.73 | 0.111 | 7.28 |
| Feet_L | 9.39 | 155.13 | **0.061** | 14.16 |

So it was not a conservative approximation. It was a different creature: a spine 9x too
thin and feet 1.5x too fat. Measured across the rig, **axial inertia moves by 77.5x on
`Back3` and 0.35x on `Feet_L`** (reported by a new permanent line in
`AgentSolver.MutoTopology`).

The deeper problem was not the number, it was that **collision read real geometry while
the mass model silently assumed a different one**. One authored value now decides how
thick a bone is for both.

### The model

`MutoTopology::CapsuleInertiaDiagLocal(mass, radius, length)` — a solid capsule of uniform
density: a cylinder of radius R and length L capped by a hemisphere at each end, long axis
local +X, mass split between cylinder and caps by volume.

    IAxial = 0.5*m_cyl*R^2 + 0.4*m_cap*R^2
    IPerp  = m_cyl*(L^2/12 + R^2/4) + m_cap*(0.4*R^2 + L^2/4 + 0.375*L*R)

Two conventions worth stating because both could reasonably have gone the other way:

- **L is the BONE length, not the collision capsule's length.**
  `BodyCapsuleHalfHeight` defaults to 0, which collapses the *collision* capsule to a
  sphere at the tip — but the bone's *mass* still spans the whole segment. Using the
  collision length would have given most bones a point-like mass distribution, which is
  worse than the rod it replaced.
- **The cylinder-length + two-caps convention matches `MutoMassAuthoringDumpTest`'s volume
  formula exactly** (`pi*r^2*L + (4/3)*pi*r^3`). The shape this derives inertia from is now
  the same shape that test derives its *suggested mass* from — previously those two
  disagreed about the creature's geometry while sitting in the same module.

### A wrong claim, caught by asserting it

The first fallback for a bone with no authored radius substituted `r = L/sqrt(120)`, on
the equivalence above, and the comment claimed it reproduced the old axial term "EXACTLY"
and the transverse one "to within 2.5%".

**Both numbers were wrong**, and the test asserting the comment failed:

| | old rod | capsule at L/sqrt(120) | error |
|---|---|---|---|
| axial | 20.00 | 19.56 | 2.2% |
| transverse | 400.0 | 515.3 | **29%** |

The equivalence solves correctly for a *cylinder*. A capsule splits mass with its caps and
then puts them **outside** L, where they contribute a large parallel-axis term. The
fallback now returns the old rod directly — exact by construction rather than by an
equivalence argument — and `BuildMutoTopology` **warns** when it fires, instead of
silently substituting a made-up thickness. Silent substitution is how the 0.05 lived
unexamined as long as it did.

Only `Pelvis` reaches that path today: it has no ground-contact geometry, so nobody ever
needed to author it a radius. Authoring one picks it up with no code change.

### Effect on the real rig (passive drop, shipped defaults)

| metric | before | after | |
|---|---|---|---|
| peak penetration | 15.34 | 18.89 | +23% |
| resting penetration | 2.60 | **1.90** | −27% |
| torso settle height | 108.68 | 85.43 | −21% |
| max joint speed | 59.1 | **28.7** | −51% |
| cost | 1.100 | 1.131 ms/substep | +3% |

Still 50 s with no divergence, and it settles *harder*: torso drift over the last 40 s is
**0.01 cm**, against 0.2 cm before.

The ablation rows moved more than the shipped one, all in the same direction — the marginal
configurations survive substantially longer:

| config | before | after |
|---|---|---|
| `row+passive` | 2.54 s | **5.54 s** |
| `glob+passive` | 2.68 s | **5.00 s** |
| `glob+pas+lim` | 2.35 s | **3.72 s** |
| `ALL row` peak pen | 24.83 | **10.84** |

That is the expected direction: more rotational inertia means the same constraint impulse
produces less angular velocity, so the Gauss-Seidel sweeps have less to chase. It is
corroboration, not proof — nothing here was tuned for.

### What this does NOT mean

- **Still a derivation, not a measurement.** Nobody has authored an inertia tensor. Mass,
  radius and length are authored; the tensor is inferred from a shape assumption. `A-01`
  narrows, it does not close.
- **Storage is still diagonal-only** (`A-04`). The capsule's own tensor is genuinely
  diagonal in its local frame, so this is exact for the un-fused bodies — but the fused-Tip
  parallel-axis step still discards real off-diagonal terms.
- **The authored radii are now load-bearing for dynamics, and their coverage is unaudited**
  (`A-06`). A wrong radius used to produce a visibly wrong contact point; it now also
  silently produces wrong inertia. `Back3` at r/L = 0.85 is a sphere, not a bone — whether
  that is the intended collision geometry or an artefact of authoring for contact only has
  never been checked.
- **Passive drop only.** Zero torque throughout, so this says nothing about how the change
  interacts with the 0.5x–5x muscle strengths from entry 027 (`X-08`).

---

## Entry 029 — O-07 re-fit. ContactHertz 15 → 45. The passive drop could never have fitted this.

**Date:** 2026-08-21
**Trigger:** direct request — redo the shipped contact constants against the current rig.
**Status:** done. `ContactHertz` 15 → **45**; `ContactDampingRatio` stays at **10**, which
the sweep independently re-selected. 13/13 affected tests pass.

### The finding that matters more than the constant

**A passive drop cannot fit these constants. All 24 (Hertz, ζ) pairs survive it.**

That is the whole of the evidence entry 018 used, and it is the whole of the evidence
every contact sweep in this project's history has used. The regime does not discriminate:
it ranks penetration quality and says nothing about stability, because nothing is unstable
in it. Under torque babble at 30% of `MaxTorquePerDOF`, 5 of the same 24 diverge — and over
50 s, **only 3 of 15 survive**.

So the criticism recorded against O-07 (wrong joint axes in entry 022, no torso collision
in entry 025, thin-rod inertia in entry 028) was real but understated. Those three make the
old number *stale*. Using a passive drop makes it *unfalsifiable*.

### The gate kept being shorter than the failure mode

Three passes, each one selecting a pair that then died just past the window it was
selected in:

| gate | winner it picked | when that winner actually died |
|---|---|---|
| 6 s driven | 30 / 20 | 12.33 s |
| 12 s driven | 45 / 2 | 28.36 s |
| 50 s driven | **45 / 10** | survives |

The selection *rule* was fixed in the test header before any numbers existed and was not
changed — gate 1 already said "no divergence". What changed each time is that the gate was
being tested over a horizon shorter than the thing it was gating. Worth stating plainly
because the failure is seductive: each intermediate table looked clean and internally
consistent, and each one was fitted to noise.

### Result

50 s, both regimes:

| pair | regime | maxPen | restPen | torsoZ | |
|---|---|---|---|---|---|
| 15 / 10 (old) | drop | 18.89 | 2.01 | 85.43 | survives |
| 15 / 10 (old) | drive | 3.5e11 | — | — | **diverges 35.55 s** |
| 45 / 10 (new) | drop | 47.95 | 2.29 | 95.15 | survives |
| 45 / 10 (new) | drive | 55.68 | 3.13 | 86.11 | survives |

On the shipped-default rig check, resting penetration more than halves: **0.83 cm**,
against 1.90 at entry 028's constants.

**The trade was made deliberately and is a real cost.** Peak penetration on the passive
impact transient gets worse, 18.89 → 47.95 cm. A transient the rig recovers from beats a
divergence it does not, and O-06 already classifies peak penetration as a quality problem
rather than a stability one — but this is the first shipped change that makes O-06 worse
on purpose.

Runner-up was 15 / 2 (peak 49.58 / 38.25, restPen 1.24 / 3.99). Rejected by the rule on
resting penetration under load, and independently unattractive: ζ = 2 is underdamped for
contact, and at 45 Hz that same ζ is what died at 3.88 s in the ceiling ladder below.

### The actuation ceiling — and a correction

At the *interim* winner 45/2, full-amplitude babble diverged at 3.88 s with the authored
muscle strengths and 3.27 s with them pinned to 1, and I concluded from that pair alone
that "full-amplitude actuation is bounded by actuation, not contact stiffness — no contact
constant can fix it."

**That was wrong, and re-measuring at the real winner is what caught it.** At 45/10:

| amplitude | strengths | maxPen | restPen | 12 s result |
|---|---|---|---|---|
| 30% | authored | 29.45 | 2.04 | survives |
| 30% | pinned to 1 | 11.18 | 1.30 | survives |
| 60% | authored | 41.01 | 1.90 | survives |
| 60% | pinned to 1 | 26.66 | 1.81 | survives |
| 100% | authored | 62.29 | 3.01 | **survives** |
| 100% | pinned to 1 | 11.34 | 1.75 | survives |

The damping ratio, not the actuation, was the binding constraint. `ζ = 10` survives full
amplitude where `ζ = 2` at the same stiffness lasted under four seconds.

### X-08, now quantified

The authored muscle strengths cost real contact quality at every amplitude, and the gap
widens with load:

| amplitude | authored maxPen | pinned to 1 | ratio |
|---|---|---|---|
| 30% | 29.45 | 11.18 | 2.6× |
| 60% | 41.01 | 26.66 | 1.5× |
| 100% | 62.29 | 11.34 | **5.5×** |

So entry 027's 0.5×–5.0× spread no longer *destabilises* at these constants, but it does
degrade penetration by up to 5.5× under full load. X-08 stays open, downgraded from a
suspected stability risk to a measured quality cost.

### Two things this surfaced that were not being looked for

**The joint-stop stiffness gap has nearly closed.** `ContactHertz` is clamped internally to
`0.25/SubstepDt`, which at the shipped 1/240 is exactly 60 Hz. `JointLimitHertz` defaults to
60 — it was *already sitting on that ceiling*, which nothing recorded. Its comment claims
joint stops are "higher than ContactHertz on purpose"; that was a 4× gap and is now 1.33×,
with no headroom left to restore it. Buying more requires a smaller `PhysicsSubstepDt`.
Filed as **O-13**.

**A late passive transient appeared.** The 50 s confirmation shows peak penetration
climbing mid-run rather than being set at impact: 23.72 cm through t = 20 s, then 47.95 by
t = 30 s, with the torso ending higher (95.15) than it sat for most of the run. The old
constant was flat at 18.89 throughout. It recovers and never diverges, but a passive rig
should be monotonically calmer, not louder at t = 25 s. **Not explained.** Filed as
**O-14**.

### What this does NOT mean

- **Torque babble is not a policy.** Independent uniform draws on all 68 DOFs every 1/60 s,
  with no temporal correlation, is harsher than anything a trained policy emits — and
  harsher than a physical actuator could do, which is really a restatement of X-02 (no
  actuator model; torque is applied instantly). It is a defensible stress test and a rough
  stand-in for early PPO exploration. It is not evidence about trained behaviour.
- **O-08 is still open.** Surviving actuation is not standing. Nothing here rewards, or
  even measures, staying upright.
- **Only two constants moved.** `Slop`, `Iterations`, `GlobalIterations`, `Cfm`, friction
  and the limb-collision and joint-limit constants were all held fixed, so this is a fit of
  the two values O-07 names and not a joint optimisation. Several of them are now fitted
  around a different `ContactHertz` than they were chosen with.
- **The rig-check ablation table is no longer comparable to entries 026 and 028.** Its rows
  track the shipped defaults by design, so changing `ContactHertz` moved every one of them.
  Most of the ablations now diverge earlier, which is expected — they lack the mechanisms
  that make 45 Hz survivable — but the numbers in those two entries should not be read
  against the numbers this test prints today.
