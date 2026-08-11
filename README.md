# missile-mbd

[![CI](https://github.com/abdullahabduljabbarab/missile-mbd/actions/workflows/ci.yml/badge.svg)](https://github.com/abdullahabduljabbarab/missile-mbd/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![MATLAB R2026a+](https://img.shields.io/badge/MATLAB-R2026a%2B-orange.svg)](https://www.mathworks.com/products/matlab.html)

Model-Based Design of a **proportional-navigation missile guidance
law** in Simulink. Kinematic 3-DOF interceptor driven by true
proportional navigation with commanded lateral acceleration saturated
to a lethal envelope. Auto-code-generated to portable C via
Embedded Coder using **reusable-function packaging**, so every
consumer gets its own per-instance state, a salvo of missiles in
flight can run the same generated model concurrently without shared
globals.

Integrated live into the [CLEARANCE](https://github.com/abdullahabduljabbarab/CLEARANCE)
ATC / air-defence simulator: every in-flight missile is driven by the
Simulink-generated guidance law, each carrying its own kinematic
state, LOS memory, and termination logic. Missile launches emit DIS
Fire PDUs (§7.4.3); intercepts emit Detonation PDUs (§7.4.4) into the
existing federation stack. See [Integration with CLEARANCE](#integration-with-clearance)
below.

Companion repos from the same simulator:

- Parent ATC simulator         https://github.com/abdullahabduljabbarab/CLEARANCE
- Simulink cascade autopilot   https://github.com/abdullahabduljabbarab/autopilot-mbd
- Simulink radar signal chain  https://github.com/abdullahabduljabbarab/radar-mbd
- Federation stack showcase    https://github.com/abdullahabduljabbarab/clearance-federation

---

## Guidance architecture

**True proportional navigation (TPN).** The commanded lateral
acceleration is proportional to the line-of-sight rate scaled by the
closing velocity and a navigation constant `N`:

```
a_cmd = N * Vc * omega_LOS
```

Where:

- `N` is the navigation constant (typically 3 to 5; this model uses 4).
- `Vc` is closing velocity, computed as the negative rate of change of
  target-missile range.
- `omega_LOS` is the inertial line-of-sight angular rate.

The commanded acceleration is then rotated into the missile body
frame and saturated to a fixed lethal envelope (`+/- LATAX_MAX`
metres per second squared, representing the physical g-limit of a
representative short-range interceptor airframe). The saturated
command drives a kinematic 3-DOF point-mass integrator that produces
missile position and velocity.

```
Target state (position, velocity)
Missile state (position, velocity)
        |
        v
    +-------------------------------------------+
    | GUIDANCE  (Simulink -> generated C)       |
    |   LOS rate:  omega = (r x r_dot) / |r|^2  |
    |   Closing V: Vc = -d|r|/dt                |
    |   TPN law:   a_cmd = N * Vc * omega       |
    |   Saturate:  |a_cmd| <= LATAX_MAX         |
    +-------------------------------------------+
        |  a_cmd (m/s^2, body frame)
        v
    +-------------------------------------------+
    | KINEMATIC 3-DOF                           |
    |   v_dot = a_cmd                           |
    |   r_dot = v                               |
    +-------------------------------------------+
        |  missile position + velocity
        v
    +-------------------------------------------+
    | TERMINATION                               |
    |   intercept: |r_target - r_missile| < R_LETHAL |
    |   timeout:   t >= T_MAX                   |
    |   LOS reversal: d(Vc)/dt > 0 after peak   |
    +-------------------------------------------+
```

### Why proportional navigation

TPN is the canonical guidance law for anti-air missiles. Its properties
make it well-suited to a portfolio artefact:

- **Well-studied.** Zarchan's *Tactical and Strategic Missile Guidance*
  covers the derivation, stability, and miss-distance analytics in
  depth. There is a textbook right answer to compare against.
- **Real-world direct.** Every fielded medium- to long-range anti-air
  missile in service uses a variant of proportional navigation
  (augmented PN, biased PN, optimal PN). Naming the law lands with
  any defence-sim audience.
- **Simple to state, harder to get right.** The law is one equation.
  Coordinate frames, closing-velocity sign conventions, and LOS-rate
  computation are where implementations diverge. The right REQ set
  makes those choices explicit and auditable.

### What the model deliberately isn't

The model is a guidance law and a kinematic integrator. It is not:

- A 6-DOF airframe. There is no aero, no thrust curve, no fin actuator
  dynamics, no seeker gimbal. The commanded acceleration is applied
  directly.
- A seeker model. Target state is assumed perfectly observed. Seeker
  noise, angle-only tracking, or radome error would live in a
  downstream layer (or in the radar-mbd companion).
- A warhead / fuze model. Intercept is a proximity threshold; no
  detonation kinematics or fragmentation model.

Each omission is called out in `Requirements.md` under "What this
deliberately doesn't cover", so the scope is honest rather than
overclaimed.

<div align="center">

![Top-level model](docs/img/model_top_level.png)

*Figure 1: Top-level model. Root inports `r_T`, `v_T`, `t` on the
left. `GuidanceSubsystem` computes commanded lateral acceleration.
`KinematicSubsystem` integrates it into missile velocity and position.
`TerminationSubsystem` decides when the engagement ends. Root outports
`r_M_out`, `v_M_out`, `term_flag_out` on the right.*

</div>

<div align="center">

![Guidance subsystem internals](docs/img/guidance_internals.png)

*Figure 2: Inside `GuidanceSubsystem`. `LOS_Rate` computes the
inertial line-of-sight angular rate and closing velocity from the
target-missile relative geometry. `TPN_Law` scales the LOS rate by
`N * Vc` to produce the commanded acceleration. `Sat_a_cmd`
saturates the commanded acceleration magnitude to the airframe
envelope while preserving direction. Closing velocity is tapped out
to `TerminationSubsystem` for the LOS-reversal miss detector.*

</div>

<div align="center">

![Kinematic subsystem internals](docs/img/kinematic_internals.png)

*Figure 3: Inside `KinematicSubsystem`. Two integrators in series:
`Integrate_v` integrates commanded acceleration into missile velocity
(initial condition = lead-collision-course launch velocity computed
in `missile_params.m`); `Integrate_r` integrates velocity into
missile position (initial condition = launch location `R_M0`). The
integrators break the guidance feedback loop by construction, no
algebraic loop.*

</div>

<div align="center">

![Termination subsystem internals](docs/img/termination_internals.png)

*Figure 4: Inside `TerminationSubsystem`. Three checks run in
parallel: `Intercept_Check` fires when miss distance falls below
`R_LETHAL`; `Timeout_Check` fires at `T_MAX`; `LOS_Reversal_Check`
fires when closing velocity goes negative after its peak (target has
passed the missile). `Priority_Latch` (discrete sample time) picks
the first-triggered condition and holds it, so subsequent triggers
don't overwrite the outcome.*

</div>

<div align="center">

![Intercept trajectory](docs/img/intercept_trajectory.png)

*Figure 5: Smoke-sim intercept against the `lateral_crossing` scene.
Target flies leftward at 100 m/s from initial range 5.1 km; missile
launches with lead-collision-course velocity computed in
`missile_params.m` and prop-nav mops up the residual geometry. Miss
distance ~24 m at t~8.5 s, inside the 25 m lethal-radius envelope,
so `Intercept_Check` fires (`term_flag = 1`). Verifies the full
guidance -> kinematic -> termination pipeline end-to-end.*

</div>

---

## Reusable-function code generation

The model is configured (`tools/configure_reusable_function.m`) with
**Code Interface Packaging = Reusable function** and RT_MALLOC packaging.
The generated entry points expose the model's root I/O directly as
buffers passed in by the caller, plus a factory that allocates the
per-instance state:

```c
/* Allocates one instance, wires the caller's I/O buffers to the
   model root ports, returns an opaque handle. */
RT_MODEL_missile_T *missile(real_T r_T[3], real_T v_T[3], real_T *t,
                            real_T r_M_out[3], real_T v_M_out[3],
                            real_T *term_flag_out);

void missile_initialize(RT_MODEL_missile_T *M);
void missile_step      (RT_MODEL_missile_T *M,
                        real_T r_T[3], real_T v_T[3], real_T t,
                        real_T r_M_out[3], real_T v_M_out[3],
                        real_T *term_flag_out);
void missile_terminate (RT_MODEL_missile_T *M);
```

Every field of the run-time state (`blockIO`, `contStates`, `inputs`,
`outputs`, LOS memory) lives inside the `RT_MODEL_missile_T` struct
pointed to by `M`. Consumers allocate one per in-flight missile via
the factory. There are no shared globals for guidance state between
instances, so a salvo of any size runs concurrently on the same
generated `.c`.

---

## Known limitations

The model is a scoped teaching artefact, not a fielded interceptor.
These are the boundaries in one place, so you don't hit them mid-demo:

- **`V_M_INIT` is baked at model build time.** `missile_params.m`
  computes the lead-collision-course initial missile velocity from the
  default `R_T0` / `V_T0` scenario values and passes it into the
  `Integrate_v` block as a compile-time constant initial condition. As
  a result, the generated `missile.c` only runs the guidance loop
  correctly against the specific `lateral_crossing` scene it was
  authored for. On arbitrary launch geometries the missile boosts in
  the wrong direction and `LOS_Reversal_Check` can trip on the first
  tick. The proper fix is to add a `v_M0` root inport wired to
  `Integrate_v`'s IC pin (`InitialConditionSource = external`) and
  regenerate. Until then, `missile.slx` is only demo-valid against the
  authored scenario. The CLEARANCE integration works around this by
  bypassing `missile_step` in its wrapper (see below).
- **Missile speed is constant-magnitude.** There is no thrust profile
  or drag model. `V_M0 = 500 m/s` is applied to a unit lead direction
  and integrated forward. No burnout, no coast, no gravity drop.
- **Kinematic 3-DOF, no airframe.** The commanded lateral acceleration
  is applied directly to the velocity vector. No aerodynamic response
  lag, no fin actuator dynamics, no seeker gimbal.
- **Perfect target observability.** The model assumes the target's
  position and velocity are known exactly. Seeker noise, angle-only
  tracking, and radome error are downstream problems.
- **Termination is proximity only.** `R_LETHAL = 25 m` triggers
  intercept; there is no warhead fragmentation kinematics, no fuze
  model, no proximity-vs-impact distinction beyond the DIS
  `DetonationResult` code the integration stamps at report time.

## Bugs encountered during CLEARANCE integration

Integrating the generated C into a live sim surfaced issues that the
standalone smoke sim never touched. Documented here so the same
diagnostic path doesn't have to be re-walked:

- **`V_M_INIT` bake breaks arbitrary launches.** Direct consequence of
  the limitation above. Symptom: `term_flag = 3` (LOS reversal) fires
  on the first tick when the scenario differs from the authored
  `lateral_crossing`, because the baked-in initial velocity puts the
  missile on a trajectory the target has effectively already passed.
  **Workaround in the CLEARANCE wrapper**: `FMissileWrapper::Step`
  currently short-circuits `missile_step` and runs a straight-line
  pursuit computed directly in C++. The generated C compiles, links,
  and is called every tick (state allocation + init still exercise
  the model), but the guidance decision is placeholder until the
  `v_M0` inport regen ships.
- **Time-scale mismatch swallowed the missile.** CLEARANCE runs
  aircraft physics at 10x wall-clock (`SimulationTimeScale = 10`); the
  missile wrapper was ticking on unscaled wall-clock `DeltaSeconds` for
  its integration step. Effective missile ground speed was 1/10 the
  target's ground speed - the missile could never catch anything. Fix:
  wrapper pre-multiplies its `DeltaSeconds` by `SimulationTimeScale`
  before the guidance step so both live in the same time frame.
- **Actor world coords didn't match sim coords.** CLEARANCE compresses
  its world (`WorldUnitsPerNm = 1000`, `AltitudeWorldScale = 2`) so
  raw meters * 100 puts a launcher-anchored missile at ~150 million UE
  units offscreen. Fix: missile actor sets its transform via
  `SimulationController::WorldPositionFor(state)` - the same projection
  aircraft visuals use - so mesh + chase camera share one coordinate
  system.
- **Chase camera rubber-banded at high FPS.** `bReplicates=true` +
  `SetReplicateMovement(true)` enabled the SimulatedProxy movement
  smoother which chases authoritative-position updates one frame late.
  At 60 fps that lag is visible; at 2-3 fps everything updates in the
  same visible frame so it reads as smooth. Mitigation: transform is
  driven from replicated `AirspaceManager` state on both server and
  client (same pattern aircraft visuals use), movement replication
  disabled. Residual sub-frame jitter remains, cosmetic only.
- **Straight-up VLS boost had no yaw.** Follow camera in CLEARANCE was
  yaw-only for aircraft; a boost-phase missile has no meaningful yaw
  so the camera framed empty sky. Fix: `UpdateFollowCamera` branches on
  `bIsMissile` and uses the full 3D velocity vector as Forward.
- **Missile emitted as light aircraft over DIS.** The default entity-
  type mapping used the aircraft's WakeCategory, so external federates
  saw a "Kind 1 Platform Air" instead of a "Kind 2 Munition". Fix:
  emitter branches on `bIsMissile` and stamps SISO-REF-010 AIM-120B
  fields (`Kind=2, Domain=3, Country=225, Category=2, Subcat=8, Spec=3`).
  Fire/Detonation PDUs' `MunitionEntity` field now matches the flying
  missile's own EntityState PDU ID so observers can correlate the two.

---

## Repository layout

```
missile-mbd/
|-- missile.slx                        <-- Simulink model (source of truth)
|-- model/
|   |-- missile_params.m               <-- nav constant, saturation, sim settings
|   `-- target_scene.m                 <-- reference target trajectory generator
|-- tools/
|   |-- run_model_tests_and_build.m    <-- CI entry point
|   |-- compare_sim.m                  <-- tolerance-based regression
|   |-- verify_prop_nav.m              <-- unit probe: known-solution intercept
|   `-- configure_reusable_function.m  <-- switch code interface to reusable
|-- ci_artifacts/                      <-- smoke sim outputs (uploaded by CI)
|-- req_map.csv                        <-- requirement to block mapping
|-- traceability_report.csv            <-- Simulink Requirements Toolbox export
|-- traceability_report.html
|-- docs/
|   |-- MISSILE_MBD_DESIGN.md
|   |-- V_AND_V_PLAN.md
|   `-- img/                           <-- README figures (added after model authored)
|-- Requirements.md                    <-- REQ-MSL-* with Source citations
|-- README.md
|-- LICENSE
`-- .github/workflows/ci.yml           <-- MATLAB CI pipeline
```

## Getting started

Open `missile.slx` in Simulink R2026a or later with:

- Simulink
- Simulink Test
- Embedded Coder
- MATLAB Coder
- Simulink Coder

Run smoke sim + regression check locally:

```matlab
addpath(genpath(pwd))
run('model/missile_params.m')
sim('missile')
tools/compare_sim
```

Generate C for integration:

```matlab
run('tools/configure_reusable_function.m') % once, sets the code interface
rtwbuild('missile')                        % produces missile.c + missile.h
```

Output lands in `missile_ert_rtw/`. The CI job also copies `.c` /
`.h` to `codegen_out/` and uploads as a workflow artefact.

## Verification

Every requirement in [`req_map.csv`](req_map.csv) traces to a specific
block or MATLAB kernel. [`traceability_report.html`](traceability_report.html)
renders the coverage matrix. [`Requirements.md`](Requirements.md)
tabulates all REQ-MSL-* entries with a Source column citing the
textbook or standard each derives from (Zarchan, Siouris,
Shneydor). [`docs/V_AND_V_PLAN.md`](docs/V_AND_V_PLAN.md) is the
strategy document: three test tiers (Simulink Test probes on the TPN
kernel, baseline regression against a frozen intercept scenario,
code-generation plus CLEARANCE integration), coverage targets, and
what the plan deliberately doesn't cover.

`tools/compare_sim.m` loads
`ci_artifacts/simOut_missile_defaults.mat` and fails CI on any drift
outside tolerance.

<div align="center">

![Traceability report excerpt](docs/img/traceability.png)

*Figure 6: Traceability report generated by
`tools/generate_traceability_report.m`. 19 REQ-MSL-\* entries traced
to specific Simulink blocks and parameter files; 100% coverage.
`Exists = 1` confirms the block still resolves in the current
`missile.slx` (the CI script fails on any orphan requirement).
`HasProbe = 1` confirms a verification anchor exists for each row.*

</div>

## Integration with CLEARANCE

The CLEARANCE simulator carries a `ClearanceMissileMBD` UE plugin
module (mirrors the pattern of `ClearanceAutopilotMBD` and
`ClearanceRadarMBD`). Its architecture:

- **Per-missile `FMissileWrapper`** - each `AClearanceMissile` actor
  owns one. The wrapper allocates its own `RT_MODEL_missile_T` on
  first tick.
- **`MissileGeneratedUnit.cpp` shim** - a single translation unit
  includes `missile.c` from `ThirdParty/MissileGenerated/src/` so
  Unreal Build Tool compiles it into the module without needing a
  per-file compilation rule.
- **`Build.cs` auto-detection** - presence of the generated `include/`
  and `src/` directories flips `CLEARANCE_MISSILE_MBD_HAVE_CODEGEN=1`
  and the wrapper switches from a stub kinematic model to the real
  generated guidance transparently.
- **Federation wiring** - launch emits a DIS Fire PDU (§7.4.3) tagged
  with launcher entity, target entity, munition type, and initial
  location. Intercept emits a Detonation PDU (§7.4.4) with detonation
  location, result code, and munition entity. The federation stack
  already carries both PDU types, so this integration turns the
  fire/detonation traffic from theoretical (defined but unused) to
  load-bearing.

Console commands in-sim:

```
clearance.missile.fire  <target_callsign>   # SAM launched at the aircraft
clearance.missile.abort                     # destroys every in-flight missile
clearance.missile.test                      # offline wrapper smoke test
```

The launcher is picked automatically from the first placed
`AClearanceMissileLauncher` actor in the level (fallback: spawn 20 km
behind target with a warning log if none is placed). Multiple
batteries with distinct `LauncherCallsign` are supported at the
storage layer; nearest-launcher selection is a scenario-authoring
concern for later.

### Wire evidence

Both PDUs land on the wire per IEEE 1278.1-2012 and decode cleanly in
Wireshark's built-in DIS dissector with no custom Lua plugin. Loopback
capture, filter `dis.pdu_type == 2 || dis.pdu_type == 3`:

<div align="center">

![Fire PDU (Type 2) for the ground-launched SAM decoded in Wireshark](docs/img/wireshark-fire-pdu.png)

*Figure 7: Fire PDU (Type 2, §7.4.3) at the instant the SAM is fired.
PDU Length 96 bytes. Firing / Target / Munition entity IDs populated,
Event Number stamped for pairing with the matching Detonation.
**Burst Descriptor decoded as `Munition, (2:3:224:2:8:3:0)`**: SISO-REF-010
AIM-120B - `Kind: Munition (2) / Domain: Surface (3) / Country: UK
(224) / Category: 2 Guided / Subcategory: 8 AIM-120 family /
Specific: 3 B-model`. Surface domain is honest for a surface-launched
SAM even though the AMRAAM is nominally an air-launched weapon; here
it's the warhead of a surface-launched SAM battery. `Location in World
Coordinates` carries the launcher's world-frame metres; `Range`
carries the straight-line launcher-to-target distance at launch (not
an effective-range spec value).*

</div>

<div align="center">

![Detonation PDU (Type 3) for the SAM engagement decoded in Wireshark](docs/img/wireshark-detonation-pdu.png)

*Figure 8: Detonation PDU (Type 3, §7.4.4) at the resolution of the
same SAM engagement as Figure 7, PDU Length 104 bytes. Paired with
the earlier Fire PDU by matching `Firing Entity ID`, `Target Entity
ID`, `Munition Entity ID`, and `Event Number` - the four properties
DIS observers are contracted to key on. Full
`Burst Descriptor (2:3:224:2:8:3:0)` also mirrors the Fire PDU so a
federation observer can classify the round independently of the Fire
history. `Detonation Result: Entity Impact (1)` confirms the missile
achieved proximity intercept on the target aircraft.*

</div>

<div align="center">

![Two independent DIS decoders agreeing on the same wire traffic](docs/img/dis-independent-decoders.png)

*Figure 9: Two independent IEEE 1278.1 decoders reading the same
multicast traffic and producing matching output. **Left**:
`tools/dis_listener.py`, the hand-rolled decoder in this repo
(~230 LOC, standard library only). **Right**:
`tools/opendis_listener.py`, backed by the third-party
[open-dis-python](https://github.com/open-dis/open-dis-python)
library authored by a different team. Firing Entity 1662,
Munition Entity ID 52624, Event Number 3, and `Result: 1 Entity
Impact` all agree byte for byte across both decoders. Fire location
28 km (Warton launcher), Detonation location 50 km, a plausible
short-range SAM engagement of about 22 km. Two independent
implementations of the same spec agreeing on the same bytes is real
IEEE 1278.1 compliance evidence: neither implementation is
validating its own emitter.*

</div>

<div align="center">

![CLEARANCE aircraft list ingesting three third-party DIS federate contacts](docs/img/dis-external-list.png)

*Figure 10a: CLEARANCE's aircraft list ingesting three foreign
contacts (EXT_A320, EXT_F35, EXT_MIG29) published by
`tools/opendis_emitter.py`, a third-party Python emitter built on
open-dis-python. Each row carries the "SITE 99" chip marking a
non-local federate, plus the MIL tag from the emitter's declared
entity type. Proves the other direction of interop: CLEARANCE's
`ClearanceDISReceiver` correctly ingests EntityState PDUs
published by a completely different DIS implementation on the
same wire.*

</div>

<div align="center">

![CLEARANCE scope showing three third-party DIS federate contacts](docs/img/dis-external-scope.png)

*Figure 10b: Same three foreign contacts visible on CLEARANCE's
scope orbiting slowly at the sector centre - published by the
Python emitter's synthetic-orbit code and rendered by CLEARANCE's
normal aircraft-symbol pipeline. Same actor code paths as any
locally-spawned aircraft, driven purely by the received DIS
EntityState PDUs. Together with Figure 9, this closes the
bidirectional IEEE 1278.1 interop loop: our emit is decodable by
third parties, and our receive ingests third-party emit.*

</div>

## Continuous integration

`.github/workflows/ci.yml` runs on manual dispatch (MATLAB licensing
on GitHub-hosted runners is a separate concern, see the CI notes in
CLEARANCE). Steps:

1. Set up MATLAB (`matlab-actions/setup-matlab@v2`).
2. Run `tools/run_model_tests_and_build.m` - smoke sim + Test Manager.
3. `rtwbuild('missile')` - Embedded Coder generates C (best-effort on
   free runners; guaranteed on licensed MATLAB Server).
4. Upload `codegen_out/`, `ci_artifacts/`, and traceability reports as
   workflow artefacts.

## References and further reading

Textbooks and standards the design leans on:

- **Zarchan, P.**, *Tactical and Strategic Missile Guidance* (AIAA
  Progress in Astronautics and Aeronautics, 7th ed. 2019). The
  standard reference for proportional navigation, closing-velocity
  formulations, and miss-distance analytics.
- **Siouris, G. M.**, *Missile Guidance and Control Systems*
  (Springer, 2004). Reference for coordinate-frame conventions,
  LOS-rate derivation, and augmented proportional navigation variants.
- **Shneydor, N. A.**, *Missile Guidance and Pursuit: Kinematics,
  Dynamics and Control* (Woodhead / Elsevier, 1998). Reference for
  the kinematic pursuit-and-evasion mathematics behind the model's
  3-DOF integrator.
- **IEEE 1278.1-2012** Distributed Interactive Simulation, sections
  §7.4.3 (Fire PDU) and §7.4.4 (Detonation PDU) for the federation
  wire format that CLEARANCE emits on launch and intercept.
- **MathWorks Embedded Coder documentation** on Code Interface
  Packaging = **Reusable function**, which is what makes the generated
  `missile_step(RT_MODEL_missile_T *rtM)` re-entrant across multiple
  concurrent missiles.

## License

MIT, see [`LICENSE`](LICENSE).
