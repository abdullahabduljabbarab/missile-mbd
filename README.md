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
- **Real-world direct.** MBDA's own portfolio (Meteor, ASRAAM, Aster,
  Sea Ceptor) all use variants of proportional navigation with
  augmentations. Naming the law lands with any defence-sim audience.
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

*(Figures placeholder: model top-level, TPN block, kinematic block,
regression traceability report to be added once .slx is authored.)*

</div>

---

## Reusable-function code generation

The model is configured (`tools/configure_reusable_function.m`) with
**Code Interface Packaging = Reusable function**. Generated entry
points take a per-instance model pointer:

```c
void missile_initialize(RT_MODEL_missile_T *rtM);
void missile_step      (RT_MODEL_missile_T *rtM);
void missile_terminate (RT_MODEL_missile_T *rtM);
```

Every field of the run-time state (`blockIO`, `contStates`, `inputs`,
`outputs`, LOS memory) lives inside the `RT_MODEL_missile_T` struct
pointed to by `rtM`. Consumers allocate one per in-flight missile.
There are **no file-scope globals** and no shared state between
instances, a salvo of any size runs concurrently on the same
generated `.c`.

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
clearance.missile.fire <launcher_callsign> <target_callsign>
clearance.missile.abort <missile_id>
```

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
