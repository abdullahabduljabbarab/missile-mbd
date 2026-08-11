# Verification and Validation Plan: missile-mbd

Companion to [`../req_map.csv`](../req_map.csv) (the source-of-truth requirement-to-block mapping) and [`../traceability_report.html`](../traceability_report.html) (the rendered coverage matrix). If `req_map.csv` answers "what is the model supposed to do?", this doc answers "how do we prove it does?".

## 1. Purpose and scope

Verification and validation on a Model-Based Design project is a proportionality exercise. This is a portfolio-scale Simulink model of a proportional-navigation guidance law, not a certified weapon-system guidance controller. So it does not need DO-178C / DO-331 tool qualification rigour, but it does need to demonstrate the discipline that a real defence programme would exhibit: traceable requirements, tiered verification, and evidence a reviewer can audit without running MATLAB.

### In scope

| Item | Notes |
|---|---|
| Every requirement in `req_map.csv` | 19 REQ-MSL-* entries across TPN guidance law, latax saturation, kinematic 3-DOF, termination, and root port contracts |
| Every block, subsystem, and root port cited by a requirement | The `Block` column of the CSV is grep-able against the model and against the `verify_*.m` scripts |
| Known-solution correctness of the TPN guidance law | `tools/verify_prop_nav.m` runs a canonical lateral-crossing intercept and asserts the miss distance falls inside the lethal radius |
| Directional-preservation of the latax saturation | Verified via out-of-envelope command probe (any commanded direction, scaled to LATAX_MAX magnitude) |
| Termination-flag correctness for each of the three termination paths | Each triggered individually by a canned scene (intercept, timeout, LOS reversal) |
| Embedded-Coder-generated C's structural equivalence to the model | Regression check via `tools/compare_sim.m` comparing generated-code sim vs `sim('missile')` reference |
| Reusable-function packaging isolation between missile instances | Verified informally through CLEARANCE integration (multi-missile salvo, no shared-state bugs); no automated multi-instance test in this repo |

### Out of scope

| Item | Reason |
|---|---|
| DO-178C / DO-331 model coverage (MC/DC, statement, decision) | Portfolio scale |
| Formal miss-distance guarantees under adversarial target maneuvers | Requires analytic worst-case analysis with airframe latax bounds; downstream work |
| Hardware-in-the-loop testing | No target hardware; CLEARANCE integration IS the loop |
| Airframe / fin actuator dynamics | Explicitly out of model scope per `Requirements.md` "What this deliberately doesn't cover" |
| Seeker noise, radome error, angle-only tracking | Out of model scope; would live in a downstream seeker layer |
| Warhead / fuze / lethality assessment | Out of model scope; intercept is a proximity threshold only |

## 2. Test tiers

Three tiers, each covering different requirement classes at different cost.

| Tier | Definition | Cost | Where they live | When to use |
|---|---|---|---|---|
| **T1 Simulink Test / probe verification** | Model-in-the-loop tests using `verify_*.m` scripts that instrument the block outputs via probes, run `sim(model)`, and assert numeric properties (intercept declared, miss distance < R_LETHAL, LATAX_MAX honoured, correct termination flag for each canned scene). | Low. Sub-second per test on a warm session. | `tools/verify_prop_nav.m`, referenced from `req_map.csv`. Add more `verify_*.m` as new REQs land. | Any pure-model requirement: guidance law behaviour, saturation, termination logic, root-port units. |
| **T2 Regression against baseline** | `tools/compare_sim.m` loads `ci_artifacts/simOut_missile_defaults.mat` (the frozen reference), reruns the model, and asserts each signal is within tolerance of the baseline. Catches accidental drift from any block-config change. | Low-medium. Requires a fresh Simulink session but runs headless. | `tools/compare_sim.m`, `tools/run_model_tests_and_build.m` (CI entry point). | Whole-model integrity checks, especially after any tuning parameter or block reconfiguration. |
| **T3 Code-generation + CLEARANCE integration** | `rtwbuild('missile')` generates C, CLEARANCE builds the wrapper module, missile flies against a target in-sim. Any behavioural regression versus pure-model sim is caught by observing miss distance, and the DIS Fire / Detonation PDUs are inspectable in Wireshark. | High. Requires full toolchain and CLEARANCE build. | `MISSILE_MBD_DESIGN.md` describes the CLEARANCE integration path; `.github/workflows/ci.yml` covers the code-generation half. | Every release. Every time N, LATAX_MAX, or termination logic changes. |

### Selection rule

Default to T1. Escalate to T2 when a model-wide integrity concern applies (baseline drift, config change). T3 always runs before a release; the same discipline the autopilot's `Kd = 0` solver-stability finding demonstrated (solver-boundary bugs invisible in pure-model sim, only visible at code-gen + integration) applies here too.

## 3. Traceability

`req_map.csv` is the traceability matrix. Every row maps one REQ-MSL-* to one Simulink block or MATLAB kernel. `traceability_report.html` renders it with hyperlinks; `traceability_report.csv` is the machine-readable form used by CI to fail on missing coverage.

### Coverage discipline

| Rule | How it is enforced |
|---|---|
| Every REQ-MSL-* must have exactly one Block cited | `req_map.csv` schema; CI script asserts no orphan REQ-IDs |
| Every Block cited must exist in the current model | `tools/run_model_tests_and_build.m` opens the model and resolves each block path; missing block fails the build |
| Every Block that a T1 probe reads must be reachable | Simulink Test assertions catch a probed-but-missing block as a test failure |

### Currently green (target state; achieved once `missile.slx` is authored)

- 19 of 19 REQ-MSL-* entries mapped
- 19 of 19 mapped blocks resolve in the current `missile.slx`
- Regression check passes against the frozen baseline in `ci_artifacts/simOut_missile_defaults.mat`
- CLEARANCE integration flies clean intercepts with Fire and Detonation PDUs observable in Wireshark

## 4. Coverage targets

Self-imposed discipline goals, not regulatory obligations.

| # | Target | Rule | Current status |
|---|---|---|---|
| 1 | REQ-MSL coverage | Every requirement has at least one T1 verification probe or a T2 regression signal | 19 of 19 mapped (probes to be attached once `missile.slx` is authored) |
| 2 | Known-solution correctness | `verify_prop_nav.m` on the `lateral_crossing` scene must declare intercept with miss < R_LETHAL | To be verified once model is authored |
| 3 | Termination-path coverage | Each of the three termination flags (intercept, timeout, LOS reversal) must be independently triggered by a canned scene | To be verified once model is authored |
| 4 | Code-generation equivalence | Generated C sim output must match pure-model sim within regression tolerance | To be verified after first codegen |
| 5 | Reusable-function isolation | Salvo of concurrent missile instances must not share state via file-scope globals | To be verified by inspection of generated `.c` after first codegen |

## 5. When to run what

| Trigger | T1 | T2 | T3 (code-gen + CLEARANCE integration) |
|---|:-:|:-:|:-:|
| Any block edit in `missile.slx` | ✓ | ✓ | |
| Any tuning parameter change in `model/missile_params.m` (N, LATAX_MAX, R_LETHAL, T_MAX) | ✓ | ✓ | ✓ |
| Any termination logic change | ✓ | ✓ | ✓ |
| MATLAB / Simulink version upgrade | ✓ | ✓ | ✓ |
| Before shipping to CLEARANCE | ✓ | ✓ | ✓ |
| Before recording a demo video | ✓ | ✓ | ✓ |

## 6. Change control

`req_map.csv` and this doc live with the model in the same repo. Changes to requirements are committed alongside the model change that motivates them.

- **New REQ-MSL-***: append to `req_map.csv` with a stable ID, add a `verify_*.m` probe or extend an existing one.
- **Removing a REQ-MSL-***: mark the row with a `[DEPRECATED]` suffix in the Description column; don't reuse the ID.
- **Changing a REQ-MSL text**: increment the description; the ID stays; the verification probe must still cover the new intent.

## 7. What this doc deliberately doesn't cover

- **Formal certification artefacts** (DO-178C model coverage, DO-331 tool qualification, hazard analysis). Not portfolio scale.
- **Adversarial worst-case miss-distance analysis** across the airframe latax envelope. Would require analytic bounds on target maneuver capability.
- **Hardware-in-the-loop testing**. No target hardware.
- **Independent verification by a separate team**. Solo portfolio project; the closest thing to independent verification is the CLEARANCE integration exercising the generated code in a different codebase.

If this missile guidance model were shipping into a certified weapon programme, every bullet above would need to be addressed. Documenting what's not done makes the current scope honest rather than pretending everything's covered.
