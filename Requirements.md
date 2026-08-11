# Requirements: missile-mbd

Every requirement covered by the Simulink model, grouped by function and traced to (a) the specific model block that implements it and (b) the external source the requirement derives from. Each REQ-ID is also tagged in [`req_map.csv`](req_map.csv) (the machine-readable form used by the traceability report). This doc adds the Source column that `req_map.csv` doesn't carry.

Companion to [`docs/V_AND_V_PLAN.md`](docs/V_AND_V_PLAN.md) which is the verification strategy behind proving each requirement.

## Numbering scheme

```
REQ-MSL-<###>
```

Numbers ascend and are never reused. Deprecated REQ-IDs stay in place with a `[DEPRECATED]` marker rather than being renumbered.

## REQ-MSL-001..004: Proportional-navigation guidance law

| ID | Requirement | Verified by (block) | Source |
|---|---|---|---|
| REQ-MSL-001 | Guidance law shall compute commanded lateral acceleration as `a_cmd = N * Vc * omega_LOS` where N is the navigation constant, Vc is closing velocity, and omega_LOS is inertial line-of-sight rate | `missile/GuidanceSubsystem/TPN_Law` | Zarchan, *Tactical and Strategic Missile Guidance* (2019), ch. 2 |
| REQ-MSL-002 | Line-of-sight angular rate shall be computed as `omega = (r x r_dot) / \|r\|^2` where r is the target-missile relative position and r_dot is its time derivative | `missile/GuidanceSubsystem/LOS_Rate` | Siouris, *Missile Guidance and Control Systems* (2004), ch. 4 |
| REQ-MSL-003 | Closing velocity shall be computed as the negative rate of change of target-missile range, `Vc = -d\|r\|/dt` | `missile/GuidanceSubsystem/Closing_Velocity` | Zarchan (2019), ch. 2 |
| REQ-MSL-004 | The navigation constant N shall be a configurable model parameter with default value 4 | `model/missile_params.m` (N) | Zarchan (2019), ch. 3: typical N range 3 to 5 for tactical intercepts |

## REQ-MSL-005..007: Lateral-acceleration saturation

| ID | Requirement | Verified by (block) | Source |
|---|---|---|---|
| REQ-MSL-005 | Commanded lateral acceleration magnitude shall be saturated to `LATAX_MAX` | `missile/GuidanceSubsystem/Sat_a_cmd` | Representative short-range interceptor g-limit; Shneydor, *Missile Guidance and Pursuit* (1998), airframe envelope tables |
| REQ-MSL-006 | `LATAX_MAX` shall be a configurable model parameter expressed in metres per second squared | `model/missile_params.m` (LATAX_MAX) | Model-integration contract for consumers with different airframe envelopes |
| REQ-MSL-007 | Saturation shall preserve the direction of the commanded acceleration vector; only its magnitude shall be limited | `missile/GuidanceSubsystem/Sat_a_cmd` | Directional-preservation convention, Zarchan (2019), ch. 6: augmented PN implementations |

## REQ-MSL-008..010: Kinematic 3-DOF dynamics

| ID | Requirement | Verified by (block) | Source |
|---|---|---|---|
| REQ-MSL-008 | Missile velocity shall be updated by integrating the saturated commanded acceleration, `v_dot = a_cmd_sat` | `missile/KinematicSubsystem/Integrate_v` | Newton's second law for a point-mass; Shneydor (1998), ch. 3 |
| REQ-MSL-009 | Missile position shall be updated by integrating missile velocity, `r_dot = v` | `missile/KinematicSubsystem/Integrate_r` | Kinematic integration, standard convention |
| REQ-MSL-010 | Initial missile velocity magnitude shall be a configurable model parameter (`V_M0`) | `model/missile_params.m` (V_M0) | Launch-condition contract; representative short-range interceptor launch speed 500 m/s |

## REQ-MSL-011..013: Termination conditions

| ID | Requirement | Verified by (block) | Source |
|---|---|---|---|
| REQ-MSL-011 | An intercept shall be declared when the target-missile range falls below `R_LETHAL` | `missile/TerminationSubsystem/Intercept_Check` | Miss-distance-to-lethal-radius convention; Zarchan (2019), ch. 3 |
| REQ-MSL-012 | A timeout shall be declared when simulation time exceeds `T_MAX` seconds | `missile/TerminationSubsystem/Timeout_Check` | Bounded engagement time; ensures deterministic finite-horizon sim |
| REQ-MSL-013 | An LOS-reversal termination shall be declared when closing velocity becomes negative after passing its peak (target has passed the missile) | `missile/TerminationSubsystem/LOS_Reversal_Check` | Kinematic miss-detection heuristic; Shneydor (1998), ch. 5 |

## REQ-MSL-014..016: Root inport contracts

| ID | Requirement | Verified by (block) | Source |
|---|---|---|---|
| REQ-MSL-014 | Root inport `r_T` shall accept target position in metres in the inertial frame | `missile/r_T` | CLEARANCE integration contract |
| REQ-MSL-015 | Root inport `v_T` shall accept target velocity in metres per second in the inertial frame | `missile/v_T` | CLEARANCE integration contract |
| REQ-MSL-016 | Root inport `t` shall accept elapsed engagement time in seconds | `missile/t` | Enables per-instance timeout tracking without host-side clock coupling |

## REQ-MSL-017..019: Root outport contracts

| ID | Requirement | Verified by (block) | Source |
|---|---|---|---|
| REQ-MSL-017 | Root outport `r_M_out` shall expose missile position in metres in the inertial frame | `missile/r_M_out` | CLEARANCE integration contract: consumed by AClearanceMissile actor for placement |
| REQ-MSL-018 | Root outport `v_M_out` shall expose missile velocity in metres per second in the inertial frame | `missile/v_M_out` | CLEARANCE integration contract: consumed for orientation and next-tick prediction |
| REQ-MSL-019 | Root outport `term_flag_out` shall expose the current termination state as an integer (`0` in-flight, `1` intercept, `2` timeout, `3` LOS reversal) | `missile/term_flag_out` | CLEARANCE integration contract: drives Detonation PDU emission and scoring |

## Coverage summary

| Function | REQs | Verification tier |
|---|---:|---|
| TPN guidance law | 4 | T1 probe on known-solution intercept + T2 baseline regression + T3 CLEARANCE integration |
| Latax saturation | 3 | T1 probe (out-of-envelope command test) + T2 baseline regression |
| Kinematic 3-DOF | 3 | T2 baseline regression + T3 CLEARANCE integration |
| Termination | 3 | T1 probe (each termination path individually triggered) + T3 |
| Root inport contracts | 3 | T2 baseline regression + T3 wrapper-side type check |
| Root outport contracts | 3 | T2 baseline regression + T3 wrapper-side type check |
| **Total** | **19** | |

## Design decisions not captured as REQs

Some design choices are deliberately not requirements because they're implementation tuning that can change without affecting the external contract:

- **Specific value of `N`.** Model parameter with a sensible default; tunable per engagement without any REQ change (REQ-MSL-004 is about configurability, not the specific value).
- **Specific value of `LATAX_MAX`.** Same reasoning; airframe-dependent.
- **50 Hz fixed-step (h = 0.02 s), ode4 solver.** Codegen configuration; matches the autopilot / radar companion models for host-side scheduling consistency, but changeable if a different target platform needs a different rate.
- **LOS-rate computation via cross-product formulation vs true angle differentiation.** The cross-product form is numerically stable near boresight; the trigonometric form is mathematically equivalent but ill-conditioned near-zero. Implementation choice, not a requirement.

## What this deliberately doesn't cover

Called out so the scope is honest, not overclaimed:

- **No 6-DOF airframe.** No aero coefficients, no thrust curve, no fin actuator dynamics, no roll-pitch-yaw coupling. Commanded acceleration is applied directly to the point-mass kinematic integrator. A real interceptor would need a full airframe model; this model is the guidance layer that would sit above one.
- **No seeker model.** Target state is assumed perfectly observed. Seeker noise, angle-only tracking, glint, and radome error would live in a downstream layer (or in the `radar-mbd` companion for the radar seeker case).
- **No warhead / fuze model.** Intercept is a proximity-threshold decision (`|r| < R_LETHAL`). Detonation kinematics, fragmentation pattern, and lethality assessment are out of scope.
- **No boost / midcourse / terminal phase separation.** The law is single-phase TPN throughout the engagement. Real missiles typically boost-coast-terminal with different guidance modes each phase.
- **No wind or atmospheric effects.** Kinematic integration is in vacuum.

If this model shipped into a real interceptor programme, every bullet above would need to be addressed. Documenting what's not done keeps the current scope honest.

## Adding a new REQ-MSL-*

1. Add a probe or extend an existing verification block in the model.
2. Append a row to `req_map.csv` with the next available ID, the block path, and the description.
3. Add a row here in the appropriate section with the Source citation.
4. Regenerate `traceability_report.html` via `slreq.generateReport` so CI covers it.

The convention is intentionally lightweight. Heavier process wouldn't survive portfolio-project cadence, and the verification discipline is proven by the test-tier + probe structure, not by the doc's typography.

## References cited in the Source column

- **Zarchan, P.**, *Tactical and Strategic Missile Guidance*, AIAA Progress in Astronautics and Aeronautics, 7th ed. 2019. The standard reference for proportional navigation.
- **Siouris, G. M.**, *Missile Guidance and Control Systems*, Springer, 2004. Coordinate-frame conventions and LOS-rate derivations.
- **Shneydor, N. A.**, *Missile Guidance and Pursuit: Kinematics, Dynamics and Control*, Woodhead / Elsevier, 1998. Kinematic pursuit-and-evasion mathematics.
- **IEEE 1278.1-2012** Distributed Interactive Simulation, sections §7.4.3 (Fire PDU) and §7.4.4 (Detonation PDU) for federation-side integration.
