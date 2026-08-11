# Missile guidance design notes

Design brief for the missile in `missile.slx`. Reads left-to-right:
what the model does, why the law is what it is, and how the generated
C ends up flying intercepts inside the CLEARANCE ATC / air-defence
simulator.

## What it is

A kinematic 3-DOF interceptor driven by **true proportional
navigation** (TPN). One guidance kernel, one saturation, one point-
mass integrator, three termination conditions.

- **Guidance kernel**: computes commanded lateral acceleration from
  the line-of-sight rate scaled by closing velocity and navigation
  constant N.
- **Saturation**: limits the commanded acceleration magnitude to a
  fixed lethal envelope, preserving direction.
- **Kinematic 3-DOF**: integrates the saturated command into missile
  velocity, integrates velocity into position. No aero, no thrust
  curve, no fin dynamics.
- **Termination**: intercept on lethal-radius proximity, timeout on
  engagement horizon, LOS reversal on target passing.

The model has three root inports (`r_T`, `v_T`, `t`) and three root
outports (`r_M_out`, `v_M_out`, `term_flag_out`). Nothing else.

## Guidance law

**True proportional navigation.** For a target-missile relative
position vector `r = r_T - r_M` with time derivative `r_dot`:

```
omega_LOS = (r x r_dot) / |r|^2         % inertial LOS angular rate
Vc        = -d|r|/dt                    % closing velocity, positive when converging
a_cmd     = N * Vc * omega_LOS          % commanded acceleration, perpendicular to LOS
```

The cross-product formulation for `omega_LOS` is numerically stable
near boresight (small angle) where the trigonometric form is ill-
conditioned. Closing velocity is signed so the guidance law naturally
disengages once the target has passed (Vc goes negative, a_cmd
reverses; LOS-reversal termination fires shortly after).

`N = 4` by default. Zarchan (2019) ch. 3 shows tactical intercepts
typically use N in `[3, 5]`; below 3 the missile lags a maneuvering
target, above 5 the airframe saturates on lateral acceleration
demand.

## Saturation

Commanded acceleration is limited to `LATAX_MAX = 40g` (~392 m/s^2),
a representative short-range interceptor g-limit. The saturation
preserves direction:

```
if |a_cmd| > LATAX_MAX
    a_cmd_sat = a_cmd * LATAX_MAX / |a_cmd|
else
    a_cmd_sat = a_cmd
end
```

Directional preservation matters. Clipping each Cartesian component
independently would rotate the commanded acceleration vector inside
the envelope and reduce the miss-distance analytic guarantees TPN
gives you. Preserving direction is the conventional choice across
Zarchan, Siouris, and Shneydor.

## Kinematic 3-DOF

Point-mass integration in the inertial frame:

```
v_M_dot = a_cmd_sat
r_M_dot = v_M
```

No airframe. No aero forces. No thrust decay. This is a guidance
model, not a missile simulation. Real airframe dynamics would sit
between `a_cmd_sat` (guidance command) and `v_M_dot` (achieved
acceleration), and would need a control system to close the gap.
That control system would be a downstream repo, mirroring how this
model sits downstream of the seeker layer.

Initial conditions:

- Missile position `R_M0 = [0; 0; 0]` at launch.
- Missile velocity initialised to `V_M0 * unit(R_T0 - R_M0)`, i.e.
  aligned with the target initial position. Represents the launcher
  pointing at the target at fire time.

## Termination

Three conditions, each producing a distinct `term_flag`:

- **Intercept (`term_flag = 1`)**: `|r_T - r_M| < R_LETHAL`. Missile
  came within the lethal radius. `R_LETHAL = 5 m` by default;
  represents the effective warhead envelope.
- **Timeout (`term_flag = 2`)**: `t >= T_MAX`. Missile ran out of
  engagement time. `T_MAX = 60 s`.
- **LOS reversal (`term_flag = 3`)**: closing velocity `Vc` becomes
  negative after passing its peak. Target has passed the missile
  without intercept; kinematic miss.

`term_flag = 0` while in-flight. First-triggered condition wins;
subsequent conditions do not overwrite.

## Code generation

Configured for **reusable-function packaging** (see
`tools/configure_reusable_function.m`). This is the important detail
for the CLEARANCE integration.

Default Embedded Coder puts model state (block IO, continuous states,
external inputs, external outputs) in file-scope globals. That's fine
for a rig with one missile but breaks the moment you want two, a
salvo of two missiles would trash each other's LOS integrators and
termination state. Reusable-function packaging moves the state into
an `RT_MODEL_missile_T` struct that the caller allocates. Every entry
point takes a pointer:

```c
/* Factory: allocates an instance, wires the caller's I/O buffers to
   the model's root inports/outports, returns the handle. */
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

CLEARANCE allocates one per in-flight missile via the factory. A
salvo of any size runs concurrently without any shared state.

Codegen target is `ert.tlc` (Embedded Real-Time) with fixed-step
solver at 0.02 s (50 Hz), ode4. Matches the autopilot / radar
companion models so the CLEARANCE-side scheduler runs everything on
the same tick boundary.

Regenerating C from the model:

```matlab
run('tools/configure_reusable_function.m')  % once per fresh workspace
rtwbuild('missile')
```

Output lands in `missile_ert_rtw/`. The five files CLEARANCE needs
are `missile.c`, `missile.h`, `missile_types.h`, `missile_private.h`,
`rtwtypes.h` plus the two Simulink runtime headers
(`rtw_continuous.h`, `rtw_solver.h`) that the .h references.

## Verification

The regression check in `tools/compare_sim.m` loads
`ci_artifacts/simOut_missile_defaults.mat` and asserts missile
position, velocity, and termination flag from a fresh sim match the
reference within tolerance. Fails CI on any drift.

`tools/verify_prop_nav.m` runs the known-solution
`lateral_crossing` scene and asserts intercept was declared, miss
distance is below `R_LETHAL`, and time-to-intercept is inside the
analytic ballpark.

The traceability report (`traceability_report.html`) maps every
tagged requirement in `req_map.csv` to a specific block. Simulink
Requirements Toolbox regenerates it via `slreq.generateReport`.

The integration test is CLEARANCE itself: fire a missile at a
maneuvering aircraft, watch the guidance close the miss distance to
below `R_LETHAL`, observe the Detonation PDU emit on intercept.

## Integration with CLEARANCE

The plugin module in CLEARANCE is `ClearanceMissileMBD`. Files:

- `MissileWrapper.h/.cpp` - thin C++ wrapper around the extern-C
  entry points. Owns one `RT_MODEL_missile_T` per instance. Called
  from `AClearanceMissile::Tick()`.
- `MissileGeneratedUnit.cpp` - compilation shim. Single TU includes
  `missile.c` under `extern "C"` so Unreal Build Tool compiles the
  generated code as part of the module without needing a per-file
  compilation rule.
- `ClearanceMissileMBD.Build.cs` - detects the presence of
  `ThirdParty/MissileGenerated/include/` and `src/`, flips
  `CLEARANCE_MISSILE_MBD_HAVE_CODEGEN=1`, and adds the include
  paths.

### Federation wiring

The federation stack already carries Fire (§7.4.3) and Detonation
(§7.4.4) PDUs, so this integration turns those PDU types from
theoretical (defined but unused) to load-bearing:

- **Launch** emits Fire PDU: firing entity (launcher callsign), target
  entity (target callsign), munition type, munition entity ID, initial
  location, initial velocity.
- **Intercept** (or timeout / LOS reversal) emits Detonation PDU:
  munition entity, target entity, detonation location, detonation
  result code (`Entity Impact`, `Ground Impact`, `Detonation`, `None`
  depending on `term_flag`).

Both PDUs go out on all four federation wires (DIS, Fast DDS, RTI
Connext, HLA) so any subscribing federate sees the weapon engagement.

Both PDUs on DIS also stamp the munition with the SISO-REF-010
AIM-120B entity type (`Kind 2 Munition, Domain 3 Anti-Air`) and the
`MunitionEntity` field matches the flying missile's own EntityState
PDU ID so a federation observer can correlate the paired traffic.

Console commands in-sim:

```
clearance.missile.fire  <target_callsign>   # SAM launched at aircraft
clearance.missile.abort                     # destroys every in-flight
clearance.missile.test                      # offline wrapper smoke test
```

The launcher is selected automatically from any placed
`AClearanceMissileLauncher` actor in the level (fallback: 20 km
behind target with a warning log).

## What's not in here

Deliberate omissions, as with the autopilot and radar companions:

- **No 6-DOF airframe.** Guidance kernel plus point-mass kinematics
  only.
- **No seeker.** Target state is assumed perfectly observed. Seeker
  glint, radome error, and angle-only tracking would live in a
  downstream layer.
- **No warhead / fuze.** Intercept is a proximity threshold.
- **No boost / midcourse / terminal phasing.** Single-phase TPN
  throughout.
- **No wind or atmospheric effects.**

Each omission is called out in `Requirements.md` too, so the scope
stays honest.

## Files

```
missile.slx                 -- source of truth
model/missile_params.m      -- N, LATAX_MAX, V_M0, R_LETHAL, T_MAX, solver settings
model/target_scene.m        -- reference target trajectory generator
tools/
  verify_prop_nav.m         -- T1 probe on known-solution intercept
  compare_sim.m             -- T2 regression baseline check
  configure_reusable_function.m -- switches code interface to reusable
  run_model_tests_and_build.m   -- CI entry
ci_artifacts/               -- reference sim outputs
req_map.csv                 -- requirement -> block ID map
traceability_report.csv/html -- rendered traceability
```
