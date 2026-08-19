# Development log

Chronological engineering journal for the missile guidance Simulink
model and its integration status with CLEARANCE. Most recent first.

Companion material:

- [docs/MISSILE_MBD_DESIGN.md](docs/MISSILE_MBD_DESIGN.md) — design
  brief for the guidance law and kinematic 3-DOF.
- [docs/INTEGRATION.md](docs/INTEGRATION.md) — integration status
  inside CLEARANCE, including the honest v1.0 live-guidance gap.
- [Requirements.md](Requirements.md) — requirements this model
  satisfies.

---

## 2026-08

### 2026-08-11 — v1.0 shipping status recorded

CLEARANCE ships v1.0 with the SAM launcher wired end to end for the
operator (select hostile track, ENGAGE, launcher fires, missile
flies, target dies through the standard mayday-descent + crash
pipeline, and DIS Fire and Detonation PDUs go out on the wire as
they should).

The v1.0 live guidance path uses a C++ pursuit fallback rather than
the generated Simulink guidance kernel. The generated code is
integrated through the standard wrapper pattern shared with the
autopilot and radar bindings, and the model itself runs cleanly on
authored verification geometry. The reason it does not drive live in
v1.0 is that the initial missile velocity remains coupled to the
authored verification scenario, so a launch from an arbitrary
CLEARANCE launcher position with an arbitrary target does not use
the generated model as its inner loop.

The production fix is known and scoped: expose launch state velocity
as a runtime input to the model, regenerate. Left as a documented
limitation rather than hidden. See
[docs/INTEGRATION.md](docs/INTEGRATION.md).

## 2026-07

### 2026-07 — Reusable function packaging

Regenerated with **Reusable Function** packaging so a salvo of
missiles in flight can run the same generated model concurrently
without shared globals. Every in-flight missile allocates one
`RT_MODEL_MissileSubsystem_T` handle carrying its own kinematic
state, LOS memory, and termination logic.

### 2026-07 — Wrapper pattern shared with autopilot and radar

`ClearanceMissileMBD` follows the same wrapper pattern as
`ClearanceAutopilotMBD` and `ClearanceRadarMBD`:

- Plugin module under
  `Plugins/ClearanceSim/Source/ClearanceMissileMBD/`.
- Public API on `FMissileWrapper`.
- Generated code under
  `ThirdParty/MissileGenerated/{include,src}` in the plugin,
  auto-detected by the module's `Build.cs`.
- If the generated code directory is absent, the module link is
  skipped and CLEARANCE falls back to the built-in pursuit path.

The wrapper compiles cleanly. Live use is gated on the launch state
issue described above.

### 2026-07 — MATLAB Function block implementation

Guidance kernel, saturation, kinematic 3-DOF integrator, and
termination logic all authored as MATLAB Function blocks against the
analytic equations. No Simulink library blocks from control system,
aerospace, or navigation toolboxes are used.

Effect on the generated code: pure ANSI C with zero external toolbox
library dependencies. Same integration discipline as the autopilot
and radar bindings.

### 2026-07 — Termination conditions

Three termination flags on `term_flag_out`:

| Flag | Condition | Semantics |
|---|---|---|
| 1 | Range below `LETHAL_RADIUS` | Intercept |
| 2 | `t >= T_MAX` | Timeout (target escaped or missile out of energy) |
| 3 | LOS reversal (Vc negative for consecutive steps) | Target passed |

CLEARANCE consumes the flag: intercept fires the detonation event
path (DIS Detonation PDU, target aircraft state change to `Destroyed`,
mayday descent + crash pipeline); timeout and LOS reversal both fire
a graceful termination that removes the missile without a detonation
event.

## 2026-06

### 2026-06 — Guidance law: True Proportional Navigation

TPN chosen over pursuit and lead pursuit for the standard reasons:

- **Pursuit** always points at current target position. Against a
  crossing target it commands excessive lateral acceleration late in
  flight and misses.
- **Lead pursuit** holds a fixed lead angle. Works for a
  constant-velocity target but degrades against a maneuvering
  target.
- **TPN** commands acceleration proportional to LOS rate scaled by
  closing velocity, driving LOS rate to zero. Optimal against a
  non-maneuvering target in the sense of minimum control effort per
  Zarchan (2019).

```
omega_LOS = (r x r_dot) / |r|^2         inertial LOS angular rate
Vc        = -d|r|/dt                    closing velocity
a_cmd     = N * Vc * omega_LOS          commanded acceleration
```

`N = 4` by default. Tactical intercepts typically use `N` in
`[3, 5]`; below 3 the missile lags a manoeuvring target, above 5 the
airframe saturates on lateral acceleration demand.

The cross-product formulation for `omega_LOS` is numerically stable
near boresight (small angle) where the trigonometric form is
ill-conditioned. Closing velocity is signed so the guidance law
naturally disengages once the target has passed.

### 2026-06 — Saturation preserves direction

Commanded acceleration is limited to `LATAX_MAX = 40g` (~392 m/s²).
Saturation preserves direction:

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

### 2026-06 — Kinematic 3-DOF integrator

Point-mass integration in the inertial frame:

```
v_M_dot = a_cmd_sat
r_M_dot = v_M
```

No airframe, no aero forces, no thrust decay. This is a guidance
model, not a full missile simulation. A real airframe would sit
between the guidance kernel and the integrator; for a portfolio
guidance model the point-mass integrator keeps the emphasis on the
guidance law itself.

## 2026-05

### 2026-05 — Requirements captured

REQ IDs in [Requirements.md](Requirements.md) covering:

- Miss distance below the lethal radius against a non-manoeuvring
  target at specified engagement geometries.
- Miss distance envelope against a manoeuvring target at bounded
  target g.
- Saturation behaviour under aggressive geometries.
- Termination flag semantics.
- Reusable function packaging discipline.
- Zero external toolbox runtime dependency in the generated code.
