# Integration with CLEARANCE

Runtime integration counterpart to
[MISSILE_MBD_DESIGN.md](MISSILE_MBD_DESIGN.md). The design brief
covers the guidance law and the kinematic 3-DOF model. This document
covers what ships in CLEARANCE v1.0, what does not, and why.

The honest headline: **the generated guidance model is integrated
through the standard wrapper pattern shared with the autopilot and
radar bindings, but CLEARANCE v1.0's live SAM engagement path uses a
C++ pursuit fallback rather than driving the missile through the
generated model.** This document explains that gap and how the
integration will complete.

## Contents

- [v1.0 shipping status](#v10-shipping-status)
- [What ships](#what-ships)
- [The gap](#the-gap)
- [Why not delay v1.0 for the fix](#why-not-delay-v10-for-the-fix)
- [Production fix, scoped](#production-fix-scoped)
- [Generated code shape](#generated-code-shape)
- [The wrapper](#the-wrapper)
- [Where the boundaries will sit](#where-the-boundaries-will-sit)
- [What the live pursuit fallback does](#what-the-live-pursuit-fallback-does)
- [Federation impact](#federation-impact)

## v1.0 shipping status

- Simulink model authored: **yes**.
- Simulink model verified against authored intercept geometries:
  **yes**.
- Embedded Coder output generated: **yes**.
- Generated code integrated through the standard CLEARANCE wrapper
  pattern: **yes**.
- Generated code driving live in-sim missiles: **no**.

The live-drive gap is a single specific issue with how launch state
is parameterised in the model, not an architectural problem with the
integration. Everything upstream and downstream of the model is
production-ready.

## What ships

- SAM launcher actor placed at the Warton scenario airfield.
- Instructor ENGAGE control on the operator instructor panel wired
  through `Server_InjectFireMissile(FName TargetCallsign)` on
  `AClearanceOperatorPC`.
- Missile actor spawn, in-flight update, and termination through
  the full CLEARANCE lifecycle.
- Target aircraft state transitions on intercept: state goes to
  `Destroyed`, mayday descent triggers, crash pipeline fires, DIS
  events emitted.
- **DIS Fire PDU** (§7.4.3) at launch, carrying paired firing /
  target / munition entity IDs, event number, and SISO-REF-010
  burst descriptor (AIM-120B family under UK country code).
- **DIS Detonation PDU** (§7.4.4) at intercept, carrying matching
  IDs and event number so a federation observer can pair the two
  purely by wire content.

The federation Detonation and Fire PDU wire evidence is captured in
the [clearance-federation](https://github.com/abdullahabduljabbarab/clearance-federation)
repository's README (Figures 4a and 4b).

## The gap

The generated Simulink guidance model requires the launch state
velocity as input to its LOS-rate initialisation. The model was
authored against a specific verification geometry (launch at a fixed
point in the inertial frame, target on a fixed initial vector), and
the initial velocity is currently baked into the model rather than
exposed as a runtime input.

A CLEARANCE launch from an arbitrary launcher position at an
arbitrary target does not match the authored geometry. Feeding the
generated model an arbitrary launch state produces a first-few-steps
trajectory that departs from the intercept manifold in a way the
saturation cannot recover from before the target is out of the
engagement envelope.

The fallback path is a straight-line pursuit implemented in C++ on
the missile actor. It always points the missile at the target's
current position with a fixed missile speed, saturates lateral
acceleration to the same envelope the model uses, and terminates on
the same lethal-radius / timeout / LOS-reversal rules.

## Why not delay v1.0 for the fix

Because v1.0's purpose is to ship the whole system for portfolio
review. Every part of CLEARANCE that depends on missiles happening
(the DIS Fire and Detonation PDUs on the wire, the mayday descent
pipeline, the operator ENGAGE control, the target aircraft
destruction, the scoring on intercepts) works cleanly with the
pursuit fallback. The generated model's live-drive gap does not
affect any of those.

Delaying the release for a scoped fix on one subsystem when
everything else is ready would trade a shipped v1.0 with a
documented limitation for an unshipped project. Documenting the gap
honestly is more useful to a reviewer than hiding it.

## Production fix, scoped

The fix has three parts:

1. **Expose launch state velocity as a runtime input to the model.**
   Add a `v_M_init` inport, rewire the LOS-rate initialisation to
   consume it.
2. **Regenerate.** Standard Embedded Coder pass with reusable
   function packaging.
3. **Wire through the wrapper.** `FMissileWrapper` gains a
   `SetLaunchState(v_M_init)` method called once at spawn from the
   missile actor.

Nothing else changes. The DSP pipeline stays identical, the
saturation stays identical, the termination stays identical, the
wire event PDUs stay identical.

Scoped as a v1.1 patch. Tracked in the parent CLEARANCE repo's
release history.

## Generated code shape

Embedded Coder produces portable ANSI C under
`ThirdParty/MissileGenerated/{include,src}` in the CLEARANCE plugin:

```
ThirdParty/MissileGenerated/
  include/
    MissileSubsystem.h                generated public header
    MissileSubsystem_types.h          typedefs, enums
    rtwtypes.h                        Embedded Coder core types
  src/
    MissileSubsystem.c                generated step function
    MissileSubsystem_data.c           generated constants
```

Two symbols matter:

- `RT_MODEL_MissileSubsystem_T` — per-instance model handle.
- `MissileSubsystem_step(handle*, ExtU*, ExtY*)` — the entry point.

Reusable Function packaging keeps every in-flight missile's state
private to its own handle.

## The wrapper

`ClearanceMissileMBD` is a plugin module at
`Plugins/ClearanceSim/Source/ClearanceMissileMBD/`. Public API on
`FMissileWrapper`:

- `Initialize()` allocates the handle, runs generated init.
- `Terminate()` runs generated termination, releases the handle.
- `SetTargetState(r_T, v_T)` writes target position and velocity.
- `SetMissileState(r_M, v_M)` writes missile kinematic state.
- `SetLaunchState(v_M_init)` writes the initial launch velocity
  (once, at spawn; consumed by the fixed model in v1.1).
- `Step()` calls `MissileSubsystem_step` and reads the output.
- `GetMissileState()` returns the updated missile state.
- `GetTerminationFlag()` returns 0 (in-flight), 1 (intercept),
  2 (timeout), or 3 (LOS reversal).

The wrapper is production-ready and links cleanly. `Build.cs`
auto-detects the generated code directory and skips the module link
if absent so CLEARANCE builds cleanly on developer boxes without the
generated code present.

## Where the boundaries will sit

When the v1.1 fix lands:

```
CLEARANCE::AClearanceOperatorPC::Server_InjectFireMissile
        |
        v
CLEARANCE::AClearanceSAMLauncher::LaunchMissile
        |
        v (spawn AClearanceMissile at launcher position)
CLEARANCE::AClearanceMissile::BeginPlay
        |
        v (Wrapper->SetLaunchState with initial velocity)
CLEARANCE::AClearanceMissile::Tick (every frame)
        |
        v (Wrapper->SetTargetState + SetMissileState)
CLEARANCE::FMissileWrapper::Step
        |
        v
GENERATED::MissileSubsystem_step (TPN + saturation + 3-DOF integrator)
        |
        v
CLEARANCE::AClearanceMissile (read new state, check termination flag)
        |
        v
Intercept   =>  DIS Detonation PDU + target destruction pipeline
Timeout     =>  quiet removal
LOS reversal => quiet removal
```

Currently the entire block from `AClearanceMissile::Tick` down to
`GetTerminationFlag` runs against the C++ pursuit fallback rather
than the generated model. All the CLEARANCE-side wiring, the wire
events, and the intercept effects are identical between the two
paths.

## What the live pursuit fallback does

Straight-line pursuit with saturation and the same termination
rules the generated model uses:

- Every tick, compute `r = r_T - r_M`.
- Set commanded velocity direction to `r / |r|`.
- Set commanded velocity magnitude to a fixed missile speed
  (representative short-range interceptor value).
- Saturate lateral acceleration change per tick to the same
  `LATAX_MAX` envelope the model uses.
- Integrate to new missile position.
- Terminate on lethal-radius proximity (intercept), engagement
  horizon timeout, or LOS reversal.

Against a non-manoeuvring target with a reasonable engagement
geometry, pursuit intercepts. Against an aggressively manoeuvring
target it can miss where TPN would not, but the CLEARANCE scenarios
that use SAM engagement are constrained enough that the fallback
produces the intended operator experience for the v1.0 release.

## Federation impact

None. The DIS Fire (§7.4.3) and Detonation (§7.4.4) PDUs emitted at
launch and at termination are identical whether the guidance path is
the C++ fallback or the generated model. The wire evidence in the
clearance-federation repository (Figures 4a and 4b) applies to both.

## Related material

- [MISSILE_MBD_DESIGN.md](MISSILE_MBD_DESIGN.md) — the model
  itself.
- [DEVLOG.md](../DEVLOG.md) — chronological record.
- [Requirements.md](../Requirements.md) — REQ IDs.
- [clearance-federation](https://github.com/abdullahabduljabbarab/clearance-federation)
  — DIS Fire and Detonation PDU wire evidence.
- CLEARANCE
  [CHANGELOG.md](https://github.com/abdullahabduljabbarab/CLEARANCE/blob/main/CHANGELOG.md)
  — release history including the v1.0 SAM engagement scope.
