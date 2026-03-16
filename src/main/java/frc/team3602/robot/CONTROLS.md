# Controls

## Purpose

This file explains the current controller bindings and driver/operator
behavior.

Use this document when you want to answer questions like:

- What does each button do?
- Which controls are protected from overlapping shot commands?
- Which control is the backup shot?
- Which autonomous named commands exist right now?

## Driver Controller 0

### Driving

- Left stick
  - translation

- Right stick X
  - rotation

- Right trigger
  - turbo drive while held

### Intake / Pivot

- Right bumper
  - drop intake path

- Left bumper
  - run intake while held

### Climber

- POV up / down
  - climber only if `ClimberConstants.kClimberEnabled` is `true`

## Operator Controller 1

### Turret

- Right trigger
  - track the current alliance tower while held

- POV up
  - turret rear preset

- POV down
  - turret right-side preset

- POV left
  - turret left-corner preset

- POV right
  - turret right-corner preset

### Shooting

- `B`
  - full tracked lerp shot while held

Behavior while `B` is held:

- turret tracks the alliance tower
- shooter follows the current lerp-table target
- feed waits until the shot is ready
- driver controller 0 rumbles when shot becomes ready
- overlapping manual operator shot controls are gated so they do not interrupt
  the tracked shot

Other operator shot controls:

- `A`
  - fixed shooter speed test at `-41.5`

- `X`
  - fixed shooter speed test at `-44.0`

- `Y`
  - manual spindexer feed while held

- Left trigger
  - failsafe shot

Failsafe shot behavior:

- turret = `180.0` degrees
- shooter = `-41.5` RPS
- after spin-up delay, spindexer and transfer feed at `-62.5` RPS

### Intake / Cleanup

- Left bumper
  - stop intake and raise/stow intake path

## Shot Modes

### Tracked Shot

Best use:

- normal scoring when tracking and shooter interpolation are working well

What it does:

- tower tracking
- lerp-table shooter speed
- readiness-gated feed
- rumble cue when ready

### Failsafe Shot

Best use:

- backup shot
- simple pit test
- recovery when tracking is not trusted

What it does:

- fixed turret direction
- fixed shooter speed
- fixed delayed feed

## Control Protection

While operator `B` is active:

- manual shooter preset buttons are gated
- turret preset buttons are gated
- the tracked shot keeps ownership of the scoring flow

This is intentional so overlapping commands do not fight each other.

## Autonomous Named Commands

Current registered PathPlanner named commands include:

- `autonShootTower`
- `autonLowerIntake`
- `autonRunIntake`
- `autonRaiseIntake`
- `autonStartIntake`
- `autonStopIntake`
- `autonStartShooter`
- `autonWaitForShooterReady`
- `autonPrepareTowerShot`
- `autonFireShot`
- `autonStopShooter`
- `autonAimTower`
- `autonAimTowerShort`
- `autonStowTurret`

## Recommended Driver/Operator Mental Model

Driver owns:

- robot movement
- intake enable
- fast positioning

Operator owns:

- turret behavior
- shot mode selection
- feed behavior
- intake cleanup/stow

## Common Questions

### Why does `B` feel different from manual controls?

Because `B` is a full superstructure shot command, not just one motor command.

### Why do some buttons stop working while `B` is held?

Because those controls are intentionally gated to prevent conflicting shooter
and turret commands.

### Why does left trigger point the turret backward?

Because that is the team's current simple failsafe shot.

### Why do turret angles use `180` for rear?

Because the turret public API now follows WPILib's signed robot-relative angle
convention.
