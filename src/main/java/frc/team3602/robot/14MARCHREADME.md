# 14 March Robot Update

## Overview

This file is a short summary of the code changes that were made around March 14.
The goal of these updates was to get the robot to a more usable MVP state for
driving, aiming, shooting, and autonomous behavior.

The biggest themes were:

- make Limelight pose data safer and more useful
- make the turret share the same pose estimate as the drivetrain
- add turret point tracking for teleop and auton
- clean up shooter command behavior so auton routines actually advance
- disable the climber safely when the hardware is not installed

## Major Changes

### 1. Limelight / MegaTag Pose Reliability

- Vision measurements now feed one shared drivetrain pose estimate.
- Auto, drivetrain logic, and turret math now use the same pose source.
- Vision frames are only accepted once instead of being reused repeatedly.
- Old or stale frames are rejected.
- Very large pose jumps are rejected to protect against bad tag solves.
- Measurement trust now changes based on robot motion.
- Camera selection uses simple hysteresis so it does not rapidly flap between
  left and right cameras.
- More dashboard values were added so we can see why a frame was accepted or
  rejected.

Useful dashboard values include:

- `Vision/Selected/PreferredCamera`
- `Vision/Left/MeasurementAgeSeconds`
- `Vision/Right/MeasurementAgeSeconds`
- `Vision/Left/TranslationErrorMeters`
- `Vision/Right/TranslationErrorMeters`

### 2. Turret Improvements

- The turret now uses the shared drivetrain pose for aiming math.
- The robot no longer has a dedicated turret Limelight.
- Turret aiming is now fully pose-based instead of mixing in a separate
  turret-camera correction path.
- The turret can track the current alliance tower target.
- Blue tower target uses `(2.03, 4.035)` meters.
- Red tower target uses `(14.51, 4.035)` meters.
- The current moving-shot lead math assumes a `70` degree launch angle and a
  `20` inch shooter release height.
- Operator controller right trigger uses this field-point tracking in teleop.
- A reusable autonomous turret tracking command was added.
- A short timeout version of that auton command was also added.
- Turret angle requests now use a linear `0` through `360` travel model instead
  of the older wrapped-angle model.
- In that travel model, `0` / `360` mean rear, `90` means left, `180` means
  forward over the intake, and `270` means right.
- The software now treats `0` and `360` as separate travel endpoints so the
  turret goes around the rear seam instead of taking a fake shortest-angle
  shortcut.
- Preset turret positions were cleaned up so right corner and neutral are now
  distinct legal positions.
- The turret hardware config now runs during subsystem startup instead of being
  left unused.
- Driver `Y` now acts like a real hold-to-aim command.

### 3. Shooter Improvements

- Shooter velocity commands now set a persistent target once instead of acting
  like never-ending sequence steps.
- This fixes command sequences that used to stall in teleop and autonomous.
- The interpolation table endpoint was corrected so long-distance shots do not
  collapse to almost zero speed.
- Shooter motor 2 now follows motor 1 through one consistent control path.
- Dashboard output now reports `Shooter Lerp Speed` instead of an unrelated
  number.
- Operator `B` can now run one combined tracked lerp shot instead of splitting
  aiming, flywheel spin-up, and feed across multiple buttons.

### 4. Autonomous / Command Flow Fixes

- `shootFailsafe()` now advances correctly instead of getting stuck on the
  shooter spin-up step.
- `autonRunBetaShot()` now advances correctly instead of hanging on the initial
  shooter command.
- `shootBall1()` now spins up, waits, and then feeds in a more readable way.

### 5. Climber Safety Change

- The climber code was not deleted.
- Instead, the climber is controlled by `ClimberConstants.kClimberEnabled`.
- It is currently set to `false` because the climber hardware / CAN ID is not
  in use right now.
- When disabled, the robot does not create or command the climber motor.
- Driver climber bindings and climber default commands are also skipped when the
  climber is disabled.

## Teleop Button Bindings

### Driver Controller 0

- Left stick: translate the swerve drivetrain
- Right stick X: rotate the drivetrain
- Right trigger: turbo speed while held, normal speed on release
- Left bumper: run intake while held
- Right bumper: drop intake while held
- POV up: raise climber, only if climber is enabled
- POV down: lower climber, only if climber is enabled

### Operator Controller 1

- Right trigger: hold to track the current alliance tower with the turret
- Left trigger: run the timed failsafe shooting routine, release early to stop
  shooter and feed
- `Y`: run spindexer feed at `-35.0` while held
- Right bumper: run shooter at the current lerp-table speed, release to stop
- `B`: hold for the full tracked lerp shot
- `A`: set shooter velocity to `-41.5`, release to stop
- `X`: set shooter velocity to `-44.0`, release to stop
- Left bumper: stop intake / raise pivot through the superstructure stop command
- Holding `B` should not block the normal intake / pivot controls
- POV up: turret to zero preset
- POV down: turret to neutral preset
- POV left: turret to left-corner preset
- POV right: turret to right-corner preset

## Default Commands

- Drivetrain: field-centric drive from driver controller sticks
- Pivot: hold pivot position
- Turret: hold current turret setpoint
- Climber: hold climber position only when climber is enabled

## Named Autonomous Commands

These are registered for PathPlanner:

- `autonRunBetaShot`
  - Runs the full simple beta shot macro: prepare, feed, then stop.

- `autonLowerIntake`
  - Drops the intake pivot for autonomous intake setup.

- `autonStartIntake`
  - Starts the intake roller for autonomous collection steps.

- `autonStopIntake`
  - Stops the intake roller without moving the pivot.

- `autonSpinUpBetaShot`
  - Starts the shooter for the beta auto shot without feeding yet.

- `autonWaitForBetaShotReady`
  - Waits for the shooter to reach the current beta ready threshold, with a
    timeout so the auto cannot hang forever.

- `autonPrepareBetaShot`
  - Spins up the shooter and tracks the alliance tower at the same time.
  - This is the main setup step for the current basic scoring auto.

- `autonFeedBetaShot`
  - Runs the spindexer / transfer feed for the beta shot for a fixed window,
    then stops the spindexer.

- `autonStopShooting`
  - Stops the shooter and spindexer after an autonomous shot.

- `autonTrackTower`
  - Continuously tracks the current alliance tower with the turret.
  - This is best used in parallel with path following or another auton step.

- `autonTrackTowerShort`
  - Tracks the current alliance tower for `2.0` seconds, then ends.
  - This is the easier version to use in a normal sequential auton.

- `moveTurretToStartAngle`
  - Sends the turret to the shared starting / handoff angle.

For the full explanation of how these auton commands are meant to be combined in
PathPlanner, see `autonREADME.md`.

## Important Notes For The Team

- The climber is intentionally disabled right now.
- The Limelight system is safer than before, but it still needs on-robot testing
  and tuning with real tags.
- There is no turret-mounted camera in the current robot configuration.
- Turret tracking now depends on the shared drivetrain pose estimate from the
  left and right Limelight pose pipeline.
- The turret now uses a `0` through `360` travel model, with `90` degrees as
  the staged startup position.
- The current intended direction map is: rear = `0` / `360`, left = `90`,
  front = `180`, and right = `270`.
- The software should never shortcut across the `0` / `360` seam. If the goal
  is on the other side of that seam, the turret should rotate the long way
  around instead.
- The current ballistic estimate in turret lead math assumes a `70` degree
  launch angle and a `20` inch shooter height until more on-robot tuning is
  completed.
- The shooter preset values and interpolation table should still be tuned with
  real practice shots.

## Testing Checklist

Use this checklist when the robot is on blocks first, then again on the carpet.

### General Startup

- Robot code deploys and enables with no unexpected CAN errors
- Driver Station shows the robot in the correct mode
- SmartDashboard / Shuffleboard values update normally
- No subsystem moves unexpectedly when the robot first enables

### Drivetrain

- Left stick drives translation normally
- Right stick X rotates the robot normally
- Right trigger increases drive speed while held
- Releasing right trigger returns the robot to normal speed
- Robot remains driveable immediately after enable

### Intake / Pivot

- Driver right bumper drops the intake
- Driver left bumper runs intake while held
- Releasing left bumper stops intake
- Operator left bumper stops intake and returns pivot through the stop command
- Pivot holds position when no button is pressed
- While operator `B` is held for the tracked shot, these intake / pivot controls
  should still work normally

### Turret Basic Function

- Turret holds its position when idle
- Operator POV up sends turret to zero preset
- Operator POV down sends turret to neutral preset
- Operator POV left sends turret to left-corner preset
- Operator POV right sends turret to right-corner preset
- Right-corner and neutral are visibly different positions
- When moving near the rear seam, the turret does not "wrap" across `0` / `360`

### Turret Field-Point Tracking

- Operator right trigger makes the turret track the current alliance tower
- As the robot rotates, the turret updates to keep aiming at that point
- If the target direction crosses the rear seam, the turret rotates the long
  way around instead of trying to wrap across `0` / `360`
- Releasing operator right trigger returns control to the normal turret hold
  behavior

### Shooter

- Operator right bumper spins the shooter to the current lerp-table speed
- Operator `B` runs the full tracked lerp shot while held
- Driver controller 0 should rumble when the tracked lerp shot becomes ready
- Operator `A` spins the shooter to `-41.5`
- Operator `X` spins the shooter to `-44.0`
- Releasing each shooter button stops the shooter
- Shooter motors sound synchronized and stable
- Dashboard `Shooter Lerp Speed` value updates when robot distance changes

### Spindexer / Feed

- Operator `Y` runs the spindexer feed while held
- Releasing operator `Y` stops the spindexer
- Operator `B` should wait to feed until the turret and shooter are ready, then
  start feeding automatically while still held
- When that tracked shot becomes ready, driver controller 0 should vibrate as a
  cue that the robot is ready to fire
- `shootFailsafe()` feeds only after the shooter spin-up delay and now ends on
  its own after the fixed feed window

### Vision / Limelight

- `Vision/Selected/PreferredCamera` changes to a real camera name when tags are
  visible
- `Vision/Left/MeasurementAgeSeconds` and
  `Vision/Right/MeasurementAgeSeconds` stay reasonable and do not grow wildly
  while tags are visible
- `Vision/Left/TranslationErrorMeters` and
  `Vision/Right/TranslationErrorMeters` stay reasonable during normal driving
- Robot pose on the dashboard changes smoothly instead of snapping repeatedly
- Covering one camera still allows the other camera to be selected when valid
- Turret aiming continues to work from field pose even though there is no
  turret-mounted Limelight

### Autonomous Commands

- `autonRunBetaShot` spins up, waits for speed, feeds, and then stops as expected
- `autonLowerIntake` drops the intake pivot
- `autonStartIntake` starts the intake roller
- `autonStopIntake` stops the intake roller
- `autonSpinUpBetaShot` starts the shooter without feeding
- `autonWaitForBetaShotReady` ends when the shooter is near speed or the timeout expires
- `autonPrepareBetaShot` spins up and tracks before feeding
- `autonFeedBetaShot` runs the feed window and then stops the spindexer
- `autonStopShooting` stops both shooter and spindexer
- `autonTrackTower` works when run in parallel with movement
- `autonTrackTowerShort` runs for about 2 seconds and then ends
- `moveTurretToStartAngle` sends turret to the shared starting / handoff angle

### Climber Disabled State

- No climber CAN errors appear with the climber disabled
- Driver POV up/down do nothing for the climber while disabled
- Robot enables normally without climber hardware installed

### Final MVP Check

- Robot can drive, intake, aim, and shoot without command lockups
- Turret and drivetrain can be used at the same time without fighting each other
- Basic auton commands complete instead of stalling
- Vision improves pose instead of making the robot obviously less stable

## Current Status

- `./gradlew.bat compileJava` passes
- The main remaining build warnings are CTRE deprecation warnings for
  `TalonFX(int, String)`
