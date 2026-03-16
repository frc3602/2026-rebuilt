# Master README

## Purpose

This is the main reference file for the current robot code.

Students, mentors, and CSAs should start here when they want to answer these
questions quickly:

- What does the robot do right now?
- Which controller runs which subsystem?
- Which document should I open next?
- What are the biggest current limitations or watch-outs?

This file is meant to stay current. It replaces the older dated status note that
used to live here.

## Document Map

Use these files together:

- `MasterREADME.md`
  - Current high-level robot overview
  - Control map
  - Important conventions
  - Common starting points for debugging

- `deploymentREADME.md`
  - What to check right after deploying code
  - Safe bring-up order
  - Step-by-step on-robot verification

- `tuningREADME.md`
  - Commissioning order
  - Turret troubleshooting
  - Dashboard values to watch while tuning

- `autonREADME.md`
  - PathPlanner named commands
  - How autonomous commands are structured
  - When to use small commands versus macros

## Robot Overview

The current robot software is organized around a few important subsystems:

- `CommandSwerveDrivetrain`
  - Field-centric swerve drive
  - Shared pose estimate used by driving, vision, and turret math

- `Limelight_Pose`
  - Accepts Limelight pose measurements
  - Chooses safe vision updates before feeding the drivetrain estimator

- `TurretSubsystem`
  - Holds fixed turret presets
  - Tracks the current alliance tower from field pose
  - Uses a public WPILib-style angle convention for readability

- `ShooterSubsystem`
  - Runs fixed shooter speeds
  - Supports lerp-table based tracked shooting

- `SpindexerSubsystem`
  - Feeds fuel into the shooter

- `PivotSubsystem` and `IntakeSubsystem`
  - Control the intake path and fuel collection flow

- `Superstructure`
  - Combines subsystem actions into reusable teleop and autonomous commands

## Where To Start In Code

If you are new to the project, these are the best first files to open:

- `RobotContainer.java`
  - Button bindings
  - Default commands
  - PathPlanner named-command registration

- `Superstructure.java`
  - Multi-step shot logic
  - Reusable autonomous helper commands

- `TurretSubsystem.java`
  - Turret presets
  - Angle conversion
  - Field-point tracking
  - Turret telemetry

- `Limelight_Pose.java`
  - Vision filtering and acceptance logic

## Current Control Map

### Driver Controller 0

- Left stick
  - Drive translation

- Right stick X
  - Rotate the robot

- Right trigger
  - Temporary turbo drive mode while held

- Right bumper
  - Drop the intake/pivot path

- Left bumper
  - Run intake while held

- POV up / down
  - Climber controls only when `ClimberConstants.kClimberEnabled` is `true`

### Operator Controller 1

- Right trigger
  - Track the current alliance tower with the turret while held

- `B`
  - Full tracked lerp shot while held
  - Tracks the target, updates shooter speed, and feeds automatically when ready

- `A`
  - Fixed shooter speed test at `-41.5`

- `X`
  - Fixed shooter speed test at `-44.0`

- `Y`
  - Manual spindexer feed while held

- Left trigger
  - Failsafe shot with fixed turret angle and fixed shooter speed

- Left bumper
  - Stop intake and raise/stow the intake path

- POV up
  - Turret rear preset

- POV down
  - Turret right-side preset

- POV left
  - Turret left-corner preset

- POV right
  - Turret right-corner preset

## Turret Angle Convention

Public turret commands now use the same signed convention recommended by
WPILib:

- `0` = forward
- `+90` = left
- `-90` = right
- `180` = rear

This is the angle convention students should use when reading logs, debugging,
and calling public turret methods.

Important detail:

- The turret still keeps a private `0` through `360` travel-angle model
  internally.
- That private model exists only so the robot can remember which side of the
  rear seam it is on and avoid fake shortest-path wraparound at the back of the
  robot.

## Current Teleop Shot Modes

There are two main teleop shooting styles right now:

### 1. Tracked Shot

Use operator `B` when you want the robot to handle most of the scoring flow.

While `B` is held:

- turret tracks the alliance tower
- shooter follows the interpolation table
- feed waits until the shot is ready
- driver controller 0 rumbles when the shot becomes ready

### 2. Failsafe Shot

Use operator left trigger when the team wants a simple backup shot.

Current failsafe behavior:

- turret command = `180.0` degrees
- shooter speed = `ShooterConstants.kShooterFailsafeSpeed`

Because the public turret convention is signed, `180.0` means straight to the
rear of the robot.

## Dashboard Values Worth Watching

These values are especially useful during pit testing and turret debugging:

- `Turret/MeasuredAimDegrees`
- `Turret/RequestedAimDegrees`
- `Turret/AimErrorDegrees`
- `Turret/MeasuredTravelDegrees`
- `Turret/RequestedTravelDegrees`
- `Turret/AtRequestedAngle`
- `Turret/MotorVoltage`

Useful shared pose values:

- `estimated drive pose x`
- `estimated drive pose y`
- `estimated drive pose rotation`

Useful vision values:

- `Vision/Selected/PreferredCamera`
- `Vision/Left/MeasurementAgeSeconds`
- `Vision/Right/MeasurementAgeSeconds`

For the full tuning workflow and symptom-based turret troubleshooting, use
`tuningREADME.md`.

## Startup And Commissioning Notes

The two biggest current pit reminders are:

- The turret and pivot still assume known staged startup positions.
  - There is not yet an absolute sensor or homing routine for those mechanisms.
  - If the robot reboots or browns out while they are not in the expected place,
    the software zero can be wrong until the mechanism is restaged.

- The climber is currently guarded by `ClimberConstants.kClimberEnabled`.
  - If climber hardware is not installed, keep that constant disabled so the
    robot does not try to create or command unused climber hardware.

Use `deploymentREADME.md` for the full safe bring-up checklist.

## Autonomous Notes

PathPlanner named commands are registered in `RobotContainer.java`.

The most important current autonomous helpers are:

- `autonPrepareBetaShot`
- `autonFeedBetaShot`
- `autonStopShooting`
- `autonTrackTower`
- `autonTrackTowerShort`
- `moveTurretToStartAngle`

For how those commands are intended to be combined, use `autonREADME.md`.

## Current Known Limits

- The turret does not yet have an absolute reference sensor.
- The pivot does not yet have an absolute reference sensor.
- Turret field tracking depends on the shared drivetrain pose estimate.
- There is no dedicated turret-mounted camera in the current setup.
- The climber path is present in software but may be intentionally disabled in
  hardware.

These are not reasons to avoid the robot. They are the main assumptions to keep
in mind while testing and debugging.

## Quick Debugging Guide

If the robot problem is mostly about:

- bad turret angle or wrong preset
  - open `tuningREADME.md`

- bad post-deploy behavior or "is the robot safe to enable?"
  - open `deploymentREADME.md`

- confusion about a PathPlanner named command
  - open `autonREADME.md`

- which button owns a feature
  - open `RobotContainer.java`

- how a full scoring sequence works
  - open `Superstructure.java`

## Build Status

Current expectation:

- `./gradlew.bat compileJava` should pass

The main remaining warning that has shown up recently is the existing CTRE
deprecation warning for `TalonFX(int, String)`.
