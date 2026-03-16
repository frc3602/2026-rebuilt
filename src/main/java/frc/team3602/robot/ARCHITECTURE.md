# Architecture

## Purpose

This file explains how the robot software is organized.

Use this document when you want to answer questions like:

- Where should I start reading the code?
- Which class owns what behavior?
- How do subsystems communicate?
- Where do pose, vision, turret aiming, and shot logic come together?

This is a software-structure document, not a tuning guide.

## High-Level Structure

The robot uses a command-based architecture.

Important top-level files:

- `RobotContainer.java`
  - Main wiring file
  - Creates subsystems
  - Registers PathPlanner named commands
  - Defines controller bindings
  - Sets default commands

- `Superstructure.java`
  - Combines multiple subsystems into larger robot actions
  - Holds multi-step teleop and auton shot logic

- `Constants.java`
  - Hardware IDs
  - Field constants
  - Shared robot constants

## Main Subsystems

### `CommandSwerveDrivetrain`

Owns:

- swerve driving
- estimated robot pose
- PathPlanner configuration
- vision measurement fusion into pose estimation
- basic field visualization

Important idea:

This subsystem is the shared source of truth for robot pose. Turret aiming,
shooter distance, autonomous path following, and vision all rely on this shared
pose estimate.

### `Limelight_Pose`

Owns:

- reading Limelight pose estimates
- checking whether vision frames are fresh
- rejecting bad measurements
- selecting the best current camera update

Important idea:

This subsystem does not directly drive the robot. It decides whether a vision
update is trustworthy enough to give to the drivetrain pose estimator.

### `TurretSubsystem`

Owns:

- turret motor control
- turret presets
- tower tracking
- lead-angle calculation while driving
- turret telemetry

Important idea:

Public turret angles use the standard WPILib signed convention:

- `0` = forward
- `+90` = left
- `-90` = right
- `180` = rear

Internally, the turret still uses a private `0` through `360` travel model so
it can handle the rear seam correctly.

### `ShooterSubsystem`

Owns:

- shooter motor control
- fixed shooter speeds
- lerp-table speed selection by distance
- shooter readiness checks

Important idea:

The shooter table is distance-based. The robot measures target distance, then
looks up the current shooter speed needed for that distance.

### `SpindexerSubsystem`

Owns:

- the main spindexer wheel
- the transfer/receive wheel
- feed velocity commands
- feed stop / reverse behavior

Important idea:

The robot usually runs the feed path in closed-loop velocity mode instead of
raw percent output.

### `PivotSubsystem`

Owns:

- intake pivot angle
- intake up/down setpoints
- holding the intake at the requested angle

Important idea:

The pivot default command keeps holding the last requested pivot position.

### `IntakeSubsystem`

Owns:

- intake roller only

Important idea:

The intake roller is separate from the intake pivot, so autos and teleop can
control them independently.

## Data Flow

### Pose And Vision Flow

1. `Limelight_Pose` reads camera pose data.
2. It filters stale or low-quality measurements.
3. `CommandSwerveDrivetrain` accepts the selected measurement.
4. The drivetrain estimator updates the robot pose.
5. Other systems use that pose.

Consumers of the shared pose include:

- turret tracking
- shooter distance lookup
- PathPlanner autonomous
- dashboard field display

### Shot Flow

For the tracked shot:

1. turret tracks the current alliance tower
2. shooter follows the current lerp-table target
3. robot waits until the shot is ready
4. spindexer feeds
5. stop command cleans up afterward

### Autonomous Flow

PathPlanner defines:

- path order
- command timing
- parallel vs sequential actions

Java defines:

- what each named command actually does
- which subsystems it uses
- when it ends

## Default Commands

Current important default commands include:

- drivetrain drive command
- pivot hold command
- turret hold-position command
- climber hold command only when enabled

Default commands matter because they define what the robot keeps doing when no
higher-priority command is using the subsystem.

## Design Philosophy

This codebase generally prefers:

- readable names
- small reusable commands
- setup commands for common combinations
- macro commands only when the full sequence is reused often

Examples:

- `autonLowerIntake` is a small command
- `autonRunIntake` is a setup command
- `autonShootTower` is a macro command

## Known Architectural Limits

- turret still depends on staged startup position
- pivot still depends on staged startup position
- turret tracking depends on good shared drivetrain pose
- turret moving-shot compensation is real, but still fairly simple compared to
  elite fully-modeled systems

## Best Files To Read First

If you are new:

1. `RobotContainer.java`
2. `Superstructure.java`
3. `TurretSubsystem.java`
4. `CommandSwerveDrivetrain.java`
5. `Limelight_Pose.java`
