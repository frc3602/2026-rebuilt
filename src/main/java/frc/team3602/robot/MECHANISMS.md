# Mechanisms

## Purpose

This file explains the robot's physical mechanisms and how the software expects
them to behave.

Use this document when you want to answer questions like:

- What does each mechanism do?
- What hardware assumptions does the code make?
- What is likely to fail after a reboot or repair?
- What should we test first on a mechanism?

## Drivetrain

Type:

- swerve drive

Software owner:

- `CommandSwerveDrivetrain.java`

What it does:

- field-centric driving
- robot pose estimation
- PathPlanner autonomous movement
- vision-assisted pose correction

What to watch:

- gyro heading
- estimated pose
- red/blue alliance perspective
- module behavior during slow test driving

Common failure points:

- wrong gyro heading
- bad module angle behavior
- pose drift
- vision corrections pulling pose incorrectly

## Turret

Software owner:

- `TurretSubsystem.java`

What it does:

- fixed presets
- alliance tower tracking
- moving-shot lead compensation
- rear-seam-aware travel control

Public angle convention:

- `0` = forward
- `+90` = left
- `-90` = right
- `180` = rear

Important physical assumption:

The turret currently does not have an absolute reference sensor. The code
assumes the turret starts in the shared startup position before boot.

Current startup angle:

- `90` degrees public aim angle

Why this matters:

If the turret is not physically staged correctly at boot, software position can
be wrong even if the motor and PID are fine.

What to test first:

1. startup staging
2. fixed presets
3. rear seam behavior
4. alliance tower tracking
5. moving-shot behavior

Common failure points:

- bad startup zero
- mechanical slip
- wrong sign or direction assumption
- pose estimate causing bad field tracking

## Shooter

Software owner:

- `ShooterSubsystem.java`

What it does:

- fixed shooter speeds
- interpolation-table based shooting
- closed-loop flywheel velocity control

Important idea:

The shooter uses a distance-to-speed table. Distance comes from robot pose and
target geometry.

What to test first:

1. both shooter motors spin together
2. fixed test speeds
3. lerp-table shot
4. repeatability across several distances

Common failure points:

- follower not staying synchronized
- wrong table values
- distance estimate error causing wrong speed target

## Spindexer And Transfer

Software owner:

- `SpindexerSubsystem.java`

What it does:

- moves fuel into the shooter
- runs spindexer and transfer together
- closed-loop feed speed control

Important idea:

The transfer wheel is smaller, so its commanded speed is scaled to keep surface
speed closer to the spindexer wheel.

What to test first:

1. feed on/off
2. both motors spin together
3. clean handoff into shooter
4. no chatter or hesitation

Common failure points:

- jam at spindexer-to-transfer handoff
- motors not matching well
- feed too early before shooter is ready

## Intake Roller

Software owner:

- `IntakeSubsystem.java`

What it does:

- pulls fuel into the robot
- can reverse for clearing or testing

Important idea:

The intake roller is separate from the pivot. That gives the code more
flexibility in teleop and autonomous.

## Intake Pivot

Software owner:

- `PivotSubsystem.java`

What it does:

- lowers the intake
- raises/stows the intake
- holds the requested pivot angle

Important physical assumption:

Like the turret, the pivot currently depends on known startup position rather
than an absolute sensor.

What to test first:

1. startup position
2. down preset
3. up/stowed preset
4. holding position after movement

Common failure points:

- bad startup zero
- mirrored motor direction problems
- mechanism drag or binding

## Climber

Software owner:

- `ClimberSubsystem.java`

Current state:

- software path exists
- hardware may be disabled

Important flag:

- `ClimberConstants.kClimberEnabled`

Why this matters:

The code can keep climber support in the project without trying to command
hardware that is not installed.

## Mechanism Interaction Notes

### Turret + Drivetrain

The turret depends on drivetrain pose for field-relative aiming.

### Shooter + Drivetrain

Shooter lerp speed depends on measured distance to the alliance tower.

### Intake + Pivot

The intake roller and pivot are separate mechanisms in software, but they are
usually used together during collection.

### Shooter + Spindexer

The shooter should already be running before the spindexer feeds. Feeding too
early usually hurts shot consistency.

## Current Highest-Risk Mechanism Assumptions

- turret startup staging
- pivot startup staging
- pose quality during tracked shots
- feed timing relative to shooter readiness
