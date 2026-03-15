# Autonomous Command Reference

## Purpose

This file explains the autonomous helper commands that are registered for
PathPlanner.

The goal is to make it easy for students to answer these questions:

- What named commands exist right now?
- What does each command do?
- When should I use each command in an auto?
- What is the simplest way to build a basic scoring auto?

## Where These Commands Are Registered

Named PathPlanner commands are registered in:

- `RobotContainer.java`

Most of the autonomous sequence logic lives in:

- `Superstructure.java`
- `TurretSubsystem.java`

## Current Named Commands

### `autonRunBetaShot`

Use this when you want one single command that performs the entire simple
autonomous shot.

What it does:

1. Starts the shooter for the beta auto shot.
2. Waits for the shooter to reach the ready speed, with a timeout.
3. Tracks the alliance tower with the turret while waiting.
4. Feeds the note with the spindexer.
5. Stops the spindexer and shooter.

Best use:

- Simple one-shot autonomous routines
- Fast testing when you want one named command instead of several smaller ones

### `autonLowerIntake`

Use this when you only want to lower the intake/pivot for autonomous.

What it does:

1. Drops the intake pivot.

Best use:

- Starting a note collection routine
- Preparing for future intake autos

### `autonSpinUpBetaShot`

Use this when you want to start the shooter early, but do not want to feed yet.

What it does:

1. Commands the shooter to the beta autonomous shot speed.

Best use:

- While finishing a short drive path
- Before a separate aiming or settle step

### `autonWaitForBetaShotReady`

Use this when you want the auto to pause until the shooter is near the target
speed.

What it does:

1. Waits until the shooter velocity is at the beta ready threshold.
2. Ends early if the timeout is reached.

Best use:

- After spin-up
- Before feeding

### `autonPrepareBetaShot`

Use this when you want one setup command for a simple shot, but still want
feeding to stay separate.

What it does:

1. Starts the shooter.
2. Tracks the alliance tower with the turret.
3. Ends when the shooter is ready or when the timeout is reached.

Best use:

- Basic autos that drive first, then prepare the shot
- PathPlanner routines where feeding should happen in a separate step

### `autonFeedBetaShot`

Use this when the robot is already aimed and the shooter is already near speed.

What it does:

1. Runs the spindexer feed at the beta shot feed speed.
2. Feeds for a fixed amount of time.
3. Stops the spindexer at the end.

Best use:

- The final release step of a shot
- Autos that separate "prepare" and "fire"

### `autonStopShooting`

Use this to clean up after an autonomous shot.

What it does:

1. Stops the spindexer.
2. Stops the shooter.

Best use:

- Immediately after feeding
- Before moving on to the next autonomous action

### `autonTrackTower`

Use this when the turret should continuously follow the current alliance tower.

What it does:

1. Recomputes the tower angle every loop.
2. Keeps the turret pointed at the alliance tower while the command is running.

Important note:

- This command does not end on its own.

Best use:

- In parallel with a path
- As a PathPlanner event command

### `autonTrackTowerShort`

Use this when you want a short tower-tracking settle step that ends by itself.

What it does:

1. Tracks the alliance tower.
2. Stops after a short timeout.

Best use:

- Sequential autos
- Short settle windows before a shot

### `moveTurretToTeleopHandoff`

Use this when you want to send the turret back to the teleop handoff position.

What it does:

1. Sends the turret to the normal teleop handoff angle.

Best use:

- At the end of an autonomous routine
- When returning to a known safe turret position

### `moveTurretToAutoStartAngle`

Use this when you want to send the turret to the autonomous starting angle.

What it does:

1. Sends the turret to the auto start angle.

Best use:

- At the beginning of an auto
- Before a predictable opening shot

## Recommended Basic Shot Sequence

For the team's current simplest scoring auto, use this order:

1. Drive a short path in PathPlanner.
2. Run `autonPrepareBetaShot`.
3. Run `autonFeedBetaShot`.
4. Run `autonStopShooting`.
5. Optionally run `moveTurretToTeleopHandoff`.

This gives the robot time to:

- move into position
- track the target
- spin up the shooter
- feed the note
- shut down cleanly

## Example PathPlanner Flow

Example simple autonomous sequence:

1. `path: ShootBetaDrive`
2. `named: autonPrepareBetaShot`
3. `named: autonFeedBetaShot`
4. `named: autonStopShooting`
5. `named: moveTurretToTeleopHandoff`

## Suggested Future Commands

These do not exist yet, but would make full autonomous routines easier to build:

- `autonRunIntake`
- `autonStopIntake`
- `autonCollectNote`
- `autonPrepareLerpShot`
- `autonFeedShot`
- `autonStopAll`

## Team Notes

- Keep command names action-based so they describe what the robot actually does.
- Prefer names like `autonTrackTower` over names that depend on old tuning notes
  or temporary numbers.
- Try to separate "prepare", "feed", and "stop" steps so autos are easier to
  tune in PathPlanner.
