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

## How PathPlanner Builds An Auto

PathPlanner is not where the robot behavior itself is written.

Instead, PathPlanner is where we describe the order of events:

- drive this path
- wait here
- run this named command
- run two things in parallel

The actual robot actions are still defined in Java.

Here is the normal flow:

1. A `.auto` file in PathPlanner defines the order of paths and named commands.
2. `RobotContainer.java` registers named commands with `NamedCommands`.
3. `Superstructure.java` and subsystem classes define what those commands do.
4. During autonomous, PathPlanner plays the sequence and calls the matching Java
   commands at the correct time.

That means PathPlanner should usually answer:

- When should this happen?
- In what order should things happen?
- Which actions happen at the same time?

And the Java code should usually answer:

- What does this action actually do?
- Which motors or subsystems does it control?
- When should this action end?

## How To Structure Autonomous Commands

The easiest autos to build usually come from using layers of commands instead of
putting everything into one giant command.

### 1. Small Commands

These do one job only.

Examples:

- `autonLowerIntake`
- `autonRaiseIntake`
- `autonStartIntake`
- `autonStopIntake`
- `autonSpinUpBetaShot`
- `autonFeedBetaShot`
- `autonStopShooting`
- `autonTrackTower`

Why these are useful:

- They are easy to reuse.
- They are easy to test.
- They are easy to rename and understand.
- They keep PathPlanner flexible.

### 2. Setup Commands

These combine a few small actions into one useful step.

Example:

- `autonPrepareBetaShot`

This command is a good setup command because it:

- starts the shooter
- tracks the tower
- waits for the shooter to be ready

but it still does not feed the fuel yet.

That is a good balance between:

- being simple enough to reuse
- being big enough to save time in PathPlanner

### 3. Full Macro Commands

These run an entire behavior from start to finish.

Example:

- `autonRunBetaShot`

Macro commands are useful when:

- you want fast testing
- you want one simple block for a very common action
- the full sequence is unlikely to change often

Macro commands become less useful when:

- different autos need slightly different timing
- you want to move and shoot in different ways
- you need to tune only one part of the sequence

Because of that, macro commands are best treated as convenience commands, not as
the only way to build autonomous routines.

## Recommended Design Rule

When adding new auton logic, try to build it in this order:

1. Write the smallest useful command first.
2. Combine those into a setup command if that makes PathPlanner easier to read.
3. Add a full macro command only if the team uses that exact sequence often.

This keeps the robot code flexible without making students drag around a huge
number of tiny one-line commands in every auto.

## Named Commands By Function

### Intake / Fuel Collection Setup

- `autonLowerIntake`
- `autonRaiseIntake`
- `autonStartIntake`
- `autonStopIntake`

### Shooter Prep

- `autonSpinUpBetaShot`
- `autonWaitForBetaShotReady`
- `autonPrepareBetaShot`

### Shot Release And Cleanup

- `autonFeedBetaShot`
- `autonStopShooting`
- `autonRunBetaShot`

### Turret Tracking / Position

- `autonTrackTower`
- `autonTrackTowerShort`
- `moveTurretToStartAngle`

## Current Named Commands

### `autonRunBetaShot`

Use this when you want one single command that performs the entire simple
autonomous shot.

What it does:

1. Starts the shooter for the beta auto shot.
2. Waits for the shooter to reach the ready speed, with a timeout.
3. Tracks the alliance tower with the turret while waiting.
4. Feeds the fuel with the spindexer.
5. Stops the spindexer and shooter.

Best use:

- Simple one-shot autonomous routines
- Fast testing when you want one named command instead of several smaller ones

### `autonLowerIntake`

Use this when you only want to lower the intake/pivot for autonomous.

What it does:

1. Drops the intake pivot.

Best use:

- Starting a fuel collection routine
- Preparing for future intake autos

### `autonRaiseIntake`

Use this when you want to bring the intake path back up during autonomous
without changing the intake roller state.

What it does:

1. Sends the pivot to the team's current raised or stowed preset.

Best use:

- Before shooting so the intake path is out of the way
- Before crossing back into a protected zone or traffic area
- Any time an auto wants to separate "stop collecting" from "stow the mechanism"

### `autonStartIntake`

Use this when you want to start only the intake roller during autonomous.

What it does:

1. Starts the intake motor.

Best use:

- After lowering the intake
- While driving toward loose fuel
- As one piece of a future collection routine

### `autonStopIntake`

Use this when you want to stop only the intake roller during autonomous.

What it does:

1. Stops the intake motor.

Best use:

- After finishing a collection step
- Before a shot
- Any time an auto should stop the roller without moving the pivot

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

### `moveTurretToStartAngle`

Use this when you want to send the turret back to its shared start/handoff
position.

What it does:

1. Sends the turret to the current starting angle.

Best use:

- At the beginning of an auto
- At the end of an autonomous routine
- Any time you want the turret in its known home position

## How To Decide Between A Macro And Smaller Steps

Use a macro command like `autonRunBetaShot` when:

- the whole sequence is always used together
- the timing is already known and stable
- the team wants the fastest possible setup for a simple auto

Use smaller steps like `autonPrepareBetaShot` and `autonFeedBetaShot` when:

- you want to tune the shot timing
- you may want to add a wait or another path in between
- you want the auto to be easier to read in PathPlanner
- you expect different autos to reuse the same pieces in different ways

For most competition autos, smaller steps are usually the better default.

## Recommended Basic Shot Sequence

For the team's current simplest scoring auto, use this order:

1. Drive a short path in PathPlanner.
2. Run `autonPrepareBetaShot`.
3. Run `autonFeedBetaShot`.
4. Run `autonStopShooting`.
5. Optionally run `moveTurretToStartAngle`.

This gives the robot time to:

- move into position
- track the target
- spin up the shooter
- feed the fuel
- shut down cleanly

## Example PathPlanner Flow

Example simple autonomous sequence:

1. `path: ShootBetaDrive`
2. `named: autonPrepareBetaShot`
3. `named: autonFeedBetaShot`
4. `named: autonStopShooting`
5. `named: moveTurretToStartAngle`

## Example Of A Good Thought Process

If the robot must:

- move slightly
- aim at the tower
- spin up the shooter
- feed the fuel

then a good way to build that auto is:

1. Let PathPlanner handle the short drive path.
2. Use a setup command like `autonPrepareBetaShot` for aiming and flywheel prep.
3. Use `autonFeedBetaShot` only when the robot is ready to release the fuel.
4. Use `autonStopShooting` so the robot ends in a known state.

This is usually better than forcing all of those decisions into one giant
command because:

- the order is easier to see
- timing is easier to tune
- future autos can reuse the same shot steps
- problems are easier to debug

## Common Mistakes To Avoid

- Do not put every autonomous behavior into one giant command.
- Do not create too many tiny commands that are too specific to one auto.
- Do not use unclear names that depend on temporary tuning numbers.
- Do not rely on long fixed waits when a command can end from a real condition.
- Do not hide important shot timing inside a macro if you know that timing will
  probably need to change later.

## Checklist For A New Auto

Before adding a new autonomous routine, ask:

- What should be controlled by PathPlanner order?
- What should be controlled by Java command logic?
- Do I need one small command, one setup command, or one macro command?
- Will another auto probably reuse this behavior later?
- Is the command name clear enough that a student can understand it quickly?

## Suggested Future Commands

These do not exist yet, but would make full autonomous routines easier to build:

- `autonRunIntake`
- `autonCollectFuel`
- `autonPrepareLerpShot`
- `autonFeedShot`
- `autonStopAll`

## Team Notes

- Keep command names action-based so they describe what the robot actually does.
- Prefer names like `autonTrackTower` over names that depend on old tuning notes
  or temporary numbers.
- Try to separate "prepare", "feed", and "stop" steps so autos are easier to
  tune in PathPlanner.
