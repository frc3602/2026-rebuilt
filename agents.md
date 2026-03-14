# AGENTS.md

## Repository Purpose
This repository contains code for a FIRST Robotics Competition (FRC) robot.

The codebase is used not only to operate the robot but also as a **teaching tool for high school students learning programming, robotics, and control systems**.

All code changes should prioritize:
- readability
- clarity
- instructional value
- maintainability

Do not optimize for cleverness or brevity if it reduces clarity.

---

# Core Principles

## 1. Code Must Be Educational
This code is read and modified by high school students.

Prefer:
- simple logic
- explicit variable names
- clear control flow

Avoid:
- unnecessary abstraction
- overly compact code
- advanced patterns that obscure understanding

Good code should help students understand **how the robot works**.

---

## 2. Comment Thoroughly

Comments are extremely important.

Every function should include:
- what it does
- why it exists
- how it interacts with the robot

Complex logic must include **step-by-step explanations**.

Example:

```java
// Calculate the desired turret angle needed to aim at the target.
// We use the robot's field position and the target location.
// This allows the turret to automatically track the goal while driving.
double turretAngle = calculateTargetAngle(robotPose);\

.