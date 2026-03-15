/*
 * Copyright (C) 2026 Team 3602 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import javax.swing.JFrame;

import frc.team3602.robot.Vision;
import frc.team3602.robot.Constants.ShooterConstants;
import frc.team3602.robot.Constants.ClimberConstants;
import frc.team3602.robot.generated.TunerConstants;
import frc.team3602.robot.subsystems.ClimberSubsystem;
import frc.team3602.robot.subsystems.CommandSwerveDrivetrain;
import frc.team3602.robot.subsystems.IntakeSubsystem;
import frc.team3602.robot.subsystems.Limelight_Pose;
import frc.team3602.robot.subsystems.PivotSubsystem;
import frc.team3602.robot.subsystems.ShooterSubsystem;
import frc.team3602.robot.subsystems.SpindexerSubsystem;
import frc.team3602.robot.subsystems.TurretSubsystem;

public class RobotContainer {
        private static final double DRIVER_SHOT_READY_RUMBLE = 1.0;

        private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired
                                                                                            // top // speed
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                          // second
                                                                                          // max angular velocity

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        private final Telemetry logger = new Telemetry(MaxSpeed);
        private SendableChooser<Command> autoChooser;
        private SendableChooser<Double> polarityChooser = new SendableChooser<>();

        public final CommandXboxController driverController = new CommandXboxController(0);
        public final CommandXboxController operatorController = new CommandXboxController(1);

        /* Subsystems */
        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        public final Vision vision = new Vision();
        public final IntakeSubsystem intake = new IntakeSubsystem();
        public final ShooterSubsystem shooter = new ShooterSubsystem(vision, drivetrain);
        public final TurretSubsystem turret = new TurretSubsystem(drivetrain);
        public final SpindexerSubsystem spindexer = new SpindexerSubsystem();
        public final PivotSubsystem pivot = new PivotSubsystem();
        private final ClimberSubsystem climberSubsys = new ClimberSubsystem();
        // Create the shared Limelight pose subsystem through its singleton accessor.
        // This avoids constructing an extra temporary object and makes it clear that
        // the whole robot is intended to use one shared vision-pose subsystem.
        public final Limelight_Pose limelightPose = Limelight_Pose.getInstance();
        public final Superstructure superStructure = new Superstructure(intake, shooter, spindexer, turret, drivetrain,
                        pivot);

        private Boolean intakeUp = (pivot.getPivotEncoder < 0);
        private Boolean intakeDown = (pivot.getPivotEncoder > 90);

        public RobotContainer() {
                // Registered Commands
                NamedCommands.registerCommand("autonRunBetaShot", superStructure.autonRunBetaShot());
                NamedCommands.registerCommand("autonLowerIntake", superStructure.autonLowerIntake());
                NamedCommands.registerCommand("autonStartIntake", superStructure.autonStartIntake());
                NamedCommands.registerCommand("autonStopIntake", superStructure.autonStopIntake());
                // Start the shooter for the simple beta autonomous shot without
                // feeding yet. This is useful while finishing a short path or aiming
                // step.
                NamedCommands.registerCommand("autonSpinUpBetaShot", superStructure.autonSpinUpBetaShot());
                // Wait until the shooter is near the beta autonomous target speed.
                // The command times out so the auto cannot hang forever.
                NamedCommands.registerCommand("autonWaitForBetaShotReady",
                                superStructure.autonWaitForBetaShotReady());
                // Prepare the basic autonomous shot by spinning up the shooter while
                // the turret tracks the alliance tower.
                NamedCommands.registerCommand("autonPrepareBetaShot", superStructure.autonPrepareBetaShot());
                // Feed the fuel for the current beta autonomous shot, then stop the
                // spindexer so the command finishes cleanly.
                NamedCommands.registerCommand("autonFeedBetaShot", superStructure.autonFeedBetaShot());
                // Stop the shooter and spindexer after an autonomous shot so later
                // actions begin from a known safe state.
                NamedCommands.registerCommand("autonStopShooting", superStructure.autonStopShooting());
                // Track the current alliance tower with the turret during
                // autonomous. This command is designed to run in parallel with path
                // following or another auton action rather than as a one-shot
                // command.
                NamedCommands.registerCommand("autonTrackTower", superStructure.autonTrackTower());
                // Short self-ending version of the turret tracking command for
                // sequential autos that need a simple "aim here briefly" step.
                NamedCommands.registerCommand("autonTrackTowerShort", superStructure.autonTrackTowerShort());
                NamedCommands.registerCommand("moveTurretToStartAngle", turret.moveToStartAngle());
              

                // named commands for pathplanner go here
                // Share the real shooter subsystem with the turret so any lead-angle
                // calculations use live shooter velocity instead of a null reference.
                turret.setShooterSubsystem(shooter);
                pivot.setDefaultCommand(pivot.holdPivot());
                if (ClimberConstants.kClimberEnabled) {
                        climberSubsys.setDefaultCommand(climberSubsys.setPosition());
                }
                turret.setDefaultCommand(turret.setPosition());
                configureBindings();
                polarityChooser.setDefaultOption("Positive", 1.0);
                polarityChooser.addOption("Negative", -1.0);
                SmartDashboard.putData("Polarity Chooser", polarityChooser);
                drivetrain.configPathplanner();
                updatePose();
                configAutonomous();
        }

        private void configureBindings() {
                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(() -> drive.withVelocityX(polarityChooser.getSelected()
                                                * -driverController.getLeftY() * MaxSpeed * drivetrain.turbo) // Drive
                                                                                                              // forward
                                                                                                              // with
                                                // negative Y
                                                // (forward)
                                                .withVelocityY(polarityChooser.getSelected()
                                                                * -driverController.getLeftX() * MaxSpeed
                                                                * drivetrain.turbo) // Drive left with negative X (left)
                                                .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive
                                                                                                                    // counterclockwise
                                                                                                                    // with
                                // negative X (left)

                                ));

                // Idle while the robot is disabled. This ensures the configured
                // neutral mode is applied to the drive motors while disabled.
                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));
                // Operator Controls

                // operatorController.rightTrigger().onTrue(superStructure.shootBall1())
                // .whileFalse(superStructure.stopShoot());
                operatorController.y().whileTrue(spindexer.setFeedVelocity(-35.0)).onFalse(spindexer.stopSpindexer());
                operatorController.leftTrigger().onTrue(superStructure.shootFailsafe())
                                .onFalse(superStructure.stopShoot()); // FAILSAFE
                // While the operator holds the right trigger, keep the turret pointed
                // at the current alliance tower. Releasing the trigger returns
                // control to the turret's default hold-position command.
                operatorController.rightTrigger().whileTrue(turret.trackOperatorFieldPoint());
                // Hold B for the full tracked lerp shot. This keeps the turret on the
                // tower, updates the shooter from the lerp table, and feeds
                // automatically once the shot is ready.
                operatorController.b().whileTrue(superStructure.shootTrackedLerpShot())
                                .onFalse(superStructure.stopShoot());
                operatorController.a().onTrue(shooter.setShootVelocity(-41.5)).onFalse(shooter.stopShooter());
                operatorController.x().onTrue(shooter.setShootVelocity(-44)).onFalse(shooter.stopShooter());
                // Use the live distance-based interpolation table while this button is
                // held so the operator can test or use the current lerp shot without
                // changing the fixed preset buttons.
                operatorController.rightBumper().onTrue(shooter.setShootVLerp()).onFalse(shooter.stopShooter());
                operatorController.leftBumper().onTrue(superStructure.stopIntake());
                operatorController.povUp().onTrue(turret.setAngleZero());
                operatorController.povDown().onTrue(turret.setAngleRight());
                operatorController.povLeft().onTrue(turret.setAngleLeftCorner());
                operatorController.povRight().onTrue(turret.setAngleRightCorner());

                // Let driver controller 0 rumble only when the operator's tracked
                // lerp-shot button is held and the robot is actually ready to fire.
                // This gives the driver a simple "send it now" cue without adding
                // another dashboard value to watch during teleop.
                new Trigger(() -> operatorController.getHID().getBButton() && superStructure.isTrackedLerpShotReady())
                                .whileTrue(driverShotReadyRumble());


                // DriverControls
                driverController.rightBumper().whileTrue(pivot.dumbDropIntake());
                driverController.leftBumper().whileTrue(intake.setIntakeSpeed()).whileFalse(intake.stopIntake());
                driverController.rightTrigger().onTrue(drivetrain.setTurbo()).onFalse(drivetrain.setNormalSpeed());
                if (ClimberConstants.kClimberEnabled) {
                        driverController.povUp().onTrue(climberSubsys.raiseClimber());
                        driverController.povDown().onTrue(climberSubsys.lowerClimber());
                }

                // Run SysId routines when holding back/start and X/Y.
                // Note that each routine should be run exactly once in a single log.
                // driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
                // driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

                // Reset the field-centric heading on left bumper press.
                // driverController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
                // drivetrain.registerTelemetry(logger::telemeterize);
        }

        /**
         * Rumbles driver controller 0 while the tracked lerp shot is ready.
         *
         * Trigger.whileTrue() schedules this command when the readiness condition is
         * true and cancels it automatically when the shot is no longer ready or the
         * operator releases B.
         */
        private Command driverShotReadyRumble() {
                return Commands.startEnd(
                                () -> driverController.getHID().setRumble(RumbleType.kBothRumble,
                                                DRIVER_SHOT_READY_RUMBLE),
                                () -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0));
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

        private void configAutonomous() {
                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData(autoChooser);
        }

        public void updatePose() {
                // Put the drivetrain's shared estimated pose on the dashboard.
                // This is the same pose used by autonomous and by Limelight vision
                // correction, so the numbers here should match how the robot behaves.
                SmartDashboard.putNumber("estimated drive pose x", drivetrain.getEstimatedPose().getX());
                SmartDashboard.putNumber("estimated drive pose y", drivetrain.getEstimatedPose().getY());
                SmartDashboard.putNumber("estimated drive pose rotation",
                                drivetrain.getEstimatedPose().getRotation().getDegrees());
        }
}
