/*
 * Copyright (C) 2026 Team 3602 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.team3602.robot.Vision;
import frc.team3602.robot.generated.TunerConstants;
import frc.team3602.robot.subsystems.CommandSwerveDrivetrain;
import frc.team3602.robot.subsystems.IntakeSubsystem;
import frc.team3602.robot.subsystems.PivotSubsystem;
import frc.team3602.robot.subsystems.ShooterSubsystem;
import frc.team3602.robot.subsystems.SpindexerSubsystem;
import frc.team3602.robot.subsystems.TurretSubsystem;

public class RobotContainer {


    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top                                                            // speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private SendableChooser<Command> autoChooser;
    private SendableChooser<Double> polarityChooser = new SendableChooser<>();

    public final CommandXboxController driverController = new CommandXboxController(0);
    public final CommandXboxController operatorController = new CommandXboxController(1);
    

    public final Vision vision = new Vision();
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final IntakeSubsystem intake = new IntakeSubsystem();
    public final ShooterSubsystem shooter = new ShooterSubsystem(vision);
    public final TurretSubsystem turret = new TurretSubsystem(drivetrain);
    public final SpindexerSubsystem spindexer = new SpindexerSubsystem();
    public final PivotSubsystem pivot = new PivotSubsystem();
    public final Superstructure superStructure = new Superstructure(intake, shooter, spindexer, turret, drivetrain, pivot);



    private Boolean intakeUp = (pivot.getPivotEncoder < 0);
    private Boolean intakeDown = (pivot.getPivotEncoder > 90);

    public RobotContainer() {
        // named commands for pathplanner go here
        pivot.setDefaultCommand(pivot.holdPivot());
        configureBindings();
        polarityChooser.setDefaultOption("Positive", 1.0);
        polarityChooser.addOption("Negative", -1.0);
        SmartDashboard.putData( "Polarity Chooser", polarityChooser);
    }
    

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(polarityChooser.getSelected()*-driverController.getLeftY() * MaxSpeed * drivetrain.turbo) // Drive forward with
                                                                                                  // negative Y
                                                                                                  // (forward)
                        .withVelocityY(polarityChooser.getSelected()*-driverController.getLeftX() * MaxSpeed * drivetrain.turbo) // Drive left with negative X (left)
                        .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                                    // negative X (left)

                ));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));
        //Operator Controls
        operatorController.y().whileTrue(superStructure.shootBall2()).whileFalse(superStructure.stopShoot());
        operatorController.povUp().onTrue(superStructure.stopIntake());
        operatorController.a().onTrue(turret.passMode());
        operatorController.back().onTrue(turret.setAngle(0)); //FAILSAFE
        
        //DriverControls
        driverController.rightBumper().onTrue(superStructure.intakeBall());
        driverController.rightTrigger().onTrue(drivetrain.setTurbo()).onFalse(drivetrain.setNormalSpeed());
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        // driverController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        drivetrain.registerTelemetry(logger::telemeterize);
    }
    

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
                // Reset our field centric heading to match the robot
                // facing away from our alliance station wall (0 deg).
                drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
                // Then slowly drive forward (away from us) for 5 seconds.
                drivetrain.applyRequest(() -> drive.withVelocityX(0.5)
                        .withVelocityY(0)
                        .withRotationalRate(0))
                        .withTimeout(5.0),
                // Finally idle for the rest of auton
                drivetrain.applyRequest(() -> idle));
    }
}
