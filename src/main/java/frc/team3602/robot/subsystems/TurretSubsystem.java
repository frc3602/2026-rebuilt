package frc.team3602.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.utility.WheelForceCalculator.Feedforwards;
import com.ctre.phoenix6.configs.MotionMagicConfigs;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import frc.team3602.robot.Constants.*;
import frc.team3602.robot.Constants.FieldConstants;
import frc.team3602.robot.LimelightHelpers.PoseEstimate;
import frc.team3602.robot.Vision;
import frc.team3602.robot.subsystems.ShooterSubsystem;

public class TurretSubsystem extends SubsystemBase {
    // The turret starts pointed at 90 degrees and can only travel to 0 degrees in
    // one direction or to 270 degrees in the other direction. Once we normalize the
    // turret angle into [-180, 180), that legal window becomes [-90, 90].
    private static final double MIN_TRACKING_ANGLE_DEGREES = -90.0;
    private static final double MAX_TRACKING_ANGLE_DEGREES = 90.0;
    private static final double STARTING_TURRET_ANGLE_DEGREES = 90.0;
    private static final double LEFT_CORNER_PRESET_DEGREES = -55.0;
    private static final double RIGHT_CORNER_PRESET_DEGREES = 55.0;
    private static final double NEUTRAL_PRESET_DEGREES = -90.0;

    public CommandSwerveDrivetrain drivetrainSubsys;
    public ShooterSubsystem shooter;

    SendableChooser<Double> startChooser = new SendableChooser<Double>();

    public TurretSubsystem(CommandSwerveDrivetrain drivetrainSubsys) {
        this.drivetrainSubsys = drivetrainSubsys;
        initializeTurretTuningState();
    }

    public TurretSubsystem() {
        initializeTurretTuningState();
    }

    /**
     * Stores the shooter subsystem reference used by the turret's lead calculations.
     *
     * The turret does not create the shooter itself. Instead, RobotContainer passes
     * the shared shooter subsystem in after construction so both subsystems use the
     * same hardware objects.
     */
    public void setShooterSubsystem(ShooterSubsystem shooter) {
        this.shooter = shooter;
    }

    /**
     * Sets up chooser options and PID tolerance used by the turret subsystem.
     *
     * We call this from every constructor so the active turret subsystem always gets
     * the same basic setup.
     */
    private void initializeTurretTuningState() {
        startChooser.setDefaultOption("Right Trench", Double.valueOf(0));
        startChooser.addOption("Right Trench", Double.valueOf(0));
        startChooser.addOption("Left Trench", Double.valueOf(180));
        turretController.setTolerance(1, 2);
        configTurretHardware();
    }

    public double getsetAnlge() {
        return startChooser.getSelected();
    }

    // Motor
    private final TalonFX turretMotor = new TalonFX(TurretConstants.kTurretMotorID, "rio");

    private double turretFeedForward = 0.0;

    // Encoder
    public Double getEncoder() {
        return (turretMotor.getRotorPosition().getValueAsDouble() * 12); // every revolution is 12 degrees because it is
                                                                         // a 30:1 gear ratio 10:1 from gear, 3:1 from
                                                                         // gear box
    }

    // Vision
    public final Vision vision = new Vision();

    // Hold the turret at its intended starting angle until another command asks
    // for a different target.
    public double setAngle = STARTING_TURRET_ANGLE_DEGREES;

    // Controllers *These PID values need to be changed*
    private final PIDController turretController = new PIDController(.04, 0.0, 0.0);
    private final PIDController aimController = new PIDController(.04, 0.0, 0);

    private final Feedforwards aimFf = new Feedforwards(0);

    // Commands

    public Command changeSetAngle(double newSetpoint) {
        return runOnce(() -> {
            setRequestedAngle(setAngle + newSetpoint);
        });
    }

    public Command setAngle(double setPosition) {
        return runOnce(() -> {
            setRequestedAngle(setPosition);
        });
    }

    public Command testTurret(double voltage) {
        return runOnce(() -> {
            turretMotor.setVoltage(voltage);
        });

    }

    public Command stopTurret() {
        return runOnce(() -> {
            turretMotor.stopMotor();
        });
    }

    public Command turretAlignment() {
        return runOnce(() -> {
            setRequestedAngle(setAngle + vision.getTurretTX());
        });
    }

    public Command autonToTeleop() {
        return runOnce(() -> {
            // Use the same handoff angle that the mechanism is expected to start at.
            setRequestedAngle(STARTING_TURRET_ANGLE_DEGREES);
        });
    }

    // Pose Aiming
    public Translation2d getTargetPose() {
        Optional<Alliance> allianceOpt = DriverStation.getAlliance();

        // Check if an alliance is present
        if (allianceOpt.isPresent()) {
            Alliance alliance = allianceOpt.get(); // unwrap the Optional

            if (alliance == Alliance.Blue) {
                return FieldConstants.kBlueTowerPosition;
            } else if (alliance == Alliance.Red) {
                return FieldConstants.kRedTowerPosition;
            }
        }

        // Default to the blue-side tower if the alliance has not been reported yet.
        return FieldConstants.kBlueTowerPosition;
    }

    /**
     * Calculates the robot-to-target distance for turret aiming logic.
     *
     * We read the drivetrain's shared estimated pose so turret calculations stay in
     * sync with the same pose used by autonomous and Limelight vision correction.
     */
    public double getDistanceToTarget() {

        // Robot pose from the drivetrain's shared estimate.
        Pose2d robotPose = drivetrainSubsys.getEstimatedPose();

        // Target field location (meters).
        Translation2d targetPosition = getTargetPose();

        // Robot position on the field.
        Translation2d robotPosition = robotPose.getTranslation();

        // Straight-line distance from the robot to the target.
        double distance = robotPosition.getDistance(targetPosition);

        double distanceFeet = Units.metersToFeet(distance);

        return distanceFeet;
    }

    public double getTurretAngleDeg() {
        // Get rotor position in motor rotations
        double motorRot = turretMotor.getRotorPosition().getValueAsDouble();

        // Convert motor rotations to turret degrees
        // Assume 30 motor rotations = 360 degrees turret rotation
        double turretDeg = motorRot * (360.0 / 30.0);

        // Clamp or normalize angle to -180° to +180° (or use 0-360 if you prefer)
        return clampAngle(turretDeg);
    }

    /**
     * Calculates the turret angle needed to face the field target.
     *
     * The turret aims by comparing the robot's corrected field pose to the target's
     * field coordinates. This lets vision-based pose corrections improve aiming
     * immediately instead of waiting for a separate odometry-only estimate.
     */
    public double calculateDesiredAngle() {

        Pose2d robotPose = drivetrainSubsys.getEstimatedPose();

        Translation2d target = getTargetPose();

        double dx = target.getX() - robotPose.getX();
        double dy = target.getY() - robotPose.getY();
        double fieldAngle = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeading = robotPose.getRotation().getDegrees();
        return clampAngle(fieldAngle - robotHeading);
    }

    /**
     * Normalize angle to [-180, 180) degrees
     */
    private double clampAngle(double angleDeg) {
        angleDeg = angleDeg % 360.0; // wrap around
        if (angleDeg > 180.0)
            angleDeg -= 360.0;
        if (angleDeg <= -180.0)
            angleDeg += 360.0;
        return angleDeg;
    }

    /**
     * Limits a value to a safe minimum and maximum.
     *
     * We use this helper to keep turret tracking setpoints inside the legal travel
     * range of the mechanism.
     */
    private double clamp(double value, double minValue, double maxValue) {
        return Math.max(minValue, Math.min(maxValue, value));
    }

    /**
     * Updates the desired turret angle while respecting the mechanism's legal
     * travel.
     *
     * All code paths that request a turret angle should go through this helper so
     * button commands, autonomous commands, and tracking commands all agree on the
     * same physical limits.
     */
    private void setRequestedAngle(double requestedAngleDegrees) {
        double normalizedAngle = clampAngle(requestedAngleDegrees);
        setAngle = clamp(normalizedAngle, MIN_TRACKING_ANGLE_DEGREES, MAX_TRACKING_ANGLE_DEGREES);
    }

    public double getBallVelocity() {
        // If the shooter subsystem was not linked in, we cannot estimate flight time.
        // Returning zero keeps the turret code safe and tells the lead calculation to
        // fall back to "aim directly at the target with no motion lead."
        if (shooter == null) {
            return 0.0;
        }

        double motorRPS = shooter.getVelocity();

        double gearRatio = 1.0; // example motor:wheel
        double wheelDiameterMeters = 0.1016; // 4 inch wheel

        double wheelRPS = motorRPS / gearRatio;

        double circumference = Math.PI * wheelDiameterMeters;

        double ballVelocity = wheelRPS * circumference;

        return ballVelocity; // meters per second
    }

    public double calculateBallTimeOfFlight() {
        // Ball Velocity m/s TODO: Must Change
        double ballVelocity = getBallVelocity();
        if (ballVelocity <= 0.0) {
            return 0.0;
        }
        // Ball Launch Angle degrees TODO: Must Change
        double launchAngleDegrees = 15;
        // Shooter Height meters
        double shooterHeight = 0.4826;
        // Target Height meters
        double targetHeight = 1.8288;
        // Convert launch angle from degrees to radians (Java's trig functions use
        // radians)
        double launchAngleRadians = Math.toRadians(launchAngleDegrees);

        // Calculate the vertical component of velocity
        double vVertical = ballVelocity * Math.sin(launchAngleRadians);

        // Calculate height difference (positive if target is higher than shooter)
        double heightDifference = targetHeight - shooterHeight;

        // Gravity constant (m/s²)
        double g = 9.81;

        // Calculate the part inside the square root: (v sin(θ))² + 2gh
        double insideSqrt = Math.pow(vVertical, 2) + 2 * g * heightDifference;

        // Take the square root (ensure it's not negative)
        double sqrtTerm = Math.sqrt(Math.max(0, insideSqrt));

        // Complete formula: t = (vVertical + sqrtTerm) / g
        double ballTimeOfFlight = (vVertical + sqrtTerm) / g;

        return ballTimeOfFlight;
    }

    /**
     * Calculates the lead-adjusted turret angle for the current tower target.
     *
     * We start with the drivetrain's shared estimated pose, compute the target angle
     * relative to the robot, then add a lead term based on the robot's sideways
     * motion during the shot's time of flight.
     *
     * Even though the method name says "offset", the value returned here is the
     * full robot-relative angle that the turret should try to reach.
     */
    public double calculateTurretOffset() {

        // Get the latest shared robot pose.
        Pose2d robotPose = drivetrainSubsys.getEstimatedPose();

        // Target position on field (meters)
        Translation2d targetPosition = getTargetPose();

        // Robot position
        Translation2d robotPosition = robotPose.getTranslation();

        // Vector from robot to target
        Translation2d robotToTarget = targetPosition.minus(robotPosition);

        // Distance to target
        double distanceToTarget = robotToTarget.getNorm();

        // Angle to target (field relative)
        Rotation2d angleToTarget = robotToTarget.getAngle();

        // Convert to robot-relative angle
        double robotHeading = robotPose.getRotation().getRadians();
        double targetYawRelative = angleToTarget.getRadians() - robotHeading;

        // Robot velocity
        double robotVelocity = drivetrainSubsys.getState().Speeds.vxMetersPerSecond;

        // Time of flight
        double timeOfFlight = calculateBallTimeOfFlight();

        if (timeOfFlight <= 0.0) {
            return Math.toDegrees(targetYawRelative);
        }

        // Lateral movement during shot
        double lateralMovement = robotVelocity * timeOfFlight;

        // Lead angle
        double leadRadians = Math.atan2(lateralMovement, distanceToTarget);

        // Final turret offset
        double turretOffsetRadians = targetYawRelative + leadRadians;

        return Math.toDegrees(turretOffsetRadians);
    }

    double voltage;

    double aimOutput;

    public boolean atTarget;

    public Command aimCommand() {
        return run(() -> {
            // Command the full lead-adjusted tower angle directly and keep applying
            // turret PID while this command owns the subsystem.
            setRequestedAngle(calculateTurretOffset());
            applyTurretPositionControl();
        });
    }

        public Command setAngleAuto() {
        return  runOnce(() -> {
            setRequestedAngle(STARTING_TURRET_ANGLE_DEGREES);
        });
    }

    public Command setAngleLeftCorner() {
        return  runOnce(() -> {
            setRequestedAngle(LEFT_CORNER_PRESET_DEGREES);
        });
    }

    public Command setAngleRightCorner() {
        return  runOnce(() -> {
            // Keep the right-corner preset inside the legal turret travel.
            // Using an explicit positive preset makes this distinct from the neutral
            // position and avoids silently clamping onto the same hard stop.
            setRequestedAngle(RIGHT_CORNER_PRESET_DEGREES);
        });
    }

    public Command setAngleNeutral() {
        return  runOnce(() -> {
            // Neutral means "park on the negative-side edge" for this mechanism.
            setRequestedAngle(NEUTRAL_PRESET_DEGREES);
        });
    }

    public Command setAngleZero() {
        return  runOnce(() -> {
            setRequestedAngle(0);
        });
    }

    public Command basicAuton() {
        return  runOnce(() -> {
            // Point at the live alliance tower target instead of using a magic angle.
            setRequestedAngle(calculateTurretOffset());
        });
    }

    /**
     * Tracks a fixed field point while the command is running.
     *
     * This is useful in autonomous because the drivetrain can move along a path
     * while the turret continuously recomputes the angle to a field coordinate.
     * The command does not finish on its own, so it should be used as a parallel
     * command, a PathPlanner event command, or with a timeout.
     */
    public Command trackFieldPoint(Translation2d fieldPoint) {
        return run(() -> {
            setRequestedAngle(calculateTurretAngleForFieldPoint(fieldPoint));
            applyTurretPositionControl();
        });
    }

    /**
     * Tracks the current alliance tower while the command is held.
     *
     * This wrapper chooses the blue or red tower based on the current alliance so
     * teleop and autonomous can share one named field target instead of a temporary
     * hard-coded point.
     */
    public Command trackOperatorFieldPoint() {
        return trackAllianceTower();
    }

    /**
     * Tracks the current alliance tower while the command is running.
     *
     * This is the high-level aiming command used by teleop and autonomous when we
     * want the turret to follow the correct tower target for the current side of the
     * field.
     */
    public Command trackAllianceTower() {
        return run(() -> {
            // Recompute the tower angle every loop so the turret follows the current
            // alliance target while the robot drives.
            setRequestedAngle(calculateTurretOffset());
            applyTurretPositionControl();
        });
    }

    /**
     * Calculates the turret angle needed to face a field coordinate.
     *
     * We convert the field point into a robot-relative angle, then clamp that angle
     * into the turret's legal travel range so the mechanism only asks for positions
     * it can physically reach.
     */
    private double calculateTurretAngleForFieldPoint(Translation2d fieldPoint) {
        Pose2d robotPose = drivetrainSubsys.getEstimatedPose();

        double deltaX = fieldPoint.getX() - robotPose.getX();
        double deltaY = fieldPoint.getY() - robotPose.getY();
        double fieldAngleToPoint = Math.toDegrees(Math.atan2(deltaY, deltaX));
        double robotHeading = robotPose.getRotation().getDegrees();
        double robotRelativeAngle = clampAngle(fieldAngleToPoint - robotHeading);

        return clamp(robotRelativeAngle, MIN_TRACKING_ANGLE_DEGREES, MAX_TRACKING_ANGLE_DEGREES);
    }


    // public Command track() {
    // return run(() -> {
    // if (vision.getTurretHasTarget()) {
    // aimOutput = aimController.calculate(vision.getTurretTX(),5);//setpoint is the
    // offset of the turret(temp)
    // setAngle = setAngle - aimOutput + calculateTurretOffset() ;
    // }
    // setAngle = turnFeedforward() + setAngle; // Adds rotational feedforward
    // atTarget = (Math.abs(setAngle - getEncoder())<0.5);
    // voltage = turretController.calculate(getEncoder(), setAngle);
    // if (voltage > 4) { //TODO: create constant for 2, do not go higher than 2
    // voltage = 4;
    // } else if (voltage < -4) {
    // voltage = -4;
    // } turretMotor.setVoltage(voltage);
    // }

    // );

    // }

    public Command aimToDesiredAngle() {
        return run(() -> {
            // Continuously point at the lead-adjusted tower angle while the driver
            // holds the button.
            setRequestedAngle(calculateTurretOffset());
            applyTurretPositionControl();
        });
    }

    // THIS IS IN PROGRESS DO NOT HATE HATER ABE
    public Command passMode() {
        return run(() -> {
            if (vision.getTurretHasTarget()) {
                aimOutput = aimController.calculate(vision.getTurretTX(), 5);// setpoint is the offset of the
                                                                             // turret(temp)
                setRequestedAngle(setAngle - aimOutput + calculateTurretOffset());

            } else {
                vision.getPose();

            }

        });
    }

    public Command setPosition() {
        return run(() -> {
            applyTurretPositionControl();
        });
    }

    /**
     * Applies the turret position PID using the current desired set angle.
     *
     * This helper is shared by both the default "hold position" command and the
     * alliance-tower tracking command so they use the same motor-control behavior.
     */
    private void applyTurretPositionControl() {
        var pidEffort = turretController.calculate(getTurretAngleDeg(), clampAngle(setAngle));
        turretMotor.setVoltage(pidEffort);
    }

    double rotationSpeed;

    // Calculations
    // public double rAlignment() {

    // double tx = vision.getTX();

    // rotationSpeed = turretController.calculate(tx, 0);

    // if (Math.abs(rotationSpeed) < 0.5) {
    // rotationSpeed = 0;
    // }

    // return rotationSpeed;

    // }
    // Getting the rotational speed in degrees per execution
    // Commands run every 20 milliSeconds / rotation per execution by 50 to get
    // rotations per second.

    public double turnFeedforward() {
        turretFeedForward = (Math.toDegrees(drivetrainSubsys.getChassisSpeeds().omegaRadiansPerSecond)) / 50;
        return turretFeedForward;
    }

    public double rAlignment() {
        // Rotation error in degrees (positive = tag is to the right, for example)
        double rotationErrorDeg = vision.getTX();

        // Tunable gain: volts per degree
        double kP = 0.1; // example value

        double voltage = rotationErrorDeg * kP;

        // Deadband
        if (Math.abs(voltage) < 0.3) {
            voltage = 0.0;
        }

        // Clamp to legal voltage range
        voltage = Math.max(-12.0, Math.min(12.0, voltage));

        return voltage;
    }

    double distance = 0;
    double angle = 0;

    // Periodic
    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Turret Encoder", getEncoder());
        // SmartDashboard.putNumber("Turret Voltage", voltage);
        // SmartDashboard.putNumber("Set Angle", setAngle);
        // SmartDashboard.putNumber("Turret Set Angle", vision.getTurretTX());
        // SmartDashboard.putNumber("Aim PID",
        // aimController.calculate(vision.getTurretTX(), 0));
        // SmartDashboard.putNumber("GetTy", vision.getTY());
        // turretFeedForward = turnFeedforward();
        // SmartDashboard.putNumber("Turret Feedforward", turretFeedForward); // Bruh
        // SmartDashboard.putNumber("Turret IMUPitch", vision.getTurretIMUPitch());
        angle = Math.toRadians(vision.getTY() + vision.getTurretIMUPitch());
        distance = (44.21875 - 15.625) / Math.tan(angle);
        SmartDashboard.putData(startChooser);
        // SmartDashboard.putNumber("counculatedDist", distance);
        // SmartDashboard.putNumber("Aim Output", aimOutput);
    }

    // Config
    /**
     * Applies the turret motor's safety and neutral-mode configuration.
     *
     * This must run during subsystem construction so the real turret motor uses the
     * current limits and neutral mode that the rest of the code expects.
     */
    private void configTurretHardware() {

        // encoder configs
        var magnetSensorConfigs = new MagnetSensorConfigs();
        magnetSensorConfigs.AbsoluteSensorDiscontinuityPoint = 1;

        // Motor configs
        var motorConfigs = new MotorOutputConfigs();
        var limitConfigs = new CurrentLimitsConfigs();

        limitConfigs.StatorCurrentLimit = 30;
        limitConfigs.SupplyCurrentLimit = 30;
        limitConfigs.SupplyCurrentLimitEnable = true;
        limitConfigs.StatorCurrentLimitEnable = true;

        turretMotor.getConfigurator().apply(limitConfigs);

        motorConfigs.NeutralMode = NeutralModeValue.Coast;
        turretMotor.getConfigurator().apply(motorConfigs);
    }
}
