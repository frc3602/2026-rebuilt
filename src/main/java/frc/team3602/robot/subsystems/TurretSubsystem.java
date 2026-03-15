package frc.team3602.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import frc.team3602.robot.Constants.*;
import frc.team3602.robot.Constants.FieldConstants;

public class TurretSubsystem extends SubsystemBase {
    // The turret uses linear travel coordinates from 0 through 360 degrees.
    // 0/360 degrees point to the rear of the robot, 90 points left, 180 points
    // forward over the intake, and 270 points right. 0 degrees and 360 degrees
    // are the same aiming direction, but they are different ends of the
    // mechanism's legal travel. That means we must not use normal "shortest
    // angle" wrapping when we command the turret.
    private static final double MIN_TRACKING_ANGLE_DEGREES = 0.0;
    private static final double MAX_TRACKING_ANGLE_DEGREES = 360.0;
    private static final double STARTING_TURRET_ANGLE_DEGREES = 90.0;
    private static final double LEFT_CORNER_PRESET_DEGREES = 305.0;
    private static final double RIGHT_CORNER_PRESET_DEGREES = 55.0;
    private static final double NEUTRAL_PRESET_DEGREES = 270.0;
    private static final double MOTOR_ROTATIONS_PER_TURRET_ROTATION = 30.0;
    private static final double TURRET_DEGREES_PER_MOTOR_ROTATION = 360.0
            / MOTOR_ROTATIONS_PER_TURRET_ROTATION;
    // These values are the current best estimates for the turret's moving-shot
    // lead math. They should be updated whenever on-robot testing gives us better
    // measured numbers for the shooter exit angle or release height.
    private static final double ASSUMED_LAUNCH_ANGLE_DEGREES = 70.0;
    private static final double ASSUMED_SHOOTER_HEIGHT_METERS = Units.inchesToMeters(20.0);
    private static final double TOWER_TARGET_HEIGHT_METERS = Units.inchesToMeters(72.0);

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
        seedTurretSensorToStartAngle();
    }

    /**
     * Seeds the Falcon's relative rotor sensor to the team's known startup angle.
     *
     * We do not have an absolute turret sensor yet, so the robot assumes the turret
     * is physically staged at the shared start angle before boot. Seeding the sensor
     * here keeps the software's 0-360 travel model aligned with that operational
     * assumption.
     */
    private void seedTurretSensorToStartAngle() {
        double startMotorRotations = STARTING_TURRET_ANGLE_DEGREES / TURRET_DEGREES_PER_MOTOR_ROTATION;
        turretMotor.setPosition(startMotorRotations);
    }

    // Review note 2026-03-15 10:15:26 -04:00: this method may be unused in the
    // current codebase.
    public double getsetAnlge() {
        return startChooser.getSelected();
    }

    // Motor
    private final TalonFX turretMotor = new TalonFX(TurretConstants.kTurretMotorID, "rio");

    private double turretFeedForward = 0.0;

    // Encoder
    // Review note 2026-03-15 10:15:26 -04:00: this method may be unused in the
    // current codebase.
    public Double getEncoder() {
        return turretMotor.getRotorPosition().getValueAsDouble() * TURRET_DEGREES_PER_MOTOR_ROTATION;
    }

    // Hold the turret at its intended starting angle until another command asks
    // for a different target.
    public double setAngle = STARTING_TURRET_ANGLE_DEGREES;

    // Controllers *These PID values need to be changed*
    private final PIDController turretController = new PIDController(.04, 0.0, 0.0);

    // Commands

    // Review note 2026-03-15 10:15:26 -04:00: this method may be unused in the
    // current codebase.
    public Command changeSetAngle(double newSetpoint) {
        return runOnce(() -> {
            setAngle = clamp(setAngle + newSetpoint, MIN_TRACKING_ANGLE_DEGREES, MAX_TRACKING_ANGLE_DEGREES);
        });
    }

    public Command setAngle(double setPosition) {
        return runOnce(() -> {
            setRequestedAngle(setPosition);
        });
    }

    // Review note 2026-03-15 10:15:26 -04:00: this method may be unused in the
    // current codebase.
    public Command testTurret(double voltage) {
        return runOnce(() -> {
            turretMotor.setVoltage(voltage);
        });

    }

    // Review note 2026-03-15 10:15:26 -04:00: this method may be unused in the
    // current codebase.
    public Command stopTurret() {
        return runOnce(() -> {
            turretMotor.stopMotor();
        });
    }

    public Command moveToStartAngle() {
        return runOnce(() -> {
            // The current start angle is also our handoff/stowed angle, so one
            // command keeps the auton API from exposing two names for the same state.
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
    // Review note 2026-03-15 10:15:26 -04:00: this method may be unused in the
    // current codebase.
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

        // Convert motor rotations into the turret's linear 0-360 travel coordinate.
        double turretDeg = motorRot * TURRET_DEGREES_PER_MOTOR_ROTATION;

        return clamp(turretDeg, MIN_TRACKING_ANGLE_DEGREES, MAX_TRACKING_ANGLE_DEGREES);
    }

    /**
     * Calculates the turret angle needed to face the field target.
     *
     * The turret aims by comparing the robot's corrected field pose to the target's
     * field coordinates. This lets vision-based pose corrections improve aiming
     * immediately instead of waiting for a separate odometry-only estimate.
     */
    // Review note 2026-03-15 10:15:26 -04:00: this method may be unused in the
    // current codebase.
    public double calculateDesiredAngle() {

        Pose2d robotPose = drivetrainSubsys.getEstimatedPose();

        Translation2d target = getTargetPose();

        double dx = target.getX() - robotPose.getX();
        double dy = target.getY() - robotPose.getY();
        double fieldAngle = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeading = robotPose.getRotation().getDegrees();
        double signedRobotRelativeAngle = normalizeSignedRobotRelativeAngle(fieldAngle - robotHeading);
        return convertSignedAimToTravelAngle(signedRobotRelativeAngle);
    }

    /**
     * Converts any direct turret travel request into the turret's linear 0-360
     * travel range.
     *
     * Examples:
     * - -55 becomes 305
     * - 370 becomes 10
     * - 360 stays 360 so the code can still command the upper travel limit
     */
    private double normalizeToTravelAngle(double angleDeg) {
        double normalizedAngle = angleDeg % 360.0;
        if (normalizedAngle < 0.0) {
            normalizedAngle += 360.0;
        }

        // If the request was exactly one full rotation, keep it at the top end of the
        // legal range instead of collapsing it back to zero.
        if (Math.abs(normalizedAngle) < 1e-9 && angleDeg > 0.0) {
            return MAX_TRACKING_ANGLE_DEGREES;
        }

        return normalizedAngle;
    }

    /**
     * Wraps a robot-relative aim angle into the normal signed range used by field
     * geometry.
     *
     * In this signed frame:
     * - 0 means straight ahead
     * - +90 means left
     * - -90 means right
     * - +/-180 means directly behind the robot
     */
    private double normalizeSignedRobotRelativeAngle(double angleDeg) {
        double normalizedAngle = angleDeg % 360.0;
        if (normalizedAngle >= 180.0) {
            normalizedAngle -= 360.0;
        }
        if (normalizedAngle < -180.0) {
            normalizedAngle += 360.0;
        }

        return normalizedAngle;
    }

    /**
     * Converts a signed robot-relative aim angle into the turret's rear-zero travel
     * coordinates.
     *
     * In the turret travel model:
     * - rear = 0 or 360
     * - left = 90
     * - front = 180
     * - right = 270
     */
    private double convertSignedAimToTravelAngle(double signedAimAngleDegrees) {
        if (Math.abs(Math.abs(signedAimAngleDegrees) - 180.0) < 1e-9) {
            return getTurretAngleDeg() > 180.0 ? MAX_TRACKING_ANGLE_DEGREES : MIN_TRACKING_ANGLE_DEGREES;
        }

        return normalizeToTravelAngle(180.0 - signedAimAngleDegrees);
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
        double normalizedAngle = normalizeToTravelAngle(requestedAngleDegrees);
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
        // Estimate how long the fuel is in the air so the turret can lead the target
        // while the robot is moving. This is only an estimate, so the launch angle and
        // shooter height must stay synced with what the real mechanism does on the
        // robot.
        double ballVelocity = getBallVelocity();
        if (ballVelocity <= 0.0) {
            return 0.0;
        }

        // Convert launch angle from degrees to radians (Java's trig functions use
        // radians)
        double launchAngleRadians = Math.toRadians(ASSUMED_LAUNCH_ANGLE_DEGREES);

        // Calculate the vertical component of velocity
        double vVertical = ballVelocity * Math.sin(launchAngleRadians);

        // Calculate height difference (positive if target is higher than shooter)
        double heightDifference = TOWER_TARGET_HEIGHT_METERS - ASSUMED_SHOOTER_HEIGHT_METERS;

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

        // Get the robot's chassis speeds. CTRE reports these in robot-relative
        // coordinates, so +X is forward relative to the robot and +Y is left.
        ChassisSpeeds robotRelativeSpeeds = drivetrainSubsys.getState().Speeds;

        // Rotate the robot-relative translation speeds into the field frame.
        // This lets us measure the robot's motion relative to the tower target
        // instead of only looking at "forward" speed from the robot's perspective.
        Translation2d fieldRelativeVelocity = new Translation2d(
                robotRelativeSpeeds.vxMetersPerSecond,
                robotRelativeSpeeds.vyMetersPerSecond).rotateBy(robotPose.getRotation());

        // Time of flight
        double timeOfFlight = calculateBallTimeOfFlight();

        double signedRobotRelativeAngleDegrees = normalizeSignedRobotRelativeAngle(
                Math.toDegrees(targetYawRelative));

        if (timeOfFlight <= 0.0) {
            return convertSignedAimToTravelAngle(signedRobotRelativeAngleDegrees);
        }

        // Build a unit vector perpendicular to the shot line.
        // Motion along this perpendicular direction is what makes us need lead.
        Translation2d targetDirectionUnit = robotToTarget.div(distanceToTarget);
        Translation2d lateralDirectionUnit = new Translation2d(
                -targetDirectionUnit.getY(),
                targetDirectionUnit.getX());

        // Project the robot's field-relative velocity onto the lateral direction so
        // diagonal and sideways motion both affect the lead angle correctly.
        double lateralSpeedMetersPerSecond = (fieldRelativeVelocity.getX() * lateralDirectionUnit.getX())
                + (fieldRelativeVelocity.getY() * lateralDirectionUnit.getY());

        // Lateral movement during the shot.
        double lateralMovement = lateralSpeedMetersPerSecond * timeOfFlight;

        // Lead angle
        double leadRadians = Math.atan2(lateralMovement, distanceToTarget);

        // Final turret offset
        double turretOffsetRadians = targetYawRelative + leadRadians;

        double signedLeadAdjustedAngleDegrees = normalizeSignedRobotRelativeAngle(
                Math.toDegrees(turretOffsetRadians));

        return convertSignedAimToTravelAngle(signedLeadAdjustedAngleDegrees);
    }

    double voltage;

    public boolean atTarget;

    public Command aimCommand() {
        return run(() -> {
            // Command the full lead-adjusted tower angle directly and keep applying
            // turret PID while this command owns the subsystem.
            setRequestedAngle(calculateTurretOffset());
            applyTurretPositionControl();
        });
    }

    public Command setAngleLeftCorner() {
        return  runOnce(() -> {
            setRequestedAngle(LEFT_CORNER_PRESET_DEGREES);
        });
    }

    public Command setAngleRightCorner() {
        return  runOnce(() -> {
            setRequestedAngle(RIGHT_CORNER_PRESET_DEGREES);
        });
    }

    public Command setAngleNeutral() {
        return  runOnce(() -> {
            // Neutral parks the turret on the robot's right side in the rear-zero
            // travel model.
            setRequestedAngle(NEUTRAL_PRESET_DEGREES);
        });
    }

    public Command setAngleZero() {
        return  runOnce(() -> {
            // Zero is the rear-left end of travel in this turret convention.
            setRequestedAngle(0);
        });
    }

    // Review note 2026-03-15 10:15:26 -04:00: this method may be unused in the
    // current codebase.
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
    // Review note 2026-03-15 10:15:26 -04:00: this method may be unused in the
    // current codebase.
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
     * We first compute the normal signed robot-relative aim angle, then convert it
     * into the turret's rear-zero travel coordinate. This keeps the field math easy
     * to reason about while still honoring the mechanism's 0/360 rear seam.
     */
    private double calculateTurretAngleForFieldPoint(Translation2d fieldPoint) {
        Pose2d robotPose = drivetrainSubsys.getEstimatedPose();

        double deltaX = fieldPoint.getX() - robotPose.getX();
        double deltaY = fieldPoint.getY() - robotPose.getY();
        double fieldAngleToPoint = Math.toDegrees(Math.atan2(deltaY, deltaX));
        double robotHeading = robotPose.getRotation().getDegrees();
        double signedRobotRelativeAngle = normalizeSignedRobotRelativeAngle(fieldAngleToPoint - robotHeading);
        double turretTravelAngle = convertSignedAimToTravelAngle(signedRobotRelativeAngle);

        return clamp(turretTravelAngle, MIN_TRACKING_ANGLE_DEGREES, MAX_TRACKING_ANGLE_DEGREES);
    }

    public Command aimToDesiredAngle() {
        return run(() -> {
            // Continuously point at the lead-adjusted tower angle while the driver
            // holds the button.
            setRequestedAngle(calculateTurretOffset());
            applyTurretPositionControl();
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
        var pidEffort = turretController.calculate(getTurretAngleDeg(), setAngle);
        turretMotor.setVoltage(pidEffort);
    }

    // Review note 2026-03-15 10:15:26 -04:00: this method may be unused in the
    // current codebase.
    public double turnFeedforward() {
        turretFeedForward = (Math.toDegrees(drivetrainSubsys.getChassisSpeeds().omegaRadiansPerSecond)) / 50;
        return turretFeedForward;
    }

    // Periodic
    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Turret Encoder", getEncoder());
        // SmartDashboard.putNumber("Turret Voltage", voltage);
        // SmartDashboard.putNumber("Set Angle", setAngle);
        // turretFeedForward = turnFeedforward();
        // SmartDashboard.putNumber("Turret Feedforward", turretFeedForward); // Bruh
        SmartDashboard.putData(startChooser);
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

        // Brake mode helps the turret hold its angle when shooter vibration or
        // robot motion tries to nudge the mechanism.
        motorConfigs.NeutralMode = NeutralModeValue.Brake;
        turretMotor.getConfigurator().apply(motorConfigs);
    }
}
