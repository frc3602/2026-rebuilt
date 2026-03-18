package frc.team3602.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import frc.team3602.robot.Constants.*;
import frc.team3602.robot.Constants.FieldConstants;

public class TurretSubsystem extends SubsystemBase {
    // Public turret angles follow WPILib's normal robot-relative convention:
    // 0 degrees = forward, +90 = left, -90 = right, and 180 = directly backward.
    //
    // The mechanism still has a numerical seam at the rear, though. Physically,
    // the turret can point backward by traveling to 0 degrees or to 360 degrees,
    // and those
    // are different legal endpoints for the motor. To keep that seam behavior
    // explicit, the subsystem uses two angle representations:
    // - signed "aim angles" for geometry, commands, and documentation
    // - 0-360 "travel angles" for the motor's actual motion limits
    private static final double MIN_TRAVEL_ANGLE_DEGREES = 0.0;
    private static final double MAX_TRAVEL_ANGLE_DEGREES = 360.0;
    private static final double STARTING_TURRET_AIM_ANGLE_DEGREES = 90.0;
    private static final double REAR_LEFT_CORNER_PRESET_AIM_DEGREES = 130.0;
    private static final double REAR_RIGHT_CORNER_PRESET_AIM_DEGREES = -130.0;
    private static final double RIGHT_PRESET_AIM_DEGREES = -90.0;
    private static final double REAR_PRESET_AIM_DEGREES = 180.0;
    private static final double MOTOR_ROTATIONS_PER_TURRET_ROTATION = 30.0;
    private static final double TURRET_DEGREES_PER_MOTOR_ROTATION = 360.0
            / MOTOR_ROTATIONS_PER_TURRET_ROTATION;
    // When the target direction is very close to the rear seam, keep the turret on
    // its current side of travel instead of letting tiny pose noise flip the
    // setpoint from 0 to 360 or back again.
    private static final double REAR_SEAM_DEADBAND_DEGREES = 2.0;
    // These values are the current best estimates for the turret's moving-shot
    // lead math. They should be updated whenever on-robot testing gives us better
    // measured numbers for the shooter exit angle or release height.
    private static final double ASSUMED_LAUNCH_ANGLE_DEGREES = 70.0;
    private static final double ASSUMED_SHOOTER_HEIGHT_METERS = Units.inchesToMeters(20.0);
    private static final double TOWER_TARGET_HEIGHT_METERS = Units.inchesToMeters(72.0);
    private static final double SHOT_READY_ANGLE_TOLERANCE_DEGREES = 1.0;
    private static final double MIN_TARGET_DISTANCE_FOR_LEAD_METERS = 1e-3;
    private static final double MIN_ACTIVE_CORRECTION_ERROR_DEGREES = 3.0;
    private static final double MIN_ACTIVE_CORRECTION_VOLTAGE = 0.45;
    private static final boolean ALLOW_DIRECT_VISION_POSE_FOR_AIMING = false;
    private static final double AIM_VISION_POSE_MAX_AGE_SECONDS = 0.20;
    private static final double AIM_VISION_MAX_LINEAR_SPEED_METERS_PER_SECOND = 0.75;
    private static final double AIM_VISION_MAX_YAW_RATE_DEGREES_PER_SECOND = 120.0;

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
        turretController.setTolerance(SHOT_READY_ANGLE_TOLERANCE_DEGREES, 2);
        setRequestedAngle(STARTING_TURRET_AIM_ANGLE_DEGREES);
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
        double startTravelAngleDegrees = convertSignedAimToTravelAngle(STARTING_TURRET_AIM_ANGLE_DEGREES);
        double startMotorRotations = startTravelAngleDegrees / TURRET_DEGREES_PER_MOTOR_ROTATION;
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
    private double requestedTurretAimAngleDegrees;
    private double requestedTurretTravelAngleDegrees;
    private double requestedTurretControlTravelAngleDegrees;

    // Controllers
    // Tracking was starting to oscillate once the turret began following live pose
    // updates, so we back the proportional gain down to keep the mechanism calmer
    // around the target instead of aggressively chasing every small angle change.
    private final PIDController turretController = new PIDController(.04, 0.0, 0.0);

    /**
     * Checks whether the turret is pointed close enough to the requested angle for
     * shooting.
     *
     * The travel model treats 0 and 360 as different travel endpoints, but for a
     * shot they are the same aiming direction. Because of that, this method uses
     * the smallest circular angle error instead of the raw travel-coordinate error.
     */
    public boolean isAtRequestedAngle() {
        double angleErrorDegrees = Math.abs(
                normalizeSignedAimAngleDegrees(getTurretAngleDegrees() - requestedTurretAimAngleDegrees));
        return angleErrorDegrees <= SHOT_READY_ANGLE_TOLERANCE_DEGREES;
    }

    // Commands

    // Review note 2026-03-15 10:15:26 -04:00: this method may be unused in the
    // current codebase.
    public Command changeSetAngle(double newSetpoint) {
        return runOnce(() -> {
            setRequestedAngle(requestedTurretAimAngleDegrees + newSetpoint);
        });
    }

    /**
     * Commands a new turret aim angle using WPILib's signed robot-relative
     * convention.
     *
     * Public callers should think in normal FRC geometry:
     * - 0 = forward
     * - +90 = left
     * - -90 = right
     * - 180 = straight backward
     *
     * The subsystem converts that signed aim angle into the mechanism's private
     * 0-360 travel coordinates before applying motor control.
     */
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

    public Command autonStowTurret() {
        return runOnce(() -> {
            // The current start angle is also our handoff/stowed angle, so one
            // command keeps the auton API from exposing two names for the same state.
            setRequestedAngle(STARTING_TURRET_AIM_ANGLE_DEGREES);
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
        Pose2d robotPose = getPoseForTurretAiming();

        // Target field location (meters).
        Translation2d targetPosition = getTargetPose();

        // Robot position on the field.
        Translation2d robotPosition = robotPose.getTranslation();

        // Straight-line distance from the robot to the target.
        double distance = robotPosition.getDistance(targetPosition);

        double distanceFeet = Units.metersToFeet(distance);

        return distanceFeet;
    }

    private double getTurretUnwrappedTravelAngleDegrees() {
        // Get rotor position in motor rotations
        double motorRot = turretMotor.getRotorPosition().getValueAsDouble();

        // Convert motor rotations into the turret's continuous travel coordinate.
        // We keep this unwrapped for control so the turret can move smoothly across
        // the rear seam without the measurement snapping from 0 to 360 or back.
        return motorRot * TURRET_DEGREES_PER_MOTOR_ROTATION;
    }

    private double getTurretTravelAngleDegrees() {
        return normalizeTravelAngle(getTurretUnwrappedTravelAngleDegrees());
    }

    /**
     * Returns the turret's current robot-relative aim angle in WPILib convention.
     *
     * This is the angle students should use for geometry reasoning, logs, and
     * command setpoints:
     * - 0 = forward
     * - +90 = left
     * - -90 = right
     * - 180 = straight backward
     */
    public double getTurretAngleDegrees() {
        return convertTravelAngleToSignedAimAngle(getTurretTravelAngleDegrees());
    }

    /**
     * Returns the turret's currently requested public aim angle.
     *
     * This is the student-facing setpoint in the standard signed WPILib
     * convention, so it is the easiest number to compare against the real turret
     * direction during commissioning.
     */
    public double getRequestedTurretAimAngleDegrees() {
        return requestedTurretAimAngleDegrees;
    }

    /**
     * Returns the turret's currently requested internal travel angle.
     *
     * This is mainly useful when debugging rear-seam behavior. Most day-to-day
     * tuning should focus on the public signed aim angle instead.
     */
    public double getRequestedTurretTravelAngleDegrees() {
        return requestedTurretTravelAngleDegrees;
    }

    /**
     * Returns the field position of the tower target currently selected for turret
     * tracking.
     *
     * Publishing this as a helper keeps the SmartDashboard code easy to read and
     * gives operators a direct way to confirm which alliance-side target the turret
     * believes it should be aiming at.
     */
    public Translation2d getCurrentTargetFieldPosition() {
        return getTargetPose();
    }

    /**
     * Returns the signed aiming error between the requested and measured turret
     * direction.
     *
     * A positive value means the turret still needs to move farther left in the
     * public WPILib convention. A negative value means it still needs to move
     * farther right.
     */
    public double getTurretAimErrorDegrees() {
        return normalizeSignedAimAngleDegrees(requestedTurretAimAngleDegrees - getTurretAngleDegrees());
    }

    /**
     * Returns the current direct robot-relative angle from the turret to the alliance
     * tower.
     *
     * This value does not include moving-shot lead. It is useful for dashboard
     * debugging because it answers the simple question, "where does the code think
     * the tower is right now?"
     */
    public double getCurrentTowerAimAngleDegrees() {
        return calculateDesiredAngle();
    }

    /**
     * Calculates the turret angle needed to face the field target.
     *
     * The turret aims by comparing the robot's corrected field pose to the target's
     * field coordinates. The returned value uses the normal signed WPILib
     * robot-relative convention so it is easy to compare with other FRC geometry.
     */
    // Review note 2026-03-15 10:15:26 -04:00: this method may be unused in the
    // current codebase.
    public double calculateDesiredAngle() {

        Pose2d robotPose = getPoseForTurretAiming();

        Translation2d target = getTargetPose();

        double dx = target.getX() - robotPose.getX();
        double dy = target.getY() - robotPose.getY();
        double fieldAngle = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeading = robotPose.getRotation().getDegrees();
        return normalizeSignedAimAngleDegrees(fieldAngle - robotHeading);
    }

    /**
     * Converts any direct internal travel-angle request into the turret's linear
     * 0-360 travel range.
     *
     * Examples:
     * - -55 becomes 305
     * - 370 becomes 10
     * - 360 stays 360 so the code can still command the upper travel limit
     */
    private double normalizeTravelAngle(double angleDeg) {
        double normalizedAngle = angleDeg % 360.0;
        if (normalizedAngle < 0.0) {
            normalizedAngle += 360.0;
        }

        // If the request was exactly one full rotation, keep it at the top end of the
        // legal range instead of collapsing it back to zero.
        if (Math.abs(normalizedAngle) < 1e-9 && angleDeg > 0.0) {
            return MAX_TRAVEL_ANGLE_DEGREES;
        }

        return normalizedAngle;
    }

    /**
     * Wraps a robot-relative aim angle into the standard signed range used by FRC
     * field geometry.
     *
     * In this signed WPILib-style frame:
     * - 0 means straight ahead
     * - +90 means left
     * - -90 means right
     * - 180 means directly behind the robot
     */
    private double normalizeSignedAimAngleDegrees(double angleDeg) {
        double normalizedAngle = angleDeg % 360.0;
        if (normalizedAngle > 180.0) {
            normalizedAngle -= 360.0;
        }
        if (normalizedAngle <= -180.0) {
            normalizedAngle += 360.0;
        }

        return normalizedAngle;
    }

    /**
     * Converts a signed WPILib-style aim angle into the turret's internal travel
     * coordinates.
     *
     * Public aim math uses:
     * - 0 = forward
     * - +90 = left
     * - -90 = right
     * - 180 = rear
     *
     * The internal travel model uses:
     * - rear = 0 or 360
     * - left = 90
     * - front = 180
     * - right = 270
     */
    private double convertSignedAimToTravelAngle(double signedAimAngleDegrees) {
        double normalizedAimAngle = normalizeSignedAimAngleDegrees(signedAimAngleDegrees);

        // If the target is very close to straight rear, prefer the current side of
        // the rear seam so tiny pose jitter does not bounce the turret between the
        // two travel endpoints.
        double distanceFromRearDegrees = 180.0 - Math.abs(normalizedAimAngle);
        if (distanceFromRearDegrees <= REAR_SEAM_DEADBAND_DEGREES) {
            return getTurretTravelAngleDegrees() > 180.0
                    ? MAX_TRAVEL_ANGLE_DEGREES
                    : MIN_TRAVEL_ANGLE_DEGREES;
        }

        return normalizeTravelAngle(180.0 - normalizedAimAngle);
    }

    /**
     * Converts the mechanism's internal travel coordinates back into a standard
     * signed WPILib-style aim angle.
     *
     * This is the inverse of {@link #convertSignedAimToTravelAngle(double)} for all
     * non-seam directions. For the rear seam, both travel endpoints map back to the
     * same public answer: 180 degrees means "straight backward."
     */
    private double convertTravelAngleToSignedAimAngle(double travelAngleDegrees) {
        double normalizedTravelAngle = normalizeTravelAngle(travelAngleDegrees);
        double signedAimAngleDegrees = 180.0 - normalizedTravelAngle;
        return normalizeSignedAimAngleDegrees(signedAimAngleDegrees);
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
        requestedTurretAimAngleDegrees = normalizeSignedAimAngleDegrees(requestedAngleDegrees);
        requestedTurretTravelAngleDegrees = clamp(
                convertSignedAimToTravelAngle(requestedTurretAimAngleDegrees),
                MIN_TRAVEL_ANGLE_DEGREES,
                MAX_TRAVEL_ANGLE_DEGREES);
        requestedTurretControlTravelAngleDegrees = chooseLegalEquivalentTravelAngle(
                getTurretUnwrappedTravelAngleDegrees(),
                requestedTurretTravelAngleDegrees);
    }

    /**
     * Chooses a legal travel target while avoiding the 0/360 seam crossing.
     *
     * The turret's legal travel lives in a single 0-360 window, but 0 and 360 are
     * physically the same edge of travel. When a request lands on the opposite side
     * of that seam, we keep the far-side target and let the position controller
     * drive the long way around through the middle of travel instead of trying to
     * "wrap" through 0.
     */
    private double chooseLegalEquivalentTravelAngle(double currentTravelDegrees, double normalizedTargetTravelDegrees) {
        double clampedTargetTravelDegrees = clamp(
                normalizedTargetTravelDegrees,
                MIN_TRAVEL_ANGLE_DEGREES,
                MAX_TRAVEL_ANGLE_DEGREES);
        return clampedTargetTravelDegrees;
    }

    public double getBallVelocity() {
        // If the shooter subsystem was not linked in, we cannot estimate flight time.
        // Returning zero keeps the turret code safe and tells the lead calculation to
        // fall back to "aim directly at the target with no motion lead."
        if (shooter == null) {
            return 0.0;
        }

        // Shooter commands currently use negative RPS to match motor direction, but
        // fuel launch speed should always be treated as a positive magnitude.
        double motorRPS = Math.abs(shooter.getVelocity());

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
     * full signed robot-relative aim angle that the turret should try to reach.
     */
    public double calculateTurretOffset() {

        // Get the freshest pose we want turret aiming to trust right now.
        Pose2d robotPose = getPoseForTurretAiming();

        // Target position on field (meters)
        Translation2d targetPosition = getTargetPose();

        // Robot position
        Translation2d robotPosition = robotPose.getTranslation();

        // Vector from robot to target
        Translation2d robotToTarget = targetPosition.minus(robotPosition);

        // Distance to target
        double distanceToTarget = robotToTarget.getNorm();

        // If the target point and robot point collapse onto nearly the same field
        // position, the shot direction is effectively undefined. Keep the current
        // request instead of dividing by a tiny number and creating invalid math.
        if (distanceToTarget <= MIN_TARGET_DISTANCE_FOR_LEAD_METERS) {
            return requestedTurretAimAngleDegrees;
        }

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

        double signedRobotRelativeAngleDegrees = normalizeSignedAimAngleDegrees(
                Math.toDegrees(targetYawRelative));

        if (timeOfFlight <= 0.0) {
            return signedRobotRelativeAngleDegrees;
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

        // The ball inherits the robot's sideways velocity as it leaves the shooter.
        // Because of that, we aim opposite the robot's lateral motion so the shot
        // drifts back onto the tower instead of farther away from it.
        double turretOffsetRadians = targetYawRelative - leadRadians;

        double signedLeadAdjustedAngleDegrees = normalizeSignedAimAngleDegrees(
                Math.toDegrees(turretOffsetRadians));

        return signedLeadAdjustedAngleDegrees;
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
            // This preset points into the back-left corner in the standard
            // robot-relative WPILib frame.
            setRequestedAngle(REAR_LEFT_CORNER_PRESET_AIM_DEGREES);
        });
    }

    public Command setAngleRightCorner() {
        return  runOnce(() -> {
            // This preset points into the back-right corner in the standard
            // robot-relative WPILib frame.
            setRequestedAngle(REAR_RIGHT_CORNER_PRESET_AIM_DEGREES);
        });
    }

    public Command setAngleRight() {
        return  runOnce(() -> {
            // This preset points directly to the robot's right side.
            setRequestedAngle(RIGHT_PRESET_AIM_DEGREES);
        });
    }

    public Command setAngleRear() {
        return  runOnce(() -> {
            // This preset points straight backward. The internal seam-handling
            // helper will choose whether the mechanism should stay on the 0-degree
            // side or the 360-degree side of travel.
            setRequestedAngle(REAR_PRESET_AIM_DEGREES);
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
     * This helper returns the standard signed robot-relative aim angle used by
     * WPILib geometry. The caller can then hand that angle to
     * {@link #setRequestedAngle(double)}, which will convert it into the internal
     * travel coordinates needed by the real mechanism.
     */
    private double calculateTurretAngleForFieldPoint(Translation2d fieldPoint) {
        Pose2d robotPose = getPoseForTurretAiming();

        double deltaX = fieldPoint.getX() - robotPose.getX();
        double deltaY = fieldPoint.getY() - robotPose.getY();
        double fieldAngleToPoint = Math.toDegrees(Math.atan2(deltaY, deltaX));
        double robotHeading = robotPose.getRotation().getDegrees();
        return normalizeSignedAimAngleDegrees(fieldAngleToPoint - robotHeading);
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
        double currentTravelDegrees = getTurretUnwrappedTravelAngleDegrees();
        double travelErrorDegrees = requestedTurretControlTravelAngleDegrees - currentTravelDegrees;

        // Control against the unwrapped travel measurement so the turret can cross
        // the rear seam smoothly without losing position continuity at 0/360.
        var pidEffort = turretController.calculate(currentTravelDegrees, requestedTurretControlTravelAngleDegrees);

        // If the turret is still clearly off target but the pure PID effort is too
        // small to overcome friction, enforce a small minimum correction voltage.
        // This helps the turret finish the move instead of stalling several degrees
        // away from the tower.
        if (Math.abs(travelErrorDegrees) > MIN_ACTIVE_CORRECTION_ERROR_DEGREES
                && Math.abs(pidEffort) < MIN_ACTIVE_CORRECTION_VOLTAGE) {
            pidEffort = Math.copySign(MIN_ACTIVE_CORRECTION_VOLTAGE, travelErrorDegrees);
        }

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
        // Publish a small set of high-value tuning numbers every loop.
        //
        // We expose both the public signed aim angles and the private travel
        // angles:
        // - Aim angles are easiest for students and operators to reason about.
        // - Travel angles help when debugging behavior around the rear seam.
        //
        // Angle error and motor voltage answer the two most common pit questions:
        // "Is the turret being asked to go where I think?" and
        // "Is the controller actually trying to move it there?"
        SmartDashboard.putNumber("Turret/MeasuredAimDegrees", getTurretAngleDegrees());
        SmartDashboard.putNumber("Turret/RequestedAimDegrees", getRequestedTurretAimAngleDegrees());
        SmartDashboard.putNumber("Turret/AimErrorDegrees", getTurretAimErrorDegrees());
        SmartDashboard.putNumber("Turret/MeasuredTravelDegrees", getTurretTravelAngleDegrees());
        SmartDashboard.putNumber("Turret/RequestedTravelDegrees", getRequestedTurretTravelAngleDegrees());
        SmartDashboard.putNumber("Turret/TargetFieldX", getCurrentTargetFieldPosition().getX());
        SmartDashboard.putNumber("Turret/TargetFieldY", getCurrentTargetFieldPosition().getY());
        SmartDashboard.putNumber("Turret/TowerAimDegrees", getCurrentTowerAimAngleDegrees());
        SmartDashboard.putString("Turret/AimPoseSource", getAimPoseSourceLabel());
        SmartDashboard.putNumber("Turret/AimPoseAgeSeconds", getAimPoseAgeSeconds());
        SmartDashboard.putBoolean("Turret/AtRequestedAngle", isAtRequestedAngle());
        SmartDashboard.putNumber("Turret/MotorVoltage", turretMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putData(startChooser);
    }

    /**
     * Returns the pose source the turret should use for aiming right now.
     *
     * We normally want all robot systems to agree on the drivetrain estimator pose,
     * but turret aiming benefits from a fresher camera correction when one is
     * available. If a recent accepted Limelight pose exists, we use it directly for
     * turret geometry so the requested angle reacts immediately instead of waiting
     * for the estimator to blend that correction in over several loops.
     */
    private Pose2d getPoseForTurretAiming() {
        Limelight_Pose limelightPose = Limelight_Pose.getInstance();

        if (ALLOW_DIRECT_VISION_POSE_FOR_AIMING && canUseVisionPoseForAiming(limelightPose)) {
            return limelightPose.poseCamEstimate.pose;
        }

        return drivetrainSubsys.getEstimatedPose();
    }

    /**
     * Returns whether the latest accepted Limelight pose is fresh enough to use for
     * turret aiming.
     *
     * We keep this separate from the drivetrain estimator on purpose. A turret only
     * needs the newest believable robot pose so it can point quickly, while the full
     * estimator is intentionally smoother because it also supports driving and
     * autonomous path following.
     */
    private boolean canUseVisionPoseForAiming(Limelight_Pose limelightPose) {
        if (limelightPose.poseCamEstimate == null) {
            return false;
        }

        double poseAgeSeconds = Timer.getFPGATimestamp() - limelightPose.poseCamEstimate.timestampSeconds;
        if (poseAgeSeconds > AIM_VISION_POSE_MAX_AGE_SECONDS) {
            return false;
        }

        ChassisSpeeds robotRelativeSpeeds = drivetrainSubsys.getState().Speeds;
        double linearSpeedMetersPerSecond = Math.hypot(
                robotRelativeSpeeds.vxMetersPerSecond,
                robotRelativeSpeeds.vyMetersPerSecond);
        double yawRateDegreesPerSecond = Math.abs(
                drivetrainSubsys.getPigeon2().getAngularVelocityZWorld().getValueAsDouble());

        return linearSpeedMetersPerSecond <= AIM_VISION_MAX_LINEAR_SPEED_METERS_PER_SECOND
                && yawRateDegreesPerSecond <= AIM_VISION_MAX_YAW_RATE_DEGREES_PER_SECOND;
    }

    /**
     * Returns a short label describing whether turret aiming is currently using the
     * freshest Limelight pose or the drivetrain estimator pose.
     */
    private String getAimPoseSourceLabel() {
        if (ALLOW_DIRECT_VISION_POSE_FOR_AIMING && canUseVisionPoseForAiming(Limelight_Pose.getInstance())) {
            return "Vision";
        }

        return "Estimator";
    }

    /**
     * Returns the age of the current accepted vision pose.
     *
     * This helps pit-side debugging because the team can tell whether turret aiming
     * is falling back to the drivetrain estimate due to stale camera data.
     */
    private double getAimPoseAgeSeconds() {
        Limelight_Pose limelightPose = Limelight_Pose.getInstance();

        if (limelightPose.poseCamEstimate == null) {
            return -1.0;
        }

        return Math.max(0.0, Timer.getFPGATimestamp() - limelightPose.poseCamEstimate.timestampSeconds);
    }

    // Config
    /**
     * Applies the turret motor's safety and neutral-mode configuration.
     *
     * This must run during subsystem construction so the real turret motor uses the
     * current limits and neutral mode that the rest of the code expects.
     */
    private void configTurretHardware() {

        // This config block is kept as a placeholder for a future magnet-based
        // absolute turret sensor or homing reference path.
        //
        // The current robot does not apply or use these settings yet because turret
        // position is still seeded from the known startup angle instead of a live
        // absolute sensor. Keeping the object here makes that future integration
        // easier to resume without re-discovering the intended config values.
        // encoder configs
        var magnetSensorConfigs = new MagnetSensorConfigs();
        magnetSensorConfigs.AbsoluteSensorDiscontinuityPoint = 1;

        // Motor configs
        var motorConfigs = new MotorOutputConfigs();
        var limitConfigs = new CurrentLimitsConfigs();
        var softwareLimitConfigs = new SoftwareLimitSwitchConfigs();

        limitConfigs.StatorCurrentLimit = 30;
        limitConfigs.SupplyCurrentLimit = 30;
        limitConfigs.SupplyCurrentLimitEnable = true;
        limitConfigs.StatorCurrentLimitEnable = true;

        turretMotor.getConfigurator().apply(limitConfigs);

        // Software limits mirror the turret's 0-360 travel model.
        // This is a safety backstop: if any command math ever requests motion
        // outside the legal turret window, Talon firmware will block that output
        // instead of pushing the mechanism into a hard stop.
        softwareLimitConfigs.ReverseSoftLimitEnable = true;
        softwareLimitConfigs.ForwardSoftLimitEnable = true;
        softwareLimitConfigs.ReverseSoftLimitThreshold = MIN_TRAVEL_ANGLE_DEGREES
            / TURRET_DEGREES_PER_MOTOR_ROTATION;
        softwareLimitConfigs.ForwardSoftLimitThreshold = MAX_TRAVEL_ANGLE_DEGREES
            / TURRET_DEGREES_PER_MOTOR_ROTATION;
        turretMotor.getConfigurator().apply(softwareLimitConfigs);

        // Brake mode helps the turret hold its angle when shooter vibration or
        // robot motion tries to nudge the mechanism.
        motorConfigs.NeutralMode = NeutralModeValue.Brake;
        turretMotor.getConfigurator().apply(motorConfigs);
    }
}
