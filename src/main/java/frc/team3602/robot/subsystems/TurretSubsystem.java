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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import frc.team3602.robot.Constants.*;
import frc.team3602.robot.LimelightHelpers.PoseEstimate;
import frc.team3602.robot.Vision;
import frc.team3602.robot.subsystems.ShooterSubsystem;

public class TurretSubsystem extends SubsystemBase {

    public CommandSwerveDrivetrain drivetrainSubsys;
    public ShooterSubsystem shooter;

    public TurretSubsystem(CommandSwerveDrivetrain drivetrainSubsys) {
        this.drivetrainSubsys = drivetrainSubsys;
    }

    // Motor
    private final TalonFX turretMotor = new TalonFX(TurretConstants.kTurretMotorID, "rio");

    public TurretSubsystem() {
        // Zero Encoder
        turretMotor.setPosition(0);

    }

    private double turretFeedForward = 0.0;

    // Encoder
    public Double getEncoder() {
        return (turretMotor.getRotorPosition().getValueAsDouble() * 12); // every revolution is 12 degrees because it is
                                                                         // a 30:1 gear ratio 10:1 from gear, 3:1 from gear box
    }

    // Vision
    public final Vision vision = new Vision();

    // Set Point *This number needs to be changed*
    public double setAngle = 0;

    // Controllers *These PID values need to be changed*
    private final PIDController turretController = new PIDController(.04, 0.0, 0.0);
    private final PIDController aimController = new PIDController(.04, 0.0, 0);

    private final Feedforwards aimFf = new Feedforwards(0);

    // Commands

    public Command changeSetAngle(double newSetpoint) {
        return runOnce(() -> {
        setAngle = setAngle + newSetpoint;
        });
    }

    public Command setAngle(double setPosition) {

        return runOnce(() -> {

            if (setPosition > 90) {
                setAngle = 90;
            } else if (setPosition < -180) {
                setAngle = -180;
            }

            else { // hehe
                this.setAngle = setPosition;
            }
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
            setAngle = setAngle + vision.getTurretTX();
        });
    }

//Pose Aiming
public Translation2d getTargetPose() {
    Optional<Alliance> allianceOpt = DriverStation.getAlliance();

    // Check if an alliance is present
    if (allianceOpt.isPresent()) {
        Alliance alliance = allianceOpt.get(); // unwrap the Optional

        if (alliance == Alliance.Blue) {
            return new Translation2d(0.0, 0.0); // example blue alliance target
        } else if (alliance == Alliance.Red) {
            return new Translation2d(1.0, 1.0); // example red alliance target
        }
    }

    // Fallback if alliance not present or invalid
    return new Translation2d(0.0, 0.0);
}
private final Translation2d TARGET = getTargetPose();

public double getDistanceToTarget() {

    // Robot pose from odometry
    Pose2d robotPose = drivetrainSubsys.getState().Pose;

    // Target field location (meters)
    Translation2d targetPosition = getTargetPose(); // TODO set correct field coordinates

    // Robot position
    Translation2d robotPosition = robotPose.getTranslation();

    // Distance between the two
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

public double calculateDesiredAngle() {

    Pose2d calcRobot = drivetrainSubsys.robotPose ;

    Translation2d target = getTargetPose(); 

    double dx = TARGET.getX() - calcRobot.getX();
    double dy = TARGET.getY() - calcRobot.getY();
    double fieldAngle   = Math.toDegrees(Math.atan2(dy, dx));
    double robotHeading = calcRobot.getRotation().getDegrees();
    return clampAngle(fieldAngle - robotHeading);
}


/**
 * Normalize angle to [-180, 180) degrees
 */
private double clampAngle(double angleDeg) {
    angleDeg = angleDeg % 360.0;   // wrap around
    if (angleDeg > 180.0) angleDeg -= 360.0;
    if (angleDeg <= -180.0) angleDeg += 360.0;
    return angleDeg;
}

public double getBallVelocity() {

    double motorRPS = shooter.getVelocity();

    double gearRatio = 1.0; // example motor:wheel
    double wheelDiameterMeters = 0.1016; // 4 inch wheel

    double wheelRPS = motorRPS / gearRatio;

    double circumference = Math.PI * wheelDiameterMeters;

    double ballVelocity = wheelRPS * circumference;

    return ballVelocity; // meters per second
}



public double calculateBallTimeOfFlight() {
    //Ball Velocity m/s TODO: Must Change
    double ballVelocity = getBallVelocity();
    //Ball Launch Angle degrees TODO: Must Change
    double launchAngleDegrees = 15;
    //Shooter Height meters
    double shooterHeight = 0.4826;
    //Target Height meters
    double targetHeight = 1.8288;
    // Convert launch angle from degrees to radians (Java's trig functions use radians)
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

public double calculateTurretOffset() {

    // Get robot pose
    Pose2d robotPose = drivetrainSubsys.getState().Pose;

    // Target position on field (meters)
    Translation2d targetPosition = new Translation2d(8.27, 4.10); // TODO change to real field location

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
            setAngle = setAngle - calculateDesiredAngle() + calculateTurretOffset();
        }
        );
    }

    // public Command track() {
    //     return run(() -> {
    //         if (vision.getTurretHasTarget()) {
    //             aimOutput = aimController.calculate(vision.getTurretTX(),5);//setpoint is the offset of the turret(temp)
    //             setAngle = setAngle - aimOutput + calculateTurretOffset() ;
    //         }
    //         setAngle = turnFeedforward() + setAngle; // Adds rotational feedforward
    //         atTarget = (Math.abs(setAngle - getEncoder())<0.5);
    //         voltage = turretController.calculate(getEncoder(), setAngle);
    //         if (voltage > 4) {  //TODO: create constant for 2, do not go higher than 2
    //             voltage = 4;
    //         } else if (voltage < -4) {
    //             voltage = -4;
    //         }            turretMotor.setVoltage(voltage);
    //     }

    //     );

    // }

public Command aimToDesiredAngle() {
    return run(() -> {

        // setAngle = aimToDesiredAngle();
    });
}



    // THIS IS IN PROGRESS DO NOT HATE HATER ABE
    public Command passMode() {
        return run(() -> {
            if (vision.getTurretHasTarget()) {
            aimOutput = aimController.calculate(vision.getTurretTX(),5);//setpoint is the offset of the turret(temp)
            setAngle = setAngle - aimOutput + calculateTurretOffset() ;

            } else {
                vision.getPose();
                
            }

        });
    }

    public Command setPosition() {
        return run(() -> {
            var pidEffort = turretController.calculate(getTurretAngleDeg(), setAngle);

            turretMotor.setVoltage(pidEffort);
        });
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
        turretFeedForward = (Math.toDegrees(drivetrainSubsys.getChassisSpeeds().omegaRadiansPerSecond)) /50;
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
        SmartDashboard.putNumber("Turret Encoder", getEncoder());
        SmartDashboard.putNumber("Turret Voltage", voltage);
        SmartDashboard.putNumber("Set Angle", setAngle);
        SmartDashboard.putNumber("Turret Set Angle", vision.getTurretTX());
        SmartDashboard.putNumber("Aim PID", aimController.calculate(vision.getTurretTX(), 0));
        SmartDashboard.putNumber("GetTy", vision.getTY());
        // turretFeedForward = turnFeedforward();
        SmartDashboard.putNumber("Turret Feedforward", turretFeedForward); // Bruh
        SmartDashboard.putNumber("Turret IMUPitch", vision.getTurretIMUPitch());
        angle = Math.toRadians(vision.getTY() + vision.getTurretIMUPitch());
        distance =(44.21875 - 15.625) / Math.tan(angle);
        SmartDashboard.putNumber("counculatedDist", distance);
        SmartDashboard.putNumber("Aim Output", aimOutput);
    }

    // Config
    private void configPivotSubsys() {

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