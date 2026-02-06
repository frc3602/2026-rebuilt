package frc.team3602.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.utility.WheelForceCalculator.Feedforwards;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.team3602.robot.Constants.*;
import frc.team3602.robot.Vision;

public class TurretSubsystem extends SubsystemBase {

    public CommandSwerveDrivetrain drivetrainSubsys;

    public TurretSubsystem(CommandSwerveDrivetrain drivetrainSubsys) {
        this.drivetrainSubsys = drivetrainSubsys;
    }

    // Motor
    private final TalonFX turretMotor = new TalonFX(TurretConstants.kTurretMotorID);
    private final CANcoder turretEncoder = new CANcoder(TurretConstants.kTurretEncoderID);

    public TurretSubsystem() {
        // Zero Encoder
        turretMotor.setPosition(0);

    }

    private double turretFeedForward = 0.0;

    // Encoder
    public Double getEncoder() {
        return (turretMotor.getRotorPosition().getValueAsDouble() * 36); // every revolution is 36 degrees because it is
                                                                         // a 10:1 gear ratio
    }

    // Vision
    public final Vision vision = new Vision();

    // Set Point *This number needs to be changed*
    public double setAngle = 0;

    // Controllers *These PID values need to be changed*
    private final PIDController turretController = new PIDController(.06, 0.0, 0);
    private final PIDController aimController = new PIDController(.04, 0.0, 0);

    private final Feedforwards aimFf = new Feedforwards(0);

    // Commands

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

    double voltage;

    public Command track() {
        return run(() -> {
            if (vision.getTurretHasTarget()) {
                setAngle = setAngle - aimController.calculate(vision.getTurretTX(), 0);
            }
            setAngle = turnFeedforward() + setAngle; // Adds rotational feedforward
            voltage = turretController.calculate(getEncoder(), setAngle);
            if (voltage > .6) {
                voltage = .6;
            } else if (voltage < -.6) {
                voltage = -.6;
            }
            turretMotor.setVoltage(voltage);
        }

        );

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
        SmartDashboard.putNumber("Distance Calculation", vision.getDist());
        SmartDashboard.putNumber("GetTy", vision.getTY());
        // turretFeedForward = turnFeedforward();
        SmartDashboard.putNumber("Turret Feedforward", turretFeedForward); // Bruh
        SmartDashboard.putNumber("Turret IMUPitch", vision.getTurretIMUPitch());
        SmartDashboard.putNumber("GetDistance", vision.getDist());
        angle = Math.toRadians(vision.getTY() + vision.getTurretIMUPitch());
        distance =(66.25 - 16.75) / Math.tan(angle);
        SmartDashboard.putNumber("counculatedDist", distance);
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