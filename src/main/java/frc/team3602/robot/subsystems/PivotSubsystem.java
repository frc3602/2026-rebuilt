package frc.team3602.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3602.robot.Constants.IntakeConstants;

public class PivotSubsystem extends SubsystemBase {

    private final TalonFX intakePivot = new TalonFX(IntakeConstants.kIntakePivotID, "rio");
    private final TalonFX intakePivotFollow = new TalonFX(IntakeConstants.kIntakePivotFollowID, "rio");

    public double getPivotEncoder;
    private double pivotSetPoint = 0;

    private double offset = 90;

    public PivotSubsystem() {
        intakePivot.setPosition(0);
        intakePivotFollow.setPosition(0);

        var motorConfig = new MotorOutputConfigs();

        var limitConfig = new CurrentLimitsConfigs();

        limitConfig.StatorCurrentLimit = IntakeConstants.kPivotCurrentLimit;
        limitConfig.StatorCurrentLimitEnable = true;

        intakePivot.getConfigurator().apply(motorConfig);
        intakePivot.getConfigurator().apply(limitConfig);

        intakePivotFollow.getConfigurator().apply(motorConfig);
        intakePivotFollow.getConfigurator().apply(limitConfig);

    }

    // Controllers
    private final PIDController pivotPID = new PIDController(0.15, 0.0, 0.0);
    private final PIDController pivotFollowerPID = new PIDController(0.15, 0.0, 0.0);
    // Commands

    public Double getRightPosition() {
        return (intakePivot.getRotorPosition().getValueAsDouble() / 125) * 360; // every revolution is 0.125 degrees
                                                                                // because it is 125 to 1 gear ratio //
                                                                                // a 30:1 gear ratio 10:1 from gear, 3:1
                                                                                // from gear box
    }

    public Double getLeftPosition() {
        return -(intakePivotFollow.getRotorPosition().getValueAsDouble() / 125) * 360; // every revolution is 0.125
                                                                                       // degrees because it is 125 to 1
                                                                                       // gear ratio // a 30:1 gear
                                                                                       // ratio 10:1 from gear, 3:1 from
                                                                                       // gear box
    }

    public Command smartRaisePivot() {
        return runOnce(() -> pivotSetPoint = pivotSetPoint - offset

        );
    }

    public Command smartDropPivot() {
        return runOnce(() -> pivotSetPoint = pivotSetPoint + offset);
    }

    public Command dumbDropIntake() {
        return runOnce(() -> pivotSetPoint = 104);
    }

    public Command dumbRaiseIntake() {
        return runOnce(() -> pivotSetPoint = 20);
    }

    public Boolean isRightDown() {
        return (getPivotEncoder > 90);
    }

    public Boolean isRightUP() {
        return (getPivotEncoder < 0);
    }

    public Command runRightPivot(double power) {
        return runOnce(() -> intakePivot.setVoltage(power));
    }

    public Command runLeftPivot(double power) {
        return runOnce(() -> intakePivotFollow.setVoltage(power));
    }

    // Execute PIDs in default command
    public Command holdPivot() {
        return run(() -> {
            intakePivot.setVoltage(pivotPID.calculate(getRightPosition(), pivotSetPoint));
            intakePivotFollow.setVoltage(-pivotFollowerPID.calculate(getLeftPosition(), pivotSetPoint));
        });
    }

    @Override
    public void periodic() {
        getPivotEncoder = getRightPosition();

        // SmartDashboard.putNumber("Inake Angle", getRightPosition());
        // SmartDashboard.putBoolean("Intake Boolean", isRightDown());
        // SmartDashboard.putNumber("Pivot Setpoint", pivotSetPoint);
        // SmartDashboard.putNumber("Pivot Voltage", intakePivot.getMotorVoltage().getValueAsDouble());
        // SmartDashboard.putNumber("Pivot Follower Voltage", intakePivotFollow.getMotorVoltage().getValueAsDouble());
        // SmartDashboard.putNumber("Right Pivot Position", getRightPosition());
        // SmartDashboard.putNumber("Left Pivot Position", getLeftPosition());
    }

}
