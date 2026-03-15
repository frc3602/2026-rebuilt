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

    // The intake pivot uses two motors, one on each side of the mechanism.
    // We control them separately because the two gearboxes are mirrored and need
    // opposite sign conventions to move together correctly.
    private final TalonFX intakePivot = new TalonFX(IntakeConstants.kIntakePivotID, "rio");
    private final TalonFX intakePivotFollow = new TalonFX(IntakeConstants.kIntakePivotFollowID, "rio");

    // Cached right-side angle for quick dashboard checks and simple threshold logic.
    public double getPivotEncoder;

    // Desired intake angle in mechanism degrees.
    // The default command continuously drives both pivot motors toward this target.
    private double pivotSetPoint = 0;

    // One "smart" step changes the intake by about a quarter-turn of the output.
    // This gives the team a simple coarse-adjust control without typing in raw
    // target angles each time.
    private double offset = 90;

    /**
     * Creates the intake pivot subsystem and applies the shared motor config.
     *
     * We zero both relative rotor sensors at startup because the current design
     * uses those sensors as the pivot reference. Both motors also receive the same
     * current limit so either side does not overpower the other during a move.
     */
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

    /**
     * Returns the right pivot angle in mechanism degrees.
     *
     * The Falcon reports rotor rotations, so we divide by the total 125:1 gear
     * reduction and then convert the result into output-shaft degrees.
     */
    public Double getRightPosition() {
        return (intakePivot.getRotorPosition().getValueAsDouble() / 125) * 360; // every revolution is 0.125 degrees
                                                                                // because it is 125 to 1 gear ratio //
                                                                                // a 30:1 gear ratio 10:1 from gear, 3:1
                                                                                // from gear box
    }

    /**
     * Returns the left pivot angle in mechanism degrees.
     *
     * The left side is mirrored relative to the right side, so we negate the
     * converted angle to keep both sensors in the same mechanism coordinate frame.
     */
    public Double getLeftPosition() {
        return -(intakePivotFollow.getRotorPosition().getValueAsDouble() / 125) * 360; // every revolution is 0.125
                                                                                       // degrees because it is 125 to 1
                                                                                       // gear ratio // a 30:1 gear
                                                                                       // ratio 10:1 from gear, 3:1 from
                                                                                       // gear box
    }

    /**
     * Raises the intake by one coarse step.
     *
     * In this angle convention, a smaller angle means the intake is higher, so we
     * subtract the offset from the current setpoint.
     */
    public Command smartRaisePivot() {
        return runOnce(() -> pivotSetPoint = pivotSetPoint - offset

        );
    }

    /**
     * Lowers the intake by one coarse step.
     *
     * In this angle convention, a larger angle means the intake is lower, so we
     * add the offset to the current setpoint.
     */
    public Command smartDropPivot() {
        return runOnce(() -> pivotSetPoint = pivotSetPoint + offset);
    }

    /**
     * Sends the intake to the team's current fully lowered preset.
     *
     * This preset is used for normal floor-intake operation and can be re-tuned if
     * the mechanism geometry changes.
     */
    public Command dumbDropIntake() {
        return runOnce(() -> pivotSetPoint = 104);
    }

    /**
     * Sends the intake to the team's current raised or stowed preset.
     *
     * This is the quick "bring the intake up" command used in teleop cleanup and
     * staging.
     */
    public Command dumbRaiseIntake() {
        return runOnce(() -> pivotSetPoint = 20);
    }

    /**
     * Returns whether the right pivot angle is below the robot enough to count as
     * "down."
     *
     * This is a simple threshold helper for quick checks, not a precision ready
     * detector.
     */
    public Boolean isRightDown() {
        return (getPivotEncoder > 90);
    }

    /**
     * Returns whether the right pivot angle is above the robot enough to count as
     * "up."
     */
    public Boolean isRightUP() {
        return (getPivotEncoder < 0);
    }

    /**
     * Applies a manual voltage directly to the right pivot motor.
     *
     * This is mainly useful for testing or emergency troubleshooting.
     */
    public Command runRightPivot(double power) {
        return runOnce(() -> intakePivot.setVoltage(power));
    }

    /**
     * Applies a manual voltage directly to the left pivot motor.
     *
     * This mirrors the right-side test helper so each side can be checked
     * independently.
     */
    public Command runLeftPivot(double power) {
        return runOnce(() -> intakePivotFollow.setVoltage(power));
    }

    /**
     * Continuously holds the intake at the current pivot setpoint.
     *
     * RobotContainer uses this as the default command so the intake keeps holding
     * its last requested angle unless another command changes the setpoint.
     */
    public Command holdPivot() {
        return run(() -> {
            // The two sides are mirrored, so the follower motor needs the opposite
            // voltage sign to move the mechanism in the same physical direction.
            intakePivot.setVoltage(pivotPID.calculate(getRightPosition(), pivotSetPoint));
            intakePivotFollow.setVoltage(-pivotFollowerPID.calculate(getLeftPosition(), pivotSetPoint));
        });
    }

    @Override
    public void periodic() {
        // Keep a cached copy of the current right-side angle for simple threshold
        // checks and optional dashboard use.
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
