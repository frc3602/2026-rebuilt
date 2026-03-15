package frc.team3602.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3602.robot.Constants.IntakeConstants;
import frc.team3602.robot.Constants.ShooterConstants;

public class IntakeSubsystem extends SubsystemBase{
    // The intake subsystem currently controls only the roller that pulls notes in
    // and pushes them back out. Pivot control lives in PivotSubsystem.
    private final TalonFX intakeMotor = new TalonFX(IntakeConstants.kIntakeMotorID, "rio");
    
    /**
     * Creates the intake subsystem and applies the roller motor config.
     */
    public IntakeSubsystem(){
        configIntakeSubsys();
    }

    private final PIDController pivotPID = new PIDController(0.15, 0.0,0.0);
    private final PIDController pivotFollowerPID = new PIDController(0.15,0.0, 0.0);

    /**
     * Starts the intake roller in the normal "collect note" direction.
     *
     * This is a one-shot command because RobotContainer handles the press/release
     * behavior and leaves the motor running until a later stop command.
     */
    public Command setIntakeSpeed() {
        return runOnce(() -> 
            intakeMotor.set(IntakeConstants.kIntakeMotorSpeed));
    
    }

    /**
     * Stops the intake roller.
     *
     * We use this on button release so the roller does not keep running after the
     * driver lets go.
     */
    public Command stopIntake() {
        return runOnce(() ->
        intakeMotor.set(0));
    }

    /**
     * Reverses the intake roller to eject a note.
     *
     * This is useful for clearing a bad pickup or backing a note out during
     * testing.
     */
    public Command reverseIntake() {
        return runOnce(() ->
        intakeMotor.set(-IntakeConstants.kIntakeMotorSpeed));
    }

    @Override
    public void periodic() {

    }

    /**
     * Applies the current safety configuration for the intake roller motor.
     *
     * Right now this is intentionally simple: we create one TalonFX config object,
     * enable current-limit support, and apply it to the intake motor so future
     * tuning happens in one obvious place.
     */
    private void configIntakeSubsys() {
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

        // Enable current-limit handling even though the exact limit values are not
        // finalized here yet. This keeps the setup path ready for future tuning.
        var currentLimitConfigs = talonFXConfigs.CurrentLimits;
        currentLimitConfigs.StatorCurrentLimitEnable = true;
        currentLimitConfigs.SupplyCurrentLimitEnable = true;
        // currentLimitConfigs.StatorCurrentLimit = 60;
        // currentLimitConfigs.SupplyCurrentLimit = 60;

        intakeMotor.getConfigurator().apply(talonFXConfigs);

    }
}

