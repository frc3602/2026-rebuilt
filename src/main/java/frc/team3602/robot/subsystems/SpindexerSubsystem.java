package frc.team3602.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3602.robot.Constants.spindexerConstants;

public class SpindexerSubsystem extends SubsystemBase {
    /* Motors */

    private final TalonFX spindexerMotor;
    private final TalonFX receiveMotor;

    /* Constructor */

    /**
     * Creates both feed motors and applies the shared closed-loop config.
     *
     * The spindexer and receive motor always work as a pair, so we configure them
     * together in one place.
     */
    public SpindexerSubsystem() {

        spindexerMotor = new TalonFX(spindexerConstants.kSpindexerMotorID, "rio");
        receiveMotor = new TalonFX(spindexerConstants.kReceiveMotorID, "rio");
        configSpindexerSubsys();
    }

    final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);

    /* Commands */

    /**
     * Applies scaled feed velocities derived from the current shooter target/speed.
     *
     * The requested shooter speed is the "top" speed. From that:
         * - spindexer runs slower than shooter
         * - feeder/receiver runs slower than spindexer
     */
    private void applyFeedVelocityFromShooter(double shooterRotationsPerSecond) {
        double spindexerRotationsPerSecond = shooterRotationsPerSecond
            * spindexerConstants.kSpindexerToShooterSpeedRatio;
        double receiverRotationsPerSecond = spindexerRotationsPerSecond
            * spindexerConstants.kFeederToSpindexerSpeedRatio;

        receiveMotor.setControl(m_request.withVelocity(receiverRotationsPerSecond));
        spindexerMotor.setControl(m_request.withVelocity(spindexerRotationsPerSecond));
    }

    /**
     * Stops both feed motors immediately.
     */
    private void stopFeedMotors() {
        spindexerMotor.set(0);
        receiveMotor.set(0);
    }

    /**
     * Stops the spindexer path with a one-shot command.
     *
     * This is used as cleanup after timed feeds and on button release in teleop.
     */
    public Command stopSpindexer() {
        return runOnce(() -> {
            stopFeedMotors();
        });
    }

    /**
     * Runs feed using a live shooter-velocity supplier.
     *
     * This keeps feed scaling tied to the shooter's current speed in real time,
     * which helps handoff stay consistent as flywheel speed changes during tracking.
     */
    public Command setFeedVelocityScaledFromShooter(DoubleSupplier shooterVelocitySupplier) {
        return runEnd(
                () -> applyFeedVelocityFromShooter(shooterVelocitySupplier.getAsDouble()),
                this::stopFeedMotors);
    }

    /**
     * Runs feed only while the ready condition is true, using live shooter speed.
     */
    public Command setFeedVelocityScaledFromShooterWhen(BooleanSupplier shouldFeed,
            DoubleSupplier shooterVelocitySupplier) {
        return runEnd(() -> {
            if (shouldFeed.getAsBoolean()) {
                applyFeedVelocityFromShooter(shooterVelocitySupplier.getAsDouble());
            } else {
                stopFeedMotors();
            }
        }, this::stopFeedMotors);
    }

    /**
     * Reverses the spindexer path to help clear jams.
     *
     * We keep this as a simple percent-output command because reverse is used as a
     * short manual recovery action rather than a carefully metered scoring feed.
     */
    // Review note 2026-03-15 10:15:26 -04:00: this method may be unused in the
    // current codebase.
    public Command setReverseSpindexerReceive() {
        return runOnce(() -> {
            receiveMotor.set(spindexerConstants.kRecieveFuelSpeed);
            spindexerMotor.set(-spindexerConstants.kSpindexerMotorSpeed);
        });
    }

    /* Periodic */

    @Override
    public void periodic() {
        // SmartDashboard.putData("spindexer sped", (Sendable) spindexerMotor.getVelocity());
        // SmartDashboard.putData("spindexer speed", (Sendable) spindexerMotor.getMotorVoltage());
    }

    /**
     * Applies the current closed-loop config shared by the spindexer and receive
     * motors.
     *
     * Both motors use Motion Magic velocity control so feed speed stays more
     * consistent as battery voltage changes during a match.
     */
    private void configSpindexerSubsys() {
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
        
        // Slot0 holds the basic feedforward and PID gains for velocity control.
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.1; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // no output for error derivative

        // set Motion Magic Velocity settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicAcceleration = 400; // Target acceleration of 400 rps/s (0.25 seconds to max)
        motionMagicConfigs.MotionMagicJerk = 6000; // Targ  et jerk of 4000 rps/s/s (0.1 seconds)

        // Protect both motors with the same current limits so the feed path behaves
        // predictably and is easier to tune.
        var currentLimitConfigs = talonFXConfigs.CurrentLimits;
        currentLimitConfigs.StatorCurrentLimitEnable = true;
        currentLimitConfigs.SupplyCurrentLimitEnable = true;
        currentLimitConfigs.StatorCurrentLimit = 40;
        currentLimitConfigs.SupplyCurrentLimit = 60;

        // Apply the same velocity-control setup to both motors. Their requested
        // speeds differ later in applyFeedVelocityRequest(), but the control style is
        // shared.
        spindexerMotor.getConfigurator().apply(talonFXConfigs);
        receiveMotor.getConfigurator().apply(talonFXConfigs);
    }

}
