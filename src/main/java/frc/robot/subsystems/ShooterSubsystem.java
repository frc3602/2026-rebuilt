package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class ShooterSubsystem extends SubsystemBase{
    private double shooter1Speed = 0;
    private double shooter2Speed = 0;
    private double feederSpeed = 0;
    //Shooter Motors
    private static TalonFX shootermotor1;
    private static TalonFX shootermotor2;
    //Feeding Motor
    private static TalonFX feedermoter;
    //Constructor
    public ShooterSubsystem() {
        shootermotor1 = new TalonFX(SHOOTER_MOTER_1_ID);
        shootermotor2 = new TalonFX(SHOOTER_MOTER_2_ID);
        feedermoter = new TalonFX(FEEDER_MOTER_ID);
    }
        //Go
    public Command setShootSpeed(double shootSpeed) {
        return runOnce(() -> {
            shootermotor1.set(shootSpeed);
            shootermotor2.set(shootSpeed);
            shooter1Speed = shootSpeed;
            shooter2Speed = shootSpeed;
        });
    }

    public Command setFeederSpeed(double speed) {
        return runOnce(() -> {
            feedermoter.set(speed);
            feederSpeed = speed;
        });
    }

        //Stop
    public Command stopShooter(double shootStop) {
        return runOnce(() -> {
            shootermotor1.set(0);
            shooter1Speed = 0;
            shooter2Speed = 0;
        });
    }

    public Command stopFeeder(double feedStop) {
        return runOnce(() -> {
            feedermoter.set(0);
            feederSpeed = 0;
        });
    }
    
    //Periodic
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter1 Speed", shooter1Speed);
        SmartDashboard.putNumber("Shooter2 Speed", shooter2Speed);
        SmartDashboard.putNumber("Feeder Speed", feederSpeed);
    }
}
