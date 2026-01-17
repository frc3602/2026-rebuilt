package frc.robot;

import frc.robot.LimelightHelpers;

public class Vision {
    
    public double getTX() {
        return LimelightHelpers.getTX("limelight-primary");
    }
}
