package frc.robot;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.LimelightHelpers.RawFiducial;


public class Vision {
    

    public double getDistToCamera(int id) {
        var tags =  LimelightHelpers.getRawFiducials("limelight-primary");
         return  0;
    }

    public double getPose2D(int x) {
        var x = LimelightHelpers.getPose2D("limelight-primary");
        return x;
    }
    
    public double getTX() {
        return LimelightHelpers.getTX("limelight-primary");
    }
}
