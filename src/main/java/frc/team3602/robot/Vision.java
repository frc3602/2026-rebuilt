package frc.team3602.robot;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3602.robot.LimelightHelpers;
import frc.team3602.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.team3602.robot.LimelightHelpers.RawFiducial;


public class Vision {


    public Vision() {
    int[] validTagID = {21, 26, 18, 8, 9, 11};
    LimelightHelpers.SetFiducialIDFiltersOverride("limelight-turret", validTagID);
    }

    public double getDistToCamera(int id) {
        var tags =  LimelightHelpers.getRawFiducials("limelight-primary");
         return  0;
    }
    
    public double getTX() {
        return LimelightHelpers.getTX("limelight-primary");
    }

    public double getTurretTX() {
        return LimelightHelpers.getTX("limelight-turret");
    }

    public boolean getHasTarget() {
        return LimelightHelpers.getTV("limelight-primary");
    }

        public boolean getTurretHasTarget() {
        return LimelightHelpers.getTV("limelight-turret");
    }

    public double getTY() {
        return LimelightHelpers.getTY("limelight-turret");
    }

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-turret");
    NetworkTableEntry ty = table.getEntry("ty");

    double targetOffsetAngle_Vertical = ty.getDouble(0.0);
    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 25.0; 

    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 20.0; 

    // distance from the target to the floor
    double goalHeightInches = 44.25; 

    double angleToGoalDegrees = limelightMountAngleDegrees + getTY();
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
    //calculated Distance
    double angle = Math.toRadians(this.getTY() + this.getTurretIMUPitch());
    double distance;
    public double getDist() {
        return   distance =(44.21875 - 15.625) / Math.tan(angle);
    }

    public double getPoseY() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-primary").pose.getY();
    }
    
    //Retrieve IMU Pitch. - for Up instead of Down 
    public double getTurretIMUPitch() {
        return -LimelightHelpers.getIMUData("limelight-turret").Pitch;
    }








}


