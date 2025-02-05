package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Vision extends SubsystemBase {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

//read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    NetworkTableEntry dis = table.getEntry("targetpose_cameraspace");
    public static double distance;
    public static double angle;
    public static double skew;

      /// 
    public void periodic() {
        
        distance = dis.getDoubleArray(new double[5])[2];
        SmartDashboard.putNumber("Distance", distance);

        angle = table.getEntry("tx").getDouble(0);
        SmartDashboard.putNumber("tx", angle);

        skew = table.getEntry("targetSkewDegrees").getDouble(0);
        SmartDashboard.putNumber("targetSkewDegrees", skew);

        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
  }
  
}

