package frc.robot.subsystems;

import java.lang.reflect.Array;

import com.fasterxml.jackson.databind.deser.ValueInstantiator.Gettable;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Vision extends SubsystemBase {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    // NetworkTableEntry tx = table.getEntry("tx");
    // NetworkTableEntry ty = table.getEntry("ty");
    // NetworkTableEntry ta = table.getEntry("ta");
    // NetworkTableEntry tv = table.getEntry("tv");

//read values periodically
    // double x = tx.getDouble(0.0);
    // double y = ty.getDouble(0.0);
    // double area = ta.getDouble(0.0);

    NetworkTableEntry dis = table.getEntry("targetpose_cameraspace");
    public static double distance;
    public static double angleY;
    public static double angleX;
    public static double skew;
    public static int targetV;
    public static boolean centered;

      /// a thing to select between bothvisions, if seen on both, pick closer
    public void periodic() {
        targetV = (int) table.getEntry("tv").getInteger(0);
        SmartDashboard.putNumber("targetV", targetV);

        distance = dis.getDoubleArray(new double[5])[2];
        SmartDashboard.putNumber("distance", distance);

        angleX = table.getEntry("tx").getDouble(0);
        SmartDashboard.putNumber("angleX", angleX);

        angleY = table.getEntry("ty").getDouble(0);
        SmartDashboard.putNumber("angleY", angleY);

        skew = dis.getDoubleArray(new double[5])[4];
        SmartDashboard.putNumber("skew", skew);
        
        if (MathUtil.applyDeadband(angleX, 2) != 0){
          centered = false;
        }
        else {
          centered = true;
        }

        SmartDashboard.putBoolean("centered", centered);
  }
  
}

