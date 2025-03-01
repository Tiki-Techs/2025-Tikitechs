package frc.robot.subsystems;

import java.lang.reflect.Array;

import com.fasterxml.jackson.databind.deser.ValueInstantiator.Gettable;
import com.fasterxml.jackson.databind.ser.AnyGetterWriter;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;



public class Vision extends SubsystemBase {
  // limelight 3 is front
    NetworkTable back = NetworkTableInstance.getDefault().getTable("limelight-back");
    NetworkTable front = NetworkTableInstance.getDefault().getTable("limelight-front");
    NetworkTableEntry backTable = back.getEntry("targetpose_cameraspace");
    NetworkTableEntry frontTable = front.getEntry("targetpose_cameraspace");
    public static double distance;
    public static double angleY;
    public static double angleX;
    public static double skew;
    public static String cam = "front";
    public String id = "";

    public void setBack () {
      distance = backTable.getDoubleArray(new double[5])[2];
      angleX = back.getEntry("tx").getDouble(0);
      angleY = back.getEntry("ty").getDouble(0);
      skew = backTable.getDoubleArray(new double[5])[4];
      cam = "back";
      double id = back.getEntry("tid").getDouble(0);
      SmartDashboard.putNumber("num id", id);
      if (((id >= 12) && (id <= 13)) || ((id >= 1) && (id <= 2))) {
        this.id = "Coral Feeder";
      }
      else if (((id >= 17) && (id <= 22)) || ((id >= 6) && (id <= 11))) {
        this.id = "Reef";
      }
      
      else if ((id == 16) || (id == 3)) {
        this.id = "Algae Processor";
      }
      
      else if (((id >= 14) && (id <= 15)) || ((id >= 4) && (id <= 5))) {
        this.id = "Net";
      }
      else {
        this.id = "";
      }
    }

    public void setFront () {
      distance = frontTable.getDoubleArray(new double[5])[2];
      angleX = front.getEntry("tx").getDouble(0);
      angleY = front.getEntry("ty").getDouble(0);
      skew = frontTable.getDoubleArray(new double[5])[4];
      cam = "front";
      double id = front.getEntry("tid").getDouble(0);
      SmartDashboard.putNumber("num id", id);
      if (((id >= 12) && (id <= 13)) || ((id >= 1) && (id <= 2))) {
        this.id = "Coral Feeder";
      }
      else if (((id >= 17) && (id <= 22)) || ((id >= 6) && (id <= 11))) {
        this.id = "Reef";
      }
      
      else if ((id == 16) || (id == 3)) {
        this.id = "Algae Processor";
      }
      
      else if (((id >= 14) && (id <= 15)) || ((id >= 4) && (id <= 5))) {
        this.id = "Net";
      }

      else {
        this.id = "";
      }
    }

      /// a thing to select between bothvisions, if seen on both, pick closer
    public void periodic() {
      double tarF = (double) front.getEntry("tv").getInteger(0);
      double tarB = (double) back.getEntry("tv").getInteger(0);
      SmartDashboard.putNumber("front", tarF);
      SmartDashboard.putNumber("back", tarB);
      if ((tarF == 1) && (tarB == 1)) {
        double disF = frontTable.getDoubleArray(new double[5])[2];
        double disB = backTable.getDoubleArray(new double[5])[2];
        if (disF > disB) {
          setBack();
        }
        else {
          setFront();
        }
      }

      else if ((tarF != 1) && (tarB == 1)) {
        setBack();
      }
      
      else if ((tarF == 1) && (tarB != 1)) {
        setFront();
      }
      else {
        distance = 0;
        angleX = 0;
        angleY = 0;
        skew = 0;
        id = "";
      }

        SmartDashboard.putNumber("distance", distance);

        SmartDashboard.putNumber("angleX", angleX);

        SmartDashboard.putNumber("angleY", angleY);

        SmartDashboard.putNumber("skew", skew);
        
        SmartDashboard.putString("id", id);
        
  }
  
}

