package frc.robot.subsystems;

import java.lang.reflect.Array;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.PhotonUtils;

import com.fasterxml.jackson.databind.deser.ValueInstantiator.Gettable;
import com.fasterxml.jackson.databind.ser.AnyGetterWriter;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;



public class Vision2 extends SubsystemBase {
  // limelight 3 is front

    public static final AprilTagFieldLayout fieldLayout                     = AprilTagFieldLayout.loadField(      AprilTagFields.k2025ReefscapeWelded);

    private Supplier<Pose2d> currentPose;

    public Vision2() {
    }

    public static Pose2d getAprilTagPose(int aprilTag, Transform2d robotOffset) { // need to get robot offset, need to use apriltag pose to drive to.
      Optional<Pose3d> aprilTagPose3d = fieldLayout.getTagPose(aprilTag);
      if (aprilTagPose3d.isPresent()) {
        return aprilTagPose3d.get().toPose2d().transformBy(robotOffset);
      } else {
        throw new RuntimeException("Cannot get AprilTag " + aprilTag + " from field " + fieldLayout.toString());
      }
    }

    
    public double getDistanceFromAprilTag(int id) {
      Optional<Pose3d> tag = fieldLayout.getTagPose(id);
      return tag.map(pose3d -> PhotonUtils.getDistanceToPose(currentPose.get(), pose3d.toPose2d())).orElse(-1.0);
    }

      /// a thing to select between bothvisions, if seen on both, pick closer
    public void periodic() {
      getAprilTagPose(0, null); // need to get robotoffset(?)

  }
  
}

