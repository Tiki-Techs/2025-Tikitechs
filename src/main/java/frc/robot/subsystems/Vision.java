package frc.robot.subsystems;

import java.lang.reflect.Array;
import java.util.Optional;

import org.photonvision.PhotonUtils;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

import com.ctre.phoenix6.mechanisms.DifferentialMechanism.DisabledReason;
import com.fasterxml.jackson.databind.deser.ValueInstantiator.Gettable;
import com.fasterxml.jackson.databind.ser.AnyGetterWriter;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.sendable.SendableBuilder.BackendKind;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;

public class Vision extends SubsystemBase {
  // limelight 3 is front
  NetworkTable back = NetworkTableInstance.getDefault().getTable("limelight-back");
  NetworkTable front = NetworkTableInstance.getDefault().getTable("limelight-front");
  NetworkTableEntry backTable = back.getEntry("targetpose_cameraspace");
  NetworkTableEntry frontTable = front.getEntry("targetpose_robotspace");
  public static double distanceFront;
  public static double distanceBack;
  public static double angleY;
  public static double angleX;
  public static double skew;
  public static String cam = "front";
  public static String id = "";
  private Pose2d frontPose;
  private Pose2d backPose;
  private boolean frontLimelightConnected = false;
  private boolean backLimelightConnected = false;
  public boolean isRed;

  StructPublisher<Pose3d> publisherF = NetworkTableInstance.getDefault()
      .getStructTopic("MyPose VF", Pose3d.struct).publish();

  StructPublisher<Pose3d> publisherB = NetworkTableInstance.getDefault()
      .getStructTopic("MyPose VB", Pose3d.struct).publish();

  // private double distanceFront = 1000;
  // private double distanceBack = 1000;
  private double numFront = 0;
  private double numBack = 0;

  // StructPublisher<Pose3d> publisherEst = NetworkTableInstance.getDefault()
  // .getStructTopic("MyPoseEst", Pose3d.struct).publish();

  public Pose3d pose2D3D(Pose2d pose2d) {
    return new Pose3d(
        new Translation3d(pose2d.getX(), pose2d.getY(), 0), // Assume 0 height (Z)
        new Rotation3d(0, 0, pose2d.getRotation().getRadians()) // Only rotate around Z
    );
  }

  public int tag = 41238149;
  public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout
      .loadField(AprilTagFields.k2025ReefscapeWelded);

  // VisionSystemSim visionSim = new VisionSystemSim("main");
  // TargetModel targetModel = new TargetModel(0.5, 0.25);
  // Pose3d targetPose = new Pose3d(16, 4, 2, new Rotation3d(0, 0, Math.PI));
  // VisionTargetSim visionTarget = new VisionTargetSim(targetPose, targetModel);

  public Vision() {

    frontPose = new Pose2d();
    backPose = new Pose2d();

    isRed = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
    // if (isRed) {
    // fieldLayout.
    // }
    // visionSim.addVisionTargets(visionTarget);
    // AprilTagFieldLayout tagLayout =
    // AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    // visionSim.addAprilTags(tagLayout);

    // CameraServer.startAutomaticCapture();

  }

  public double getDistanceFromAprilTag(int id) {
    Optional<Pose3d> tag = fieldLayout.getTagPose(id); // switch field to be in vision sub
    return tag.map(
        pose3d -> PhotonUtils.getDistanceToPose(RobotContainer.drivebase.getRobotPose().toPose2d(), pose3d.toPose2d()))
        .orElse(1000.0);
  }

  public int closestTag() {
    double lowest = 1000;
    if (isRed) {
      for (int i = 0; i < 6; i++) {
        if (getDistanceFromAprilTag(i + 6) < lowest) {
          lowest = getDistanceFromAprilTag(i + 6);
          tag = (-(i + 6) % 6) + 1;
          SmartDashboard.putNumber("closest id", tag);
        }
      }
    } else {
      for (int i = 0; i < 6; i++) {
        if (getDistanceFromAprilTag(i + 17) < lowest) {
          lowest = getDistanceFromAprilTag(i + 17);
          tag = (i + 17) % 6;
          SmartDashboard.putNumber("closest id", tag);
        }
      }
    }
    return (int) tag;
  }

  // This is a suss function - need to test it; This method is taken from 1884
  // Griffins, and should be modified
  public boolean isInMap(Pose2d currentPose) {
    return ((currentPose.getX() >= 0.0 && currentPose.getX() <= fieldLayout.getFieldLength())
        && (currentPose.getY() >= 0.0 && currentPose.getY() <= fieldLayout.getFieldWidth()));
    // return true;
  }

  // This method is derived from 1884 Griffins' Vision. We may want to use the NT4
  // getAtomic() that syncs rio FPGA and coprocessor clock.
  public double getTotalLatency(String limelightName) {
    return LimelightHelpers.getLatency_Pipeline(limelightName) + LimelightHelpers.getLatency_Capture(limelightName); // just
                                                                                                                     // replace
                                                                                                                     // visionconfig
                                                                                                                     // thing
                                                                                                                     // with
                                                                                                                     // limelight
                                                                                                                     // name
  }

  /**
   * Gets the camera capture time in seconds. Only used for limelight; Taken from
   * 1884 Griffins
   *
   * @param latencyMillis the latency of the camera in milliseconds
   * @return the camera capture time in seconds
   */
  public double getTimestampSeconds(double latencyMillis) {
    // SmartDashboard.putNumber("timer test", Timer.getFPGATimestamp());
    // SmartDashboard.putNumber("second timer test", latencyMillis/1000d);
    return Timer.getFPGATimestamp() - (latencyMillis / 1000d);
  }

  // boolean tempVisionTest = DriverStation.getAlliance().isPresent() &&
  // DriverStation.getAlliance().get() == Alliance.Red;
  // double lowest = 1000;
  // if (tempVisionTest) {
  // for (int i = 0; i < 6; i++) {
  // if (getDistanceFromAprilTag(i+6) < lowest) {
  // lowest = getDistanceFromAprilTag(i+6);
  // tag = i+6;
  // // SmartDashboard.putNumber("closest id", tag);
  // }
  // }
  // }
  // else {
  // for (int i = 0; i < 6; i++) {
  // if (getDistanceFromAprilTag(i+17) < lowest) {
  // lowest = getDistanceFromAprilTag(i+17);
  // tag = (i+17)%6;
  // // SmartDashboard.putNumber("closest id", tag);
  // }
  // }
  // }

  public void setFront() {
    distanceFront = frontTable.getDoubleArray(new double[5])[2];
    angleX = front.getEntry("tx").getDouble(0);
    angleY = front.getEntry("ty").getDouble(0);
    skew = frontTable.getDoubleArray(new double[5])[4];
    cam = "front";
    double id = front.getEntry("tid").getDouble(0);
    // SmartDashboard.putNumber("num id", id);
    if (((id >= 12) && (id <= 13)) || ((id >= 1) && (id <= 2))) {
      this.id = "Coral Feeder";
    } else if (((id >= 17) && (id <= 22)) || ((id >= 6) && (id <= 11))) {
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

  public Pose2d visionBotPoseFront() {
    return frontPose;
  }

  public Pose2d visionBotPoseBack() {
    return backPose;
  }

  public Vector<N3> trustFactors(double distance, double tags) {
    double trust = (tags * 2) / (distance + 1);
    SmartDashboard.putNumber("trust", trust);
    return createVisionMeasurementStdDevs(trust, trust, 10);
  }

  public Vector<N3> createVisionMeasurementStdDevs(double x, double y, double theta) {
    return VecBuilder.fill(x, y, Units.degreesToRadians(theta));
  }

  public boolean visionAccurate(Pose2d currentPose) {
    return isInMap(currentPose);
  }

  public Vector<N3> modVisionStdDevsF(double modnum1, double modnum2) {
    return VecBuilder.fill(Math.max(modnum1 * distanceFront, modnum2), Math.max(modnum1 * distanceFront, modnum2),
        Units.degreesToRadians(10000000));
  }

  public Vector<N3> modVisionStdDevsB(double modnum1, double modnum2) {
    return VecBuilder.fill(Math.max(modnum1 * distanceFront, modnum2), Math.max(modnum1 * distanceFront, modnum2),
        Units.degreesToRadians(10000000));
  }

  private Pose2d flipPose(Pose2d pose) {
    Translation2d center = fieldLayout.getOrigin().toPose2d().getTranslation();
    Translation2d poseTranslation = pose.getTranslation();
    poseTranslation = poseTranslation.rotateAround(center, Rotation2d.k180deg);
    return new Pose2d(poseTranslation, pose.getRotation().rotateBy(Rotation2d.k180deg));
  }

  /// a thing to select between bothvisions, if seen on both, pick closer
  public void periodic() {
    // distanceFront = frontTable.getDoubleArray(new double[5])[2];

    // distanceBack = frontTable.getDoubleArray(new double[5])[2];

    // numFront = front.getEntry("tv").getDouble(0);

    // numBack = back.getEntry("tv").getDouble(0);

    // SmartDashboard.putNumber("disf", distanceFront);
    // SmartDashboard.putNumber("disb", distanceBack);
    // SmartDashboard.putNumber("tarf", numFront);
    // SmartDashboard.putNumber("tarb", numBack);

    // double tarF = (double) front.getEntry("tv").getInteger(0);
    // double tarB = (double) back.getEntry("tv").getInteger(0);
    // // SmartDashboard.putNumber("front", tarF);
    // // SmartDashboard.putNumber("back", tarB);
    // if ((tarF == 1) && (tarB == 1)) {
    // double disF = frontTable.getDoubleArray(new double[5])[2];
    // double disB = backTable.getDoubleArray(new double[5])[2];
    // if (disF > disB) {
    // setBack();
    // }
    // else {
    // setFront();
    // }
    // }

    // else if ((tarF != 1) && (tarB == 1)) {
    // setBack();
    // }

    // else if ((tarF == 1) && (tarB != 1)) {
    // setFront();
    // }
    // else {
    // distance = 0;
    // angleX = 0;
    // angleY = 0;
    // skew = 0;
    // id = "";
    // }

    // 8.308467, 1.442593 and 1.451102
    /* Ensures empty json not fed to pipeline */
    frontLimelightConnected = NetworkTableInstance.getDefault()
        .getTable("limelight-front")
        .getEntry("tv")
        .getInteger(0) != 0;

    backLimelightConnected = NetworkTableInstance.getDefault()
        .getTable("limelight-back")
        .getEntry("tv")
        .getDouble(0) != 0;

    // LimelightHelpers.
    distanceFront = NetworkTableInstance.getDefault()
        .getTable("limelight-front")
        .getEntry("targetpose_robotspace")
        .getDoubleArray(new double[5])[2];

    distanceBack = NetworkTableInstance.getDefault()
        .getTable("limelight-back")
        .getEntry("targetpose_robotspace")
        .getDoubleArray(new double[5])[2];

    SmartDashboard.putBoolean("llB", backLimelightConnected);
    SmartDashboard.putBoolean("llF", frontLimelightConnected);
    SmartDashboard.putNumber("disf", distanceFront);
    SmartDashboard.putNumber("disb", distanceBack);

    if (frontLimelightConnected && distanceFront <= 2) {
      // jsonResults = LimelightHelpers.getLatestResults(VisionConfig.POSE_LIMELIGHT);

      // estimatePose = LimelightHelpers.getBotPose2d_wpiBlue("limelight-front");
      if (isRed) {
        // LimelightHelpers.SetRobotOrientation("limelight-front",
        // PoseEstimator.getInstance().getOdometryPose().getRotation().getDegrees()-180,
        // 0, 0, 0, 0, 0);

        // LimelightHelpers.PoseEstimate frontEstimate =
        // LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("limelight-front");

        // if (visionAccurate(frontEstimate.pose)) {
        // // Blue alliance means origin is bottom right of the field
        // // getTotalLatency("limelight-front");
        frontPose = null;
        // frontPose = flipPose(frontEstimate.pose);
        // }

        // LimelightHelpers.SetRobotOrientation("limelight-front",
        // PoseEstimator.getInstance().getOdometryPose().getRotation().getDegrees(), 0,
        // 0, 0, 0, 0);
      } else {
        LimelightHelpers.SetRobotOrientation("limelight-front",
            PoseEstimator.getInstance().getOdometryPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);

        LimelightHelpers.PoseEstimate frontEstimate = LimelightHelpers
            .getBotPoseEstimate_wpiBlue_MegaTag2("limelight-front");

        if (visionAccurate(frontEstimate.pose)) {
          // Blue alliance means origin is bottom right of the field
          // getTotalLatency("limelight-front");
          frontPose = frontEstimate.pose;
        }

      }

    } else {
      frontPose = null;
    }

    if (backLimelightConnected && distanceFront <= 4.5) {
      if (isRed) {
        // LimelightHelpers.SetRobotOrientation("limelight-back",
        // PoseEstimator.getInstance().getOdometryPose().getRotation().getDegrees()-180,
        // 0, 0, 0, 0, 0);

        // LimelightHelpers.PoseEstimate backEstimate =
        // LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("limelight-back");

        // if (visionAccurate(backEstimate.pose)) {
        backPose = null;
        // }
        // LimelightHelpers.SetRobotOrientation("limelight-back",
        // PoseEstimator.getInstance().getOdometryPose().getRotation().getDegrees(), 0,
        // 0, 0, 0, 0);
      } else {
        LimelightHelpers.SetRobotOrientation("limelight-back",
            PoseEstimator.getInstance().getOdometryPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);

        LimelightHelpers.PoseEstimate backEstimate = LimelightHelpers
            .getBotPoseEstimate_wpiBlue_MegaTag2("limelight-back");

        if (visionAccurate(backEstimate.pose)) {
          backPose = backEstimate.pose;
        }
      }
    } else {
      backPose = null;
    }

    if (frontPose != null) {

      publisherF.set(pose2D3D(frontPose));
    }
    if (backPose != null) {
      publisherB.set(pose2D3D(backPose));
    }

  }

}
