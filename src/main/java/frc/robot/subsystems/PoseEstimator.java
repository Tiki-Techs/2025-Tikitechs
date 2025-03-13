package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

// import frc.robot.RobotMap.PoseConfig;
// import frc.robot.RobotMap.VisionConfig;
// import frc.robot.core.MAXSwerve.MaxSwerveConstants;
// import frc.robot.subsystems.Drivetrain;

/** Reports our expected, desired, and actual poses to dashboards */ // ADD A THING TO FULLY TRUST VISION WHEN DISABLED!!!!!!!!
public class PoseEstimator extends SubsystemBase {
  private static PoseEstimator instance;





    Pose2d estimatedRobotPose = new Pose2d();
Pose3d poseB = new Pose3d();
StructPublisher<Pose3d> publisherEst = NetworkTableInstance.getDefault()
  .getStructTopic("MyPoseEst", Pose3d.struct).publish();
  




  public static PoseEstimator getInstance() {
    if (instance == null) instance = new PoseEstimator(); // Need to take swerve out of constructor and use the getInstance originally used.
    return instance;
  }

  // PoseConfig config;
  // private PoseTelemetry telemetry;
  private Pose2d odometryPose = new Pose2d();
  private Pose2d desiredPose = new Pose2d();
  private Pose2d estimateFrontPose = new Pose2d();
  private Pose2d estimateBackPose = new Pose2d();
  private boolean frontChanged = false;
  private boolean backChanged = false;
  private boolean isDisabled = true;

  private final SwerveDrivePoseEstimator poseEstimator;
  private final SwerveSubsystem drivetrain;

  public PoseEstimator() {
    // config = new PoseConfig();
    // telemetry = new PoseTelemetry(this);

    // drivetrain = SwerveSubsystem.getInstance();
    drivetrain = RobotContainer.drivebase;

    // Maxswerve Version from MAXSwerve.java in core
    poseEstimator =
        new SwerveDrivePoseEstimator(
            drivetrain.getKinematics(),
            drivetrain.getYaw(),
            drivetrain.getSwerveModulePositions(),
            drivetrain.getPose(),
            createStateStdDevs(
                5, 5, 10),
            createVisionMeasurementStdDevs(
                30, 30, 1000000));
  }


  
public Pose3d pose2D3D (Pose2d pose2d) {
        return new Pose3d(
            new Translation3d(pose2d.getX(), pose2d.getY(), 0), // Assume 0 height (Z)
            new Rotation3d(0, 0, pose2d.getRotation().getRadians()) // Only rotate around Z
        );
    }




  @Override
  public void periodic() {
    updateOdometryEstimate(); // Updates using wheel encoder data only
    // Updates using the vision estimate
    isDisabled = DriverStation.isDisabled();
    SmartDashboard.putBoolean("isdisabled", isDisabled);
    estimateFrontPose = RobotContainer.m_vision.visionBotPoseFront();
    estimateBackPose = RobotContainer.m_vision.visionBotPoseBack();
    // publisherEst.set(pose2D3D(estimateFrontPose));
    publisherEst.set(pose2D3D(getPosition()));

    if (estimateFrontPose != null) { // Limelight mode; originally had a boolean for whether limelight was true or false, but 3880 only runs limelight (2024-2025)
      double currentTimestamp = RobotContainer.m_vision.getTimestampSeconds(RobotContainer.m_vision.getTotalLatency("limelight-front"));
      if (isEstimateReady(estimateFrontPose)) { // Does making so many bot pose variables impact accuracy?
        SmartDashboard.putBoolean("is estimate ready", true);
        if (!isDisabled) {
          poseEstimator.setVisionMeasurementStdDevs(createVisionMeasurementStdDevs(8, 8, 0));
          addVisionMeasurement(estimateFrontPose, currentTimestamp);
        SmartDashboard.putBoolean("is ready and disabled", false);
        }
        else {
          // poseEstimator.setVisionMeasurementStdDevs(createVisionMeasurementStdDevs(1000,1000, 0));
          addVisionMeasurement(estimateFrontPose, currentTimestamp);
          
        SmartDashboard.putBoolean("is ready and disabled", true);
        }
        frontChanged = true;
      }
      else {
        SmartDashboard.putBoolean("is estimate ready", false);
      }
    }
    else {
      frontChanged = false;
    }

    if (estimateBackPose != null) {
      double currentTimestamp = RobotContainer.m_vision.getTimestampSeconds(RobotContainer.m_vision.getTotalLatency("limelight-front"));
      if (isEstimateReady(estimateBackPose)) {
        if (!isDisabled) {
          poseEstimator.setVisionMeasurementStdDevs(createVisionMeasurementStdDevs(30, 30, 300000));
          addVisionMeasurement(estimateBackPose, currentTimestamp);
        }
        else {
          poseEstimator.setVisionMeasurementStdDevs(createVisionMeasurementStdDevs(4,4, 3000000));
          addVisionMeasurement(estimateBackPose, currentTimestamp);
        }
        backChanged = true;
      }
    }
    else {
      backChanged = false;
    }
    // // TODO Photonvision mode - Needs editing and filtering
    // if (VisionConfig.IS_PHOTON_VISION_MODE && estimatePose != null) { // Limelight mode
    //   double photonTimestamp = Vision.getInstance().getPhotonTimestamp();
    //   if (isEstimateReady(estimatePose)) { // Does making so many bot pose variables impact accuracy?
    //     addVisionMeasurement(estimatePose, photonTimestamp);
    //   }
    // }

    // Update for telemetry
    setEstimatedPose(getPosition());
    setOdometryPose(drivetrain.swerveDrive.getPose());

    //UNTESTED - ALWAYS SETS DRIVETRAIN ODOMETRY TO THE POSE-ESTIMATOR ODOMETRY
    //NOT GREAT FOR ERROR CHECKING POSE ESTIMATOR! - SET TO FALSE
    if(frontChanged || backChanged){ // Boolean for whether or note vision override is enabled. Assumption is that this will just be used when robot is disabled.
      drivetrain.swerveDrive.resetOdometry(getPosition());
    }


    // telemetry.updatePoseOnField("VisionPose", Robot.vision.botPose);
    // telemetry.updatePoseOnField("OdometryPose", odometryPose); // Was not commented
    // telemetry.updatePoseOnField( // Was not commented
        // "EstimatedPose", estimatePose); // Need to uncomment and fix to work here. // Was not commented
  }

  // /**
  //  * @return true if estimated pose is on the chargestation by using the field-space
  // chargestation
  //  *     dimensions
  //  */
  // public boolean isOnChargeStation() {
  //     return ((getBestPose().getX() > 2.9 && getBestPose().getX() < 4.8)
  //             && (getBestPose().getY() > 1.54 && getBestPose().getY() < 3.99));
  // }

  // /**
  //  * Returns the most accurate pose. If we are not confident that vision is accurate,
  //  * estimatedPose is considered to be most accurate.
  //  *
  //  * @return vision pose or estimated pose
  //  */
  // public Pose2d getBestPose() {
  //     if (Robot.vision.visionAccurate()) {
  //         return Robot.vision.botPose;
  //     } else {
  //         return estimatePose;
  //     }
  // }

  /**
   * Helper method for comparing vision pose against odometry pose. Does not account for difference
   * in rotation. Will return false vision if it sees no targets or if the vision estimated pose is
   * too far from the odometry estimate
   *
   * @return whether or not pose should be added to estimate or not
   */
  public boolean isEstimateReady(Pose2d pose) {
    /* Disregard Vision if there are no targets in view */
    if (RobotContainer.m_vision.visionAccurate(pose)) { // visionAccurate method sees if Apriltags present in Vision.java
      return true;
    }

    // Disregard measurements too far away from odometry
    // this can be tuned to find a threshold that helps us remove jumping vision
    // poses
    return (Math.abs(pose.getX() - odometryPose.getX()) <= 0) // VisionConfig.DIFFERENCE_CUTOFF_THRESHOLD
        && (Math.abs(pose.getY() - odometryPose.getY()) <= 0);
  }

  /** Sets the Odometry Pose to the given pose */
  public void setOdometryPose(Pose2d pose) {
    odometryPose = pose;
  }

  /** Returns the Odometry Pose from drivetrain */
  public Pose2d getOdometryPose() {
    return odometryPose;
  }

  /** Sets the desired pose of the robot */
  public void setDesiredPose(Pose2d pose) {
    desiredPose = pose;
  }

  /** Sets the estimated pose to the given pose */
  public void setEstimatedPose(Pose2d pose) {
    estimateFrontPose = pose;
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometryEstimate() {
    poseEstimator.update(drivetrain.getYaw(), drivetrain.getSwerveModulePositions());
  }

  /**
   * @see edu.wpi.first.math.estimator.PoseEstimator#addVisionMeasurement(Pose2d, double)
   */
  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
    poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
  }

  /**
   * Reset the pose estimator location and Drivetrain odometry - NEEDS TO BE TESTED
   *
   * @param poseMeters
   */
  public void resetPoseEstimate(Pose2d poseMeters) {
    poseEstimator.resetPosition(drivetrain.getYaw(), drivetrain.getSwerveModulePositions(), poseMeters);
    drivetrain.resetOdometry(poseMeters);
  }

  public void resetHeading(Rotation2d angle) {
    drivetrain.resetOdometry(new Pose2d(drivetrain.getPose().getTranslation(), angle));
    resetPoseEstimate(new Pose2d(estimateFrontPose.getTranslation(), angle));
  }

  public void resetLocationEstimate(Translation2d translation) {
    resetPoseEstimate(new Pose2d(translation, new Rotation2d(0)));
  }

  /**
   * Gets the pose of the robot at the current time as estimated by the poseEstimator. This includes
   * vision and odometry combined together.
   *
   * @return The estimated robot pose in meters.
   */
  public Pose2d getPosition() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Get the heading of the robot estimated by the poseEstimator. Use this in most places we would
   * use the gyro.
   *
   * @return
   */
  public Rotation2d getHeading() {
    return estimateFrontPose.getRotation();
  }

  public Translation2d getLocation() {
    return estimateFrontPose.getTranslation();
  }

  public Pose2d getEstimatedPose() {
    return estimateFrontPose;
  }

  /**
   * Creates a vector of standard deviations for the states. Standard deviations of model states.
   * Increase these numbers to trust your model's state estimates less.
   *
   * @param x in meters
   * @param y in meters
   * @param theta in degrees
   * @return the Vector of standard deviations need for the poseEstimator
   */
  public Vector<N3> createStateStdDevs(double x, double y, double theta) {
    return VecBuilder.fill(x, y, Units.degreesToRadians(theta));
  }

  /**
   * Creates a vector of standard deviations for the local measurements. Standard deviations of
   * encoder and gyro rate measurements. Increase these numbers to trust sensor readings from
   * encoders and gyros less.
   *
   * @param theta in degrees per second
   * @param s std for all module positions in meters per sec
   * @return the Vector of standard deviations need for the poseEstimator
   */
  public Vector<N5> createLocalMeasurementStdDevs(double theta, double s) {
    return VecBuilder.fill(Units.degreesToRadians(theta), s, s, s, s);
  }

  /**
   * Creates a vector of standard deviations for the vision measurements. Standard deviations of
   * global measurements from vision. Increase these numbers to trust global measurements from
   * vision less.
   *
   * @param x in meters
   * @param y in meters
   * @param theta in degrees
   * @return the Vector of standard deviations need for the poseEstimator
   */
  public Vector<N3> createVisionMeasurementStdDevs(double x, double y, double theta) {
    return VecBuilder.fill(x, y, Units.degreesToRadians(theta));
  }

  /**
   * Commnad to reset odometry of drivetrain and pose esimator to the one from vision
   * @return a command to reset the Pose Estimator and Drivetrain to the vision pose
   */
  public Command resetOdometryVisionCommand(){
    return new InstantCommand(() -> resetPoseEstimate(RobotContainer.m_vision.visionBotPoseFront()));
  }

  public Command tempResetOdometryCOmmand(){
    return new InstantCommand(() -> resetPoseEstimate(new Pose2d(2, 5.52, new Rotation2d(0))));
  }
  
}