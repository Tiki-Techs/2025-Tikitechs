package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

// import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
// import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import java.io.File;
import java.util.Map;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.photonvision.PhotonUtils;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveModuleConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase{
    public SwerveDrive swerveDrive;
    private static SwerveSubsystem instance;
    private final Field2d m_field = new Field2d();
    
    private final Field2d m_fieldEstimated = new Field2d();
    // double maximumSpeed = Units.feetToMeters(25);
    // double maximumSpeed = Units.feetToMeters(17.1);
    
    double maximumSpeed = Units.feetToMeters(17.1);
    public static double visDist = 0.4;    
    Map<String, Double> visionDistances = Map.of(
            "Coral Feeder", 0.5,
            "Algae Processor", 0.6,
            "Net", 0.7,
            "Reef", 1.0
        );
    
    public PIDController turnPID = new PIDController(0.055, 0, 0.000); // TUNE PID
    public PIDController angleXPID = new PIDController(0.05, 0, 0.000); // TUNE PID
    public PIDController distancePID = new PIDController(0.065, 0, 0.000); // TUNE PID
    public double driveCoeff = 1;
    public double rotAlign = 0;
    public final Timer m_Timer = new Timer();
    double tag = -124912;
    Pose3d robotPose = new Pose3d();
    Pose2d estimatedRobotPose = new Pose2d();
// Pose3d poseB = new Pose3d();
StructPublisher<Pose3d> publisher = NetworkTableInstance.getDefault()
  .getStructTopic("MyPose", Pose3d.struct).publish();

  
// StructPublisher<Pose3d> publisherTogether = NetworkTableInstance.getDefault()
// .getStructTopic("MyPoseTogether", Pose3d.struct).publish();
  
// StructPublisher<Pose3d> publisherEst = NetworkTableInstance.getDefault()
// .getStructTopic("MyPoseEst", Pose3d.struct).publish();
// StructArrayPublisher<Pose3d> arrayPublisher = NetworkTableInstance.getDefault()
//   .getStructArrayTopic("MyPoseArray", Pose3d.struct).publish();

    public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    // public static SwerveSubsystem getInstance() {
    //  " if (instance == null) instance = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve"));
    //   return instance;
    // }"

    // File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
    public SwerveSubsystem(File directory){
        try
        {
                SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed);
      // Alternative method if you don't want to supply the conversion factor via JSON files.
      // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
        } catch (Exception e)
        {
          throw new RuntimeException(e);
        }
        // Do this in either robot or subsystem init
        SmartDashboard.putData("Field", m_field);

        swerveDrive.setMaximumAllowableSpeeds(3, 3);

    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
   
    
    final boolean enableFeedforward = false; // unsure if want, needs research
    AutoBuilder.configure(
      this::getPose, // Robot pose supplier
      this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      (speeds, feedforwards) -> {if (enableFeedforward) {
        swerveDrive.drive(speeds, 
        swerveDrive.kinematics.toSwerveModuleStates(speeds),
        feedforwards.linearForces()); 
        // unsure on this, but is what is in the other team's code as well as a command built into swerve drive
      }
      else {
        swerveDrive.setChassisSpeeds(speeds);
      }
    },
      // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
      // new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
      //         new PIDConstants(0.03, 0.0, 0.0), // Translation PID constants
      //         new PIDConstants(0.08, 0.0, 0.0) // Rotation PID constants
      // ),
      new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
              new PIDConstants(1.7, 0.0, 0.11), // Translation PID constants
              new PIDConstants(1.2, 0.0, 0.09) // Rotation PID constants
      ),
 
      Constants.robotConfig, // The robot configuration
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
     // check on the thing about warmup in the other code, and check their basic auto selection/init.
      this // Reference to this subsystem to set requirements
);
    }

    public SwerveDriveKinematics getKinematics(){
      return swerveDrive.kinematics;
    }

    public Rotation2d getYaw() {
      return swerveDrive.getYaw();
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
      return swerveDrive.getModulePositions();
    }
        
    public Pose2d getPose(){
      return swerveDrive.getPose();
    }


    public ChassisSpeeds getRobotRelativeSpeeds(){
      return swerveDrive.getRobotVelocity();
      // HolonomicDriveController m_a = 
    }

    // public void drive() {
    //   swerveDrive.drive();
    // }

    public void resetOdometry(Pose2d initialHolonomicPose) {
      swerveDrive.resetOdometry(initialHolonomicPose); 
      
    }

    
    public void updateOdometry(){
        swerveDrive.updateOdometry();
    }

  public Command stopDrive()
{
return run(() -> {
swerveDrive.driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(0, 0,
                                          swerveDrive.getOdometryHeading().getRadians(),
                                          swerveDrive.getOdometryHeading().getRadians(),
                                          swerveDrive.getMaximumChassisVelocity()));
});
}

public Command rotateAlign()
{
return run(() -> {
swerveDrive.driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(0, 0,
                                          Math.toRadians(-60 * RobotContainer.m_vision.closestTag() + rotAlign),
                                          swerveDrive.getOdometryHeading().getRadians(),
                                          swerveDrive.getMaximumChassisVelocity()));
});
}

public Command rotateAlignL1()
{
return run(() -> {
swerveDrive.driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(0, 0,
                                          Math.toRadians(-60 * RobotContainer.m_vision.closestTag() + rotAlign + 90),
                                          swerveDrive.getOdometryHeading().getRadians(),
                                          swerveDrive.getMaximumChassisVelocity()));
});
}

// (-60 * side.getAsDouble())-90

public Command zeroGyro()
{
return run(() -> {
  swerveDrive.zeroGyro();
});
}


  public Command driveCommandF(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
{
return run(() -> {

Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
                                                     translationY.getAsDouble()), 0.8);

// Make the robot move
swerveDrive.driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(
                                          scaledInputs.getX(), 
                                          scaledInputs.getY(),
                                          swerveDrive.getOdometryHeading().getRadians() + angularRotationX.getAsDouble()*1.4,
                                          swerveDrive.getOdometryHeading().getRadians(),
                                          swerveDrive.getMaximumChassisVelocity()));
});
}


public Command driveCommandFScaledDown(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
{
return run(() -> {

Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
                                                     translationY.getAsDouble()), 0.8);

// Make the robot move
swerveDrive.driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(
                                          scaledInputs.getX()*0.3, 
                                          scaledInputs.getY()*0.3,
                                          swerveDrive.getOdometryHeading().getRadians() + angularRotationX.getAsDouble()*0.5,
                                          swerveDrive.getOdometryHeading().getRadians(),
                                          swerveDrive.getMaximumChassisVelocity()));
});
}



public Command driveCommandR(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
{
return run(() -> {

Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
                                                     translationY.getAsDouble()), 0.8);

// Make the robot move
swerveDrive.drive(swerveDrive.swerveController.getTargetSpeeds(
                                          scaledInputs.getX()*0.4, 
                                          scaledInputs.getY()*0.4,
                                          swerveDrive.getOdometryHeading().getRadians() + angularRotationX.getAsDouble()*0.30,
                                          swerveDrive.getOdometryHeading().getRadians(),
                                          swerveDrive.getMaximumChassisVelocity()));
});
}

  public Command x() {
    return new InstantCommand(
      () -> {
        SmartDashboard.putString("test 111", "getName()");
  //       m_Timer.reset();
  //       m_Timer.start();
  //       while (m_Timer.get() <= 1) {
  // swerveDrive.drive(swerveDrive.swerveController.getTargetSpeeds(
  //   0.07, 
  //   0,
  //   swerveDrive.getOdometryHeading().getRadians(),
  //   swerveDrive.getOdometryHeading().getRadians(),
  //   swerveDrive.getMaximumChassisVelocity()));
  //       }
      });
    }

  //   public Command x2() {
  //     return new RunCommand(
  //         () -> {
  //             m_Timer.reset();
  //             m_Timer.start();
  //         while (m_Timer.get() <= 0.5) {
  //           swerveDrive.drive(swerveDrive.swerveController.getTargetSpeeds(
  //                   0, 
  //                   0,
  //                   swerveDrive.getOdometryHeading().getRadians()-2,
  //                   swerveDrive.getOdometryHeading().getRadians(),
  //                   swerveDrive.getMaximumChassisVelocity()));
  //         }}
  //     );
  // }

  public Command x2() {
    return new RunCommand(
      () ->
          swerveDrive.drive(swerveDrive.swerveController.getTargetSpeeds(
                  0, 
                  0,
                  swerveDrive.getOdometryHeading().getRadians()-2,
                  swerveDrive.getOdometryHeading().getRadians(),
                  swerveDrive.getMaximumChassisVelocity()))
    );}

  public double getOdometryHeading(){
    return swerveDrive.getOdometryHeading().getDegrees();
  }
   
  public double calcDrive() {
    // driveCoeff = 0.2 + (0.8 * (50.9472-RobotContainer.m_elevator.encoderValue));
    driveCoeff = 0.2 + (0.8 * (25.9472-RobotContainer.m_elevator.elevatorPosition));
    if (driveCoeff > 1) {
      driveCoeff = 1;
    }

    return driveCoeff;
  }


public Pose3d getRobotPose() {
  return robotPose;
}

public int together() {
  int tg = closestTag();
  if (tg != 0) {
    SmartDashboard.putNumber("tg 0", tg);
    return 1;
  }
  else {
    return 0;
  }
}

    public double getDistanceFromAprilTag(int id) {
      Optional<Pose3d> tag = fieldLayout.getTagPose(id);
      return tag.map(pose3d -> PhotonUtils.getDistanceToPose(robotPose.toPose2d(), pose3d.toPose2d())).orElse(1000.0);
    }

    public int closestTag() {
    boolean tempVisionTest = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
    double lowest = 1000;
    if (tempVisionTest) {
      for (int i = 0; i < 6; i++) {
        if (getDistanceFromAprilTag(i+6) < lowest) {
          lowest = getDistanceFromAprilTag(i+6);
          tag = (-(i+6)%6)+1;
          // SmartDashboard.putNumber("closest id", tag);
        }
      }
    }
    else {
      for (int i = 0; i < 6; i++) {
        if (getDistanceFromAprilTag(i+17) < lowest) {
          lowest = getDistanceFromAprilTag(i+17);
          tag = (i+17)%6;
          SmartDashboard.putNumber("closest id", tag);
        }
      }
    }
    return (int)tag;
  }

  public Command pt1() {
    
        return new InstantCommand(
                () -> {
    SmartDashboard.putNumber("pt1", tag);
                }, this);
  }


// Pose at midpoint between tags 18 and 21 (which are opposite on blue reef)
private static final Translation2d REEF_CENTER_BLUE = fieldLayout.getTagPose(18).get().toPose2d().getTranslation()
    .plus(fieldLayout.getTagPose(21).get().toPose2d().getTranslation()).div(2);

// Pose at midpoint between tags 10 and 7 (which are opposite on red reef)
private static final Translation2d REEF_CENTER_RED = fieldLayout.getTagPose(10).get().toPose2d().getTranslation()
    .plus(fieldLayout.getTagPose(7).get().toPose2d().getTranslation()).div(2);

private static boolean flipToRed; // whether to use red reef (otherwise blue)

// Distance from center of robot to center of reef


public static final Distance DISTANCE_TO_REEF = Inches.of(29 / 2).plus(Inches.of(4)); // 4 is bumper thickness


// Found by taking distance from tag 18 to center and adding offset from reef
private static final Distance REEF_APOTHEM = Meters.of(
        fieldLayout.getTagPose(18).get().toPose2d().getTranslation().getDistance(REEF_CENTER_BLUE))
        .plus(DISTANCE_TO_REEF);

// translation to move from centered on a side to scoring position for the left branch
private static final Translation2d CENTERED_TO_LEFT_BRANCH = new Translation2d(Meters.of(0),
        Inches.of(12.94 / 2));

/**
 * Calculates the pose of the robot for scoring on a branch or trough.
 *
 * @param side The side of the reef (0 for left, increases clockwise).
 * @param relativePos The relative position on the reef (-1 for right branch, 0 for center, 1 for left branch).
 * @return The calculated Pose2d for scoring.
 */
public static Pose2d getReefPose(int side, int relativePos) {
    // determine whether to use red or blue reef position
    flipToRed = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;

    // initially do all calculations from blue, then flip later
    Translation2d reefCenter = REEF_CENTER_BLUE;

    // robot position centered on close reef side
    Translation2d translation = reefCenter.plus(new Translation2d(REEF_APOTHEM.unaryMinus(), Meters.zero()));
    // translate to correct branch (left, right, center)
    translation = translation.plus(CENTERED_TO_LEFT_BRANCH.times(relativePos));
    // rotate to correct side
    translation = translation.rotateAround(reefCenter, Rotation2d.fromDegrees(-60 * side));

    // make pose from translation and correct rotation
    Pose2d reefPose = new Pose2d(translation,
            Rotation2d.fromDegrees(-60 * side));

    if (flipToRed) {
        reefPose = flipPose(reefPose);
    }

    return reefPose;
}

private static Pose2d flipPose(Pose2d pose) {
    Translation2d center = REEF_CENTER_BLUE.interpolate(REEF_CENTER_RED, 0.5);
    Translation2d poseTranslation = pose.getTranslation();
    poseTranslation = poseTranslation.rotateAround(center, Rotation2d.k180deg);
    return new Pose2d(poseTranslation, pose.getRotation().rotateBy(Rotation2d.k180deg));
}



public Pose3d pose2D3D (Pose2d pose2d) {
        return new Pose3d(
            new Translation3d(pose2d.getX(), pose2d.getY(), 0), // Assume 0 height (Z)
            new Rotation3d(0, 0, pose2d.getRotation().getRadians()) // Only rotate around Z
        );
    }

public Rotation3d rot2D3D(Rotation2d rot2d) {
  return new Rotation3d(0, 0, rot2d.getDegrees());
}



@Override
public void periodic(){
  
  if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
    rotAlign = 180;
  }
  else {
    rotAlign = 0;
  }
    swerveDrive.updateOdometry();
    // robotPose = new Pose3d(
    //     RobotContainer.poseEstimator.getPosition().getX(),
    //     RobotContainer.poseEstimator.getPosition().getY(),
    //     0,
    //     rot2D3D(RobotContainer.poseEstimator.getPosition().getRotation())
        
    // );
    closestTag();
    robotPose = new Pose3d(swerveDrive.getPose().getX(), swerveDrive.getPose().getY(), 0, swerveDrive.getGyroRotation3d());
    
    // estimatedRobotPose = PoseEstimator.getInstance().getPosition();
    
    // estimatedRobotPose = PoseEstimator.getInstance().getEstimatedPose();
    // estimatedRobotPose = RobotContainer.pe.getPosition();
    
    // Publish the updated pose
    publisher.set(pose2D3D(swerveDrive.getPose()));
    // publisherEst.set(pose2D3D(estimatedRobotPose));
    // publisherTogether.set(pose2D3D(swerveDrive.getPose()));
    
    SmartDashboard.putNumber("Encoder Back Left", swerveDrive.getModules()[2].getAbsoluteEncoder().getAbsolutePosition());
    SmartDashboard.putNumber("Encoder Front Right", swerveDrive.getModules()[1].getAbsoluteEncoder().getAbsolutePosition());
    SmartDashboard.putNumber("Encoder Front Left", swerveDrive.getModules()[0].getAbsoluteEncoder().getAbsolutePosition());
    SmartDashboard.putNumber("Encoder Back Right", swerveDrive.getModules()[3].getAbsoluteEncoder().getAbsolutePosition());
    // SmartDashboard.putNumber("Encoder Front Right relative", swerveDrive.getModules()[1].getRelativePosition());
    // SmartDashboard.putNumber("Distance from 20", getDistanceFromAprilTag(20));
    // SmartDashboard.putNumber("Odometry", swerveDrive.getOdometryHeading().getDegrees());
    // SmartDashboard.putNumber("gyro test", swerveDrive.getGyroRotation3d().getAngle());
    m_field.setRobotPose(swerveDrive.getPose());
    m_fieldEstimated.setRobotPose(estimatedRobotPose);
    // m_field.setRobotPose(estimatedRobotPose);
    // SmartDashboard.putNumber("map test", distance());
    // SmartDashboard.putBoolean("Drive Toggle", RobotContainer.scaledToggle);
    }
  }

// }
