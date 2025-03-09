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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase{
    private SwerveDrive swerveDrive;
    private final Field2d m_field = new Field2d();
    // double maximumSpeed = Units.feetToMeters(25);
    double maximumSpeed = Units.feetToMeters(17.1);
    public static double visDist = 0.4;    public static final AprilTagFieldLayout fieldLayout                     = AprilTagFieldLayout.loadField(      AprilTagFields.k2025ReefscapeWelded);

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
    double tag = -123912;

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
        swerveDrive.resetOdometry(new Pose2d(8.12, 5.81, new Rotation2d()));
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
      new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
              new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
              new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
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
                                          swerveDrive.getOdometryHeading().getRadians() + angularRotationX.getAsDouble()*0.8,
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
                                          scaledInputs.getX(), 
                                          scaledInputs.getY(),
                                          swerveDrive.getOdometryHeading().getRadians() + angularRotationX.getAsDouble()*3.14,
                                          swerveDrive.getOdometryHeading().getRadians(),
                                          swerveDrive.getMaximumChassisVelocity()));
});
}

  public double getOdometryHeading(){
    return swerveDrive.getOdometryHeading().getDegrees();
  }
   
  public double calcDrive() {
    // driveCoeff = 0.2 + (0.8 * (50.9472-RobotContainer.m_elevator.encoderValue));
    driveCoeff = 0.2 + (0.8 * (25.9472-RobotContainer.m_elevator.encoderValue));
    if (driveCoeff > 1) {
      driveCoeff = 1;
    }

    return driveCoeff;
  }
   
Pose3d robotPose = new Pose3d();
// Pose3d poseB = new Pose3d();
StructPublisher<Pose3d> publisher = NetworkTableInstance.getDefault()
  .getStructTopic("MyPose", Pose3d.struct).publish();
StructArrayPublisher<Pose3d> arrayPublisher = NetworkTableInstance.getDefault()
  .getStructArrayTopic("MyPoseArray", Pose3d.struct).publish();



// public Command testAutoAnd(){
//   PathConstraints constraints = new PathConstraints(
//         3.0, 4.0,
//         Units.degreesToRadians(540), Units.degreesToRadians(720));

// // Since AutoBuilder is configured, we can use it to build pathfinding commands
// // PathPlannerPath path = PathPlannerPath.
// return AutoBuilder.pathfindToPose(
        
//   getReefPose(1, 0),
//         constraints);
//   // return null;
// }

//     public double getDistanceFromAprilTag(int id) {
//       Optional<Pose3d> tag = fieldLayout.getTagPose(id);
//       return tag.map(pose3d -> PhotonUtils.getDistanceToPose(robotPose.toPose2d(), pose3d.toPose2d())).orElse(1000.0);
//     }

//   //   public int closestTag() {
//   //   boolean tempVisionTest = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
//   //   double lowest = 1000;
//   //   if (tempVisionTest) {
//   //     for (int i = 0; i < 6; i++) {
//   //       if (getDistanceFromAprilTag(i+6) < lowest) {
//   //         lowest = getDistanceFromAprilTag(i+6);
//   //         tag = i+6;
//   //         SmartDashboard.putNumber("closest id", tag);
//   //       }
//   //     }
//   //   }
//   //   else {
//   //     for (int i = 0; i < 6; i++) {
//   //       if (getDistanceFromAprilTag(i+17) < lowest) {
//   //         lowest = getDistanceFromAprilTag(i+17);
//   //         tag = i+17;
//   //         SmartDashboard.putNumber("closest id", tag);
//   //       }
//   //     }
//   //   }
//   //   return (int)tag;
//   // }

// // Pose at midpoint between tags 18 and 21 (which are opposite on blue reef)
// private static final Translation2d REEF_CENTER_BLUE = fieldLayout.getTagPose(18).get().toPose2d().getTranslation()
//     .plus(fieldLayout.getTagPose(21).get().toPose2d().getTranslation()).div(2);

// // Pose at midpoint between tags 10 and 7 (which are opposite on red reef)
// private static final Translation2d REEF_CENTER_RED = fieldLayout.getTagPose(10).get().toPose2d().getTranslation()
//     .plus(fieldLayout.getTagPose(7).get().toPose2d().getTranslation()).div(2);

// private static boolean flipToRed; // whether to use red reef (otherwise blue)

// // Distance from center of robot to center of reef


// public static final Distance DISTANCE_TO_REEF = Inches.of(29 / 2).plus(Inches.of(4)); // 4 is bumper thickness


// // Found by taking distance from tag 18 to center and adding offset from reef
// private static final Distance REEF_APOTHEM = Meters.of(
//         fieldLayout.getTagPose(18).get().toPose2d().getTranslation().getDistance(REEF_CENTER_BLUE))
//         .plus(DISTANCE_TO_REEF);

// // translation to move from centered on a side to scoring position for the left branch
// private static final Translation2d CENTERED_TO_LEFT_BRANCH = new Translation2d(Meters.of(0),
//         Inches.of(12.94 / 2));

// /**
//  * Calculates the pose of the robot for scoring on a branch or trough.
//  *
//  * @param side The side of the reef (0 for left, increases clockwise).
//  * @param relativePos The relative position on the reef (-1 for right branch, 0 for center, 1 for left branch).
//  * @return The calculated Pose2d for scoring.
//  */
// public static Pose2d getReefPose(int side, int relativePos) {
//     // determine whether to use red or blue reef position
//     flipToRed = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;

//     // initially do all calculations from blue, then flip later
//     Translation2d reefCenter = REEF_CENTER_BLUE;

//     // robot position centered on close reef side
//     Translation2d translation = reefCenter.plus(new Translation2d(REEF_APOTHEM.unaryMinus(), Meters.zero()));
//     // translate to correct branch (left, right, center)
//     translation = translation.plus(CENTERED_TO_LEFT_BRANCH.times(relativePos));
//     // rotate to correct side
//     translation = translation.rotateAround(reefCenter, Rotation2d.fromDegrees(-60 * side));

//     // make pose from translation and correct rotation
//     Pose2d reefPose = new Pose2d(translation,
//             Rotation2d.fromDegrees(-60 * side));

//     if (flipToRed) {
//         reefPose = flipPose(reefPose);
//     }

//     return reefPose;
// }

// private static Pose2d flipPose(Pose2d pose) {
//     Translation2d center = REEF_CENTER_BLUE.interpolate(REEF_CENTER_RED, 0.5);
//     Translation2d poseTranslation = pose.getTranslation();
//     poseTranslation = poseTranslation.rotateAround(center, Rotation2d.k180deg);
//     return new Pose2d(poseTranslation, pose.getRotation().rotateBy(Rotation2d.k180deg));
// }











@Override
public void periodic(){
    swerveDrive.updateOdometry();
    robotPose = new Pose3d(
        swerveDrive.getPose().getX(),
        swerveDrive.getPose().getY(),
        0,
        swerveDrive.getGyroRotation3d()
        
    );
    
    // boolean tempVisionTest = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
    // double lowest = 1000;
    // if (tempVisionTest) {
    //   for (int i = 0; i < 6; i++) {
    //     if (getDistanceFromAprilTag(i+6) < lowest) {
    //       lowest = getDistanceFromAprilTag(i+6);
    //       tag = i-7;
    //       SmartDashboard.putNumber("closest id", tag);
    //     }
    //   }
    // }
    // else {
    //   for (int i = 0; i < 6; i++) {
    //     if (getDistanceFromAprilTag(i+17) < lowest) {
    //       lowest = getDistanceFromAprilTag(i+17);
    //       tag = i-7;
    //       SmartDashboard.putNumber("closest id", tag);
    //     }
    //   }
    // }
  
    // Publish the updated pose
    publisher.set(robotPose);
    SmartDashboard.putNumber("Encoder Back Left", swerveDrive.getModules()[2].getAbsoluteEncoder().getAbsolutePosition());
    SmartDashboard.putNumber("Encoder Front Right", swerveDrive.getModules()[1].getAbsoluteEncoder().getAbsolutePosition());
    SmartDashboard.putNumber("Encoder Front Left", swerveDrive.getModules()[0].getAbsoluteEncoder().getAbsolutePosition());
    SmartDashboard.putNumber("Encoder Back Right", swerveDrive.getModules()[3].getAbsoluteEncoder().getAbsolutePosition());
    SmartDashboard.putNumber("Encoder Front Right relative", swerveDrive.getModules()[1].getRelativePosition());
    // SmartDashboard.putNumber("Distance from 20", getDistanceFromAprilTag(20));
    // SmartDashboard.putNumber("Odometry", swerveDrive.getOdometryHeading().getDegrees());
    // SmartDashboard.putNumber("gyro test", swerveDrive.getGyroRotation3d().getAngle());
    m_field.setRobotPose(swerveDrive.getPose());
    
    // SmartDashboard.putNumber("map test", distance());
    // SmartDashboard.putBoolean("Drive Toggle", RobotContainer.scaledToggle);
    }
  }

// }
