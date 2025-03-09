package frc.robot.subsystems;


// import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
// import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import java.io.File;
import java.util.Map;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
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
    double maximumSpeed = Units.feetToMeters(12);
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
                                          scaledInputs.getX()*.1, 
                                          scaledInputs.getY()*.1,
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
   
  public Command AlignRight() {
    return run (() -> {
    //   if (Vision.cam == "back") {
    //     if(MathUtil.applyDeadband(Vision.skew+7, 3) != 0){
    //       swerveDrive.drive(
    //       swerveDrive.swerveController.getTargetSpeeds(0,
    //       // 0, Math.min(0.15*(Math.signum(Vision.angleX-Vision.skew)), (Vision.angleX-Vision.skew)*0.2), 
    //       0,
    //       // swerveDrive.getOdometryHeading().getRadians()-(0.1*Math.signum(Vision.skew)), 
    //       swerveDrive.getOdometryHeading().getRadians()-(turnPID.calculate(0, Vision.skew+7)), 
    //       swerveDrive.getOdometryHeading().getRadians(), 
    //       swerveDrive.getMaximumChassisVelocity()));
    //       // SmartDashboard.putString("Align Test 1", "skew");
    //   }

    //     else if (MathUtil.applyDeadband(Vision.angleX, 5) != 0){
    //       swerveDrive.drive(swerveDrive.swerveController.getTargetSpeeds(
    //         0, angleXPID.calculate(0, Vision.angleX),
    //       swerveDrive.getOdometryHeading().getRadians(), 
    //       swerveDrive.getOdometryHeading().getRadians(), 
    //       swerveDrive.getMaximumChassisVelocity()));
    //       // SmartDashboard.putString("Align Test 1", "center");
    //     }

    //     else if (MathUtil.applyDeadband(Vision.distance-distance(), 0.05) != 0){
    //       swerveDrive.drive(swerveDrive.swerveController.getTargetSpeeds(
    //         distancePID.calculate(Vision.distance-distance(), 0), 0,
    //       swerveDrive.getOdometryHeading().getRadians(), 
    //       swerveDrive.getOdometryHeading().getRadians(), 
    //       swerveDrive.getMaximumChassisVelocity()));
    //       // SmartDashboard.putString("Align Test 1", "distance");
    //     }

    //     else {
    //       swerveDrive.drive(swerveDrive.swerveController.getTargetSpeeds(0, 0, swerveDrive.getOdometryHeading().getRadians(), swerveDrive.getOdometryHeading().getRadians(), swerveDrive.getMaximumChassisVelocity()));
    //       // SmartDashboard.putString("Align Test 1", "good");
    //     }
    //   }
    // else if (Vision.cam == "front") {
      if(MathUtil.applyDeadband(Vision.skew+0.3, 2) != 0){
        swerveDrive.drive(
        swerveDrive.swerveController.getTargetSpeeds(0,
        0,
        swerveDrive.getOdometryHeading().getRadians()-(Math.min(0.15, turnPID.calculate(0, Vision.skew+0.3))), 
        swerveDrive.getOdometryHeading().getRadians(), 
        swerveDrive.getMaximumChassisVelocity()));
        // AprilTagFields.k2025Reefscape;
        // SmartDashboard.putString("Align Test 1", "skew");
        SmartDashboard.putNumber("PID Target", swerveDrive.getOdometryHeading().getRadians()-(Math.min(0.15, turnPID.calculate(0, Vision.skew+0.3))));
    }


      else if (MathUtil.applyDeadband(Vision.angleX+13.5, 2) != 0){
        swerveDrive.drive(swerveDrive.swerveController.getTargetSpeeds(
        0, angleXPID.calculate(Math.min(0.15, Vision.skew+13.5)),
        swerveDrive.getOdometryHeading().getRadians(), 
        swerveDrive.getOdometryHeading().getRadians(), 
        swerveDrive.getMaximumChassisVelocity()));
        // SmartDashboard.putString("Align Test 1", "center");
        SmartDashboard.putNumber("PID Target", angleXPID.calculate(Math.min(0.15, Vision.skew+13.5)));
      }

      else if (MathUtil.applyDeadband(Vision.distance-0.3, 0.025) != 0){
        swerveDrive.drive(swerveDrive.swerveController.getTargetSpeeds(
        distancePID.calculate(0, Math.min(0.15, Vision.distance-0.3)), 0,
        swerveDrive.getOdometryHeading().getRadians(), 
        swerveDrive.getOdometryHeading().getRadians(), 
        swerveDrive.getMaximumChassisVelocity()));
        // SmartDashboard.putString("Align Test 1", "distance");
        SmartDashboard.putNumber("PID Target", distancePID.calculate(0, Math.min(0.15, Vision.distance-0.3)));
      }

      else {
        swerveDrive.drive(swerveDrive.swerveController.getTargetSpeeds(0, 0, swerveDrive.getOdometryHeading().getRadians(), swerveDrive.getOdometryHeading().getRadians(), swerveDrive.getMaximumChassisVelocity()));
        // SmartDashboard.putString("Align Test 1", "good");
      }
    // }
    });
  }
  //  37.8 to 1
  //  51.975

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



@Override
public void periodic(){
    swerveDrive.updateOdometry();
    robotPose = new Pose3d(
        swerveDrive.getPose().getX(),
        swerveDrive.getPose().getY(),
        0,
        swerveDrive.getGyroRotation3d()
    );
    
  

    // Publish the updated pose
    publisher.set(robotPose);
    SmartDashboard.putNumber("Encoder Back Left", swerveDrive.getModules()[2].getAbsoluteEncoder().getAbsolutePosition());
    SmartDashboard.putNumber("Encoder Front Right", swerveDrive.getModules()[1].getAbsoluteEncoder().getAbsolutePosition());
    SmartDashboard.putNumber("Encoder Front Left", swerveDrive.getModules()[0].getAbsoluteEncoder().getAbsolutePosition());
    SmartDashboard.putNumber("Encoder Back Right", swerveDrive.getModules()[3].getAbsoluteEncoder().getAbsolutePosition());
    SmartDashboard.putNumber("Encoder Front Right relative", swerveDrive.getModules()[1].getRelativePosition());
    // SmartDashboard.putNumber("Odometry", swerveDrive.getOdometryHeading().getDegrees());
    // SmartDashboard.putNumber("gyro test", swerveDrive.getGyroRotation3d().getAngle());
    m_field.setRobotPose(swerveDrive.getPose());
  
    if(MathUtil.applyDeadband(Vision.skew+0.3, 3) != 0){
        SmartDashboard.putString("Align Test 1", "skew");
    }

    else if (MathUtil.applyDeadband(Vision.angleX+13.5, 2) != 0){
      SmartDashboard.putString("Align Test 1", "center");
    }

    else if (MathUtil.applyDeadband(Vision.distance-0.3, 0.05) != 0){
      SmartDashboard.putString("Align Test 1", "distance");
    }

    else {
      SmartDashboard.putString("Align Test 1", "good");
    }
    
    // SmartDashboard.putNumber("map test", distance());
    // SmartDashboard.putBoolean("Drive Toggle", RobotContainer.scaledToggle);
    }
  }

// }
