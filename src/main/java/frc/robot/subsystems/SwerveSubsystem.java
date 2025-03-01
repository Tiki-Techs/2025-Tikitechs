package frc.robot.subsystems;


// import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
// import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import java.io.File;
import java.util.Map;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
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
    double maximumSpeed = Units.feetToMeters(12.5);
    public static double visDist = 0.4;
    Map<String, Double> visionDistances = Map.of(
            "Coral Feeder", 0.5,
            "Algae Processor", 0.6,
            "Net", 0.7,
            "Reef", 0.4
        );

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
   
    
    AutoBuilder.configure(
      this::getPose, // Robot pose supplier
      this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      (speeds, feedforwards) -> getRobotRelativeSpeeds(), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
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
     
      this // Reference to this subsystem to set requirements
);
    }

    public double distance () {
      if (RobotContainer.m_vision.id != ""){
        return visionDistances.get(RobotContainer.m_vision.id);
      }
      else {
        if (Vision.cam == "back") {
          return 1.0; // change
        }
        else {
          return -1.0; // change
        }
      }
      // return 1.0;
    }
    

        
    public Pose2d getPose(){
      return getPose();
    }
    public ChassisSpeeds getRobotRelativeSpeeds(){
      return getRobotRelativeSpeeds();
    }
    public void resetOdometry(Pose2d initialHolonomicPose) {
      swerveDrive.resetOdometry(initialHolonomicPose); 
    }

    
    public void updateOdometry(){
        swerveDrive.updateOdometry();
    }

    /**
   * Command to drive the robot using translative values and heading as a setpoint.
   *
   * @param translationX Translation in the X direction.
   * @param translationY Translation in the Y direction.
   * @param headingX     Heading X to calculate angle of the joystick.
   * @param headingY     Heading Y to calculate angle of the joystick.
   * @return Drive command.
   */
  
  public Command driveCommandF(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
                              DoubleSupplier headingY)
  {
    // swerveDrive.setHeadingCorrection(true); 
    return run(() -> {

      Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
                                                                                 translationY.getAsDouble()), 0.8);

      // Make the robot move
      swerveDrive.driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
                                                                      headingX.getAsDouble(),
                                                                      headingY.getAsDouble(),
                                                                      swerveDrive.getOdometryHeading().getRadians(),
                                                                      swerveDrive.getMaximumChassisVelocity()));
    });
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
                                          swerveDrive.getOdometryHeading().getRadians() + angularRotationX.getAsDouble()*3.14,
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



public Command driveCommandR(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX, DoubleSupplier headingY)
{
  // swerveDrive.setHeadingCorrection(true); 
return run(() -> {

Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
                                                     translationY.getAsDouble()), 0.8);

// Make the robot move
swerveDrive.drive(swerveDrive.swerveController.getTargetSpeeds(
                                          scaledInputs.getX(), 
                                          scaledInputs.getY(),
                                          headingX.getAsDouble(),
                                          headingY.getAsDouble(),
                                          swerveDrive.getOdometryHeading().getRadians(),
                                          swerveDrive.getMaximumChassisVelocity()));
});
}



  /**
   * Command to drive the robot using translative values and heading as angular velocity.
   *
   * @param translationX     Translation in the X direction.
   * @param translationY     Translation in the Y direction.
   * @param angularRotationX Rotation of the robot to set
   * @return Drive command.
   */

  

  public double getOdometryHeading(){
    return swerveDrive.getOdometryHeading().getDegrees();
  }
   
  public Command AlignTest() {
    return run (() -> {
      if (Vision.cam == "back") {
        if(MathUtil.applyDeadband(Vision.skew, 6) != 0){
          swerveDrive.drive(
          swerveDrive.swerveController.getTargetSpeeds(0,
          // 0, Math.min(0.15*(Math.signum(Vision.angleX-Vision.skew)), (Vision.angleX-Vision.skew)*0.2), 
          0,
          // swerveDrive.getOdometryHeading().getRadians()-(0.1*Math.signum(Vision.skew)), 
          swerveDrive.getOdometryHeading().getRadians()-(0.5*Math.signum(Vision.skew)), 
          swerveDrive.getOdometryHeading().getRadians(), 
          swerveDrive.getMaximumChassisVelocity()));
          // SmartDashboard.putString("Align Test 1", "skew");
      }

        else if (MathUtil.applyDeadband(Vision.angleX, 5) != 0){
          swerveDrive.drive(swerveDrive.swerveController.getTargetSpeeds(
          0, 0.05*Math.signum(Vision.angleX-Vision.skew),
          swerveDrive.getOdometryHeading().getRadians(), 
          swerveDrive.getOdometryHeading().getRadians(), 
          swerveDrive.getMaximumChassisVelocity()));
          // SmartDashboard.putString("Align Test 1", "center");
        }

        else if (MathUtil.applyDeadband(Vision.distance-distance(), 0.05) != 0){
          swerveDrive.drive(swerveDrive.swerveController.getTargetSpeeds(
          -0.2*(Math.signum(Vision.distance-distance())), 0,
          swerveDrive.getOdometryHeading().getRadians(), 
          swerveDrive.getOdometryHeading().getRadians(), 
          swerveDrive.getMaximumChassisVelocity()));
          // SmartDashboard.putString("Align Test 1", "distance");
        }

        else {
          swerveDrive.drive(swerveDrive.swerveController.getTargetSpeeds(0, 0, swerveDrive.getOdometryHeading().getRadians(), swerveDrive.getOdometryHeading().getRadians(), swerveDrive.getMaximumChassisVelocity()));
          // SmartDashboard.putString("Align Test 1", "good");
        }
      }
    else if (Vision.cam == "front") {
      if(MathUtil.applyDeadband(Vision.skew, 6) != 0){
        swerveDrive.drive(
        swerveDrive.swerveController.getTargetSpeeds(0,
        0,
        swerveDrive.getOdometryHeading().getRadians()-(0.5*Math.signum(Vision.angleX-Vision.skew)), 
        swerveDrive.getOdometryHeading().getRadians(), 
        swerveDrive.getMaximumChassisVelocity()));
        // SmartDashboard.putString("Align Test 1", "skew");
    }

      else if (MathUtil.applyDeadband(Vision.angleX, 5) != 0){
        swerveDrive.drive(swerveDrive.swerveController.getTargetSpeeds(
        0, -0.05*Math.signum(Vision.angleX-Vision.skew),
        swerveDrive.getOdometryHeading().getRadians(), 
        swerveDrive.getOdometryHeading().getRadians(), 
        swerveDrive.getMaximumChassisVelocity()));
        // SmartDashboard.putString("Align Test 1", "center");
      }

      else if (MathUtil.applyDeadband(Vision.distance-distance(), 0.05) != 0){
        swerveDrive.drive(swerveDrive.swerveController.getTargetSpeeds(
        0.2*(Math.signum(Vision.distance-distance())), 0,
        swerveDrive.getOdometryHeading().getRadians(), 
        swerveDrive.getOdometryHeading().getRadians(), 
        swerveDrive.getMaximumChassisVelocity()));
        // SmartDashboard.putString("Align Test 1", "distance");
      }

      else {
        swerveDrive.drive(swerveDrive.swerveController.getTargetSpeeds(0, 0, swerveDrive.getOdometryHeading().getRadians(), swerveDrive.getOdometryHeading().getRadians(), swerveDrive.getMaximumChassisVelocity()));
        // SmartDashboard.putString("Align Test 1", "good");
      }
    }
    });
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
        0, // Set Z to 0 or use a sensor if available
        swerveDrive.getGyroRotation3d()
    );
    
    // Publish the updated pose
    publisher.set(robotPose);
    // SmartDashboard.putNumber("Encoder Back Left", swerveDrive.getModules()[2].getAbsoluteEncoder().getAbsolutePosition());
    // SmartDashboard.putNumber("Encoder Front Right", swerveDrive.getModules()[1].getAbsoluteEncoder().getAbsolutePosition());
    // SmartDashboard.putNumber("Encoder Front Left", swerveDrive.getModules()[0].getAbsoluteEncoder().getAbsolutePosition());
    // SmartDashboard.putNumber("Encoder Back Right", swerveDrive.getModules()[3].getAbsoluteEncoder().getAbsolutePosition());
    // SmartDashboard.putNumber("Odometry", swerveDrive.getOdometryHeading().getDegrees());
    // SmartDashboard.putNumber("gyro test", swerveDrive.getGyroRotation3d().getAngle());
    m_field.setRobotPose(swerveDrive.getPose());

    if(MathUtil.applyDeadband(Vision.skew, 6) != 0){
        SmartDashboard.putString("Align Test 1", "skew");
    }

    else if (MathUtil.applyDeadband(Vision.angleX, 5) != 0){
      SmartDashboard.putString("Align Test 1", "center");
    }

    // else if (MathUtil.applyDeadband(Vision.distance-distance(), 0.05) != 0){
    //   SmartDashboard.putString("Align Test 1", "distance");
    // }

    else {
      SmartDashboard.putString("Align Test 1", "good");
    }
    
    SmartDashboard.putNumber("map test", distance());
    }
  }

// }
