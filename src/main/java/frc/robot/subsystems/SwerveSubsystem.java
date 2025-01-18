package frc.robot.subsystems;


import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;

public class SwerveSubsystem extends SubsystemBase{
    private SwerveDrive swerveDrive;
    private final Field2d m_field = new Field2d();
    // double maximumSpeed = Units.feetToMeters(25);
    double maximumSpeed = Units.feetToMeters(12.5);
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
        swerveDrive.resetOdometry(new Pose2d(2, 2, new Rotation2d()));
    }

    public void zeroGyro() {
        swerveDrive.zeroGyro();
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
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
                              DoubleSupplier headingY)
  {
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









  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
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































  /**
   * Command to drive the robot using translative values and heading as angular velocity.
   *
   * @param translationX     Translation in the X direction.
   * @param translationY     Translation in the Y direction.
   * @param angularRotationX Rotation of the robot to set
   * @return Drive command.
   */
  // public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  // {
  //   return run(() -> {
  //     // Make the robot move
  //     swerveDrive.drive(new Translation2d(translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
  //                                         translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()),
  //                       angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
  //                       true,
  //                       false);
    
  //   });
  // }

Pose3d robotPose = new Pose3d();
// Pose3d poseB = new Pose3d();
StructPublisher<Pose3d> publisher = NetworkTableInstance.getDefault()
  .getStructTopic("MyPose", Pose3d.struct).publish();
StructArrayPublisher<Pose3d> arrayPublisher = NetworkTableInstance.getDefault()
  .getStructArrayTopic("MyPoseArray", Pose3d.struct).publish();

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
    SmartDashboard.putNumber("Encoder Back Left", swerveDrive.getModules()[2].getAbsoluteEncoder().getAbsolutePosition());
    SmartDashboard.putNumber("Encoder Front Right", swerveDrive.getModules()[1].getAbsoluteEncoder().getAbsolutePosition());
    SmartDashboard.putNumber("Encoder Front Left", swerveDrive.getModules()[0].getAbsoluteEncoder().getAbsolutePosition());
    SmartDashboard.putNumber("Encoder Back Right", swerveDrive.getModules()[3].getAbsoluteEncoder().getAbsolutePosition());
    SmartDashboard.putNumber("Odometry", swerveDrive.getOdometryHeading().getDegrees());
    SmartDashboard.putNumber("gyro test", swerveDrive.getGyroRotation3d().getAngle());
    m_field.setRobotPose(swerveDrive.getPose());
    }

}
