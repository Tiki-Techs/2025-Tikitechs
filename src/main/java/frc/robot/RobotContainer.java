// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Controller;
import frc.robot.subsystems.Controller;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.MotorRunnerElevManual;
import frc.robot.subsystems.MotorRunnerArmManual;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;
import frc.lib.util.KeyboardAndMouse;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ElevatorConstants;

import java.io.File;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

 
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  
  public static final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve"));
  public static final Vision m_vision = new Vision();
  private final SendableChooser<Command> autoChooser;
  

  // Only run these without jack and ruby
  // public final MotorRunnerArmManual test2 = new MotorRunnerArmManual();
  // public final MotorRunnerElevManual test1 = new MotorRunnerElevManual();


  // public final Intake m_intake = new Intake();


  // These have to be commented out without jack and ruby
  private static final Arm m_arm = new Arm();
  // public static final Elevator m_elevator = new Elevator();
  // private final ControllerTest m_controller = new ControllerTest(m_elevator, m_arm);


  // private final Controller m_controller = new Controller(m_elevator, m_arm, m_intake);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort); 
  public static final CommandXboxController m_mechController = new CommandXboxController(1);
  // private final CommandXboxController m_elevatorController = new CommandXboxController(OperatorConstants.ELEVATOR_GAMEPAD_PORT);

 /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    autoChooser = AutoBuilder.buildAutoChooser();








    // KeyboardAndMouse.getInstance().key("a").onTrue(new InstantCommand( () -> Intake.m_Leader.set(0.3)));
    // autoChooser.setDefaultOption("Mid Auto", AutoBuilder.buildAuto(middleAuto));










    
  }

  Command driveFieldOrientedDirectAngle = drivebase.driveCommandF(
        () -> MathUtil.applyDeadband(-m_driverController.getLeftY(), ControllerConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-m_driverController.getLeftX(), ControllerConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(-m_driverController.getRightX(), 0.15),
        // () -> 0,
        () -> MathUtil.applyDeadband(-m_driverController.getRightY(), 0.15));

    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommandF(
        () -> MathUtil.applyDeadband(-m_driverController.getLeftY(), ControllerConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-m_driverController.getLeftX(), ControllerConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(-m_driverController.getRightX(), ControllerConstants.RIGHT_X_DEADBAND));
        
    Command robotOrientedAngularVelocity = drivebase.driveCommandR(
      () -> MathUtil.applyDeadband(-m_driverController.getLeftY(), ControllerConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(-m_driverController.getLeftX(), ControllerConstants.LEFT_X_DEADBAND),
      () -> MathUtil.applyDeadband(-m_driverController.getRightX(), ControllerConstants.RIGHT_X_DEADBAND));
      
    Command robotOrientedDirectAngle = drivebase.driveCommandF(
        () -> MathUtil.applyDeadband(-m_driverController.getLeftY(), ControllerConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-m_driverController.getLeftX(), ControllerConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(-m_driverController.getRightX(), 0.15),
        // () -> 0,
        () -> MathUtil.applyDeadband(-m_driverController.getRightY(), 0.15));

    Command stopDrive = drivebase.stopDrive();
    Command zeroGyro = drivebase.zeroGyro();
    // Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
    //     () -> MathUtil.applyDeadband(-m_driverController.getLeftY(), ControllerConstants.LEFT_Y_DEADBAND),
    //     () -> 0,
    //     () -> MathUtil.applyDeadband(-m_driverController.getRightX(), ControllerConstants.RIGHT_X_DEADBAND));

    Command autoAlign = drivebase.AlignTest();
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
  //  */

  private void configureBindings() {
    // drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    // m_driverController.a().whileTrue(zeroGyro);
    // m_driverController.b().whileTrue(autoAlign);
    // m_mechController.rightBumper().whileTrue(m_controller.PIDStop());
    // //NEVER PUT BELOW 12.5. JUST DON'T TOUCH ANY OF THE NUMBERS. OR ANY OF THE LETTERS. OR THE COMPUTER? UNLESS YOU'RE ON DISABLING.
    // m_mechController.a().whileTrue(m_controller.setpoint(10, 18.5));
    // m_mechController.x().whileTrue(m_controller.setpoint(20, 26.5));
    // m_mechController.b().whileTrue(m_controller.setpoint(30, 20));
    // m_mechController.y().whileTrue(m_controller.setpoint(-20, 24.5));
    // m_mechController.leftBumper().whileTrue(m_controller.setpoint(-90, 31.5));
  }


  //   // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
  //   // cancelling on release.
  //   m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  // }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

   // providing auto path from pathplanner
  public Command getAutonomousCommand() {
 
    return new PathPlannerAuto("Test");
  }

  public void periodic(){
    // SmartDashboard.putData("Auto Chooser", autoChooser);

  }

}
