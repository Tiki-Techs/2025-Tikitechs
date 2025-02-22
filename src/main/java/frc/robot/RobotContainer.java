// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Controller;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.MotorRunnerElevManual;
import frc.robot.subsystems.MotorRunnerArmManual;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;
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
  
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve"));
  // private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  public final Vision m_vision = new Vision();
  private final SendableChooser<Command> autoChooser;
  private final Arm m_arm = new Arm();
  private final Elevator m_elevator = new Elevator();
  private final Intake m_intake = new Intake();
  private final Controller m_controller = new Controller(m_elevator, m_arm, m_intake);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort); 
  public static final CommandXboxController m_mechController = new CommandXboxController(1);
  // private final CommandXboxController m_elevatorController = new CommandXboxController(OperatorConstants.ELEVATOR_GAMEPAD_PORT);

 /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    autoChooser = AutoBuilder.buildAutoChooser();
    // autoChooser.setDefaultOption("Mid Auto", AutoBuilder.buildAuto(middleAuto));
  }

  Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(m_driverController.getLeftY(), ControllerConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(m_driverController.getLeftX(), ControllerConstants.LEFT_X_DEADBAND),
        () -> m_driverController.getRightX(),
        () -> m_driverController.getRightY());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(-m_driverController.getLeftY(), ControllerConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-m_driverController.getLeftX(), ControllerConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(-m_driverController.getRightX(), ControllerConstants.RIGHT_X_DEADBAND));
    // Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
    //     () -> MathUtil.applyDeadband(-m_driverController.getLeftY(), ControllerConstants.LEFT_Y_DEADBAND),
    //     () -> 0,
    //     () -> MathUtil.applyDeadband(-m_driverController.getRightX(), ControllerConstants.RIGHT_X_DEADBAND));
    Command autoAlign = drivebase.autoAlign();
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
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    
    // m_driverController.a().whileTrue(m_controller.mY());
    // m_driverController.b().whileTrue(m_controller.mB());
    // m_driverController.x().whileTrue(m_controller.mZero());
    // m_driverController.leftStick().whileTrue(m_controller.mL());
  
    // m_driverController.x().whileTrue(drivebase.AlignTest());
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
