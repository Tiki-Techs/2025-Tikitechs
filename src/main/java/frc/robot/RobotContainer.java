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
import frc.robot.subsystems.GroundIntake;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ElevatorConstants;

import java.io.File;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
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

  // MAY CAUSE DELAY IN DEPLOYMENT
  private final SendableChooser<Command> autoChooser;
  
  public static final Manipulator m_manipulator = new Manipulator();
  public static final GroundIntake m_groundintake = new GroundIntake();
  public static Arm m_arm = new Arm();
  public static final Elevator m_elevator = new Elevator();
  public static final Controller m_controller = new Controller(m_elevator, m_arm, m_manipulator);

  // public static PoseEstimator pe = PoseEstimator.getInstance();
  // public static PoseEstimator poseEstimator = new PoseEstimator();
  // public static final Vision m_vision = new Vision();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort); 
  public static final CommandXboxController m_mechController = new CommandXboxController(1);
  // private final CommandXboxController m_elevatorController = new CommandXboxController(OperatorConstants.ELEVATOR_GAMEPAD_PORT);

  // VARIABLES HOLDING AUTO NAMES (AS NAMED IN PATHPLANNER)
  private final String centerLeave = "Center Leave";
  private final String rightBack = "Right Back";
  private final String L1NetCenter = "L1 Net Center";
  private final String centerL1Score = "Center L1 Score";
  private final String L1NetCenter2 = "L1 Net Center2";
  private final String rightLeave = "Right Leave";
  private final String rightL1 = "Right L1";
  private final String leftLeave = "Left Leave";
  private final String leftL1 = "Left L1";

 /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    // LETS YOU REGISTER "NAMED COMMANDS" FOR USE IN PATHPLANNER. YOUR SECOND PARAMETER MUST RETURN A COMMAND
    NamedCommands.registerCommand("A1", m_groundintake.autoSpit());
    NamedCommands.registerCommand("A2", m_groundintake.autoSpitOff());
    NamedCommands.registerCommand("Reef Low", m_controller.setpoint(-100, 32.66));
    NamedCommands.registerCommand("Reef Low Stow", m_controller.setpoint(0, 32.66));
    NamedCommands.registerCommand("Net", m_controller.setpoint(-20.5, 50.5));
    NamedCommands.registerCommand("A3", m_manipulator.autoSpit());
    NamedCommands.registerCommand("A4", m_manipulator.autoSpitOff());
    NamedCommands.registerCommand("Stow", m_controller.setpoint(0, 0));
    NamedCommands.registerCommand("Zero Gyro", drivebase.zeroGyro());    


    autoChooser = new SendableChooser<Command>();
    // ADDS DIFFERENT AUTOS TO THE AUTOCHOOSER.
    autoChooser.setDefaultOption("Center Leave", AutoBuilder.buildAuto(centerLeave));
    autoChooser.addOption("Right Back", AutoBuilder.buildAuto(rightBack)); // forward, forward
    autoChooser.addOption("Right Leave", AutoBuilder.buildAuto(rightLeave)); // forward, forward
    autoChooser.addOption("Right L1", AutoBuilder.buildAuto(rightL1)); // l1 to back, foward (won't work)
    autoChooser.addOption("Left Leave", AutoBuilder.buildAuto(leftLeave)); // forward, forward
    autoChooser.addOption("Left L1", AutoBuilder.buildAuto(leftL1)); // l1 to reef, forward (won't work)
    autoChooser.addOption("Center L1 Score", AutoBuilder.buildAuto(centerL1Score)); // l1 to reef, forward (won't work)
    autoChooser.addOption("L1 Net Center2", AutoBuilder.buildAuto(L1NetCenter2)); // l1 to reef, forward(won't work)
    autoChooser.addOption("L1 Net Center", AutoBuilder.buildAuto(L1NetCenter)); // l1 to reef, forward(won't work)

    /* THIS PUTS A LIST OF AUTOS TO CHOOSE FROM ON SMARTDASHBOARD. SELECT YOUR AUTO BY CLICKING ON IT (YOU CANNOT CLICK AN AUTO
     * WHILE IN EDITING MODE, CTRL+E TO CHANGE) */
    SmartDashboard.putData(autoChooser);
  }

  // CREATES A NEW COMMAND VARIABLE IN THIS CLASS.............................. CHANGE NAME
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommandF(
        () -> MathUtil.applyDeadband(-m_driverController.getLeftY(), ControllerConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-m_driverController.getLeftX(), ControllerConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(-m_driverController.getRightX(), ControllerConstants.RIGHT_X_DEADBAND));
        
    Command robotOrientedAngularVelocity = drivebase.driveCommandFSlow(
      () -> MathUtil.applyDeadband(-m_driverController.getLeftY(), ControllerConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(-m_driverController.getLeftX(), ControllerConstants.LEFT_X_DEADBAND),
      () -> MathUtil.applyDeadband(-m_driverController.getRightX(), ControllerConstants.RIGHT_X_DEADBAND));
    
    /* THESE ARE ALSO COMMAND VARIABLES. IF WE WANTED, WE COULD JUST DIRECTLY CALL whileTrue(drivebase.stopDrive())
    */
    Command stopDrive = drivebase.stopDrive();
    Command zeroGyro = drivebase.zeroGyro();

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

    // DEFAULT DRIVE COMMAND
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    // ZEROES THE GYRO (THE WAY THE ROBOT IS FACING BECOMES THE NEW "FRONT")
    m_driverController.x().whileTrue(zeroGyro);

    /* ROBOT IS UNABLE TO DRIVE WHILE THIS IS HELD, CANNOT MOVE (WILL INTERRUPT JOYSTICKS, BUT PRIMARILY USED TO INTERRUPT
     * PATHFINDING COMMANDS LIKE AUTO ALIGN. OUR AUTO ALIGN MADE THE ROBOT NAVIGATE TO A PATH WHEN A BUTTON WAS PRESSED ONCE,
     * AND IT WOULDN'T STOP FOLLOWING THE PATH WHEN JOYSTICKS WERE MOVED. SO THE DRIVER COULD PRESS THIS TO STOP AUTO ALIGNING.
    ) */
    m_driverController.a().whileTrue(stopDrive);

    // WHILE THE RIGHT TRIGGER IS HELD, USES A DIFFERENT DRIVE COMMAND (THIS IS JUST A SLOWED DOWN DRIVE COMMAND)
    m_driverController.rightTrigger(0.3).whileFalse(driveFieldOrientedAnglularVelocity);
    m_driverController.rightTrigger(0.3).whileTrue(robotOrientedAngularVelocity);

    // SETS THE CLIMB VARIABLE TO TRUE
    m_driverController.y().whileTrue(m_controller.climb());


    /*  LIKE A REGULAR SETPOINT COMMAND, BUT USES A CERTAIN VARIABLE (BASED OFF THE ROBOT'S HEADING) TO DECIDE WHICH DIRECTION TO
     * PUT THE ARM
    */
    m_mechController.povUp().whileTrue(m_controller.net());

    // BASIC SETPOINT COMMANDS
    // GROUND PICKUP
    m_mechController.povDown().whileTrue(m_controller.setpoint(-120, 14.1)); 
    // reef low algae
    m_mechController.povRight().whileTrue(m_controller.setpoint(-100, 32.66));
    // reef high algae
    m_mechController.povLeft().whileTrue(m_controller.setpoint(-100, 49.7));
    // Processor
    m_mechController.y().whileTrue(m_controller.setpoint(-88.39, 5.52));
    // GROUND INTAKE DOWN
    m_mechController.b().whileTrue(m_groundintake.haveFalse());
    // GROUND INTAKE UP
    m_mechController.x().whileTrue(m_groundintake.haveTrue());
    // STOW
    m_mechController.a().whileTrue(m_controller.setpoint(0, 0));
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
 
    return autoChooser.getSelected();
  }

}
