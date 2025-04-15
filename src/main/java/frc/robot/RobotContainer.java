// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoAlignL1;
import frc.robot.commands.AutoAlignTest;
import frc.robot.commands.Autos;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Controller;
import frc.robot.subsystems.Controller;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.GroundIntake;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.MotorRunnerElevManual;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.MotorRunnerArmManual;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;
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
  public static boolean scaledToggle = false;

  // MAY CAUSE DELAY IN DEPLOYMENT
  private final SendableChooser<Command> autoChooser;
  

  // Only run these without jack and ruby
  // public final MotorRunnerArmManual test2 = new MotorRunnerArmManual();
  // public final MotorRunnerElevManual test1 = new MotorRunnerElevManual();


  public static final Manipulator m_manipulator = new Manipulator();

  public static final GroundIntake m_groundintake = new GroundIntake();

  // These have to be commented out without jack and ruby
  public static Arm m_arm = new Arm();
  public static final Elevator m_elevator = new Elevator();
  public static final Controller m_controller = new Controller(m_elevator, m_arm, m_manipulator);

  // public static PoseEstimator pe = PoseEstimator.getInstance();
  public static PoseEstimator poseEstimator = new PoseEstimator();
  public static final Vision m_vision = new Vision();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort); 
  public static final CommandXboxController m_mechController = new CommandXboxController(1);
  // private final CommandXboxController m_elevatorController = new CommandXboxController(OperatorConstants.ELEVATOR_GAMEPAD_PORT);

  private final String centerLeave = "Center Leave";
  private final String centerReef = "Center Reef";
  private final String centerRam = "Center Ram";
  // private final String centerL1 = "Source Side Auto";
  private final String rightLeave = "Right Leave";
  private final String rightReef = "Right Reef";
  private final String rightL1 = "Right L1";
  private final String leftLeave = "Left Leave";
  private final String leftReef = "Left Reef";
  private final String leftL1 = "Left L1";
  private final String l4test = "L4 Test";
 /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    // MAY CAUSE DELAY IN DEPLOYMENT
    
    // NamedCommands.registerCommand("Spit", new AutoAlignTest(drivebase, 0));
    // NamedCommands.registerCommand("Off", m_groundintake.autoSpitOff());   
    NamedCommands.registerCommand("L4 Setpoint", m_controller.setpoint(-25, 49.33));
    
    NamedCommands.registerCommand("Manipulator Output", m_manipulator.spit());
    
    NamedCommands.registerCommand("L4 Setpoint Drop", m_controller.setpoint(-120, 49.33));


    autoChooser = new SendableChooser<Command>();
    autoChooser.setDefaultOption("Center Leave", AutoBuilder.buildAuto(centerLeave));
    autoChooser.addOption("Center Reef", AutoBuilder.buildAuto(centerReef));
    autoChooser.addOption("Center Ram", AutoBuilder.buildAuto(centerRam));
    autoChooser.addOption("Right Leave", AutoBuilder.buildAuto(rightLeave));
    autoChooser.addOption("Right Reef", AutoBuilder.buildAuto(rightReef));
    autoChooser.addOption("Right L1", AutoBuilder.buildAuto(rightL1));
    autoChooser.addOption("Left Leave", AutoBuilder.buildAuto(leftLeave));
    autoChooser.addOption("Left L1", AutoBuilder.buildAuto(leftL1));
    autoChooser.addOption("Left Reef", AutoBuilder.buildAuto(leftReef));
    autoChooser.addOption("L4 Test", AutoBuilder.buildAuto(l4test));


    // autoChooser = AutoBuilder.buildAutoChooser();
    // autoChooser.setDefaultOption("l1 test", driveFieldOrientedAnglularVelocity);;
    SmartDashboard.putData(autoChooser);


    // KeyboardAndMouse.getInstance().key("a").onTrue(new InstantCommand( () -> Intake.m_Leader.set(0.3)));
    // autoChooser.setDefaultOption("Mid Auto", AutoBuilder.buildAuto(middleAuto));


 
  }

    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommandF(
        () -> MathUtil.applyDeadband(-m_driverController.getLeftY(), ControllerConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-m_driverController.getLeftX(), ControllerConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(-m_driverController.getRightX(), ControllerConstants.RIGHT_X_DEADBAND));
        
    Command robotOrientedAngularVelocity = drivebase.driveCommandR(
      () -> MathUtil.applyDeadband(-m_driverController.getLeftY(), ControllerConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(-m_driverController.getLeftX(), ControllerConstants.LEFT_X_DEADBAND),
      () -> MathUtil.applyDeadband(-m_driverController.getRightX(), ControllerConstants.RIGHT_X_DEADBAND));
      
    Command stopDrive = drivebase.stopDrive();
    Command zeroGyro = drivebase.zeroGyro();
    Command rotateAlign = drivebase.rotateAlign();
    Command rotateAlignL1 = drivebase.rotateAlignL1();
    // Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
    //     () -> MathUtil.applyDeadband(-m_driverController.getLeftY(), ControllerConstants.LEFT_Y_DEADBAND),
    //     () -> 0,
    //     () -> MathUtil.applyDeadband(-m_driverController.getRightX(), ControllerConstants.RIGHT_X_DEADBAND));


    // Command x = drivebase.x();
    Command pt1 = drivebase.pt1();
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
    // PUT GYRO BACK

    // m_driverController.b().onTrue(new AutoAlignTest());
    m_driverController.x().whileTrue(zeroGyro); // PUT THIS BACK
    m_driverController.a().whileTrue(stopDrive); // PUT THIS BACK
    m_driverController.b().whileTrue(new AutoAlignL1(drivebase));

    m_driverController.rightTrigger(0.3).whileFalse(driveFieldOrientedAnglularVelocity);
    m_driverController.rightTrigger(0.3).whileTrue(robotOrientedAngularVelocity);
    // m_driverController.leftTrigger(0.3).onTrue(x);

    m_driverController.leftBumper().onTrue(new AutoAlignTest(drivebase, 1)); // take out
    m_driverController.rightBumper().onTrue(new AutoAlignTest(drivebase, -1)); // take out
    
    m_driverController.povUp().whileTrue(rotateAlign); // take out
    
    // m_driverController.povDown().onTrue(new AutoAlignL1(drivebase)); // take out

    m_mechController.b().whileTrue(m_groundintake.haveFalse());
    m_mechController.y().whileTrue(m_controller.handoff3());
    m_mechController.x().whileTrue(m_groundintake.haveTrue());
    m_mechController.a().whileTrue(m_controller.setpoint(0, 0));
    // l2: -27 and 4.3
    m_mechController.povUp().whileTrue(m_controller.setpoint(-25, 9.155)); // L2
    m_mechController.povRight().whileTrue(m_controller.setpoint(-25, 25.38)); // L3
    m_mechController.povDown().whileTrue(m_controller.setpoint(-25, 49.33)); // L4
    m_mechController.povLeft().whileTrue(m_groundintake.l1CommandTrue());
    m_mechController.povLeft().whileFalse(m_groundintake.l1CommandFalse());
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
