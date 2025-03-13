// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoAlignTest;
import frc.robot.commands.Autos;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Controller;
import frc.robot.subsystems.Controller;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.GroundIntake;
import frc.robot.subsystems.Intake;
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


  public static final Intake m_intake = new Intake();

  public static final GroundIntake m_groundintake = new GroundIntake();

  // These have to be commented out without jack and ruby
  public static Arm m_arm = new Arm();
  public static final Elevator m_elevator = new Elevator();
  public static final Controller m_controller = new Controller(m_elevator, m_arm, m_intake);

  // public static PoseEstimator pe = PoseEstimator.getInstance();
  public static PoseEstimator poseEstimator = new PoseEstimator();
  public static final Vision m_vision = new Vision();
  // private static final Controller m_controller = new Controller(m_elevator, m_arm, m_intake);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort); 
  public static final CommandXboxController m_mechController = new CommandXboxController(1);
  // private final CommandXboxController m_elevatorController = new CommandXboxController(OperatorConstants.ELEVATOR_GAMEPAD_PORT);

 /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    // MAY CAUSE DELAY IN DEPLOYMENT
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData(autoChooser);

    NamedCommands.registerCommand("Spit", m_groundintake.autoSpit());

    // KeyboardAndMouse.getInstance().key("a").onTrue(new InstantCommand( () -> Intake.m_Leader.set(0.3)));
    // autoChooser.setDefaultOption("Mid Auto", AutoBuilder.buildAuto(middleAuto));


 
  }

    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommandF(
        () -> MathUtil.applyDeadband(-m_driverController.getLeftY(), ControllerConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-m_driverController.getLeftX(), ControllerConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(-m_driverController.getRightX(), ControllerConstants.RIGHT_X_DEADBAND));

    Command driveFieldOrientedAnglularVelocityScaledDown = drivebase.driveCommandF(
        () -> MathUtil.applyDeadband(-m_driverController.getLeftY(), ControllerConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-m_driverController.getLeftX(), ControllerConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(-m_driverController.getRightX(), ControllerConstants.RIGHT_X_DEADBAND));

    public Command scaledToggle() {
      scaledToggle = !scaledToggle;
      if (scaledToggle) {
        return driveFieldOrientedAnglularVelocityScaledDown;
      }
      else {
        return driveFieldOrientedAnglularVelocity;
      }
    }
        
    Command robotOrientedAngularVelocity = drivebase.driveCommandR(
      () -> MathUtil.applyDeadband(-m_driverController.getLeftY(), ControllerConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(-m_driverController.getLeftX(), ControllerConstants.LEFT_X_DEADBAND),
      () -> MathUtil.applyDeadband(-m_driverController.getRightX(), ControllerConstants.RIGHT_X_DEADBAND));
      
    Command stopDrive = drivebase.stopDrive();
    Command zeroGyro = drivebase.zeroGyro();
    // Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
    //     () -> MathUtil.applyDeadband(-m_driverController.getLeftY(), ControllerConstants.LEFT_Y_DEADBAND),
    //     () -> 0,
    //     () -> MathUtil.applyDeadband(-m_driverController.getRightX(), ControllerConstants.RIGHT_X_DEADBAND));


    Command testAutoAnd1 = drivebase.testAutoAnd1();

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

    m_driverController.b().whileFalse(driveFieldOrientedAnglularVelocity);
    m_driverController.b().whileTrue(robotOrientedAngularVelocity);

    // PUT GYRO BACK
    m_mechController.a().whileTrue(m_groundintake.haveSwitch());


    // m_mechController.b().whileTrue(m_groundintake.off());
    // m_mechController.x().whileTrue(m_controller.setpoint(-180, 139));
    m_mechController.y().whileTrue(m_controller.setpoint(-38, 47.6)); // l4
    m_mechController.b().whileTrue(m_controller.setpoint(-31, 19.4)); // l3
    m_mechController.povLeft().whileTrue(m_controller.setpoint(0, 0)); // l3
    // l2: -27 and 4.3
    m_mechController.povUp().whileTrue(m_controller.handoff());
    m_mechController.povDown().whileTrue(m_controller.handoffFalse());



    // m_mechController.povUp().onTrue(m_groundintake.l1CommandTrue());  // take out
    // m_mechController.povUp().onFalse(m_groundintake.l1CommandFalse());




    m_driverController.povLeft().whileTrue(new AutoAlignTest(drivebase, 1)); // take out
    m_driverController.povRight().whileTrue(new AutoAlignTest(drivebase, -1)); // take out
    // m_driverController.a().whileTrue(pt1);

    // // m_driverController.x().whileTrue(scaledToggle());  // take out
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
