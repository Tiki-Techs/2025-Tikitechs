// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.reflect.Parameter;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ClimbSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimbDefaultCommand extends Command {
  private CommandXboxController m_gamepad;

  private SlewRateLimiter m_Limiter = new SlewRateLimiter(2);
  /** Creates a new ClimbDefaultCommand. */
  public ClimbDefaultCommand(ClimbSubsystem subsys, CommandXboxController gamepad) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_gamepad = gamepad;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Limiter.reset(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double speed = m_Limiter.calculate(m_gamepad.getLeftY());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
