// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import com.pathplanner.lib.config.RobotConfig;
// import frc.robot.subsystems.Controller.State;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  //Elevator constants
  // public static final List<Entry<Double, State>> SHOOTER_MAP = Arrays.asList(
  //       Map.entry(0.0, new State(300, 20, 0.2)),
  //       Map.entry(1.0, new State(100, -20, 0.2)));
  public static class ElevatorConstants{
    public static final int ELEVATOR_MOTOR_1 = 0;
    public static final int ELEVATOR_MOTOR_2 = 0;
    public static final int LEVEL_0 = 0;  
  
  }
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int ELEVATOR_GAMEPAD_PORT = 1;
  }
  public static class ControllerConstants {
    public static final double LEFT_X_DEADBAND = 0.15;
    public static final double LEFT_Y_DEADBAND = 0.15;
    public static final double RIGHT_X_DEADBAND = 0.15;
  }
  public static class MotorRunnerConstants {
    public static final int talonid8 = 8;
    public static final int talonid7 = 7;
    public static final int talonid6 = 6;
    public static final int talonid5 = 5;
    public static final int talonid4 = 4;
    public static final int talonid3 = 3;
    public static final int talonid2 = 2;
    public static final int talonid = 1;
  }
  
public static RobotConfig robotConfig;
  static{
    try{
      robotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
  }
}

  
 
