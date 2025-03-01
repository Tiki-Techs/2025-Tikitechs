package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
// import frc.robot.subsystems.ArmTest;
// import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Elevator;

public class Controller extends SubsystemBase{
    public Elevator elevator;
    public Arm arm;
    // public Arm arm;
    // public Intake intake;
    // public Intake intake;
    public static boolean elevatorControl = true;
    public boolean isDriving = false;
    public static double[] elevSafety = {0, 3, 5, 9}; 
    public static double[] armSafety = {1, 3, 4, 5}; 

    // public Controller (Elevator elevator, Arm arm, Intake intake) {
    //     this.elevator = elevator;
    //     this.arm = arm;
    //     this.intake = intake;

    public Controller (Elevator elevator, Arm arm) {
        this.elevator = elevator;
        this.arm = arm;
    }

    public void rumble(boolean rumble){
        // arm.tryRumble(rumble);
        elevator.tryRumble(rumble);
    }

    public Command setpointArm(double armPoint, double elevatorPoint){
        return new InstantCommand(
                () -> {
                    arm.setpoint(armPoint);
                    // elevator.setpoint(elevatorPoint);
                    rumble(true);
                }, this);

    }

    
    public Command setpointElevator(double armPoint, double elevatorPoint){
        return new InstantCommand(
                () -> {
                    // arm.setpoint(armPoint);
                    elevator.setpoint(elevatorPoint);
                    rumble(true);
                }, this);

    }

    public Command setpoint(double armPoint, double elevatorPoint){
        return new InstantCommand(
                () -> {
                    arm.setpoint(armPoint);
                    elevator.setpoint(elevatorPoint);
                    rumble(true);
                }, this);

    }


    public Command PIDStop(){
        return new InstantCommand(
                () -> {
                    arm.setpoint(arm.realEncoderValue);
                    elevator.setpoint = elevator.encoderValue;
                    rumble(false);
                }, this);

    }
  
    @Override
    public void periodic (){ // add in stuff based on vision, too
        if (arm.readyRumble){
            // RobotContainer.m_driverController.setRumble(RumbleType.kBothRumble, 1);
            RobotContainer.m_driverController.setRumble(RumbleType.kBothRumble, 1); // figure out rumble = false
            // SmartDashboard.putBoolean("Elevator/Arm", manual);
        }
        else {
            RobotContainer.m_driverController.setRumble(RumbleType.kBothRumble, 0); // figure out rumble = false
        }    
        // if ((MathUtil.applyDeadband(RobotContainer.m_driverController.getLeftX(), 0.15) != 0) || 
        // (MathUtil.applyDeadband(RobotContainer.m_driverController.getLeftY(), 0.15) != 0)){
        //     elevator.setpoint(elevator.neutral);
        //     arm.setpoint(arm.neutral);
        // }
    }
}
