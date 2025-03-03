package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
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
    // public Intake intake;
    public boolean isDriving = false;
    // public boolean readyOutput = false;
    public static double[] elevSafety = {0, 5.58, 9.508, 14.9, 20.27, 25.06, 31.6, 38.31, 39.55, 53.68, 60.44, 74.42, 83.35, 102.3, 109, 240}; 
    public static double[] armSafety =  {86, 91.6, 95.1, 100.4, 104, 107.8, 113.4, 119.7, 125.4, 129.27, 129.3, 137.3, 139.8, 144, 180, 180.1};
    
    public static double[] elevSafety2 = {0, 0.0000001, 5.58, 9.508, 14.9, 20.27, 25.06, 31.6, 38.31, 39.55, 53.68, 60.44, 74.42, 83.35, 102.3, 105}; 
    public static double[] armSafety2 =  {0, 86, 91.6, 95.1, 100.4, 104, 107.8, 113.4, 119.7, 125.4, 129.27, 129.3, 137.3, 139.8, 144, 180};
    
    // DigitalInput limitSwitch = new DigitalInput(200);  // check THE NEW INTAKE LIMIT SWITCH. COULD BE MADE IN INTAKE INSTEAD


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

    public Command setpointArm(double armPoint){
        return new InstantCommand(
                () -> {
                    arm.setpoint(armPoint);
                    rumble(true);
                }, this);

    }

    
    public Command setpointElevator(double elevatorPoint){
        return new InstantCommand(
                () -> {
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

    public Command pickup(){
        return new InstantCommand(
                () -> {
                    arm.setpoint(Arm.down);
                    // may just be able to do arm and elevator at same time
                    elevator.setpoint(Elevator.down2);
                    // intake in/mech control, decide later
                    // if (!limitSwitch.get()){ // don't use an if, do an until
                    //     setpoint(arm.up, arm.up);
                    //     have = true;
                    // }
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
    public void periodic (){ // add in stuff based on vision, too?
        // if (GroundIntake.have && !Intake.have) {
        //     setpoint(Arm.down, Elevator.down);
        // }
        
        if (arm.readyRumble && elevator.readyRumble){
            // RobotContainer.m_driverController.setRumble(RumbleType.kBothRumble, 1);
            RobotContainer.m_driverController.setRumble(RumbleType.kBothRumble, 1); // figure out rumble = false
            // SmartDashboard.putBoolean("Elevator/Arm", manual);
        }
        else {
            RobotContainer.m_driverController.setRumble(RumbleType.kBothRumble, 0); // figure out rumble = false
        }    
        // UNFINISHED CODE FOR GOING TO A SETPOINT WHEN DRIVING. MAY NOT USED
        // if ((MathUtil.applyDeadband(RobotContainer.m_driverController.getLeftX(), 0.15) != 0) || 
        // (MathUtil.applyDeadband(RobotContainer.m_driverController.getLeftY(), 0.15) != 0)){
        //     elevator.setpoint(elevator.neutral);
        //     arm.setpoint(arm.neutral);
        // }
    }
}
// 