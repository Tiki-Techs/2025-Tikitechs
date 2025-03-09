package frc.robot.subsystems;

import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
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
    public Intake intake;
    public boolean isDriving = false;
    public final Timer m_Timer = new Timer();
    public static double time;

    // public boolean readyOutput = false;
    // public static double[] elevSafety = {0, 5.58, 9.508, 14.9, 20.27, 25.06, 31.6, 38.31, 39.55, 53.68, 60.44, 74.42, 83.35, 102.3, 109, 240}; 
    // public static double[] armSafety =  {86, 91.6, 95.1, 100.4, 104, 107.8, 113.4, 119.7, 125.4, 129.27, 129.3, 137.3, 139.8, 144, 180, 180.1};
    
    // public static double[] elevSafety2 = {0, 0.0000001, 5.58, 9.508, 14.9, 20.27, 25.06, 31.6, 38.31, 39.55, 53.68, 60.44, 74.42, 83.35, 102.3, 105}; 
    // public static double[] armSafety2 =  {0, 86, 91.6, 95.1, 100.4, 104, 107.8, 113.4, 119.7, 125.4, 129.27, 129.3, 137.3, 139.8, 144, 180};
    
    public static double[] elevSafetyGI = {0, 9.616284, 12.2061, 14.22276, 16.196964, 19.593444, 22.374312, 24.62448, 25.813248, 27.5964, 28.23324, 31.20537228, 1000.9472}; 
    public static double[] armSafetyGI =  {91.0, 91.001, 99.3, 104.9, 108.9, 117.2, 122.6, 128.4, 131.4, 137.2, 147.1, 180, 180.1};

    public static double[] elevSafetyGI2 = {0, 0.001, 10.951364, 13.54118, 15.55784, 17.532044, 21.353084, 23.709392, 25.95956, 27.148328, 28.93148, 29.56832, 29.56832001};
    public static double[] armSafetyGI2 =  {0, 91.0, 91.001, 99.3, 104.9, 108.9, 117.2, 122.6, 128.4, 131.4, 137.2, 147.1, 180};

    public static double[] elevSafetyBumper = {0, 1.783152, 3.651216, 6.028752, 7.620852, 9.913476, 13.501008, 14.137848, 15.793632, 17.895204, 20.251512, 1000.9472}; 
    public static double[] armSafetyBumper =  {91.2, 97.6, 102.8, 107.8, 112.0, 115.2, 119.1, 122.1, 127.4, 136.5, 180, 180.1};

    // public static double[] elevSafetyBumper2 = {0, 8.4, 17.2, 28.4, 35.9, 46.7, 63.6, 66.6, 74.4, 84.3, 95.4, 95.401};
    // public static double[] armSafetyBumper2 =  {91.2, 97.6, 102.8, 107.8, 112.0, 115.2, 119.1, 122.1, 127.4, 136.5, , 180};
    

    public static double[] elevSafetyBumper2 = {0, 0.001, 1.783152, 3.651216, 6.028752, 7.620852, 9.913476, 13.501008, 14.137848, 15.793632, 17.895204, 17.895204001};
    public static double[] armSafetyBumper2 =  {0, 91.2, 97.6, 102.8, 107.8, 112.0, 115.2, 119.1, 122.1, 127.4, 136.5, 180};
    


    // DigitalInput limitSwitch = new DigitalInput(200);  // check THE NEW INTAKE LIMIT SWITCH. COULD BE MADE IN INTAKE INSTEAD


    public Controller (Elevator elevator, Arm arm, Intake intake) {
        this.elevator = elevator;
        this.arm = arm;
        this.intake = intake;
    }
    // public Controller (Elevator elevator, Arm arm) {
    //     this.elevator = elevator;
    //     this.arm = arm;
    // }

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

    public Command setpointArmDown(){
        return new InstantCommand(
                () -> {
                    arm.setpoint(arm.down);
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

    public Command handoff(){
        return new InstantCommand(
                () -> {
                    m_Timer.start();
                    Intake.handoff = true;
                    GroundIntake.handoff = true;
                }, this);

    }

    // public Command pickup(){
    //     return new InstantCommand(
    //             () -> {
    //                 arm.setpoint(Arm.down);
    //                 // may just be able to do arm and elevator at same time
    //                 elevator.setpoint(Elevator.down2);
    //                 // intake in/mech control, decide later
    //                 // if (!limitSwitch.get()){ // don't use an if, do an until
    //                 //     setpoint(arm.up, arm.up);
    //                 //     have = true;
    //                 // }
    //                 rumble(true);
    //             }, this);

    // }


    public Command PIDStop(){
        return new InstantCommand(
                () -> {
                    arm.kill();
                    elevator.kill();
                    rumble(false);
                }, this);

    }

    public void timerStop(){
        m_Timer.reset();
    }
  
    @Override
    public void periodic (){ // add in stuff based on vision, too?
        if (GroundIntake.have && !Intake.have) {
            // setpoint(Arm.down, Elevator.down);
            SmartDashboard.putString("Controller Action", "Pick Up Coral");
        }
        else {
            SmartDashboard.putString("Controller Action", "Nothing");
        }
        
        time = m_Timer.get();

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