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
import frc.robot.Robot;
import frc.robot.RobotContainer;
// import frc.robot.subsystems.ArmTest;
// import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Elevator;

/**
 * 
 */
public class Controller extends SubsystemBase {
    public Elevator elevator;
    public Arm arm;
    public Manipulator manipulator;

    /**
     * Boolean for whether or not we should automatically intake on the manipulator. (Modified in this class' periodic and accessed in the manipulator's periodic)
     */
    public boolean algaeIntake = false;

    /**
     * Lets us change the arm's hard stop value to let us climb: when true, the arm's hard stop value is lowered, letting us move the arm fully downwards. (Modified in a command inside this class which is only called by driver input, and accessed in the arm's periodic)
     */
    public boolean climb = false;
    
    /**
     * When true, the arm and elevator will stow automatically if the manipulator's limit switch is hit. (Modified in this class' periodic and accessed in the manipulator's periodic)
     */
     public boolean stow = false;

    // public boolean readyOutput = false;
    // public static double[] elevSafety = {0, 5.58, 9.508, 14.9, 20.27, 25.06,
    // 31.6, 38.31, 39.55, 53.68, 60.44, 74.42, 83.35, 102.3, 109, 240};
    // public static double[] armSafety = {86, 91.6, 95.1, 100.4, 104, 107.8, 113.4,
    // 119.7, 125.4, 129.27, 129.3, 137.3, 139.8, 144, 180, 180.1};

    // public static double[] elevSafety2 = {0, 0.0000001, 5.58, 9.508, 14.9, 20.27,
    // 25.06, 31.6, 38.31, 39.55, 53.68, 60.44, 74.42, 83.35, 102.3, 105};
    // public static double[] armSafety2 = {0, 86, 91.6, 95.1, 100.4, 104, 107.8,
    // 113.4, 119.7, 125.4, 129.27, 129.3, 137.3, 139.8, 144, 180};

    public static double[] elevSafetyGI = { 0, 12.616284+Arm.armChange, 16.2061+Arm.armChange, 18.22276+Arm.armChange, 20.196964+Arm.armChange, 23.593444+Arm.armChange, 26.374312+Arm.armChange, 28.62448+Arm.armChange,
            29.413248+Arm.armChange, 29.5964+Arm.armChange, 30.23324+Arm.armChange, 31.20537228+Arm.armChange, 1000.9472+Arm.armChange };
    public static double[] armSafetyGI = { 91.0, 91.001, 99.3, 104.9, 108.9, 117.2, 122.6, 128.4, 131.4, 137.2, 147.1,
            180, 180.1 };

    public static double[] elevSafetyGI2 = { 0, 0.001, 12.001364+Elevator.changeHandoff, 14.59118+Elevator.changeHandoff, 16.60784+Elevator.changeHandoff, 18.682044+Elevator.changeHandoff, 22.503084+Elevator.changeHandoff, 24.759392+Elevator.changeHandoff,
            27.00956+Elevator.changeHandoff, 28.198328+Elevator.changeHandoff, 29.98148+Elevator.changeHandoff, 30.61832+Elevator.changeHandoff, 30.61832001+Elevator.changeHandoff};
    public static double[] armSafetyGI2 = { 0, 91.0, 91.001, 99.3, 104.9, 108.9, 117.2, 122.6, 128.4, 131.4, 137.2,
            147.1, 180 };

    public static double[] elevSafetyBumper = { 0, 1.783152, 3.651216, 6.028752, 7.620852, 9.913476, 13.501008,
            14.137848, 15.793632, 17.895204, 20.251512, 1000.9472 };
    public static double[] armSafetyBumper = { 91.2, 97.6, 102.8, 107.8, 112.0, 115.2, 119.1, 122.1, 127.4, 136.5, 180,
            180.1 };

    public static double[] elevSafetyBumper2 = { 0, 0.001, 1.783152, 3.651216, 6.028752, 7.620852, 9.913476, 13.501008,
            14.137848, 15.793632, 17.895204, 17.895204001 };
    public static double[] armSafetyBumper2 = { 0, 91.2, 97.6, 102.8, 107.8, 112.0, 115.2, 119.1, 122.1, 127.4, 136.5,
            180 };

    public Controller(Elevator elevator, Arm arm, Manipulator manipulator) {
        this.elevator = elevator;
        this.arm = arm;
        this.manipulator = manipulator;
    }

    /**
     * Used for testing. Essentially the same as our actual setpoint command, but only for the arm.
     */
    public Command setpointArm(double armPoint) {
        return new InstantCommand(
                () -> {
                    arm.setpoint(armPoint);
                }, this);

    }

    /**
     *  Used for testing. Essentially the same as our actual setpoint command, but only for the elevator.
     */
    public Command setpointElevator(double elevatorPoint) {
        return new InstantCommand(
                () -> {
                    elevator.setpoint(elevatorPoint);
                }, this);

    }

    /**
     * Takes an arm setpoint in degrees (180 to -180, with negative going towards the front of the robot) and an elevator setpoint (in inches)
     */
    public Command setpoint(double armPoint, double elevatorPoint) {
        return new InstantCommand(
                () -> {
                    arm.setpoint(armPoint);
                    elevator.setpoint(elevatorPoint);
                }, this);

    }

    /**
     * Sets predetermined arm and elevator setpoints. Uses a variable (based off robot heading) in SwerveSubsystem to determine which direction to move the arm (positive/negative).
     */
    public Command net() {
        return new RunCommand(
                () -> {
                    arm.setpoint(16.45 * RobotContainer.drivebase.netMult);
                    elevator.setpoint(50.5);
                }, this);

    }

    /**
     * Takes an arm setpoint in degrees (180 to -180, with negative going towards the front of the robot) and an elevator setpoint (in inches). Unlike the primary setpoint command, this method returns void, allowing us to call it from inside methods, classes, and periodics.
     */
    public void setpoint2(double armPoint, double elevatorPoint) {
        arm.setpoint(armPoint);
        elevator.setpoint(elevatorPoint);
    }

    /**
     * Sets predetermined arm and elevator setpoints, and sets Controller's climb variable to true. This variable modifies the arm's downward hard stop.
     */
    public Command climb() {
        return new InstantCommand(
                () -> {
                    climb = true;
                    arm.setpoint(-180);
                    elevator.setpoint(29.55);
                }, this);

    }

    /**
     * I don't really feel like documenting this right now. We don't even do handoff. The command is never called.
     */
    public Command handoff3() {
        return new RunCommand(
                () -> {
                    if (RobotContainer.m_groundintake.getHave()) {
                        if (!arm.testHandoff && !elevator.testHandoff) {
                            setpoint2(-180, elevator.elevHandoffPosition + 5.5);
                        } else {
                            setpoint2(-180, elevator.elevHandoffPosition);
                        }
                        Manipulator.handoff = true;
                        GroundIntake.handoff = true;
                    }
                }, this);
    }

    @Override
    public void periodic() {

        // IF OUR ARM AND ELEVATOR ARE AT THE GROUND PICKUP POSITION, WE SHOULD BE INTAKING, AND WHEN THE LIMIT SWITCH IS HIT, WE SHOULD STOW.
        if
        ((MathUtil.applyDeadband(elevator.elevatorPosition-14.1, 1.4) == 0) && (MathUtil.applyDeadband(arm.armPosition+120, 8) == 0)) {
            algaeIntake = true;
            stow = true;
        }

        // IF OUR ARM AND ELEVATOR ARE AT THE LOW OR HIGH REEF ALGAE PICKUP POSITION, WE SHOULD BE INTAKING, BUT SHOULD NOT STOW WHEN THE LIMIT SWITCH IS HIT (DANGEROUS, COULD HIT REEF)
        else if 
        ((MathUtil.applyDeadband(elevator.elevatorPosition-32.66, 1.4) == 0) && (MathUtil.applyDeadband(arm.armPosition+100, 8) == 0)
        ||
        (MathUtil.applyDeadband(elevator.elevatorPosition-49.7, 1.4) == 0) && (MathUtil.applyDeadband(arm.armPosition+100, 8) == 0))
         {
            algaeIntake = true;
            stow = false;
        }

        // IF NEITHER OF THE ABOVE ARE TRUE, WE SHOULD NOT AUTOMATICALLY INTAKE OR STOW WHEN THE LIMIT SWITCH IS HIT.
        else {
            algaeIntake = false;
            stow = false;
        }
    }
}
//