// add second encoder logic later

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class Manipulator extends SubsystemBase {

    public static TalonFX manipulatorPower = new TalonFX(13);

    public DigitalInput limitSwitch = new DigitalInput(4);

    // BOOLEAN FOR WHETHER OR NOT THE MANIPULATOR IS HOLDING A PIECE (ONLY MODIFIED IN THIS CLASS' PERIODIC)
    
    /**
     * 
     */
    public static boolean have = false;

    // BOOLEAN FOR WHETHER OR NOT WE'RE HOLDING CORAL (MODIFIED IN PERIODIC)
    
    /**
     * 
     */
    public static boolean coral = true;

    // WHETHER OR NOT WE'RE HOLDING ALGAE (MODIFIED IN PERIODIC)
    
    /**
     * 
     */
    public static boolean algae = false;

    // WAS USED WHEN WE DID HANDOFF. HANDOFF IS NEVER CHANGED TO TRUE, SO PRETTY MUCH ALL OF THE CODE JUST IGNORES IT.
    /**
     * 
     */
    public static boolean handoff = false;

    // USED TO MAKE THE MANIPULATOR SPIT OUT IN AUTO. WE SET THE VARIABLE TO TRUE WHEN WE WANT IT TO SPIT, AND FALSE OTHERWISE
    /**
     * 
     */
    public boolean auto = false;


    public Manipulator() {
    }
    
    
    /**
     * 
     */
    public Command autoSpit() {
        return new InstantCommand(() -> {
            auto = true;
        }, this);
    }

    /**
     * 
     */
    public Command autoSpitOff() {
        return new InstantCommand(() -> {
            auto = false;
        }, this);
    }

    @Override
    public void periodic() {

        // IF THE LIMIT SWITCH IS PRESSED
        if (!limitSwitch.get()) {
            /* IF WE DON'T "HAVE", THE FOLLOWING CODE WILL CHANGE THE ALGAE/CORAL BOOLEANS. HAVE IS SET TO TRUE AFTER THIS
             * BLOCK OF CODE. THIS MEANS THAT WE WILL ONLY RUN IT THE FIRST TIME THE LIMIT SWITCH IS PRESSED.
            */
            if (!have) {
                // IF THE ARM AND ELEVATOR ARE IN A POSITION TO PICK UP CORAL (AND THE SWITCH WAS HIT), WE HAVE CORAL.
                if ((MathUtil.applyDeadband(RobotContainer.m_arm.armPositionTransformed - RobotContainer.m_arm.down,
                        10) == 0)
                        && (MathUtil.applyDeadband(
                                RobotContainer.m_elevator.elevatorPosition - Elevator.elevHandoffLowerLimit,
                                13) == 0)) {
                    coral = true;
                } 
                // OTHERWISE, WE HAVE ALGAE.
                else {
                    algae = true;
                }
                /* THIS IS INSIDE THE IF STATEMENT BECAUSE, OTHERWISE, THE VARIABLES WOULD CHANGE AS THE POSITION MOVED 
                 * OUTSIDE OR INSIDE OF THE "POSITION TO PICK UP CORAL". THIS IS MEANT TO TELL IS WHAT WE ARE HOLDING, NOT WHAT
                 * WE ARE IN POSITION TO PICK UP, SO IT SHOULD ONLY RECOGNIZE THE POSITION WE WERE IN WHEN WE PICKED "SOMETHING" UP.
                */
            }

            // IF STOW IS TRUE (AND THE SWITCH IS HIT) WE MAKE THE ARM AND ELEVATOR GO INTO STOW POSITIONS.
            if (RobotContainer.m_controller.stow) {
                RobotContainer.m_arm.setpoint(0);
                RobotContainer.m_elevator.setpoint(0);
            }
            // NOW WE SET HAVE TO TRUE.
            have = true;
            // handoff = false;
            // GroundIntake.handoff = false;
        } 
        else {
            // IF THE LIMIT SWITCH IS NOT BEING PRESSED, HAVE, CORAL, AND ALGAE ARE FALSE.
            have = false;
            coral = false;
            algae = false;
        }

        // THIS CODE NEVER RUNS SINCE HANDOFF IS NEVER SET TO TRUE.
        if (handoff) {
            // manipulatorPower.set(-0.6);
        }

        // THIS MAKES THE MANIPULATOR OUTPUT AT FULL POWER WHEN THE AUTO VARIABLE IS SET TO TRUE.
        else if (auto) {
            manipulatorPower.set(-1);
        }

        // IF THE RIGHT TRIGGER IS PRESSED BEYOND A CERTAIN POINT, THE MANIPULATOR WILL OUTPUT.
        else if (MathUtil.applyDeadband(RobotContainer.m_mechController.getRightTriggerAxis(), 0.15) != 0) {
            manipulatorPower.set(-RobotContainer.m_mechController.getRightTriggerAxis());
        }

        // IF WE HAVE AN ALGAE, THE MANIPULATOR CONSTANTLY INTAKES AT 4% TO PREVENT THE ALGAE FROM SLIPPING OUT.
        else if (have && algae) {
            manipulatorPower.set(0.04);
        }

        // IF WE HAVE A CORAL, THE MANIPULATOR DOES NOTHING, BECAUSE THE CORAL DOESN'T SLIP OUT.
        else if (have && coral) {
            manipulatorPower.set(0);
        }

        // IF ALGAEINTAKE IS TRUE (IF WE'RE AT A POSITION WHERE WE SHOULD BE CONSTANTLY INTAKING), WE INTAKE AT FULL POWER. NOTE HOW THE HAVE && ALGAE COMMAND IS ABOVE THIS, OVERRIDING IT. IF THAT IF STATEMENT IS TRUE, THIS ONE WILL NOT BE CHECKED. THIS WAY, WE WILL NOT CONTINUE INTAKING AT FULL SPEED AFTER GETTING A PIECE.
        else if (RobotContainer.m_controller.algaeIntake){
            manipulatorPower.set(1);
        }

        // IF THE LEFT TRIGGER IS PRESSED BEYOND A CERTAIN POINT, THE MANIPULATOR INTAKES. SIMILARLY TO THE ABOVE STATEMENT, NOTE HOW THIS STATEMENT IS OVERRIDEN TO PREVENT DAMAGE TO THE PIECE.
        else if (MathUtil.applyDeadband(RobotContainer.m_mechController.getLeftTriggerAxis(), 0.15) != 0) {
            manipulatorPower.set(RobotContainer.m_mechController.getLeftTriggerAxis());
        }

        // SETS THE POWER TO 0 OTHERWISE. THIS WAY, WE WON'T CONTINUE TO RUN WHATEVER WE LAST SET THE SPEED TO.
        else {
            manipulatorPower.set(0);
        }

        SmartDashboard.putNumber("Manipulator Current", manipulatorPower.getSupplyCurrent().getValueAsDouble());
    }
}
