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
import frc.robot.RobotContainer;

public class Manipulator extends SubsystemBase {
    public static TalonFX manipulatorPower = new TalonFX(13);
    public DigitalInput limitSwitch = new DigitalInput(4);
    public double motorValue;
    public static boolean have = false;
    public static boolean coral = true;
    public static boolean algae = false;
    public static boolean handoff = false;
    public boolean drop = false;
    // ADD THE LIMIT SWITCH FOR 4

    public Manipulator() {
        // m_leader.set(0.3);
    }

    public void setSpeed(double speed) {
        manipulatorPower.set(speed);
    }

    public Command spit () {
        return new InstantCommand(
            () -> setSpeed(0.6), this
        ).withTimeout(.5);
    }

    @Override
    public void periodic() {


        if (!limitSwitch.get()) {
            if (!have) {
                if ((MathUtil.applyDeadband(RobotContainer.m_arm.armPositionTransformed - RobotContainer.m_arm.down,
                        10) == 0)
                        && (MathUtil.applyDeadband(
                                RobotContainer.m_elevator.elevatorPosition - Elevator.elevHandoffLowerLimit,
                                13) == 0)) {
                    coral = true;
                    SmartDashboard.putString("holding", "coral");
                } else {
                    algae = true;
                    SmartDashboard.putString("holding", "algae");
                }
            }
            have = true;
            handoff = false;
            GroundIntake.handoff = false;
            RobotContainer.m_groundintake.have = false;
        } else {
            have = false;
            coral = false;
            algae = false;
            drop = false;

            SmartDashboard.putString("holding", "nothing");
        }
        SmartDashboard.putBoolean("have", have);
        motorValue = manipulatorPower.get();
        // SmartDashboard.putNumber("Intake Actual Speed", motorValue);
        // SmartDashboard.putNumber("Intake Set Speed", speed);

        // if (drop) {
        // if (RobotContainer.m_arm.testDrop && RobotContainer.m_elevator.testDrop) {
        // manipulatorPower.set(0.5);
        // }
        // }

        // else

        if (handoff) {
            manipulatorPower.set(-0.6);
        }

        else if (MathUtil.applyDeadband(RobotContainer.m_mechController.getRightTriggerAxis(), 0.15) != 0) {
            // m_Leader.set(Math.signum(RobotContainer.m_driverController.getLeftY()));
            manipulatorPower.set(RobotContainer.m_mechController.getRightTriggerAxis() * .6);

            // SmartDashboard.putNumber("right",
            // RobotContainer.m_mechController.getRightTriggerAxis());
        }

        else if (have && algae) {
            manipulatorPower.set(-0.07);
        }

        else if (have && coral) {
            manipulatorPower.set(0);
        }

        else if (MathUtil.applyDeadband(RobotContainer.m_mechController.getLeftTriggerAxis(), 0.15) != 0) {
            // m_Leader.set(Math.signum(RobotContainer.m_driverController.getLeftY()));
            manipulatorPower.set(-RobotContainer.m_mechController.getLeftTriggerAxis() * 0.5);
            // SmartDashboard.putNumber("left",
            // RobotContainer.m_mechController.getLeftTriggerAxis());
        }

        else {
            manipulatorPower.set(0);
        }
    }
}
