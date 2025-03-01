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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ControllerConstants;
import frc.robot.RobotContainer;

public class Intake extends SubsystemBase{
    public static TalonFX m_Leader = new TalonFX(13);


    public PIDController pid = new PIDController(0.2, 0, 0);

    public double motorValue;
    public double speed = 0;
    // ADD THE LIMIT SWITCH FOR 4

    public Intake(){
        // m_leader.set(0.3);
    }

    public void setSpeed(double speed){
        m_Leader.set(speed);
        this.speed = speed;
    }

    @Override
    public void periodic(){
        motorValue = m_Leader.get();
        // SmartDashboard.putNumber("Intake Actual Speed", motorValue);
        // SmartDashboard.putNumber("Intake Set Speed", speed);
        if (MathUtil.applyDeadband(RobotContainer.m_mechController.getRightTriggerAxis(), 0.15) != 0){
            // m_Leader.set(Math.signum(RobotContainer.m_driverController.getLeftY()));
            m_Leader.set(RobotContainer.m_mechController.getRightTriggerAxis()*0.5);
            // SmartDashboard.putNumber("right", RobotContainer.m_mechController.getRightTriggerAxis());
        }
        else if (MathUtil.applyDeadband(RobotContainer.m_mechController.getLeftTriggerAxis(), 0.15) != 0){
            // m_Leader.set(Math.signum(RobotContainer.m_driverController.getLeftY()));
            m_Leader.set(-RobotContainer.m_mechController.getLeftTriggerAxis()*0.5);
            // SmartDashboard.putNumber("left", RobotContainer.m_mechController.getLeftTriggerAxis());
        }
        else {
            m_Leader.set(0);
        }
    // SmartDashboard.putNumber("motor", motorValue);
    }

       
}
