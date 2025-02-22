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
    SparkFlex m_leader = new SparkFlex(13, MotorType.kBrushless);
    SparkFlex m_Follower = new SparkFlex(14, MotorType.kBrushless);


    public PIDController pid = new PIDController(0.2, 0, 0);

    public double motorValue;
    public double speed = 0;

    public Intake(){
        // m_leader.set(0.3);
    }

    public void setSpeed(double speed){
        m_leader.set(speed);
        this.speed = speed;
    }

    @Override
    public void periodic(){
        motorValue = m_leader.get();
        SmartDashboard.putNumber("Intake Actual Speed", motorValue);
        SmartDashboard.putNumber("Intake Set Speed", speed);
        if (MathUtil.applyDeadband(RobotContainer.m_mechController.getRightTriggerAxis(), 0.15) != 0){
            // m_Leader.set(Math.signum(RobotContainer.m_driverController.getLeftY()));
            m_leader.set(RobotContainer.m_mechController.getRightTriggerAxis());
        }
        else {
            m_leader.set(0);
        }
    
    }

       
}
