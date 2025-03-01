// add second encoder logic later

package frc.robot.subsystems;

import org.dyn4j.geometry.MassType;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ControllerConstants;
import frc.robot.RobotContainer;

public class GroundIntake extends SubsystemBase{

    // hood neo, power vortex

    SparkFlex m_power = new SparkFlex(14, MotorType.kBrushless);
    SparkMax m_hood = new SparkMax(15, MotorType.kBrushless);

    public GroundIntake(){
        
    }

    public void setSpeed(double speed){
    }

    @Override
    public void periodic(){
        if (RobotContainer.m_mechController.povLeft().getAsBoolean()) {
            m_power.set(0.02);
        }
        else {
            m_power.set(0);
        }
        if (RobotContainer.m_mechController.povRight().getAsBoolean()) {

        }
        else {
            m_hood.set(0);
        }
    }

       
}
