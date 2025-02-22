// add second encoder logic later

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ControllerConstants;
import frc.robot.RobotContainer;

public class Intake extends SubsystemBase{
    public TalonFX m_Leader = new TalonFX(14);
    public TalonFX m_Follower = new TalonFX(13);
    public Follower follower = new Follower(14, true); // CHECK MASTER DIRECTION
    public PIDController pid = new PIDController(0.2, 0, 0);

    public double motorValue;
    public double speed = 0;

    public Intake(){
    }

    // public boolean getLimitSwitchTop(){
    //     return upperLimitSwitch.get();
    // }

    // public boolean getLimitSwitchBottom(){
    //     return lowerLimitSwitch.get();
    // }

    public void setSpeed(double speed){
        m_Leader.set(speed);
        this.speed = speed;
    }

    @Override
    public void periodic(){
        motorValue = m_Leader.get();
        SmartDashboard.putNumber("Intake Actual Speed", motorValue);
        SmartDashboard.putNumber("Intake Set Speed", speed);
    }
}
