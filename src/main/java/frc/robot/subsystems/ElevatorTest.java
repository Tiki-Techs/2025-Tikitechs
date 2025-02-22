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

public class ElevatorTest extends SubsystemBase{
    public TalonFX m_Leader = new TalonFX(12);
    public TalonFX m_Follower = new TalonFX(11);
    public Follower follower = new Follower(12, false);
    // DigitalInput lowerLimitSwitch = new DigitalInput(0);
    // DigitalInput upperLimitSwitch = new DigitalInput(1);
    public double canDown;
    public double canUp;
    public PIDController pid = new PIDController(0.2, 0, 0);

    public double encoderValue;
    public double setpoint;
    public double drive;
    public double neutral;
    public boolean tryRumble;
    public boolean readyRumble;
    public double align;
    public double coralFeeder = 5;
    public double algaeGround = 1.5;
    public double l1;
    public double l2;
    public double l3 = 200;
    public double l4 = 414;
    public double algaeLow;
    public double algaeHigh;
    public boolean manual;
    public double tol = 3;
    public double speed;

    public ElevatorTest(){
        // m_Follower.setControl(follower);
        setpoint = getRotation();
    }

    public void setEncoder(double value){
        m_Leader.set(value);
    }

    public double getRotation(){
        return m_Leader.getPosition().getValueAsDouble();
    }

    // public boolean getLimitSwitchTop(){
    //     return upperLimitSwitch.get();
    // }

    // public boolean getLimitSwitchBottom(){
    //     return lowerLimitSwitch.get();
    // }

    public void setpoint(double setpoint){
        this.setpoint = setpoint;
    }

    public void tryRumble(boolean rumble){
        this.tryRumble = rumble;
    }

    public void manual (double speed){
        m_Leader.set(speed);
        setpoint = encoderValue;
    }

    public void setManual(boolean manual){
        this.manual = manual;
    }

    @Override
    public void periodic(){
        encoderValue = getRotation();
        double rightY = MathUtil.applyDeadband(RobotContainer.m_driverController.getRightY(), 0.15);
        if (rightY==0) {
            speed = pid.calculate(encoderValue, setpoint);
            m_Leader.set(pid.calculate(encoderValue, setpoint));
        }
        if (tryRumble){
            if (MathUtil.applyDeadband(encoderValue-setpoint, tol) == 0){
                readyRumble = true;
            }
            else {
                readyRumble = false;
            }
        }
        SmartDashboard.putBoolean("rumble true", readyRumble);
        SmartDashboard.putBoolean("rumble try", tryRumble);
        SmartDashboard.putNumber("motor speed", speed);
        SmartDashboard.putNumber("Calc", MathUtil.applyDeadband(encoderValue-setpoint, tol));
        // if (getLimitSwitchBottom()){
        //     canDown = 0;
        // }
        // else {
        //     canDown = 1;
        // }
        // if (getLimitSwitchTop()){
        //     canUp = 0;
        // }
        // else {
        //     canUp = 1;
        // }

        SmartDashboard.putNumber("True encoder value", encoderValue);
        SmartDashboard.putNumber("True setpoint elev", setpoint);

        // SmartDashboard.putBoolean("top limit", getLimitSwitchTop());
        // SmartDashboard.putBoolean("bottom limit", getLimitSwitchBottom());
    }
}
