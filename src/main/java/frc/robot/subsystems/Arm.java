
package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ControllerConstants;
import frc.robot.RobotContainer;

public class Arm extends SubsystemBase{
    public TalonFX m_Leader = new TalonFX(12);
    public TalonFX m_Follower = new TalonFX(11);
    public Follower follower = new Follower(12, false);
    DigitalInput rightLimitSwitch = new DigitalInput(2);
    DigitalInput leftLimitSwitch = new DigitalInput(3);
    public double canLeft = 0;
    public double canRight = 0;
    public PIDController pid = new PIDController(0.2, 0, 0);
    public DutyCycleEncoder degEncoder = new DutyCycleEncoder(1);

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
    public double try1 = 30;
    public double try2 = -20;
    public double testlim1 = 60;
    public double testlim2 = -60;
    public double deg;

    public Arm(){
        m_Follower.setControl(follower);
        setpoint = getRotation();
    }

    public void setEncoder(double value){
        m_Leader.set(value);
    }

    public double getRotation(){
        return m_Leader.getPosition().getValueAsDouble();
    }

    public void setpoint(double setpoint){
        this.setpoint = setpoint;
    }

    public void tryRumble(boolean rumble){
        this.tryRumble = rumble;
    }

    @Override
    public void periodic(){ // Needs implementation: All of manual control, including limit switch logic. Should change limit switch logic for both, use if, else, limit, set.
        encoderValue = getRotation();
        deg = degEncoder.get();
        SmartDashboard.putNumber("throughbore arm", deg);
        // If upper limit switch is not hit, canDown is 1. If lower is not hit, canUp is 1. Add logic to set encoders to x when limit is hit.
        if (!leftLimitSwitch.get() || encoderValue < testlim2){
            canLeft = 0;
        }
        else {
            canLeft = 1;
        }
        if (!rightLimitSwitch.get() || encoderValue > testlim1){
            canRight = 0;
        }
        else {
            canRight = 1;
        }

        double rightX = -MathUtil.applyDeadband(RobotContainer.m_mechController.getRightX(), 0.37);
        if (rightX==0) {
            speed = pid.calculate(encoderValue, setpoint);
            // Prevents movement if limit switch is hit.
            if (speed<0){
                speed *= canRight;
            }
            else {
                speed*=canLeft;
            }
            // Sets speed.
            m_Leader.set(speed);
        }
        // else if (!Controller.manual) {
        else{
            if (rightX < 0){
                m_Leader.set(rightX*canLeft);
            }
            else {
                m_Leader.set(rightX*canRight);
            }
            setpoint = encoderValue;
        }
        // else {
        //     setpoint = encoderValue;
        // }
        
        // Rumble logic.
        if (tryRumble){
            if (MathUtil.applyDeadband(encoderValue-setpoint, tol) == 0){
                readyRumble = true;
            }
            else {
                readyRumble = false;
            }
        }

        SmartDashboard.putNumber("Arm motor speed", speed);
        SmartDashboard.putNumber("CanRight", canRight);
        SmartDashboard.putNumber("CanLeft", canLeft);
        SmartDashboard.putBoolean("left limit", leftLimitSwitch.get());
        
        SmartDashboard.putBoolean("right limit", rightLimitSwitch.get());
        SmartDashboard.putNumber("Arm True encoder value", encoderValue);
        SmartDashboard.putNumber("Arm True setpoint elev", setpoint);
    }
}
