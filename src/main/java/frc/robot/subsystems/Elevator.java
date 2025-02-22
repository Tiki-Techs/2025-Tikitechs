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

public class Elevator extends SubsystemBase{
    public TalonFX m_Leader = new TalonFX(10); // DO NOT RUN WITHOUT CHECING IDS
    public TalonFX m_Follower = new TalonFX(9);
    public Follower follower = new Follower(10, false);
    DigitalInput lowerLimitSwitch = new DigitalInput(0);
    DigitalInput upperLimitSwitch = new DigitalInput(1);
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

    public Elevator(){
        // m_Follower.setControl(follower);
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
        // If upper limit switch is not hit, canDown is 1. If lower is not hit, canUp is 1. Add logic to set encoders to x when limit is hit.
        if (upperLimitSwitch.get()){
            canDown = 0;
        }
        else {
            canDown = 1;
        }
        if (lowerLimitSwitch.get()){
            canUp = 0;
        }
        else {
            canUp = 1;
        }

        double rightY = MathUtil.applyDeadband(RobotContainer.m_driverController.getRightY(), 0.15);
        if (rightY==0) {
            speed = pid.calculate(encoderValue, setpoint);
            // Prevents movement if limit switch is hit.
            if (speed>0){
                speed *= canUp;
            }
            else {
                speed*=canDown;
            }
            // Sets speed.
            m_Leader.set(speed);
        }
        else if (Controller.manual) {
            m_Leader.set(rightY);
        }

        // Rumble logic.
        if (tryRumble){
            if (MathUtil.applyDeadband(encoderValue-setpoint, tol) == 0){
                readyRumble = true;
            }
            else {
                readyRumble = false;
            }
        }

        SmartDashboard.putNumber("Elevator motor speed", speed);
        SmartDashboard.putNumber("CanUp", canUp);
        SmartDashboard.putNumber("CanDown", canDown);
        SmartDashboard.putNumber("Elevator True encoder value", encoderValue);
        SmartDashboard.putNumber("Elevator True setpoint elev", setpoint);
    }
}
