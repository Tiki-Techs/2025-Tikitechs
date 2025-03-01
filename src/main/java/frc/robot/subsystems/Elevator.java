// add second encoder logic later

package frc.robot.subsystems;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

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

// 12.5 c, 


public class Elevator extends SubsystemBase{
    public TalonFX m_Leader = new TalonFX(10); // DO NOT RUN WITHOUT CHECING IDS
    public TalonFX m_Follower = new TalonFX(9);
    public Follower follower = new Follower(10, false);
    DigitalInput lowerLimitSwitch = new DigitalInput(2);
    DigitalInput upperLimitSwitch = new DigitalInput(3);
    public double canDown;
    public double canUp;
    public PIDController pid = new PIDController(0.08, 0, 0);
    public DutyCycleEncoder degEncoder = new DutyCycleEncoder(0);
    private final SplineInterpolator interpolator = new SplineInterpolator();
    private PolynomialSplineFunction m_elevInterpolator;

    public double encoderValue;
    public double setpoint;
    public boolean tryRumble;
    public boolean readyRumble;
    public double test1 = 60;
    public double test2 = 100;

    public double tol = 3;
    public double tol2 = 3;
    public double speed;
    public double top = 100;
    public double bottom = 30;

    public Elevator(){
        m_Follower.setControl(follower);
        setpoint = getRotation();
        double[] elev = Controller.elevSafety;
        double[] arm = Controller.armSafety;
        m_elevInterpolator = interpolator.interpolate(arm, elev);
    }

    public void setEncoder(double value){
        m_Leader.set(value);
    }

    public double getRotation(){
        return m_Leader.getPosition().getValueAsDouble();
    }

    public void setpoint(double setpoint){
        this.setpoint = transform(setpoint);
    }

    public void tryRumble(boolean rumble){
        this.tryRumble = rumble;
    }
    
    // Transforms setpoint into motor turns.
    public double transform (double val) {
        return (val-12.5)/0.209653;
    }

    @Override
    public void periodic(){ // Needs implementation: All of manual control, including limit switch logic. Should change limit switch logic for both, use if, else, limit, set.
        encoderValue = getRotation();
        // SmartDashboard.putNumber("15", transform(15));
        // SmartDashboard.putNumber("31.5", transform(31.5));
        // If upper limit switch is not hit, canDown is 1. If lower is not hit, canUp is 1. Add logic to set encoders to x when limit is hit.
        // if ((!lowerLimitSwitch.get())){ 
        //     canDown = 0;
        // }
        // else {
        //     canDown = 1;
        // }
        // if ((!upperLimitSwitch.get())){
        //     canUp = 0;
        // }
        // else {
        //     canUp = 1;
        // }

        if ((encoderValue < 20)){ 
            canDown = 0;
        }
        else {
            canDown = 1;
        }
        if ((encoderValue > 210)){
            canUp = 0;
        }
        else {
            canUp = 1;
        }

        double leftY = -MathUtil.applyDeadband(RobotContainer.m_mechController.getLeftY(), 0.15);
        if (leftY==0) {
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
        // else if (Controller.manual) {
        else {
            // if (((MathUtil.applyDeadband(encoderValue-top, tol2) == 0) || (MathUtil.applyDeadband(encoderValue-bottom, tol2) == 0))){
            //     canDown *= 0.2;
            //     canUp *= 0.2;
            // }
            if (leftY > 0){
                m_Leader.set(leftY*canUp);
            }
            else {
                m_Leader.set(leftY*canDown);
            }
            setpoint = encoderValue;
            tryRumble = false;
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

        SmartDashboard.putNumber("(Elevator) CanUp", canUp);
        SmartDashboard.putNumber("(Elevator) CanDown", canDown);
        SmartDashboard.putNumber("(Elevator) True encoder value", encoderValue);
        SmartDashboard.putNumber("(Elevator) True setpoint elev", setpoint);
        // SmartDashboard.putNumber("rightY", leftY);
    }
}
