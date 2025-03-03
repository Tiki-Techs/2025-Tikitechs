// add second encoder logic later

package frc.robot.subsystems;

import org.apache.commons.math3.analysis.interpolation.LinearInterpolator;
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
    public TalonFX m_Leader = new TalonFX(10);
    public TalonFX m_Follower = new TalonFX(9);
    public Follower follower = new Follower(10, false);
    DigitalInput lowerLimitSwitch = new DigitalInput(3);
    DigitalInput upperLimitSwitch = new DigitalInput(2);
    public double canDown;
    public double canUp;
    public PIDController pid = new PIDController(0.008, 0, 0);
    public DutyCycleEncoder degEncoder = new DutyCycleEncoder(0);
    private final LinearInterpolator interpolator = new LinearInterpolator();
    private PolynomialSplineFunction m_elevInterpolator;

    public double encoderValue;
    public double setpoint;
    public boolean tryRumble;
    public boolean readyRumble;
    public double test1 = 60;
    public double test2 = 100;
    public static double down = 140;
    public static double down2;
    public static boolean there = false;
    public double up;

    public double tol = 3;
    public double tol2 = 3;
    public double speed;
    public double top = 100;
    public double bottom = 30;

    public Elevator(){
        m_Follower.setControl(follower);
        setpoint = getRotation();
        double[] elev = Controller.elevSafety2;
        double[] arm = Controller.armSafety2;
        m_elevInterpolator = interpolator.interpolate(arm, elev);
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
    
    // Transforms setpoint into motor turns.
    public double transform (double val) {
        return (val-12.5)/0.209653;
    }

        // Transforms setpoint into motor turns.
    public double antiTransform (double val) {
        return (val*0.209653)+12.5;
    }


    @Override
    public void periodic(){
        encoderValue = getRotation();
        // SmartDashboard.putNumber("safety elev", encoderValue);
        // If upper limit switch is not hit, canDown is 1. If lower is not hit, canUp is 1.
        // SmartDashboard.putNumber("elevator value", m_elevInterpolator.value(Math.abs(RobotContainer.m_arm.realEncoderValue)));
        if ((!lowerLimitSwitch.get())
        ){ // ZERO ENCODERS AND WHATNOT, NEED TO ACCOUNT FOR ENCODER OFFSET THINGY IN OTHER PARTS OF THE CODE
            canDown = 0;
            m_Leader.setPosition(0); // CONSIDER SLIGHTLY HIGHER THAN ZERO (THE VALUE AS SOON AS WE TOUCH THE SWITCH)
            setpoint = m_Leader.getPosition().getValueAsDouble();
        }
        else {
            canDown = 1;
        }
        if ((!upperLimitSwitch.get())){
            canUp = 0;
            m_Leader.setPosition(229); // CHECK VALUE........
            setpoint = m_Leader.getPosition().getValueAsDouble();
        }
        else {
            canUp = 1;
        }
        // SmartDashboard.putNumber("elevtest1", m_elevInterpolator.value(Math.abs(RobotContainer.m_arm.realEncoderValue)));
        // if (encoderValue < m_elevInterpolator.value(Math.abs(RobotContainer.m_arm.realEncoderValue))) {
        //     canDown = 0;
        // }
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
            if (leftY > 0){
                m_Leader.set(leftY*canUp);
            }
            else {
                m_Leader.set(leftY*canDown);
            }
            setpoint = encoderValue;
            tryRumble = false;
        }

        if (MathUtil.applyDeadband(encoderValue-setpoint, tol) == 0){
            there = true;
        }
        else {
            there = false;
        }

        // Rumble logic.
        if (tryRumble && there){
                readyRumble = true;
            }
        else {
            readyRumble = false;
        }

        SmartDashboard.putNumber("(Elevator) CanUp", canUp);
        SmartDashboard.putNumber("(Elevator) CanDown", canDown);
        SmartDashboard.putNumber("(Elevator) True encoder value", encoderValue);
        SmartDashboard.putNumber("(Elevator) True setpoint elev", setpoint);
    }
}
