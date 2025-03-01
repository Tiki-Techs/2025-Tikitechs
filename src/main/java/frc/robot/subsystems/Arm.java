
package frc.robot.subsystems;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import org.opencv.core.Mat;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ControllerConstants;
import frc.robot.RobotContainer;

public class Arm extends SubsystemBase{
    // public TalonFX m_Leader = new TalonFX(12);
    // public TalonFX m_Follower = new TalonFX(11);
    // public Follower follower = new Follower(12, false);



    public TalonFX m_Leader = new TalonFX(9);
    public TalonFX m_Follower = new TalonFX(10);



    public PIDController pid = new PIDController(0.05, 0, 0);
    // public DutyCycleEncoder degEncoder = new DutyCycleEncoder(1, 360, 263.448);
    public DutyCycleEncoder degEncoder = new DutyCycleEncoder(1);
    private final SplineInterpolator interpolator = new SplineInterpolator();
    private PolynomialSplineFunction m_armInterpolator;

    public double realEncoderValue;
    public double desiredEncoderValue;
    public double drivePosition;
    public boolean tryRumble = false;
    public boolean readyRumble = false;
    public double canRight = 0;
    public double canLeft = 0;
    public double minElevator;
    public boolean hasAlgae = false;

    public double coralFeeder = 5;
    public double algaeGround = 1.5;
    public double l1;
    public double l2;
    public double l3 = 200;
    public double l4 = 414;
    public double algaeLow;
    public double algaeHigh;
    public boolean positiveUp;

    public double tol = 3;
    public double speed;

    public double try1 = -45;
    public double try2 = 45;
    public double testlim1 = 60;
    public double testlim2 = -60;
    // public double conv = .933399, 52.668457; 0.006835, 21.764160; 0.164826, -36.416016; 0.314060, -100.060547;

    public Arm(){
        // m_Follower.setControl(follower);
        desiredEncoderValue = transform(degEncoder.get());
        double[] elev = Controller.elevSafety;
        double[] arm = Controller.armSafety;
        m_armInterpolator = interpolator.interpolate(elev, arm);
        // desiredEncoderValue = 1;
    }

    public void setpoint(double setpoint){
        this.desiredEncoderValue = setpoint;
    }

    public void tryRumble(boolean rumble){
        this.tryRumble = rumble;
    }

    // Transforms throughbore value into something that works with PID
    public double transform(double encoder) {
        double ret;
        if (encoder > 180) {
            ret = 360-encoder;
        }
        else {
            ret = -encoder;
        }


        return ret;
    }

    @Override
    public void periodic(){ // Needs implementation: All of manual control, including limit switch logic. Should change limit switch logic for both, use if, else, limit, set.
        // Gets current position.
        double test = degEncoder.get();
        SmartDashboard.putNumber("througharm", test);
        realEncoderValue = transform(degEncoder.get());
        // Checks if able to move left and right (manual), needs to implement stuff with elevator map and whatnot
        // if (realEncoderValue){
            // canLeft = 0;
        // }
        // else {
            canLeft = 1;
        // }
        // if (realEncoderValue > -180){
            // canRight = 0;
        // }
        // else {
            canRight = 1;
        // }
        double rightX = -MathUtil.applyDeadband(RobotContainer.m_mechController.getRightX(), 0.15);
        // If the right stick is not being used, will move to desired position. Will go in the quickest safe direction.
        if (rightX==0) {
            speed = pid.calculate(realEncoderValue, desiredEncoderValue);
            // Sets speed.
            m_Leader.set(speed);
            // SmartDashboard.putString("Control", "Not Manual");
        }
        // If the manual control is set to arm, the arm will move in the direction of the right joystick as long as it is safe to do so.
        // else if (!Controller.manual) {
        else{
            tryRumble = false;
            if (rightX > 0){
                m_Leader.set(rightX*canLeft);
            }
            else {
                m_Leader.set(rightX*canRight);
            }
            // Sets the desired position to the current position to prevent the arm from returning to its previous position when right stick is let go.
            desiredEncoderValue = realEncoderValue;
            // SmartDashboard.putString("Control", "Manual");
        }

        // If position is within a certain range of desired position, will indicate.
        if (tryRumble){
            if (MathUtil.applyDeadband(realEncoderValue-desiredEncoderValue, tol) == 0){
                readyRumble = true;
            }
            else {
                readyRumble = false;
            }
        }
        else {
            readyRumble = false;
        }

        // SmartDashboard.putNumber("Arm motor speed", speed);
        // SmartDashboard.putNumber("RightX", rightX);
        // SmartDashboard.putNumber("(Arm) CanRight", canRight);
        // SmartDashboard.putNumber("(Arm) CanLeft", canLeft);
        SmartDashboard.putNumber("(Arm) True encoder value", realEncoderValue);
        SmartDashboard.putNumber("(Arm) True setpoint elev", desiredEncoderValue);
        double temp = m_armInterpolator.value(0);
        
        double temp2 = m_armInterpolator.value(2);
        
        double temp3 = m_armInterpolator.value(3);
        
        double temp4 = m_armInterpolator.value(4);

        SmartDashboard.putNumber("test", temp);
        SmartDashboard.putNumber("test2", temp2);
        SmartDashboard.putNumber("test3", temp3);
        SmartDashboard.putNumber("test4", temp4);
    }
}
