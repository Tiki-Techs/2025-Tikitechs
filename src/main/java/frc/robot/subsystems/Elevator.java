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
import frc.robot.Robot;
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
    public PIDController pid = new PIDController(0.07, 0, 0);
    public DutyCycleEncoder degEncoder = new DutyCycleEncoder(0);
    private final LinearInterpolator interpolator = new LinearInterpolator();
    private PolynomialSplineFunction m_elevInterpolatorGI;
    private PolynomialSplineFunction m_elevInterpolatorBumper;

    public double encoderValue;
    public double setpoint;
    public boolean tryRumble;
    public boolean readyRumble;
    public static double down = 29.7192;
    public static boolean there = false;
    public double up;
    public double groundIntakeLevel = 16.6754; // change later
    // public double elevarmground = 30.5634;
    public double elevarmground = 30.2;
    public static double gearRatio = 37.8;
    public static double gearCoeff = (1.756*Math.PI*2)/gearRatio;
    public boolean testHandoff = false;
    
        
        public double aSDHUAWDUJHAOSDJUHWODIntakeLevel = 34.60164; // change later
    
        public double tol = 0.19684;
        public double tol2 = 0.19684;
        public double speed;
    
        public Elevator(){
            m_Follower.setControl(follower);
            setpoint = antiTransform(getRotation());
            double[] elevGI = Controller.elevSafetyGI2;
            double[] armGI = Controller.armSafetyGI2;
            
            double[] elevBumper = Controller.elevSafetyBumper2;
            double[] armBumper = Controller.armSafetyBumper2;
            m_elevInterpolatorGI = interpolator.interpolate(armGI, elevGI);
            m_elevInterpolatorBumper = interpolator.interpolate(armBumper, elevBumper);
        }
    
        public double getRotation(){
            return m_Leader.getPosition().getValueAsDouble();
        }
    
        public void kill () {
            m_Leader.set(0);
        }
    
        public void setpoint(double setpoint){
            this.setpoint = setpoint;
        }
    
        public void tryRumble(boolean rumble){
            this.tryRumble = rumble;
        }
        
        // Transforms setpoint into motor turns.
        public double transform (double val) {
            return (val)/gearCoeff;
    }

        // Transforms motor turns into setpoint.
    public double antiTransform (double val) {
        return (val*gearCoeff);
    }

    // x = (1.756*pi*2)/gearratio


    @Override
    public void periodic(){
        encoderValue = antiTransform(getRotation());
        // SmartDashboard.putNumber("safety elev", encoderValue);
        // If upper limit switch is not hit, canDown is 1. If lower is not hit, canUp is 1.
        // SmartDashboard.putNumber("elevator value", m_elevInterpolator.value(Math.abs(RobotContainer.m_arm.realEncoderValue)));
        if ((!lowerLimitSwitch.get())
        ){ // ZERO ENCODERS AND WHATNOT, NEED TO ACCOUNT FOR ENCODER OFFSET THINGY IN OTHER PARTS OF THE CODE
            canDown = 0;
            // m_Leader.setPosition(0); // CONSIDER SLIGHTLY HIGHER THAN ZERO (THE VALUE AS SOON AS WE TOUCH THE SWITCH)
            // setpoint = m_Leader.getPosition().getValueAsDouble();
        }
        else {
            canDown = 1;
        }
        if ((!upperLimitSwitch.get())){
            canUp = 0;
            // m_Leader.setPosition(transform(48.61212)); // CHECK VALUE........
            // setpoint = antiTransform(m_Leader.getPosition().getValueAsDouble());
            // encoderValue = antiTransform(getRotation());
        }
        else {
            canUp = 1;
        }
        if (((setpoint < groundIntakeLevel) || (encoderValue < groundIntakeLevel)) && (((RobotContainer.m_groundintake.desiredEncoderValue > 182)) || ((RobotContainer.m_groundintake.realEncoderValue > 182)))) {
            canDown = 0;
        }
        // SmartDashboard.putNumber("elevtest1", m_elevInterpolator.value(Math.abs(RobotContainer.m_arm.realEncoderValue)));

        if (((!RobotContainer.m_groundintake.clearArmOutside) && (encoderValue < m_elevInterpolatorBumper.value(Math.abs(RobotContainer.m_arm.realEncoderValue)))) || ((!RobotContainer.m_groundintake.clearArmOutside) && (encoderValue < m_elevInterpolatorGI.value(Math.abs(RobotContainer.m_arm.realEncoderValue))))) {
            canDown = 0;
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
            if (leftY > 0){
                m_Leader.set(leftY*canUp);
            }
            else {
                m_Leader.set(leftY*canDown);
            }
            setpoint = encoderValue;
            tryRumble = false;
        }

        if ((MathUtil.applyDeadband(encoderValue-elevarmground, tol) == 0) && (MathUtil.applyDeadband(setpoint-elevarmground, tol) == 0)){
            there = true;
        }
        else {
            there = false;
        }

        if (MathUtil.applyDeadband(encoderValue-elevarmground+5.5, tol) == 0 && (MathUtil.applyDeadband(setpoint-elevarmground+5.5, tol) == 0)) {
            testHandoff = true;
        }
        else {
            testHandoff = false;
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
        SmartDashboard.putNumber("(Elevator) True inch encoder value", encoderValue);
        SmartDashboard.putNumber("(Elevator) True setpoint elev", setpoint);
    }
}
