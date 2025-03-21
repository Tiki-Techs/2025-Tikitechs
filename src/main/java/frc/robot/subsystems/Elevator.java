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
    public TalonFX elevatorLeader = new TalonFX(10);
    public TalonFX elevatorFollower = new TalonFX(9);
    public Follower follower = new Follower(10, false);
    DigitalInput lowerLimitSwitch = new DigitalInput(3);
    DigitalInput upperLimitSwitch = new DigitalInput(2);
    public double elevatorCanDown;
    public double elevatorCanUp;
    public PIDController pid = new PIDController(0.07, 0, 0);
    // public DutyCycleEncoder degEncoder = new DutyCycleEncoder(0); // Never actually used. Elevator is ran off motor encoder values, assuming the elevator
    // is at the very bottom when powered on.
    private final LinearInterpolator interpolator = new LinearInterpolator();
    private PolynomialSplineFunction m_elevInterpolatorGI;
    private PolynomialSplineFunction m_elevInterpolatorBumper;

    public double elevatorPosition;
    public double elevatorSetpoint;
    public boolean tryRumble;
    public boolean readyRumble;
    public static double elevHandoffLowerLimit = 29.7192;
    public static boolean there = false;
    public double up;
    public double elevGroundIntakeLowerLimit = 16.6754; // change later
    // public double elevarmground = 30.5634;
    public double elevHandoffPosition = 30.2;
    public static double gearRatio = 37.8;
    public static double gearCoeff = (1.756*Math.PI*2)/gearRatio;
    public boolean testHandoff = false;
    
        
        public double aSDHUAWDUJHAOSDJUHWODIntakeLevel = 34.60164; // change later, an arbitrary value to get ground intake to stow faster... change to use either lowerlimit or position, plus x inches
    
        public double tol = 0.19684;
        public double tol2 = 0.19684;
        public double speed;
    
        public Elevator(){
            elevatorFollower.setControl(follower);
            elevatorSetpoint = antiTransform(getRotation());
            double[] elevGI = Controller.elevSafetyGI2;
            double[] armGI = Controller.armSafetyGI2;
            
            double[] elevBumper = Controller.elevSafetyBumper2;
            double[] armBumper = Controller.armSafetyBumper2;
            m_elevInterpolatorGI = interpolator.interpolate(armGI, elevGI);
            m_elevInterpolatorBumper = interpolator.interpolate(armBumper, elevBumper);
        }
    
        public double getRotation(){
            return elevatorLeader.getPosition().getValueAsDouble();
        }
    
        public void kill () {
            elevatorLeader.set(0);
        }
    
        public void setpoint(double setpoint){
            this.elevatorSetpoint = setpoint;
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
        elevatorPosition = antiTransform(getRotation());
        // SmartDashboard.putNumber("safety elev", encoderValue);
        // If upper limit switch is not hit, canDown is 1. If lower is not hit, canUp is 1.
        // SmartDashboard.putNumber("elevator value", m_elevInterpolator.value(Math.abs(RobotContainer.m_arm.realEncoderValue)));
        if ((!lowerLimitSwitch.get())
        ){ // ZERO ENCODERS AND WHATNOT, NEED TO ACCOUNT FOR ENCODER OFFSET THINGY IN OTHER PARTS OF THE CODE
            elevatorCanDown = 0;
            // m_Leader.setPosition(0); // CONSIDER SLIGHTLY HIGHER THAN ZERO (THE VALUE AS SOON AS WE TOUCH THE SWITCH)
            // setpoint = m_Leader.getPosition().getValueAsDouble();
        }
        else {
            elevatorCanDown = 1;
        }
        if ((!upperLimitSwitch.get())){
            elevatorCanUp = 0;
            // m_Leader.setPosition(transform(48.61212)); // CHECK VALUE........
            // setpoint = antiTransform(m_Leader.getPosition().getValueAsDouble());
            // encoderValue = antiTransform(getRotation());
        }
        else {
            elevatorCanUp = 1;
        }
        if (((elevatorSetpoint < elevGroundIntakeLowerLimit) || (elevatorPosition < elevGroundIntakeLowerLimit)) && (((RobotContainer.m_groundintake.desiredEncoderValue > 182)) || ((RobotContainer.m_groundintake.realEncoderValue > 182)))) {
            elevatorCanDown = 0;
        }
        // SmartDashboard.putNumber("elevtest1", m_elevInterpolator.value(Math.abs(RobotContainer.m_arm.realEncoderValue)));

        if (((!RobotContainer.m_groundintake.clearArmOutside) && (elevatorPosition < m_elevInterpolatorBumper.value(Math.abs(RobotContainer.m_arm.armPosition)))) || ((!RobotContainer.m_groundintake.clearArmOutside) && (elevatorPosition < m_elevInterpolatorGI.value(Math.abs(RobotContainer.m_arm.armPosition))))) {
            elevatorCanDown = 0;
        }
        
        double leftY = -MathUtil.applyDeadband(RobotContainer.m_mechController.getLeftY(), 0.15);
        if (leftY==0) {
            speed = pid.calculate(elevatorPosition, elevatorSetpoint);
            // Prevents movement if limit switch is hit.
            if (speed>0){
                speed *= elevatorCanUp;
            }
            else {
                speed*=elevatorCanDown;
            }
            // Sets speed.
            elevatorLeader.set(speed);
        }
        // else if (Controller.manual) {
        else {
            if (leftY > 0){
                elevatorLeader.set(leftY*elevatorCanUp);
            }
            else {
                elevatorLeader.set(leftY*elevatorCanDown);
            }
            elevatorSetpoint = elevatorPosition;
            tryRumble = false;
        }

        if ((MathUtil.applyDeadband(elevatorPosition-elevHandoffPosition, tol) == 0) && (MathUtil.applyDeadband(elevatorSetpoint-elevHandoffPosition, tol) == 0)){
            there = true;
        }
        else {
            there = false;
        }

        if (MathUtil.applyDeadband(elevatorPosition-elevHandoffPosition+5.5, tol) == 0 && (MathUtil.applyDeadband(elevatorSetpoint-elevHandoffPosition+5.5, tol) == 0)) {
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

        SmartDashboard.putNumber("(Elevator) CanUp", elevatorCanUp);
        SmartDashboard.putNumber("(Elevator) CanDown", elevatorCanDown);
        SmartDashboard.putNumber("(Elevator) True inch encoder value", elevatorPosition);
        SmartDashboard.putNumber("(Elevator) True setpoint elev", elevatorSetpoint);
    }
}
