
package frc.robot.subsystems;

import java.util.Calendar;

import org.apache.commons.math3.analysis.interpolation.LinearInterpolator;
import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import org.dyn4j.world.Island;
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

    // ACTUAL CLASSES
    public TalonFX m_Leader = new TalonFX(12); // PUT MOTORS BACK ON BRAKE MODE
    public TalonFX m_Follower = new TalonFX(11);
    public Follower follower = new Follower(12, false);
    public PIDController pid = new PIDController(0.014, 0, 0.00008); // TUNE PID
    public DutyCycleEncoder degEncoder = new DutyCycleEncoder(1, 360, 149.14728);
    // public DutyCycleEncoder degEncoder = new DutyCycleEncoder(1);
    private final LinearInterpolator interpolator = new LinearInterpolator();
    private PolynomialSplineFunction m_armInterpolatorGI;
    private PolynomialSplineFunction m_armInterpolatorBumper;

    // LOGIC VARIABLES
    public double realEncoderValue;
    public double throughboreValue;
    public double realEncoderValue2;
    public double desiredEncoderValue;
    public boolean tryRumble = false;
    public boolean readyRumble = false;
    public double canRight = 0;
    public double canLeft = 0;
    public double isCCW = -456787;
    public double speed = 0;
    public double tol = 3;
    public boolean hasAlgae = false; // SHOULD NOT NEED

    // SETPOINT VARIABLES
    public double coralFeeder = 5;
    public double algaeGround = 1.5;
    public double l1;
    public double l2;
    public double l3 = 200;
    public double l4 = 414;
    public double algaeLow;
    public double algaeHigh;
    public boolean positiveUp;
    public double down;
    public double up;
    public boolean there;

    // CLASS CONSTRUCTOR, CALLED ON INITIALIZATION OF CLASS (DEPLOYMENT OF CODE IF CLASS IS CREATED IN ROBOT CONTAINER).
    public Arm(){
        m_Follower.setControl(follower);
        // ON DEPLOYMENT, SETPOINT IS SET TO CURRENT POINT TO PREVENT MOVEMENT.
        desiredEncoderValue = transform(degEncoder.get());
        // INTERPOLATOR AND SPLINE FUNCTION TRANSFORM GIVEN SETS OF VALUES TO A FUNCTION. THE TYPE OF
        // FUNCTION (CUBIC, SQUARED, ETC) CAN CHANGE. USED TO DETERMINE WHAT ARM VALUES ARE SAFE BASED
        // ON THE CURRENT ELEVATOR POSITION.
        double[] elevGI = Controller.elevSafetyGI;
        double[] armGI = Controller.armSafetyGI;
        
        double[] elevBumper = Controller.elevSafetyBumper;
        double[] armBumper = Controller.armSafetyBumper;
        m_armInterpolatorGI = interpolator.interpolate(elevGI, armGI);
        m_armInterpolatorBumper = interpolator.interpolate(elevBumper, armBumper);
    }

    public double down () {
        return down;
    }

    public void kill () {
        m_Leader.set(0);
    }
    // public boolean there () {

    // }

    // SETTER TO CHANGE SETPOINT FROM OTHER CLASSES.
    public void setpoint(double setpoint){
        this.desiredEncoderValue = setpoint;
    }

    // LOGIC FOR WHETHER OR NOT THE CONTROLLER IS MEANT TO RUMBLE. SEE ARM PERIODIC, CONTROLLER PERIODIC, AND OTHER REFERENCES.
    public void tryRumble(boolean rumble){
        this.tryRumble = rumble;
    }

    // TRANSFORMS THROUGHBORE ENCODER VALUE INTO SOMETHING EASILY USED WITH PID CONTROL.
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

    // CALLED EVERY 20 MILLISECONDS
    @Override
    public void periodic(){
        // TRANSFORMS THROUGHBORE VALUE FOR PID USAGE. TOP MIDDLE BECOMES 0, LEFT MIDDLE BECOMES 90, RIGHT MIDDLE
        // BECOMES 90, BOTTOM MIDDLE BECOMES 180/-180.
        // SmartDashboard.putNumber("through", degEncoder.get());
        throughboreValue = degEncoder.get();
        realEncoderValue = transform(degEncoder.get());
        realEncoderValue2 = realEncoderValue;
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

        // WHEN WITHIN 130 DEGREES OF THE TOP ON EITHER SIDE, SETS ISCCW. THIS WAY, ISCCW IS NOT SET IMMEDIATELY
        // WHEN GOING PAST THE BOTTOM MIDDLE
        if (realEncoderValue < 90 && realEncoderValue > 0){
            isCCW = 1;
        }
        else if (realEncoderValue > -90 && realEncoderValue < 0) {
            isCCW = -1;
        }
        this.down = 180*isCCW;
        // SmartDashboard.putNumber("Arm value", m_armInterpolator.value(Math.abs(RobotContainer.m_elevator.encoderValue)));
        // IF THE ARM BEGAN CCW AND IS BETWEEN -160 AND -140 (IN THE LOWER RIGHT QUADRANT), CANNOT MOVE FURTHER CCW,
        // VICE VERSA. THIS PREVENTS TWISTING OF THE INTAKE WIRES.
        if (((isCCW == 1) && ((MathUtil.applyDeadband(realEncoderValue+165, 15) == 0)) || (realEncoderValue2 > 0 && realEncoderValue2 < 90)
         || (((realEncoderValue > m_armInterpolatorBumper.value(Math.abs(RobotContainer.m_elevator.encoderValue))) && RobotContainer.m_groundintake.clearArm) || (realEncoderValue > m_armInterpolatorGI.value(Math.abs(RobotContainer.m_elevator.encoderValue))) && !RobotContainer.m_groundintake.clearArm)
         )) {
            canRight = 0;
        }
        if ((((isCCW == -1) && (((MathUtil.applyDeadband(realEncoderValue-165, 15) == 0))))
         || (((realEncoderValue < -m_armInterpolatorBumper.value(Math.abs(RobotContainer.m_elevator.encoderValue))) && RobotContainer.m_groundintake.clearArm) || (realEncoderValue < -m_armInterpolatorGI.value(Math.abs(RobotContainer.m_elevator.encoderValue))) && !RobotContainer.m_groundintake.clearArm)
         )) {
            canLeft = 0;
        }

        // GETS RIGHT JOYSTICK X VALUE OF THE MECH CONTROLLER IF IT IS MOVED PAST A CERTAIN RANGE, OTHERWISE 0.
        double rightX = -MathUtil.applyDeadband(RobotContainer.m_mechController.getRightX(), 0.15)*.75;
        if (isCCW == 1) {
            realEncoderValue2 = Math.abs(realEncoderValue);
        }
        else {
            realEncoderValue2 = -Math.abs(realEncoderValue);
        }
        // RIGHT JOYSTICK IS USED FOR MANUAL CONTROL. IF NOT USING MANUAL CONTROL, SETS SPEED BASED ON PID.
        if (rightX==0) {
            // LOGIC PREVENTING TWISTING OF WIRES.
            if (isCCW == 1) {
                realEncoderValue2 = Math.abs(realEncoderValue);
            }
            else {
                realEncoderValue2 = -Math.abs(realEncoderValue);
            }
            // SmartDashboard.putString("test", "PID");
            speed = pid.calculate(realEncoderValue2, desiredEncoderValue);
            // LOGIC THAT SHOULD NOT NEED TO BE USED.
            if (speed > 0){
                speed *= canRight;
                // m_Leader.set(rightX*canLeft*isLeft);
            }
            else {
                speed *= canLeft;
                // m_Leader.set(rightX*canRight*(-isLeft));
            }
            // Sets speed.
            // m_Leader.set(speed);
            // SmartDashboard.putString("Control", "Not Manual");
        }
        // IF MANUALLY CONTROLLING:
        else{
            // SHOULD NOT RUMBLE WHEN AT SETPOINT (SEE BELOW, SEE CONTROLLER PERIODIC)
            tryRumble = false;
            
            // SmartDashboard.putString("test", "not PID");
            // LOGIC FOR MANUAL CONTROL.
                if (rightX > 0) {
                    speed = rightX*canRight*1; // ccw and joystick left
                }
                else {
                    speed = rightX*canLeft*1; // ccw and joystick right
                }
            if (isCCW == 1 && realEncoderValue < 0) {
                desiredEncoderValue = 180;
            }
            else if (isCCW == -1 && realEncoderValue > 0){
                desiredEncoderValue = -180;
            }
            else {
                desiredEncoderValue = realEncoderValue2;
            }
            // else {
            //     if (rightX > 0) {
            //         speed = rightX*canRight*1; // cw and joystick left
            //     }
            //     else {
            //         speed = rightX*canLeft*1; // cw and joystick right
            //     }
            // }
            // SIMPLIFIED LOGIC FOR MANUAL CONTROL (BASED ON THE ABOVE)
            // if (rightX > 0) {
            //     speed = rightX*canCW*-1;
            // }
            // else {
            //     speed = rightX*canCCW*1;
            // }
            
            // SETS SETPOINT TO CURRENT VALUE SO THAT, WHEN RIGHT STICK IS RELEASED, PID WILL NOT MAKE ARM RETURN TO LAST SETPOINT POSITION.
            // desiredEncoderValue = realEncoderValue2;
            // SmartDashboard.putString("Control", "Manual");
        }
        // SETS MOTORS TO WHATEVER SPEED WAS DETERMINED IN THE PREVIOUS CODE
        if (speed > 1) {
            speed = 1;
        }
        else if (speed < -1) {
            speed = -1;
        }
        if (isCCW == 0) {
            m_Leader.set(0);
        }
        else {
            m_Leader.set(speed*.5);
        }

        // IF MEANT TO RUMBLE, WILL CHECK WHETHER OR NOT THE POSITION IS WITHIN A CERTAIN RANGE OF THE DESIRED POSITION.
        // IF SO, WILL SET READY RUMBLE TO TRUE. SEE CONTROLLER PERIODIC.
        if (MathUtil.applyDeadband(realEncoderValue2-desiredEncoderValue, tol) == 0) {
            there = true;
        }
        else {
            there = false;
        }

        if (tryRumble && there){
            readyRumble = true;
        }
        else {
                readyRumble = false;
            }

        SmartDashboard.putNumber("(Arm) True encoder value", realEncoderValue);
        SmartDashboard.putNumber("(Arm) True encoder value2", realEncoderValue2);
        SmartDashboard.putNumber("down ARM val", down);
        SmartDashboard.putNumber("(Arm) True setpoint elev", desiredEncoderValue);
        SmartDashboard.putNumber("Left", isCCW);
        SmartDashboard.putNumber("Can Left", canLeft);
        SmartDashboard.putNumber("Can Right", canRight);
        SmartDashboard.putNumber("Speed", speed);
        // double temp = m_armInterpolator.value(0);
        
        // double temp2 = m_armInterpolator.value(2);
        
        // double temp3 = m_armInterpolator.value(3);
        
        // double temp4 = m_armInterpolator.value(4);

        // SmartDashboard.putNumber("test", temp);
        // SmartDashboard.putNumber("test2", temp2);
        // SmartDashboard.putNumber("test3", temp3);
        // SmartDashboard.putNumber("test4", temp4);
    }
}
