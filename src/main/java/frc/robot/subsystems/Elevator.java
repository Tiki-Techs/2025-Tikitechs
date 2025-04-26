// add second encoder logic later

package frc.robot.subsystems;

import org.apache.commons.math3.analysis.interpolation.LinearInterpolator;
import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

// 137.8125

public class Elevator extends SubsystemBase {
    /**
     * 
     */
    public TalonFX elevatorLeader = new TalonFX(10);
    /**
     * 
     */
    public TalonFX elevatorFollower = new TalonFX(9);
    /**
     * 
     */
    public Follower follower = new Follower(10, false);
    /**
     * 
     */
    DigitalInput lowerLimitSwitch = new DigitalInput(3);
    /**
     * 
     */
    DigitalInput upperLimitSwitch = new DigitalInput(2);
    /**
     * 
     */
    public double elevatorCanDown;
    /**
     * 
     */
    public double elevatorCanUp;
    /**
     * 
     */
    public ProfiledPIDController pid = new ProfiledPIDController(0.4, 0, 0, 
    /**
     * 
     */
    new TrapezoidProfile.Constraints(50, 375));
    /**
     * 
     */
    private final LinearInterpolator interpolator = new LinearInterpolator();
    /**
     * 
     */
    private PolynomialSplineFunction m_elevInterpolatorGI;
    /**
     * 
     */
    private PolynomialSplineFunction m_elevInterpolatorBumper;

    /**
     * 
     */
    public double elevatorPosition;
    /**
     * 
     */
    public double elevatorSetpoint;
    /**
     * 
     */
    public static double elevHandoffLowerLimit = 29.7192;
    /**
     * 
     */
    public static boolean there = false;
    /**
     * 
     */
    public double up;
    /**
     * 
     */
    public double elevGroundIntakeLowerLimit = 18.9754; // change later
    // public double elevarmground = 30.5634;
    /**
     * 
     */
    public static double changeHandoff = 0.1;
    /**
     * 
     */
    public double elevHandoffPosition = 30.65+changeHandoff;
    // public double changeHandoff = 0;
    /**
     * 
     */
    public static double gearRatio = 37.8;
    /**
     * 
     */
    public static double gearCoeff = (1.756 * Math.PI * 2) / gearRatio;
    /**
     * 
     */
    public boolean testHandoff = false;
    /**
     * 
     */
    public boolean testDrop = false;
    /**
     * 
     */
    public double l4Score = 49.33;
    /**
     * 
     */
    public DutyCycleEncoder elevatorEncoder = new DutyCycleEncoder(0);

    /**
     * 
     */
    public double aSDHUAWDUJHAOSDJUHWODIntakeLevel = 36.60164; // change later, an arbitrary value to get ground intake
                                                               // to stow faster... change to use either lowerlimit or
                                                               // position, plus x inches

    /**
     * 
     */
    public double tol = 0.19684;
    /**
     * 
     */
    public double tol2 = 0.5384;
    /**
     * 
     */
    public double speed;

    public Elevator() {
        elevatorFollower.setControl(follower);
        elevatorSetpoint = antiTransform(getRotation());
        double[] elevGI = Controller.elevSafetyGI2;
        double[] armGI = Controller.armSafetyGI2;

        double[] elevBumper = Controller.elevSafetyBumper2;
        double[] armBumper = Controller.armSafetyBumper2;
        m_elevInterpolatorGI = interpolator.interpolate(armGI, elevGI);
        m_elevInterpolatorBumper = interpolator.interpolate(armBumper, elevBumper);
    }

    /**
     * Returns the motor's relative encoder value.
     */
    public double getRotation() {
        return elevatorLeader.getPosition().getValueAsDouble();
    }

    /**
     * Changes the elevator's setpoint.
     */
    public void setpoint(double setpoint) {
        this.elevatorSetpoint = setpoint;
    }

    /**
     * Transforms setpoint (inches) into motor turns. Not really used. Could be used for initial PID values?
     */
    public double transform(double val) {
        return (val) / gearCoeff;
    }

    /**
     * Transforms motor turns into setpoint (inches).
     */
    public double antiTransform(double val) {
        return (val * gearCoeff);
    }

    // x = (1.756*pi*2)/gearratio

    @Override
    public void periodic() {
        elevatorPosition = antiTransform(getRotation());
        if ((!lowerLimitSwitch.get())) { // ZERO ENCODERS AND WHATNOT, NEED TO ACCOUNT FOR ENCODER OFFSET THINGY IN
                                         // OTHER PARTS OF THE CODE
            elevatorCanDown = 0;
            elevatorLeader.setPosition(0); // CONSIDER SLIGHTLY HIGHER THAN ZERO (THE VALUE AS
            // SOON AS WE TOUCH THE SWITCH)
            // setpoint = m_Leader.getPosition().getValueAsDouble();
        } else {
            elevatorCanDown = 1;
        }

        if ((!upperLimitSwitch.get())) {
            elevatorCanUp = 0;
            // m_Leader.setPosition(transform(48.61212)); // CHECK VALUE........
            // setpoint = antiTransform(m_Leader.getPosition().getValueAsDouble());
            // encoderValue = antiTransform(getRotation());
        } else {
            elevatorCanUp = 1;
        }

        if (((elevatorSetpoint < elevGroundIntakeLowerLimit) || (elevatorPosition < elevGroundIntakeLowerLimit)) &&
                (((RobotContainer.m_groundintake.desiredEncoderValue > 182))
                        || ((RobotContainer.m_groundintake.realEncoderValue > 182)))) {
            elevatorCanDown = 0;
        }
        // SmartDashboard.putNumber("elevtest1",
        // m_elevInterpolator.value(Math.abs(RobotContainer.m_arm.realEncoderValue)));

        if (((RobotContainer.m_groundintake.clearArmOutside) &&
                (elevatorPosition < m_elevInterpolatorBumper.value(Math.abs(RobotContainer.m_arm.armPosition))))
                || ((!RobotContainer.m_groundintake.clearArmOutside) &&
                        (elevatorPosition < m_elevInterpolatorGI.value(Math.abs(RobotContainer.m_arm.armPosition))))) {
            elevatorCanDown = 0;
        }

        if (RobotContainer.m_controller.climb && elevatorPosition < 29.4 && RobotContainer.m_arm.climbArm) {
            elevatorCanDown = 0;
        }

        double leftY = -MathUtil.applyDeadband(RobotContainer.m_mechController.getLeftY(), 0.15);
        if (leftY == 0) {
            speed = pid.calculate(elevatorPosition, elevatorSetpoint);
            // Prevents movement if limit switch is hit.
            if (speed > 0) {
                speed *= elevatorCanUp;
            } else {
                speed *= elevatorCanDown;
            }
            // Sets speed.
            MathUtil.clamp(speed, -1, 1);
            elevatorLeader.set(speed*0.6);
        }
        // else if (Controller.manual) {
        else {
            if (leftY > 0) {
                elevatorLeader.set(leftY * elevatorCanUp * 1);
            } else {
                elevatorLeader.set(leftY * elevatorCanDown * 1);
            }
            elevatorSetpoint = elevatorPosition;
            pid.reset(elevatorPosition);
        }

        if ((MathUtil.applyDeadband(elevatorPosition - elevHandoffPosition, tol) == 0)
                && (MathUtil.applyDeadband(elevatorSetpoint - elevHandoffPosition, tol) == 0)) {
            there = true;
        } else {
            there = false;
        }

        if ((MathUtil.applyDeadband(elevatorPosition - l4Score, tol) == 0)
                && (MathUtil.applyDeadband(elevatorSetpoint - l4Score, tol) == 0)) {
            testDrop = true;
        } else {
            testDrop = false;
        }

        if (MathUtil.applyDeadband(elevatorPosition - elevHandoffPosition + 5.5, tol2) == 0
                && (MathUtil.applyDeadband(elevatorSetpoint - elevHandoffPosition + 5.5, tol2) == 0)) {
            testHandoff = true;
        } else {
            testHandoff = false;
        }

        SmartDashboard.putNumber("(Elevator) CanUp", elevatorCanUp);
        SmartDashboard.putNumber("(Elevator) CanDown", elevatorCanDown);
        SmartDashboard.putNumber("(Elevator) True inch encoder value", elevatorPosition);
        SmartDashboard.putNumber("(Elevator) True setpoint elev", elevatorSetpoint);
    }
}
