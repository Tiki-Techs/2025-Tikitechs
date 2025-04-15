
package frc.robot.subsystems;

import org.apache.commons.math3.analysis.interpolation.LinearInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Arm extends SubsystemBase {

    // ACTUAL CLASSES
    public TalonFX armLeader = new TalonFX(12); // FIGURE OUT MOTION MAGIC, SWITCH TO SHUFFLEBOARD(?)
    public TalonFX armFollower = new TalonFX(11);
    public Follower follower = new Follower(12, false);
    public ProfiledPIDController pid = new ProfiledPIDController(0.030, 0, 0.000,
            new TrapezoidProfile.Constraints(130, 560)); // TUNE PID, went from 14 to 20 just now
    // public PIDController pid = new PIDController(0.020, 0, 0.0000); // TUNE PID,
    // went from 14 to 20 just now
    public DutyCycleEncoder degEncoder = new DutyCycleEncoder(1, 360, 325.7);
    // public DutyCycleEncoder degEncoder = new DutyCycleEncoder(1);
    private final LinearInterpolator interpolator = new LinearInterpolator();
    private PolynomialSplineFunction m_armInterpolatorGI;
    private PolynomialSplineFunction m_armInterpolatorBumper;

    // LOGIC VARIABLES
    public double armPosition = 12;
    // public double throughboreValue;
    public double armPositionTransformed;
    public double armSetpoint;
    public double armPseudoSetpoint;
    public boolean tryRumble = false;
    public boolean readyRumble = false;
    public double canRight = 0;
    public double canLeft = 0;
    public double isCCW = -456787;
    public double speed = 0;
    public double armTolerance = 4;
    public double l4Score = -37;

    public double modS = 0.75;
    // public double algaeLow;
    // public double algaeHigh;
    public boolean positiveUp;
    public double down;
    public static boolean armThere;
    public boolean testHandoff = false;
    public boolean testDrop = false;
    public static double armChange = 0;

    // CLASS CONSTRUCTOR, CALLED ON INITIALIZATION OF CLASS (DEPLOYMENT OF CODE IF
    // CLASS IS CREATED IN ROBOT CONTAINER).
    public Arm() {
        armFollower.setControl(follower);
        // ON DEPLOYMENT, SETPOINT IS SET TO CURRENT POINT TO PREVENT MOVEMENT.
        armSetpoint = transform(degEncoder.get());
        // INTERPOLATOR AND SPLINE FUNCTION TRANSFORM GIVEN SETS OF VALUES TO A
        // FUNCTION. THE TYPE OF
        // FUNCTION (CUBIC, SQUARED, ETC) CAN CHANGE. USED TO DETERMINE WHAT ARM VALUES
        // ARE SAFE BASED
        // ON THE CURRENT ELEVATOR POSITION.
        double[] elevGI = Controller.elevSafetyGI;
        double[] armGI = Controller.armSafetyGI;

        double[] elevBumper = Controller.elevSafetyBumper;
        double[] armBumper = Controller.armSafetyBumper;
        m_armInterpolatorGI = interpolator.interpolate(elevGI, armGI);
        m_armInterpolatorBumper = interpolator.interpolate(elevBumper, armBumper);
    }

    public double down() {
        return down;
    }

    public void kill() {
        armLeader.set(0);
    }
    // public boolean there () {

    // }

    // SETTER TO CHANGE SETPOINT FROM OTHER CLASSES.
    public void setpoint(double setpoint) {
        this.armSetpoint = setpoint;
    }

    public void setpointDrop(double setpoint) {
        this.armSetpoint = setpoint;
    }

    // LOGIC FOR WHETHER OR NOT THE CONTROLLER IS MEANT TO RUMBLE. SEE ARM PERIODIC,
    // CONTROLLER PERIODIC, AND OTHER REFERENCES.
    public void tryRumble(boolean rumble) {
        this.tryRumble = rumble;
    }

    // TRANSFORMS THROUGHBORE ENCODER VALUE INTO SOMETHING EASILY USED WITH PID
    // CONTROL.
    public double transform(double encoder) {
        double ret;
        if (encoder > 180) {
            ret = 360 - encoder;
        } else {
            ret = -encoder;
        }
        return ret;
    }

    // CALLED EVERY 20 MILLISECONDS
    @Override
    public void periodic() {
        // TRANSFORMS THROUGHBORE VALUE FOR PID USAGE. TOP MIDDLE BECOMES 0, LEFT MIDDLE
        // BECOMES 90, RIGHT MIDDLE
        // BECOMES 90, BOTTOM MIDDLE BECOMES 180/-180.

        // transform makes the throughbore give 180 to -180 instead of 0 to 360. think
        // this has to do with arm wrapping logic, not really necessary now (could just
        // change throughbore zero?)
        armPosition = transform(degEncoder.get());
        armPositionTransformed = armPosition;
        armPseudoSetpoint = -500000;

        // tells which side the arm is on, used to prevent wrapping. only changes when
        // in the top half (to prevent arm thinking it came from the other side after
        // passing the bottom)

        if (armPosition < 90 && armPosition > 0) {
            isCCW = 1;
        } else if (armPosition > -90 && armPosition < 0) {
            isCCW = -1;
        }
        this.down = 180 * isCCW;

        // SmartDashboard.putNumber("Arm value",
        // m_armInterpolator.value(Math.abs(RobotContainer.m_elevator.encoderValue)));
        // IF THE ARM BEGAN CCW AND IS BETWEEN -160 AND -140 (IN THE LOWER RIGHT
        // QUADRANT), CANNOT MOVE FURTHER CCW,
        // VICE VERSA. THIS PREVENTS TWISTING OF THE INTAKE WIRES.

        // checks: if arm comes from one side and tries to wrap, cannot move; not sure
        // what the second part is; if the arm's position is lower than what is
        // determined to
        // be safe by the interpolator in controller, cannot move (checks individually
        // for whether or not ground intake is cleared);

        // canRight = 1;


        if ((isCCW == 1) && ((MathUtil.applyDeadband(armPosition + 165, 15) == 0))) {
            canRight = 0;
        } else if (armPositionTransformed > 0 && armPositionTransformed < 90) {
            canRight = 0;
        } else if (((armPosition > m_armInterpolatorBumper.value(Math.abs(RobotContainer.m_elevator.elevatorPosition)))
                && RobotContainer.m_groundintake.clearArmOutside)) {
            armPseudoSetpoint = -m_armInterpolatorBumper.value(Math.abs(RobotContainer.m_elevator.elevatorPosition));
        } else if (((armPosition > m_armInterpolatorGI.value(Math.abs(RobotContainer.m_elevator.elevatorPosition)))
                && !RobotContainer.m_groundintake.clearArmOutside)) {
            armPseudoSetpoint = -m_armInterpolatorGI.value(Math.abs(RobotContainer.m_elevator.elevatorPosition));
        } else {
            canRight = 1;
        }


        if ((((isCCW == -1) && (((MathUtil.applyDeadband(armPosition - 165, 15) == 0))))
                || (((armPosition < -m_armInterpolatorBumper
                        .value(Math.abs(RobotContainer.m_elevator.elevatorPosition)))
                        && RobotContainer.m_groundintake.clearArmOutside)
                        || (armPosition < -m_armInterpolatorGI
                                .value(Math.abs(RobotContainer.m_elevator.elevatorPosition)))
                                && !RobotContainer.m_groundintake.clearArmOutside))) {
            canLeft = 0;
        } else if (((armPosition < -m_armInterpolatorBumper.value(Math.abs(RobotContainer.m_elevator.elevatorPosition)))
                && RobotContainer.m_groundintake.clearArmOutside)) {
            armPseudoSetpoint = -m_armInterpolatorBumper.value(Math.abs(RobotContainer.m_elevator.elevatorPosition));
        } else if ((armPosition < -m_armInterpolatorGI.value(Math.abs(RobotContainer.m_elevator.elevatorPosition)))
                && !RobotContainer.m_groundintake.clearArmOutside && !armThere) {
            armPseudoSetpoint = -m_armInterpolatorGI.value(Math.abs(RobotContainer.m_elevator.elevatorPosition));
        } else {
            canLeft = 1;
        }
        SmartDashboard.putBoolean("armthere", armThere);

        // if the arm is within 7 degrees of an "unsafe" value as determined by the
        // controller's interpolator, speed is decreased to 6 % from 50%. This decreases
        // constant starting and stopping, which weakens rivets and is just generally
        // bad practice. should instead use a profiled pid or a slew rate limiter?

        // if ((armSetpoint <
        // -m_armInterpolatorGI.value(Math.abs(RobotContainer.m_elevator.elevatorPosition)))
        // && (MathUtil.applyDeadband(armSetpoint +
        // m_armInterpolatorGI.value(Math.abs(RobotContainer.m_elevator.elevatorPosition)),
        // 7) == 0) && (!RobotContainer.m_groundintake.clearArmOutside)) {
        // modS = 0.06;
        // canLeft = 1;
        // }
        // else {
        // modS = 0.5;
        // }

        SmartDashboard.putNumber("mods", modS);

        double rightX = -MathUtil.applyDeadband(RobotContainer.m_mechController.getRightY(), 0.15);

        // weird stuff about arm wrapping i forget
        if (isCCW == 1) {
            armPositionTransformed = Math.abs(armPosition);
        } else {
            armPositionTransformed = -Math.abs(armPosition);
        }

        // if not using right joystick, use pid control with setpoints
        if (rightX == 0) {
            modS = 0.75;
            // LOGIC PREVENTING TWISTING OF WIRES.
            if (isCCW == 1) {
                armPositionTransformed = Math.abs(armPosition);
            } else {
                armPositionTransformed = -Math.abs(armPosition);
            }
            // SmartDashboard.putString("test", "PID");
            if (armPseudoSetpoint == -500000 || armThere) {
                speed = pid.calculate(armPositionTransformed, armSetpoint);
            } else{
                speed = pid.calculate(armPositionTransformed, armPseudoSetpoint);
            }
            // LOGIC THAT SHOULD NOT NEED TO BE USED.
            if (speed > 0) {
                speed *= canRight;
                // m_Leader.set(rightX*canLeft*isLeft);
            } else {
                speed *= canLeft;
                // m_Leader.set(rightX*canRight*(-isLeft));
            }
        }

        // if manually controlling, set a speed based on the controller input.
        else {
            // SHOULD NOT RUMBLE WHEN AT SETPOINT (SEE BELOW, SEE CONTROLLER PERIODIC)
            tryRumble = false;
            modS = 0.5;

            // SmartDashboard.putString("test", "not PID");
            // LOGIC FOR MANUAL CONTROL.
            if (rightX > 0) {
                speed = rightX * canRight * 1; // ccw and joystick left
            } else {
                speed = rightX * canLeft * 1; // ccw and joystick right
            }
            if (isCCW == 1 && armPosition < 0) {
                armSetpoint = 180;
            } else if (isCCW == -1 && armPosition > 0) {
                armSetpoint = -180;
            } else {
                armSetpoint = armPositionTransformed;
            }

            pid.reset(armPositionTransformed);
            // else {
            // if (rightX > 0) {
            // speed = rightX*canRight*1; // cw and joystick left
            // }
            // else {
            // speed = rightX*canLeft*1; // cw and joystick right
            // }
            // }
            // SIMPLIFIED LOGIC FOR MANUAL CONTROL (BASED ON THE ABOVE)
            // if (rightX > 0) {
            // speed = rightX*canCW*-1;
            // }
            // else {
            // speed = rightX*canCCW*1;
            // }
        }

        // prevents speed from passing 1 so modification can function
        if (speed > 1) {
            speed = 1;
        } else if (speed < -1) {
            speed = -1;
        }
        if (isCCW == -456787) {
            armLeader.set(0);
        } else {
            armLeader.set(speed * modS);
        }

        // IF MEANT TO RUMBLE, WILL CHECK WHETHER OR NOT THE POSITION IS WITHIN A
        // CERTAIN RANGE OF THE DESIRED POSITION.
        // IF SO, WILL SET READY RUMBLE TO TRUE. SEE CONTROLLER PERIODIC.

        // checks if the arm is in the correct state for handoff, for both(?)
        if (MathUtil.applyDeadband(armPositionTransformed + 180, armTolerance) == 0) {
            armThere = true;
        } else {
            armThere = false;
        }

        if (MathUtil.applyDeadband(armPositionTransformed - l4Score, armTolerance) == 0) {
            testDrop = true;
        } else {
            testDrop = false;
        }

        if (MathUtil.applyDeadband(armPositionTransformed + 180, armTolerance) == 0) {
            testHandoff = true;
        } else {
            testHandoff = false;
        }

        SmartDashboard.putNumber("(Arm) True encoder value", armPosition);
        SmartDashboard.putNumber("(Arm) True encoder value2", armPositionTransformed);
        SmartDashboard.putNumber("down ARM val", down);
        SmartDashboard.putNumber("(Arm) True setpoint elev", armSetpoint);
        SmartDashboard.putNumber("(Arm) Pseudo setpoint value", armPseudoSetpoint);
        SmartDashboard.putNumber("Left", isCCW);
        SmartDashboard.putNumber("Can Left", canLeft);
        SmartDashboard.putNumber("Can Right", canRight);
    }
}
