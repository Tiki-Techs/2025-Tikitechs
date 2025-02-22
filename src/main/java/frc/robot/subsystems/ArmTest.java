package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ControllerConstants;

public class ArmTest extends SubsystemBase {
    // public TalonFX armLeader = new TalonFX(3);
    // public TalonFX armFollower = new TalonFX(0);
    // private Follower follower = new Follower(0, false);
    // public PIDController pid = new PIDController(0, 0, 0);
    // public DutyCycleEncoder encoder = new DutyCycleEncoder(0);
    public double l2;
    public double l3;
    public double coralFeeder;
    private double setpoint;
    public boolean tryRumble;
    public double encoderValue;
    public boolean manual = false;
    public PIDController PID = new PIDController(0.5, 0.0, 0.0);

    public ArmTest () {
        // armFollower.setControl(follower);
        // setpoint = armLeader.getPosition().getValueAsDouble();
    }

    public void setpoint(double setpoint){
        // this.setpoint = setpoint;
        this.setpoint = setpoint;
    }

    public void tryRumble(boolean rumble){
        this.tryRumble = rumble;
    }

    // public double getRotation(){
    //     // return encoder.get();
    // }
    
    @Override
    public void periodic () {
        // encoderValue = armLeader.getPosition().getValueAsDouble();
            // armLeader.set(pid.calculate(encoderValue, setpoint));
        // SmartDashboard.putBoolean("ElevatorTest setpoint called", manual);
        SmartDashboard.putNumber("Arm Encoder", encoderValue);
        SmartDashboard.putNumber("setpoint arm", setpoint);
        // armLeader.set(PID.calculate(encoderValue, setpoint));

        // if not override, to setpoint
        // if setpoint within x of setpoint and if rumbletrue, system rumble indicator
    }
}
