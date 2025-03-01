package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MotorRunnerElevManual extends SubsystemBase {
    
    public TalonFX m_Leader = new TalonFX(10);
    public TalonFX m_follower = new TalonFX(9);
    public Follower follower = new Follower(10, false);
    DigitalInput lowerLimitSwitch = new DigitalInput(2);
    DigitalInput upperLimitSwitch = new DigitalInput(3);
    public DutyCycleEncoder degEncoder = new DutyCycleEncoder(0);
    public double canDown;
    public double canUp;


    public MotorRunnerElevManual(){
        m_follower.setControl(follower);
    }
    
@Override
public void periodic(){
    if ((!lowerLimitSwitch.get())){ 
        canDown = 0;
    }
    else {
        canDown = 1;
    }
    if ((!upperLimitSwitch.get())){
        canUp = 0;
    }
    else {
        canUp = 1;
    }

    double y = -RobotContainer.m_mechController.getLeftY();
    if (MathUtil.applyDeadband(y, 0.15) != 0){
                if (y > 0) {
                    m_Leader.set(y*canUp);
                }
                else {
                    m_Leader.set(y*canDown);
                }
            }
    else {
        m_Leader.set(0);
    }

    double e3 = degEncoder.get();
    double e1 = m_Leader.getPosition().getValueAsDouble();
    double e2 = m_follower.getPosition().getValueAsDouble();
    SmartDashboard.putNumber("e1", e1);
    SmartDashboard.putNumber("e2", e2);
    SmartDashboard.putNumber("trueencodervalue", e3);
    SmartDashboard.putNumber("(Elevator) CanUp", canUp);
    SmartDashboard.putNumber("(Elevator) CanDown", canDown);
    SmartDashboard.putNumber("y", y);
    // SmartDashboard.putBoolean("lowLim", !lowerLimitSwitch.get());
    
    // SmartDashboard.putBoolean("upperLim", !upperLimitSwitch.get());

    // double value = m_Leader.getPosition().getValueAsDouble();
    // SmartDashboard.putNumber("Encoder Position", value);
    // SmartDashboard.putNumber("speed", m_Leader.get());
}

}

