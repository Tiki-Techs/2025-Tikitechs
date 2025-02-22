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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MotorRunnerElevManual extends SubsystemBase {
    
    public TalonFX m_Leader = new TalonFX(10);
    // public SparkMax test = new SparkMax(10, null)
    public TalonFX m_follower = new TalonFX(9);
    public Follower follower = new Follower(10, false);
    
    // public TalonFX m_Follower = new TalonFX(0);
    // public Follower follower = new Follower(12, false);

    public MotorRunnerElevManual(){
        m_follower.setControl(follower);
        // m_Leader.setPosition(0);
        // m_Leader.set(0.5);
    }
// 
    // public double getEncoderValue(){
    //     return encoderBR.get();
    // }

    public Command motorStartTalonCommand(double speed) {

        return new RunCommand(
                () -> {
                    // m_Leader.set(speed);
                }, this);

    }

    public Command motorStartTalonCommand(boolean test) {

        return new RunCommand(
                () -> {
                    // talonFX.setVoltage(1);
                }, this);

    }


public Command motorStopTalonCommand (){

return new RunCommand(
    () -> {
    // talonFX.set(0);
    // talonFX2.set(0);
    }, this);    
}

@Override
public void periodic(){
    if (MathUtil.applyDeadband(RobotContainer.m_driverController.getLeftY(), 0.15) != 0){
                // m_Leader.set(Math.signum(RobotContainer.m_driverController.getLeftY()));
                m_Leader.set(RobotContainer.m_driverController.getLeftY());
            }
    else {
        m_Leader.set(0);
    }
    double e1 = m_Leader.getPosition().getValueAsDouble();
    double e2 = m_follower.getPosition().getValueAsDouble();
    SmartDashboard.putNumber("e1", e1);
    SmartDashboard.putNumber("e2", e2);
    // double value = m_Leader.getPosition().getValueAsDouble();
    // SmartDashboard.putNumber("Encoder Position", value);
    // SmartDashboard.putNumber("speed", m_Leader.get());
}

}

