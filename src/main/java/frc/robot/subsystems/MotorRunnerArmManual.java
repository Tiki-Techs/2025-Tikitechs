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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MotorRunnerArmManual extends SubsystemBase {
    
    public TalonFX m_Leader = new TalonFX(12);
    public TalonFX m_follower = new TalonFX(11);
    public Follower follower = new Follower(12, false);
    public DutyCycleEncoder degEncoder = new DutyCycleEncoder(1);
    

    public MotorRunnerArmManual(){
        m_follower.setControl(follower);
    }

@Override
public void periodic(){
    double test = degEncoder.get();
    SmartDashboard.putNumber("througharm", test);
    if (MathUtil.applyDeadband(RobotContainer.m_mechController.getRightX(), 0.15) != 0){
                // m_Leader.set(Math.signum(RobotContainer.m_driverController.getLeftY()));
                m_Leader.set(RobotContainer.m_mechController.getRightX());
            }
    else {
        m_Leader.set(0);
    }
    double e1 = m_Leader.getPosition().getValueAsDouble();
    double e2 = m_follower.getPosition().getValueAsDouble();
    SmartDashboard.putNumber("Arm Leader", e1);
    SmartDashboard.putNumber("Arm Follower", e2);
}

}

