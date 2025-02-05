package frc.robot.subsystems;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;


public class ClimbSubsytem extends SubsystemBase{
private final SparkMax m_motor1 = new SparkMax(1, MotorType.kBrushless) {
    
};
