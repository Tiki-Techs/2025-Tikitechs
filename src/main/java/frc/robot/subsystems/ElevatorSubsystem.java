package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ElevatorSubsystem extends SubsystemBase {
    private final TalonFX m_motor1 = new TalonFX(ElevatorConstants.ELEVATOR_MOTOR_1);
    private final TalonFX m_motor2 = new TalonFX(ElevatorConstants.ELEVATOR_MOTOR_2);
    private final Follower m_follower = new Follower(ElevatorConstants.ELEVATOR_MOTOR_1, true);
    private final DigitalInput m_toplimitSwitch = new DigitalInput(0);
    private final DigitalInput m_bottomlimitSwitch = new DigitalInput(1);
    private double m_motorSpeed = 0;

    public ElevatorSubsystem(){
        m_motor2.setControl(m_follower);
    }

    public void setMotorSpeed(double speed){
        m_motor1.set(speed);
    }

   @Override
   public void periodic() {
   }
//    public void setMotorSpeed(double speed) {
//        if (speed > 0) {
//          if (m_toplimitSwitch.get()) {
//             //  We are going up and top limit is tripped so stop
//                motor.set(0);
//            } else {
//             //  We are going up but top limit is not tripped so go at commanded speed
//             motor.set(speed);
//            }
//        } else {
//           if (m_bottomlimitSwitch.get()) {
//                 // We are going down and bottom limit is tripped so stop
//                motor.set(0);
//            } else {
//                 // We are going down but bottom limit is not tripped so go at commanded speed
//                motor.set(speed);
//            }
//        }
//    }
}

