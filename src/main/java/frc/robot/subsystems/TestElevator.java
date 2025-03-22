package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.RobotContainer;

public class TestElevator {
    // Limit switch x2 Motors x2
    public TalonFX elevatorMotor = new TalonFX(0);
    public TalonFX elevatorMotor2 = new TalonFX(1);
    public DigitalInput upperLimit = new DigitalInput(0);
    public DigitalInput lowerLimit = new DigitalInput(1);

    public int upEngaged = 0; // 1 press dowm, 0 disengaged 
    public int lowEngaged = 0; 

    public void elevatorLimit() {
        if(!upperLimit.get()) {
            upEngaged = 1;
        } else {
            upEngaged = 0;
        }
        
        if(!lowerLimit.get()) {
            lowEngaged = 1;
        } else {
            lowEngaged = 0;
        }

        if(upEngaged == 1 || lowEngaged == 1){
            elevatorMotor.set(0);
        }

    }

    public void elevatorManual() {
        double x = RobotContainer.m_driverController.getLeftY(); 
        elevatorMotor.set(RobotContainer.m_driverController.getLeftY());

       // if (){

        }


    }


