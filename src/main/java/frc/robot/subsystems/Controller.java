package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
// import frc.robot.subsystems.ArmTest;
// import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ElevatorTest;

public class Controller extends SubsystemBase{
    public ElevatorTest elevator;
    public ArmTest arm;
    public Intake intake;
    // public Intake intake;
    public boolean manual = true;

    public Controller (ElevatorTest elevator, ArmTest arm, Intake intake) {
        this.elevator = elevator;
        this.arm = arm;
        this.intake = intake;
        }

    // public void dL () {
    //     elevator.setpoint(elevator.drive);
    //     arm.setpoint(arm.drive);
    //     rumble(false);
    // }

    // public void dPOVU () {
    //     // auto align
    //     // will call neutral
    //     // elevator
    //     // arm
    //     // set tryrumble to false for all
    //     elevator.setpoint(elevator.neutral);
    //     arm.setpoint(arm.neutral);
    //     rumble(false);
    // }

    // public void dPOVL () {
    //     // coral left
    //     // will either angle or drive (pref. angle)
    //     // elevator
    //     // arm
    //     // rumble
    //     elevator.setpoint(elevator.neutral);
    //     arm.setpoint(arm.neutral);
    //     rumble(false);
    // }

    // public void dPOVR () {
    //     // coral right
    //     // same as above
    //     // 
    //     //
    //     elevator.setpoint(elevator.neutral);
    //     arm.setpoint(arm.neutral);
    //     rumble(false);
    // }

    // public void dX () {
    //     // climb
    //     // climb?
    //     // 
    //     //
    // }

    public void rumble(boolean rumble){
        arm.tryRumble(rumble);
        elevator.tryRumble(rumble);
    }

    // public Command mA(){
    //     return new InstantCommand(
    //             () -> {
    //                 arm.setpoint(arm.l2);
    //                 elevator.setpoint(elevator.l2);
    //                 rumble(true);
    //             }, this);

    // }
    
    public Command mB(){
        return new InstantCommand(
                () -> {
                    // arm.setpoint(arm.l3);
                    elevator.setpoint(elevator.l3);
                    rumble(true);
                }, this);

    }

    public Command mZero(){
        return new InstantCommand(
                () -> {
                    // arm.setpoint(arm.l3);
                    elevator.setpoint(elevator.encoderValue);
                    rumble(false);
                }, this);

    }
  
    // public Command mX(){
    //     return new InstantCommand(
    //             () -> {
    //                 arm.setpoint(arm.l1);
    //                 elevator.setpoint(elevator.l1);
    //                 rumble(true);
    //             }, this);

    // }
    
    public Command mY(){
        return new InstantCommand(
                () -> {
                    // arm.setpoint(arm.l4);
                    elevator.setpoint(elevator.l4);
                    rumble(true);
                }, this);

    }
    // public Command mPOVD(){
    //     return new InstantCommand(
    //             () -> {
    //                 arm.setpoint(arm.algaeLow);
    //                 elevator.setpoint(elevator.algaeLow);
    //                 rumble(true);
    //             }, this);

    // }
    
    // public Command mPOVU(){
    //     return new InstantCommand(
    //             () -> {
    //                 arm.setpoint(arm.algaeHigh);
    //                 elevator.setpoint(elevator.algaeHigh);
    //                 rumble(true);
    //             }, this);

    // }
    // public Command mLD(){
    //     return new InstantCommand(
    //             () -> {
    //                 arm.setpoint(arm.algaeGround);
    //                 elevator.setpoint(elevator.algaeGround);
    //                 rumble(true);
    //             }, this);

    // }
    
    // public Command mLU(){
    //     return new InstantCommand(
    //             () -> {
    //                 arm.setpoint(arm.coralFeeder);
    //                 elevator.setpoint(elevator.coralFeeder);
    //                 rumble(true);
    //             }, this);

    // }

    public Command mL(){
        return new InstantCommand(
                () -> {
                    if (MathUtil.applyDeadband(RobotContainer.m_driverController.getLeftY(), 0.15) > 0){
                    // arm.setpoint(arm.coralFeeder);
                    elevator.setpoint(elevator.coralFeeder);
                    rumble(true);}
                    else if (MathUtil.applyDeadband(RobotContainer.m_driverController.getLeftY(), 0.15) < 0){
                        // arm.setpoint(arm.coralFeeder);
                        elevator.setpoint(elevator.algaeGround);
                        rumble(true);}
                }, this);

    }

    public Command mR(){
    
        return new InstantCommand(
                () -> {
                    if (manual) {
                        elevator.setManual(true);
                        elevator.manual(RobotContainer.m_driverController.getRightY()); // add deadband
                    }
                    else {
                        // arm.setManual(true);
                        // arm.manual(RobotContainer.m_driverController.getRightY());
                    }
                    rumble(false);
                }, this);

    }
    
    public Command mLB () {
        return new InstantCommand(
            () -> {
                manual = !manual;
            });
            
    }

    public Command mLT(){
        return new InstantCommand(
                () -> {
                    
                    rumble(true);
                }, this);

    }

    @Override
    public void periodic (){
        if (elevator.readyRumble){
            // RobotContainer.m_driverController.setRumble(RumbleType.kBothRumble, 1);
            RobotContainer.m_driverController.setRumble(RumbleType.kBothRumble, 1); // figure out rumble = false
            // SmartDashboard.putBoolean("Elevator/Arm", manual);
        }
        else {
            RobotContainer.m_driverController.setRumble(RumbleType.kBothRumble, 0); // figure out rumble = false
        }
    }
}
