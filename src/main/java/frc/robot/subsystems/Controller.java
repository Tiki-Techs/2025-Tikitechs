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
import frc.robot.subsystems.Elevator;

public class Controller extends SubsystemBase{
    public Elevator elevator;
    public Arm arm;
    public Intake intake;
    // public Intake intake;
    public static boolean manual = true;

    public Controller (Elevator elevator, Arm arm, Intake intake) {
        this.elevator = elevator;
        this.arm = arm;
        this.intake = intake;
        
        // REMOVE
        // var shooterMap = ShooterConstants.SHOOTER_MAP;
        // double[] distances = new double[shooterMap.size()];
        // double[] flywheelSpeeds = new double[shooterMap.size()];
        // double[] angles = new double[shooterMap.size()];

        // for (int i = 0; i < shooterMap.size(); i++) {
        //     distances[i] = shooterMap.get(i).getKey();
        //     flywheelSpeeds[i] = shooterMap.get(i).getValue().speed;
        //     angles[i] = shooterMap.get(i).getValue().angle;
        // }

        // m_shooterFlywheelCurve = SPLINE_INTERPOLATOR.interpolate(distances, flywheelSpeeds);
        // m_shooterAngleCurve = SPLINE_INTERPOLATOR.interpolate(distances, angles);
        // REMOVE, AND ABOVE NEEDS ADDITIONS
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

    public Command individualTest(){
        return new InstantCommand(
                () -> {
                    // arm.setpoint(arm.l4);
                    elevator.setpoint(elevator.l4);
                    rumble(true);
                }, this);

    }

    
    public Command individualTest2(){
        return new InstantCommand(
                () -> {
                    // arm.setpoint(arm.l4);
                    elevator.setpoint(elevator.l3);
                    rumble(true);
                }, this);

    }

    public Command togetherTest(){
        return new InstantCommand(
                () -> {
                    arm.setpoint(arm.l4);
                    elevator.setpoint(elevator.l4);
                    rumble(true);
                }, this);

    }

    public Command togetherTest2(){
        return new InstantCommand(
                () -> {
                    arm.setpoint(arm.l4);
                    elevator.setpoint(elevator.l4);
                    rumble(true);
                }, this);

    }

    public Command rumbleOff(){
        return new InstantCommand(
                () -> {
                    rumble(false);
                }, this);

    }

    
    public Command intake(){
        return new InstantCommand(
                () -> {
                    // uintake code
                    rumble(false);
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

    // REMOVE
    public static class State {
        public final double elev;
        public final double arm;
        public final double speed;

        public State(double elev, double arm, double speed) {
            this.elev = elev;
            this.arm = arm;
            this.speed = speed;
        }
    }
    // REMOVE

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
        if ((MathUtil.applyDeadband(RobotContainer.m_driverController.getLeftX(), 0.15) != 0) || 
        (MathUtil.applyDeadband(RobotContainer.m_driverController.getLeftY(), 0.15) != 0)){
            elevator.setpoint(elevator.neutral);
            arm.setpoint(arm.neutral);
        }
    }
}
