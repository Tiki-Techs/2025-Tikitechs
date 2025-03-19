// add second encoder logic later

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class GroundIntake extends SubsystemBase{

    // hood neo, power vortex

    SparkFlex m_power = new SparkFlex(14, MotorType.kBrushless);
    TalonFX m_hood = new TalonFX(15);

    public double hoodSpeed = 0;
    public double powerSpeed = 0;
    public double realEncoderValue;
    public double desiredEncoderValue = 0;
    public double canDown = 0;
    public double canUp = 0;
    // consider doing a breakPID for the stuff
    public DutyCycleEncoder encoder = new DutyCycleEncoder(5, 360, 0);
    // public PIDController pid = new PIDController(0.0045, 0, 0);
    // public PIDController pid = new PIDController(0.0050, 0, 0);
    
    public PIDController pid = new PIDController(0.0050, 0, 0);
    private final SlewRateLimiter limiter = new SlewRateLimiter(5.4);
    public double intake = 67.0;
    public double l1Value = 173;
    // public double mainIntakeHas = 120; // x
    public double mainIntakeHas = 173; // x
    public double groundIntakeHas = 234.5;
    // public double groundIntakeHas = 229.5;
    
    public static boolean have = true;
    public double tempIntake = 0;
    public double current = 0;
    public final Timer m_Timer = new Timer();
    public boolean wait = false;
    public boolean clearArmSelf = false;
    public boolean clearArmOutside = false;
    public static boolean handoff = false;
    public boolean handoffThere = false;
    public boolean l1 = false;
    public boolean auto = false;

    public GroundIntake(){
        
    }

    public void setSpeed(double speed){
    }

    // public Command intake() {
    //     return new InstantCommand(() -> {
    //     hoodSpeed = pid.calculate(realEncoderValue, intake);
    //     if (hoodSpeed > 0) {
    //         m_hood.set(canUp*hoodSpeed);
    //     }
    //     else {
    //         m_hood.set(canDown*hoodSpeed);
    //     }
    //     if (!have || wait){
    //     // m_power.set(0.35);
    //     }
    //     else {
    //         m_power.set(0);
    //     }
    //     }, this);
    // }

    
    public void intake2() {
        desiredEncoderValue = intake;
        hoodSpeed = pid.calculate(realEncoderValue, intake);
        // hoodSpeed = 0;
        if (hoodSpeed > 0) {
            m_hood.set(canUp*hoodSpeed);
        }
        else {
            m_hood.set(canDown*hoodSpeed);
        }
        if (!have || wait){
        m_power.set(limiter.calculate(0.5));
        }
        else {
            m_power.set(0);
        }
    }

    public void handoff() {
        desiredEncoderValue = groundIntakeHas;
        hoodSpeed = pid.calculate(realEncoderValue, groundIntakeHas);
        // hoodSpeed = 0;
        if (hoodSpeed > 0) {
            m_hood.set(canUp*hoodSpeed);
        }
        else {
            m_hood.set(hoodSpeed*3);
        }
    }

    public Command off() {
        return new InstantCommand(() -> {
            
        desiredEncoderValue = realEncoderValue;
        hoodSpeed = pid.calculate(realEncoderValue, realEncoderValue);
        if (hoodSpeed > 0) {
            m_hood.set(canUp*hoodSpeed);
        }
        else {
            m_hood.set(canDown*hoodSpeed);
        }
        // if (!have){
        m_power.set(0);
    }, this);
        // }
    }

    public boolean getHave() {
        return have;
    }

    public void l1() {
            desiredEncoderValue = l1Value;
        hoodSpeed = pid.calculate(realEncoderValue, l1Value);
        if (hoodSpeed > 0) {
            m_hood.set(canUp*hoodSpeed);
        }
        else {
            m_hood.set(canDown*hoodSpeed);
        }
        // m_power.set(0);
    }

    public Command haveFalse() {
        return new InstantCommand(() -> {
            have = false;
            RobotContainer.m_intake.handoff = false;
            handoff = false;
    }, this);
    }
    public Command haveTrue() {
        return new InstantCommand(() -> {
            have = true;
    }, this);
    }

    public Command l1CommandTrue() {
        return new InstantCommand(() -> {
            l1 = true;
    }, this);
    }

    public Command l1CommandFalse() {
        return new InstantCommand(() -> {
            l1 = false;
    }, this);
    }

    // public Command autoSpit() {
    //     return new InstantCommand(() -> {
    //         m_Timer.reset();
    //         m_Timer.start();
    //     while (m_Timer.get() <= 2) {
    //         m_power.set(-0.5);
    //         SmartDashboard.putBoolean("test auto gi", false);
    //     }
    //         m_power.set(0);
    //         SmartDashboard.putBoolean("test auto gi", true);
    // }, this);
    // }

    public Command autoSpit() {
        return new InstantCommand(() -> {
            auto = true;
            m_power.set(-0.2);
    }, this);
    }

    
    public Command autoSpitOff() {
        return new InstantCommand(() -> {
            auto = false;
            m_power.set(0);
    }, this);
    }

    public void groundIntakeHas() {
        if (!clearArmSelf){
            desiredEncoderValue = l1Value;
            hoodSpeed = pid.calculate(realEncoderValue, l1Value);
        if (hoodSpeed > 0) {
            m_hood.set(hoodSpeed);
        }
        else {
            m_hood.set(canDown*hoodSpeed);
        }
        }
        else {
            desiredEncoderValue = groundIntakeHas;
            hoodSpeed = pid.calculate(realEncoderValue, groundIntakeHas);
            
        if (hoodSpeed > 0) {
            m_hood.set(canUp*hoodSpeed);
        }
        else {
            m_hood.set(canDown*hoodSpeed);
        }
    }
        // do a double if there then speed, maybe. if so, speed at .5
    }
    
    public void mainIntakeHas() {
        desiredEncoderValue = mainIntakeHas;
        hoodSpeed = pid.calculate(realEncoderValue, mainIntakeHas);
        if (hoodSpeed > 0) {
            m_hood.set(canUp*hoodSpeed);
        }
        else {
            m_hood.set(canDown*hoodSpeed);
        }
        // m_power.set(0);
    }
    // public void output () {
    //     m_hood.set(pid.calculate(realEncoderValue, 0)); // change setpoint and power
    //     if (Arm.there && Elevator.there)
    //     m_power.set(0.75);
    // }

    // public void have () {
    //     m_Timer.start();
    //     while (m_Timer.get() < 0.08) {
    //         m_power.set(-0.1);
    //         wait = true;
    //     }
    //     m_power.set(0);
    //     wait = false;
    // }

    // public void notHave() {
    //     try {
    //     wait(100);
    //     }
    //     catch (Exception e) {
    //     have = false;
    //     }
    // }

    public void waiting(){
        if (m_Timer.get() < 0.6) {
            wait = true;
            // waiting();
        }
        else {
            wait = false;
            m_Timer.reset();
        }
    }

    // @Override
    // public void peri

    @Override
    public void periodic(){
    
        // double tempsoe = m_hood.get();
        // if (tempsoe > 1) {
        //     tempsoe = 1;
        // }
        // else if (tempsoe < -1) {
        //     tempsoe = -1;
        // }
        // SmartDashboard.putNumber("Hood Power", tempsoe);

        if ((((RobotContainer.m_elevator.setpoint < RobotContainer.m_elevator.groundIntakeLevel) || (RobotContainer.m_elevator.encoderValue < RobotContainer.m_elevator.groundIntakeLevel)) || (((RobotContainer.m_elevator.setpoint < RobotContainer.m_elevator.elevarmground) || (RobotContainer.m_elevator.encoderValue < RobotContainer.m_elevator.elevarmground)) && ((Math.abs(RobotContainer.m_arm.desiredEncoderValue) > 135) || (Math.abs(RobotContainer.m_arm.realEncoderValue2) > 135))))) {
            clearArmSelf = false;
        }
        else {
            clearArmSelf = true;
        }
        SmartDashboard.putBoolean("cleararmself", clearArmSelf);
        // if (MathUtil.applyDeadband(realEncoderValue-mainIntakeHas, 5) == 0) {
        //     clearArmOutside = false;
        // }
        // else {
        //     clearArmOutside = true;
        // }
        if (((realEncoderValue > intake-5) && (realEncoderValue < mainIntakeHas+5)) && ((desiredEncoderValue > intake-5) && (desiredEncoderValue < mainIntakeHas+5))) {
            clearArmOutside = true;
        }
        else {
            clearArmOutside = false;
        }
        SmartDashboard.putBoolean("cleararmoutside", clearArmOutside);
        realEncoderValue = encoder.get();
        current = m_power.getOutputCurrent();

        if (realEncoderValue > groundIntakeHas || (((RobotContainer.m_elevator.setpoint < RobotContainer.m_elevator.groundIntakeLevel && !clearArmSelf) || (RobotContainer.m_elevator.encoderValue < RobotContainer.m_elevator.groundIntakeLevel && !clearArmSelf)) || (((RobotContainer.m_elevator.setpoint < RobotContainer.m_elevator.elevarmground-3 && !clearArmSelf) || (RobotContainer.m_elevator.encoderValue < RobotContainer.m_elevator.elevarmground-3 && !clearArmSelf)) && ((Math.abs(RobotContainer.m_arm.desiredEncoderValue) > 135) || (Math.abs(RobotContainer.m_arm.realEncoderValue2) > 135))))) {
            canUp = 0;
        }
        else {
            canUp = 1;
        }
        if (realEncoderValue < intake || (((RobotContainer.m_elevator.setpoint < RobotContainer.m_elevator.aSDHUAWDUJHAOSDJUHWODIntakeLevel) || (RobotContainer.m_elevator.encoderValue < RobotContainer.m_elevator.aSDHUAWDUJHAOSDJUHWODIntakeLevel)) && ((Math.abs(RobotContainer.m_arm.realEncoderValue) > 135) || ((Math.abs(RobotContainer.m_arm.desiredEncoderValue) > 135))))) {
            canDown = 0;
        }
        else {
            canDown = 1;
        }

        if ((current > 80 && m_power.get() > 0) || wait) {
            if (!wait) {
                m_Timer.start();
                wait = true;
            }
            if (m_Timer.get() > 0.9) {
                wait = false;
                have = true;
                m_Timer.reset();
                m_Timer.stop();
            }
            // put have and wait togehter if can
        }


        if (MathUtil.applyDeadband(realEncoderValue-groundIntakeHas, 5) == 0 && handoff) {
            handoffThere = true;
        }
        else {
            handoffThere = false;
        }
        SmartDashboard.putBoolean("handoffthere", handoffThere);


        SmartDashboard.putBoolean("handoff ground intake", handoff);
        if (l1) {
            l1();
        }
        
        else if (handoff) {
            handoff();
            SmartDashboard.putBoolean("arm there", Arm.there);
            
            SmartDashboard.putBoolean("elev there", Elevator.there);
            
            SmartDashboard.putBoolean("intk there", RobotContainer.m_intake.m_Leader.get() <= -0.5);
            if ((Arm.there && Elevator.there) && (RobotContainer.m_intake.m_Leader.get() <= -0.5)) {
                m_power.set(-0.25); // check off later
            }
        }
        else if(!Intake.have && !have){
            intake2();
            // m_power.set(0.1);
            SmartDashboard.putString("Ground Intake Action", "intake2test");
        }
        else if (have && !Intake.have && !RobotContainer.m_mechController.rightBumper().getAsBoolean()
        //  && (RobotContainer.m_elevator.setpoint > RobotContainer.m_elevator.groundIntakeLevel || RobotContainer.m_elevator.encoderValue > RobotContainer.m_elevator.groundIntakeLevel)
        ){
            groundIntakeHas();
            // m_power.set(0.2);
            SmartDashboard.putString("Ground Intake Action", "give to intake");
        }
        // else if (have) {
        //     // do a thingy..?
        //     SmartDashboard.putString("Ground Intake Action", "wait to give to intake");
        // }
        else {
            // m_hood.set(pid.calculate(realEncoderValue, 0)); // change setpoint
            mainIntakeHas();
            // m_power.set(0);
            SmartDashboard.putString("Ground Intake Action", "wait to score");
        }
        if (auto) {

        }
        else if (RobotContainer.m_mechController.rightBumper().getAsBoolean()){
            // m_Leader.set(Math.signum(RobotContainer.m_driverController.getLeftY()));
            if (l1) {
             m_power.set(-0.4);   
            }
            else {
                m_power.set(-0.15);
            }
            // SmartDashboard.putNumber("right", RobotContainer.m_mechController.getRightTriggerAxis());
            have = false;
            // l1 = false;
            // if (have) {
            //     have = false;
            // }
        }
        // else if ((RobotContainer.m_mechController.leftBumper().getAsBoolean() || (!wait && have)) && !handoff){
        //     m_power.set(limiter.calculate(0));
        // }
        // real version above

        else if (RobotContainer.m_mechController.leftBumper().getAsBoolean()){
            m_power.set(limiter.calculate(0.5));
        }
        else if (((!wait && have)) && !handoff){
            m_power.set(limiter.calculate(0.05));
        }
        else if (Intake.have) {
            m_power.set(0.05);
        }
        // else {
        //     m_power.set(0);
        // }
        // else if (RobotContainer.m_mechController.x().getAsBoolean() && !have){
        //     // m_Leader.set(Math.signum(RobotContainer.m_driverController.getLeftY()));
        //     m_power.set(0.45);
        //     // SmartDashboard.putNumber("left", RobotContainer.m_mechController.getLeftTriggerAxis());
        // }
        
        // else if (RobotContainer.m_mechController.a().getAsBoolean()){
        //     m_power.set(0.25);
        // }
        
        // else if (RobotContainer.m_mechController.y().getAsBoolean()){
        //     m_power.set(0.35);
        // }
        // else if (RobotContainer.m_mechController.b().getAsBoolean() && !have){
        //     // m_Leader.set(Math.signum(RobotContainer.m_driverController.getLeftY()));
        //     m_power.set(0.55);
        //     // SmartDashboard.putNumber("left", RobotContainer.m_mechController.getLeftTriggerAxis());
        // }
        // else {
        //     if (!wait) {
        //         m_power.set(0);
        //         }
        // }
        // if (RobotContainer.m_mechController.povUp().getAsBoolean()){
        //     m_hood.set(0.17);


        //     // desiredEncoderValue = realEncoderValue;


        // }
        // else if (RobotContainer.m_mechController.povDown().getAsBoolean()){
        //     m_hood.set(-0.17);
        // }

        //     // desiredEncoderValue = realEncoderValue;


        // }
        // else {
        //     m_hood.set(0);


        //     // pid command


        // }

        // if (MathUtil.applyDeadband(RobotContainer.m_driverController.getRightTriggerAxis(), 0.15) != 0){
        //     m_power.set(-RobotContainer.m_driverController.getRightTriggerAxis()*0.45);
        //     have = false;
        // }
        // else if (MathUtil.applyDeadband(RobotContainer.m_driverController.getLeftTriggerAxis(), 0.15) != 0 && !have && !wait){
        //     m_power.set(RobotContainer.m_driverController.getLeftTriggerAxis()*0.35);
        // }
        // else {
        //     if (!wait) {
        //         m_power.set(0);
        //         }
        // }
        // if (MathUtil.applyDeadband(RobotContainer.m_driverController.getLeftY(), 0.15) != 0) {
        //     m_hood.set(-RobotContainer.m_driverController.getLeftY()*0.1);
        // }
        // else {
        //     m_hood.set(0);
        // }


        SmartDashboard.putBoolean("have ground", have);
        SmartDashboard.putBoolean("wait", wait);
        // SmartDashboard.putNumber("voltage", m_power.getBusVoltage());
        SmartDashboard.putNumber("current", m_power.getOutputCurrent());
        SmartDashboard.putNumber("groundIntake through encoder", realEncoderValue);
        SmartDashboard.putNumber("ground intake desired value", desiredEncoderValue);
        SmartDashboard.putNumber("ground intake hood speed", m_hood.get());
        SmartDashboard.putNumber("gi canup", canUp);
        SmartDashboard.putNumber("gi candown", canDown);
        SmartDashboard.putNumber("Ground Intake Timer", m_Timer.get());
    }

       
}
