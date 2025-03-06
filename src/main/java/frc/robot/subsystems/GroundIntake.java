// add second encoder logic later

package frc.robot.subsystems;

import org.dyn4j.geometry.MassType;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ControllerConstants;
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
    public PIDController pid = new PIDController(0.0015, 0, 0);
    public double intake = 10.6;
    public double l1 = 0;
    public double mainIntakeHas = 133;
    public double groundIntakeHas = 175;
    public static boolean have = false;
    public double tempIntake = 0;
    public double current = 0;
    public final Timer m_Timer = new Timer();
    public boolean wait = false;

    public GroundIntake(){
        
    }

    public void setSpeed(double speed){
    }

    public Command intake() {
        return new InstantCommand(() -> {
        hoodSpeed = pid.calculate(realEncoderValue, intake);
        if (hoodSpeed > 0) {
            m_hood.set(canUp*hoodSpeed);
        }
        else {
            m_hood.set(canDown*hoodSpeed);
        }
        if (!have || wait){
        m_power.set(0.35);
        }
        else {
            m_power.set(0);
        }
        }, this);
    }

    
    public void intake2() {
        hoodSpeed = pid.calculate(realEncoderValue, intake);
        if (hoodSpeed > 0) {
            m_hood.set(canUp*hoodSpeed);
        }
        else {
            m_hood.set(canDown*hoodSpeed);
        }
        if (!have || wait){
        m_power.set(0.35);
        }
        else {
            m_power.set(0);
        }
    }

    public Command off() {
        return new InstantCommand(() -> {
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


    public Command l1() {
        return new InstantCommand(() -> {
        hoodSpeed = pid.calculate(realEncoderValue, l1);
        if (hoodSpeed > 0) {
            m_hood.set(canUp*hoodSpeed);
        }
        else {
            m_hood.set(canDown*hoodSpeed);
        }
        m_power.set(0);
    }, this);
    }

    public Command haveSwitch() {
        return new InstantCommand(() -> {
            have = !have;
    }, this);
    }

    public void groundIntakeHas() {
        hoodSpeed = pid.calculate(realEncoderValue, groundIntakeHas);
        if (hoodSpeed > 0) {
            m_hood.set(canUp*hoodSpeed);
        }
        else {
            m_hood.set(canDown*hoodSpeed);
        }
        // do a double if there then speed, maybe. if so, speed at .5
    }
    
    public void mainIntakeHas() {
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
        wait = true;
        try {
            wait(500);
        }
        catch (Exception e) {

        }
        wait = false;
    }

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


        SmartDashboard.putNumber("Power Power", m_power.get());
        realEncoderValue = encoder.get();
        current = m_power.getOutputCurrent();

        if (realEncoderValue > groundIntakeHas || (RobotContainer.m_elevator.setpoint < RobotContainer.m_elevator.groundIntakeLevel || RobotContainer.m_elevator.encoderValue < RobotContainer.m_elevator.groundIntakeLevel)) {
            canUp = 0;
        }
        else {
            canUp = 1;
        }
        if (realEncoderValue < intake) {
            canDown = 0;
        }
        else {
            canDown = 1;
        }

        if (current > 80
        //  && MathUtil.applyDeadband(RobotContainer.m_mechController.getLeftTriggerAxis(), 0.15) == 0
         ) {
            // have();
            waiting();
            have = true;
        }
        
        if(!Intake.have && !have){
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

        if (RobotContainer.m_mechController.rightBumper().getAsBoolean()){
            // m_Leader.set(Math.signum(RobotContainer.m_driverController.getLeftY()));
            m_power.set(-0.50);
            // SmartDashboard.putNumber("right", RobotContainer.m_mechController.getRightTriggerAxis());
            have = false;
            // if (have) {
            //     have = false;
            // }
        }
        else if (RobotContainer.m_mechController.leftBumper().getAsBoolean() && !have){
            m_power.set(0);
        }
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
        // SmartDashboard.putNumber("voltage", m_power.getBusVoltage());
        SmartDashboard.putNumber("current", m_power.getOutputCurrent());
        SmartDashboard.putNumber("groundIntake through encoder", realEncoderValue);
    }

       
}
