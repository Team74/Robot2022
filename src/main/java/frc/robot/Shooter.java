// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class Shooter {

    CANSparkMax flywheelMotor1;
    CANSparkMax flywheelMotor2;
    CANSparkMax indexerMotor;
    CANSparkMax intakeMotor;

    RelativeEncoder flywheelEncoder;
    private SparkMaxPIDController velocityController;

    double flywheelSpeed;
    double overCurrentTime = 0; 
    double flywheelCurrent; 

    public Shooter(int flywheelMotorPort1, int flywheelMotorPort2, int indexerMotorPort, int intakeMotorPort){

        flywheelMotor1 = new CANSparkMax(flywheelMotorPort1, MotorType.kBrushed);
        flywheelMotor2 = new CANSparkMax(flywheelMotorPort2, MotorType.kBrushed);
        indexerMotor = new CANSparkMax(indexerMotorPort, MotorType.kBrushless);
        intakeMotor = new CANSparkMax(intakeMotorPort, MotorType.kBrushless);

        flywheelMotor1.restoreFactoryDefaults();
        flywheelMotor2.restoreFactoryDefaults();

        flywheelMotor2.follow(flywheelMotor1, true);

        flywheelEncoder = flywheelMotor1.getEncoder(Type.kQuadrature, 4096);

        velocityController = flywheelMotor1.getPIDController();

        velocityController.setP(0.006);
        velocityController.setI(0);
        velocityController.setD(0);
        velocityController.setIZone(0);
        velocityController.setFF(0.000025);
        velocityController.setOutputRange(-1.0, 1.0);
    }

    public void spinFlywheel(double flywheelPower){

        flywheelMotor1.set(flywheelPower);

        SmartDashboard.putNumber("Flywheel Power", flywheelPower);
    }

    public void setFlywheelSpeed(double desiredSpeed){
        SmartDashboard.putNumber("Desired Speed", desiredSpeed);

        desiredSpeed = desiredSpeed * 0.125;

        velocityController.setReference(desiredSpeed, CANSparkMax.ControlType.kVelocity);

        SmartDashboard.putNumber("PID output", flywheelMotor1.get());
    }

    public void spinIndexer(double indexerPower){

        indexerMotor.set(-indexerPower);
        SmartDashboard.putNumber("Index Power", indexerPower);
    }

    public void spinIntake(double intakePower){

        intakeMotor.set(-intakePower);
        SmartDashboard.putNumber("Intake Power", intakePower);
    }  

    public double getFlywheelSpeed(){

        flywheelSpeed = flywheelEncoder.getVelocity();
        flywheelSpeed = flywheelSpeed*8.0;

        SmartDashboard.putNumber("Flywheel Speed", flywheelSpeed);

        return flywheelSpeed;

    }

    public void getFlywheelPosition(){
        SmartDashboard.putNumber("Flywheel Position", flywheelEncoder.getPosition());
    }

    public boolean isCurrentHigh(){

        flywheelCurrent = (flywheelMotor1.getOutputCurrent() + flywheelMotor2.getOutputCurrent())*0.5;

        SmartDashboard.putNumber("flywheelCurrent", flywheelCurrent);

        if(flywheelCurrent > 1000){
            overCurrentTime = overCurrentTime + 0.02;
        }

        if(overCurrentTime < 1.0){
            return false;
        }else{
            return true;
        }
    }



}
