// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class Shooter {

    /*CANSparkMax flywheelMotor1;
    CANSparkMax flywheelMotor2;*/
    CANSparkMax indexerMotor;
    CANSparkMax intakeMotor;

    TalonFX newFlywheel;




    //RelativeEncoder flywheelEncoder;
    private SparkMaxPIDController velocityController;

    double flywheelSpeed;
    double overCurrentTime = 0; 
    double flywheelCurrent; 

    public Shooter(int flywheelMotorPort1, int flywheelMotorPort2, int indexerMotorPort, int intakeMotorPort){

        newFlywheel = new TalonFX(25);

        /*flywheelMotor1 = new CANSparkMax(flywheelMotorPort1, MotorType.kBrushed);
        flywheelMotor2 = new CANSparkMax(flywheelMotorPort2, MotorType.kBrushed);*/
        indexerMotor = new CANSparkMax(indexerMotorPort, MotorType.kBrushless);
        intakeMotor = new CANSparkMax(intakeMotorPort, MotorType.kBrushless);

        newFlywheel.configFactoryDefault();
        newFlywheel.setNeutralMode(NeutralMode.Coast);

        /* Config sensor used for Primary PID [Velocity] */
        newFlywheel.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
											

        /* Config the peak and nominal outputs */
        newFlywheel.configNominalOutputForward(0, 30);
        newFlywheel.configNominalOutputReverse(0, 30);
        newFlywheel.configPeakOutputForward(1, 30);
        newFlywheel.configPeakOutputReverse(-1, 30);

        /* Config the Velocity closed loop gains in slot0 */
        newFlywheel.config_kF(0, 0.0465, 30);
        newFlywheel.config_kP(0, 0.1, 30);
        newFlywheel.config_kI(0, 0.0, 30);
        newFlywheel.config_kD(0, 0.01, 30);
    
        newFlywheel.configClosedloopRamp(0.5);

    }

    public void spinFlywheel(double flywheelPower){

        newFlywheel.set(ControlMode.PercentOutput, -flywheelPower);

        //flywheelMotor1.set(flywheelPower);

        SmartDashboard.putNumber("Flywheel Power", flywheelPower);
    }

    public void setFlywheelSpeed(double desiredSpeed){

        SmartDashboard.putNumber("Desired Speed", desiredSpeed);

        desiredSpeed = -desiredSpeed*2048.0/600.0;

        newFlywheel.set(ControlMode.Velocity, desiredSpeed);
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

        flywheelSpeed = newFlywheel.getSelectedSensorVelocity();
        flywheelSpeed = -flywheelSpeed*600.0/2048.0;            //Actually returns number of sensor ticks per 100 ms
                                                              //2048 sensor ticks per rotation, 600*100ms per minute
        SmartDashboard.putNumber("Flywheel Speed", flywheelSpeed);

        return flywheelSpeed;

    }

    public void getFlywheelPosition(){
        //SmartDashboard.putNumber("Flywheel Position", flywheelEncoder.getPosition());
    }

    public boolean isCurrentHigh(){

        //flywheelCurrent = (flywheelMotor1.getOutputCurrent() + flywheelMotor2.getOutputCurrent())*0.5;

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

    public void changeIndexIdle(boolean brakeMode){
        if(brakeMode){
            indexerMotor.setIdleMode(IdleMode.kBrake);
        }else{
            indexerMotor.setIdleMode(IdleMode.kCoast);
        }
    }
}
