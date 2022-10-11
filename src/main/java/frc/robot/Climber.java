package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber {
    
    CANSparkMax leftClimberExtension;
    CANSparkMax rightClimberExtension; 

    CANSparkMax climberTilt; 

    double leftClimberExtensionPosition;
    double rightClimberExtensionPosition;

    RelativeEncoder leftClimberExtensionEncoder;
    RelativeEncoder rightClimberExtensionEncoder; 

    DigitalInput tiltLimit;

    AnalogInput leftStringPot;
    AnalogInput rightStringPot;

    AnalogPotentiometer leftPotentiometer;
    AnalogPotentiometer rightPotentiometer;

    SparkMaxPIDController leftExtendController;
    SparkMaxPIDController rightExtendController;


    public Climber(int leftClimberExtensionMotorPort, int rightClimberExtensionMotorPort, int tiltMotorPort){

        leftClimberExtension = new CANSparkMax(leftClimberExtensionMotorPort, MotorType.kBrushless);
        rightClimberExtension = new CANSparkMax(rightClimberExtensionMotorPort, MotorType.kBrushless);

        climberTilt = new CANSparkMax(tiltMotorPort, MotorType.kBrushless);

        tiltLimit = new DigitalInput(4);

        leftStringPot = new AnalogInput(2);
        rightStringPot = new AnalogInput(0);

        leftClimberExtension.restoreFactoryDefaults();
        rightClimberExtension.restoreFactoryDefaults();

        leftClimberExtensionEncoder = leftClimberExtension.getEncoder();
        rightClimberExtensionEncoder = rightClimberExtension.getEncoder();

        leftClimberExtension.setIdleMode(IdleMode.kBrake);
        rightClimberExtension.setIdleMode(IdleMode.kBrake);
        climberTilt.setIdleMode(IdleMode.kBrake);

        leftExtendController = leftClimberExtension.getPIDController();
        rightExtendController = rightClimberExtension.getPIDController();

        leftExtendController.setP(0.006);
        leftExtendController.setI(0);
        leftExtendController.setD(0);
        leftExtendController.setIZone(0);
        leftExtendController.setFF(0.000015);
        leftExtendController.setOutputRange(-1.0, 1.0);

        rightExtendController.setP(0.006);
        rightExtendController.setI(0);
        rightExtendController.setD(0);
        rightExtendController.setIZone(0);
        rightExtendController.setFF(0.000015);
        rightExtendController.setOutputRange(-1.0, 1.0);


        /*leftClimberExtension.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        leftClimberExtension.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

        leftClimberExtension.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 15);
        leftClimberExtension.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);

        rightClimberExtension.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        rightClimberExtension.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

        rightClimberExtension.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 0);
        rightClimberExtension.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, -15); */
    }

    public void getExtensionPositions(){
        
        leftClimberExtensionPosition = leftClimberExtensionEncoder.getPosition();
        rightClimberExtensionPosition = rightClimberExtensionEncoder.getPosition();

        SmartDashboard.putNumber("Left Climber Position", leftClimberExtensionPosition);
        SmartDashboard.putNumber("Right Climber Position", rightClimberExtensionPosition);

    }

    public void moveClimberManual(double climberPower, boolean limitOveride, boolean lowHeight){

        double maxHeight = 4.07;

        if(lowHeight){
            maxHeight = 3.1;
        }

        if(((leftStringPot.getAverageVoltage() > 0.36 || climberPower > 0.0) && (leftStringPot.getAverageVoltage() < (maxHeight+0.00) || climberPower < 0.0)) || limitOveride){
            leftClimberExtension.set(climberPower);
        }else{
            leftClimberExtension.set(0.0);
        }

        if(((rightStringPot.getAverageVoltage() > 0.20 || climberPower > 0.0) && (rightStringPot.getAverageVoltage() < (maxHeight-0.17) || climberPower < 0.22)) || limitOveride){
            rightClimberExtension.set(-climberPower);
        }else{
            rightClimberExtension.set(0.0);
        }
    }

    public void tiltClimberManual(double tiltPower, boolean limitOverride){

        if(tiltPower < 0.0 || limitOverride || !tiltLimit.get()){
            climberTilt.set(tiltPower);
        }else{
            climberTilt.set(0.0);
        }

    }

    public void moveClimberAuto(double desiredPosition){

        leftExtendController.setReference(-desiredPosition, CANSparkMax.ControlType.kPosition);
        rightExtendController.setReference(desiredPosition, CANSparkMax.ControlType.kPosition);

    }

    public void moveLeft(double leftPower, boolean limitOveride, boolean lowHeight){

        double maxHeight = 4.07;

        if(lowHeight){
            maxHeight = 3.1;
        }

        if(((leftStringPot.getAverageVoltage() > 0.36 || leftPower > 0.0) && (leftStringPot.getAverageVoltage() < (maxHeight+0.22) || leftPower < 0.0)) || limitOveride){
            leftClimberExtension.set(leftPower);
        }else{
            leftClimberExtension.set(0.0);
        }

    }
    public void moveRight(double rightPower, boolean limitOveride, boolean lowHeight){

        double maxHeight = 4.07;

        if(lowHeight){
            maxHeight = 3.1;
        }

        if(((rightStringPot.getAverageVoltage() > 0.20 || rightPower > 0.0) && (rightStringPot.getAverageVoltage() < (maxHeight+0.00) || rightPower < 0.0)) || limitOveride){
            rightClimberExtension.set(-rightPower);
        }else{
            rightClimberExtension.set(0.0);
        }

    }

    public void outputClimberPositions(){

        SmartDashboard.putNumber("Left Climber Reading", leftStringPot.getAverageVoltage());
        SmartDashboard.putNumber("Right Climber Reading", rightStringPot.getAverageVoltage());

    }
}
