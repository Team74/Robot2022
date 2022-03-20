package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber {
    
    CANSparkMax leftClimberExtension;
    CANSparkMax rightClimberExtension; 

    CANSparkMax climberTilt; 

    double leftClimberExtensionPosition;
    double rightClimberExtensionPosition;

    RelativeEncoder leftClimberExtensionEncoder;
    RelativeEncoder rightClimberExtensionEncoder; 

    SparkMaxPIDController leftExtendController;
    SparkMaxPIDController rightExtendController;


    public Climber(int leftClimberExtensionMotorPort, int rightClimberExtensionMotorPort, int tiltMotorPort){

        leftClimberExtension = new CANSparkMax(leftClimberExtensionMotorPort, MotorType.kBrushless);
        rightClimberExtension = new CANSparkMax(rightClimberExtensionMotorPort, MotorType.kBrushless);

        climberTilt = new CANSparkMax(tiltMotorPort, MotorType.kBrushless);

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

    public void moveClimberManual(double climberPower){

        leftClimberExtension.set(climberPower);
        rightClimberExtension.set(-climberPower);

    }

    public void tiltClimberManual(double tiltPower){

        climberTilt.set(tiltPower);

    }

    public void moveClimberAuto(double desiredPosition){

        leftExtendController.setReference(-desiredPosition, CANSparkMax.ControlType.kPosition);
        rightExtendController.setReference(desiredPosition, CANSparkMax.ControlType.kPosition);

    }

    public void moveLeft(double leftPower){

        leftClimberExtension.set(leftPower);

    }
    public void moveRight(double rightPower){

        rightClimberExtension.set(-rightPower);

    }
}
