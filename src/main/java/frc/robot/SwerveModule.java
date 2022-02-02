package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;

public class SwerveModule{

    double offsetAngle;

    CANSparkMax driveMotor;
    CANSparkMax rotationMotor;
    DutyCycleEncoder swerveEncoder;

    PIDController angleAdjuster;

    public SwerveModule(int driveMotorPort, int rotationMotorPort, int rotationEncoderPort, double rotationEncoderOffsetAngle){

        driveMotor = new CANSparkMax(driveMotorPort, MotorType.kBrushless);
        rotationMotor = new CANSparkMax(rotationMotorPort, MotorType.kBrushless);
        swerveEncoder = new DutyCycleEncoder(rotationEncoderPort);

        angleAdjuster = new PIDController(1.0,0.0,0.00);

        angleAdjuster.enableContinuousInput(-180,180);
        angleAdjuster.setTolerance(10,20);
        angleAdjuster.reset();

        offsetAngle = rotationEncoderOffsetAngle;

    }

    public double getSwerveAngle(){

        double Angle = 0.0;

        Angle = swerveEncoder.get();

        Angle = Angle - (int)Angle;
        Angle = Angle + 1;
        Angle = Angle - (int)Angle;
        Angle = Angle*360;

        Angle = Angle - offsetAngle;

        if(Angle < 0){
            Angle = Angle + 360;
        }

        Angle = Angle-180;

        return Angle;
    }

    public void GoToAngle(double desiredAngle){

        double rotationMotorSpeed; 
        
        rotationMotorSpeed = angleAdjuster.calculate(getSwerveAngle(), desiredAngle);

        rotationMotorSpeed = rotationMotorSpeed/(180.0);
        rotationMotorSpeed = MathUtil.clamp(rotationMotorSpeed, -0.5, 0.5);

        if(angleAdjuster.atSetpoint()){
            rotationMotor.set(0);
            System.out.println("atSetPoint error");
        } else{
            rotationMotor.set(rotationMotorSpeed);
        }

        SmartDashboard.putNumber("Rotation Speed", rotationMotorSpeed);

    }

    public void setDriveMotor(double motorSpeed){

        driveMotor.set(motorSpeed);

    }


}
