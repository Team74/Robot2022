package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//Allows the Auton to be created with the Auton Class
public class AutonThree extends Auton {

  double time;
  double lastSavedTime = 0.0;
  double proximityDelay;
  int index;

  public AutonThree(SwerveDrive drive, Shooter shooter, AHRS gyro, Servo servo, boolean colorIsBlue, ColorSensorV3 colorSensor){

    super(drive, shooter, gyro, servo, colorIsBlue, colorSensor);

  }

  public void run(double time, double x){
    this.time = time;

    NetworkTableEntry pipeline = table.getEntry("pipeline");

    SmartDashboard.putNumber("Auton Index", index);
    SmartDashboard.putNumber("Time", time);
    SmartDashboard.putNumber("Last Saved Time", lastSavedTime);

    switch (index) {
      case 0:    //Init, reset gyro, tilt limelight foreward

        if(lastSavedTime == 0.0) lastSavedTime = time;

        shooter.spinFlywheel(0.0);
        shooter.spinIntake(1.0);
        shooter.spinIndexer(0.0);
        gyro.reset();

        if((time-lastSavedTime)>0.75){
          index = index+1;
        }

        if(colorIsBlue){
          //Change To Blue Ball Pipeline
          pipeline.setNumber(1);
        }else{
          //Change To Red Ball Pipeline
          pipeline.setNumber(2);
        }
        break;
      case 1:
        servo.set(0.9);
        index = index+1;

        break;
      case 2: //Move Forward and pickup Ball

        shooter.spinIntake(1.00);

        if(colorSensor.getProximity() > 200){

          proximityDelay = proximityDelay + 0.02;
          drive.moveRobotAbsolute(0.0, 0.0, 0.0);


        }else{
          proximityDelay = 0.0;
          drive.moveRobotAbsolute(0.0, 0.22, 0.0);
        }
        
        if((proximityDelay>0.14)||((time-lastSavedTime)>3.25)){
          index = index+1;
          servo.set(0.0);
          //Change To Vision Pipeline
          pipeline.setNumber(0);
        }

        break;
      case 3:  //Move Towards driver station, spin towards Goal and start flywheel

        shooter.spinIndexer(-0.1);
        shooter.spinIntake(0.0);
        shooter.setFlywheelSpeed(4600);
        drive.moveRobotAbsolute(0.0, 0.0, 0.75);

        if(Math.abs(drive.getGyroAngle()-165.0) < 5.0){
          index = index+1;
          lastSavedTime = time;
        }

        break;
      case 4:

        drive.moveRobotAbsolute(0.0, 0.0, x*0.1);

        shooter.spinIndexer(-0.1);
        shooter.spinIntake(0.0);
        shooter.setFlywheelSpeed(4600);


        if(Math.abs(time-lastSavedTime)>1.5){

          index=index+1;
          lastSavedTime = time;

        }

        break;
      case 5:

        drive.moveRobotAbsolute(0.0, 0.0, 0.0);
        shooter.setFlywheelSpeed(4400.0);
        shooter.spinIndexer(1.0);
        shooter.spinIntake(0.5);

        if((time-lastSavedTime)>2.0){
          index = index+1;
        }
        break;
      case 6:

        shooter.spinIntake(0.0);
        drive.moveRobotAbsolute(0.0, 0.0, 0.0);
        shooter.spinFlywheel(0.0);
        shooter.spinIndexer(0.0);

        servo.set(0.0);
        
        break;
    }
  }
}