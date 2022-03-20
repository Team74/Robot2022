package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//Allows the Auton to be created with the Auton Class
public class AutonThree extends Auton {
  
  double time;
  double lastSavedTime = 0;
  int index = 0;

  public AutonThree(SwerveDrive drive, Shooter shooter, AHRS gyro, Servo servo, boolean colorIsBlue, ColorSensorV3 colorSensor){

    super(drive, shooter, gyro, servo, colorIsBlue, colorSensor);

  }

  public void run(double time, double x){
    this.time = time;

    NetworkTableEntry pipeline = table.getEntry("pipeline");

    switch (index) {
      case 0:    //Init, reset gyro, tilt limelight foreward

        servo.set(0.9);

        gyro.reset();

        index = index+1;

        if(colorIsBlue){
          //Change To Blue Ball Pipeline
        }else{
          //Change To Red Ball Pipeline
        }

      case 1: //Move Forward and pickup Ball

        drive.moveRobotAbsolute(x, 1.0, 0.0);
        shooter.spinIntake(1.0);

        if(colorSensor.getProximity() > 200){

          index = index+1;

          lastSavedTime = time;

          servo.set(0.0);
          //Change To Vision Pipeline

        }

        break;
      case 2:  //Move Towards driver station, spin towards Goal and start flywheel

        if(Math.abs(drive.getGyroAngle()-225) > 10){
          drive.moveRobotAbsolute(1.0, -0.3, 1.0);
        }else{
          drive.moveRobotAbsolute(1.0, -0.3, x);
        }

        shooter.spinIntake(0.0);
        shooter.setFlywheelSpeed(9000);


        if((time-lastSavedTime)>0.2){

          index=index+1;

        }

        break;
      case 3:  //Continue flywheel and lock onto goal

      shooter.setFlywheelSpeed(9000);

      if(x > 3){
        drive.moveRobotAbsolute(0.0, 0.0, x);
      }else{
        drive.moveRobotAbsolute(0.0, 0.0, 0.0);

        if(shooter.getFlywheelSpeed()>8500){
          index = index+1;

          lastSavedTime = time;
        }
      }

        break;
      case 4:  //Shoot Balls

      shooter.setFlywheelSpeed(9000);
      shooter.spinIndexer(1.0);

      if((time-lastSavedTime)>0.5){
        index = index+1;

        servo.set(0.9);

        if(colorIsBlue){
          //Change To Blue Ball Pipeline
        }else{
          //Change To Red Ball Pipeline
        }
      }

        break;
      case 5:   //Stop flywheel, spin towards next ball

        shooter.spinFlywheel(0.0);
      
        drive.moveRobotAbsolute(0.0, 0.0, -1.0);

        if(Math.abs(drive.getGyroAngle()-170)<10){
          index = index+1;
        }

        break;
      case 6:   //Drive Forward and pick up ball 3

        drive.moveRobotRelative(x, 1.0, 0.0);
        shooter.spinIntake(1.0);

        if(colorSensor.getProximity() > 200){

          index = index+1;

        }

        break;
      case 7:   //Turn towards Ball 4, drive towards it and pick it up

        if(Math.abs(drive.getGyroAngle()-225) > 10){
          drive.moveRobotAbsolute(1.0, 0.1, 1.0);
        }else{
          drive.moveRobotAbsolute(1.0, 0.1, x);
          shooter.spinIntake(1.0);

        }

        if(colorSensor.getProximity() > 200){

          index = index+1;

          lastSavedTime = time;

          servo.set(0.0);
          //Change To Vision Pipeline

        }

        break;
      case 8:   //Get into shooting position

        shooter.spinIntake(0.0);
        shooter.setFlywheelSpeed(9000.0);

        if(Math.abs(drive.getGyroAngle()-250) > 10){
          drive.moveRobotAbsolute(-1.0, -0.1, 1.0);
        }else{
          drive.moveRobotAbsolute(-1.0, -0.1, x);
        }

        if((time-lastSavedTime)>1){
          index = index+1;
        }

        break;
      case 9:

      drive.moveRobotAbsolute(0.0, 0.0, x);

        shooter.setFlywheelSpeed(9000);
        if(x>3){

          drive.moveRobotAbsolute(0.0, 0.0, x);

        }else if(shooter.getFlywheelSpeed()>8500){
          index = index+1;
          lastSavedTime=time;

        }else{
          drive.moveRobotAbsolute(0.0, 0.0,0.0);
        }
      
        break;
      case 10:
        
        drive.moveRobotAbsolute(0.0, 0.0,0.0);

        shooter.spinIndexer(1.0);

        if((time-lastSavedTime)>1.0){
          index = index+1;
        }
        break;
      case 11:
        shooter.spinIndexer(0.0);
        shooter.spinFlywheel(0.0);
        shooter.spinIntake(0.0);

        drive.moveRobotAbsolute(0.0, 0.0,0.0);

        break;
    }
    
  }

}