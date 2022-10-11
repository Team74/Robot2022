package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//Allows the Auton to be created with the Auton Class
public class AutonFive extends Auton {
  
  double time;
  double lastSavedTime = 0;
  int index = 0;

  public AutonFive(SwerveDrive drive, Shooter shooter, AHRS gyro, Servo servo, boolean colorIsBlue, ColorSensorV3 colorSensor){

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
          pipeline.setNumber(1);
        }else{
          //Change To Red Ball Pipeline
          pipeline.setNumber(2);
        }

      case 1: //Move Forward and pickup Ball

        drive.moveRobotAbsolute(x*0.03, 0.25, 0.0);
        shooter.spinIntake(1.0);

        if(colorSensor.getProximity() > 200){

          index = index+1;

          lastSavedTime = time;

          servo.set(0.0);
          //Change To Vision Pipeline
          pipeline.setNumber(0);

        }

        break;
      case 2:  //Turn Everything Off

        drive.moveRobotAbsolute(0.0, 0.0, 0.0);
        shooter.spinIntake(0.0);

        break;
    }
    
  }

}