package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//Allows the Auton to be created with the Auton Class
public class AutonTwo extends Auton {

  double time;

  public AutonTwo(SwerveDrive drive, Shooter shooter, AHRS gyro, Servo servo, boolean colorIsBlue, ColorSensorV3 colorSensor){

    super(drive, shooter, gyro, servo, colorIsBlue, colorSensor);

  }

  public void run(double time, double x){
    this.time = time;

    SmartDashboard.putNumber("Time", time);

    if(0.0 < time  && time <= 2.5){

      drive.moveRobotRelative(0.0, 0.2, 0.0);
      shooter.spinIntake(0.5);
      shooter.spinFlywheel(0.0);

    }else if(2.5 < time && time <= 2.5){

      drive.moveRobotRelative(0.0, 0.0, 0.0);
      shooter.spinIntake(1.0);

    }else if(2.5 < time && time <= 4.5){

      drive.moveRobotRelative(0.0, (time-2.5)*5%2.0-1.0, 0.0);
      shooter.spinIntake(1.0);

    }else if(4.5 < time && time <= 6.6){

      drive.moveRobotAbsolute(0.0, 0.0, 0.45);
      shooter.spinIntake(0.0);

    }else if(6.6 < time && time <= 8.0){
      if(Math.abs(x)>2);

      System.out.println("Block 5");           

      drive.moveRobotAbsolute(0.0, 0.0, x*0.05);

    }else if(time <= 9.0){

      System.out.println("Block 6");

      shooter.setFlywheelSpeed(8900);
      drive.moveRobotAbsolute(0.0, 0.0, 0.0);

    }else if(time <= 11){

      System.out.println("Block 7");

      shooter.setFlywheelSpeed(8900);
      shooter.spinIndexer(1.0);
      drive.moveRobotAbsolute(0.0, 0.0, 0.0);

    }else{
      System.out.println("Block 8");

      shooter.spinFlywheel(0.0);
      shooter.spinIndexer(0.0);
      drive.moveRobotAbsolute(0.0, 0.0, 0.0);
    }

  }
}