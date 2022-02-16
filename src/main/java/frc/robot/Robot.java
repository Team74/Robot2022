// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();


  XboxController driverController = new XboxController(0);              //Creates the driverController Obect

  AHRS gyro = new AHRS();

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  SwerveModule swerveModules[] = new SwerveModule[]{     //Drive Motor, Rotation Motor, Rotation Encoder, Rotation Offset
    //new SwerveModule(5,14,0,165.76),           //Front Left
    //new SwerveModule(12,15,1,175.15),           //Front Right
    //new SwerveModule(11,3,2,-1.58),           //Back Left
    //new SwerveModule(8, 7, 0, 280.8)            //Back Right
  };


  SwerveModule swervePod = new SwerveModule(8, 7, 0, 280.8);           //Back Right


  SwerveDrive drive = new SwerveDrive(swerveModules, gyro);

  double driveControllerLeftX;
  double driveControllerLeftY;

  double desiredSwerveAngle;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    //read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    driveControllerLeftX = driverController.getLeftX();
    driveControllerLeftY = -driverController.getLeftY();

    
    if(driveControllerLeftY>0.1){

      desiredSwerveAngle = Math.atan(driveControllerLeftX / driveControllerLeftY);
      desiredSwerveAngle = desiredSwerveAngle*(180.0/Math.PI);

      if(desiredSwerveAngle<0.0){
        desiredSwerveAngle = desiredSwerveAngle+360.0;
      }

    } else if(driveControllerLeftY<-0.1){

      desiredSwerveAngle = Math.atan(driveControllerLeftX / driveControllerLeftY);
      desiredSwerveAngle = desiredSwerveAngle*(180.0/Math.PI);
      desiredSwerveAngle = desiredSwerveAngle+180.0;

    }else if(driveControllerLeftX>0.1){

      desiredSwerveAngle = 90.0;

    } else if(driveControllerLeftX<-0.1){

      desiredSwerveAngle = 270.0;

    } else {

      if(swervePod.getSwerveAngle() >= 270.0 || swervePod.getSwerveAngle() <= 90.0) desiredSwerveAngle = 0.0; // pavlo's code
      else desiredSwerveAngle = 180.0;
      //desiredSwerveAngle = 0.0;

    }


    //swervePod.setDriveMotor(-Math.sqrt(driveControllerLeftX*driveControllerLeftX + driveControllerLeftY*driveControllerLeftY));
    swervePod.GoToAngle(desiredSwerveAngle);

    SmartDashboard.putNumber("Drive Left X", driveControllerLeftX);
    SmartDashboard.putNumber("Drive Left Y", driveControllerLeftY);

    SmartDashboard.putNumber("Desired Swerve Angle", desiredSwerveAngle);

    SmartDashboard.putNumber("Robot Angle", drive.getGyroAngle());
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}



//test commit