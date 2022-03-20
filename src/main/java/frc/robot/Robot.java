// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.ColorSensorV3;

import org.opencv.core.Mat;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kAutoOne = "Auton 1";
  private static final String kAutoTwo = "Auton 2";
  private static final String kAutoThree = "Auton 3";

  private static final String kBlue = "Blue";
  private static final String kRed = "Red";


  private String m_autoSelected;
  private final SendableChooser<String> m_auto_chooser = new SendableChooser<>();

  private boolean m_colorSelected;
  private final SendableChooser<String> m_color_chooser = new SendableChooser<>();


  XboxController driverController = new XboxController(0);              //Creates the driverController Obect
  XboxController opController = new XboxController(1);                 //Creates the opController Obect


  AHRS gyro = new AHRS();
  Shooter shooter = new Shooter(14, 13, 11, 15);
  Climber climber = new Climber(19, 18, 10);

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  SwerveModule swerveModules[] = new SwerveModule[]{     //Drive Motor, Rotation Motor, Rotation Encoder, Rotation Offset
    new SwerveModule(4, 3, 3, -104.5),           //Front Left
    new SwerveModule(2, 1, 0, -149.3),           //Front Right
    new SwerveModule(6, 5, 2, -101.6),           //Back Left
    new SwerveModule(8, 7, 1, 135.0)            //Back Right
  };

  Servo servo = new Servo(0);

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);



  SwerveDrive drive = new SwerveDrive(swerveModules, gyro);

  private final Timer m_timer = new Timer();     //Creates the timer obect.  

  Auton auton;

  double driveControllerLeftX;
  double driveControllerLeftY;

  double driveControllerRightX;

  boolean flywheelOn;
  double flywheelSpeed = 0.0;
  double indexTime;
  double currentSpeed;

  boolean flywheelOverCurrent;

  double time;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    CameraServer.startAutomaticCapture();


    m_auto_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_auto_chooser.addOption("Auton 1", kAutoOne);
    m_auto_chooser.addOption("Auton 2", kAutoTwo);
    m_auto_chooser.addOption("Auton 3", kAutoThree);

    
    m_color_chooser.setDefaultOption("Blue", kBlue);
    m_color_chooser.addOption("Red", kRed);

    SmartDashboard.putData("Auto choices", m_auto_chooser);
    SmartDashboard.putData("Color choices", m_color_chooser);


    gyro.reset();

    flywheelOn = false;
    flywheelSpeed = 0.0;

    indexTime = 0.0;
    
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
    m_timer.reset();
    m_timer.start();

    m_autoSelected = m_auto_chooser.getSelected();

    if(m_color_chooser.getSelected() == kBlue){
      m_colorSelected = true;
    }else{
      m_colorSelected = false;
    }

    System.out.println("Auto selected: " + m_autoSelected);
    System.out.println("Auto selected: " + m_autoSelected);


    gyro.reset();

    flywheelOn = false;
    flywheelSpeed = 0.0;

    switch (m_autoSelected) {

      case kAutoOne:
        // Put custom auto code here
        auton = new AutonOne(drive, shooter, gyro, servo, m_colorSelected, colorSensor);

        break;
      case kAutoTwo:
        // Put custom auto code here
        auton = new AutonTwo(drive, shooter, gyro, servo, m_colorSelected, colorSensor);

        break;
      case kAutoThree:
        // Put custom auto code here
        auton = new AutonThree(drive, shooter, gyro, servo, m_colorSelected, colorSensor);

        break;    
      case kDefaultAuto:
      default:
        // Put default auto code here

        break;
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    //read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);


    auton.run(time, x);
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

    flywheelOn = false;
    flywheelSpeed = 0.0;
    currentSpeed = 0.5;
    NetworkTableEntry pipeline = table.getEntry("getpipe");

    pipeline.forceSetString("Blue_Ball_2022");

    SmartDashboard.putNumber("Pre Pipeline", pipeline.getDouble(0.5));

    NetworkTableEntry pipeline = table.getEntry("pipeline");
    boolean rv = pipeline.setNumber(1.0);
    System.out.println(rv);

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    


    

    NetworkTableEntry ledMode = table.getEntry("ledMode");
    ledMode.setNumber(0);      // 1 = force off,   0 = follow pipeline

    //read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    //SmartDashboard.putNumber("Pipeline", pipeline.getDouble(0.5));
    SmartDashboard.putNumber("Pipeline", table.getEntry("getpipe").getDouble(0.5));

    driveControllerLeftX = driverController.getLeftX();
    driveControllerLeftY = -driverController.getLeftY();
    driveControllerRightX = driverController.getRightX();

    if(Math.abs(driveControllerLeftX)<0.05){
      driveControllerLeftX = 0;
    }
    if(Math.abs(driveControllerLeftY)<0.05){
      driveControllerLeftY = 0;
    }
    if(Math.abs(driveControllerRightX)<0.05){
      driveControllerRightX = 0;
    }



    if(driverController.getRightBumperPressed()){
      currentSpeed = currentSpeed + 0.25;  
    }

    if(driverController.getLeftBumperPressed()){
      currentSpeed = currentSpeed - 0.25;  
    }

    currentSpeed = MathUtil.clamp(currentSpeed, 0.25, 0.75);

    driveControllerLeftX = driveControllerLeftX*currentSpeed;
    driveControllerLeftY = driveControllerLeftY*currentSpeed;

    driveControllerRightX = driveControllerRightX*0.5;

    SmartDashboard.putNumber("Current Speed", currentSpeed);

    SmartDashboard.putNumber("Drive Left X", driveControllerLeftX);
    SmartDashboard.putNumber("Drive Left Y", driveControllerLeftY);

    SmartDashboard.putNumber("Limelight x", x);

    if(driverController.getLeftTriggerAxis() > 0.1 && driveControllerRightX == 0 && Math.abs(x) > 3){

      driveControllerRightX = x/10;

    }

    drive.moveRobotAbsolute(driveControllerLeftX, driveControllerLeftY, driveControllerRightX);

    if(driverController.getYButtonPressed()){
      gyro.reset();
    }



    if(opController.getRightTriggerAxis() > 0.1){
      shooter.spinIntake(opController.getRightTriggerAxis());

    }else if(driverController.getRightTriggerAxis() > 0.1){
      shooter.spinIntake(driverController.getRightTriggerAxis());

    }else if(opController.getRightBumper()){
      shooter.spinIntake(-1.0);
    }else{
      shooter.spinIntake(0.0);
      SmartDashboard.putNumber("Intake Power", 0.0);
    }

    if(opController.getLeftBumperPressed()){
      indexTime = 0.4;
    }

    if(indexTime > 0.0){
      shooter.spinIndexer(1.0);
      indexTime = indexTime - 0.02;

    }else if(opController.getLeftTriggerAxis() > 0.1){
      shooter.spinIndexer(opController.getLeftTriggerAxis());

    }else if(opController.getRightBumper()){
      shooter.spinIndexer(-1.0);
    }else{
      shooter.spinIndexer(0.0);
      SmartDashboard.putNumber("Index Power", 0.0);

    }

    if(opController.getStartButtonPressed()){
      flywheelOn = !flywheelOn;
      flywheelOverCurrent = false;
    }

    if(flywheelOverCurrent == false){
      flywheelOverCurrent = shooter.isCurrentHigh();
    }

    /*if(flywheelOverCurrent){
      flywheelOn = false;
    }*/

    SmartDashboard.putBoolean("Flywheel OverCurrented", flywheelOverCurrent);

    if(flywheelOn){

      if(opController.getBButton()){
        flywheelSpeed = 6000.0;
      }else{
        flywheelSpeed = 9000.0;

      }

      shooter.setFlywheelSpeed(flywheelSpeed);

    }else if(opController.getBackButton()){
      flywheelSpeed = -10.0;
      shooter.spinFlywheel(-1.0);

    }else{
      flywheelSpeed = 0.0;
      shooter.spinFlywheel(0.0);

    }


    shooter.getFlywheelSpeed();

    shooter.getFlywheelPosition();

    //Climber Controls

    climber.getExtensionPositions();

    if(opController.getAButton()){  //Manual Climber Controls

      if(opController.getPOV() == 0){
        climber.moveClimberManual(0.75);
      }else if(opController.getPOV()==180){
        climber.moveClimberManual(-0.75);
      }else if(opController.getPOV()==90){
        climber.moveRight(-0.75);
      }else if(opController.getPOV()==270){
        climber.moveLeft(-0.75);
      }else{
        climber.moveClimberManual(0.0);
      }

      if(Math.abs(opController.getLeftX()) > 0.1){

        climber.tiltClimberManual(opController.getLeftX());

      }else{

        climber.tiltClimberManual(0.0);

      }

    }/*else if(opController.getYButton()){  //"Automatic" Climber Controls

      if(opController.getPOV() == 0){
        
      }else if(opController.getPOV()==180){

      }else{
        
      }

    }*/else{

      if(Math.abs(opController.getLeftY())>0.2){
        climber.moveLeft(opController.getLeftY()*-1.0);
      }else{
        climber.moveLeft(0.0);
      }

      if(Math.abs(opController.getRightY())>0.2){
        climber.moveRight(opController.getRightY()*-1.0);
      }else{
        climber.moveRight(0.0);
      }
    }

    if(opController.getXButton()){
      servo.set(0.9);
    }else{
      servo.set(0.0);
    }

    Color detectedColor = colorSensor.getColor();

    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("IR", colorSensor.getIR());

    SmartDashboard.putNumber("Proximity", colorSensor.getProximity());



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