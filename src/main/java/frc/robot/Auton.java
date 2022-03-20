package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Servo;

public class Auton {
    protected SwerveDrive drive;
    protected Shooter shooter;
    protected AHRS gyro;
    protected boolean colorIsBlue;
    protected ColorSensorV3 colorSensor;
    protected Servo servo;
    protected NetworkTable table;

    public Auton(SwerveDrive drive, Shooter shooter, AHRS gyro, Servo servo, boolean colorIsBlue, ColorSensorV3 colorSensor){
        this.drive = drive;
        this.shooter = shooter;
        this.gyro = gyro;
        this.colorIsBlue = colorIsBlue;
        this.colorSensor = colorSensor;
        this.servo = servo;

        table = NetworkTableInstance.getDefault().getTable("limelight");

    }

    public void run(double time, double x){
        // Runs whatevet Auton you have selected
    }
}