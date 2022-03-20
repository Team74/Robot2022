// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class SwerveDrive {

    SwerveModule[] swerveModules;
    SwerveDriveKinematics driveKinematics;
    ChassisSpeeds robotSpeed;
    AHRS gyro;
    boolean moduleStateInitialized = false;

    SwerveModuleState[] moduleStates;

    public SwerveDrive(SwerveModule[] module, AHRS gyro){
        swerveModules = module; 

        driveKinematics = new SwerveDriveKinematics(
            new Translation2d(0.38, 0.235),     //Front Left
            new Translation2d(0.38, -0.235),    //Front Right 
            new Translation2d(-0.38, 0.235),    //Back Left 
            new Translation2d(-0.38, -0.235)    //Back Right
        ); 

        this.gyro = gyro;
    }



    public double getGyroAngle(){

        double gyroAngle = gyro.getAngle();

        gyroAngle = gyroAngle%360;

        if(gyroAngle<0){
            gyroAngle = gyroAngle+360;
        }


        SmartDashboard.putNumber("Robot Angle", gyroAngle);

        return gyroAngle;
    }


    public void moveRobotAbsolute(double xVelocity, double yVelocity, double rotationSpeed){
        boolean freezeWheelPosition = (xVelocity == 0 && yVelocity == 0 && rotationSpeed == 0);

        robotSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(yVelocity, -xVelocity, -rotationSpeed, Rotation2d.fromDegrees(-getGyroAngle()));

        if (!freezeWheelPosition || !moduleStateInitialized) {
            moduleStates = driveKinematics.toSwerveModuleStates(robotSpeed);
            moduleStateInitialized = true;
        }

        for(int index = 0; index<4; index++){
            SwerveModuleState newModuleState = SwerveModuleState.optimize(moduleStates[index], new Rotation2d(swerveModules[index].getSwerveAngle()*Math.PI/180));
            swerveModules[index].setDriveMotor(freezeWheelPosition ? 0 : newModuleState.speedMetersPerSecond);
            swerveModules[index].GoToAngle(newModuleState.angle.getDegrees());
        }
    }

    public void moveRobotRelative(double xVelocity, double yVelocity, double rotationSpeed){

        robotSpeed = new ChassisSpeeds(yVelocity, -xVelocity, -rotationSpeed);

        moduleStates = driveKinematics.toSwerveModuleStates(robotSpeed);

        for(int index = 0; index<4; index++){
            SwerveModuleState newModuleState = SwerveModuleState.optimize(moduleStates[index], new Rotation2d(swerveModules[index].getSwerveAngle()*Math.PI/180));
            swerveModules[index].setDriveMotor(newModuleState.speedMetersPerSecond);
            swerveModules[index].GoToAngle(newModuleState.angle.getDegrees());
        }

    }

    public void stopSwerveDrive(){
        ChassisSpeeds robotSpeed = new ChassisSpeeds(
            0.0,
            0.0,
            0.0
        );

        moduleStates = driveKinematics.toSwerveModuleStates(robotSpeed);

        for(int index = 0; index<4; index++){
            SwerveModuleState newModuleState = SwerveModuleState.optimize(moduleStates[index], new Rotation2d(swerveModules[index].getSwerveAngle()*Math.PI/180));
            swerveModules[index].setDriveMotor(newModuleState.speedMetersPerSecond);
            swerveModules[index].GoToAngle(newModuleState.angle.getDegrees());
        }
        
    }

}
