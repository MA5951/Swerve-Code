// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Swerve;

import com.ma5951.utils.Logger.LoggedDouble;

import frc.robot.Subsystem.Swerve.Util.Gyro;

/** Add your docs here. */
public class CollisionDtector {

    private Gyro gyro;
    private LoggedDouble yaw;
    private LoggedDouble pitch;
    private LoggedDouble roll;
    private LoggedDouble accelX;
    private LoggedDouble accelY;


    public CollisionDtector(Gyro device) {
        gyro = device;
        yaw = new LoggedDouble("/Swerve/Collision Detector/Yaw");
        pitch = new LoggedDouble("/Swerve/Collision Detector/Pitch");
        roll = new LoggedDouble("/Swerve/Collision Detector/Roll");
        accelX = new LoggedDouble("/Swerve/Collision Detector/Accel X");
        accelY = new LoggedDouble("/Swerve/Collision Detector/Accel Y");
    }

    public void update() {
        yaw.update(gyro.getYaw());
        pitch.update(gyro.getPitch());
        roll.update(gyro.getRoll());
        accelX.update(gyro.getAccelX());
        accelY.update(gyro.getAccelY());
    }



}
