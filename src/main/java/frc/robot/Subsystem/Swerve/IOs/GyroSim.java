// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Swerve.IOs;

import org.opencv.core.RotatedRect;

import com.ma5951.utils.Logger.LoggedDouble;
import com.ma5951.utils.Utils.ConvUtil;
import com.ma5951.utils.Utils.DriverStationUtil;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Subsystem.Swerve.Util.Gyro;
import frc.robot.Subsystem.Swerve.Util.GyroData;

public class GyroSim implements Gyro{

    private LoggedDouble yaw;
    private LoggedDouble pitch;
    private LoggedDouble roll;
    private LoggedDouble accelX;
    private LoggedDouble accelY;

    private GyroData gyroData;

    private double yawValue;

    public GyroSim(String type) {

        yaw = new LoggedDouble("/Swerve/" + type + "/Yaw");
        pitch = new LoggedDouble("/Swerve/" + type + "/Pitch");
        roll = new LoggedDouble("/Swerve/" + type + "/Roll");
        accelX = new LoggedDouble("/Swerve/" + type + "/Accel X");
        accelY = new LoggedDouble("/Swerve/" + type + "/Accel Y");

        gyroData = new GyroData();

    }

    public void reset() {
    }

    public double getYaw() {
        return yawValue;
    }

    public double getAbsYaw() {
        return getYaw();
    }

    public double getPitch() {
        return 0;
    }

    public double getRoll() {
        return 0;
    }

    public double getAccelX() {
        return 0;
    }

    public double getAccelY() {
        return 0;
    }

    public GyroData update(ChassisSpeeds robotSpeeds) {
        gyroData.updateData(
            0d,
            0d,
            0d,
            yawValue += ConvUtil.RadiansToDegrees(robotSpeeds.omegaRadiansPerSecond) * 0.02,
            0d,
            0d,
            0d,
            0d,
            yawValue,
            new Rotation2d[1]
        );
        
        yaw.update(yawValue);

        return gyroData;
    }

    




}
