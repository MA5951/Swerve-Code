// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve.IOs;

import com.ma5951.utils.Logger.LoggedDouble;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Swerve.Util.Gyro;

public class GyroSim extends Gyro{

    private LoggedDouble yaw;
    private LoggedDouble pitch;
    private LoggedDouble roll;
    private LoggedDouble accelX;
    private LoggedDouble accelY;

    private double yawValue;

    public GyroSim(String type) {

        yaw = new LoggedDouble("/Swerve/" + type + "/Yaw");
        pitch = new LoggedDouble("/Swerve/" + type + "/Pitch");
        roll = new LoggedDouble("/Swerve/" + type + "/Roll");
        accelX = new LoggedDouble("/Swerve/" + type + "/Accel X");
        accelY = new LoggedDouble("/Swerve/" + type + "/Accel Y");

    }


    public void reset() {
    }

    public double getYaw() {
        return yawValue;
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

    public void update(ChassisSpeeds robotSpeeds) {
        yaw.update(getYaw());
        pitch.update(getPitch());
        roll.update(getRoll());
        accelX.update(getAccelX());
        accelY.update(getAccelY());

        yawValue += Units.radiansToDegrees(robotSpeeds.omegaRadiansPerSecond * 0.02);
    }




}
