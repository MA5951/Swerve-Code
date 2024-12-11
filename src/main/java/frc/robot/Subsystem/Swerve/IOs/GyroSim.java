// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Swerve.IOs;


import org.ironmaple.simulation.drivesims.GyroSimulation;

import com.ma5951.utils.Logger.LoggedDouble;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Subsystem.Swerve.SwerveConstants;
import frc.robot.Subsystem.Swerve.Util.Gyro;
import frc.robot.Subsystem.Swerve.Util.GyroData;

public class GyroSim implements Gyro{

    private GyroSimulation gyroSimulation = SwerveConstants.SWERVE_DRIVE_SIMULATION.getGyroSimulation();
    private LoggedDouble yaw;

    private GyroData gyroData;

    private double yawValue;

    public GyroSim(String type) {

        yaw = new LoggedDouble("/Swerve/" + type + "/Yaw");

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
            gyroSimulation.getGyroReading().getDegrees(),
            0d,
            0d,
            0d,
            0d,
            gyroSimulation.getGyroReading().getDegrees(),
            gyroSimulation.getCachedGyroReadings() 
        );
        
        yaw.update(gyroSimulation.getGyroReading().getDegrees());

        return gyroData;
    }

    




}
