// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ma5951.utils.Logger.LoggedDouble;

/** Add your docs here. */
public class CollisionDtector {

    private Pigeon2 gyro;
    private LoggedDouble yaw;
    private LoggedDouble pitch;
    private LoggedDouble roll;
    private LoggedDouble accelX;
    private LoggedDouble accelY;
    private LoggedDouble accelZ;


    public CollisionDtector(Pigeon2 device) {
        gyro = device;
        yaw = new LoggedDouble("/Swerve/Collision Detector/Yaw");
        pitch = new LoggedDouble("/Swerve/Collision Detector/Pitch");
        roll = new LoggedDouble("/Swerve/Collision Detector/Roll");
        accelX = new LoggedDouble("/Swerve/Collision Detector/Accel X");
        accelY = new LoggedDouble("/Swerve/Collision Detector/Accel Y");
        accelZ = new LoggedDouble("/Swerve/Collision Detector/Accel Z");
    }

    public void update() {
        yaw.update(gyro.getYaw().getValueAsDouble());
        pitch.update(gyro.getPitch().getValueAsDouble());
        roll.update(gyro.getRoll().getValueAsDouble());
        accelX.update(gyro.getAccelerationX().getValueAsDouble());
        accelY.update(gyro.getAccelerationY().getValueAsDouble());
        accelZ.update(gyro.getAccelerationZ().getValueAsDouble());
    }



}
