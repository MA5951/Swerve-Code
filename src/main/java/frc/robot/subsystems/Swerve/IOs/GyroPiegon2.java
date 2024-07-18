// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve.IOs;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ma5951.utils.Logger.LoggedDouble;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.Swerve.Util.Gyro;


public class GyroPiegon2 extends Gyro{

    private String name;

    private Pigeon2 gyro;

    private StatusSignal<Double> gyroYaw;
    private StatusSignal<Double> gyroPitch;
    private StatusSignal<Double> gyroRoll;
    private StatusSignal<Double> gyroAccelX;
    private StatusSignal<Double> gyroAccelY;

    private LoggedDouble yaw;
    private LoggedDouble pitch;
    private LoggedDouble roll;
    private LoggedDouble accelX;
    private LoggedDouble accelY;
    
    public GyroPiegon2(String type, String canBus ,int id) {
        name = type;

        gyro = new Pigeon2(id, canBus);

        gyroYaw = gyro.getYaw();
        gyroPitch = gyro.getPitch();
        gyroRoll = gyro.getRoll();
        gyroAccelX = gyro.getAccelerationX();
        gyroAccelY = gyro.getAccelerationY();

        yaw = new LoggedDouble("/Swerve/" + name + "/Yaw");
        pitch = new LoggedDouble("/Swerve/" + name + "/Pitch");
        roll = new LoggedDouble("/Swerve/" + name + "/Roll");
        accelX = new LoggedDouble("/Swerve/" + name + "/Accel X");
        accelY = new LoggedDouble("/Swerve/" + name + "/Accel Y");
        
    }

    public void reset() {
        gyro.reset();
    }

    public double getYaw() {
        gyroYaw.refresh();
        return gyroYaw.getValueAsDouble();
    }

    public double getPitch() {
        gyroPitch.refresh();
        return gyroPitch.getValueAsDouble();
    }

    public double getRoll() {
        gyroRoll.refresh();
        return gyroRoll.getValueAsDouble();
    }

    public double getAccelX() {
        gyroAccelX.refresh();
        return gyroAccelX.getValueAsDouble();
    }

    public double getAccelY() {
        gyroAccelY.refresh();
        return gyroAccelY.getValueAsDouble();
    }

    public void update(ChassisSpeeds robotSpeeds) {
        yaw.update(getYaw());
        pitch.update(getPitch());
        roll.update(getRoll());
        accelX.update(getAccelX());
        accelY.update(getAccelY());

        
    }


}
