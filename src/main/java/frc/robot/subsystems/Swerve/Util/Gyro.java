package frc.robot.subsystems.Swerve.Util;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public abstract class Gyro {
    
    public abstract void reset();

    public abstract double getPitch();

    public abstract double getRoll();

    public abstract double getYaw();

    public abstract double getAccelX();

    public abstract double getAccelY();

    public abstract void update(ChassisSpeeds robotSpeeds);

    
}
