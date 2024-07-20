package frc.robot.subsystems.Swerve.Util;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface Gyro {

    void reset();

    double getPitch();

    double getRoll();

    double getYaw();

    double getAccelX();

    double getAccelY();

    void update(ChassisSpeeds robotSpeeds);

}

