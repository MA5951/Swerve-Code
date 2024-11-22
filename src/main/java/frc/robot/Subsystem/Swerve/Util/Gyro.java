package frc.robot.Subsystem.Swerve.Util;


public interface Gyro {

    void reset();

    double getPitch();

    double getRoll();

    double getYaw();

    double getAbsYaw();

    double getAccelX();

    double getAccelY();

    GyroData update();

}

