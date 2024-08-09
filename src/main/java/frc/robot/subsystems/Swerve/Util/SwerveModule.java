package frc.robot.subsystems.Swerve.Util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModule {

    //Return degrees
    double getAbsoluteEncoderPosition();

    //Returns distance in meters
    double getDrivePosition();

    //Return position in degrees
    double getTurningPosition();

    //Return meters per second
    double getDriveVelocity();

    //Between 1 to -1
    void turningMotorSetPower(double power);

    //Between 1 to -1
    void driveMotorSetPower(double power);

    //Between 0 to 12
    void turningMotorSetVoltage(double volt);

    //Between 0 to 12
    void driveMotorSetVoltage(double volt);

    //Turns in degrees
    void turningUsingPID(double setPoint);

    //Drives in meters per second
    void driveUsingPID(double setPoint);

    void setNeutralModeDrive(Boolean isBrake);

    void setNeutralModeTurn(Boolean isBrake);

    void resetSteer();

    void update();

    default SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                getDrivePosition(), new Rotation2d(Math.toRadians(getTurningPosition())));
    }

    default SwerveModuleState getState() {
        return new SwerveModuleState(
                getDriveVelocity(), new Rotation2d(Math.toRadians(getTurningPosition())));
    }

    default void setDesiredState(SwerveModuleState desiredState) {
        driveUsingPID(desiredState.speedMetersPerSecond);
        turningUsingPID(desiredState.angle.getRadians());
    }
}
