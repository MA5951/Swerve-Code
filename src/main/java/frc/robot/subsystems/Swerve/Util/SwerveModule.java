package frc.robot.subsystems.Swerve.Util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public abstract class SwerveModule {
    
    //Return degrees
    public abstract double getAbsoluteEncoderPosition();

    //Returns distance in meters
    public abstract double getDrivePosition();

    //Return position in degrees
    public abstract double getTurningPosition();

    //Return meters per secound
    public abstract double getDriveVelocity();

    //Between 1 to -1
    public abstract void turningMotorSetPower(double power);

    //Between 1 to -1
    public abstract void driveMotorSetPower(double power);

    //Between 0 to 12
    public abstract void turningMotorSetVoltage(double volt);

    //Between 0 to 12
    public abstract void driveMotorSetVoltage(double volt);

    //Turns in degrees
    public abstract void turningUsingPID(double setPoint);

    //Drives in meter per secound
    public abstract void driveUsingPID(double setPoint);

    public abstract void setNutralModeDrive(Boolean isBrake);

    public abstract void setNutralModeTurn(Boolean isBrake);

    public abstract void update();

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                getDrivePosition(), new Rotation2d(Math.toRadians(getTurningPosition())));
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                getDriveVelocity(), new Rotation2d(Math.toRadians(getTurningPosition())));
    };

    public void setDesiredState(SwerveModuleState desiredState) {
        driveUsingPID(desiredState.speedMetersPerSecond);
        turningUsingPID(desiredState.angle.getDegrees());
    };


}