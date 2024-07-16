package frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public abstract class SwerveModule {
    public abstract double getAbsoluteEncoderPosition();

    public abstract double getDrivePosition();

    public abstract double getTurningPosition();

    public abstract double getDriveVelocity();

    public abstract void resetEncoders();

    public abstract void resetSteer();

    public abstract void resetDrive();

    public abstract void turningMotorSetPower(double power);

    public abstract void driveMotorSetPower(double power);

    public abstract void turningMotorSetVoltage(double volt);

    public abstract void driveMotorSetVoltage(double volt);

    public abstract void turningUsingPID(double setPoint);

    public abstract void driveUsingPID(double setPoint);

    public abstract void setInvertedTurning(Boolean mode);

    public abstract double getCurrent();

    public abstract void update();

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                getDrivePosition(), new Rotation2d(Math.toRadians(getTurningPosition())));
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                getDriveVelocity(), new Rotation2d(Math.toRadians(getTurningPosition())));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        turningUsingPID(desiredState.angle.getDegrees());
        driveUsingPID(desiredState.speedMetersPerSecond);
    }


}