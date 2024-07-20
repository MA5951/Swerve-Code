// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import com.ma5951.utils.Logger.LoggedPose2d;

import edu.wpi.first.math.kinematics.SwerveDriveOdometry;

/** Add your docs here. */
public class SwerveOdometry {

    private SwerveDriveOdometry odometry;
    private LoggedPose2d robotPose;

    public SwerveOdometry() {
        robotPose = new LoggedPose2d("/Swerve/SwerveOdometry/Robot Pose");
        odometry = new SwerveDriveOdometry(SwerveConstants.kinematics, SwerveSubsystem.getInstance().getRotation2d(), SwerveSubsystem.getInstance().getSwerveModulePositions());
    }

    public void update() {
        odometry.update(SwerveSubsystem.getInstance().getRotation2d(), SwerveSubsystem.getInstance().getSwerveModulePositions());
        robotPose.update(odometry.getPoseMeters());
    }
}
