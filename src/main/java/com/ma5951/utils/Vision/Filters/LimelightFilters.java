// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Linear Velocity
//Angular Velocity
//In field and not in specefied boxes
//Flickering pose 1. stationery flickring 2. distance from odometry (only auto)
//not deafult pose


//Is Flickering detection 

package com.ma5951.utils.Vision.Filters;

import java.util.function.Supplier;

import com.ma5951.utils.Vision.Limelights.Limelight3G;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Add your docs here. */
public class LimelightFilters {
    private Limelight3G limelight;
    private LimelightFiltersConfig config;
    private Supplier<Pose2d> robotPoSupplier;
    private Supplier<ChassisSpeeds> robotSpeedsSupplier;

    public LimelightFilters(Limelight3G camera , LimelightFiltersConfig configuration , Supplier<Pose2d> robotPose , Supplier<ChassisSpeeds> robotSpeeds) {
        limelight = camera;
        config = configuration;
        robotPoSupplier = robotPose;
        robotSpeedsSupplier = robotSpeeds;
    }

    public boolean isValidForReset() {
        return limelight.isTarget();
    }

    public boolean isValidForUpdate() {
        return linearVelocityFilter() &&;
    }

    public boolean linearVelocityFilter() {
        return robotSpeedsSupplier.get().vxMetersPerSecond <= config.robotUpdateSpeed.vxMetersPerSecond &&
        robotSpeedsSupplier.get().vyMetersPerSecond <= config.robotUpdateSpeed.vyMetersPerSecond &&
        robotSpeedsSupplier.get().omegaRadiansPerSecond <= config.robotUpdateSpeed.omegaRadiansPerSecond;
    }

    public boolean inField() {
        return config.fieldRectangle.
    }

    
}
