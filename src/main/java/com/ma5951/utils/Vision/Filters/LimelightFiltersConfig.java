// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.Vision.Filters;

/** Add your docs here. */
public class LimelightFiltersConfig {

    private double ROBOT_LINEAR_SPEED_FOR_UPDATE;
    private double ROBOT_ANGULAR_SPEED_FOR_UPDATE;
    private boolean UPDATE_IN_AUTO;
    private double ODOMETRY_TO_VISION_DISTANCE;
    private double STUDERING_DISTANCE;

    public LimelightFiltersConfig(
        boolean UPDATE_IN_AUTO,
        double ROBOT_LINEAR_SPEED_FOR_UPDATE,
        double ROBOT_ANGULAR_SPEED_FOR_UPDATE,
        double ODOMETRY_TO_VISION_DISTANCE,
        double STUDERING_DISTANCE
    ) {
        this.UPDATE_IN_AUTO = UPDATE_IN_AUTO;
        this.ROBOT_LINEAR_SPEED_FOR_UPDATE = ROBOT_LINEAR_SPEED_FOR_UPDATE;
        this.ROBOT_ANGULAR_SPEED_FOR_UPDATE = ROBOT_ANGULAR_SPEED_FOR_UPDATE;
        this.ODOMETRY_TO_VISION_DISTANCE = ODOMETRY_TO_VISION_DISTANCE;
        this.STUDERING_DISTANCE = STUDERING_DISTANCE;
    }

    public boolean getUpdateInAuto() {
        return UPDATE_IN_AUTO;
    }

    public double getRobotLinearSpeedForUpdate() {
        return ROBOT_LINEAR_SPEED_FOR_UPDATE;
    }

    public double getRobotAngularSpeedForUpdate() {
        return ROBOT_ANGULAR_SPEED_FOR_UPDATE;
    }

    public double getOdometryToVisionDistance() {
        return ODOMETRY_TO_VISION_DISTANCE;
    }

    public double getStuderingDistance() {
        return STUDERING_DISTANCE;
    }


}
