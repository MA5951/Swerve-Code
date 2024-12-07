
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
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;

public class LimelightFilters {
    private Limelight3G limelight;
    private LimelightFiltersConfig config;
    private Supplier<Pose2d> robotPoSupplier;
    private Supplier<ChassisSpeeds> robotSpeedsSupplier;
    private Translation2d robotPose;
    private ChassisSpeeds robotSpeeds;

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
        return inVelocityFilter() && inField() && notInFieldObstacles() && inOdometryRange() && ;
    }

    private boolean inVelocityFilter() {
        robotSpeeds = robotSpeedsSupplier.get();
        return robotSpeeds.vxMetersPerSecond <= config.robotUpdateSpeed.vxMetersPerSecond &&
        robotSpeeds.vyMetersPerSecond <= config.robotUpdateSpeed.vyMetersPerSecond &&
        robotSpeeds.omegaRadiansPerSecond <= config.robotUpdateSpeed.omegaRadiansPerSecond;
    }

    private boolean inField() {
        return config.fieldRectangle.contains(robotPoSupplier.get().getTranslation());
    }

    private boolean notInFieldObstacles() {
        if (config.fieldObstaclesRectangles != null) {
            robotPose = robotPoSupplier.get().getTranslation();
            for (Rectangle2d obstacles: config.fieldObstaclesRectangles) {
                if (obstacles.contains(robotPose)) {
                    return false;
                }
            }
        }

        return true;
    }

    private boolean inOdometryRange() {
        if ((config.visionToOdometryInTeleop && DriverStation.isTeleop()) || DriverStation.isAutonomous()) {
            robotPose = robotPoSupplier.get().getTranslation();
            return robotPose.getDistance(limelight.getEstimatedPose().pose.getTranslation()) < config.visionToOdometry;
        }
        return true; 
    }
    
    private boolean shouldUpdateInAuto() {
        if ((config.updateInAuto && DriverStation.isAutonomous()) || DriverStation.) {
            return true;
        }

        return false;
    }
}
