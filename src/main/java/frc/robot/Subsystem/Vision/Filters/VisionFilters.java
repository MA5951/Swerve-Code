
//Linear Velocity
//Angular Velocity
//In field and not in specefied boxes
//Flickering pose 1. stationery flickring 2. distance from odometry (only auto)
//not deafult pose


//Is Flickering detection 

package frc.robot.Subsystem.Vision.Filters;

import java.util.function.Supplier;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Subsystem.Vision.VisionIO;

public class VisionFilters {
    private VisionIO visionIO;
    private VisionFiltersConfig config;
    private Supplier<Pose2d> robotPoSupplier;
    private Supplier<ChassisSpeeds> robotSpeedsSupplier;
    private Translation2d robotPose;
    private ChassisSpeeds robotSpeeds;
    private Pose2d deafultPose = new Pose2d();
    private Supplier<Double> robotVelocityVectorSupplier;

    public VisionFilters(VisionIO VisionIO , VisionFiltersConfig configuration , Supplier<Pose2d> robotPose , Supplier<ChassisSpeeds> robotSpeeds , Supplier<Double> robotVelocityVector) {
        visionIO = VisionIO;
        config = configuration;
        robotPoSupplier = robotPose;
        robotSpeedsSupplier = robotSpeeds;
        robotVelocityVectorSupplier = robotVelocityVector;
    }

    public boolean isValidForReset() {
        return visionIO.getTargetCount() > 1 && visionIO.getRawFiducial().distToCamera < 2 && visionIO.getRawFiducial().ambiguity < 0.5 && robotSpeeds.vxMetersPerSecond < 0.05 &&
        robotSpeeds.vyMetersPerSecond < 0.05 &&
        robotSpeeds.omegaRadiansPerSecond < 0.05;
    }

    public boolean isValidForUpdate(Pose2d visionPose2d) {
        return inVelocityFilter() && inField() && notInFieldObstacles() && inOdometryRange(visionPose2d) && shouldUpdateByRobotState() && notDeafultPose() && isVisionMatchingVelocity(visionPose2d);
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

    private boolean inOdometryRange(Pose2d visiPose2d) {
        if ((config.visionToOdometryInTeleop && DriverStation.isTeleop()) || DriverStation.isAutonomous()) {
            robotPose = robotPoSupplier.get().getTranslation();
            return robotPose.getDistance(visiPose2d.getTranslation()) < config.visionToOdometry;
        }
        return true; 
    }

    private boolean isVisionMatchingVelocity(Pose2d visionPose2d) {
        return (robotPoSupplier.get().getTranslation().getDistance(visionPose2d.getTranslation()) <= robotVelocityVectorSupplier.get() * 0.02 + 0.35) ;
    }
    
    private boolean shouldUpdateByRobotState() {
        if ((config.updateInAuto && DriverStation.isAutonomous()) || !DriverStation.isAutonomous()) {
            return true;
        }

        return false;
    }

    private boolean notDeafultPose() {
        return visionIO.getEstimatedPose().pose !=deafultPose;
    }

}
