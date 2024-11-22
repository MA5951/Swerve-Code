// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.PoseEstimation;


import com.ma5951.utils.Logger.LoggedBool;
import com.ma5951.utils.Logger.LoggedPose2d;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Subsystem.Swerve.SwerveConstants;
import frc.robot.Subsystem.Swerve.SwerveSubsystem;

/** Add your docs here. */
public class PoseEstimator {
    private static PoseEstimator poseEstimator;

    private SwerveDrivePoseEstimator robotPoseEstimator;
    private Vision vision;
    private SwerveSubsystem swerve = SwerveSubsystem.getInstance();
    private int updateNum = 15;
    private int firstUpdate = 0;
    private boolean shouldUpdate = false;

    private LoggedPose2d estimatedRobotPose;
    private LoggedBool odometryUpdateConstrains;
    private LoggedBool visionUpdateConstrains;
    private LoggedBool updatedVisionLog;


    public PoseEstimator() {
        vision = Vision.getInstance();
        robotPoseEstimator = new SwerveDrivePoseEstimator(SwerveConstants.kinematics , swerve.getRotation2d() , 
        new  SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
        },
        new Pose2d(),
        PoseEstimatorConstants.ODOMETRY_DEVS,//Oodmetry Devs
        PoseEstimatorConstants.VISION_DEVS);//Vision Devs

        estimatedRobotPose = new LoggedPose2d("/Pose Estimator/Estimated Robot Pose");
        odometryUpdateConstrains = new LoggedBool("/Pose Estimator/Odometry Update Constrains");
        visionUpdateConstrains = new LoggedBool("/Pose Estimator/Vision Update Constrains");
        updatedVisionLog = new LoggedBool("/Pose Estimator/Vision Update");
    }

    public void resetPose(Pose2d pose) {
        robotPoseEstimator.resetPosition(swerve.getRotation2d() , swerve.getSwerveModulePositions() ,pose) ;
        //robotPoseEstimator.reset
    }
    
    public void updateOdometry(SwerveModulePosition[] wheelPositions , Rotation2d yaw , double timestemp) {
        //robotPoseEstimator.updateWithTime(Timer.getFPGATimestamp(),swerve.getRotation2d(), swerve.getSwerveModulePositions());
        robotPoseEstimator.updateWithTime(timestemp,yaw, wheelPositions);
    }


    public void updateVision() {
        shouldUpdate = false;
        
        if (PoseEstimatorConstants.VISION_UPDATE_CONSTRAINS.get()) {
            if (vision.getEstiman().getTranslation().getX() != 0 ) {
                if (firstUpdate < 10) {
                    updateNum = 0;
                    shouldUpdate = true;
                } else {
                    // if (vision.getEstiman().getTranslation().getDistance(getEstimatedRobotPose().getTranslation())
                    // < PoseEstimatorConstants.VISION_TO_ODOMETRY_DIFRANCE ) {
                    //     updateNum = 0;
                    //     shouldUpdate = true;
                    // }
                    updateNum = 0;
                    shouldUpdate = true;
                }
            } 
        }

        if (updateNum < 10 && shouldUpdate) {
            updateNum++;
            if (firstUpdate < 15) {
                firstUpdate++;
            }
            robotPoseEstimator.addVisionMeasurement(vision.getEstiman(), Timer.getFPGATimestamp());
            updatedVisionLog.update(true);
        } else {
            updatedVisionLog.update(false);
        }
    }

    public Pose2d getEstimatedRobotPose() {
        return robotPoseEstimator.getEstimatedPosition();
    }


    public void update() {
        //updateOdometry();
        updateVision();
        estimatedRobotPose.update(getEstimatedRobotPose());
        odometryUpdateConstrains.update(PoseEstimatorConstants.ODOMETRY_UPDATE_CONSTRAINS.get());
        visionUpdateConstrains.update(PoseEstimatorConstants.VISION_UPDATE_CONSTRAINS.get());

        
    }


    public static PoseEstimator getInstance() {
        if (poseEstimator == null) {
          poseEstimator = new PoseEstimator();
        }
        return poseEstimator;
        }



}
