// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Vision;

import com.ma5951.utils.Logger.LoggedBool;
import com.ma5951.utils.Logger.LoggedDouble;
import com.ma5951.utils.Logger.LoggedInt;
import com.ma5951.utils.Logger.LoggedPose2d;
import com.ma5951.utils.Vision.Limelights.LimelightHelpers.PoseEstimate;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystem.PoseEstimation.PoseEstimator;
import frc.robot.Subsystem.Swerve.SwerveSubsystem;
import frc.robot.Subsystem.Vision.Filters.VisionFilters;

public class Vision extends SubsystemBase {
  private static Vision vision;

  private VisionIO visionIO = VisionConstants.getVisionIO();

  private VisionFilters visionFilters = new VisionFilters(visionIO, VisionConstants.FILTERS_CONFIG, () -> PoseEstimator.getInstance().getEstimatedRobotPose(), () -> SwerveSubsystem.getInstance().getRobotRelativeSpeeds()
  , () -> SwerveSubsystem.getInstance().getVelocityVector());

  private LoggedPose2d visionPose2dLog;
  private LoggedDouble tXLog;
  private LoggedDouble tYLog;
  private LoggedBool hasTargetLog;
  private LoggedInt targetCountLog;
  private LoggedBool isValidLog;
  private LoggedBool isValidForResetLog;
  private PoseEstimate visionPoseEstimate;
  private boolean isUpdateForOdometry;
  private boolean isUpdateGyro;
  private boolean didUpdatedGyro;

  public Vision() {
    visionPose2dLog = new LoggedPose2d("/Subsystems/Vision/Vision Pose");
    tXLog = new LoggedDouble("/Subsystems/Vision/Tx");
    tYLog = new LoggedDouble("/Subsystems/Vision/Ty");
    hasTargetLog = new LoggedBool("/Subsystems/Vision/Has Target");
    targetCountLog = new LoggedInt("/Subsystems/Vision/Target Count");
    isValidLog = new LoggedBool("/Subsystems/Vision/Is Valid For Update");
    isValidForResetLog = new LoggedBool("/Subsystems/Vision/Is Valid For Reset");
  }

  public double getTx() {
    return visionIO.getTx();
  }

  public double getTy() {
    return visionIO.getTy();
  }

  public double getTa() {
    return visionIO.getTa();
  }

  public boolean isTarget() {
    return visionIO.isTarget();
  }

  public int getTargetCount() {
    return visionIO.getTargetCount();
  }

  public void updateOdometry() {
    PoseEstimator.getInstance().updateVision(visionPoseEstimate.pose , visionPoseEstimate.timestampSeconds);
  }

  public static Vision getInstance() {
    if (vision == null) {
      vision = new Vision();
    }
    return vision;
  }

  @Override
  public void periodic() {
    visionIO.update();

    visionPoseEstimate = visionIO.getEstimatedPose();
    isUpdateForOdometry = visionFilters.isValidForUpdate(visionPoseEstimate.pose);
    isUpdateGyro = visionFilters.isValidForReset();
    
    
    visionPose2dLog.update(visionPoseEstimate.pose);
    tXLog.update(visionIO.getTx());
    tYLog.update(visionIO.getTy());
    hasTargetLog.update(visionIO.isTarget());
    targetCountLog.update(visionIO.getTargetCount());
    isValidLog.update(isUpdateForOdometry);
    isValidForResetLog.update(visionFilters.isValidForReset());

    if (isUpdateForOdometry) {
      updateOdometry();
    }

    if (!didUpdatedGyro) {
      if (isUpdateGyro) {
        didUpdatedGyro = true;
        SwerveSubsystem.getInstance().getGyro().updateOffset();
      }
    }
    
  }
}
