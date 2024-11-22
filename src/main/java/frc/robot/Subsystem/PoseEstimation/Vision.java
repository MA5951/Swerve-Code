// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.PoseEstimation;


import com.ma5951.utils.Logger.LoggedBool;
import com.ma5951.utils.Logger.LoggedDouble;
import com.ma5951.utils.Logger.LoggedPose2d;
import com.ma5951.utils.Vision.Limelights.Limelight3G;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;
import frc.robot.Subsystem.Swerve.SwerveSubsystem;

public class Vision extends SubsystemBase {
  private static Vision vision;

  private Limelight3G limelight;
  private LoggedDouble ditanceToSpeakerLog;
  private LoggedPose2d poseLog;
  private LoggedDouble tagIdLog;
  private LoggedDouble tXLog;
  private LoggedDouble tYLog;
  private LoggedBool isTagLog;

  private Vision() {
    limelight = new Limelight3G(PortMap.Vision.CAMERA_NAME , VisionConstants.CAMERA_HIGHT , VisionConstants.CAMERA_ANGLE
    ,() -> SwerveSubsystem.getInstance().getFusedHeading());

    ditanceToSpeakerLog = new LoggedDouble("/Vision/Distance To Speaker");
    tagIdLog = new LoggedDouble("/Vision/Tag Id");
    tXLog = new LoggedDouble("/Vision/Tx");
    tYLog = new LoggedDouble("/Vision/Ty");
    isTagLog = new LoggedBool("/Vision/Is Tag");
    poseLog = new LoggedPose2d("/Vision/Pose");
  }

  public Pose2d getEstiman() {
    if (limelight.getEstimatedPose().pose != null) {
      return limelight.getEstimatedPose().pose;
    } else {
      return new Pose2d();
    }
    //return new Pose2d();
  }

  public double getTimeStamp() {
    return limelight.getEstimatedPose().timestampSeconds;
  }

  public boolean isTag() {
    return limelight.isTarget();
  }

  public double getTagID() {
    return limelight.getTagID();
  }

  public double getTx() {
    return limelight.getTx();
  }

  public double getTy() {
    return limelight.getTy();
  }

  // public double getDistanceToTag() {
  //   return limelight.getDistanceToTag();
  // }

  // public double getDistance() {
  //   return limelight.distance(VisionConstants.aprilTagsHights);
  // }

  public void filterSpeaker() {
    limelight.filterTags(new int[] {7 , 4});
  }

  public void resetFilter() {
    limelight.filterTags(new int[] {});
  }

  public double getTagArea() {
    return limelight.getRawDetection().ta;
  }

  public static Vision getInstance() {
    if (vision == null) {
      vision = new Vision();
    }
    return vision;
  }

  public void resetHeading() {
    //limelight.resetHeading();
  }

  @Override
  public void periodic() {
    limelight.update();

    poseLog.update(getEstiman());
    //ditanceToSpeakerLog.update(getDistance());
    tagIdLog.update(getTagID());
    tXLog.update(getTx());
    tYLog.update(getTy());
    isTagLog.update(isTag());
  }

}
