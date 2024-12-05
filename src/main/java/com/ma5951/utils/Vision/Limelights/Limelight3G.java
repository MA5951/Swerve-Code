/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ma5951.utils.Vision.Limelights;

import java.util.function.Supplier;

import com.ma5951.utils.Utils.DriverStationUtil;
import com.ma5951.utils.Vision.Limelights.LimelightHelpers.PoseEstimate;
import com.ma5951.utils.Vision.Limelights.LimelightHelpers.RawDetection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Limelight3G {
  private String name;
  private double cammeraHight;
  private double cammeraAngle;
  private double offset = 0;
  private Supplier<Double> robotAngleSupplier;

  /**
   * Limelight3G class to get data from vision.
   * 
   * @param cammeraName The camera name.
   * @param cammeraHight The hight from the middle of the lens to the ground.
   * @param cammeraAngle The angle of the camera from the ground (o is horizontal).
   * @param angleSupplier The angle of the robot, **MUST** be absolute 0 is red alliance wall.
  **/
  public Limelight3G(
    String cammeraName , double cammeraHight , double cammeraAngle , Supplier<Double> angleSupplier){
      name  = cammeraName;
      this.cammeraHight = cammeraHight;
      this.cammeraAngle = cammeraAngle;
      robotAngleSupplier = angleSupplier;
  }

  public LimelightHelpers.PoseEstimate getEstimatedPose() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name) != null ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name) : new PoseEstimate();
  }

  public boolean isTarget() {
    return LimelightHelpers.getTV(name);
  }

  public LimelightHelpers.RawDetection getRawDetection() {
    return isTarget() ? LimelightHelpers.getRawDetections(name)[0] : new RawDetection(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  }

  public void filterTags(int[] tagsArry) {
    LimelightHelpers.SetFiducialIDFiltersOverride(name , tagsArry);
  }

  public double getTx() {
    return isTarget() ? LimelightHelpers.getTX(name) : 0;
  }

  public double getTy() {
    return isTarget() ? LimelightHelpers.getTY(name) : 0;
  }

  public int getTagID() {
    return ((int)LimelightHelpers.getFiducialID(name));
  }

  public void update() {
    LimelightHelpers.SetRobotOrientation(name, robotAngleSupplier.get(), 0, 0, 0, 0, 0);
  }
}