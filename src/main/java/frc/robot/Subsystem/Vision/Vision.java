// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  private static Vision vision;

  private VisionIO visionIO = VisionConstants.getVisionIO();

  public Vision() {}


  public static Vision getInstance() {
    if (vision == null) {
      vision = new Vision();
    }
    return vision;
    }

  @Override
  public void periodic() {
    visionIO.update();
    System.out.println(visionIO.getTagID());
  }
}
