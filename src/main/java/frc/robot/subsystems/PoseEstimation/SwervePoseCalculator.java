// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.PoseEstimation;

import frc.robot.subsystems.Swerve.CollisionDtector;
import frc.robot.subsystems.Swerve.SkidDetector;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

/** Add your docs here. */
public class SwervePoseCalculator {

    private CollisionDtector collisionDtector;
    private SkidDetector skidDetector;

    public SwervePoseCalculator() {
        //collisionDtector = new CollisionDtector(SwerveSubsystem.getInstance().getGyro());
        skidDetector = new SkidDetector(SwerveSubsystem.getInstance().getOdometryUpdate());
    }

    
    public void update() {
        collisionDtector.update();
    }



}
