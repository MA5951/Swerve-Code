// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.PoseEstimation;


import java.util.function.Supplier;

import com.ma5951.utils.Utils.ConvUtil;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Subsystem.Swerve.SwerveSubsystem;

/** Add your docs here. */
public class PoseEstimatorConstants {

    public final static Vector<N3> ODOMETRY_DEVS = VecBuilder.fill(0.7, 0.7, 0.7);
    public final static Vector<N3> VISION_DEVS = VecBuilder.fill(0.7, 0.7, 0.1);
    
    public final static double MAX_LINEAR_VELOCITY_FOR_UPDATE = 4;//Meters per second
    public final static double MAX_ANGULAR_VELOCITY_FOR_UPDATE = ConvUtil.DegreesToRadians(720);//Radians per second
    public final static double VISION_TO_ODOMETRY_DIFRANCE = 1;
    //public final static double VISION_TO_ODOMETRY_DIFRANCE = 1;
    
    public static Supplier<Boolean> VISION_UPDATE_CONSTRAINS =() -> (DriverStation.isTeleop() && Math.abs(SwerveSubsystem.getInstance().getRobotRelativeSpeeds().vxMetersPerSecond) < MAX_LINEAR_VELOCITY_FOR_UPDATE 
    && Math.abs(SwerveSubsystem.getInstance().getRobotRelativeSpeeds().vyMetersPerSecond) < MAX_LINEAR_VELOCITY_FOR_UPDATE
    && Math.abs(SwerveSubsystem.getInstance().getRobotRelativeSpeeds().omegaRadiansPerSecond) < MAX_ANGULAR_VELOCITY_FOR_UPDATE 
    && Vision.getInstance().isTag()) ||  
    (DriverStation.isAutonomous() && Math.abs(SwerveSubsystem.getInstance().getRobotRelativeSpeeds().vxMetersPerSecond) < 0.5
    && Math.abs(SwerveSubsystem.getInstance().getRobotRelativeSpeeds().vyMetersPerSecond) < 0.5
    && Math.abs(SwerveSubsystem.getInstance().getRobotRelativeSpeeds().omegaRadiansPerSecond) < MAX_ANGULAR_VELOCITY_FOR_UPDATE 
    && Vision.getInstance().isTag());

    public static Supplier<Boolean> ODOMETRY_UPDATE_CONSTRAINS = () -> DriverStation.isEnabled() && !DriverStation.isTest();

    public final static double xyDEVS_COEFFICIENT = 0.02;
    public final static double thetaDEVS_COEFFICIENT = 0.04;
}
