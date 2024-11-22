// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import com.ma5951.utils.StateControl.StatesTypes.State;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotControl.SuperStructure;
import frc.robot.Subsystem.PoseEstimation.VisionConstants;
import frc.robot.Utils.ShootingParameters;

public class RobotConstants {

    public static final double kDELTA_TIME = 0.02;

    public static final SuperStructure SUPER_STRUCTURE = new SuperStructure();

    public static final State IDLE = new State("IDLE"); //Systems IDLE 0
    public static final State INTAKE = new State("INTAKE"); //Intaking a note //Did 2
    public static final State EJECT = new State("EJECT"); //Ejecting a note from shooter //Did 4
    public static final State WARMING = new State("WARMING"); //Warimng up shooter and arm for shooting //Did 6
    public static final State AMP = new State("AMP"); //Amping //Did 8
    public static final State FEEDING = new State("FEEDING"); //Feeding 10
    public static final State SOURCE_INTAKE = new State("SOURCE_INTAKE"); //Source intake 12
    public static final State STATIONARY_SHOOTING = new State("STATIONARY_SHOOTING"); //Stationary auto shoot //Did 14
    public static final State PRESET_SHOOTING = new State("PRESET_SHOOTING"); //Preset shooting 16

    
    public static final double ShootingTolerance = 15;

    public static final double FeedingOffsetY = 1.5;
    public static final double FeedingOffsetX = 0;

    public static final double WARM_VOLTAGE = 2;

    public static final double DISTANCE_TO_WARM = 7.5;//Warm raidus in meters
    public static final double DISTANCE_TO_CLOSE_ARM = 0.5;//
    public static final double DISTANCE_TO_HIGH_FEED = 5;
    public static final double DISTANCE_TO_SHOOT = 4.9;//4.5
    public static final double DISTANCE_TO_START_AUTO = 0.1;

    public static final ShootingParameters FEEDING_SHOOTING_PARAMETERS = new ShootingParameters(2550, 3650, 45, 0);
    public static final ShootingParameters SUBWOOF_SHOOTING_PARAMETERS = new ShootingParameters(2500, 4000, 63, 0);
    public static final ShootingParameters PODIUM_SHOOTING_PARAMETERS = new ShootingParameters(5500, 5500,  36.4 , 3.5);


    //Interpolation\[]
    //Shooting Points

    private static final ShootingParameters Point1 = new ShootingParameters(0, 0, 0 , 0);

    private static final ShootingParameters Point2 = new ShootingParameters(0, 0, 0 , 0);
    private static final ShootingParameters Point3 = new ShootingParameters(0, 0, 0 , 0);
    private static final ShootingParameters Point4 = new ShootingParameters(0, 0, 0 , 0);
    private static final ShootingParameters Point5 = new ShootingParameters(0, 0, 0 , 0);
    private static final ShootingParameters Point6 = new ShootingParameters(0, 0, 0 , 0);

    

    public static final ShootingParameters[] PointsArry = new ShootingParameters[] {
        Point1,
        Point2,
        Point3,
        Point4,
        Point5,
        Point6
    };

    //XY = 0 is the corenr of the red humean source
    //Y is side to side form driver station view
    //X is front to back from driver station view 
    //FieldConstants
    public static final Pose2d BLUE_SPEAKER = new Pose2d(0 , 5.548 , new Rotation2d(0));
    public static final Pose2d RED_SPEAKER = new Pose2d(16.579 , 5.548 , new Rotation2d(0));
    public static final Pose2d BLUE_AMP = new Pose2d(1.842 , 8.204 , new Rotation2d(0));
    public static final Pose2d RED_AMP = new Pose2d(14.701 , 8.204 , new Rotation2d(0));


}
