
package frc.robot;



import com.ma5951.utils.StateControl.StatesTypes.State;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotControl.SuperStructure;

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

    
    //FieldConstants
    public static final Pose2d BLUE_SPEAKER = new Pose2d(0 , 5.548 , new Rotation2d(0));
    public static final Pose2d RED_SPEAKER = new Pose2d(16.579 , 5.548 , new Rotation2d(0));
    public static final Pose2d BLUE_AMP = new Pose2d(1.842 , 8.204 , new Rotation2d(0));
    public static final Pose2d RED_AMP = new Pose2d(14.701 , 8.204 , new Rotation2d(0));


}
