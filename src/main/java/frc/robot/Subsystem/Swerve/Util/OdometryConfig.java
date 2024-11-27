
package frc.robot.Subsystem.Swerve.Util;


import frc.robot.Subsystem.PoseEstimation.PoseEstimator;
import frc.robot.Subsystem.Swerve.CollisionDtector;
import frc.robot.Subsystem.Swerve.SkidDetector;
import frc.robot.Subsystem.Swerve.SwerveSubsystem;

public class OdometryConfig {

    public SwerveSubsystem swerveSubsystem;
    public PoseEstimator poseEstimator;
    public SkidDetector skidDetector;
    public CollisionDtector  collisionDetector;
    public boolean updateInCollision;
    public boolean updateInSkid;
    public double skidRatio;
    public double collisionForce;


    // public OdometryConfig() {
    //     this(null, null, null, null, false, false, 1, 0);
    // }

    public OdometryConfig(
        SwerveSubsystem SwerveSubsystem , 
        PoseEstimator PoseEstimator , 
        SkidDetector SkidDetector , 
        CollisionDtector  CollisionDetector ,
        boolean UpdateInCollision , 
        boolean UpdateInSkid ,
        double SkidRatio , 
        double CollisionForce) {
        updateInCollision = UpdateInCollision;
        updateInSkid = UpdateInSkid;
        swerveSubsystem = SwerveSubsystem;
        poseEstimator = PoseEstimator;
        skidDetector = SkidDetector;
        collisionForce = CollisionForce;
        skidRatio = SkidRatio;
    }



}
