
package frc.robot.Subsystem.Swerve.Util;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
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

    public void updateHardwereData() {
        swerveSubsystem.updateHardwereData();
    }

    public SwerveModulePosition[] getCurrentPositions() {
        return swerveSubsystem.getSwerveModulePositions();
    }

    public Rotation2d getRotation() {
        return new Rotation2d(swerveSubsystem.getFusedHeading());
    }



}
