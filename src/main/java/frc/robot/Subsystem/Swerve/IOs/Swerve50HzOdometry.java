
package frc.robot.Subsystem.Swerve.IOs;

import com.ma5951.utils.Logger.LoggedBool;
import com.ma5951.utils.Logger.LoggedDouble;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Subsystem.Swerve.Util.OdometryConfig;
import frc.robot.Subsystem.Swerve.Util.SwerveOdometry;

public class Swerve50HzOdometry implements SwerveOdometry{

    private OdometryConfig odometryConfig;

    private LoggedBool skidDetectedLog;
    private LoggedBool collisionDetectedLog;
    private LoggedDouble lastSkidLog;
    private LoggedDouble lastCollisionLog;


    private boolean skidDetected;
    private boolean collisionDetected;
    private double lastSkid;
    private double lastCollid;

    public Swerve50HzOdometry(OdometryConfig Config) {
        odometryConfig = Config;
        skidDetectedLog = new LoggedBool("/Subsystems/Swerve/Odometry/Skid Detected");
        collisionDetectedLog = new LoggedBool("/Subsystems/Swerve/Odometry/Collision Detected");
        lastSkidLog = new LoggedDouble("/Subystems/Swerve/Odometry/Last Skid");
        lastCollisionLog = new LoggedDouble("/Subystems/Swerve/Odometry/Last Collision");
    }


    public void updateOdometry() {
        odometryConfig.updateHardwereData();

        skidDetected = Math.abs(odometryConfig.skidDetector.getSkiddingRatio()) > odometryConfig.skidRatio;
        collisionDetected = odometryConfig.collisionDetector.getForce() > odometryConfig.collisionForce;

        skidDetectedLog.update(skidDetected);
        collisionDetectedLog.update(collisionDetected);

        if (skidDetected) {
            lastSkid = Timer.getFPGATimestamp();
            lastSkidLog.update(lastSkid);
        }

        if (collisionDetected) {
            lastCollid = Timer.getFPGATimestamp();
            lastCollisionLog.update(lastCollid);
        }
        
        if (((skidDetected && odometryConfig.updateInSkid) || !skidDetected) && ((collisionDetected && odometryConfig.updateInCollision) || !collisionDetected)) {
            odometryConfig.poseEstimator.updateOdometry(odometryConfig.getCurrentPositions(), odometryConfig.getRotation() , Timer.getFPGATimestamp());
        }
    }

    public double getLastSkid() {
        return lastSkid;
    }

    public double getLastCollision() {
        return lastCollid;
    }


}
   