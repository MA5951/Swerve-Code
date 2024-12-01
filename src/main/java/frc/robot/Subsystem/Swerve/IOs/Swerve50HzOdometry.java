
package frc.robot.Subsystem.Swerve.IOs;

import com.ma5951.utils.Logger.LoggedBool;
import com.ma5951.utils.Logger.LoggedDouble;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Subsystem.Swerve.Util.OdometryConfig;
import frc.robot.Subsystem.Swerve.Util.SwerveOdometry;

public class Swerve50HzOdometry implements SwerveOdometry{

    private OdometryConfig config;

    private LoggedBool skidDetectedLog;
    private LoggedBool collisionDetectedLog;
    private LoggedDouble lastSkidLog;
    private LoggedDouble lastCollisionLog;


    private boolean skidDetected;
    private boolean collisionDetected;
    private double lastSkid;
    private double lastCollid;

    public Swerve50HzOdometry(OdometryConfig Config) {
        config = Config;
        skidDetectedLog = new LoggedBool("/Subsystems/Swerve/Odometry/Skid Detected");
        collisionDetectedLog = new LoggedBool("/Subsystems/Swerve/Odometry/Collision Detected");
        lastSkidLog = new LoggedDouble("/Subystems/Swerve/Odometry/Last Skid");
        lastCollisionLog = new LoggedDouble("/Subystems/Swerve/Odometry/Last Collision");
    }


    public void updateOdometry() {
        config.updateHardwereData();

        skidDetected = Math.abs(config.skidDetector.getSkiddingRatio()) > config.skidRatio;
        collisionDetected = config.collisionDetector.getForce() > config.collisionForce;

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
        
        if (((skidDetected && config.updateInSkid) || !skidDetected) && ((collisionDetected && config.updateInCollision) || !collisionDetected)) {
            config.poseEstimator.updateOdometry(config.getCurrentPositions(), config.getRotation() , Timer.getFPGATimestamp());
        }
    }

    public double getLastSkid() {
        return lastSkid;
    }

    public double getLastCollision() {
        return lastCollid;
    }


}
   