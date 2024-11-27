
package frc.robot.Subsystem.Swerve.IOs;

import com.ma5951.utils.Logger.LoggedBool;

import frc.robot.Subsystem.Swerve.Util.OdometryConfig;
import frc.robot.Subsystem.Swerve.Util.SwerveOdometry;

public class Swerve50HzOdometry implements SwerveOdometry{

    private OdometryConfig config;

    private LoggedBool skidDetectedLog;
    private LoggedBool collisionDetectedLog;

    private boolean skidDetected;
    private boolean collisionDetected;
    private double lastSkid;
    private double lastCollid;
    private boolean updated;

    public Swerve50HzOdometry(OdometryConfig Config) {
        skidDetectedLog = new LoggedBool("/Subsystems/Swerve/Odometry/Skid Detected");
        collisionDetectedLog = new LoggedBool("/Subsystems/Swerve/Odometry/Collision Detected");
    }


    public void updateOdometry() {
        updated = false;
        skidDetected = Math.abs(config.skidDetector.getSkiddingRatio()) > config.skidRatio;
        collisionDetected = config.collisionDetector.getForce() > config.collisionForce;

        
        
        if (
            (!skidDetected && !collisionDetected ) || 
            (skidDetected && config.updateInSkid) && (!collisionDetected && !config.updateInCollision) ||
            (collisionDetected && config.updateInCollision) && (!skidDetected) ||
            (skidDetected && collisionDetected && config.updateInCollision && config.updateInSkid)
        ) {

        }
    }


}
   