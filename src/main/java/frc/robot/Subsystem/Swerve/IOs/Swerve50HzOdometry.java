
package frc.robot.Subsystem.Swerve.IOs;

import com.ma5951.utils.Logger.LoggedBool;
import com.ma5951.utils.Logger.LoggedDouble;

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
    private boolean updated;

    public Swerve50HzOdometry(OdometryConfig Config) {
        skidDetectedLog = new LoggedBool("/Subsystems/Swerve/Odometry/Skid Detected");
        collisionDetectedLog = new LoggedBool("/Subsystems/Swerve/Odometry/Collision Detected");
        lastSkidLog = new LoggedDouble("/Subystems/Swerve/Odometry/Last Skid");
    }


    public void updateOdometry() {
        updated = false;
        skidDetected = Math.abs(config.skidDetector.getSkiddingRatio()) > config.skidRatio;
        collisionDetected = config.collisionDetector.getForce() > config.collisionForce;

        
        
        if (((skidDetected && config.updateInSkid) || !skidDetected) && ((collisionDetected && config.updateInCollision) || !collisionDetected)) {

        }
    }


}
   