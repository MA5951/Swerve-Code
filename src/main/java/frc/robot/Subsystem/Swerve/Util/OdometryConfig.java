
package frc.robot.Subsystem.Swerve.Util;



public class OdometryConfig {

    public boolean updateInCollision;
    public boolean updateInSkid;
    public double skidRatio;
    public double collisionForce;


 
    public OdometryConfig(
        boolean UpdateInCollision , 
        boolean UpdateInSkid ,
        double SkidRatio , 
        double CollisionForce) {
        updateInCollision = UpdateInCollision;
        updateInSkid = UpdateInSkid;
        collisionForce = CollisionForce;
        skidRatio = SkidRatio;
    }

}
