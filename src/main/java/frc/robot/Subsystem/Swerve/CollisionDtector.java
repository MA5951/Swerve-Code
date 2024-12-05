
package frc.robot.Subsystem.Swerve;


import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Subsystem.Swerve.Util.GyroData;

public class CollisionDtector {

    private Supplier<GyroData> gyroData;


    public CollisionDtector(Supplier<GyroData> GyroData) {
        gyroData = GyroData;
    }

    public double getForce() {
        return Math.sqrt(Math.pow(gyroData.get().getAccelX(), 2) +
        Math.pow(gyroData.get().getAccelY(), 2));
    }

    public Translation2d getCollisionVector() {
        return new Translation2d(gyroData.get().getAccelX(), gyroData.get().getAccelY());
    }

}
