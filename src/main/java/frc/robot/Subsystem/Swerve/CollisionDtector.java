
package frc.robot.Subsystem.Swerve;


import java.util.function.Supplier;

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

}
