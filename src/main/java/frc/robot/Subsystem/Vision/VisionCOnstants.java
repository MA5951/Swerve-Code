
package frc.robot.Subsystem.Vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class VisionConstants {


    // Our camera is mounted 0.1 meters forward and 0.5 meters up from the robot pose,
    // (Robot pose is considered the center of rotation at the floor level, or Z =
    // 0)
    public final static Translation3d robotToCameraTrl = new Translation3d(0.1, 0, 0.5);
    // and pitched 15 degrees up.
    public final static Rotation3d robotToCameraRot = new Rotation3d(0, Math.toRadians(-15), 0);
    public final static Transform3d robotToCamera = new Transform3d(robotToCameraTrl, robotToCameraRot);

    public static final VisionIO getVisionIO() {
        return new VisionSim();
    }
}
