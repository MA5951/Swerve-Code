
package frc.robot.Subsystem.Vision;

import com.ma5951.utils.Utils.ConvUtil;

import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Subsystem.Vision.Filters.VisionFiltersConfig;

public class VisionConstants {


    public final static Translation3d robotToCameraTrl = new Translation3d(0.1, 0, 0.5);
    public final static Rotation3d robotToCameraRot = new Rotation3d(0, Math.toRadians(-15), 0);
    public final static Transform3d robotToCamera = new Transform3d(robotToCameraTrl, robotToCameraRot);

    public final static VisionFiltersConfig FILTERS_CONFIG = new VisionFiltersConfig(
        true, 
        new ChassisSpeeds(4, 4, ConvUtil.DegreesToRadians(720)), 
        new Rectangle2d(new Translation2d(), new Translation2d(16.58 , 8.20)), 
        null, 
        1, 
        false);

    public static final VisionIO getVisionIO() {
        return new VisionSim();
    }
}
