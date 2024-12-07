
package com.ma5951.utils.Vision.Filters;

import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class LimelightFiltersConfig {

    public boolean updateInAuto;
    public ChassisSpeeds robotUpdateSpeed;
    public Rectangle2d fieldRectangle;
    public Rectangle2d[] fieldObstaclesRectangles;
    public double visionToOdometry;
    public boolean visionToOdometryInTeleop;


    public LimelightFiltersConfig(
        boolean UpdateInAuto,
        ChassisSpeeds RobotUpdateSpeed,
        Rectangle2d FieldRectangle,
        Rectangle2d[] FieldObstaclesRectangles,
        double VisionToOdometry,
        boolean VisionToOdometryInTeleop
    ) {
        updateInAuto = UpdateInAuto;
        robotUpdateSpeed = RobotUpdateSpeed;
        fieldRectangle = new Rectangle2d(new Translation2d(), new Translation2d());
        fieldObstaclesRectangles = FieldObstaclesRectangles;
        visionToOdometry = VisionToOdometry;
        visionToOdometryInTeleop = VisionToOdometryInTeleop;
    }

    public LimelightFiltersConfig() {
        this(false, new ChassisSpeeds(), new Rectangle2d(null, null), new Rectangle2d[1], 0d, false);
    }

    


}
