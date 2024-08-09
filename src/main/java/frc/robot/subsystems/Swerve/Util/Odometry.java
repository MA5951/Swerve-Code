package frc.robot.subsystems.Swerve.Util;

import edu.wpi.first.math.geometry.Pose2d;

public interface Odometry {
    
    Pose2d getPose();

    Pose2d update();

}
