
package frc.robot.Utils;

import edu.wpi.first.math.kinematics.SwerveModulePosition;

public record OdometryUpdate(SwerveModulePosition[] WheelPositions , double TimeStamp) {}
