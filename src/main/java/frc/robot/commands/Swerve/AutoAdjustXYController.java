
package frc.robot.commands.Swerve;

import java.util.function.Supplier;

import com.ma5951.utils.Swerve.SwerveController;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Subsystem.Swerve.SwerveConstants;

public class AutoAdjustXYController implements SwerveController {

    private Supplier<Pose2d> currentPoseSupplier;
    private ProfiledPIDController xController = new ProfiledPIDController(
        SwerveConstants.XY_KP, SwerveConstants.XY_KI, SwerveConstants.XY_KD, SwerveConstants.XY_CONSTRAINTS);
    private ProfiledPIDController yController;

    public AutoAdjustXYController(Supplier<Pose2d> robotPoseSupplier) {
        currentPoseSupplier = robotPoseSupplier;

        xController.setTolerance(0);

    }

    public ChassisSpeeds update() {
        return new ChassisSpeeds();
    }

    


}
