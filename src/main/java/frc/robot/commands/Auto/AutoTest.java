
package frc.robot.commands.Auto;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.Subsystem.PoseEstimation.PoseEstimator;
import frc.robot.Subsystem.Swerve.SwerveAutoFollower;

public class AutoTest extends SequentialCommandGroup {
  
  private PathConstraints constraints = new PathConstraints(
        3.0, 4.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));
  
  public AutoTest() {




    addCommands(
      SwerveAutoFollower.followPath("To midline"),
      new ParallelDeadlineGroup(new WaitUntilCommand(() -> Robot.getSensor() ), SwerveAutoFollower.followPath("Center Run")),
      SwerveAutoFollower.pathFindToPose(getShootingPose(), constraints)
    );
  }

  private Pose2d getShootingPose() {
    if (PoseEstimator.getInstance().getEstimatedRobotPose().getY() < 5.3) {
      return new Pose2d(4.94, 4.7, new Rotation2d(Math.toRadians(163)));
    }
    return new Pose2d(5.4, 6.5, new Rotation2d(Math.toRadians(-170)));
  }
  }