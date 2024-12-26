
package frc.robot;


import com.ma5951.utils.DeafultRobotContainer;
import com.ma5951.utils.DashBoard.AutoOption;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystem.PoseEstimation.PoseEstimator;
import frc.robot.Subsystem.Swerve.SwerveAutoFollower;
import frc.robot.Subsystem.Swerve.SwerveSubsystem;
import frc.robot.Subsystem.Vision.Vision;
import frc.robot.commands.Swerve.FieldCentricDriveController;
import frc.robot.commands.Swerve.TeleopSwerveController;

public class RobotContainer extends DeafultRobotContainer{



  public RobotContainer() {
    super(0, 1, 2, 3 );
    SwerveSubsystem.getInstance();
    Vision.getInstance();
    PoseEstimator.getInstance();
    SwerveAutoFollower.getInstance();
    
    configureBindings();
    setUpAutoCommands();
    CommandScheduler.getInstance().setDefaultCommand(SwerveSubsystem.getInstance(), new TeleopSwerveController(driverController));
  }

  public void setUpAutoCommands() {
    setAutoOptions(new AutoOption[] {new AutoOption("Test", SwerveAutoFollower.followPath("Test"), new Pose2d(1.383, 5.528, new Rotation2d(0)))});
  }

  private void configureBindings() {

    //Update Offset
    new Trigger(() -> driverController.getTriangleButton()).onTrue(new InstantCommand(() -> FieldCentricDriveController.updateDriveHeading()));



  }


}