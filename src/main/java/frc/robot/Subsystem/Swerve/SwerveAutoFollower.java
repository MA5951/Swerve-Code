
// package frc.robot.Subsystem.Swerve;



// import com.ma5951.utils.Logger.LoggedPose2d;
// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.commands.PathPlannerAuto;
// import com.pathplanner.lib.config.RobotConfig;
// import com.pathplanner.lib.config.PIDConstants;
// import frc.robot.Utils.PPHolonomicDriveController;
// import com.pathplanner.lib.util.PathPlannerLogging;

// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Subsystem.PoseEstimation.PoseEstimator;

// public class SwerveAutoFollower {

//     private SwerveSubsystem swerve = SwerveSubsystem.getInstance();
//     private PoseEstimator poseEstimate = PoseEstimator.getInstance();
//     private LoggedPose2d targetPoseLog;
//     private LoggedPose2d currentPoseLog;
//     private RobotConfig config;

//     public SwerveAutoFollower() {
//         targetPoseLog = new LoggedPose2d("/Auto/Target Pose");
//         currentPoseLog = new LoggedPose2d("/Auto/Current Pose");
        
//         try{
//             config = RobotConfig.fromGUISettings();
//           } catch (Exception e) {
//               e.printStackTrace();
//           }

//         AutoBuilder.configure(
//             () -> poseEstimate.getEstimatedRobotPose() ,
//             pose -> poseEstimate.resetPose(pose), 
//             () -> swerve.getRobotRelativeSpeeds(),
//             (speeds , feedforwards) -> swerve.drive(speeds , feedforwards),
//             new PPHolonomicDriveController(
//                 new PIDConstants(0, 0, 0),
//                 new PIDConstants(0, 0, 0),
//                 new PIDConstants(0, 0, 0)),
//                 config,
//             () -> {
//                 var alliance = DriverStation.getAlliance();
//                 if (alliance.isPresent()) {
//                   return alliance.get() == DriverStation.Alliance.Red;
//                 }
//                 return false;
//               }, swerve);


        
//         PathPlannerLogging.setLogTargetPoseCallback(pose -> targetPoseLog.update(pose));
//         PathPlannerLogging.setLogCurrentPoseCallback(pose -> currentPoseLog.update(pose));
//     }

//     public static Command buildAuto(String autoName) {
//         return new PathPlannerAuto(autoName);
//     }
// }
