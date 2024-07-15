package frc.robot.commands.oldSwerve;
// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.swerve;

// import java.util.function.Supplier;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.subsystems.Vision.AprilTagVision;
// import frc.robot.subsystems.swerve.SwerveConstants;
// import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;

// public class AngleAdjust extends Command {
//   private static PIDController pid;

//   private static SwerveDrivetrainSubsystem swerve;
//   private static AprilTagVision vision;
//   private Supplier<Double> angle;
//   private Supplier<Double> xSupplier;
//   private Supplier<Double> ySupplier;
//   private boolean dontStopAtPoint = false;
//   private boolean useGyro;
//   private boolean aligningToSpeaker = false;
//   public static double yDis;
//   public static double xDis;

//   public static boolean atPoint() {
//     return pid.atSetpoint();
//   }

//   public static double getAngle() {
//     // double xTrget = DriverStation.getAlliance().get() == Alliance.Red ? 
//     //   Constants.FieldConstants.SPEAKER_TAGET_X_RED : Constants.FieldConstants.SPEAKER_TARGET_X_BLUE;
//     // double yTrget = Constants.FieldConstants.SPEAKER_TARGET_Y;
//     // xDis = Math.abs(vision.getEstimatedPose().getX() - xTrget);
//     // yDis = Math.abs(vision.getEstimatedPose().getY() - yTrget);
//     // double angle = Math.atan(yDis / xDis);
//     // angle = DriverStation.getAlliance().get() == Alliance.Blue ?
//     //   -(angle - Math.PI) : angle;
//     // if (vision.getEstimatedPose().getY() > yTrget) {
//     //   angle = -angle;
//     // }
//     return 0d;//angle;
//   }

//   public AngleAdjust(Supplier<Double> angle,
//     Supplier<Double> xSupplier, Supplier<Double> ySupplier, boolean useGyro, boolean dontStopAtPoint
//     , boolean aligningToSpeaker) {
//     swerve = SwerveDrivetrainSubsystem.getInstance();
//     vision = AprilTagVision.getInstance();
//     addRequirements(swerve);

//     this.useGyro = useGyro;
//     this.dontStopAtPoint = dontStopAtPoint;
//     this.aligningToSpeaker = aligningToSpeaker;

//     this.xSupplier = xSupplier;
//     this.ySupplier = ySupplier;

//     pid = new PIDController(
//       SwerveConstants.THATA_KP,
//       SwerveConstants.THATA_KI,
//       SwerveConstants.THATA_KD
//     );
//     pid.setTolerance(SwerveConstants.ANGLE_PID_TOLORANCE);
//     this.angle = angle;
//     pid.enableContinuousInput(-Math.PI, Math.PI);
//   }

//   public void setUseGyro(boolean use) {
//     useGyro = use;
//   }

//   public AngleAdjust(Supplier<Double> angle,
//     Supplier<Double> xSupplier, Supplier<Double> ySupplier, boolean useGyro, boolean dontStopAtPoint) {
//       this(angle, xSupplier, ySupplier, useGyro, dontStopAtPoint, false);
//     }

//   public AngleAdjust(Supplier<Double> angle,
//     Supplier<Double> xSupplier, Supplier<Double> ySupplier) {
//       this(angle, xSupplier, ySupplier, false, false, false);
//     }

//   public AngleAdjust( Supplier<Double> xSupplier, Supplier<Double> ySupplier , boolean Speaker , boolean stop) {
//       this(AngleAdjust::getAngle, xSupplier, ySupplier, false, !stop , Speaker);
//     }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     pid.setSetpoint(angle.get());
//     if (aligningToSpeaker) {
//       pid.setTolerance(swerve.getAngleTolorance(angle.get()));
//     }
//     double xSpeed = SwerveDrivetrainSubsystem.getInstance().isXYReversed ? 
//       ySupplier.get() : xSupplier.get();
//     double ySpeed = SwerveDrivetrainSubsystem.getInstance().isXYReversed ?
//       xSupplier.get() : ySupplier.get();
//     xSpeed = Math.abs(xSpeed) < 0.1 ? 0 : xSpeed;
//     ySpeed = Math.abs(ySpeed) < 0.1 ? 0 : ySpeed;
//     xSpeed = xSpeed *
//       swerve.maxVelocity *
//       (SwerveDrivetrainSubsystem.getInstance().isXReversed ? -1 : 1);
//     ySpeed = ySpeed *
//       swerve.maxVelocity *
//       (SwerveDrivetrainSubsystem.getInstance().isYReversed ? -1 : 1);

//     Supplier<Double> getMeserment = useGyro ? () -> Math.toRadians(swerve.getFusedHeading()) : swerve.getPose().getRotation()::getRadians;
//     swerve.drive(
//       xSpeed, ySpeed,
//       pid.calculate(getMeserment.get())
//       , true);

//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     swerve.stop();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return pid.atSetpoint() && !dontStopAtPoint;
//   }
// }
