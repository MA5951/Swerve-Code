package frc.robot.commands.oldSwerve;
// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.swerve;

// import java.util.function.Supplier;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.Conveyr.Conveyr;
// import frc.robot.subsystems.Vision.NoteDetector;
// import frc.robot.subsystems.swerve.SwerveConstants;
// import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;

// public class AssistedDriving extends Command {
//   /** Creates a new swerveJoystickCommand. */
//   private final SwerveDrivetrainSubsystem swerve;
//   //private final NoteDetector noteDetector;
//   //private final Conveyr conveyr;
//   // private final SlewRateLimiter xRateLimiter, yRateLimiter, turningRateLimiter;
//   private Supplier<Double> xSpeedSupplier;
//   private Supplier<Double> ySpeedSupplier;
//   private Supplier<Double> turningSpeedSupplier;
//   private boolean filedReletive;
//   private PIDController pidController;
//   private ChassisSpeeds robotSpeed;

//   /**
//    * @param xSpeedSupplier       Percentage (-1 - 1)
//    * @param ySpeedSupplier       Percentage (-1 - 1)
//    * @param turningSpeedSupplier Percentage (-1 - 1)
//    */
//   public AssistedDriving(
//       Supplier<Double> xSpeedSupplier,
//       Supplier<Double> ySpeedSupplier,
//       Supplier<Double> turningSpeedSupplier,
//       boolean filedReletive) {
//     swerve = SwerveDrivetrainSubsystem.getInstance();
//     //noteDetector = NoteDetector.getInstance();
//     //conveyr = Conveyr.getInstance();
//     this.xSpeedSupplier = xSpeedSupplier;
//     this.ySpeedSupplier = ySpeedSupplier;
//     this.turningSpeedSupplier = turningSpeedSupplier;
//     this.filedReletive = filedReletive;

//     //pidController = new PIDController(SwerveConstants.ASSISTED_INTAKE_P, SwerveConstants.ASSISTED_INTAKE_I, SwerveConstants.ASSISTED_INTAKE_D);

//     addRequirements(swerve);
//   }

//   public AssistedDriving(
//       Supplier<Double> xSpeedSupplier,
//       Supplier<Double> ySpeedSupplier,
//       Supplier<Double> turningSpeedSupplier) {
//         this(xSpeedSupplier, ySpeedSupplier, turningSpeedSupplier, true);
//       }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     double xSpeed = SwerveDrivetrainSubsystem.getInstance().isXYReversed ? ySpeedSupplier.get() : xSpeedSupplier.get();
//     double ySpeed = SwerveDrivetrainSubsystem.getInstance().isXYReversed ? xSpeedSupplier.get() : ySpeedSupplier.get();
//     double turningSpeed = turningSpeedSupplier.get();

//     // if (noteDetector.getNoteYLocationRelativRobot() < SwerveConstants.ASSISTED_INTAKE_DISTANCE
//     //   && noteDetector.hasTarget()
//     //   && !conveyr.isRing()) {
//     //   robotSpeed = ChassisSpeeds.fromRobotRelativeSpeeds(
//     //     pidController.calculate(noteDetector.getNoteXLocationRelativRobot(), 0)
//     //    * SwerveConstants.ASSISTED_INTAKE_POWER,
//     //     0,
//     //     0,
//     //      new Rotation2d(
//     //               Math.toRadians((swerve.getFusedHeading() - swerve.getOffsetAngle()))));
//     // } else if (true) {
      
//     // }
    
    
//     xSpeed = Math.abs(xSpeed) < 0.1 ? 0 : xSpeed;
//     ySpeed = Math.abs(ySpeed) < 0.1 ? 0 : ySpeed;

//     turningSpeed = (Math.abs(turningSpeed) < 0.1 ? 0 : turningSpeed) * -1;

//     xSpeed = xSpeed *
//         swerve.maxVelocity *
//         (SwerveDrivetrainSubsystem.getInstance().isXReversed ? -1 : 1);
//     ySpeed = ySpeed *
//         swerve.maxVelocity *
//         (SwerveDrivetrainSubsystem.getInstance().isYReversed ? -1 : 1);
//     turningSpeed = turningSpeed *
//         swerve.maxAngularVelocity;

//     swerve.drive(xSpeed, ySpeed, turningSpeed, filedReletive);

//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     swerve.stop();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }