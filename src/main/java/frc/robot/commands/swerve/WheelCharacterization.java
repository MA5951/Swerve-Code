// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.swerve;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.swerve.SwerveConstants;
// import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;

// public class WheelCharacterization extends Command {

//   private SwerveDrivetrainSubsystem swerve;
//   private double[] driveOffsets = new double[3];//front left - front right - back left- back right
//   private double[] turnAmount = new double[3];//front left - front right - back left- back right
//   private double[] raidus = new double[3];//front left - front right - back left- back right
//   private double gyroOffset = 0;

//   public WheelCharacterization() {
//     swerve = SwerveDrivetrainSubsystem.getInstance();
//     addRequirements(swerve);
    
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     System.out.println("Runing wheel calibration");
//     swerve.stop();
//     for (int i = 0; i < 4 ; i++) {
//       driveOffsets[i] = swerve.getModulesArry()[i].getDrivePosition();
//     }
//     gyroOffset = swerve.getFusedHeading();
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     swerve.drive(0, 0, 0.2, false);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     swerve.stop();
//     for (int i = 0; i < 4 ; i++) {
//       turnAmount[i] = swerve.getModulesArry()[i].getDrivePosition() - driveOffsets[i];
//     }
//     double robotTurn = Math.toRadians(swerve.getFusedHeading() - gyroOffset);
//     double pathLength = SwerveConstants.distanceFromCenter * robotTurn;
//     for (int i = 0; i < 4 ; i++) {
//       raidus[i] = (pathLength / turnAmount[i]) / (2 * Math.PI);
//     }
//     System.out.println(raidus[0] + " -- " +raidus[1] + " -- " + raidus[2] + " -- " + raidus[3]);
//     double avrage = (raidus[0] + raidus[1] + raidus[2] + raidus[3]) / 4;
//     System.out.println("Avrage: " + avrage);

//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return swerve.getFusedHeading() - gyroOffset > 360;
//   }
// }
