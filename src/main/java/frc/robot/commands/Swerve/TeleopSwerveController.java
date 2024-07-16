// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class TeleopSwerveController extends Command {
  
  private DriveController driveCommand;
  private ChassisSpeeds driveControllerSpeeds;

  private SwerveSubsystem swerve;
  private ChassisSpeeds robotSpeeds;
  
  public TeleopSwerveController(CommandPS5Controller controller) {
    swerve = SwerveSubsystem.getInstance();
    
    driveCommand = new DriveController(controller);
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    driveCommand.initialize();
  }

  @Override
  public void execute() {
    driveCommand.execute();
    driveControllerSpeeds = driveCommand.getChassisSpeed();

    //State meachin to switch aulter the final chassis speed
    if (true) {
      robotSpeeds = driveControllerSpeeds;
    }

    swerve.drive(robotSpeeds);
  }

  @Override
  public void end(boolean interrupted) {
    driveCommand.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
