// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Swerve.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;


public class RobotContainer {

  public static final CommandPS5Controller driveController = 
   new CommandPS5Controller(0);

  public RobotContainer() {
    configureBindings();
  }

  
  private void configureBindings() {
    driveController.button(4).whileTrue(new InstantCommand(() -> SwerveSubsystem.getInstance().updateOffset()));

    

  }

  
  public Command getAutonomousCommand() {
    return null;
  }
}
