// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ma5951.utils.StateControl.StatesTypes.State;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystem.Swerve.SwerveSubsystem;
import frc.robot.commands.Swerve.FieldCentricDriveController;
import frc.robot.commands.Swerve.TeleopSwerveController;

public class RobotContainer {
  public static State currentRobotState = RobotConstants.IDLE;
  public static State lastRobotState = currentRobotState;

  public static XboxController driverController = new XboxController(PortMap.Controllers.driveID);

  public RobotContainer() {
    SwerveSubsystem.getInstance();
    configureBindings();
    
    CommandScheduler.getInstance().setDefaultCommand(SwerveSubsystem.getInstance(), new TeleopSwerveController(RobotContainer.driverController));
  }
  
  public static void update() {
  }

  public void setIDLE() {
      lastRobotState = currentRobotState;
      currentRobotState = RobotConstants.IDLE;
  }

  private void configureBindings() {
    new Trigger(() -> driverController.getYButton()).onTrue(new InstantCommand(() -> FieldCentricDriveController.updateDriveHeading()));
  }

  public String getAutonomousName() {
    return null;
  }
  
  public Command getAutonomousCommand() {
    return null;
  }
}
