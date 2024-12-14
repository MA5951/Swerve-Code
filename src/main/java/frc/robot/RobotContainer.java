// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ma5951.utils.DeafultRobotContainer;
import com.ma5951.utils.DashBoard.AutoSelector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystem.Swerve.SwerveSubsystem;
import frc.robot.Subsystem.Vision.Vision;
import frc.robot.commands.Swerve.FieldCentricDriveController;
import frc.robot.commands.Swerve.TeleopSwerveController;

public class RobotContainer extends DeafultRobotContainer{

  public static boolean alignForAmp = true;


  public RobotContainer() {
    super(0, 1, 2, 3 );
    SwerveSubsystem.getInstance();
    Vision.getInstance();
    configureBindings();
    //new SwerveAutoFollower();
    
    setUpAutoCommands();
    CommandScheduler.getInstance().setDefaultCommand(SwerveSubsystem.getInstance(), new TeleopSwerveController(driverController));
  }

  public void setUpAutoCommands() {
    
  }



  private void configureBindings() {

    //Update Offset
    new Trigger(() -> driverController.getTriangleButton()).onTrue(new InstantCommand(() -> FieldCentricDriveController.updateDriveHeading()));



  }

  
  public Command getAutonomousCommand() {
    return null;
  }
}
