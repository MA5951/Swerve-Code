// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ma5951.utils.DashBoard.AutoOption;
import com.ma5951.utils.DashBoard.AutoSelector;
import com.ma5951.utils.StateControl.StatesTypes.State;
import com.ma5951.utils.StateControl.StatesTypes.StatesConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystem.PoseEstimation.PoseEstimator;
import frc.robot.Subsystem.PoseEstimation.Vision;
import frc.robot.Subsystem.Swerve.SwerveAutoFollower;
import frc.robot.Subsystem.Swerve.SwerveSubsystem;
import frc.robot.Utils.ShootingParameters;
import frc.robot.commands.Swerve.AngleAdjustController;
import frc.robot.commands.Swerve.FieldCentricDriveController;
import frc.robot.commands.Swerve.TeleopSwerveController;

public class RobotContainer {
  public static State currentRobotState = RobotConstants.IDLE;
  public static State lastRobotState = currentRobotState;

  public static PS5Controller driverController = new PS5Controller(PortMap.Controllers.driveID);
  public static PS5Controller oporatorController = new PS5Controller(PortMap.Controllers.operatorID);
  public static PS5Controller driverControllerRumble = new PS5Controller(2);

  public static boolean alignForAmp = true;

  private static AutoSelector autoSelector;

  public RobotContainer() {
    SwerveSubsystem.getInstance();
    Vision.getInstance();
    configureBindings();
    new SwerveAutoFollower();
    
    autoSelector = new AutoSelector(() -> PoseEstimator.getInstance().getEstimatedRobotPose());
    CommandScheduler.getInstance().setDefaultCommand(SwerveSubsystem.getInstance(), new TeleopSwerveController(RobotContainer.driverController));
  }

  public void setUpAutoCommands() {

  }

  public void toggleAmpAlign() {
    if (alignForAmp) {
      alignForAmp = false;
    } else {
      alignForAmp = true;
    }
  }

  public static void update() {
    autoSelector.updateViz();
  }

  public static void setIDLE() {
      lastRobotState = currentRobotState;
      currentRobotState = RobotConstants.IDLE;
  }

  public void setIDLE_AFTER_INTAKE() {
      lastRobotState = currentRobotState;
      currentRobotState = RobotConstants.IDLE;
  }

  public void setINTAKE() {
      lastRobotState = currentRobotState;  
      currentRobotState = RobotConstants.INTAKE;
  }

  public void setEJECT() {
    lastRobotState = currentRobotState;
    currentRobotState = RobotConstants.EJECT;
  }

  public void setWARMING() {
    lastRobotState = currentRobotState;
    currentRobotState = RobotConstants.WARMING;
  }

  public void setAMP() {
    lastRobotState = currentRobotState;
    currentRobotState = RobotConstants.AMP;
  }

  public void setFEEDING() {
    lastRobotState = currentRobotState;
    currentRobotState = RobotConstants.FEEDING;
  }

  public void setSOURCE_INTAKE() {
    lastRobotState = currentRobotState;
    currentRobotState = RobotConstants.SOURCE_INTAKE;
  }

  public void setSTATIONARY_SHOOTING() {
    lastRobotState = currentRobotState;
    currentRobotState = RobotConstants.STATIONARY_SHOOTING;
    
  }

  public void setPRESET_SHOOTING() {
    lastRobotState = currentRobotState;
    currentRobotState = RobotConstants.PRESET_SHOOTING;
  }


  private void configureBindings() {

    //Update Offset
    new Trigger(() -> driverController.getTriangleButton()).onTrue(new InstantCommand(() -> FieldCentricDriveController.updateDriveHeading()));



  }

  public String getAutonomousName() {
    return autoSelector.getSelectedAuto().getName();
  }

  public boolean getIsPathPLannerAuto() {
    return autoSelector.getSelectedAuto().isPathPlannerAuto();
  }

  public AutoOption getCurrentSelectedAutoOption() {
    return autoSelector.getSelectedAuto();
  }
  
  public Command getAutonomousCommand() {
    return null;
  }
}
