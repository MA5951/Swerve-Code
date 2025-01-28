// Copyright (c) FIRST and other WPILib contributors.
// Open Sostatic urce Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import com.ma5951.utils.Logger.LoggedString;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystem.Swerve.SwerveSubsystem;

public class TeleopSwerveController extends Command {
  
  private FieldCentricDriveController driveController;
  private AngleAdjustController angleAdjustController;
  private RelativAngleAdjustController relativAngleAdjustController;
  private ChassisSpeeds driveControllerSpeeds;
  private ChassisSpeeds angleAdjustControllerSpeeds;
  private ChassisSpeeds relativAngleAdjustControllerSpeeds;
  
  public static boolean atPoint;
  

  private SwerveSubsystem swerve;
  private ChassisSpeeds robotSpeeds;
  private LoggedString xyControllerLog;
  private LoggedString theathControllerLog;
  
  public TeleopSwerveController(XboxController controller) {
    swerve = SwerveSubsystem.getInstance();
    
    driveController = new FieldCentricDriveController(controller , () -> controller.getRightBumperButton() , 
    0.4 , () -> SwerveSubsystem.getInstance().getFusedHeading());
    angleAdjustController = new AngleAdjustController( () -> SwerveSubsystem.getInstance().getFusedHeading());

    xyControllerLog = new LoggedString("/Swerve/Controllers/XY Controller");
    theathControllerLog = new LoggedString("/Swerve/Controllers/Theath Controller");


    addRequirements(swerve);
  }

  @Override
  public void initialize() {
  }   

  @Override
  public void execute() {

    driveControllerSpeeds = driveController.update();
    xyControllerLog.update("Drive Controller");
    theathControllerLog.update("Drive Controller");
    robotSpeeds = driveControllerSpeeds;


    
    swerve.drive(robotSpeeds);
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
