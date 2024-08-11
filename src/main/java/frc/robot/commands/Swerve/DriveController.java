// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class DriveController extends Command {

  private CommandPS5Controller Controller;
  private double xSpeed;
  private double ySpeed;
  private double turningSpeed;
  private ChassisSpeeds speed;

  public DriveController(CommandPS5Controller controller) {
    Controller = controller;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    xSpeed = Controller.getLeftX();
    ySpeed = Controller.getLeftY();
    turningSpeed = Controller.getRightX();

    xSpeed = Math.abs(xSpeed) < 0.1 ? 0 : xSpeed * -1;
    ySpeed = Math.abs(ySpeed) < 0.1 ? 0 : ySpeed * -1;
    turningSpeed = (Math.abs(turningSpeed) < 0.1 ? 0 : turningSpeed) * -1;

    speed = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed,
                  new Rotation2d(
                    Math.toRadians((SwerveSubsystem.getInstance().getFusedHeading()
                     - SwerveSubsystem.getInstance().getOffsetAngle()))));


  }

  public ChassisSpeeds getChassisSpeed() {
    return speed;
  }

  @Override
  public void end(boolean interrupted) {
    
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
