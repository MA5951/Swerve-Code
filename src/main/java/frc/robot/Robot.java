// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ma5951.utils.Logger.MALog;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Utils.RobotClock;
import frc.robot.commands.Swerve.DriveController;
import frc.robot.commands.Swerve.TeleopSwerveController;
import frc.robot.subsystems.PoseEstimation.SwervePoseCalculator;
import frc.robot.subsystems.Swerve.SwerveSubsystem;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  
  private RobotContainer m_robotContainer;

 
  @Override
  public void robotInit() {
    RobotClock.getInstance();
    m_robotContainer = new RobotContainer();
    SwerveSubsystem.getInstance();
    //SwervePoseCalculator.getInstance();
    MALog.getInstance();
  }


  @Override
  public void robotPeriodic() {
    
    CommandScheduler.getInstance().run();
    //SwervePoseCalculator.getInstance().update();;
  }

  @Override
  public void disabledInit() {
    MALog.getInstance().stopLog();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    MALog.getInstance().startLog();
    CommandScheduler.getInstance().setDefaultCommand(
      SwerveSubsystem.getInstance(),
      new TeleopSwerveController(RobotContainer.driveController));
  
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
