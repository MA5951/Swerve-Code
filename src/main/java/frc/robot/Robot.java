// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.ironmaple.simulation.SimulatedArena;

import com.ma5951.utils.Logger.MALog;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystem.PoseEstimation.PoseEstimator;
import frc.robot.Subsystem.Swerve.SwerveConstants;



public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  

  private RobotContainer m_robotContainer;
  private boolean isTeleop = false;
  public static boolean isStartingPose = false;

  @Override
  public void robotInit() {
    MALog.getInstance();
    m_robotContainer = new RobotContainer();
    PoseEstimator.getInstance();

    MALog.getInstance().startLog();

    
    
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    PoseEstimator.getInstance().update();
    
    
  }

  @Override
  public void disabledInit() {

    if (isTeleop) {
      MALog.getInstance().stopLog();
    }
  }

  @Override
  public void disabledPeriodic() {
    

  }

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

    isTeleop = true;
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
  public void simulationInit() {
    SimulatedArena.getInstance().addDriveTrainSimulation(SwerveConstants.SWERVE_DRIVE_SIMULATION);
  }

  @Override
  public void simulationPeriodic() {
    SimulatedArena.getInstance().simulationPeriodic();
  }

  public int getStateAsNum() {
    if (RobotContainer.currentRobotState == RobotConstants.IDLE) {
      return 0;
    } else if (RobotContainer.currentRobotState == RobotConstants.INTAKE) {
      return 2;
    } else if (RobotContainer.currentRobotState == RobotConstants.EJECT) {
      return 4;
    } else if (RobotContainer.currentRobotState == RobotConstants.WARMING) {
      return 6;
    } else if (RobotContainer.currentRobotState == RobotConstants.AMP) {
      return 8;
    } else if (RobotContainer.currentRobotState == RobotConstants.FEEDING) {
      return 10;
    } else if (RobotContainer.currentRobotState == RobotConstants.SOURCE_INTAKE) {
      return 12;
    } else if (RobotContainer.currentRobotState == RobotConstants.STATIONARY_SHOOTING) {
      return 14;
    } else if (RobotContainer.currentRobotState == RobotConstants.PRESET_SHOOTING) {
      return 16;
    } else {
      return 0;
    } 
  }
}
