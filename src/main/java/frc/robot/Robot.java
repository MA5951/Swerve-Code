// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;



public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private SparkMax driveMotor;


  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    // m_robotContainer.setIDLE();

    // driveMotor = new SparkMax(2, MotorType.kBrushless);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    RobotConstants.SUPER_STRUCTURE.update();
    RobotContainer.update();

    // driveMotor.setVoltage(12);;
  }

  @Override
  public void disabledInit() {
    m_robotContainer.setIDLE();
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
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.cancel();
    //   }

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

  public int getStateAsNum() {
    if (RobotContainer.currentRobotState == RobotConstants.IDLE) {
      return 0;
    }  else {
      return 0;
    } 
  }
}
