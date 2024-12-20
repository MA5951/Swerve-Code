// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Intake;

import com.ma5951.utils.Logger.LoggedBool;
import com.ma5951.utils.Logger.LoggedDouble;
import com.ma5951.utils.StateControl.StatesTypes.StatesConstants;
import com.ma5951.utils.StateControl.Subsystems.StateControlledSubsystem;

import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.Subsystem.Intake.IOs.IntakeIO;

public class Intake extends StateControlledSubsystem {
  private static Intake intake;

  private IntakeIO intakeIO =  IntakeConstants.getIntakeIO();

  private LoggedDouble offsetLog;
  private LoggedBool IntakeCanMove;
  private LoggedBool EjectCanMove;
  private LoggedBool CanMove;
  
  private Intake() {
    super(IntakeConstants.SYSTEM_STATES , "Intake");
    intakeIO.setNutralMode(true);
    board.addNum("Intake Adjust" , 1);
    offsetLog = new LoggedDouble("/Subsystems/Intake/Offset");
    IntakeCanMove = new LoggedBool("/Subsystems/Intake/Can Move/Intake");
    EjectCanMove = new LoggedBool("/Subsystems/Intake/Can Move/Eject");
    CanMove = new LoggedBool("/Subsystems/Intake/Can Move");
  }

  public double getAppliedVolts() {
    return intakeIO.getAppliedVolts();
  }

  public double getCurrentDraw() {
    return intakeIO.getCurrentDraw();
  }

  public void turnOnIntke() {
    setVoltage(IntakeConstants.INTAKE_POWER);
  }

  public void turnOnEjectIntake() {
    setVoltage(IntakeConstants.EJECT_POWER);
  }

  public void turnOffIntke() {
    setVoltage(0);
  }

  public void setVoltage(double voltage ) {
    intakeIO.setVoltage(voltage * board.getNum("Intake Adjust"));
  }

  public void setPower(double power) {
    setVoltage(power * 12);
  }

  @Override
  public boolean canMove() {
      return true;
  }

  public static Intake getInstance() {
    if (intake == null) {
        intake = new Intake();  
    }
    return intake;
  }

  @Override
  public void periodic() {
    super.periodic();
    intakeIO.updatePeriodic();
   
    board.addNum("Applied Volts", getAppliedVolts());

    offsetLog.update(board.getNum("Intake Adjust"));
    CanMove.update(canMove());
  }
}