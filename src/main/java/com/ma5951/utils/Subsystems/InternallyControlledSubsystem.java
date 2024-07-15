package com.ma5951.utils.Subsystems;

public interface InternallyControlledSubsystem extends MotorSubsystem{
  public void calculate(double setPoint);

  public boolean atPoint();

  public void setSetPoint(double setPoint);
}
