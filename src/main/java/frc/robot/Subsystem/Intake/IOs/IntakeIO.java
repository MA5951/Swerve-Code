// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Intake.IOs;

/** Add your docs here. */
public interface IntakeIO {

    double getCurrentDraw(); //Return Current Draw In Amp

    double getVelocity(); //Retrun System Velocity In RPM

    double getMotorTemp(); //Return Motor Temp In Celecuis

    double getAppliedVolts(); //Return Applied Volts

    void  setNutralMode(boolean isBrake); //Sets between coast and brake 

    void setVoltage(double volt); //Sets motor voltage between -12 to 12

    void updatePeriodic(); //Update Periodic

}