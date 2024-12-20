// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Intake.IOs;

import com.ma5951.utils.Logger.LoggedDouble;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.RobotConstants;
import frc.robot.Subsystem.Intake.IntakeConstants;

/** Add your docs here. */
public class IntakeIOSim implements IntakeIO{

    private DCMotorSim motor;
    private double appliedVolts;

    private LoggedDouble motorTempLog;
    private LoggedDouble appliedVoltsLog;
    private LoggedDouble velocityLog;
    private LoggedDouble currentDrawLog;

    public IntakeIOSim() {
        //motor = new DCMotorSim(DCMotor.getFalcon500(1), IntakeConstants.Gear, 0.05);
     
        motorTempLog = new LoggedDouble("/Subsystems/Intake/Sim/Motor Temp");
        appliedVoltsLog = new LoggedDouble("/Subsystems/Intake/Sim/Applied Voltage");
        velocityLog = new LoggedDouble("/Subsystems/Intake/Sim/Velocity");
        currentDrawLog = new LoggedDouble("/Subsystems/Intake/Sim/Motor Current");

    }

    public double getCurrentDraw() {
        return motor.getCurrentDrawAmps();
    }

    public double getVelocity() {
      return  motor.getAngularVelocityRPM();
    }
    
    public double getMotorTemp() {
        return 0;
    }

    public double getAppliedVolts() {
       return appliedVolts;
    }

    public void setNutralMode(boolean isBrake) {
        
    }
     
    public void setVoltage(double volt) {
        
        appliedVolts = volt;
        motor.setInputVoltage(volt);
    }
     
    public void updatePeriodic() {
        motor.update(RobotConstants.kDELTA_TIME);    
        
        motorTempLog.update(getMotorTemp());
        appliedVoltsLog.update(getAppliedVolts());
        velocityLog.update(getVelocity());
        currentDrawLog.update(getCurrentDraw());
    }

}