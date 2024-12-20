// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Intake.IOs;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ma5951.utils.Logger.LoggedDouble;
import com.ma5951.utils.Utils.ConvUtil;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.PortMap;
import frc.robot.Subsystem.Intake.IntakeConstants;

public class IntakeIOReal implements IntakeIO {

    private TalonFX intakeMotor;
    private TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    private StatusSignal<Current> currentDraw;
    private StatusSignal<AngularVelocity> velocity;
    private StatusSignal<Temperature> motorTemp;
    private StatusSignal<Voltage> appliedVolts;

    private LoggedDouble motorTempLog;
    private LoggedDouble appliedVoltsLog;
    private LoggedDouble velocityLog;
    private LoggedDouble currentDrawLog;

    public IntakeIOReal() {
        intakeMotor = new TalonFX(PortMap.Intake.KrakenIntakeMotor , PortMap.CanBus.CANivoreBus);

        configTalonFX();
        
        currentDraw = intakeMotor.getStatorCurrent();
        velocity = intakeMotor.getVelocity();
        motorTemp = intakeMotor.getDeviceTemp();
        appliedVolts = intakeMotor.getMotorVoltage();

        motorTempLog = new LoggedDouble("/Subsystems/Intake/Real/Motor Temp");
        appliedVoltsLog = new LoggedDouble("/Subsystems/Intake/Real/Applied Voltage");
        velocityLog = new LoggedDouble("/Subsystems/Intake/Real/Intake Velocity");
        currentDrawLog = new LoggedDouble("/Subsystems/Intake/Real/Motor Current");

        
    }

    public void configTalonFX() {
        motorConfig.Feedback.SensorToMechanismRatio = IntakeConstants.Gear;
        
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        motorConfig.Voltage.PeakForwardVoltage = 12;
        motorConfig.Voltage.PeakReverseVoltage = -12;

        // motorConfig.CurrentLimits.SupplyCurrentLimitEnable = IntakeConstants.IsCurrentLimitEnabled;
        // motorConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.ContinuesCurrentLimit;
        // motorConfig.CurrentLimits.SupplyCurrentThreshold = IntakeConstants.PeakCurrentLimit;
        // motorConfig.CurrentLimits.SupplyTimeThreshold = IntakeConstants.PeakCurrentTime;

        intakeMotor.getConfigurator().apply(motorConfig);
    }

    public double getCurrentDraw() {
        return currentDraw.getValueAsDouble();
    }

    public double getVelocity() {
        return ConvUtil.RPStoRPM(velocity.getValueAsDouble());
    }

    public double getMotorTemp() {
        return motorTemp.getValueAsDouble();
    }
    
    public double getAppliedVolts() {
        return appliedVolts.getValueAsDouble();
    }

    public void setNutralMode(boolean isBrake) {
        if (isBrake) {
            motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        } else {
            motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        }
        intakeMotor.getConfigurator().apply(motorConfig);
    }

    public void setVoltage(double volt) {
        intakeMotor.setVoltage(volt);
    }

    public void updatePeriodic() {
        currentDraw.refresh();
        velocity.refresh();
        motorTemp.refresh();
        appliedVolts.refresh();

        motorTempLog.update(getMotorTemp());
        appliedVoltsLog.update(getAppliedVolts());
        velocityLog.update(getVelocity());
        currentDrawLog.update(getCurrentDraw());
    }



}