
package com.ma5951.utils.ControlledMotors.Sim;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

public class TalonFXMotorSim {

    private TalonFX talonFX;
    private TalonFXConfiguration config;
    private TalonFXSimState talonFxSim;

    public TalonFXMotorSim(TalonFX motor , TalonFXConfiguration motorConfig) {
        talonFX = motor;
        config = motorConfig;

        motor.getConfigurator().apply(config);
        talonFxSim = motor.getSimState();

        talonFxSim.addRotorPosition(null);

    }

    public StatusSignal<Current> getCurrent() {
        return talonFX.getStatorCurrent();
    }

    public StatusSignal<AngularVelocity> getVelocity() {
        return talonFX.getVelocity();
    }

    public StatusSignal<Voltage> getAppliedVolts() {
        return talonFX.getMotorVoltage();
    }

    public StatusSignal<AngularAcceleration> getAcceleration() {
        return talonFX.getAcceleration();
    }

    public StatusSignal<Angle> getPosition() {
        return talonFX.getRotorPosition();
    }




}
