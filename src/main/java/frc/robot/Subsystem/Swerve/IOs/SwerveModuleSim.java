// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Swerve.IOs;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import static edu.wpi.first.units.Units.*;

import java.util.Arrays;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ma5951.utils.RobotConstantsMAUtil;
import com.ma5951.utils.ControlledMotors.TalonFXSim;
import com.ma5951.utils.Logger.LoggedDouble;
import com.ma5951.utils.Utils.ConvUtil;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.PortMap;
import frc.robot.Subsystem.Swerve.SwerveConstants;
import frc.robot.Subsystem.Swerve.Util.SwerveModule;
import frc.robot.Subsystem.Swerve.Util.SwerveModuleData;
import frc.robot.Subsystem.Swerve.Util.TunerConstants;
import frc.robot.Utils.PhoenixUtil;

public class SwerveModuleSim implements SwerveModule {


    private final SimulatedMotorController.GenericMotorController driveMotor;
    private final SimulatedMotorController.GenericMotorController turnMotor;

    private boolean driveClosedLoop = false;
    private boolean turnClosedLoop = false;
    private final PIDController driveController;
    private final PIDController turnController;
    private double driveFFVolts = 0.0;
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    private double radiansPerSecond;

    private SwerveModuleData moduleData = new SwerveModuleData();
    private final SwerveModuleSimulation moduleSimulation;

    private LoggedDouble DrivePosition;
    private LoggedDouble DriveVelocity;
    private LoggedDouble DriveCurrent;
    private LoggedDouble DriveVolts;
    private LoggedDouble SteerPosition;
    private LoggedDouble SteerCurrent;
    private LoggedDouble SteerVolts;
    private LoggedDouble AbsAngle;
    private LoggedDouble DriveTemp;
    private LoggedDouble SteerTemp;

    public SwerveModuleSim(String moduleNameN, SwerveModuleSimulation  moduleSimulation , boolean isDriveMotorReversed, boolean isTurningMotorReversed) {
        this.driveMotor = moduleSimulation
                .useGenericMotorControllerForDrive()
                .withCurrentLimit(Amps.of(TunerConstants.FrontLeft.SlipCurrent));
        this.turnMotor = moduleSimulation
                .useGenericControllerForSteer()
                .withCurrentLimit(Amps.of(20));

        this.driveController = new PIDController(2, 0.0, 0.0);
        this.turnController = new PIDController(100, 0.0, 0.0);
        
        turnController.enableContinuousInput(-Math.PI, Math.PI);

        DrivePosition = new LoggedDouble("/Swerve/Modules/" + moduleNameN + "Sim" + "/Drive Position");
        DriveVelocity = new LoggedDouble("/Swerve/Modules/" + moduleNameN + "Sim" + "/Drive Velocity");
        DriveCurrent = new LoggedDouble("/Swerve/Modules/" + moduleNameN + "Sim" + "/Drive Current");
        DriveVolts = new LoggedDouble("/Swerve/Modules/" + moduleNameN + "Sim" + "/Drive Volts");
        SteerPosition = new LoggedDouble("/Swerve/Modules/" + moduleNameN + "Sim" + "/Steer Position");
        SteerCurrent = new LoggedDouble("/Swerve/Modules/" + moduleNameN + "Sim" + "/Steer Current");
        SteerVolts = new LoggedDouble("/Swerve/Modules/" + moduleNameN + "Sim" + "/Steer Volts");
        AbsAngle = new LoggedDouble("/Swerve/Modules/" + moduleNameN + "Sim" + "/Absolute Angle");
        DriveTemp = new LoggedDouble("/Swerve/Modules/" + moduleNameN + "Sim" + "/Drive Temp");
        SteerTemp = new LoggedDouble("/Swerve/Modules/" + moduleNameN + "Sim" + "/Steer Temp");

        this.moduleSimulation = moduleSimulation;


        
    }

    public SwerveModuleData update() {

        if (driveClosedLoop) {
            driveAppliedVolts = driveFFVolts
                    + driveController.calculate(
                            moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond));
        } else {
            driveController.reset();
        }
        if (turnClosedLoop) {
            turnAppliedVolts = turnController.calculate(
                    moduleSimulation.getSteerAbsoluteFacing().getRadians());
        } else {
            turnController.reset();
        }

        System.out.println("UPDATEEEEEEEEEEEEE");
        driveMotor.requestVoltage(Volts.of(driveAppliedVolts));
        turnMotor.requestVoltage(Volts.of(turnAppliedVolts));


        moduleData.updateData(
            getDrivePosition(),
            getDriveVelocity(),
            getDriveCurrent(),
            getDriveVolts(),
            getDriveTemp(),
            getSteerTemp(),
            getSteerPosition(),
            getSteerCurrent(),
            getSteerVolts(),
            getAbsolutePosition(),
            getSteerVelocity(), 
            Arrays.stream(moduleSimulation.getCachedDriveWheelFinalPositions())
                .mapToDouble(angle -> angle.in(Rotation) * SwerveConstants.WHEEL_CIRCUMFERENCE)
                .toArray(), 
            moduleSimulation.getCachedSteerAbsolutePositions());


        DrivePosition.update(getDrivePosition());
        DriveVelocity.update(getDriveVelocity());
        DriveCurrent.update(getDriveCurrent());
        DriveVolts.update(getDriveVolts());
        SteerPosition.update(getSteerPosition());
        SteerCurrent.update(getSteerCurrent());
        SteerVolts.update(getSteerVolts());
        AbsAngle.update(getAbsolutePosition());
        DriveTemp.update(getDriveTemp());
        SteerTemp.update(getSteerTemp());

        return moduleData;

    }

    public double getSteerVelocity() {
        return moduleSimulation.getSteerAbsoluteEncoderSpeed().in(RotationsPerSecond);
    }

    public double getDriveVolts() {
        return driveAppliedVolts;
    }

    public double getSteerVolts() {
        return turnAppliedVolts;
    }

    public double getDriveCurrent() {
        return Math.abs(moduleSimulation.getDriveMotorStatorCurrent().in(Amps));
    }

    public double getSteerCurrent() {
        return Math.abs(moduleSimulation.getSteerMotorStatorCurrent().in(Amps));
    }

    public double getAbsolutePosition() {
        return moduleSimulation.getSteerAbsoluteFacing().getDegrees();
    }

    public double getDrivePosition() {
        return moduleSimulation.getDriveWheelFinalPosition().in(Rotations) * SwerveConstants.WHEEL_CIRCUMFERENCE;
    }

    public double getSteerPosition() {
        return moduleSimulation.getSteerAbsoluteFacing().getDegrees();
    }

    public double getDriveVelocity() {
        return (moduleSimulation.getDriveWheelFinalSpeed().in(RotationsPerSecond) * 60) * Math.PI * (SwerveConstants.WHEEL_RADIUS * 2) / 60;
    }

    public double getDriveTemp() {
        return 0;
    }

    public double getSteerTemp() {
        return 0;
    }

    public void turningMotorSetPower(double power) {
        turningMotorSetVoltage(12 * power);
    }

    public void driveMotorSetPower(double power) {
        driveMotorSetVoltage(12 * power);
    }

    public void turningMotorSetVoltage(double volt) {
        turnClosedLoop = false;
        turnAppliedVolts = volt;
    }

    public void driveMotorSetVoltage(double volt) {
        driveClosedLoop = false;
        driveAppliedVolts = volt;
    }

    public void turningUsingPID(double setPoint) {
        turnClosedLoop = true;
        turnController.setSetpoint(setPoint);
    }

    public void driveUsingPID(double setPoint, double feedForward) {
        driveClosedLoop = true;
        radiansPerSecond = (setPoint / SwerveConstants.WHEEL_RADIUS) / (2 * Math.PI ) * 6.2831853071796;
        driveController.setSetpoint(radiansPerSecond);
    }

    public void setNeutralModeDrive(Boolean isBrake) {

    }

    public void setNeutralModeTurn(Boolean isBrake) {
    }

    public void resetSteer() {
    }

}