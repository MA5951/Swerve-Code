// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Swerve.IOs;


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
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Subsystem.Swerve.SwerveConstants;
import frc.robot.Subsystem.Swerve.Util.SwerveModule;
import frc.robot.Subsystem.Swerve.Util.SwerveModuleData;

public class SwerveModuleSim implements SwerveModule{

    private final TalonFXSim driveSim;
    private final TalonFXSim turnSim;

    private final PIDController driveFeedback =
      new PIDController(3, 0.0, 0.0, RobotConstantsMAUtil.KDELTA_TIME);
    private final PIDController turnFeedback =
      new PIDController(12, 0.0, 0.0, RobotConstantsMAUtil.KDELTA_TIME);

    private PositionVoltage pidTurnController = new PositionVoltage(0);
    private VelocityVoltage driveController = new VelocityVoltage(0);
    private double rpsDriveSetPoint;


    private SwerveModuleData moduleData = new SwerveModuleData();

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

    public SwerveModuleSim(String moduleNameN) {
        driveSim = new TalonFXSim(DCMotor.getKrakenX60(1), SwerveConstants.DRIVE_GEAR_RATIO, 
        0.025,  SwerveConstants.DRIVE_GEAR_RATIO);

        turnSim = new TalonFXSim(DCMotor.getFalcon500(1), SwerveConstants.TURNING_GEAR_RATIO, 
        0.004,  SwerveConstants.TURNING_GEAR_RATIO);

        turnFeedback.enableContinuousInput(-Math.PI, -Math.PI);
        

        driveSim.setController(driveFeedback);
        turnSim.setController(turnFeedback);

        DrivePosition = new LoggedDouble("/Swerve/Modules/" + moduleNameN + "Sim" + "/Drive Position");
        DriveVelocity = new LoggedDouble("/Swerve/Modules/" + moduleNameN + "Sim" +"/Drive Velocity");
        DriveCurrent = new LoggedDouble("/Swerve/Modules/" + moduleNameN + "Sim" +"/Drive Current");
        DriveVolts = new LoggedDouble("/Swerve/Modules/" + moduleNameN + "Sim" +"/Drive Volts");
        SteerPosition = new LoggedDouble("/Swerve/Modules/" + moduleNameN + "Sim" +"/Steer Position");
        SteerCurrent = new LoggedDouble("/Swerve/Modules/" + moduleNameN + "Sim" +"/Steer Current");
        SteerVolts = new LoggedDouble("/Swerve/Modules/" + moduleNameN + "Sim" +"/Steer Volts");
        AbsAngle = new LoggedDouble("/Swerve/Modules/" + moduleNameN + "Sim" +"/Absolute Angle");
        DriveTemp = new LoggedDouble("/Swerve/Modules/" + moduleNameN + "Sim" +"/Drive Temp");
        SteerTemp = new LoggedDouble("/Swerve/Modules/" + moduleNameN + "Sim" +"/Steer Temp");
    }

    public void resetSteer() {
        
    }

    public double getDriveTemp() {
        return 0;
    }

    public double getSteerTemp() {
        return 0;
    }

    public double getDriveCurrent() {
        return driveSim.getAppliedCurrent();
    }

    public double getSteerCurrent() {
        return turnSim.getAppliedCurrent();
    }

    public double getDriveVolts() {
        return driveSim.getAppliedVoltage();
    }

    public double getSteerVolts() {
        return turnSim.getAppliedVoltage();
    }

    public double getAbsolutePosition() {
        return turnSim.getPosition() * 360 ;
    }

    public double getDrivePosition() {
        //Return distance in meters
        return driveSim.getPosition() * SwerveConstants.WHEEL_CIRCUMFERENCE;
    }

    public double getSteerPosition() {
        //Radians
        return ConvUtil.RotationsToRadians(turnSim.getPosition()) ;
    }

    public double getDriveVelocity() {
        //Meter Per Secound
        return (driveSim.getVelocity() * 60) * Math.PI * (SwerveConstants.WHEEL_RADIUS * 2) / 60;
    }

    public void setNeutralModeDrive(Boolean isBrake) {
        
    }

    public void setNeutralModeTurn(Boolean isBrake) {
        
    }

    public void turningMotorSetPower(double power) {
        turnSim.setControl(new DutyCycleOut(power));
    }

    public void driveMotorSetPower(double power) {
        driveSim.setControl(new DutyCycleOut(power));
    }

    public void turningMotorSetVoltage(double volt) {
        turnSim.setControl(new VoltageOut(volt));
    }

    public void driveMotorSetVoltage(double volt) {
        driveSim.setControl(new VoltageOut(volt));
    }

    public void turningUsingPID(double setPointRdians) {
        turnSim.setControl(pidTurnController.withPosition(Units.radiansToRotations(setPointRdians)));
    }

    public void driveUsingPID(double setPointMPS , double feedForward) {
        //Meter Per Secound setPointMPS / SwerveConstants.WHEEL_CIRCUMFERENCE
        rpsDriveSetPoint = (setPointMPS / SwerveConstants.WHEEL_RADIUS) / (2 * Math.PI );
        driveSim.setControl(driveController.withVelocity(rpsDriveSetPoint)
        .withFeedForward(feedForward)
        .withSlot(SwerveConstants.SLOT_CONFIG));
    }

    public SwerveModuleData update() {
        
        driveSim.update(Timer.getFPGATimestamp());
        turnSim.update(Timer.getFPGATimestamp());
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
            0d, 
            new double[1], 
            new Rotation2d[1]);

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

}
