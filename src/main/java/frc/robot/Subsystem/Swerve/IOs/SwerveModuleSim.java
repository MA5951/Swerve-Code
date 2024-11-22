// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Swerve.IOs;

import com.ma5951.utils.RobotConstantsMAUtil;
import com.ma5951.utils.Logger.LoggedDouble;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Subsystem.Swerve.SwerveConstants;
import frc.robot.Subsystem.Swerve.Util.SwerveModule;
import frc.robot.Subsystem.Swerve.Util.SwerveModuleData;

public class SwerveModuleSim implements SwerveModule{

    private final DCMotorSim driveSim;
    private final DCMotorSim turnSim;

    private final PIDController driveFeedback =
      new PIDController(38, 0.0, 0.0, RobotConstantsMAUtil.KDELTA_TIME);
    private final PIDController turnFeedback =
      new PIDController(20, 0.0, 0.0, RobotConstantsMAUtil.KDELTA_TIME);

    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;
    private double drivePose = 0;

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
        driveSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(SwerveConstants.DRIVE_kV, SwerveConstants.DRIVE_kA), DCMotor.getKrakenX60Foc(1) , new double[] {});
        turnSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(SwerveConstants.TURNING_kV, SwerveConstants.TURNING_kA), DCMotor.getFalcon500(1), null);


        turnFeedback.enableContinuousInput(0, 360);

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
        return Math.abs(driveSim.getCurrentDrawAmps());
    }

    public double getSteerCurrent() {
        return Math.abs(turnSim.getCurrentDrawAmps());
    }

    public double getDriveVolts() {
        return driveAppliedVolts;
    }

    public double getSteerVolts() {
        return turnAppliedVolts;
    }

    public double getAbsolutePosition() {
        return turnSim.getAngularPositionRotations() * 360 ;
    }

    public double getDrivePosition() {
        //Return distance in meters
        drivePose += driveSim.getAngularPositionRotations() * SwerveConstants.WHEEL_CIRCUMFERENCE * RobotConstantsMAUtil.KDELTA_TIME;
        return drivePose;
       
    }

    public double getSteerPosition() {
        //Degrees
        return  turnSim.getAngularPositionRotations() * 360 ;
    }

    public double getDriveVelocity() {
        //Meter Per Secound
        return driveSim.getAngularPositionRotations() * SwerveConstants.WHEEL_CIRCUMFERENCE;
    }

    public void setNeutralModeDrive(Boolean isBrake) {
        
    }

    public void setNeutralModeTurn(Boolean isBrake) {
        
    }

    public void turningMotorSetPower(double power) {
        turnSim.setInputVoltage(12 * power);
    }

    public void driveMotorSetPower(double power) {
        driveSim.setInputVoltage(12 * power);
    }

    public void turningMotorSetVoltage(double volt) {
        turnAppliedVolts = MathUtil.clamp(volt, -12.0, 12.0);
        turnSim.setInputVoltage(turnAppliedVolts);
    }

    public void driveMotorSetVoltage(double volt) {
        driveAppliedVolts = MathUtil.clamp(volt, -12.0, 12.0);
        driveSim.setInputVoltage(driveAppliedVolts);
    }

    public void turningUsingPID(double setPointRdians) {
        
        turnSim.setInputVoltage(turnFeedback.calculate(Units.degreesToRadians(getSteerPosition()), setPointRdians));
    }

    public void driveUsingPID(double setPointMPS , double feedforward) {
        //Meter Per Secound setPointMPS / SwerveConstants.WHEEL_CIRCUMFERENCE
        driveSim.setInputVoltage(driveFeedback.calculate(getDriveVelocity(), setPointMPS));
    }

    public SwerveModuleData update() {
        
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

        driveSim.update(RobotConstantsMAUtil.KDELTA_TIME);
        turnSim.update(RobotConstantsMAUtil.KDELTA_TIME);

        return moduleData;

    }

}
