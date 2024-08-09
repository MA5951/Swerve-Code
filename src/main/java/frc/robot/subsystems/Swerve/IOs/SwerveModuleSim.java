// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve.IOs;

import com.ma5951.utils.RobotConstants;
import com.ma5951.utils.Logger.LoggedDouble;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.Swerve.SwerveConstants;
import frc.robot.subsystems.Swerve.Util.SwerveModule;

public class SwerveModuleSim implements SwerveModule{

    private final DCMotorSim driveSim;
    private final DCMotorSim turnSim;

    private final PIDController driveFeedback =
      new PIDController(38, 0.0, 0.0, RobotConstants.KDELTA_TIME);
    private final PIDController turnFeedback =
      new PIDController(0.08, 0.0, 0.0, RobotConstants.KDELTA_TIME);

    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;
    private double drivePose = 0;

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
        driveSim = new DCMotorSim(DCMotor.getKrakenX60Foc(1), SwerveConstants.DRIVE_GEAR_RATIO, 0.025);
        turnSim = new DCMotorSim(DCMotor.getFalcon500Foc(1), SwerveConstants.TURNING_GEAR_RATIO, 0.004);


        turnFeedback.enableContinuousInput(0, 360);

        DrivePosition = new LoggedDouble("/Swerve/Modules/" + moduleNameN + "Sim" + "/Drive Position");
        DriveVelocity = new LoggedDouble("/Swerve/Modules/" + moduleNameN + "Sim" +"/Drive Velocity");
        DriveCurrent = new LoggedDouble("/Swerve/Modules/" + moduleNameN + "Sim" +"/Drive Current");
        DriveVolts = new LoggedDouble("/Swerve/Modules/" + moduleNameN + "Sim" +"/Drive Volts");
        SteerPosition = new LoggedDouble("/Swerve/Modules/" + moduleNameN + "Sim" +"/Steer Position");
        SteerCurrent = new LoggedDouble("/Swerve/Modules/" + moduleNameN + "Sim" +"/Steer Current");
        SteerVolts = new LoggedDouble("/Swerve/Modules/" + moduleNameN + "Sim" +"/Steer Volts");
        AbsAngle = new LoggedDouble("/Swerve/Modules/" + moduleNameN + "Sim" +"/Absolute Angle");
        DriveTemp = new LoggedDouble("/Swerve/Modules/" + moduleNameN + "/Drive Temp");
        SteerTemp = new LoggedDouble("/Swerve/Modules/" + moduleNameN + "/Steer Temp");
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

    public double getAbsoluteEncoderPosition() {
        return turnSim.getAngularPositionRotations() * 360 ;
    }

    public double getDrivePosition() {
        //Return distance in meters
        drivePose += driveSim.getAngularPositionRotations() * SwerveConstants.WHEEL_CIRCUMFERENCE * RobotConstants.KDELTA_TIME;
        return drivePose;
       
    }

    public double getTurningPosition() {
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

    public void turningUsingPID(double setPointDEGREES) {
        //Degrees
        turnSim.setInputVoltage(turnFeedback.calculate(getTurningPosition(), setPointDEGREES));
    }

    public void driveUsingPID(double setPointMPS) {
        //Meter Per Secound setPointMPS / SwerveConstants.WHEEL_CIRCUMFERENCE
        driveSim.setInputVoltage(driveFeedback.calculate(getDriveVelocity(), setPointMPS));
    }

    public void update() {
        
        DrivePosition.update(getDrivePosition());
        DriveVelocity.update(getDriveVelocity());
        DriveCurrent.update(getDriveCurrent());
        DriveVolts.update(getDriveVolts());
        SteerPosition.update(getTurningPosition());
        SteerCurrent.update(getSteerCurrent());
        SteerVolts.update(getSteerVolts());
        AbsAngle.update(getAbsoluteEncoderPosition());
        DriveTemp.update(getDriveTemp());
        SteerTemp.update(getSteerTemp());

        driveSim.update(RobotConstants.KDELTA_TIME);
        turnSim.update(RobotConstants.KDELTA_TIME);

    }

}
