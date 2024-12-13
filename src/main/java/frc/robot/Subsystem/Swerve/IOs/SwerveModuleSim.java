// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Swerve.IOs;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

import static edu.wpi.first.units.Units.*;

import java.util.Arrays;

import com.ma5951.utils.Logger.LoggedDouble;
import frc.robot.Subsystem.Swerve.SwerveConstants;
import frc.robot.Subsystem.Swerve.Util.SwerveModuleData;
import frc.robot.Utils.PhoenixUtil;

public class SwerveModuleSim extends SwerveModuleTalonFX {




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

    public SwerveModuleSim(String moduleNameN , int driveID, int turningID, int absoluteEncoderID, boolean isDriveMotorReversed, boolean isTurningMotorReversed,SwerveModuleSimulation simulation) {
        super(moduleNameN, driveID, turningID, absoluteEncoderID, isDriveMotorReversed, isTurningMotorReversed);
        
        
        moduleSimulation = simulation;
        moduleSimulation.useDriveMotorController(new PhoenixUtil.TalonFXMotorControllerSim(driveMotor ,isDriveMotorReversed ));
        moduleSimulation.useSteerMotorController(new PhoenixUtil.TalonFXMotorControllerWithRemoteCancoderSim(turningMotor, isTurningMotorReversed, absoluteEcoder, false, edu.wpi.first.units.Units.Degrees.of(0)));

        

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



        
    }

    public SwerveModuleData update() {
        refreshAll();


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

}