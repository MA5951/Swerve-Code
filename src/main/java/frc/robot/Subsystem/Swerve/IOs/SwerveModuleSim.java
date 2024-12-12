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

    private double rpsDriveSetPoint;

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

                this.driveController = new PIDController(0.05, 0.0, 0.0);
                this.turnController = new PIDController(8.0, 0.0, 0.0);
        
                // Enable wrapping for turn PID
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

        moduleSimulation.useDriveMotorController(
                new PhoenixUtil.TalonFXMotorControllerSim(driveMotor, isDriveMotorReversed));

        moduleSimulation.useSteerMotorController(new PhoenixUtil.TalonFXMotorControllerWithRemoteCancoderSim(
                turningMotor,
                isTurningMotorReversed,
                absoluteEcoder,
                false,
                Rotations.of(0)));
        
    }

    public SwerveModuleData update() {

        driveSim.update(Timer.getFPGATimestamp());
        turnSim.update(Timer.getFPGATimestamp());


        moduleData.updateData(
            moduleSimulation.getDriveWheelFinalPosition().in(Revolutions),//todo
            moduleSimulation.getDriveWheelFinalSpeed().in(RevolutionsPerSecond),//todo
            moduleSimulation.getDriveMotorStatorCurrent().in(Amps),
            moduleSimulation.getDriveMotorAppliedVoltage().in(Volts),
            getDriveTemp(),
            getSteerTemp(),
            getSteerPosition(),
            moduleSimulation.getSteerMotorStatorCurrent().in(Amps),
            moduleSimulation.getSteerMotorAppliedVoltage().in(Volts),
            moduleSimulation.getSteerAbsoluteFacing().getDegrees(),
            moduleSimulation.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond),
            Arrays.stream(moduleSimulation.getCachedDriveWheelFinalPositions())
                .mapToDouble(angle -> angle.in(Revolutions))
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