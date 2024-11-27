package frc.robot.Subsystem.Swerve.IOs;


import java.util.Queue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ma5951.utils.Logger.LoggedDouble;
import com.ma5951.utils.Utils.ConvUtil;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Subsystem.Swerve.PhoenixOdometryThread;
import frc.robot.Subsystem.Swerve.SwerveConstants;
import frc.robot.Subsystem.Swerve.Util.SwerveModule;
import frc.robot.Subsystem.Swerve.Util.SwerveModuleData;

public class SwerveModuleTalonFX implements SwerveModule {

    private final TalonFX driveMotor;
    private final TalonFX turningMotor;
    private final CANcoder absoluteEcoder;

    private TalonFXConfiguration driveConfiguration = new TalonFXConfiguration();
    private TalonFXConfiguration turningConfiguration = new TalonFXConfiguration();
    private SwerveModuleData moduleData = new SwerveModuleData();

    private boolean isDriveMotorReversed;
    private boolean isTurningMotorReversed;
    private double rpsDriveSetPoint;

    //private MotionMagicVoltage turnController = new MotionMagicVoltage(0);
    private PositionVoltage pidTurnController = new PositionVoltage(0);
    private VelocityVoltage driveController = new VelocityVoltage(0);
    


    private StatusSignal<Angle> drivePosition;
    private StatusSignal<AngularVelocity> driveVelocity;
    private StatusSignal<Current> driveCurrent;
    private StatusSignal<Voltage> driveVolts;
    private StatusSignal<Temperature> driveTemp;
    private StatusSignal<Temperature> steerTemp;
    private StatusSignal<Angle> steerPosition;
    private StatusSignal<Current> steerCurrent;
    private StatusSignal<Voltage> steerVolts;
    private StatusSignal<Angle> absAngle;
    private StatusSignal<AngularVelocity> steerVelocitt;
    private String moduleName;

    private LoggedDouble DrivePosition;
    private LoggedDouble DriveVelocity;
    private LoggedDouble DriveCurrent;
    private LoggedDouble DriveVolts;
    private LoggedDouble SteerPosition;
    private LoggedDouble SteerCurrent;
    private LoggedDouble SteerVolts;
    private LoggedDouble AbsAngle;
    private LoggedDouble velociSteer;
    private LoggedDouble DriveTemp;
    private LoggedDouble SteerTemp;

    private final Queue<Double> drivePositionQueue;
    private final Queue<Double> turnPositionQueue;

    

    public SwerveModuleTalonFX(String moduleNameN , int driveID,
            int turningID, int absoluteEncoderID, boolean isDriveMotorReversed,
            boolean isTurningMotorReversed, CANBus canbus) {
        
        this.driveMotor = new TalonFX(driveID, canbus);
        this.turningMotor = new TalonFX(turningID, canbus);
        this.absoluteEcoder = new CANcoder(absoluteEncoderID, canbus);
      

        this.isDriveMotorReversed = isDriveMotorReversed;
        this.isTurningMotorReversed = isTurningMotorReversed;
        this.moduleName = moduleNameN;

        DrivePosition = new LoggedDouble("/Swerve/Modules/" + moduleName + "/Drive Position");
        DriveVelocity = new LoggedDouble("/Swerve/Modules/" + moduleName + "/Drive Velocity");
        DriveCurrent = new LoggedDouble("/Swerve/Modules/" + moduleName + "/Drive Current");
        DriveVolts = new LoggedDouble("/Swerve/Modules/" + moduleName + "/Drive Volts");
        DriveTemp = new LoggedDouble("/Swerve/Modules/" + moduleName + "/Drive Temp");
        SteerTemp = new LoggedDouble("/Swerve/Modules/" + moduleName + "/Steer Temp");
        SteerPosition = new LoggedDouble("/Swerve/Modules/" + moduleName + "/Steer Position");
        SteerCurrent = new LoggedDouble("/Swerve/Modules/" + moduleName + "/Steer Current");
        SteerVolts = new LoggedDouble("/Swerve/Modules/" + moduleName + "/Steer Volts");
        AbsAngle = new LoggedDouble("/Swerve/Modules/" + moduleName + "/Absolute Angle");
        velociSteer = new LoggedDouble("/Swerve/Modules/" + moduleName + "/Steer Velocity");
        

        drivePosition = driveMotor.getPosition();
        driveVelocity = driveMotor.getVelocity();
        driveCurrent = driveMotor.getStatorCurrent();
        driveVolts = driveMotor.getMotorVoltage();
        driveTemp = driveMotor.getDeviceTemp();
        steerTemp = turningMotor.getDeviceTemp();
        steerPosition = turningMotor.getPosition();
        steerCurrent = turningMotor.getStatorCurrent();
        steerVolts = turningMotor.getMotorVoltage();
        absAngle = absoluteEcoder.getAbsolutePosition();
        steerVelocitt = turningMotor.getVelocity();

        driveMotor.setPosition(0);

        drivePositionQueue = PhoenixOdometryThread.getInstance().registerSignal(driveMotor, drivePosition);
        turnPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(turningMotor, steerPosition);
        
        

        configTurningMotor();
        configDriveMotor();
        resetSteer();

        BaseStatusSignal.setUpdateFrequencyForAll(SwerveConstants.ODOMETRY_UPDATE_RATE, 
        drivePosition , steerPosition);
    }

    private void configTurningMotor() {
        
        turningConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        turningConfiguration.Feedback.SensorToMechanismRatio = SwerveConstants.TURNING_GEAR_RATIO;

        turningConfiguration.ClosedLoopGeneral.ContinuousWrap = true;

        turningConfiguration.MotorOutput.Inverted = 
            isTurningMotorReversed ? InvertedValue.Clockwise_Positive : 
            InvertedValue.CounterClockwise_Positive;

        turningConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;


        turningConfiguration.Slot0.kP = SwerveConstants.TURNING_kP;
        turningConfiguration.Slot0.kI = SwerveConstants.TURNING_kI;
        turningConfiguration.Slot0.kD = SwerveConstants.TURNING_kD;
        



        turningConfiguration.CurrentLimits.SupplyCurrentLimitEnable = 
            SwerveConstants.TURNING_ENABLE_CURRENT_LIMIT;
        turningConfiguration.CurrentLimits.SupplyCurrentLimit =
            SwerveConstants.TURNING_CURRENT_LIMIT;
        turningConfiguration.CurrentLimits.SupplyCurrentLowerLimit =
            SwerveConstants.TURNING_CONTINUOUS_LOWER_LIMIT;
        turningConfiguration.CurrentLimits.SupplyCurrentLowerTime = 
            SwerveConstants.TURNING_CONTINUOUS_CURRENT_DURATION;

        turningMotor.getConfigurator().apply(turningConfiguration);

        
    }

    private void configDriveMotor() {
        driveConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        driveConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -80;

        driveConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        driveConfiguration.Feedback.SensorToMechanismRatio = SwerveConstants.DRIVE_GEAR_RATIO;

        driveConfiguration.ClosedLoopRamps.TorqueClosedLoopRampPeriod = SwerveConstants.CLOSED_LOOP_RAMP;

        driveConfiguration.ClosedLoopGeneral.ContinuousWrap = false;

        driveConfiguration.MotorOutput.Inverted = 
            isDriveMotorReversed ? InvertedValue.Clockwise_Positive : 
            InvertedValue.CounterClockwise_Positive;

        driveConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        driveConfiguration.Slot0.kP = SwerveConstants.DRIVE_kP;
        driveConfiguration.Slot0.kI = SwerveConstants.DRIVE_kI;
        driveConfiguration.Slot0.kD = SwerveConstants.DRIVE_kD;
        driveConfiguration.Slot0.kS = SwerveConstants.DRIVE_kS;
        driveConfiguration.Slot0.kV = SwerveConstants.DRIVE_kV;
        driveConfiguration.Slot0.kA = SwerveConstants.DRIVE_kA;


        driveConfiguration.CurrentLimits.SupplyCurrentLimitEnable = 
            SwerveConstants.DRIVE_ENBLE_CURRENT_LIMIT;
        driveConfiguration.CurrentLimits.SupplyCurrentLimit =
            SwerveConstants.DRIVE_CURRENT_LIMIT;
        driveConfiguration.CurrentLimits.SupplyCurrentLowerLimit =
            SwerveConstants.DRIVE_CONTINUOS_LOWER__LIMIT;
        driveConfiguration.CurrentLimits.SupplyCurrentLowerTime = 
            SwerveConstants.DRIVE_CONTINUOUS_CURRENT_DURATION;

        driveMotor.getConfigurator().apply(driveConfiguration);
    }

    public void resetSteer() {
        turningMotor.setPosition((getAbsolutePosition() / 360 ));
    }

    public double getDriveTemp() {
        return driveTemp.getValueAsDouble();
    }

    public double getSteerTemp() {
        return steerTemp.getValueAsDouble();
    }

    public double getDriveCurrent() {
        return driveCurrent.getValueAsDouble();
    }

    public double getSteerCurrent() {
        return steerCurrent.getValueAsDouble();
    }

    public double getDriveVolts() {
        return driveVolts.getValueAsDouble();
    }

    public double getSteerVolts() {
        return steerVolts.getValueAsDouble();
    }

    public double getAbsolutePosition() {
        return absAngle.getValueAsDouble() * 360;
    }

    public double getDrivePosition() {
        //Return distance in meters
        return drivePosition.getValueAsDouble() * SwerveConstants.WHEEL_CIRCUMFERENCE;
    }

    public double getSteerPosition() {
        return ConvUtil.RotationsToRadians(steerPosition.getValueAsDouble());
    }

    public double getDriveVelocity() {
        return (driveVelocity.getValueAsDouble() * 60) * Math.PI * (SwerveConstants.WHEEL_RADIUS * 2) / 60;
    }

    public double getSteerVelocity() {
        return steerVelocitt.getValueAsDouble();
    }

    public void setNeutralModeDrive(Boolean isBrake) {
        driveConfiguration.MotorOutput.NeutralMode = isBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        driveMotor.getConfigurator().apply(driveConfiguration);
    }

    public void setNeutralModeTurn(Boolean isBrake) {
        turningConfiguration.MotorOutput.NeutralMode = isBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        turningMotor.getConfigurator().apply(turningConfiguration);
    }

    public void turningMotorSetPower(double power) {
        turningMotor.set(power);
    }

    public void driveMotorSetPower(double power) {
        driveMotor.set(power);
    }

    public void turningMotorSetVoltage(double volt) {
        turningMotor.setVoltage(volt);
    }

    public void driveMotorSetVoltage(double volt) {
        driveMotor.setVoltage(volt);
    }

    public void turningUsingPID(double setPointRdians) {
        //Degrees
        //turningMotor.setControl(turnController.withPosition(Units.radiansToRotations(setPointRdians)).withSlot(SwerveConstants.SLOT_CONFIG));
        turningMotor.setControl(pidTurnController.withPosition(Units.radiansToRotations(setPointRdians)).withSlot(SwerveConstants.SLOT_CONFIG));
    }
    //In Ammperes
    public void driveUsingPID(double setPointMPS , double feedForward) {
        rpsDriveSetPoint = (setPointMPS / SwerveConstants.WHEEL_RADIUS) / (2 * Math.PI );
        driveMotor.setControl(driveController.withVelocity(rpsDriveSetPoint)
        .withFeedForward(feedForward)
        .withSlot(SwerveConstants.SLOT_CONFIG));

    }

    public void logData(SwerveModuleData swerveModuleData) {
        DrivePosition.update(swerveModuleData.getDrivePosition());
        DriveVelocity.update(swerveModuleData.getDriveVelocity());
        DriveCurrent.update(swerveModuleData.getDriveCurrent());
        DriveVolts.update(swerveModuleData.getDriveVolts());
        SteerPosition.update(swerveModuleData.getSteerPosition() % 360);
        SteerCurrent.update(swerveModuleData.getSteerCurrent());
        SteerVolts.update(swerveModuleData.getSteerVolts());
        AbsAngle.update(swerveModuleData.getAbsoluteAngle());
        velociSteer.update(swerveModuleData.getSteerVelocity());
        DriveTemp.update(swerveModuleData.getDriveTemp());
        SteerTemp.update(swerveModuleData.getSteerTemp());
    }

    public SwerveModuleData update() {
        BaseStatusSignal.refreshAll(
            driveTemp.refresh(),
            steerTemp.refresh(),
            driveCurrent.refresh(),
            steerCurrent.refresh(),
            driveVolts.refresh(),
            steerVolts.refresh(),
            absAngle.refresh(),
            drivePosition.refresh(),
            steerPosition.refresh(),
            driveVelocity.refresh(),
            steerVelocitt.refresh()
        );

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
            drivePositionQueue.stream()
            .mapToDouble(
                signalValue -> Units.rotationsToRadians(signalValue) * SwerveConstants.WHEEL_RADIUS)
            .toArray(), 
            turnPositionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new));

        logData(moduleData);

        drivePositionQueue.clear();
        turnPositionQueue.clear();

        return moduleData;
    }

}