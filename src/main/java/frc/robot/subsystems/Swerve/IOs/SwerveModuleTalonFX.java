package frc.robot.subsystems.Swerve.IOs;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ma5951.utils.Logger.LoggedDouble;

import frc.robot.subsystems.Swerve.SwerveConstants;
import frc.robot.subsystems.Swerve.Util.SwerveModule;

public class SwerveModuleTalonFX extends SwerveModule {

    private final TalonFX driveMotor;
    private final TalonFX turningMotor;
    private final CANcoder absoluteEcoder;

    private TalonFXConfiguration driveConfiguration = new TalonFXConfiguration();
    private TalonFXConfiguration turningConfiguration = new TalonFXConfiguration();
    private CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();

    private double canCoderOffset;
    private boolean isDriveMotorReversed;
    private boolean isTurningMotorReversed;
    private boolean isAbsoluteEncoderReversed;

    private MotionMagicTorqueCurrentFOC turnController = new MotionMagicTorqueCurrentFOC(0);
    private VelocityTorqueCurrentFOC driveController = new VelocityTorqueCurrentFOC(0);



    private StatusSignal<Double> drivePosition;
    private StatusSignal<Double> driveVelocity;
    private StatusSignal<Double> driveCurrent;
    private StatusSignal<Double> driveVolts;
    private StatusSignal<Double> steerPosition;
    private StatusSignal<Double> steerCurrent;
    private StatusSignal<Double> steerVolts;
    private StatusSignal<Double> absAngle;
    private String moduleName;

    private LoggedDouble DrivePosition;
    private LoggedDouble DriveVelocity;
    private LoggedDouble DriveCurrent;
    private LoggedDouble DriveVolts;
    private LoggedDouble SteerPosition;
    private LoggedDouble SteerCurrent;
    private LoggedDouble SteerVolts;
    private LoggedDouble AbsAngle;

    

    public SwerveModuleTalonFX(String moduleNameN , int driveID,
            int turningID, int absoluteEncoderID, boolean isDriveMotorReversed,
            boolean isTurningMotorReversed,boolean isCancoderInverted,
            double offsetEncoder, String canbus) {
        
        this.driveMotor = new TalonFX(driveID, canbus);
        this.turningMotor = new TalonFX(turningID, canbus);
        this.absoluteEcoder = new CANcoder(absoluteEncoderID, canbus);
      

        this.isDriveMotorReversed = isDriveMotorReversed;
        this.isTurningMotorReversed = isTurningMotorReversed;
        this.isAbsoluteEncoderReversed = isCancoderInverted;
        this.canCoderOffset = offsetEncoder;
        this.moduleName = moduleNameN;

        DrivePosition = new LoggedDouble("/Swerve/" + moduleName + "/Drive Position");
        DriveVelocity = new LoggedDouble("/Swerve/" + moduleName + "/Drive Velocity");
        DriveCurrent = new LoggedDouble("/Swerve/" + moduleName + "/Drive Current");
        DriveVolts = new LoggedDouble("/Swerve/" + moduleName + "/Drive Volts");
        SteerPosition = new LoggedDouble("/Swerve/" + moduleName + "/Steer Position");
        SteerCurrent = new LoggedDouble("/Swerve/" + moduleName + "/Steer Current");
        SteerVolts = new LoggedDouble("/Swerve/" + moduleName + "/Steer Volts");
        AbsAngle = new LoggedDouble("/Swerve/" + moduleName + "/Absolute Angle");
       

        drivePosition = driveMotor.getPosition();
        driveVelocity = driveMotor.getVelocity();
        driveCurrent = driveMotor.getStatorCurrent();
        driveVolts = driveMotor.getMotorVoltage();
        steerPosition = turningMotor.getPosition();
        steerCurrent = turningMotor.getStatorCurrent();
        steerVolts = turningMotor.getMotorVoltage();
        absAngle = absoluteEcoder.getAbsolutePosition();
        

        configTurningMotor();
        configDriveMotor();
        configCANCoder();
    }

    private void configTurningMotor() {
        turningConfiguration.Feedback.FeedbackRemoteSensorID = absoluteEcoder.getDeviceID();
        turningConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
        turningConfiguration.Feedback.RotorToSensorRatio = 1;
        turningConfiguration.Feedback.SensorToMechanismRatio = SwerveConstants.TURNING_GEAR_RATIO;

        turningConfiguration.ClosedLoopGeneral.ContinuousWrap = false;

        turningConfiguration.MotorOutput.Inverted = 
            isTurningMotorReversed ? InvertedValue.Clockwise_Positive : 
            InvertedValue.CounterClockwise_Positive;

        turningConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;


        turningConfiguration.Slot0.kP = SwerveConstants.TURNING_kP;
        turningConfiguration.Slot0.kI = SwerveConstants.TURNING_kI;
        turningConfiguration.Slot0.kD = SwerveConstants.TURNING_kD;
        turningConfiguration.MotionMagic.MotionMagicCruiseVelocity = SwerveConstants.TURNING_CTUISE_VELOCITY;
        turningConfiguration.MotionMagic.MotionMagicAcceleration = SwerveConstants.TURNING_ACCELERATION;
        turningConfiguration.MotionMagic.MotionMagicJerk = SwerveConstants.TURNING_JERK;


        turningConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 
            SwerveConstants.TURNING_PEAK_CURRENT_LIMIT_TORQUE_CURRENT;
        turningConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = 
            -SwerveConstants.TURNING_PEAK_CURRENT_LIMIT_TORQUE_CURRENT;

        turningConfiguration.CurrentLimits.SupplyCurrentLimitEnable = 
            SwerveConstants.TURNING_ENABLE_CURRENT_LIMIT;
        turningConfiguration.CurrentLimits.SupplyCurrentLimit =
            SwerveConstants.TURNING_CONTINUOUS_CURRENT_LIMIT;
        turningConfiguration.CurrentLimits.SupplyCurrentThreshold =
            SwerveConstants.TURNING_PEAK_CURRENT_LIMIT;
        turningConfiguration.CurrentLimits.SupplyTimeThreshold = 
            SwerveConstants.TURNING_PEAK_CURRENT_DURATION;
        
        turningMotor.getConfigurator().apply(turningConfiguration);
    }

    private void configDriveMotor() {
        driveConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        driveConfiguration.Feedback.SensorToMechanismRatio = SwerveConstants.DRIVE_GEAR_RATIO;

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


        driveConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 
            SwerveConstants.DRIVE_PEAK_CURRENT_LIMIT_TORQUE_CURRENT;
        driveConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = 
            -SwerveConstants.DRIVE_PEAK_CURRENT_LIMIT_TORQUE_CURRENT;

        driveConfiguration.CurrentLimits.SupplyCurrentLimitEnable = 
            SwerveConstants.DRIVE_ENBLE_CURRENT_LIMIT;
        driveConfiguration.CurrentLimits.SupplyCurrentLimit = 
            SwerveConstants.DRIVE_CONTINUOS_CURRENT_LIMIT;
        driveConfiguration.CurrentLimits.SupplyCurrentThreshold = 
            SwerveConstants.DRIVE_PEAK_CURRENT_LIMIT;
        driveConfiguration.CurrentLimits.SupplyTimeThreshold = 
            SwerveConstants.DRIVE_PEAK_CURRENT_DURATION;

        driveMotor.getConfigurator().apply(driveConfiguration);
    }

    private void configCANCoder() {
        canCoderConfig.MagnetSensor.MagnetOffset = canCoderOffset / 360; //Convert between angles to rotations
        
        canCoderConfig.MagnetSensor.SensorDirection = 
            isAbsoluteEncoderReversed ? SensorDirectionValue.Clockwise_Positive :
            SensorDirectionValue.CounterClockwise_Positive;

        absoluteEcoder.getConfigurator().apply(canCoderConfig);
    }

    public double getDriveCurrent() {
        driveCurrent.refresh();
        return driveCurrent.getValueAsDouble();
    }

    public double getSteerCurrent() {
        steerCurrent.refresh();
        return steerCurrent.getValueAsDouble();
    }

    public double getDriveVolts() {
        driveVolts.refresh();
        return driveVolts.getValueAsDouble();
    }

    public double getSteerVolts() {
        steerVolts.refresh();
        return steerVolts.getValueAsDouble();
    }

    public double getAbsoluteEncoderPosition() {
        absAngle.refresh();
        return absAngle.getValueAsDouble();
    }

    public double getDrivePosition() {
        //Return distance in meters
        drivePosition.refresh();
        return drivePosition.getValueAsDouble() * SwerveConstants.WHEEL_CIRCUMFERENCE;
    }

    public double getTurningPosition() {
        //Degrees
        steerPosition.refresh();
        return steerPosition.getValueAsDouble() * 360;
    }

    public double getDriveVelocity() {
        //Meter Per Secound
        driveVelocity.refresh();
        return driveVelocity.getValue() * SwerveConstants.WHEEL_CIRCUMFERENCE;
    }

    public void setNutralModeDrive(Boolean isBrake) {
        driveConfiguration.MotorOutput.NeutralMode = isBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        driveMotor.getConfigurator().apply(driveConfiguration);
    }

    public void setNutralModeTurn(Boolean isBrake) {
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

    public void turningUsingPID(double setPointDEGREES) {
        //Degrees
        turningMotor.setControl(turnController.withPosition(setPointDEGREES / 360).withSlot(SwerveConstants.SLOT_CONFIG));
    }

    public void driveUsingPID(double setPointMPS) {
        //Meter Per Secound
        driveMotor.setControl(driveController.withVelocity(setPointMPS / SwerveConstants.WHEEL_CIRCUMFERENCE).withSlot(SwerveConstants.SLOT_CONFIG));
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

    }

}