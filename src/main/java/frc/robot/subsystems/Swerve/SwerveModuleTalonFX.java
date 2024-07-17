package frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class SwerveModuleTalonFX extends SwerveModule {

    private final TalonFX driveMotor;
    private final TalonFX turningMotor;
    private final CANcoder absoluteEcoder;

    private TalonFXConfiguration driveConfig = new TalonFXConfiguration();
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
    private StatusSignal<Double> steerPosition;
    private StatusSignal<Double> absAngle;

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(SwerveConstants.DRIVE_KS,
            SwerveConstants.DRIVE_KV);

    private double angleOffset;
    private double dirvePoseOffset;

    //Absolute Position
    //Velocity
    //Current
    //Relativ Position

    public SwerveModuleTalonFX(int driveID,
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

        
       

        drivePosition = driveMotor.getPosition();
        driveVelocity = driveMotor.getVelocity();
        steerPosition = turningMotor.getPosition();
        absAngle = absoluteEcoder.getAbsolutePosition();
        

        configTurningMotor();
        configDriveMotor();
        configCANCoder();
        resetEncoders();
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
        TalonFXConfiguration driveConfiguration = new TalonFXConfiguration();

        driveConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        driveConfiguration.ClosedLoopGeneral.ContinuousWrap = false;

        driveConfiguration.MotorOutput.Inverted = 
            isDriveMotorReversed ? InvertedValue.Clockwise_Positive : 
            InvertedValue.CounterClockwise_Positive;

        driveConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        driveConfiguration.Slot0.kP = SwerveConstants.DRIVE_PID_KP;
        driveConfiguration.Slot0.kI = SwerveConstants.DRIVE_PID_KI;
        driveConfiguration.Slot0.kD = SwerveConstants.DRIVE_PID_KD;

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

    //Turn to signal
    public double getCurrent() { //CHANGE NAME AND ADD STEER
        return driveMotor.getStatorCurrent().getValue();
    }

    public double getAbsoluteEncoderPosition() {
        absAngle.refresh();
        return absAngle.getValueAsDouble();
    }

    public double getDrivePosition() {
        drivePosition.refresh();
        return drivePosition.getValueAsDouble();
    }

    /*
     * Get Relativ Turn Position 
     * 
     * @return Degrees Of The Wheel 
     */
    public double getTurningPosition() {
        steerPosition.refresh();
        return steerPosition.getValueAsDouble() * 360;
    }

    public double getDriveVelocity() {
        driveVelocity.refresh();
        return driveVelocity.getValue();
    }

    public void setNutralModeDrive(Boolean isBrake) {
        TalonFXConfiguration driveConfiguration = new TalonFXConfiguration();
        driveConfiguration.MotorOutput.NeutralMode = isBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        driveMotor.getConfigurator().apply(driveConfiguration);
    }

    //Add setBrake to steer
    //Make Signal standarts and make internal signal standart
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

    public void setInvertedTurning(Boolean turningMode) {
        turningMotor.setInverted(turningMode);
    }

    public void turningUsingPID(double setPoint) {
        //Degrees
        turningMotor.setControl(turnController.withPosition(setPoint / 360).withSlot(SwerveConstants.SLOT_CONFIG));
    }

    public void driveUsingPID(double setPoint) {
        driveMotor.setControl();
    }

    public void update() {
        


    }
}