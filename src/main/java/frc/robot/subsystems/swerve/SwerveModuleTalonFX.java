package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ma5951.utils.RobotConstants;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class SwerveModuleTalonFX extends SwerveModule {

    private final TalonFX driveMotor;
    private final TalonFX turningMotor;

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(SwerveConstants.DRIVE_KS,
            SwerveConstants.DRIVE_KV);

    private final CANcoder absoluteEcoder;

    private final boolean isAbsoluteEncoderReversed;
    private final double offsetEncoder;

    private PositionVoltage m_angleSetter = new PositionVoltage(0);
    private VelocityTorqueCurrentFOC m_velocitySetter = new VelocityTorqueCurrentFOC(0);

    private StatusSignal<Double> m_drivePosition;
    private StatusSignal<Double> m_driveVelocity;
    private StatusSignal<Double> m_steerPosition;
    private StatusSignal<Double> m_steerVelocity;

    public SwerveModuleTalonFX(String tabName, int driveID,
            int turningID, int absoluteEncoderID, boolean isDriveMotorReversed,
            boolean isTurningMotorReversed, boolean isAbsoluteEncoderReversed,
            double offsetEncoder) {
        this.absoluteEcoder = new CANcoder(absoluteEncoderID);

        this.offsetEncoder = offsetEncoder;
        this.isAbsoluteEncoderReversed = isAbsoluteEncoderReversed;

        this.driveMotor = new TalonFX(driveID);
        this.turningMotor = new TalonFX(turningID);

        driveMotor.setControl(new StaticBrake());
        turningMotor.setControl(new StaticBrake());

        driveMotor.setInverted(isDriveMotorReversed);
        turningMotor.setInverted(isTurningMotorReversed);

        configTurningMotor();
        configDriveMotor();
        resetEncoders();

        m_drivePosition = driveMotor.getPosition();
        m_driveVelocity = driveMotor.getVelocity();
        m_steerPosition = turningMotor.getPosition();
        m_steerVelocity = turningMotor.getVelocity();
    }

    private void configTurningMotor() {
        TalonFXConfiguration turningConfiguration = new TalonFXConfiguration();

        turningConfiguration.Slot0.kP = SwerveConstants.turningPIDKP;
        turningConfiguration.Slot0.kI = SwerveConstants.turningPIDKI;
        turningConfiguration.Slot0.kD = SwerveConstants.turningPIDKD;
        turningConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = SwerveConstants.turningPeakCurrentLimit;
        turningConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -SwerveConstants.turningPeakCurrentLimit;
        turningMotor.getConfigurator().apply(turningConfiguration);
    }

    private void configDriveMotor() {

        TalonFXConfiguration driveConfiguration = new TalonFXConfiguration();

        driveConfiguration.Slot0.kP = SwerveConstants.turningPIDKP;
        driveConfiguration.Slot0.kI = SwerveConstants.turningPIDKI;
        driveConfiguration.Slot0.kD = SwerveConstants.turningPIDKD;

        driveConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = SwerveConstants.DRIVE_PEAK_CURRENT_LIMIT;
        driveConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -SwerveConstants.DRIVE_PEAK_CURRENT_LIMIT;

        driveMotor.getConfigurator().apply(driveConfiguration);
    }

    public void setAccelerationLimit(double limit) {
        TalonFXConfiguration driveConfiguration = new TalonFXConfiguration();
        ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs();

        closedLoopRampsConfigs.DutyCycleClosedLoopRampPeriod = limit;

        driveMotor.getConfigurator().apply(driveConfiguration);
    }

    public double getAbsoluteEncoderPosition() {
        absoluteEcoder.getAbsolutePosition().refresh();
        return isAbsoluteEncoderReversed ? 360 - absoluteEcoder.getAbsolutePosition().getValue()
                : absoluteEcoder.getAbsolutePosition().getValue();
    }

    public double getDrivePosition() {
        m_drivePosition.refresh();
        return m_drivePosition.getValue()
                * SwerveConstants.DISTANCE_PER_PULSE;
    }

    public double getTurningPosition() {
        m_steerPosition.refresh();
        return m_steerPosition.getValue()
                * SwerveConstants.ANGLE_PER_PULSE;
    }

    public double getDriveVelocity() {
        m_driveVelocity.refresh();
        return (m_driveVelocity.getValue()
                * SwerveConstants.DISTANCE_PER_PULSE) /
                SwerveConstants.VELOCITY_TIME_UNIT_IN_SECONDS;
    }

    public double getTurningVelocity() {
        m_steerVelocity.refresh();
        return (m_steerPosition.getValue()
                * SwerveConstants.ANGLE_PER_PULSE) /
                SwerveConstants.VELOCITY_TIME_UNIT_IN_SECONDS;
    }

    public void resetEncoders() {
        FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
        feedbackConfigs.FeedbackRotorOffset = (getAbsoluteEncoderPosition() - offsetEncoder) /
                SwerveConstants.ANGLE_PER_PULSE;
        turningMotor.getConfigurator().apply(feedbackConfigs);
    }

    public void turningMotorSetPower(double power) {
        turningMotor.set(power);
    }

    public void driveMotorSetPower(double power) {
        driveMotor.set(power);
    }

    public void setInvertedTurning(Boolean turningMode) {
        turningMotor.setInverted(turningMode);
    }

    public void turningUsingPID(double setPoint) {
        turningMotor.setControl(
                m_angleSetter.withPosition(setPoint
                        / SwerveConstants.ANGLE_PER_PULSE)
                        .withSlot(0));
    }

    public void driveUsingPID(double setPoint) {
        driveMotor.setControl(
                m_velocitySetter.withFeedForward(
                        feedforward.calculate(
                                getDriveVelocity(), setPoint, RobotConstants.KDELTA_TIME))
                        .withSlot(0).withVelocity(setPoint));
    }
}