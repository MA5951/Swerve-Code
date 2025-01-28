package frc.robot.Subsystem.Swerve.IOs;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.util.Units;
import frc.robot.Subsystem.Swerve.Util.SwerveModule;
import frc.robot.Subsystem.Swerve.Util.SwerveModuleData;
import frc.robot.Subsystem.Swerve.SwerveConstants;

public class SwerveModuleSparkMax implements SwerveModule {

    private final SparkMax driveMotor;
    private final SparkMax turningMotor;
    private final SwerveModuleData moduleData = new SwerveModuleData();

    private final boolean isDriveMotorReversed;
    private final boolean isTurningMotorReversed;

    public SwerveModuleSparkMax(String moduleName, int driveID, int turningID, boolean isDriveMotorReversed, boolean isTurningMotorReversed) {
        this.driveMotor = new SparkMax(driveID, MotorType.kBrushless);
        this.turningMotor = new SparkMax(turningID, MotorType.kBrushless);

        this.isDriveMotorReversed = isDriveMotorReversed;
        this.isTurningMotorReversed = isTurningMotorReversed;

        System.out.println("moduleName " + moduleName + " " + "driveID " + driveID + " " + "turningID " + turningID);

        configureDriveMotor();
        configureTurningMotor();
    } 

    private void configureDriveMotor() {
        SparkMaxConfig driveConfig = new SparkMaxConfig();

        driveConfig
            .inverted(isDriveMotorReversed)
            .idleMode(IdleMode.kBrake);

        driveConfig.encoder
            .positionConversionFactor(SwerveConstants.DRIVE_POSITION_CONVERSION)
            .velocityConversionFactor(SwerveConstants.DRIVE_VELOCITY_CONVERSION);

        driveConfig.closedLoop.velocityFF(10)
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(SwerveConstants.DRIVE_kP, SwerveConstants.DRIVE_kI, SwerveConstants.DRIVE_kD);

        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void configureTurningMotor() {
        SparkMaxConfig turningConfig = new SparkMaxConfig();

        turningConfig
            .inverted(isTurningMotorReversed)
            .idleMode(IdleMode.kBrake);

        turningConfig.encoder
            .positionConversionFactor(SwerveConstants.TURNING_POSITION_CONVERSION)
            .velocityConversionFactor(SwerveConstants.TURNING_VELOCITY_CONVERSION);

        turningConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(SwerveConstants.TURNING_kP, SwerveConstants.TURNING_kI, SwerveConstants.TURNING_kD);

        turningMotor.configure(turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public double getDrivePosition() {
        return driveMotor.getEncoder().getPosition();
    }

    @Override
    public double getDriveVelocity() {
        return driveMotor.getEncoder().getVelocity();
    }

    @Override
    public double getSteerPosition() {
        return Units.degreesToRadians(turningMotor.getEncoder().getPosition());
    }

    public double getSteerVelocity() {
        return turningMotor.getEncoder().getVelocity();
    }

    @Override
    public double getDriveTemp() {
        return driveMotor.getMotorTemperature();
    }

    @Override
    public double getSteerTemp() {
        return turningMotor.getMotorTemperature();
    }

    @Override
    public void turningUsingPID(double setPointRadians) {
        turningMotor.getClosedLoopController().setReference(Units.radiansToDegrees(setPointRadians), ControlType.kPosition);
        System.out.println(setPointRadians);
    }   

    @Override
    public void driveUsingPID(double setPointMPS, double feedForward) {  
        // System.out.println(setPointMPS);    
        // driveMotor.set(setPointMPS);
        driveMotor.getClosedLoopController().setReference(setPointMPS, ControlType.kVelocity);
    }

    @Override
    public void resetSteer() {
        turningMotor.getEncoder().setPosition(0);
    }

    @Override
    public void setNeutralModeDrive(Boolean isBrake) {
        SparkMaxConfig driveConfig = new SparkMaxConfig();
        driveConfig.idleMode(isBrake ? IdleMode.kBrake : IdleMode.kCoast);
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setNeutralModeTurn(Boolean isBrake) {
        SparkMaxConfig turningConfig = new SparkMaxConfig();
        turningConfig.idleMode(isBrake ? IdleMode.kBrake : IdleMode.kCoast);
        turningMotor.configure(turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void driveMotorSetVoltage(double volt) {
        driveMotor.setVoltage(volt);
    }

    @Override
    public void turningMotorSetVoltage(double volt) {
        turningMotor.setVoltage(volt);
    }

    @Override
    public SwerveModuleData update() {
        moduleData.updateData(
            getDrivePosition(),
            getDriveVelocity(),
            driveMotor.getOutputCurrent(),
            driveMotor.getBusVoltage(),
            getDriveTemp(),
            getSteerTemp(),
            getSteerPosition(),
            turningMotor.getOutputCurrent(),
            turningMotor.getBusVoltage(),
            getAbsolutePosition(),
            getSteerVelocity(),
            null, // Placeholder for drive position queue
            null  // Placeholder for turn position queue
        );

        return moduleData;
    }

    public double getAbsolutePosition() {
        return turningMotor.getEncoder().getPosition();
    }

    @Override
    public void driveMotorSetPower(double power) {
        driveMotor.set(power);
    }

    @Override
    public void turningMotorSetPower(double power) {
        turningMotor.set(power);
    }
}
