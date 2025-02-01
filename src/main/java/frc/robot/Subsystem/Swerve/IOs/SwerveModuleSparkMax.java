package frc.robot.Subsystem.Swerve.IOs;

import com.ma5951.utils.Logger.LoggedDouble;
import com.revrobotics.spark.ClosedLoopSlot;
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

    private LoggedDouble TurnSetPointLog;

    private LoggedDouble velocitySetPointLog;;

    private double TurnSetPoint;
    private double VelocitySetPoint;

    public SwerveModuleSparkMax(String moduleName, int driveID, int turningID,
                                boolean isDriveMotorReversed, boolean isTurningMotorReversed) {
        this.driveMotor = new SparkMax(driveID, MotorType.kBrushless);
        this.turningMotor = new SparkMax(turningID, MotorType.kBrushless);

        TurnSetPointLog = new LoggedDouble("/Swerve/SwerveModuleSparkMax/Turn Set Point");
        velocitySetPointLog = new LoggedDouble("/Swerve/SwerveModuleSparkMax/Velocity Set Point");

        this.isDriveMotorReversed = isDriveMotorReversed;
        this.isTurningMotorReversed = isTurningMotorReversed;

        System.out.println("moduleName " + moduleName +
                           " driveID " + driveID + 
                           " turningID " + turningID);

        configureDriveMotor();
        configureTurningMotor();
    }

    private void configureDriveMotor() {
        SparkMaxConfig driveConfig = new SparkMaxConfig();

        driveConfig
            .inverted(isDriveMotorReversed)
            .idleMode(IdleMode.kBrake);

        // IMPORTANT: Make sure these conversion factors 
        // actually convert to "meters" for position and "meters/second" for velocity
        driveConfig.encoder
            .positionConversionFactor(SwerveConstants.DRIVE_POSITION_CONVERSION)  // e.g. meters per rotation
            .velocityConversionFactor(SwerveConstants.DRIVE_VELOCITY_CONVERSION); // e.g. meters/sec per RPM

        // Typical fix: Start with a small feedforward or 0, 
        // and ensure kP is not zero for velocity control
        driveConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(SwerveConstants.DRIVE_kP, SwerveConstants.DRIVE_kI, SwerveConstants.DRIVE_kD)
            .velocityFF(0);  // or just .velocityFF(0.0)

        // Use NO_RESET so we don't keep resetting safe parameters each time
        driveMotor.configure(driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        // If you want them saved to flash so they survive power cycles:
        // driveMotor.configure(driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void configureTurningMotor() {
        SparkMaxConfig turningConfig = new SparkMaxConfig();

        turningConfig
            .inverted(isTurningMotorReversed)
            .idleMode(IdleMode.kBrake);

        // Must convert position from "rotations" to "degrees" (if you’re using degrees)
        turningConfig.encoder
            .positionConversionFactor(SwerveConstants.TURNING_POSITION_CONVERSION)
            .velocityConversionFactor(SwerveConstants.TURNING_VELOCITY_CONVERSION);

        turningConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(SwerveConstants.TURNING_kP, SwerveConstants.TURNING_kI, SwerveConstants.TURNING_kD)
            // For turning, you may also want positionWrapping if the angle crosses 0°-360°:
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0, 360);

        turningMotor.configure(turningConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
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
        // If the turning motor is configured in “degrees,” 
        // convert to radians if your code uses radians externally
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
        // If the turning encoder is in degrees, convert your radian setpoint to degrees:
        double degrees = Units.radiansToDegrees(setPointRadians);
        TurnSetPointLog.update(setPointRadians);
        turningMotor.getClosedLoopController().setReference(degrees, ControlType.kPosition);
        //System.out.println("Turn setpoint [rad->deg]: " + degrees);
        
    }

    @Override
    public void driveUsingPID(double setPointMPS, double feedForward) {  
        driveMotor.getClosedLoopController().setReference(setPointMPS, ControlType.kVelocity , ClosedLoopSlot.kSlot0 ,(setPointMPS/SwerveConstants.MAX_VELOCITY)*12);
        velocitySetPointLog.update(setPointMPS);
        //System.out.println("Drive velocity setpoint (m/s): " + setPointMPS);
    }

    @Override
    public void resetSteer() {
        turningMotor.getEncoder().setPosition(0);
    }

    @Override
    public void setNeutralModeDrive(Boolean isBrake) {
        SparkMaxConfig driveConfig = new SparkMaxConfig();
        driveConfig.idleMode(isBrake ? IdleMode.kBrake : IdleMode.kCoast);
        driveMotor.configure(driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void setNeutralModeTurn(Boolean isBrake) {
        SparkMaxConfig turningConfig = new SparkMaxConfig();
        turningConfig.idleMode(isBrake ? IdleMode.kBrake : IdleMode.kCoast);
        turningMotor.configure(turningConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
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
