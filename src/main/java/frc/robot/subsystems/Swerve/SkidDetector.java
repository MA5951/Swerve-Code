// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;



import java.util.function.Supplier;

import com.ma5951.utils.Logger.LoggedDouble;
import com.ma5951.utils.Utils.VectorUtil;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Add your docs here. */
public class SkidDetector {
    private SwerveDriveKinematics kinematics;
    private Supplier<SwerveModuleState[]> statesSupplier;
    private SwerveModuleState[] rotationalStates;
    private double[] statesTranslationMag =  new double[4];
    private double angularVelocity;
    private double maxTranslationSpeed = 0;
    private double minTranslationSpeed = 0;

    private LoggedDouble skidRatio;

    public SkidDetector(SwerveDriveKinematics kinematics,Supplier<SwerveModuleState[]> states) {
        this.kinematics = kinematics;
        statesSupplier = states;

        skidRatio = new LoggedDouble("/Swerve/Skid Detector");

    }

    public void update() {
        skidRatio.update(getSkidRatio());
    }

    public double getSkidRatio() {
        angularVelocity = kinematics.toChassisSpeeds(statesSupplier.get()).omegaRadiansPerSecond;
        rotationalStates = SwerveSubsystem.getInstance().generateStates(new ChassisSpeeds(0, 0, angularVelocity), SwerveConstants.optimize);
        
        for (int i = 0; i < statesSupplier.get().length; i++ ) {
            final Translation2d swerveStateAsVector = VectorUtil.getVectorFromSwerveState(statesSupplier.get()[i]),
                                swerveStateRotatinAsVector = VectorUtil.getVectorFromSwerveState(rotationalStates[i]),
                                swerveStateTranslationAsVector = swerveStateAsVector.minus(swerveStateRotatinAsVector);
            statesTranslationMag[i] = swerveStateTranslationAsVector.getNorm();
        }

        maxTranslationSpeed = Math.max(statesTranslationMag[0], Math.max(statesTranslationMag[1] , Math.max(statesTranslationMag[2], statesTranslationMag[3])));
        minTranslationSpeed = Math.min(statesTranslationMag[0], Math.min(statesTranslationMag[1] , Math.min(statesTranslationMag[2], statesTranslationMag[3])));
        
        return maxTranslationSpeed / minTranslationSpeed;
    }

}
