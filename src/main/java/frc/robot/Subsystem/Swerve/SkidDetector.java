// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Swerve;

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
    private ChassisSpeeds measurSpeeds;
    private SwerveModuleState[] swerveStatesRotationalPart;
    private double[] swerveStatesTranslationalPartMagnitudes = new double[kinematics.getModules().length];
    private Translation2d swerveStateMeasuredAsVector;
    private Translation2d swerveStatesRotationalPartAsVector;
    private Translation2d swerveStatesTranslationalPartAsVector;
    private double maximumTranslationalSpeed = 0;
    private double minimumTranslationalSpeed = Double.POSITIVE_INFINITY;

    private LoggedDouble skidRatio;

    public SkidDetector(SwerveDriveKinematics kinematics,Supplier<SwerveModuleState[]> states) {
        this.kinematics = kinematics;
        statesSupplier = states;

        skidRatio = new LoggedDouble("/Swerve/Skid Detector/Skid Ration");

    }

    public void update() {
        skidRatio.update(getSkiddingRatio(statesSupplier.get() , kinematics));
    }

    public double getSkiddingRatio(
            SwerveModuleState[] swerveStatesMeasured, SwerveDriveKinematics swerveDriveKinematics) {
        measurSpeeds = swerveDriveKinematics.toChassisSpeeds(swerveStatesMeasured);
        measurSpeeds.vxMetersPerSecond = 0;
        measurSpeeds.vyMetersPerSecond = 0;
        swerveStatesRotationalPart = swerveDriveKinematics.toSwerveModuleStates( measurSpeeds );
        
        for (int i = 0; i < swerveStatesMeasured.length; i++) {
            swerveStateMeasuredAsVector = VectorUtil.getVectorFromSwerveState(swerveStatesMeasured[i]);
            swerveStatesRotationalPartAsVector = VectorUtil.getVectorFromSwerveState(swerveStatesRotationalPart[i]);
            swerveStatesTranslationalPartAsVector = swerveStateMeasuredAsVector.minus(swerveStatesRotationalPartAsVector);
            swerveStatesTranslationalPartMagnitudes[i] = swerveStatesTranslationalPartAsVector.getNorm();
        }

        
        for (double translationalSpeed : swerveStatesTranslationalPartMagnitudes) {
            maximumTranslationalSpeed = Math.max(maximumTranslationalSpeed, translationalSpeed);
            minimumTranslationalSpeed = Math.min(minimumTranslationalSpeed, translationalSpeed);
        }

        return maximumTranslationalSpeed / minimumTranslationalSpeed;
    }

//     public double getSkiddingStandardDeviation(
//             SwerveModuleState[] measuredSwerveStates, SwerveDriveKinematics swerveDriveKinematics) {
//         final ChassisSpeeds measuredChassisSpeed =
//                 swerveDriveKinematics.toChassisSpeeds(measuredSwerveStates);
                
//         final SwerveModuleState[] idealSwerveStatesGivenNoSkidding =
//                 swerveDriveKinematics.toSwerveModuleStates(measuredChassisSpeed); //what????

//         double totalSquaredDeviation = 0;
//         for (int i = 0; i < 4; i++)
//             totalSquaredDeviation +=
//                     getSquaredDifferenceBetweenTwoSwerveStates(
//                             measuredSwerveStates[i], idealSwerveStatesGivenNoSkidding[i]);

//         final double variance = totalSquaredDeviation / 4;
//         return Math.sqrt(variance);
//     }

//     //(meters/seconds)^2 //why???
//     private double getSquaredDifferenceBetweenTwoSwerveStates(
//             SwerveModuleState swerveModuleState1, SwerveModuleState swerveModuleState2) {
//         final Translation2d
//                 swerveState1VelocityVector =
//                 new Translation2d(swerveModuleState1.speedMetersPerSecond, swerveModuleState1.angle),
//                 swerveState2VelocityVector =
//                         new Translation2d(swerveModuleState2.speedMetersPerSecond, swerveModuleState2.angle);

//         return Math.pow(swerveState1VelocityVector.getDistance(swerveState2VelocityVector), 2);
//     }

}
