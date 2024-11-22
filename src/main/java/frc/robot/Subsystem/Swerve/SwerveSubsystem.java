// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Swerve;

import java.util.Arrays;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.stream.IntStream;

import com.ma5951.utils.RobotConstantsMAUtil;
import com.ma5951.utils.Logger.LoggedDouble;
import com.ma5951.utils.Logger.LoggedSwerveStates;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//The orde of te modules is a STANDART and it is
//Front Left
//Front Right
//Rear Left
//Raer Right
import frc.robot.Utils.ModuleLimits;
import frc.robot.Utils.SwerveSetpoint;
import frc.robot.Subsystem.Swerve.Util.SwerveModule;
import frc.robot.Subsystem.Swerve.Util.SwerveModuleData;
import frc.robot.Subsystem.PoseEstimation.PoseEstimator;
import frc.robot.Subsystem.Swerve.Util.Gyro;
import frc.robot.Subsystem.Swerve.Util.GyroData;

public class 
SwerveSubsystem extends SubsystemBase {
  private static SwerveSubsystem swerveSubsystem;

  private SwerveSetpointGenerator setpointGenerator;
  private LoggedSwerveStates currenStatesLog;
  private LoggedSwerveStates setPoinStatesLog;
  private LoggedDouble offsetprintLog;
  private LoggedDouble swerevXvelocityLog;
  private LoggedDouble swerevYvelocityLog;
  private LoggedDouble swerevXaccelLog;
  private LoggedDouble swerevYaccelLog;
  private LoggedDouble swerevTheatavelocityLog;
  private LoggedDouble swerveTheataaccelLog;
  

  private final SwerveModule[] modulesArry = SwerveConstants.getModulesArry();
  private final Gyro gyro = SwerveConstants.getGyro();
  private final SwerveDriveKinematics kinematics = SwerveConstants.kinematics;
  private SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
  private SwerveModuleState[] currentStates = new SwerveModuleState[4];
  private SwerveModulePosition[] currentPositions = new  SwerveModulePosition[4];
  private SwerveModulePosition[] wheelPositions = new  SwerveModulePosition[4];
  private SwerveModulePosition[] lastPositions = new  SwerveModulePosition[] {
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition()
};
  private SwerveModuleData[] modulesData = new SwerveModuleData[4];
  private GyroData gyroData = new GyroData();
  private ModuleLimits currentLimits = SwerveConstants.DEFUALT;
  private ChassisSpeeds currentChassisSpeeds;
  private double offsetAngle = 0;
  private double lastXvelocity;
  private double lastYvelocity;
  private double lastTheatavelocity;
  private double lastTime;

  public double[] timestamps = new double[] {};

  public static final Lock odometryLock = new ReentrantLock();
  public static final Queue<Double> timestampQueue = new ArrayBlockingQueue<>(20);
  
  

  private SwerveSetpoint currentSetpoint = new SwerveSetpoint(
    new ChassisSpeeds(),
          new SwerveModuleState[] {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
  });

  public SwerveSubsystem() {

    setpointGenerator = new SwerveSetpointGenerator(kinematics , new Translation2d[] {
      SwerveConstants.frontLeftLocation,
      SwerveConstants.frontRightLocation,
      SwerveConstants.rearLeftLocation,
      SwerveConstants.rearRightLocation
    });


    currenStatesLog = new LoggedSwerveStates("/Swerve/States/Current States");
    setPoinStatesLog = new LoggedSwerveStates("/Swerve/States/SetPoint States");
    swerevXvelocityLog = new LoggedDouble("/Swerve/Chassis Speed/X Velocity");
    swerevYvelocityLog = new LoggedDouble("/Swerve/Chassis Speed/Y Velocity");
    swerevTheatavelocityLog = new LoggedDouble("/Swerve/Chassis Speed/Theat Velocity");
    swerevXaccelLog = new LoggedDouble("/Swerve/Chassis Speed/X Accel");
    swerevYaccelLog = new LoggedDouble("/Swerve/Chassis Speed/Y Accel");
    swerveTheataaccelLog = new LoggedDouble("/Swerve/Chassis Speed/Theath Accel");
    offsetprintLog = new LoggedDouble("/Swerve/Gyro Offset Angle");

    for (int i = 0; i < 4 ; i++) {
      modulesArry[i].setNeutralModeDrive(true);
      modulesArry[i].setNeutralModeTurn(true);
    }

    

    gyro.reset();
  }

  public SwerveModulePosition[] getSwerveModulePositions() {
    for (int i = 0; i<4 ; i++) {
      currentPositions[i] = modulesArry[i].getPosition(modulesData[i]);
    }

    return currentPositions;
  }

  public SwerveModuleState[] getSwerveModuleStates() {
    for (int i = 0; i<4 ; i++) {
      currentStates[i] = modulesArry[i].getState(modulesData[i]);
    }

    return currentStates;
  }

  public double getOffsetAngle() {
    return offsetAngle;
  }

  public void updateOffset(double offset) {
    offsetAngle = offset;
  }

  public void updateOffset() {
    offsetAngle = getFusedHeading();
  }

  public void resetGyro() {
    gyro.reset();
  }

  public double getFusedHeading() {
    return gyro.getYaw();
  }

  public double getRoll() {
    return gyro.getRoll();
  }

  public double getPitch() {
    return gyro.getPitch();
  }

  public double getAbsYaw() {
    return gyro.getAbsYaw();
  }

  public double getVelocityVector(){
    ChassisSpeeds speeds = getRobotRelativeSpeeds();
    return Math.sqrt(Math.pow(speeds.vxMetersPerSecond, 2) +
      Math.pow(speeds.vyMetersPerSecond, 2));
  }

  //TODO: Critical fix nedded
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(getSwerveModuleStates());
  }

  public Rotation2d getRotation2d() {
    return new Rotation2d(Math.toRadians(getFusedHeading()));
  }

  public SwerveModuleState[] generateStates(ChassisSpeeds chassiSpeeds , boolean optimize , boolean scale) {
    if (scale) { // move to controler
      chassiSpeeds.omegaRadiansPerSecond = chassiSpeeds.omegaRadiansPerSecond * SwerveConstants.MAX_ANGULAR_VELOCITY;
      chassiSpeeds.vxMetersPerSecond = chassiSpeeds.vxMetersPerSecond * SwerveConstants.MAX_VELOCITY;
      chassiSpeeds.vyMetersPerSecond = chassiSpeeds.vyMetersPerSecond * SwerveConstants.MAX_VELOCITY;
    }


    if (optimize) {
      currentSetpoint =
      setpointGenerator.generateSetpoint(
       getCurrentLimits(), currentSetpoint, chassiSpeeds, RobotConstantsMAUtil.KDELTA_TIME);

    for (int i = 0; i < modulesArry.length; i++) {
      optimizedSetpointStates[i] = currentSetpoint.moduleStates()[i];

    }

    return optimizedSetpointStates;
    } else {
      return kinematics
        .toSwerveModuleStates(chassiSpeeds);
    }
  }

  public void setModules(SwerveModuleState[] states) {
    modulesArry[0].setDesiredState(states[0]);
    modulesArry[1].setDesiredState(states[1]);
    modulesArry[2].setDesiredState(states[2]);
    modulesArry[3].setDesiredState(states[3]);
  }

  public void drive(ChassisSpeeds chassisSpeeds , boolean isAuto) {
    SwerveModuleState[] states = generateStates(chassisSpeeds, SwerveConstants.optimize , !isAuto);

    SwerveModuleState[] Optistates = new SwerveModuleState[] {states[1] , states[3] , states[0] , states[2]};
    setPoinStatesLog.update(Optistates);
    setModules(Optistates);
  }

  public void drive(ChassisSpeeds chassisSpeeds , DriveFeedforwards feedforwards) {
    SwerveModuleState[] states = generateStates(chassisSpeeds, SwerveConstants.optimize , true);

    SwerveModuleState[] Optistates = new SwerveModuleState[] {states[1] , states[3] , states[0] , states[2]};
    setPoinStatesLog.update(Optistates);
    setModules(Optistates);
  }

  public ModuleLimits getCurrentLimits() {
    return currentLimits;
  }

  public void setCurrentLimits(ModuleLimits newLimits) {
    currentLimits = newLimits;
  }

  public SwerveModule[] getModulesArry() {
    return modulesArry;
  }

  public SwerveModuleState calculateFFTorque(SwerveModuleState desiState) {
     Vector<N2> wheelDirection =
              VecBuilder.fill(
                  desiState.angle.getCos(),
                  desiState.angle.getSin());
          //wheelForces is junk
          Vector<N2> wheelForces = new Vector<>(wheelDirection);//rajectoryController.getModuleForces().get(i);
          double wheelTorque =
              wheelForces.dot(wheelDirection) * SwerveConstants.WHEEL_RADIUS;
      return new SwerveModuleState(wheelTorque, desiState.angle);
  }

  public static SwerveSubsystem getInstance() {
  if (swerveSubsystem == null) {
    swerveSubsystem = new SwerveSubsystem();
  }
  return swerveSubsystem;
  }

  @Override
  public void periodic() {
    odometryLock.lock();

    timestamps =
        timestampQueue.stream().mapToDouble(Double::valueOf).toArray();
    if (timestamps.length == 0) {
      timestamps = new double[] {Timer.getFPGATimestamp()};
    }
    timestampQueue.clear();

    gyroData = gyro.update();
    for (int i = 0; i < 4 ; i++) {
      modulesData[i] = modulesArry[i].update();
    }

    odometryLock.unlock();
    
    int minOdometryUpdates =
        IntStream.of(
                timestamps.length,
                Arrays.stream(modulesData)
                    .mapToInt(modulesData -> modulesData.getSteerPositionQueue().length) //TODO: Cheack
                    .min()
                    .orElse(0))
            .min()
            .orElse(0);
    minOdometryUpdates = Math.min(gyroData.getYawPositionQueue().length, minOdometryUpdates);
  
    for (int i = 0; i < minOdometryUpdates; i++) {
      int odometryIndex = i;
      Rotation2d yaw = gyroData.getYawPositionQueue()[i];
      for (int a = 0; a < modulesArry.length ; a++) {
        wheelPositions[a] = modulesArry[a].getModulePositions(modulesData[a])[odometryIndex];
      }
      // Filtering based on delta wheel positions
      boolean includeMeasurement = true;
      // if (lastPositions != null) {
      //   double dt = timestamps[i] - lastTime;
      //   for (int j = 0; j < modulesArry.length; j++) {
      //     double velocity =
      //         (wheelPositions[j].distanceMeters
      //                 - lastPositions[j].distanceMeters)
      //             / dt;
      //     double omega =
      //         wheelPositions[j].angle.minus(lastPositions[j].angle).getRadians()
      //             / dt;
      //     // Check if delta is too large
      //     if (Math.abs(omega) > currentLimits.maxSteeringVelocity() * 5.0
      //         || Math.abs(velocity) > currentLimits.maxDriveVelocity() * 5.0) {
      //       includeMeasurement = false;
      //       break;
      //     }
      //   }
      // }

      if (includeMeasurement) {
        //System.out.println("updated");
        lastPositions = wheelPositions;
        PoseEstimator.getInstance().updateOdometry(wheelPositions, yaw, timestamps[i]);
        lastTime = timestamps[i];
      }
    }



    




   
    currentChassisSpeeds = kinematics.toChassisSpeeds(getSwerveModuleStates());
    currenStatesLog.update(getSwerveModuleStates());
    offsetprintLog.update(offsetAngle);
    
    swerevXvelocityLog.update(currentChassisSpeeds.vxMetersPerSecond);
    swerevYvelocityLog.update(currentChassisSpeeds.vyMetersPerSecond);
    swerveTheataaccelLog.update(currentChassisSpeeds.omegaRadiansPerSecond);
    swerevXaccelLog.update((lastXvelocity - currentChassisSpeeds.vxMetersPerSecond) / 0.2);
    swerevYaccelLog.update((lastYvelocity - currentChassisSpeeds.vyMetersPerSecond) / 0.2);
    swerevTheatavelocityLog.update((lastTheatavelocity - currentChassisSpeeds.omegaRadiansPerSecond) / 0.2);

    lastXvelocity = currentChassisSpeeds.vxMetersPerSecond;
    lastYvelocity = currentChassisSpeeds.vyMetersPerSecond;
    lastTheatavelocity = currentChassisSpeeds.omegaRadiansPerSecond;

    
  }
}
