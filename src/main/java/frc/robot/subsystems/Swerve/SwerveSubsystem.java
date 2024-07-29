// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import com.ma5951.utils.RobotConstants;
import com.ma5951.utils.Logger.LoggedSwerveStates;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//The orde of te modules is a STANDART and it is
//Front Left
//Front Right
//Rear Left
//Raer Right
import frc.robot.Utils.ModuleLimits;
import frc.robot.Utils.OdometryUpdate;
import frc.robot.Utils.RobotClock;
import frc.robot.Utils.SwerveSetpoint;
import frc.robot.subsystems.Swerve.Util.SwerveModule;
import frc.robot.subsystems.Swerve.Util.Gyro;

public class SwerveSubsystem extends SubsystemBase {
  private static SwerveSubsystem swerveSubsystem;

  private SwerveSetpointGenerator setpointGenerator;
  private LoggedSwerveStates currenStates;
  private LoggedSwerveStates setPoinStates;

  private final SwerveModule[] modulesArry = SwerveConstants.getModulesArry();
  private final Gyro gyro = SwerveConstants.getGyro();
  private final SwerveDriveKinematics kinematics = SwerveConstants.kinematics;
  SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
  private ModuleLimits currentLimits = SwerveConstants.DEFUALT;
  private double offsetAngle = 0;

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

    currenStates = new LoggedSwerveStates("/Swerve/Current States");
    setPoinStates = new LoggedSwerveStates("/Swerve/SetPoint States");

    for (int i = 0; i < 4 ; i++) {
      modulesArry[i].setNeutralModeDrive(false);
      modulesArry[i].setNeutralModeTurn(false);
    }

  }

  public SwerveModulePosition[] getSwerveModulePositions() {
    return new SwerveModulePosition[] {
        modulesArry[0].getPosition(),
        modulesArry[1].getPosition(),
        modulesArry[2].getPosition(),
        modulesArry[3].getPosition()
      };
  }

  private SwerveModuleState[] getSwerveModuleStates() {
    return new SwerveModuleState[] {
        modulesArry[0].getState(),
        modulesArry[1].getState(),
        modulesArry[2].getState(),
        modulesArry[3].getState()
    };
  }

  public double getOffsetAngle() {
    return offsetAngle;
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

  public double getVelocity() {
    ChassisSpeeds speeds = getRobotRelativeSpeeds();
    return Math.sqrt(Math.pow(speeds.vxMetersPerSecond, 2) +
      Math.pow(speeds.vyMetersPerSecond, 2));
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(getSwerveModuleStates());
  }

  public Rotation2d getRotation2d() {
    return new Rotation2d(Math.toRadians(getFusedHeading()));
  }

  public SwerveModuleState[] generateStates(ChassisSpeeds chassiSpeeds , boolean optimize) {
    chassiSpeeds = new ChassisSpeeds(chassiSpeeds.vxMetersPerSecond * SwerveConstants.MAX_VELOCITY,
     chassiSpeeds.vyMetersPerSecond * SwerveConstants.MAX_VELOCITY, chassiSpeeds.omegaRadiansPerSecond * SwerveConstants.MAX_ANGULAR_VELOCITY);
    
    if (optimize) {
      currentSetpoint =
      setpointGenerator.generateSetpoint(
        new ModuleLimits(4.5, 20 , Units.degreesToRadians(1080)), currentSetpoint, chassiSpeeds, RobotConstants.KDELTA_TIME);

    for (int i = 0; i < modulesArry.length; i++) {
      optimizedSetpointStates[i] =
         SwerveModuleState.optimize(new SwerveModuleState(currentSetpoint.moduleStates()[i].speedMetersPerSecond, currentSetpoint.moduleStates()[i].angle), new Rotation2d(Units.degreesToRadians(modulesArry[i].getTurningPosition())));
      //optimizedSetpointStates[i] = currentSetpoint.moduleStates()[i];
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

  public void drive(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] states = generateStates(chassisSpeeds, true);
    setPoinStates.update(states);
    setModules(states);
  }

  public OdometryUpdate getOdometryUpdate() {
    return new OdometryUpdate(getSwerveModulePositions(), RobotClock.getInstance().getRobotTimeStamp());
  }

  public ModuleLimits getCurrentLimits() {
    return currentLimits;
  }

  public void setCurrentLimits(ModuleLimits newLimits) {
    currentLimits = newLimits;
  }

  public static SwerveSubsystem getInstance() {
  if (swerveSubsystem == null) {
    swerveSubsystem = new SwerveSubsystem();
  }
  return swerveSubsystem;
  }

  @Override
  public void periodic() {
    for (int i = 0; i < 4 ; i++) {
      modulesArry[i].update();
    }
    modulesArry[0].turningUsingPID(180);
    gyro.update(kinematics.toChassisSpeeds(getSwerveModuleStates()));
    currenStates.update(getSwerveModuleStates());

    


    //Limits state meachinp
    // if (RobotContainer.driveController.R2().getAsBoolean()) {
    //   currentLimits = SwerveConstants.Slow10Precent;
    // } else if (RobotContainer.driveController.L2().getAsBoolean()) {
    //   currentLimits = SwerveConstants.Slow40Precent;
    // }

  }
}
