// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ma5951.utils.DashBoard.MAShuffleboard;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;
//The orde of te modules is a STANDART and it is
//Front Left
//Front Right
//Rear Left
//Raer Right
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Utils.ModuleLimits;
import frc.robot.Utils.OdometryUpdate;
import frc.robot.Utils.RobotClock;
import frc.robot.Utils.SwerveSetpoint;

public class SwerveSubsystem extends SubsystemBase {
  private static SwerveSubsystem swerveSubsystem;

  private SwerveSetpointGenerator setpointGenerator;
  private MAShuffleboard board;


  public final static SwerveModule[] modulesArry = new SwerveModule[] {
    SwerveConstants.frontLeftModule , SwerveConstants.frontRightModule , SwerveConstants.rearLeftModule , SwerveConstants.rearRightModule};
  private ModuleLimits currentLimits;

  private static SwerveModuleState[] simModulesStates = {
    new SwerveModuleState(0, new Rotation2d(0)),
    new SwerveModuleState(0, new Rotation2d(0)), 
    new SwerveModuleState(0, new Rotation2d(0)),
    new SwerveModuleState(0, new Rotation2d(0)), 
  };

  private static SwerveModulePosition[] getSwerveModulePositions() {
    if (Robot.isReal()) {
      return new SwerveModulePosition[] {
        modulesArry[0].getPosition(),
        modulesArry[1].getPosition(),
        modulesArry[2].getPosition(),
        modulesArry[3].getPosition()
      };
    } else {
      return new SwerveModulePosition[] {
        new SwerveModulePosition(simModulesStates[0].speedMetersPerSecond, simModulesStates[0].angle),
        new SwerveModulePosition(simModulesStates[2].speedMetersPerSecond, simModulesStates[2].angle),
        new SwerveModulePosition(simModulesStates[1].speedMetersPerSecond, simModulesStates[1].angle),
        new SwerveModulePosition(simModulesStates[3].speedMetersPerSecond, simModulesStates[3].angle)
      };
    }
  }

  private static SwerveModuleState[] getSwerveModuleStates() {
    if (Robot.isReal()) {
      return new SwerveModuleState[] {
        modulesArry[0].getState(),
        modulesArry[1].getState(),
        modulesArry[2].getState(),
        modulesArry[3].getState()
    };
    } else {
      return new SwerveModuleState[] {
        simModulesStates[0],
        simModulesStates[1],
        simModulesStates[2],
        simModulesStates[3]
    };
    }
  }

  private SwerveSetpoint currentSetpoint = new SwerveSetpoint(
    new ChassisSpeeds(),
          new SwerveModuleState[] {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
  });

  private final Pigeon2 gyro = new Pigeon2(PortMap.Swerve.Pigeon2ID, PortMap.CanBus.RioBus);
  private Double simAngle;
  private double offsetAngle = 0;

  public SwerveSubsystem() {

    setpointGenerator = new SwerveSetpointGenerator(SwerveConstants.kinematics , new Translation2d[] {
      SwerveConstants.frontLeftLocation,
      SwerveConstants.frontRightLocation,
      SwerveConstants.rearLeftLocation,
      SwerveConstants.rearRightLocation
    });

    board = new MAShuffleboard("Swerve");

  }

  public void resetEncoders() {
    modulesArry[0].resetEncoders();
    modulesArry[1].resetEncoders();
    modulesArry[2].resetEncoders();
    modulesArry[3].resetEncoders();
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
    if (Robot.isReal()) {
      StatusSignal<Double> yaw = gyro.getYaw();
      yaw.refresh();
      return yaw.getValue();
    } else {
      if (simAngle == null) {
        return 0d;
      } else {
        return simAngle;
      }
    }
  }

  public double getRoll() {
    if (Robot.isReal()) {
      StatusSignal<Double> roll = gyro.getRoll();
      roll.refresh();
      return roll.getValue();
    } else {
      return 0d;
    }
  }

  public double getPitch() {
    if (Robot.isReal()) {
      StatusSignal<Double> pitch = gyro.getPitch();
      pitch.refresh();
      return pitch.getValue();
    } else {
      return 0d;
    }
  }

  public double getVelocity() {
    ChassisSpeeds speeds = getRobotRelativeSpeeds();
    return Math.sqrt(Math.pow(speeds.vxMetersPerSecond, 2) +
      Math.pow(speeds.vyMetersPerSecond, 2));
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return SwerveConstants.kinematics.toChassisSpeeds(getSwerveModuleStates());
  }

  public Rotation2d getRotation2d() {
    return new Rotation2d(Math.toRadians(getFusedHeading()));
  }

  public void stop() {
    modulesArry[0].stop();
    modulesArry[1].stop();
    modulesArry[2].stop();
    modulesArry[3].stop();
  }

  public SwerveModuleState[] generateStates(ChassisSpeeds chassiSpeeds , boolean optimize) {
    if (optimize) {
      currentSetpoint =
      setpointGenerator.generateSetpoint(
          new ModuleLimits(Units.feetToMeters(15.0), Units.feetToMeters(75.0), Units.degreesToRadians(1080)), currentSetpoint, chassiSpeeds, 0.02);
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];

    for (int i = 0; i < modulesArry.length; i++) {
      // Optimize setpoints
      optimizedSetpointStates[i] =
          SwerveModuleState.optimize(currentSetpoint.moduleStates()[i], new Rotation2d(modulesArry[i].getAbsoluteEncoderPosition()));
    }

    return optimizedSetpointStates;
    } else {
      return SwerveConstants.kinematics
        .toSwerveModuleStates(chassiSpeeds);
    }
  }



  public void setModules(SwerveModuleState[] states) {
    SwerveConstants.frontLeftModule.setDesiredState(states[0]);
    SwerveConstants.frontRightModule.setDesiredState(states[1]);
    SwerveConstants.rearLeftModule.setDesiredState(states[2]);
    SwerveConstants.rearRightModule.setDesiredState(states[3]);
    if (Robot.isReal()) {
      
    } else {
      simModulesStates[0].speedMetersPerSecond += states[0].speedMetersPerSecond * 0.02;
      simModulesStates[1].speedMetersPerSecond += states[1].speedMetersPerSecond * 0.02;
      simModulesStates[2].speedMetersPerSecond += states[2].speedMetersPerSecond * 0.02;
      simModulesStates[3].speedMetersPerSecond += states[3].speedMetersPerSecond * 0.02;

      simModulesStates[0].angle = states[0].angle;
      simModulesStates[1].angle = states[1].angle;
      simModulesStates[2].angle = states[2].angle;
      simModulesStates[3].angle = states[3].angle;
    }
  }

  public void drive(double x, double y, double omega, boolean fieldRelative , boolean optimize) {
    ChassisSpeeds chassisSpeeds = fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(x, y, omega,
                  new Rotation2d(
                    Math.toRadians((getFusedHeading() - offsetAngle))))
                  : new ChassisSpeeds(x, y, omega);
    SwerveModuleState[] setPointStates = generateStates(chassisSpeeds , optimize);
  
    setModules(new SwerveModuleState[] {setPointStates[0] , setPointStates[2] , setPointStates[1] , setPointStates[3]});
    if (!Robot.isReal()) {
      simAngle += Math.toDegrees(chassisSpeeds.omegaRadiansPerSecond) * 0.02;
    }
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] states = generateStates(chassisSpeeds, false);
    setModules(new SwerveModuleState[] {states[0] , states[1] , states[2] , states[3]});
      if (!Robot.isReal()) {
        simAngle += Math.toDegrees(chassisSpeeds.omegaRadiansPerSecond) * 0.02;
    }
  }

  public OdometryUpdate getOdometryUpdate() {
    return new OdometryUpdate(getSwerveModulePositions(), RobotClock.getInstance().getRobotTimeStamp());
  }

  public void printAbsolutePositions() {
    board.addNum("Front Left Absolute" , modulesArry[0].getAbsoluteEncoderPosition());
    board.addNum("Front Rigth Absolute" , modulesArry[1].getAbsoluteEncoderPosition());
    board.addNum("Rear Left Absolute" , modulesArry[2].getAbsoluteEncoderPosition());
    board.addNum("Rear Right Absolute" , modulesArry[3].getAbsoluteEncoderPosition());
  }

  public ModuleLimits getCurrentLimits() {
    return currentLimits;
  }

  public void setCurrentLimits(ModuleLimits newLimits) {
    currentLimits = newLimits;
  }

  public Pigeon2 getGyro() {
    return gyro;
  }

  public static SwerveSubsystem getInstance() {
  if (swerveSubsystem == null) {
    swerveSubsystem = new SwerveSubsystem();
  }
  return swerveSubsystem;
  }

  @Override
  public void periodic() {
    


    //Limits state meachin
    if (RobotContainer.driveController.R2().getAsBoolean()) {
      currentLimits = SwerveConstants.Slow10Precent;
    } else if (RobotContainer.driveController.L2().getAsBoolean()) {
      currentLimits = SwerveConstants.Slow40Precent;
    }
  }
}
