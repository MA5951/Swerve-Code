// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.SwerveNew.SwerveSubsystem;

/** Add your docs here. */
public class RobotClock {
    private static RobotClock robotClock;

    public RobotClock() {
        
    }

    public double getRobotTimeStamp() {
        return Timer.getFPGATimestamp();
    }


    public static RobotClock getInstance() {
        if (robotClock == null) {
            robotClock = new RobotClock();
        }
        return robotClock;
    }

}
