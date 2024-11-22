
package com.ma5951.utils.Utils;

import java.sql.Driver;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Subsystem.Swerve.SwerveConstants;

public class DriverStationUtil {

    public static Alliance getAlliance() {
        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get() == Alliance.Blue) {
                return Alliance.Blue;
            } else {
                return Alliance.Red;
            }
        }

        return Alliance.Blue;
    }

   



}
