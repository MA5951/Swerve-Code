
package frc.robot.RobotControl;


import com.ma5951.utils.StateControl.GenericSuperStracture;

import frc.robot.Subsystem.Swerve.SwerveSubsystem;

public class SuperStructure extends GenericSuperStracture{



    public SuperStructure() {
        super(() -> SwerveSubsystem.getInstance().getVelocityVector());
    }

    public void update() {

    }


}
