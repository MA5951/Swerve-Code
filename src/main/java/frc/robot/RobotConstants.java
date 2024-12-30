
package frc.robot;



import com.ma5951.utils.StateControl.StatesTypes.State;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotControl.SuperStructure;

public class RobotConstants {

    public static final double kDELTA_TIME = 0.02;

    public static final SuperStructure SUPER_STRUCTURE = new SuperStructure();

    public static final State IDLE = new State("IDLE");

    //FieldConstants
    public static final Translation2d FieldCorner = new Translation2d(16.58 , 8.20);



}
