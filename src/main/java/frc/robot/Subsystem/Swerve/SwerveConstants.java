package frc.robot.Subsystem.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.PortMap;
import frc.robot.Utils.ModuleLimits;
import frc.robot.Subsystem.Swerve.IOs.GyroPiegon2;
import frc.robot.Subsystem.Swerve.IOs.SwerveModuleSparkMax;
import frc.robot.Subsystem.Swerve.Util.Gyro;
import frc.robot.Subsystem.Swerve.Util.SwerveModule;

public class SwerveConstants {
        public final static boolean optimize = true;
        
        
        // swerve constants
        public final static double WIDTH = 0.58;
        public final static double LENGTH = 0.59;
        public final static double RADIUS = Math.sqrt(
                        Math.pow(WIDTH, 2) + Math.pow(LENGTH, 2)) / 2.0;

        // Modules constants
        public final static double TURNING_GEAR_RATIO = 150d / 7;
        public final static double DRIVE_GEAR_RATIO = 6.12;
        public final static double WHEEL_RADIUS = 0.0508 ;
        public final static double WHEEL_CIRCUMFERENCE =  2 *WHEEL_RADIUS * Math.PI;

        public final static double DRIVE_POSITION_CONVERSION = WHEEL_CIRCUMFERENCE;
        public final static double DRIVE_VELOCITY_CONVERSION = 2 * Math.PI * WHEEL_RADIUS / 60;

        public final static double TURNING_POSITION_CONVERSION = 360.0 / TURNING_GEAR_RATIO;
        public final static double TURNING_VELOCITY_CONVERSION = 1.0;

        public final static double VELOCITY_TIME_UNIT_IN_SECONDS = 1;

        // front left module
        public final static boolean FRONT_LEFT_MOUDLE_IS_DRIVE_MOTOR_REVERSED = true;
        public final static boolean FRONT_LEFT_MODULES_IS_TURNING_MOTOR_REVERSED = true;
        public final static boolean FRONT_LEFT_MODULE_IS_ABSOLUTE_ENCODER_REVERSED = false;

        // front right module
        public final static boolean FRONT_RIGHT_MOUDLE_IS_DRIVE_MOTOR_REVERSED = false;
        public final static boolean FRONT_RIGHT_MODULES_IS_TURNING_MOTOR_REVERSED = true;
        public final static boolean FRONT_RIGHT_MODULE_IS_ABSOLUTE_ENCODER_REVERSED = false;

        // rear left module
        public final static boolean REAR_LEFT_MOUDLE_IS_DRIVE_MOTOR_REVERSED = true;
        public final static boolean REAR_LEFT_MODULES_IS_TURNING_MOTOR_REVERSED = true;
        public final static boolean REAR_LEFT_MODULE_IS_ABSOLUTE_ENCODER_REVERSED = false;

        // rear right module
        public final static boolean REAR_RIGHT_MOUDLE_IS_DRIVE_MOTOR_REVERSED = false;
        public final static boolean REAR_RIGHT_MODULES_IS_TURNING_MOTOR_REVERSED = true;
        public final static boolean REAR_RIGHT_MODULE_IS_ABSOLUTE_ENCODER_REVERSED = false;

        //Module locations
        public static final Translation2d frontLeftLocation = new Translation2d(
                -WIDTH / 2,
                LENGTH / 2); 

        public static final Translation2d frontRightLocation = new Translation2d(
                WIDTH / 2,
                LENGTH / 2);

        public static final Translation2d rearLeftLocation = new Translation2d(
                -WIDTH / 2,
                -LENGTH / 2);

        public static final Translation2d rearRightLocation = new Translation2d(
                WIDTH / 2,
                -LENGTH / 2);

        public final static Translation2d[] modulesLocationArry = new Translation2d[] {frontLeftLocation , frontRightLocation , rearLeftLocation , rearRightLocation};




        //IOs
        public static final SwerveModule[] getModulesArry() {
                final SwerveModule frontLeftModule = new SwerveModuleSparkMax(
                        "Front Left",
                        PortMap.Swerve.leftFrontDriveID,
                        PortMap.Swerve.leftFrontTurningID,
                        SwerveConstants.FRONT_LEFT_MOUDLE_IS_DRIVE_MOTOR_REVERSED,
                        SwerveConstants.FRONT_LEFT_MODULES_IS_TURNING_MOTOR_REVERSED
                );

                final SwerveModule frontRightModule = new SwerveModuleSparkMax(
                "Front Right",
                        PortMap.Swerve.rightFrontDriveID,
                        PortMap.Swerve.rightFrontTurningID,
                        SwerveConstants.FRONT_RIGHT_MOUDLE_IS_DRIVE_MOTOR_REVERSED,
                        SwerveConstants.FRONT_RIGHT_MODULES_IS_TURNING_MOTOR_REVERSED
                );

                final SwerveModule rearLeftModule = new SwerveModuleSparkMax(
                        "Rear Left",
                        PortMap.Swerve.leftBackDriveID,
                        PortMap.Swerve.leftBackTurningID,
                        SwerveConstants.REAR_LEFT_MOUDLE_IS_DRIVE_MOTOR_REVERSED,
                        SwerveConstants.REAR_LEFT_MODULES_IS_TURNING_MOTOR_REVERSED
                );

                final SwerveModule rearRightModule = new SwerveModuleSparkMax(
                        "Rear Right",
                        PortMap.Swerve.rightBackDriveID,
                        PortMap.Swerve.rightBackTurningID,
                        SwerveConstants.REAR_RIGHT_MOUDLE_IS_DRIVE_MOTOR_REVERSED,
                        SwerveConstants.REAR_RIGHT_MODULES_IS_TURNING_MOTOR_REVERSED
                );
                return new SwerveModule[] {
        frontLeftModule , frontRightModule , rearLeftModule , rearRightModule};
        }

        public static final Gyro getGyro() {
                return new GyroPiegon2("Piegon 2", PortMap.CanBus.RioBus , PortMap.Swerve.Pigeon2ID);
        }

        // Modules config
        public final static int SLOT_CONFIG = 0;
        
        // MotionMagic PID
        public final static double TURNING_kP = 0.1;
        public final static double TURNING_kI = 0;
        public final static double TURNING_kD = 0;
        public final static double TURNING_kS = 0;
        public final static double TURNING_kV = 0;
        public final static double TURNING_kA = 0;
        public final static double TURNING_CTUISE_VELOCITY = 224 * (1/0.02); //RPS
        public final static double TURNING_ACCELERATION = 150;
        public final static double TURNING_JERK = 0;

        // Ramp
        public final static double OPEN_LOOP_RAMP = 0.2;
        public final static double CLOSED_LOOP_RAMP = 0;
        // Current Limit
        public final static int TURNING_CURRENT_LIMIT = 50;
        public final static int TURNING_CONTINUOUS_LOWER_LIMIT = 25;
        public final static double TURNING_CONTINUOUS_CURRENT_DURATION = 0.1;
        public final static boolean TURNING_ENABLE_CURRENT_LIMIT = true;

        // Modules drive config
        // PID
        public final static double DRIVE_kP = 0.15;
        public final static double DRIVE_kI = 0;
        public final static double DRIVE_kD = 0;
        public final static double DRIVE_kS = 0;
        public final static double DRIVE_kV = 0 ;//857
        public final static double DRIVE_kA = 0;
        
        // Current Limit
        public final static int DRIVE_CURRENT_LIMIT = 80;
        public final static int DRIVE_CONTINUOS_LOWER__LIMIT = 35;
        public final static double DRIVE_CONTINUOUS_CURRENT_DURATION = 0.15;
        public final static boolean DRIVE_ENBLE_CURRENT_LIMIT = true;

        //Swerve physics
        public final static double MAX_VELOCITY =  5.2;
        public final static double MAX_ANGULAR_VELOCITY = MAX_VELOCITY / RADIUS;// Radians
        public final static SwerveDriveKinematics kinematics = new SwerveDriveKinematics(SwerveConstants.frontLeftLocation, SwerveConstants.frontRightLocation,
        SwerveConstants.rearLeftLocation, SwerveConstants.rearRightLocation);

        

        //Module Limits 
        public final static ModuleLimits DEFUALT =  new ModuleLimits(5.2, Units.feetToMeters(65) , Units.degreesToRadians(700));
        //Swerve controllers

        //Swerve AngleAdjust
        public final static double THATA_KP = 0.45;
        public final static double THATA_KI = 0;
        public final static double THATA_KD = 0.0;
        public final static double ANGLE_PID_TOLORANCE = Math.toRadians(3);

        //Swerve AutoAdjustXY
        public final static double XY_KP = 0.45;
        public final static double XY_KI = 0;
        public final static double XY_KD = 0.0;
        public final static Constraints XY_CONSTRAINTS = new Constraints(3.5, 3);
        public final static double XY_TOLORANCE = 0.05;

        //Swerve RelativAngleAdjust
        public final static double RELATIV_THATA_KP = 0.00615;
        public final static double RELATIV_THATA_KI = 0;
        public final static double RELATIV_THATA_KD = 0.0016;
        public final static double RELATIV_ANGLE_PID_TOLORANCE = 3;

        //Swerve DRIVE CONTROLLER 
        public final static double DRIVER_XY_SCALER = 1;
        public final static double DRIVER_THATA_SCALER = 0.7;

}