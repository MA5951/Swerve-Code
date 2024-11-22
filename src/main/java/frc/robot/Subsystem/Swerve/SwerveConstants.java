package frc.robot.Subsystem.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.PortMap;
import frc.robot.Robot;
import frc.robot.Utils.ModuleLimits;
import frc.robot.Subsystem.Swerve.IOs.GyroPiegon2;
import frc.robot.Subsystem.Swerve.IOs.GyroSim;
import frc.robot.Subsystem.Swerve.IOs.SwerveModuleSim;
//import frc.robot.Subsystem.Swerve.IOs.SwerveModuleSim;
import frc.robot.Subsystem.Swerve.IOs.SwerveModuleTalonFX;
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
        public final static boolean REAR_LEFT_MOUDLE_IS_DRIVE_MOTOR_REVERSED = false;
        public final static boolean REAR_LEFT_MODULES_IS_TURNING_MOTOR_REVERSED = false;
        public final static boolean REAR_LEFT_MODULE_IS_ABSOLUTE_ENCODER_REVERSED = false;

        // rear right module
        public final static boolean REAR_RIGHT_MOUDLE_IS_DRIVE_MOTOR_REVERSED = true;
        public final static boolean REAR_RIGHT_MODULES_IS_TURNING_MOTOR_REVERSED = false;
        public final static boolean REAR_RIGHT_MODULE_IS_ABSOLUTE_ENCODER_REVERSED = false;

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

        // public static final Translation2d frontLeftLocation = new Translation2d(
        //         LENGTH / 2,
        //         WIDTH / 2);

        // public static final Translation2d frontRightLocation = new Translation2d(
        //         LENGTH / 2,
        //         -WIDTH / 2);

        // public static final Translation2d rearLeftLocation = new Translation2d(
        //         -LENGTH / 2,
        //         WIDTH / 2);

        // public static final Translation2d rearRightLocation = new Translation2d(
        //         -LENGTH / 2,
        //         -WIDTH / 2);
        

        //IOs
        public static final SwerveModule[] getModulesArry() {
                if (Robot.isReal()) {
                        final SwerveModule frontLeftModule = new SwerveModuleTalonFX(
                                "Front Left",
                                PortMap.Swerve.leftFrontDriveID,
                                PortMap.Swerve.leftFrontTurningID,
                                PortMap.Swerve.leftFrontAbsoluteEncoder,
                                SwerveConstants.FRONT_LEFT_MOUDLE_IS_DRIVE_MOTOR_REVERSED,
                                SwerveConstants.FRONT_LEFT_MODULES_IS_TURNING_MOTOR_REVERSED,
                                PortMap.CanBus.CANivoreBus);

                        final SwerveModule frontRightModule = new SwerveModuleTalonFX(
                        "Front Right",
                                PortMap.Swerve.rightFrontDriveID,
                                PortMap.Swerve.rightFrontTurningID,
                                PortMap.Swerve.rightFrontAbsoluteEncoder,
                                SwerveConstants.FRONT_RIGHT_MOUDLE_IS_DRIVE_MOTOR_REVERSED,
                                SwerveConstants.FRONT_RIGHT_MODULES_IS_TURNING_MOTOR_REVERSED,
                                PortMap.CanBus.CANivoreBus);

                        final SwerveModule rearLeftModule = new SwerveModuleTalonFX(
                                "Rear Left",
                                PortMap.Swerve.leftBackDriveID,
                                PortMap.Swerve.leftBackTurningID,
                                PortMap.Swerve.leftBackAbsoluteEncoder,
                                SwerveConstants.REAR_LEFT_MOUDLE_IS_DRIVE_MOTOR_REVERSED,
                                SwerveConstants.REAR_LEFT_MODULES_IS_TURNING_MOTOR_REVERSED,
                                PortMap.CanBus.CANivoreBus);

                        final SwerveModule rearRightModule = new SwerveModuleTalonFX(
                                "Rear Right",
                                PortMap.Swerve.rightBackDriveID,
                                PortMap.Swerve.rightBackTurningID,
                                PortMap.Swerve.rightBackAbsoluteEncoder,
                                SwerveConstants.REAR_RIGHT_MOUDLE_IS_DRIVE_MOTOR_REVERSED,
                                SwerveConstants.REAR_RIGHT_MODULES_IS_TURNING_MOTOR_REVERSED,
                                PortMap.CanBus.CANivoreBus);
                        return new SwerveModule[] {
                frontLeftModule , frontRightModule , rearLeftModule , rearRightModule};
                } else {
                        final SwerveModule frontLeftModule = new SwerveModuleSim("Front Left");
                        final SwerveModule frontRightModule = new SwerveModuleSim("Front Right");
                        final SwerveModule rearLeftModule = new SwerveModuleSim("Rear Left");
                        final SwerveModule rearRightModule = new SwerveModuleSim("Rear Right");

                        return new SwerveModule[] {
                frontLeftModule , frontRightModule , rearLeftModule , rearRightModule};
                }

        }

        public static final Gyro getGyro() {
                if (Robot.isReal()) {
                        return new GyroPiegon2("Piegon 2", PortMap.CanBus.CANivoreBus , PortMap.Swerve.Pigeon2ID);
                } else {
                        return new GyroSim("Gyro Sim");
                }
        }

        public final static Translation2d[] modulesLocationArry = new Translation2d[] {frontLeftLocation , frontRightLocation , rearLeftLocation , rearRightLocation};

        // Modules config
        public final static int SLOT_CONFIG = 0;
        
        // MotionMagic PID
        public final static double TURNING_kP = 90;
        public final static double TURNING_kI = 0;
        public final static double TURNING_kD = 0;
        public final static double TURNING_kS = 0;
        public final static double TURNING_kV = 0;
        public final static double TURNING_kA = 0;
        public final static double TURNING_CTUISE_VELOCITY = 224 * (1/0.02); //RPS
        public final static double TURNING_ACCELERATION = 150;
        public final static double TURNING_JERK = 0;

        // Ramp
        public final static double OPEN_LOOP_RAMP = 0.25;
        public final static double CLOSED_LOOP_RAMP = 0;
        // Current Limit
        public final static int TURNING_CURRENT_LIMIT = 50;
        public final static int TURNING_CONTINUOUS_LOWER_LIMIT = 25;
        public final static double TURNING_CONTINUOUS_CURRENT_DURATION = 0.1;
        public final static boolean TURNING_ENABLE_CURRENT_LIMIT = true;

        // Modules drive config
        // PID
        public final static double DRIVE_kP = 0;
        public final static double DRIVE_kI = 0;
        public final static double DRIVE_kD = 0;
        public final static double DRIVE_kS = 0;
        public final static double DRIVE_kV = 0.800 ;//857
        public final static double DRIVE_kA = 0;
        
        // Current Limit
        public final static int DRIVE_CURRENT_LIMIT = 80;
        public final static int DRIVE_CONTINUOS_LOWER__LIMIT = 35;
        public final static double DRIVE_CONTINUOUS_CURRENT_DURATION = 0.15;
        public final static boolean DRIVE_ENBLE_CURRENT_LIMIT = true;

        //Swerve physics
        public final static double MAX_VELOCITY =  5.2;
        //public final static double MAX_ACCELERATION = (10.91 / 1.15) * 1.3; 
        public final static double MAX_ANGULAR_VELOCITY = MAX_VELOCITY / RADIUS;// Radians

        public final static SwerveDriveKinematics kinematics = new SwerveDriveKinematics(SwerveConstants.frontLeftLocation, SwerveConstants.frontRightLocation,
        SwerveConstants.rearLeftLocation, SwerveConstants.rearRightLocation);

        //Swerve controllers

        //Swerve theta PID_CONTROLLER radians
        public final static double THATA_KP = 0.45;
        public final static double THATA_KI = 0;
        public final static double THATA_KD = 0.0;
        public final static double ANGLE_PID_TOLORANCE = Math.toRadians(3);

        //Swerve theta PID_CONTROLLER degrees
        public final static double RELATIV_THATA_KP = 0.00615;
        public final static double RELATIV_THATA_KI = 0;
        public final static double RELATIV_THATA_KD = 0.0016;
        public final static double RELATIV_ANGLE_PID_TOLORANCE = 3;

        public final static double TIME_AT_SET_POINT = 0.1;

        //Swerve theta PID_CONTROLLER lock
        public final static double THATA_LOCK_KP = 0.01;
        public final static double THATA_LOCK_KI = 0;
        public final static double THATA_LOCK_KD = 0.00015;
        public final static double THATA_LOCK_PID_TOLORANCE = 2;
        public final static double THATA_LOCK_THRESHOLD= 0.0;

        //Swerve DRIVE CONTROLLER 
        public final static double DRIVER_XY_SCALER = 1;
        public final static double DRIVER_THATA_SCALER = 0.7;

        //Module Limits //75 Accl //700/800
        public final static ModuleLimits DEFUALT =  new ModuleLimits(5.2, Units.feetToMeters(65) , Units.degreesToRadians(700)); 
        
        //Collision Detector
        public final static double COLLISION_THRESHOLD = 1.85;

        //Oגometry
        public static final double ODOMETRY_UPDATE_RATE = 200;

        //TODO: REFRASH ALL STATUS SIGNAL FUNCTION
}