package frc.robot.subsystems.Swerve;

import javax.sound.sampled.Port;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.PortMap;
import frc.robot.Robot;
import frc.robot.Utils.ModuleLimits;
import frc.robot.subsystems.Swerve.IOs.GyroPiegon2;
import frc.robot.subsystems.Swerve.IOs.GyroSim;
import frc.robot.subsystems.Swerve.IOs.SwerveModuleSim;
import frc.robot.subsystems.Swerve.IOs.SwerveModuleTalonFX;
import frc.robot.subsystems.Swerve.Util.Gyro;
import frc.robot.subsystems.Swerve.Util.SwerveModule;

public class SwerveConstants {
        // swerve constants
        public final static double WIDTH = 0.545;
        public final static double LENGTH = 0.545;
        public final static double RADIUS = Math.sqrt(
                        Math.pow(WIDTH, 2) + Math.pow(LENGTH, 2)) / 2.0;

        // Modules constants
        public final static double TURNING_GEAR_RATIO = 150d / 7;
        public final static double DRIVE_GEAR_RATIO = 6.75;
        private final static double WHEEL_RADIUS = 0.0508;
        public final static double WHEEL_CIRCUMFERENCE =  WHEEL_RADIUS * Math.PI;

        public final static double VELOCITY_TIME_UNIT_IN_SECONDS = 1;

        public final static double DISTANCE_PER_PULSE = (((2 * WHEEL_RADIUS * Math.PI)
                        / DRIVE_GEAR_RATIO)); // * 0.92378753;
        public final static double ANGLE_PER_PULSE = 360d / TURNING_GEAR_RATIO;

        // front left module
        public final static double FRONT_LEFT_MODULE_OFFSET_ENCODER = 9.4;
        public final static boolean FRONT_LEFT_MOUDLE_IS_DRIVE_MOTOR_REVERSED = true;
        public final static boolean FRONT_LEFT_MODULES_IS_TURNING_MOTOR_REVERSED = true;
        public final static boolean FRONT_LEFT_MODULE_IS_ABSOLUTE_ENCODER_REVERSED = false;

        // front right module
        public final static double FRONT_RIGHT_MODULE_OFFSET_ENCODER = 162.8;
        public final static boolean FRONT_RIGHT_MOUDLE_IS_DRIVE_MOTOR_REVERSED = false;
        public final static boolean FRONT_RIGHT_MODULES_IS_TURNING_MOTOR_REVERSED = true;
        public final static boolean FRONT_RIGHT_MODULE_IS_ABSOLUTE_ENCODER_REVERSED = false;

        // rear left module
        public final static double REAR_LEFT_MODULE_OFFSET_ENCODER = 199.6;
        public final static boolean REAR_LEFT_MOUDLE_IS_DRIVE_MOTOR_REVERSED = true;
        public final static boolean REAR_LEFT_MODULES_IS_TURNING_MOTOR_REVERSED = true;
        public final static boolean REAR_LEFT_MODULE_IS_ABSOLUTE_ENCODER_REVERSED = false;

        // rear right module
        public final static double REAR_RIGHT_MODULE_OFFSET_ENCODER = 214;
        public final static boolean REAR_RIGHT_MOUDLE_IS_DRIVE_MOTOR_REVERSED = false;
        public final static boolean REAR_RIGHT_MODULES_IS_TURNING_MOTOR_REVERSED = true;
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
        
        public static final double distanceFromCenter = Math.sqrt(Math.pow(WIDTH, 2) + Math.pow(LENGTH, 2)) / 2;


        //IO
        public static final SwerveModule[] getModulesArry() {
                if (Robot.isReal()) {
                        final SwerveModule frontLeftModule = new SwerveModuleTalonFX(
                                "Front Left",
                                PortMap.Swerve.leftFrontDriveID,
                                PortMap.Swerve.leftFrontTurningID,
                                PortMap.Swerve.leftFrontAbsoluteEncoder,
                                SwerveConstants.FRONT_LEFT_MOUDLE_IS_DRIVE_MOTOR_REVERSED,
                                SwerveConstants.FRONT_LEFT_MODULES_IS_TURNING_MOTOR_REVERSED,
                                SwerveConstants.FRONT_LEFT_MODULE_IS_ABSOLUTE_ENCODER_REVERSED,
                                SwerveConstants.FRONT_LEFT_MODULE_OFFSET_ENCODER,
                                PortMap.CanBus.CANivoreBus);

                        final SwerveModule frontRightModule = new SwerveModuleTalonFX(
                        "Front Right",
                                PortMap.Swerve.rightFrontDriveID,
                                PortMap.Swerve.rightFrontTurningID,
                                PortMap.Swerve.rightFrontAbsoluteEncoder,
                                SwerveConstants.FRONT_RIGHT_MOUDLE_IS_DRIVE_MOTOR_REVERSED,
                                SwerveConstants.FRONT_RIGHT_MODULES_IS_TURNING_MOTOR_REVERSED,
                                SwerveConstants.FRONT_RIGHT_MODULE_IS_ABSOLUTE_ENCODER_REVERSED,
                                SwerveConstants.FRONT_RIGHT_MODULE_OFFSET_ENCODER,
                                PortMap.CanBus.CANivoreBus);

                        final SwerveModule rearLeftModule = new SwerveModuleTalonFX(
                                "Rear Right",
                                PortMap.Swerve.leftBackDriveID,
                                PortMap.Swerve.leftBackTurningID,
                                PortMap.Swerve.leftBackAbsoluteEncoder,
                                SwerveConstants.REAR_LEFT_MOUDLE_IS_DRIVE_MOTOR_REVERSED,
                                SwerveConstants.REAR_LEFT_MODULES_IS_TURNING_MOTOR_REVERSED,
                                SwerveConstants.REAR_LEFT_MODULE_IS_ABSOLUTE_ENCODER_REVERSED,
                                SwerveConstants.REAR_LEFT_MODULE_OFFSET_ENCODER,
                                PortMap.CanBus.CANivoreBus);

                        final SwerveModule rearRightModule = new SwerveModuleTalonFX(
                                "Rear Left",
                                PortMap.Swerve.rightBackDriveID,
                                PortMap.Swerve.rightBackTurningID,
                                PortMap.Swerve.rightBackAbsoluteEncoder,
                                SwerveConstants.REAR_RIGHT_MOUDLE_IS_DRIVE_MOTOR_REVERSED,
                                SwerveConstants.REAR_RIGHT_MODULES_IS_TURNING_MOTOR_REVERSED,
                                SwerveConstants.REAR_RIGHT_MODULE_IS_ABSOLUTE_ENCODER_REVERSED,
                                SwerveConstants.REAR_RIGHT_MODULE_OFFSET_ENCODER,
                                PortMap.CanBus.CANivoreBus);
                        return new SwerveModule[] {
                frontLeftModule , frontRightModule , rearLeftModule , rearRightModule};
                } else {
                        final SwerveModule frontLeftModule = new SwerveModuleSim("Front Left", 0);
                        final SwerveModule frontRightModule = new SwerveModuleSim("Front Right", 0);
                        final SwerveModule rearLeftModule = new SwerveModuleSim("Rear Left", 0);
                        final SwerveModule rearRightModule = new SwerveModuleSim("Rear Right", 0);

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

        // Modules turning config
        public final static int SLOT_CONFIG = 0;
        
        // MotionMagic PIDF
        public final static double TURNING_kP = 3.6;
        public final static double TURNING_kI = 0;
        public final static double TURNING_kD = 0;
        public final static double TURNING_kS = 0;
        public final static double TURNING_kV = 0;
        public final static double TURNING_kA = 0;
        public final static double TURNING_CTUISE_VELOCITY = 0;
        public final static double TURNING_ACCELERATION = 0;
        public final static double TURNING_JERK = 0;

        // Ramp
        public final static double OPEN_LOOP_RAMP = 0.25;
        public final static double CLOSED_LOOP_RAMP = 0;
        // Current Limit
        public final static int TURNING_PEAK_CURRENT_LIMIT_TORQUE_CURRENT = 400;
        public final static int TURNING_PEAK_CURRENT_LIMIT = 40;
        public final static int TURNING_CONTINUOUS_CURRENT_LIMIT = 25;
        public final static double TURNING_PEAK_CURRENT_DURATION = 0.1;
        public final static boolean TURNING_ENABLE_CURRENT_LIMIT = true;

        // Modules drive config
        // PID
        public final static double DRIVE_kP = 0.1;
        public final static double DRIVE_kI = 0;
        public final static double DRIVE_kD = 0;
        public final static double DRIVE_kS = 0;
        public final static double DRIVE_kV = 2.4;
        // Current Limit
        public final static int DRIVE_PEAK_CURRENT_LIMIT_TORQUE_CURRENT = 60;
        public final static int DRIVE_PEAK_CURRENT_LIMIT = 60;
        public final static int DRIVE_CONTINUOS_CURRENT_LIMIT = 35;
        public final static double DRIVE_PEAK_CURRENT_DURATION = 0.1;
        public final static boolean DRIVE_ENBLE_CURRENT_LIMIT = true;

        //Swerve physics
        public final static double MAX_VELOCITY =  5.1;
        //public final static double MAX_ACCELERATION = (10.91 / 1.15) * 1.3; 
        public final static double MAX_ANGULAR_VELOCITY = MAX_VELOCITY / RADIUS;// Radians
        public final static SwerveDriveKinematics kinematics = new SwerveDriveKinematics(SwerveConstants.frontLeftLocation, SwerveConstants.frontRightLocation,
        SwerveConstants.rearLeftLocation, SwerveConstants.rearRightLocation);

        //Swerve controllers
        //Swerve CONTROLLER
        public final static double KP_TRANSLATION = 9; // 3.3;
        public final static double KI_TRANSLATION = 0; //0.0009;

        //Swerve theta PID_CONTROLLER radians
        public final static double THATA_KP = 5.4;
        public final static double THATA_KI = 0.4;
        public final static double THATA_KD = 0.0;

        //Module Limits
        public final static ModuleLimits DEFUALT = new ModuleLimits(5.3, Units.feetToMeters(75.0), Units.degreesToRadians(1080)); 
        public final static ModuleLimits Slow40Precent = new ModuleLimits(5.3 * 0.4, Units.feetToMeters(75.0), Units.degreesToRadians(1080)); 
        public final static ModuleLimits Slow10Precent = new ModuleLimits(5.3 * 0.1, Units.feetToMeters(75.0), Units.degreesToRadians(1080)); 
        
        
        //Collision Detector
        public final static double COLLISION_THRESHOLD = 2d;
}