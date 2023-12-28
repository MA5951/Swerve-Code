package frc.robot.subsystems.swerve;

public class SwerveConstants {
        // swerve constants
        public final static double WIDTH = 0.66;
        public final static double LENGTH = 0.66;
        public final static double RADIUS = Math.sqrt(
                        Math.pow(WIDTH, 2) + Math.pow(LENGTH, 2)) / 2.0;

        // Modules constants
        public final static double TURNING_GEAR_RATIO = 150d / 7;
        private final static double DRIVE_GEAR_RATIO = 6.12;
        private final static double WHEEL_RADIUS = 0.05;

        public final static double VELOCITY_TIME_UNIT_IN_SECONDS = 1;

        public final static double DISTANCE_PER_PULSE = (2 * WHEEL_RADIUS * Math.PI)
                        / DRIVE_GEAR_RATIO;
        public final static double ANGLE_PER_PULSE = 360d / TURNING_GEAR_RATIO;

        // front left module
        public final static double FRONT_LEFT_MODULE_OFFSET_ENCODER = 88;
        public final static boolean FRONT_LEFT_MOUDLE_IS_DRIVE_MOTOR_REVERSED = true;
        public final static boolean FRONT_LEFT_MODULES_IS_TURNING_MOTOR_REVERSED = false;
        public final static boolean FRONT_LEFT_MODULE_IS_ABSOLUTE_ENCODER_REVERSED = true;

        // front right module
        public final static double FRONT_RIGHT_MODULE_OFFSET_ENCODER = 77.255859375;
        public final static boolean FRONT_RIGHT_MOUDLE_IS_DRIVE_MOTOR_REVERSED = false;
        public final static boolean FRONT_RIGHT_MODULES_IS_TURNING_MOTOR_REVERSED = false;
        public final static boolean FRONT_RIGHT_MODULE_IS_ABSOLUTE_ENCODER_REVERSED = true;

        // rear left module
        public final static double REAR_LEFT_MODULE_OFFSET_ENCODER = 207.59765625;
        public final static boolean REAR_LEFT_MOUDLE_IS_DRIVE_MOTOR_REVERSED = true;
        public final static boolean REAR_LEFT_MODULES_IS_TURNING_MOTOR_REVERSED = false;
        public final static boolean REAR_LEFT_MODULE_IS_ABSOLUTE_ENCODER_REVERSED = true;

        // rear right module
        public final static double REAR_RIGHT_MODULE_OFFSET_ENCODER = 77;
        public final static boolean REAR_RIGHT_MOUDLE_IS_DRIVE_MOTOR_REVERSED = false;
        public final static boolean REAR_RIGHT_MODULES_IS_TURNING_MOTOR_REVERSED = false;
        public final static boolean REAR_RIGHT_MODULE_IS_ABSOLUTE_ENCODER_REVERSED = true;

        // Modules turning config
        // PID
        public final static double turningPIDKP = 3.6;
        public final static double turningPIDKI = 0;
        public final static double turningPIDKD = 0;
        // Ramp
        public final static double openloopRamp = 0.25;
        public final static double closedloopRamp = 0;
        // Current Limit
        public final static int turningContinuousCurrentLimit = 25;
        public final static int turningPeakCurrentLimit = 400;
        public final static double turningPeakCurrentDuration = 0.1;
        public final static boolean turningEnableCurrentLimit = true;

        // Modules drive config
        // PID
        public final static double DRIVE_PID_KP = 3.1;
        public final static double DRIVE_PID_KI = 0;
        public final static double DRIVE_PID_KD = 0.03;
        public final static double DRIVE_KS = 0.05 * 12;
        public final static double DRIVE_KV = 0.206 * 12;
        // Current Limit
        public final static int DRIVE_CONTINUOS_CURRENT_LIMIT = 35;
        public final static int DRIVE_PEAK_CURRENT_LIMIT = 400;
        public final static double DRIVE_PEAK_CURRENT_DURATION = 0.1;
        public final static boolean DRIVE_ENBLE_CURRENT_LIMIT = true;

        // swerve physics
        public final static double MAX_VELOCITY = 5.3;
        public final static double MAX_ACCELERATION = Math.pow(MAX_VELOCITY, 2) / RADIUS;
        public final static double MAX_ANGULAR_VELOCITY = MAX_VELOCITY / RADIUS; // radians

        // swerve controllers

        // swerve x CONTROLLER
        public final static double KP_X = 3.3;
        public final static double KI_X = 0.0009;

        // swerve y CONTROLLER
        public final static double KP_Y = 3.3;
        public final static double KI_Y = 0.0009;

        // swerve theta PID_CONTROLLER radians
        public final static double THATA_KP = 2.9;
        public final static double THATA_KI = 0;
        public final static double THATA_KD = 0;

        // swerve theta PROFILED_PID_CONTROLLER radians
        public final static double PROFILED_THATA_KP = 3;
        public final static double PROFILED_THATA_KI = 0.003;
        public final static double PROFILED_THATA_KD = 0;
        public final static double MAX_ANGULAR_ACCELERATION = 15;

        // gyro drift detection
        public static final double ANGLE_CHANGE_THRESHOLD = 1.0; // TODO: 
        public static final double DETECTION_TIME = 0.1;         // in seconds TODO:
        public static final double UPDATE_RATE = 20.0;           // Update rate in Hz
}