package org.firstinspires.ftc.teamcode.Swerve;

/**
 * unified constants for swerve drive configuration.
 * used by both teleop SwerveDrive and Pedro Pathing SwerveDrivetrain.
 */
public class SwerveConstants {
    // --- hardware names ---
    // drive motor names
    public static String LEFT_FRONT_MOTOR = "leftFront";
    public static String RIGHT_FRONT_MOTOR = "rightFront";
    public static String LEFT_BACK_MOTOR = "leftBack";
    public static String RIGHT_BACK_MOTOR = "rightBack";

    // steering servo names (cr servos)
    public static String STEER_LF = "steerFL";
    public static String STEER_RF = "steerFR";
    public static String STEER_LB = "steerBL";
    public static String STEER_RB = "steerBR";

    // analog encoder names
    public static String SENSOR_LF = "sensorFL";
    public static String SENSOR_RF = "sensorFR";
    public static String SENSOR_LB = "sensorBL";
    public static String SENSOR_RB = "sensorBR";

    // --- robot dimensions (meters) ---
    /// TODO: MEASURE
    public static double TRACK_WIDTH = 0.35;  // L-R distance between wheels
    public static double WHEEL_BASE = 0.35;   // F-B distance between wheels

    // --- voltage compensation ---
    public static double NOMINAL_VOLTAGE = 13.1;

    // --- gear ratio ---
    // gear reduction: 3.2 servo rotations = 1 module rotation
    // sensor is on servo shaft (before reduction)
    public static double GEAR_RATIO = 3.2;

    // --- analog sensor ---
    public static double MAX_SENSOR_VOLTAGE = 3.3;

    // --- steering pid constants ---
    /// TODO: TUNE
    public static double STEER_KP = 0.02;
    public static double STEER_KSTATIC = 0.05;
    public static double STEER_TOLERANCE = 2.0; // degrees

    // --- drive thresholds ---
    public static double INPUT_DEADBAND = 0.05;
    public static double SPEED_DEADBAND = 0.01;
}
