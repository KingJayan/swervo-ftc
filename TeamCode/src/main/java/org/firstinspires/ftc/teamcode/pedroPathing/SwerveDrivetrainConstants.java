package org.firstinspires.ftc.teamcode.pedroPathing;

import org.firstinspires.ftc.teamcode.Swerve.SwerveConstants;

/**
 * constants for pedro pathing swerve drivetrain.
 * delegates to unified SwerveConstants for hardware names and dimensions.
 * @deprecated use SwerveConstants directly
 */
@Deprecated
public class SwerveDrivetrainConstants {
    // drive motor names (delegated to SwerveConstants)
    public String leftFrontMotorName = SwerveConstants.LEFT_FRONT_MOTOR;
    public String rightFrontMotorName = SwerveConstants.RIGHT_FRONT_MOTOR;
    public String leftBackMotorName = SwerveConstants.LEFT_BACK_MOTOR;
    public String rightBackMotorName = SwerveConstants.RIGHT_BACK_MOTOR;

    // steering servo names (delegated to SwerveConstants)
    public String steerLFName = SwerveConstants.STEER_LF;
    public String steerRFName = SwerveConstants.STEER_RF;
    public String steerLBName = SwerveConstants.STEER_LB;
    public String steerRBName = SwerveConstants.STEER_RB;

    // analog encoder names (delegated to SwerveConstants)
    public String sensorLFName = SwerveConstants.SENSOR_LF;
    public String sensorRFName = SwerveConstants.SENSOR_RF;
    public String sensorLBName = SwerveConstants.SENSOR_LB;
    public String sensorRBName = SwerveConstants.SENSOR_RB;

    // robot dimensions (delegated to SwerveConstants)
    public static double TRACK_WIDTH = SwerveConstants.TRACK_WIDTH;
    public static double WHEEL_BASE = SwerveConstants.WHEEL_BASE;

    // voltage compensation (delegated to SwerveConstants)
    public static double NOMINAL_VOLTAGE = SwerveConstants.NOMINAL_VOLTAGE;

    // gear ratio (delegated to SwerveConstants)
    public static double GEAR_RATIO = SwerveConstants.GEAR_RATIO;

    // steering pid (delegated to SwerveConstants)
    public static double STEER_KP = SwerveConstants.STEER_KP;
    public static double STEER_KSTATIC = SwerveConstants.STEER_KSTATIC;
    public static double STEER_TOLERANCE = SwerveConstants.STEER_TOLERANCE;
}