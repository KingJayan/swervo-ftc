package org.firstinspires.ftc.teamcode.Swerve;

/**
 * data class representing a swerve module's target state.
 * contains angle (degrees) and speed (-1 to 1).
 */
public class SwerveModuleState {
    public double angle;
    public double speed;

    public SwerveModuleState(double angle, double speed) {
        this.angle = angle;
        this.speed = speed;
    }

    public SwerveModuleState() {
        this(0, 0);
    }

    /**
     * optimization: if target > 90 deg from current, flip drive and rotate 180.
     * reduces max rotation to 90 deg for faster response.
     * @param currentAngle the module's current angle in degrees
     * @return optimized state (may have flipped speed and adjusted angle)
     */
    public SwerveModuleState optimize(double currentAngle) {
        double error = angleWrap(angle - currentAngle);

        if (Math.abs(error) > 90.0) {
            return new SwerveModuleState(angleWrap(angle + 180.0), -speed);
        }
        return new SwerveModuleState(angle, speed);
    }

    /**
     * wrap angle to -180 to 180 range
     */
    private static double angleWrap(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }
}
