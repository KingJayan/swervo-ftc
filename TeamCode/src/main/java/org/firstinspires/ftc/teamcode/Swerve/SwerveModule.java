package org.firstinspires.ftc.teamcode.Swerve;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

public class SwerveModule {
    private final DcMotorEx drive;
    private final CRServo steer;
    private final AnalogInput sensor;

    // pid constants (loaded from SwerveConstants)
    /// TODO: TUNE
    private double kP;
    private double kStatic;
    private double angleTolerance;

    // odometer tracking for rollover
    private double lastServoAngle = 0.0;
    private double totalServoRotations = 0.0;
    private boolean initialized = false;

    // current state
    private double currentModuleAngle = 0.0;
    private double targetAngle = 0.0;
    private double drivePower = 0.0;
    private boolean driveReversed = false;

    public SwerveModule(DcMotorEx drive, CRServo steer, AnalogInput sensor) {
        this.drive = drive;
        this.steer = steer;
        this.sensor = sensor;

        // load constants
        this.kP = SwerveConstants.STEER_KP;
        this.kStatic = SwerveConstants.STEER_KSTATIC;
        this.angleTolerance = SwerveConstants.STEER_TOLERANCE;

        // configure drive motor
        this.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * must be called every loop cycle to track servo position rollovers.
     * since sensor is on servo shaft (before 3.2:1 reduction), we track
     * cumulative servo rotations to calculate true module angle.
     */
    public void update() {
        double currentServoAngle = getServoAngle();

        if (!initialized) {
            lastServoAngle = currentServoAngle;
            initialized = true;
            return;
        }

        // detect rollover: if angle jumps by more than 180, it wrapped around
        double delta = currentServoAngle - lastServoAngle;

        if (delta > 180.0) {
            // wrapped from low to high (e.g., 350 -> 10 reads as -340, so delta was negative)
            // actually: went backwards across 0 (e.g., 10 -> 350)
            totalServoRotations -= 1.0;
        } else if (delta < -180.0) {
            // wrapped from high to low (e.g., 10 -> 350 reads as +340)
            // actually: went forwards across 0 (e.g., 350 -> 10)
            totalServoRotations += 1.0;
        }

        lastServoAngle = currentServoAngle;

        // calculate module angle: total servo travel / gear ratio
        // totalServoRotations tracks full rotations, currentServoAngle is partial
        double totalServoDegrees = (totalServoRotations * 360.0) + currentServoAngle;
        currentModuleAngle = totalServoDegrees / SwerveConstants.GEAR_RATIO;
    }

    /**
     * read raw servo angle from analog sensor (0-360 degrees)
     */
    private double getServoAngle() {
        double voltage = sensor.getVoltage();
        voltage = Math.min(SwerveConstants.MAX_SENSOR_VOLTAGE, Math.max(0, voltage));
        return (voltage / SwerveConstants.MAX_SENSOR_VOLTAGE) * 360.0;
    }

    /**
     * set target state for module using SwerveModuleState.
     * applies optimization automatically.
     * @param state the desired module state
     */
    public void setTargetState(SwerveModuleState state) {
        SwerveModuleState optimized = state.optimize(currentModuleAngle);
        this.targetAngle = optimized.angle;
        this.drivePower = optimized.speed;
        this.driveReversed = (optimized.speed != state.speed);
    }

    /**
     * set target state for module. applies optimization automatically.
     * @param angle target module angle in degrees
     * @param power drive motor power (-1 to 1)
     */
    public void setTargetState(double angle, double power) {
        setTargetState(new SwerveModuleState(angle, power));
    }

    /**
     * execute control loop - call after update() and setTargetState()
     * @param voltageCompensation multiplier for voltage compensation (12.0 / currentVoltage)
     */
    public void execute(double voltageCompensation) {
        // --- steering p-controller ---
        double error = angleWrap(targetAngle - currentModuleAngle);

        double steerPower = 0.0;
        if (Math.abs(error) > angleTolerance) {
            // p term
            steerPower = kP * error;

            // add static friction compensation
            if (steerPower > 0) {
                steerPower += kStatic;
            } else if (steerPower < 0) {
                steerPower -= kStatic;
            }

            // clamp to valid range
            steerPower = Math.max(-1.0, Math.min(1.0, steerPower));
        }

        steer.setPower(steerPower);

        // --- drive motor ---
        double finalDrivePower = drivePower;
        if (driveReversed) {
            finalDrivePower = -finalDrivePower;
        }

        // apply voltage compensation
        finalDrivePower *= voltageCompensation;
        finalDrivePower = Math.max(-1.0, Math.min(1.0, finalDrivePower));

        drive.setPower(finalDrivePower);
    }

    /**
     * wrap angle to -180 to 180 range
     */
    private double angleWrap(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }

    /**
     * get current module angle (after gear reduction)
     */
    public double getCurrentAngle() {
        return currentModuleAngle;
    }

    /**
     * get target angle
     */
    public double getTargetAngle() {
        return targetAngle;
    }

    /**
     * stop all motion
     */
    public void stop() {
        drive.setPower(0);
        steer.setPower(0);
    }

    // --- tuning setters ---
    public void setKp(double kP) { this.kP = kP; }
    public void setKStatic(double kStatic) { this.kStatic = kStatic; }
    public void setAngleTolerance(double tolerance) { this.angleTolerance = tolerance; }

    /** reload constants from SwerveConstants */
    public void reloadConstants() {
        this.kP = SwerveConstants.STEER_KP;
        this.kStatic = SwerveConstants.STEER_KSTATIC;
        this.angleTolerance = SwerveConstants.STEER_TOLERANCE;
    }
}
