package org.firstinspires.ftc.teamcode.Swerve;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * teleop swerve drive controller.
 * handles 4 independent swerve modules with imu-based field-centric control.
 */
public class SwerveDrive {
    private final SwerveModule lf;
    private final SwerveModule rf;
    private final SwerveModule lb;
    private final SwerveModule rb;
    private final IMU imu;
    private final VoltageSensor voltageSensor;
    private final SwerveKinematics kinematics;

    private double headingOffset = 0.0;

    public SwerveDrive(HardwareMap hardwareMap) {
        // init drive motors
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, SwerveConstants.LEFT_FRONT_MOTOR);
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, SwerveConstants.RIGHT_FRONT_MOTOR);
        DcMotorEx leftBack = hardwareMap.get(DcMotorEx.class, SwerveConstants.LEFT_BACK_MOTOR);
        DcMotorEx rightBack = hardwareMap.get(DcMotorEx.class, SwerveConstants.RIGHT_BACK_MOTOR);

        // init steering servos
        CRServo steerFL = hardwareMap.get(CRServo.class, SwerveConstants.STEER_LF);
        CRServo steerFR = hardwareMap.get(CRServo.class, SwerveConstants.STEER_RF);
        CRServo steerBL = hardwareMap.get(CRServo.class, SwerveConstants.STEER_LB);
        CRServo steerBR = hardwareMap.get(CRServo.class, SwerveConstants.STEER_RB);

        // init analog sensors
        AnalogInput sensorFL = hardwareMap.get(AnalogInput.class, SwerveConstants.SENSOR_LF);
        AnalogInput sensorFR = hardwareMap.get(AnalogInput.class, SwerveConstants.SENSOR_RF);
        AnalogInput sensorBL = hardwareMap.get(AnalogInput.class, SwerveConstants.SENSOR_LB);
        AnalogInput sensorBR = hardwareMap.get(AnalogInput.class, SwerveConstants.SENSOR_RB);

        // create modules
        lf = new SwerveModule(leftFront, steerFL, sensorFL);
        rf = new SwerveModule(rightFront, steerFR, sensorFR);
        lb = new SwerveModule(leftBack, steerBL, sensorBL);
        rb = new SwerveModule(rightBack, steerBR, sensorBR);

        // init kinematics
        kinematics = new SwerveKinematics();

        // init imu
        imu = hardwareMap.get(IMU.class, "imu");
        /// TODO: FIX orientation for your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        ));
        imu.initialize(parameters);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    /** update all module odometry - must be called every loop */
    public void update() {
        lf.update();
        rf.update();
        lb.update();
        rb.update();
    }

    /**
     * field-centric drive
     */
    public void driveFieldCentric(double x, double y, double rx) {
        double headingRad = Math.toRadians(getHeading());
        double[] robotVel = SwerveKinematics.fieldToRobot(x, y, headingRad);
        driveRobotCentric(robotVel[0], robotVel[1], rx);
    }

    /**
     * robot-centric drive (no imu transform)
     */
    public void driveRobotCentric(double x, double y, double rx) {
        double inputMagnitude = Math.hypot(x, y) + Math.abs(rx);
        if (inputMagnitude < SwerveConstants.INPUT_DEADBAND) {
            stop();
            return;
        }

        SwerveModuleState[] states = kinematics.calculate(x, y, rx);
        setModuleStates(states);
        executeModules();
    }

    /**
     * drive with selectable mode
     */
    public void drive(double x, double y, double rx, boolean fieldCentric) {
        if (fieldCentric) {
            driveFieldCentric(x, y, rx);
        } else {
            driveRobotCentric(x, y, rx);
        }
    }

    /** set states for all modules */
    private void setModuleStates(SwerveModuleState[] states) {
        lf.setTargetState(states[0]);
        rf.setTargetState(states[1]);
        lb.setTargetState(states[2]);
        rb.setTargetState(states[3]);
    }

    /** execute all modules with voltage compensation */
    private void executeModules() {
        double voltageCompensation = SwerveConstants.NOMINAL_VOLTAGE / voltageSensor.getVoltage();
        lf.execute(voltageCompensation);
        rf.execute(voltageCompensation);
        lb.execute(voltageCompensation);
        rb.execute(voltageCompensation);
    }

    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - headingOffset;
    }

    public void resetYaw() {
        headingOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void stop() {
        lf.stop();
        rf.stop();
        lb.stop();
        rb.stop();
    }

    // --- accessors ---
    public SwerveModule getModuleFL() { return lf; }
    public SwerveModule getModuleFR() { return rf; }
    public SwerveModule getModuleBL() { return lb; }
    public SwerveModule getModuleBR() { return rb; }

    public double getVoltage() { return voltageSensor.getVoltage(); }

    /** reload constants for all modules */
    public void reloadConstants() {
        lf.reloadConstants();
        rf.reloadConstants();
        lb.reloadConstants();
        rb.reloadConstants();
    }
}
