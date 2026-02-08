package org.firstinspires.ftc.teamcode.Swerve;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class SwerveDrive {
    private final SwerveModule lf;
    private final SwerveModule rf;
    private final SwerveModule lb;
    private final SwerveModule rb;
    private final IMU imu;
    private final VoltageSensor voltageSensor;

    private static final double NOMINAL_VOLTAGE = 13.1;

    // module positions relative to center (meters)
    /// TOOD: MEASURE
    private static final double TRACK_WIDTH = 0.35;  //L-R distance between wheels
    private static final double WHEEL_BASE = 0.35;   //F-B distance between wheels

    //module position vectors
    private static final double[] MODULE_X = { WHEEL_BASE / 2,  WHEEL_BASE / 2, -WHEEL_BASE / 2, -WHEEL_BASE / 2 };
    private static final double[] MODULE_Y = { TRACK_WIDTH / 2, -TRACK_WIDTH / 2, TRACK_WIDTH / 2, -TRACK_WIDTH / 2 };

    private double headingOffset = 0.0;

    public SwerveDrive(HardwareMap hardwareMap) {
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        DcMotorEx leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        DcMotorEx rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        CRServo steerFL = hardwareMap.get(CRServo.class, "steerFL");
        CRServo steerFR = hardwareMap.get(CRServo.class, "steerFR");
        CRServo steerBL = hardwareMap.get(CRServo.class, "steerBL");
        CRServo steerBR = hardwareMap.get(CRServo.class, "steerBR");

        AnalogInput sensorFL = hardwareMap.get(AnalogInput.class, "sensorFL");
        AnalogInput sensorFR = hardwareMap.get(AnalogInput.class, "sensorFR");
        AnalogInput sensorBL = hardwareMap.get(AnalogInput.class, "sensorBL");
        AnalogInput sensorBR = hardwareMap.get(AnalogInput.class, "sensorBR");

        lf = new SwerveModule(leftFront, steerFL, sensorFL);
        rf = new SwerveModule(rightFront, steerFR, sensorFR);
        lb = new SwerveModule(leftBack, steerBL, sensorBL);
        rb = new SwerveModule(rightBack, steerBR, sensorBR);

        imu = hardwareMap.get(IMU.class, "imu");
        /// TODO:FIX
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        ));
        imu.initialize(parameters);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    public void update() {
        lf.update();
        rf.update();
        lb.update();
        rb.update();
    }

    /**
     * field-centric drive
     */
    public void drivefcd(double x, double y, double rx) {
        double heading = getHeading();
        double headingRad = Math.toRadians(heading);

        double cos = Math.cos(-headingRad);
        double sin = Math.sin(-headingRad);

        double robotX = x * cos - y * sin;
        double robotY = x * sin + y * cos;

        drive(robotX, robotY, rx);
    }

    /**
     * robot-centric drive (no imu transform)
     */
    public void drive(double x, double y, double rx) {
        double inputMagnitude = Math.hypot(x, y) + Math.abs(rx);
        if (inputMagnitude < 0.05) {
            stop();
            return;
        }

        double[] wheelX = new double[4];
        double[] wheelY = new double[4];
        double[] wheelSpeed = new double[4];
        double[] wheelAngle = new double[4];

        double maxSpeed = 0.0;

        for (int i = 0; i < 4; i++) {
            double rotX = -MODULE_X[i] * rx;
            double rotY = MODULE_Y[i] * rx;

            wheelX[i] = x + rotX;
            wheelY[i] = y + rotY;

            wheelSpeed[i] = Math.hypot(wheelX[i], wheelY[i]);
            wheelAngle[i] = Math.toDegrees(Math.atan2(wheelX[i], wheelY[i]));

            maxSpeed = Math.max(maxSpeed, wheelSpeed[i]);
        }

        if (maxSpeed > 1.0) {
            for (int i = 0; i < 4; i++) {
                wheelSpeed[i] /= maxSpeed;
            }
        }

        lf.setTargetState(wheelAngle[0], wheelSpeed[0]);
        rf.setTargetState(wheelAngle[1], wheelSpeed[1]);
        lb.setTargetState(wheelAngle[2], wheelSpeed[2]);
        rb.setTargetState(wheelAngle[3], wheelSpeed[3]);

        double voltageCompensation = NOMINAL_VOLTAGE / voltageSensor.getVoltage();

        lf.execute(voltageCompensation);
        rf.execute(voltageCompensation);
        lb.execute(voltageCompensation);
        rb.execute(voltageCompensation);
    }

    /**
     * drive with selectable mode
     */
    public void drive(double x, double y, double rx, boolean fieldCentric) {
        if (fieldCentric) {
            drivefcd(x, y, rx);
        } else {
            drive(x, y, rx);
        }
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

    public SwerveModule getModuleFL() { return lf; }
    public SwerveModule getModuleFR() { return rf; }
    public SwerveModule getModuleBL() { return lb; }
    public SwerveModule getModuleBR() { return rb; }

    public double getVoltage() { return voltageSensor.getVoltage(); }
}
