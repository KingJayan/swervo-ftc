package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.Drivetrain;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Swerve.SwerveConstants;
import org.firstinspires.ftc.teamcode.Swerve.SwerveKinematics;
import org.firstinspires.ftc.teamcode.Swerve.SwerveModule;
import org.firstinspires.ftc.teamcode.Swerve.SwerveModuleState;

/**
 * custom swerve drivetrain for pedro pathing.
 * handles 4 independent swerve modules with cr servo steering and analog encoders.
 */
public class SwerveDrivetrain extends Drivetrain {
    private final SwerveModule lf;
    private final SwerveModule rf;
    private final SwerveModule lb;
    private final SwerveModule rb;
    private final VoltageSensor voltageSensor;
    private final SwerveKinematics kinematics;

    private final double[] driveOutputs = new double[8];

    // velocity tracking
    private double xVelocity = 0;
    private double yVelocity = 0;

    public SwerveDrivetrain(HardwareMap hardwareMap) {
        // init drive motors
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, SwerveConstants.LEFT_FRONT_MOTOR);
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, SwerveConstants.RIGHT_FRONT_MOTOR);
        DcMotorEx leftBack = hardwareMap.get(DcMotorEx.class, SwerveConstants.LEFT_BACK_MOTOR);
        DcMotorEx rightBack = hardwareMap.get(DcMotorEx.class, SwerveConstants.RIGHT_BACK_MOTOR);

        // init steering servos
        CRServo steerLF = hardwareMap.get(CRServo.class, SwerveConstants.STEER_LF);
        CRServo steerRF = hardwareMap.get(CRServo.class, SwerveConstants.STEER_RF);
        CRServo steerLB = hardwareMap.get(CRServo.class, SwerveConstants.STEER_LB);
        CRServo steerRB = hardwareMap.get(CRServo.class, SwerveConstants.STEER_RB);

        // init analog sensors
        AnalogInput sensorLF = hardwareMap.get(AnalogInput.class, SwerveConstants.SENSOR_LF);
        AnalogInput sensorRF = hardwareMap.get(AnalogInput.class, SwerveConstants.SENSOR_RF);
        AnalogInput sensorLB = hardwareMap.get(AnalogInput.class, SwerveConstants.SENSOR_LB);
        AnalogInput sensorRB = hardwareMap.get(AnalogInput.class, SwerveConstants.SENSOR_RB);

        // create modules
        lf = new SwerveModule(leftFront, steerLF, sensorLF);
        rf = new SwerveModule(rightFront, steerRF, sensorRF);
        lb = new SwerveModule(leftBack, steerLB, sensorLB);
        rb = new SwerveModule(rightBack, steerRB, sensorRB);

        // init kinematics
        kinematics = new SwerveKinematics();

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        setNominalVoltage(SwerveConstants.NOMINAL_VOLTAGE);
    }

    /** update module odometry - must be called every loop */
    public void update() {
        lf.update();
        rf.update();
        lb.update();
        rb.update();
    }

    @Override
    public double[] calculateDrive(Vector correctiveVector, Vector headingVector, Vector centripetalVector, double currentHeading) {
        double x = correctiveVector.getXComponent() + centripetalVector.getXComponent();
        double y = correctiveVector.getYComponent() + centripetalVector.getYComponent();
        double rx = headingVector.getMagnitude() * Math.signum(headingVector.getTheta());

        SwerveModuleState[] states = kinematics.calculate(x, y, rx);

        // pack: [lfAngle, lfSpeed, rfAngle, rfSpeed, lbAngle, lbSpeed, rbAngle, rbSpeed]
        driveOutputs[0] = states[0].angle; driveOutputs[1] = states[0].speed;
        driveOutputs[2] = states[1].angle; driveOutputs[3] = states[1].speed;
        driveOutputs[4] = states[2].angle; driveOutputs[5] = states[2].speed;
        driveOutputs[6] = states[3].angle; driveOutputs[7] = states[3].speed;

        return driveOutputs;
    }

    @Override
    public void runDrive(double[] outputs) {
        update();

        double totalSpeed = Math.abs(outputs[1]) + Math.abs(outputs[3]) + Math.abs(outputs[5]) + Math.abs(outputs[7]);
        if (totalSpeed < SwerveConstants.SPEED_DEADBAND) {
            stop();
            return;
        }

        // set module states
        lf.setTargetState(outputs[0], outputs[1]);
        rf.setTargetState(outputs[2], outputs[3]);
        lb.setTargetState(outputs[4], outputs[5]);
        rb.setTargetState(outputs[6], outputs[7]);

        // execute with voltage compensation
        double voltageComp = isVoltageCompensation() ? getNominalVoltage() / getVoltage() : 1.0;
        lf.execute(voltageComp);
        rf.execute(voltageComp);
        lb.execute(voltageComp);
        rb.execute(voltageComp);
    }

    @Override
    public void updateConstants() {
        lf.reloadConstants();
        rf.reloadConstants();
        lb.reloadConstants();
        rb.reloadConstants();
    }

    @Override
    public void breakFollowing() {
        stop();
    }

    @Override
    public void startTeleopDrive() {
        // nothing special needed
    }

    @Override
    public void startTeleopDrive(boolean fieldCentric) {
        // nothing special needed
    }

    @Override
    public double xVelocity() {
        return xVelocity;
    }

    @Override
    public double yVelocity() {
        return yVelocity;
    }

    @Override
    public void setXVelocity(double v) {
        xVelocity = v;
    }

    @Override
    public void setYVelocity(double v) {
        yVelocity = v;
    }

    @Override
    public double getVoltage() {
        return voltageSensor.getVoltage();
    }

    @Override
    public String debugString() {
        return String.format("lf:%.1f rf:%.1f lb:%.1f rb:%.1f",
                lf.getCurrentAngle(), rf.getCurrentAngle(), lb.getCurrentAngle(), rb.getCurrentAngle());
    }

    public void stop() {
        lf.stop();
        rf.stop();
        lb.stop();
        rb.stop();
    }

    // --- accessors ---
    public SwerveModule getModuleLF() { return lf; }
    public SwerveModule getModuleRF() { return rf; }
    public SwerveModule getModuleLB() { return lb; }
    public SwerveModule getModuleRB() { return rb; }
}
