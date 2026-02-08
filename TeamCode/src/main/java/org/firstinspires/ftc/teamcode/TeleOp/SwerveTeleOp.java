package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.helpers.Toggle;

@TeleOp(name = "Swerve TeleOp")
public class SwerveTeleOp extends OpMode {

    private SwerveDrive swerve;
    private boolean fieldCentric = true;
    Toggle dtToggle = new Toggle();

    @Override
    public void init() {
        swerve = new SwerveDrive(hardwareMap);
        swerve.resetYaw();

        telemetry.addLine("swerve initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {

        swerve.update();
    }


    @Override
    public void loop() {

        swerve.update();

        // get joystick inputs
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;


        boolean prevState = dtToggle.get();
        boolean currState = dtToggle.update(gamepad1.ps);
        if (currState && !prevState) {
            fieldCentric = !fieldCentric;
        }

        swerve.drive(x, y, rx, fieldCentric);

        if (gamepad1.dpad_down) {
            swerve.resetYaw();
        }

        // telemetry
        telemetry.addData("mode", fieldCentric ? "FIELD-CENTRIC" : "ROBOT-CENTRIC");
        telemetry.addData("heading", "%.1f°", swerve.getHeading());
        telemetry.addData("voltage", "%.2fV", swerve.getVoltage());
        telemetry.addLine();
        telemetry.addData("fl", "cur: %.1f° | tgt: %.1f°",
                swerve.getModuleFL().getCurrentAngle(),
                swerve.getModuleFL().getTargetAngle());
        telemetry.addData("fr", "cur: %.1f° | tgt: %.1f°",
                swerve.getModuleFR().getCurrentAngle(),
                swerve.getModuleFR().getTargetAngle());
        telemetry.addData("bl", "cur: %.1f° | tgt: %.1f°",
                swerve.getModuleBL().getCurrentAngle(),
                swerve.getModuleBL().getTargetAngle());
        telemetry.addData("br", "cur: %.1f° | tgt: %.1f°",
                swerve.getModuleBR().getCurrentAngle(),
                swerve.getModuleBR().getTargetAngle());
        telemetry.update();
    }

    @Override
    public void stop() {
        swerve.stop();
    }
}
