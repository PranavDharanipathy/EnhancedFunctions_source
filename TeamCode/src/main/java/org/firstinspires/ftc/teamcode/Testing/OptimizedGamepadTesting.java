package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.EnhancedFunctions.OptimizedGamepad;

@TeleOp (group = "testing")
public class OptimizedGamepadTesting extends OpMode {

    private OptimizedGamepad opt_gamepad1 = new OptimizedGamepad(gamepad1);

    public void init() {}

    public void printBlank() {
        telemetry.addLine(" ");
    }

    @Override
    public void loop() {

        telemetry.addData("a", opt_gamepad1.booleans.a());
        telemetry.addData("a", opt_gamepad1.booleans.a());
        telemetry.addData("a", opt_gamepad1.booleans.a());
        telemetry.addData("a", opt_gamepad1.booleans.a());

        printBlank();

        telemetry.addData("main_button", opt_gamepad1.booleans.main_button());

        printBlank();

        telemetry.addData("dpad_up", opt_gamepad1.booleans.dpad_up());
        telemetry.addData("dpad_down", opt_gamepad1.booleans.dpad_down());
        telemetry.addData("dpad_left", opt_gamepad1.booleans.dpad_left());
        telemetry.addData("dpad_right", opt_gamepad1.booleans.dpad_right());

        printBlank();

        telemetry.addData("left_stick_y", opt_gamepad1.floats.left_stick_y());
        telemetry.addData("left_stick_x", opt_gamepad1.floats.left_stick_x());
        telemetry.addData("right_stick_y", opt_gamepad1.floats.right_stick_y());
        telemetry.addData("right_stick_x", opt_gamepad1.floats.right_stick_x());

        printBlank();

        telemetry.addData("atRest", opt_gamepad1.booleans.atRest());

        telemetry.update();

    }
}
