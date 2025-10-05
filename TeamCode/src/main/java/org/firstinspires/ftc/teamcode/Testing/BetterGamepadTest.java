package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.EnhancedFunctions.BetterGamepad;

@TeleOp (group = "testing")
public class BetterGamepadTest extends OpMode {

    private BetterGamepad betterGamepad1;

    @Override
    public void init() {

        betterGamepad1 = new BetterGamepad(gamepad1);
    }

    @Override
    public void loop() {

        telemetry.addData("a", betterGamepad1.a());
        telemetry.addData("a has just been pressed", betterGamepad1.aHasJustBeenPressed);

        telemetry.update();

    }
}
