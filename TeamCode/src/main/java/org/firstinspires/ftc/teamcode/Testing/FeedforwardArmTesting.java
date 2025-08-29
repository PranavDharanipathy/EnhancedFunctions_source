package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.EnhancedFunctions.FeedforwardArm;

@TeleOp(group = "testing")
public class FeedforwardArmTesting extends OpMode {

    private FeedforwardArm arm;

    @Override
    public void init() {
        arm = new FeedforwardArm(hardwareMap, "arm", 1, 1, 547.7, 0);
    }

    @Override
    public void loop() {

    }
}
