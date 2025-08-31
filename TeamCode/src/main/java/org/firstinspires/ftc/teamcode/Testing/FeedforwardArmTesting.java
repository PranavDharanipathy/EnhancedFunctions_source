package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.EnhancedFunctions.FeedforwardArm;

@TeleOp(group = "testing")
public class FeedforwardArmTesting extends OpMode {

    private FeedforwardArm arm;

    private static final double[] VOLTAGE_DATA = {13, 13, 13};
    private static final double[] KF_DATA = {13, 13, 13};

    @Override
    public void init() {
        arm = new FeedforwardArm(hardwareMap, "arm", 1, 1, 537.7, 0);
        arm.changeVoltageData(VOLTAGE_DATA, KF_DATA);
    }

    @Override
    public void loop() {
        arm.update();
    }
}
