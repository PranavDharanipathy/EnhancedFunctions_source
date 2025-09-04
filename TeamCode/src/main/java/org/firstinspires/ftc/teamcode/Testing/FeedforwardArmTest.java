package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.EnhancedFunctions.FeedforwardArm;

@TeleOp(group = "testing")
public class FeedforwardArmTest extends OpMode {

    private FeedforwardArm arm;

    //arbitrary
    private static final double[] VOLTAGE_DATA = {13, 13, 13};
    private static final double[] KF_DATA = {13, 13, 13};

    public static int TARGET_POSITION1 = 200;
    public static int TARGET_POSITION2 = -200;
    public static int TARGET_POSITION1_THRESHOLD = 10;
    public static int TARGET_POSITION2_THRESHOLD = 10;

    private VoltageSensor batteryVoltageSensor;

    @Override
    public void init() {

        batteryVoltageSensor = hardwareMap.voltageSensor.get("Control Hub");

        //all values are arbitrary
        arm = new FeedforwardArm(hardwareMap, "arm", batteryVoltageSensor, 1, 13.0, 1, 537.7, 0);

        //arm data given
        arm.changeVoltageData(VOLTAGE_DATA, KF_DATA);

        //2 positions and their thresholds are given
        arm.addThreshold(TARGET_POSITION1, TARGET_POSITION1_THRESHOLD);
        arm.addThreshold(TARGET_POSITION2, TARGET_POSITION2_THRESHOLD);
    }

    @Override
    public void loop() {

        if (gamepad1.a) {
            arm.setPosition(TARGET_POSITION1,1);
        }
        else if (gamepad1.b) {
            arm.setPosition(TARGET_POSITION2,1);
        }

        arm.update();
    }
}
