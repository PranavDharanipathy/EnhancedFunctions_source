package org.firstinspires.ftc.teamcode.Tuners;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.EnhancedFunctions.FeedforwardArm;

@TeleOp(group = "testing")
public class FeedforwardArmTuner extends LinearOpMode {

    public static double TICKS_PER_REV = 537.7;
    public static double kf = 1;

    private FeedforwardArm arm;

    private VoltageSensor batteryVoltageSensor;

    @Override
    public void runOpMode() {

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        arm = new FeedforwardArm(hardwareMap, "arm", kf, 1, TICKS_PER_REV, 0);
        arm.tuningMode();

        if (isStopRequested()) return;
        waitForStart();

        while (opModeIsActive() && !gamepad1.dpad_up) {
            telemetry.addLine("First put the arm at it's startPosition.");
            telemetry.addData("Here is it's position in ticks", arm.motor.getCurrentPosition());
            telemetry.addLine("Click dpad_up to move to next step.");
            telemetry.update();
        }

        boolean curr_gamepad1a = false;
        boolean prev_gamepad1a;

        boolean curr_gamepad1b = false;
        boolean prev_gamepad1b;

        boolean curr_gamepad1x = false;
        boolean prev_gamepad1x;

        boolean curr_gamepad1y = false;
        boolean prev_gamepad1y;
        while (opModeIsActive()) {

            prev_gamepad1a = curr_gamepad1a;
            curr_gamepad1a = gamepad1.a;

            prev_gamepad1b = curr_gamepad1b;
            curr_gamepad1b = gamepad1.b;

            prev_gamepad1x = curr_gamepad1x;
            curr_gamepad1x = gamepad1.x;

            prev_gamepad1y = curr_gamepad1y;
            curr_gamepad1y = gamepad1.y;

            if (curr_gamepad1a && !prev_gamepad1a) {
                kf+=0.01;
            }
            else if (curr_gamepad1b && !prev_gamepad1b) {
                kf+=0.001;
            }
            else if (curr_gamepad1x && !prev_gamepad1x) {
                kf-=0.01;
            }
            else if (curr_gamepad1y && !prev_gamepad1y) {
                kf-=0.001;
            }

            arm.kfUpdatingForTuning(kf);
            arm.update();

            telemetry.addLine("Get the arm to hold at the horizontal position (parallel to the ground).");
            telemetry.addLine(" ");
            telemetry.addLine("Tune as high as you can using a.");
            telemetry.addLine("Then switch to tuning using b.");
            telemetry.addLine("a: 0.01");
            telemetry.addLine("b: 0.001");
            telemetry.addLine("x: -0.01");
            telemetry.addLine("y: -0.001");
            telemetry.addLine(" ");
            telemetry.addData("kf", kf);
            telemetry.addLine(" ");
            telemetry.addLine("You must run this more that one at a different voltage.");
            telemetry.addData("Voltage", batteryVoltageSensor.getVoltage());
            telemetry.addLine(" ");
            telemetry.addLine("Add your tuned kf value to the KF_DATA list.");
            telemetry.addLine("Add your voltage value to the VOLTAGE_DATA list.");
            telemetry.update();
        }

    }
}
