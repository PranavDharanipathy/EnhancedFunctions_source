package org.firstinspires.ftc.teamcode.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.EnhancedFunctions.FeedforwardArm;

@Config
@TeleOp(group = "tuning")
public class FeedforwardArmTuner extends LinearOpMode {

    public static double TICKS_PER_REV = 537.7;
    public static double kf = 0; //tune from dashboard (192.168.43.1:8080/dash)

    /*
    * Set kf to 0.001
    * and slowly increase to tune.
    */

    private FeedforwardArm arm;

    private VoltageSensor batteryVoltageSensor;

    @Override
    public void runOpMode() {

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        arm = new FeedforwardArm(hardwareMap, "arm", kf, batteryVoltageSensor.getVoltage(), 1, TICKS_PER_REV, 0);
        arm.tuningMode();

        telemetry.addLine("First put the arm at it's startPosition.");
        telemetry.addData("Here is the arm's position in ticks", arm.motor.getCurrentPosition());
        telemetry.update();

        if (isStopRequested()) return;
        waitForStart();

        while (opModeIsActive()) {
            arm.kfUpdatingForTuning(kf);
            arm.update();

            telemetry.addLine("Get the arm to hold at the horizontal position (parallel to the ground).");
            telemetry.addLine(" ");
            telemetry.addData("kf", kf);
            telemetry.addLine(" ");
            telemetry.addLine("You must run this 3 times in total, each time at a different voltage.");
            telemetry.addData("Voltage", batteryVoltageSensor.getVoltage());
            telemetry.addLine(" ");
            telemetry.addLine("Add your tuned kf value to the KF_DATA list.");
            telemetry.addLine("Add your voltage value to the VOLTAGE_DATA list.");
            telemetry.addLine("Choose the kf at the most common voltage your bot runs at and set that as your kf when initializing your arm.");
            telemetry.addLine("Choose the corresponding battery volatage and set that as your kvoltage.");
            telemetry.update();
        }

    }
}
