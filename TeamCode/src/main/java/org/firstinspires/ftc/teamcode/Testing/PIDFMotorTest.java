package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.EnhancedFunctions.PIDFMotor;

@TeleOp(group = "testing")
public class PIDFMotorTest extends OpMode {

    private PIDFMotor motor;

    public static double kp = 1;
    public static double ki = 1;
    public static double kd = 1;
    public static double kf = 1;
    public static double TICKS_PER_REV = 537.7;
    public static double GEAR_RATIO = 1; //output / input
    public static int targetPosition = 300;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motor = new PIDFMotor(hardwareMap, "motor");
        motor.initialize(kp, ki, kd, kf, TICKS_PER_REV, GEAR_RATIO);
    }

    @Override
    public void loop() {

        motor.setPosition(targetPosition);
        motor.update();

    }
}