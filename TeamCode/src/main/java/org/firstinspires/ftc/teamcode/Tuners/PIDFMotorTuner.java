package org.firstinspires.ftc.teamcode.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.EnhancedFunctions.PIDFMotor;

@Config
@TeleOp(group = "tuning")
public class PIDFMotorTuner extends OpMode {

    private PIDFMotor motor;

    public static double kp, ki, kd, kf, TICKS_PER_REV, GEAR_RATIO;
    public static int targetPosition;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motor = new PIDFMotor(hardwareMap, "motor");
        motor.initialize(kp, ki, kd, kf, TICKS_PER_REV, GEAR_RATIO);
    }

    @Override
    public void loop() {

        motor.setPosition(targetPosition);
        motor.updateCoefficients(kp, ki, kd, kf);
        motor.update();

        telemetry.addData("target position", motor.internalMotor.getTargetPosition());
        telemetry.addData("current position", motor.internalMotor.getCurrentPosition());
        telemetry.update();

    }
}