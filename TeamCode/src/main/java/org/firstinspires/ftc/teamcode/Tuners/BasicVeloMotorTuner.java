package org.firstinspires.ftc.teamcode.Tuners;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.EnhancedFunctions.BasicVeloMotor;

@Config
@TeleOp (group = "tuning")
public class BasicVeloMotorTuner extends OpMode {

    private BasicVeloMotor motor;

    public static double VELOCITY = 500;

    public static double KP = 0;
    public static double KD = 0;
    public static double KF = 0;

    @Override
    public void init() {

        motor = new BasicVeloMotor(hardwareMap, "motor");
    }

    @Override
    public void loop() {

        motor.setVelocityPDFCoefficients(KP, KD, KF); //coeffs are updated

        motor.setVelocity(VELOCITY);

        telemetry.addData("velocity", motor.getVelocity());
        telemetry.update();
    }
}
