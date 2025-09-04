package org.firstinspires.ftc.teamcode.Tuners;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.EnhancedFunctions.PurePDMotor;

@Config
@TeleOp(group = "tuning")
public class PurePDMotorTuner extends OpMode {

    //FOR BOTH PDMotor and PDLoopManager TUNING!!!

    private PurePDMotor motor;

    //chose a target position
    public static int targetPositon = 300;

    //tune slowly by an increment of 0.01 - go lower if you need
    public static double kP = 1;
    public static double kD = 0;

    @Override
    public void init() {

        motor = new PurePDMotor(hardwareMap, "motor", 1, 0);
    }

    @Override
    public void loop() {

        // motor PD loop instance
        motor.tunePDCoefficients(targetPositon, kP, kD);
    }
}
