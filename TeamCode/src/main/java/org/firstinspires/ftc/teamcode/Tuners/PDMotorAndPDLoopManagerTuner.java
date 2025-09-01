package org.firstinspires.ftc.teamcode.Tuners;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.EnhancedFunctions.PDMotor;

@TeleOp(group = "tuning")
public class PDMotorAndPDLoopManagerTuner extends OpMode {

    //FOR BOTH PDMotor and PDLoopManager TUNING!!!

    private PDMotor motor;

    //chose a target position
    public static int targetPositon = 300;

    //tune slowly by an increment of 0.01 - go lower if you need
    public static double kP = 1;
    public static double kD = 0;

    @Override
    public void init() {

        motor = new PDMotor(hardwareMap, "motor", 1, 0);
    }

    @Override
    public void loop() {

        // motor PD loop instance
        motor.tunePDCoefficients(targetPositon, kP, kD);
    }
}
