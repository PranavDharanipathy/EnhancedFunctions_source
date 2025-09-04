package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.EnhancedFunctions.PurePDMotor;

@TeleOp(group = "testing")
public class PurePDMotorTest extends OpMode {

    private PurePDMotor motor;

    private int targetPositon;

    @Override
    public void init() {
        motor = new PurePDMotor(hardwareMap, "motor", 1, 0);
    }

    @Override
    public void loop() {

        // setting target position
        if (gamepad1.a) {
            targetPositon = 500;
        }
        else if (gamepad1.b) {
            targetPositon = -500;
        }
        else {
            targetPositon = 0;
        }

        // motor PD loop instance
        motor.update(targetPositon);
    }
}
