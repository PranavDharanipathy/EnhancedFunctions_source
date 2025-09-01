package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.EnhancedFunctions.TickrateChecker;

@TeleOp(group = "testing")
public class TickrateCheckerTest extends OpMode {

    private DcMotor motor;

    @Override
    public void init() {

        motor = hardwareMap.get(DcMotor.class, "motor");
    }

    @Override
    public void loop() {

        TickrateChecker.startOfLoop();

        if (gamepad1.a) {
            motor.setPower(1);
        }
        else if (gamepad1.b) {
            motor.setPower(-1);
        }
        else {
            motor.setPower(0);
        }

        // .getTimePerTick can run in front of .endOfLoop
        //NOTHING MUST EVER RUN IN FRONT OF .startOfLoop AND NOTHING MUST EVER RUN BEHIND .endOfLoop
        telemetry.addData("Time per tick", TickrateChecker.getTimePerTick());
        telemetry.update();

        TickrateChecker.endOfLoop();
    }
}
