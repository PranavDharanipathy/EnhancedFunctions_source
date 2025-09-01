package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.EnhancedFunctions.TeleOpBaseOpMode;

@TeleOp(group = "testing")
public class TeleOpBaseOpModeTest extends TeleOpBaseOpMode {

    private DcMotorEx motorEx;
    private DcMotor motor;
    private Servo servo;

    @Override
    public void runOpMode() throws InterruptedException {

        motorEx = (DcMotorEx) generalMap.get(motorEx,"motorEx");
        motor = motorMap.get("motor");
        servo = servoMap.get("servo");

        if (isStopRequested()) return;
        waitForStart();

        while (opModeIsActive()) {}

    }
}
