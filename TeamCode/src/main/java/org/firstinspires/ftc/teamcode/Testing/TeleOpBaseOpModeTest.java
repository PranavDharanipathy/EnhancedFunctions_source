package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.EnhancedFunctions.TeleOpBaseOpMode;

@TeleOp(group = "testing")
public class TeleOpBaseOpModeTest extends TeleOpBaseOpMode {

    private DcMotorEx motorEx;
    private DcMotor motor;
    private Servo servo;
    private CRServo crservo;

    @Override
    public void runOpMode() throws InterruptedException {

        //used to initialize without inputting a HardwareDevice
        motorEx = generalMap.get("motorEx");
        motor = generalMap.get("motor");
        servo = generalMap.get("servo");
        crservo = generalMap.get("crservo");

        if (isStopRequested()) return;
        waitForStart();

        while (opModeIsActive()) {}

    }
}
