package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.EnhancedFunctions.PDLoopManager;
import org.firstinspires.ftc.teamcode.EnhancedFunctions.TeleOpBaseOpMode;

@TeleOp(group = "testing")
public class PDLoopManagerTest extends TeleOpBaseOpMode {

    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;
    private DcMotor motor4;

    private PDLoopManager pdLoopManager;

    @Override
    public void runOpMode() throws InterruptedException {

        motor1 = generalMap.get("motor1");
        motor2 = generalMap.get("motor2");
        motor3 = generalMap.get("motor3");
        motor4 = generalMap.get("motor4");

        motor1.setTargetPosition(300);
        motor2.setTargetPosition(300);
        motor3.setTargetPosition(300);
        motor4.setTargetPosition(300);

        pdLoopManager = new PDLoopManager();
        pdLoopManager.changeWaitTimeBetweenRuns(8);

        if (isStopRequested()) return;
        waitForStart();

        pdLoopManager.addPDLoop(motor1, 1, 0);
        pdLoopManager.addPDLoop(motor2, 1, 0);
        pdLoopManager.addPDLoop(motor3, 1, 0);
        pdLoopManager.addPDLoop(motor4, 1, 0);

        while (opModeIsActive()) {
            if (gamepad1.a) {
                pdLoopManager.addPDLoop(motor1, 1, 0);
                pdLoopManager.addPDLoop(motor2, 1, 0);
                pdLoopManager.addPDLoop(motor3, 1, 0);
                pdLoopManager.addPDLoop(motor4, 1, 0);
            }
            else if (gamepad1.b) {
                pdLoopManager.removePDLoop(motor1);
                pdLoopManager.removePDLoop(motor2);
                pdLoopManager.removePDLoop(motor3);
                pdLoopManager.removePDLoop(motor4);
            }
        }

        // Manager executor closed
        pdLoopManager.killManager();

    }
}
