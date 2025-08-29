package org.firstinspires.ftc.teamcode.Testing;

import static android.os.SystemClock.sleep;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.EnhancedFunctions.TargetPositionHolder;

public class TargetPositionHolderTest extends OpMode {

    private DcMotor exampleMotor1;
    private DcMotor exampleMotor2;

    private VoltageSensor batteryVoltageSensor;

    private TargetPositionHolder robot = new TargetPositionHolder();

    @Override
    public void init() {
        exampleMotor1 = hardwareMap.get(DcMotor.class, "motor1");
        exampleMotor1.setZeroPowerBehavior(BRAKE);

        exampleMotor2 = hardwareMap.get(DcMotor.class, "motor2");
        exampleMotor2.setZeroPowerBehavior(BRAKE);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
    }


    @Override
    public void start() {
        robot.holdDcMotor(exampleMotor1, 1000, batteryVoltageSensor);
        robot.holdDcMotor(exampleMotor2, 1000, batteryVoltageSensor, "TICKS_PER_REV", 1044);
    }

    @Override
    public void loop() {

        telemetry.addLine("Motors are holding");
        telemetry.update();
    }

    @Override
    public void stop() {

        //all motors are stopped when '.killExecutor()' is called
        robot.killExecutor();
    }

}
