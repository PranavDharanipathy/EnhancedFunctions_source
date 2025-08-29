package org.firstinspires.ftc.teamcode.Testing;

import org.firstinspires.ftc.teamcode.EnhancedFunctions.UnboundedMotor;
import org.firstinspires.ftc.teamcode.EnhancedFunctions.UnboundedMotor.ScheduleAtFixedRate;
import org.firstinspires.ftc.teamcode.EnhancedFunctions.UnboundedMotor.ScheduleWithFixedDelay;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(group = "testing")
public class UnboundedMotorTest extends OpMode {

    private UnboundedMotor motor;

    @Override
    public void init() {
        motor = new UnboundedMotor(hardwareMap, "motor");
        motor.setMaxCurrent(5500);
        motor.setMode(UnboundedMotor.RunMode.STOP_AND_RESET_ENCODER); //automatically makes it run using encoder
    }

    @Override
    public void start() {
        motor.start(new ScheduleWithFixedDelay(20));
    }

    @Override
    public void loop() {
        motor.setPosition(1000);
        motor.setPower(1);
        //automatically runs to position
    }

    @Override
    public void stop() {
        motor.kill(); //motor and its runner closed
    }
}