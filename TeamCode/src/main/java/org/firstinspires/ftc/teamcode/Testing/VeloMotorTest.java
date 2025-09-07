package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.EnhancedFunctions.VeloMotor;

@TeleOp (group = "testing")
public class VeloMotorTest extends OpMode {

    private VeloMotor motor;

    @Override
    public void init() {

        motor = new VeloMotor(hardwareMap, "motor");

        motor.setVelocityPDFCoefficients(3, 1, 1); //completely arbitrary coeffs - just an example
    }

    @Override
    public void loop() {

        motor.setVelocity(500);

        telemetry.addData("velocity", motor.getVelocity());
        telemetry.update();
    }
}
