package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.EnhancedFunctions.OurColorSensor;

@TeleOp (group = "testing")
public class OurColorSensorTest extends OpMode {

    private OurColorSensor ourColorSensor = new OurColorSensor();

    @Override
    public void init() {
        ourColorSensor.initialize(hardwareMap);
    }

    @Override
    public void loop() {

        telemetry.addData("Color", ourColorSensor.getHue());

    }
}
