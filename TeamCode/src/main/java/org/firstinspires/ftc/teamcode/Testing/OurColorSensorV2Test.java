package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.EnhancedFunctions.OurColorSensorV2;

@TeleOp (group = "testing")
public class OurColorSensorV2Test extends OpMode {

    private OurColorSensorV2 ourColorSensor = new OurColorSensorV2();

    @Override
    public void init() {
        ourColorSensor.initialize(hardwareMap, 3);
    }

    @Override
    public void loop() {

        telemetry.addData("Raw Color", ourColorSensor.getRawHue());
        telemetry.addData("Normalized Color", ourColorSensor.getNormalizedHue());
        telemetry.update();

    }
}
