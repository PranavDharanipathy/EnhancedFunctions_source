package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.EnhancedFunctions.OurColorSensorV2;

@TeleOp (group = "testing")
public class OurColorSensorV2Test extends OpMode {

    private OurColorSensorV2 ourColorSensorV2 = new OurColorSensorV2();

    @Override
    public void init() {
        ourColorSensorV2.initialize(hardwareMap, 3);
    }

    @Override
    public void loop() {

        telemetry.addData("Normalized Color", ourColorSensorV2.getNormalizedHue());
        telemetry.update();

    }
}
