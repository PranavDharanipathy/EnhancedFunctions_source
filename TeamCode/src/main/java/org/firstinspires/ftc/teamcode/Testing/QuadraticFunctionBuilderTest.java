package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.EnhancedFunctions.QuadraticFunctionBuilder.getQuadraticInformationFromData;

@TeleOp(group = "testing")
public class QuadraticFunctionBuilderTest extends OpMode {

    private final double[] X_DATA = {1, 2, 3};
    private final double[] Y_DATA = {1, 2, 4};

    public void init() {}

    @Override
    public void loop() {
        telemetry.addData("", getQuadraticInformationFromData(X_DATA, Y_DATA));
        telemetry.update();
    }
}
