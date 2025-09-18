package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.EnhancedFunctions.SimpleDataFile_XML;

@TeleOp (group = "testing")
public class SimpleDataFile_XMLTest extends OpMode {

    private SimpleDataFile_XML saver;

    private int saved_i;

    @Override
    public void init() {

        saver = new SimpleDataFile_XML("TEST", hardwareMap.appContext);

        saved_i = (int) saver.loadData("test_i", 0.0f);
    }

    @Override
    public void start() {

        saved_i++;
        saver.saveData("", saved_i);

        telemetry.addData("saved_i", saved_i);
        telemetry.update();
    }

    @Override
    public void loop() {

        if (gamepad1.a) {
            saver.deleteDataFile("TEST");
            requestOpModeStop();
        }

    }
}