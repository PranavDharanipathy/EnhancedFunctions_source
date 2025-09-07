package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.EnhancedFunctions.DataSaver;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

@TeleOp (group = "testing")
public class DataSaverTest extends OpMode {

    private static final Logger log = LoggerFactory.getLogger(DataSaverTest.class);
    private DataSaver saver;

    private int saved_i;

    @Override
    public void init() {

        saver = new DataSaver("TEST", hardwareMap.appContext);

        saved_i = (int) saver.loadData("test_i", 0.0f);

        log.info("Saved_i updated: {}", saved_i);
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