package org.firstinspires.ftc.teamcode.EnhancedFunctions;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class TeleOpBaseOpMode extends LinearOpMode {

    public TeleOpBaseOpMode() {}

    public volatile GeneralMap generalMap;

    public void initializeDevices() {
        generalMap = new GeneralMap(hardwareMap);
    }

}
