package org.firstinspires.ftc.teamcode.EnhancedFunctions;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.EnhancedFunctions.Maps.GeneralMap;
import org.firstinspires.ftc.teamcode.EnhancedFunctions.Maps.MotorMap;
import org.firstinspires.ftc.teamcode.EnhancedFunctions.Maps.ServoMap;

public abstract class TeleOpBaseOpMode extends LinearOpMode {

    public TeleOpBaseOpMode() {}

    public volatile GeneralMap generalMap = new GeneralMap(hardwareMap);
    public volatile MotorMap motorMap = new MotorMap(hardwareMap);
    public volatile ServoMap servoMap = new ServoMap(hardwareMap);

}
