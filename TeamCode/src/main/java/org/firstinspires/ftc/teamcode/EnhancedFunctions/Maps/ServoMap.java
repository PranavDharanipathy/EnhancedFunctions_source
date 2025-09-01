package org.firstinspires.ftc.teamcode.EnhancedFunctions.Maps;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ServoMap {

    private HardwareMap hardwareMap;

    public ServoMap(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public Servo get(String deviceName) {
        return hardwareMap.get(Servo.class, deviceName);
    }
}
