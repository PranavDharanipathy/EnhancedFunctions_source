package org.firstinspires.ftc.teamcode.EnhancedFunctions.Maps;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class GeneralMap {

    private HardwareMap hardwareMap;

    public GeneralMap(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public HardwareDevice get(HardwareDevice hardwareDevice, String deviceName) {
        return hardwareMap.get(hardwareDevice.getClass(), deviceName);
    }
}
