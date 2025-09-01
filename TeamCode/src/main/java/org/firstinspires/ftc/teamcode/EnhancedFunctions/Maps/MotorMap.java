package org.firstinspires.ftc.teamcode.EnhancedFunctions.Maps;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MotorMap {

    private HardwareMap hardwareMap;

    public MotorMap(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public DcMotor get(String deviceName) {
        return hardwareMap.get(DcMotor.class, deviceName);
    }
}
