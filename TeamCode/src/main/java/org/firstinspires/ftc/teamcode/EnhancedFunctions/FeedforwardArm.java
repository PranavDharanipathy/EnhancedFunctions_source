package org.firstinspires.ftc.teamcode.EnhancedFunctions;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class FeedforwardArm {

    private DcMotorEx motor; /// must have motor-encoder (wire) attached

    public FeedforwardArm(HardwareMap hardwareMap, String deviceName, double kf, int tolerance) {

        this.kf = kf;

        this.motor = hardwareMap.get(DcMotorEx.class, deviceName);
        motor.setTargetPositionTolerance(tolerance);
    }

    private double kf;

    public void update() {


    }

}
