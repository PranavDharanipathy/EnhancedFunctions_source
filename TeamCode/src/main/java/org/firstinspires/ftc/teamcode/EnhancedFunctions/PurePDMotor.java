package org.firstinspires.ftc.teamcode.EnhancedFunctions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PurePDMotor {

    private DcMotor motor;

    private double kP;
    private double kD;

    private ElapsedTime timer = new ElapsedTime();

    private double prevTime = 0, prevError = 0;
    private double p = 0, d = 0;
    private double power;

    /// @param kP - Proportional coefficient
    /// @param kD - Derivative coefficient
    public PurePDMotor(HardwareMap hardwareMap, String deviceName, double kP, double kD) {

        motor = hardwareMap.get(DcMotor.class, deviceName);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.kP = kP;
        this.kD = kD;
    }

    public void setDirection(DcMotor.Direction direction) {
        motor.setDirection(direction);
    }

    /// motor PD loop instance
    public void update(int targetPosition) {

        double currTime = timer.milliseconds();
        double error = targetPosition - motor.getCurrentPosition();

        //proportional
        p = kP * error;

        //derivative
        d = kD * (error - prevError) / (currTime - prevTime);

        power = p + d;
        motor.setPower(power);

        prevError = error;
        prevTime = currTime;
    }

    /// motor PD loop instance for tuning
    public void tunePDCoefficients(int targetPosition, double kP, double kD) {

        double currTime = timer.milliseconds();
        double error = targetPosition - motor.getCurrentPosition();

        //proportional
        p = kP * error;

        //derivative
        d = kD * (error - prevError) / (currTime - prevTime);

        power = p + d;
        motor.setPower(power);

        prevError = error;
        prevTime = currTime;
    }

}
