package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.EnhancedFunctions.ExtremePrecisionVeloMotor;

@Config
@TeleOp(group = "testing")
public class ExtremePrecisionVeloMotorTest extends OpMode {

    public static double VELOCITY = 500; //arbitrary number

    public static double VELOCITY_MARGIN_OF_ERROR = 5; //arbitrary number
    public static double STABILITY_MARGIN_OF_ERROR = 10; //arbitrary number

    private ExtremePrecisionVeloMotor motor;

    /* NO TUNER PROVIDED!
    *
    * MAKE YOUR OWN TUNER - look at the others and use that (printing a graph on FTC Dashboard is a good idea)
    *
    * TUNING STEPS IN ORDER:
    * Weigh/measure parts to provide setInternalParameters
    * Set all coefficients to 0 except for kPIDFUnitsPerVolt, set that to 1
    * Tune ks
    * Tune kv
    * Tune ka
    * Tune kf - try to keep it as low AS POSSIBLE, if it's even a bit too high, it can easily mess up your PIDFVAS system
    * Tune kp
    * Tune kd
    * Tune ki - try to keep it as low as possible
    * Set kPIDFUnitsPerVolt (kv, ka, and ks should not interfere when getting this)
    * Set everything except kPIDFUnitsPerVolt and ks to 0
    * Update ks
    * Add the other coefficients back
    * After tuning these, you may or may not want to change your kf - update your kPIDFUnitsPerVolt
    */

    @Override
    public void init() {

        motor = new ExtremePrecisionVeloMotor(hardwareMap, "motor");
        // arbitrary numbers
        motor.setVelocityPIDFVASCoefficients(30,1.2, 0.65, 0.5, 0.2, 0.029,0.0265,0.25);
        motor.setInternalParameters(8192, 400, 8,12, 312);
    }

    @Override
    public void start() {

        motor.setVelocity(VELOCITY);
    }

    @Override
    public void loop() {

        motor.update();

        telemetry.addData("Frontend Calculated Velocity", motor.getFrontendCalculatedVelocity());
        telemetry.addData("Target Acceleration", motor.getTargetAcceleration());
        telemetry.addData("Is motor at velocity and stable?", motor.isAtVelocityAndStable(VELOCITY_MARGIN_OF_ERROR, STABILITY_MARGIN_OF_ERROR));
        telemetry.update();
    }
}
