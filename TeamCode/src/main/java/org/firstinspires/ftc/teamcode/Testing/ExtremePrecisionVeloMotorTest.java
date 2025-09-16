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
    * Set all coefficients to 0 except for kPDFUnitsPerVolt, set that to 1
    * Tune kf - try to keep it as low AS POSSIBLE, if it's even a bit too high, it can easily mess up your PDFVAS system
    * Tune kp
    * Tune kd
    * Set kPDFUnitsPerVolt
    * Set kp, kd, and kf to 0 but save their values
    * Tune ks
    * Add back the kp, kd, and kf values
    * Tune kv
    * Tune ka
    * After tuning these, you may or may not want to change your kf, if you're going to change it, make sure you update kPDFUnitsPerVolt and ks as well
    */

    @Override
    public void init() {

        motor = new ExtremePrecisionVeloMotor(hardwareMap, "motor");
        //arbitrary numbers
        motor.setVelocityPDFVASCoefficients(4,1.2,0.11,0.043, 0.029,0.0265,0.25);
        motor.setInternalParameters(8440, 400, 8);
    }

    @Override
    public void start() {

        motor.setVelocity(VELOCITY);
    }

    @Override
    public void loop() {

        motor.update(ExtremePrecisionVeloMotor.METHOD_OF_VELOCITY_CALCULATION.INTERNAL);

        telemetry.addData("Frontend Calculated Velocity", motor.getFrontendCalculatedVelocity());
        telemetry.addData("Internally Calculated Velocity", motor.getInternallyCalculatedVelocity());
        telemetry.addData("Target Acceleration", motor.getTargetAcceleration());
        telemetry.addData("Is motor at velocity and stable?", motor.isAtVelocityAndStable(ExtremePrecisionVeloMotor.METHOD_OF_VELOCITY_CALCULATION.INTERNAL, VELOCITY_MARGIN_OF_ERROR, STABILITY_MARGIN_OF_ERROR));
        telemetry.update();
    }
}
