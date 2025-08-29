package org.firstinspires.ftc.teamcode.EnhancedFunctions;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(group = "testing")
public class FeedforwardArm {

    public DcMotorEx motor; /// must have motor-encoder (wire) attached

    public FeedforwardArm(HardwareMap hardwareMap, String deviceName, double kf, int tolerance, double TICKS_PER_REV, double startPositionInTicks) {

        this.kf = kf;

        this.TICKS_PER_REV = TICKS_PER_REV;
        startPosition = startPositionInTicks;

        this.motor = hardwareMap.get(DcMotorEx.class, deviceName);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setTargetPositionTolerance(tolerance);
    }

    private double kf;
    private final double TICKS_PER_REV;
    private double startPosition;

    public double getAngle(double ticks) {

        // 0 degrees is parallel to the ground and 90 degrees is pointing straight up
        /** GIVEN FORMAT
         *  -90 degrees is horizontal
         *  0 degrees is pointing straight up
         *  90 degrees is horizontal (the other way)**/
        return (360 * ((ticks + startPosition) / TICKS_PER_REV));
    }

    public void setAngle(double angle) {

        motor.setTargetPosition((int) Math.round(((angle / 360) * TICKS_PER_REV) - startPosition));

        /// HOW TO GIVE ANGLE VS HOW CODE PROCESSES THEM:
        /* HOW TO GIVE       HOW THE CODE PROCESSES
        *     0                  0
        * -90 ∩ 90      =>   180 ∩ 90     [IN DEGREES]
        *
        * '∩' represents the arc that the arm follows in its movement.
        */
    }

    public void update() {

        /** REQUIRED FORMAT
         *  180 degrees must be horizontal
         *  90 degrees must be pointing straight up
         *  0 degrees must be horizontal (the other way)**/

        // converting -90-0-90 to 180-90-0
        double adjustedAngle = 90 - getAngle(motor.getCurrentPosition());
        double k = kf * Math.cos(adjustedAngle);

        motor.setPower(k);
    }

    public void stopAndResetEncoder() {

        startPosition = motor.getCurrentPosition();

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void kfUpdatingForTuning(double kf) {
        this.kf = kf;
    }

}