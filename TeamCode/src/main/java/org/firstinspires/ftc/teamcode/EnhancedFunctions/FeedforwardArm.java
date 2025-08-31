package org.firstinspires.ftc.teamcode.EnhancedFunctions;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp(group = "testing")
public class FeedforwardArm {

    /// MUST HAVE MOTOR-ENCODER (WIRE) ATTACHED!
    /// NEVER CALL .setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER) DIRECTLY ON THIS OBJECT!!
    public DcMotorEx motor;

    private VoltageSensor batteryVoltageSensor;

    public FeedforwardArm(HardwareMap hardwareMap, String deviceName, double kf, int tolerance, double TICKS_PER_REV, double startPositionInTicks) {

        this.kf = kf;

        this.TICKS_PER_REV = TICKS_PER_REV;
        startPosition = startPositionInTicks;

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        this.motor = hardwareMap.get(DcMotorEx.class, deviceName);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setTargetPositionTolerance(tolerance);
    }

    private double kf;
    private final double TICKS_PER_REV;
    private double startPosition;

    //defaults set
    private double[] VOLTAGE_DATA = {13, 13};
    private double[] KF_DATA = {13, 13};

    /**
     * @param VOLTAGE_DATA - voltage data when kf was collected
     * @param KF_DATA - kf values
     **/
    public void changeVoltageData(double[] VOLTAGE_DATA, double[] KF_DATA) {

        this.VOLTAGE_DATA = VOLTAGE_DATA;
        this.KF_DATA = KF_DATA;
    }

    private boolean TUNING = false;

    /// activated tuning mode
    /// @
    public void tuningMode() {
        TUNING = true;
    }

    private boolean DISABLE = false; //feedforward power being used by default

    /** GIVEN FORMAT
     * -90 degrees is horizontal
     * 0 degrees is pointing straight up
     * 90 degrees is horizontal (the other way)
     **/
    public double getAngle(double ticks) {
        return (360 * ((ticks + startPosition) / TICKS_PER_REV));
    }

    /** HOW TO GIVE ANGLE VS HOW CODE PROCESSES THEM:
     HOW TO GIVE       HOW THE CODE PROCESSES
     *     0                  0
     * -90 ∩ 90      =>   180 ∩ 90     [IN DEGREES]
     *
     * '∩' represents the arc that the arm follows in its movement.
     **/
    public void setAngle(double angle) {
        motor.setTargetPosition((int) Math.round(((angle / 360) * TICKS_PER_REV) - startPosition));
    }

    ///method containing code that must be continuously run
    public void update() {

        /* REQUIRED FORMAT
         *  180 degrees must be horizontal
         *  90 degrees must be pointing straight up
         *  0 degrees must be horizontal (the other way)*/

        // converting -90-0-90 to 180-90-0
        double adjustedAngle = 90 - getAngle(motor.getCurrentPosition());
        double k;

        if (TUNING) {
            k = kf * Math.cos(adjustedAngle);
        }
        else {
            k = (getVoltageQuadraticOutput(batteryVoltageSensor.getVoltage()) / batteryVoltageSensor.getVoltage()) * kf * Math.cos(adjustedAngle);
        }

        if (!DISABLE) motor.setPower(k);
    }

    /// always call this method instead of calling .motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); directly
    public void stopAndResetEncoder() {

        startPosition = motor.getCurrentPosition();

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setPower(double motorPower) {
        motor.setPower(motorPower);
    }

    public void setPositionWithCustomPower(int targetPosition, double motorPower) {

        enableCustomPower();
        motor.setTargetPosition(targetPosition);
        motor.setPower(motorPower);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setTargetPosition(int targetPosition) {

        enableCustomPower();
        motor.setTargetPosition(targetPosition);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void enableCustomPower() {
        DISABLE = true;
    }

    public void disableCustomPower() { //switches back to feedforward
        DISABLE = false;
    }

    public void kfUpdatingForTuning(double kf) { //used in the tuner
        this.kf = kf;
    }

    /** on base graph:
     * x represents voltage
     * y represents kf
     * on solution:
     * x represents voltage
     * y represents power multiplier
    **/
    private double getVoltageQuadraticOutput(double voltage) {

        double[] quadInfo = QuadraticFunctionBuilder.getQuadraticInformationFromData(VOLTAGE_DATA, KF_DATA);

        return quadInfo[0] * StrictMath.pow(voltage,2) + quadInfo[1] * voltage + quadInfo[2];
    }

}