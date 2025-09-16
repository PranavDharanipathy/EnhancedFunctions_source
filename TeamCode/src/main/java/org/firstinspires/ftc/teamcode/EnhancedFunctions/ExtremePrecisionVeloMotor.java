package org.firstinspires.ftc.teamcode.EnhancedFunctions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/// USES EXTERNAL ENCODER - REV Through Bore Encoder is highly recommended.
/// <p>|<p>
/// PDFVAS measured by external encoder.
/// <p>P: Proportional
/// <p>D: Derivative
/// <p>F: Holding Feedforward
/// <p>V: Velocity Feedforward
/// <p>A: Acceleration Feedforward
/// <p>S: Static Friction
public final class ExtremePrecisionVeloMotor {

    public DcMotorEx internalMotor;

    private ElapsedTime timer = new ElapsedTime();

    public ExtremePrecisionVeloMotor(HardwareMap hardwareMap, String deviceName) {

        internalMotor = hardwareMap.get(DcMotorEx.class, deviceName);

        internalMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        internalMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setDirection(DcMotorEx.RunMode direction) {
        internalMotor.setMode(direction);
    }

    public double kp;
    public double kd;
    public double kf;
    public double kv;
    public double ka;
    public double ks;

    private double kPDFUnitsPerVolt;

    private double FN;

    private double SHAFT_RADIUS;

    private double lastTargetVelocity = 0; //starts at 0
    private double targetVelocity;
    private double currentVelocity;

    private double lastCurrentVelocity = 0;

    private double targetAcceleration;

    private double EXTERNAL_ENCODER_RESOLUTION;

    /// @param EXTERNAL_ENCODER_RESOLUTION Is the encoder ticks needed by the external encoder for it to complete a full 360 degree turn,
    ///     it may be listed on the website that it was bought from.
    /// @param massInGrams Is the amount of mass in grams that is connected to the motor.
    /// @param SHAFT_DIAMETER Is the diameter of shaft connecting to motor.
    public void setInternalParameters(double EXTERNAL_ENCODER_RESOLUTION, double massInGrams, double SHAFT_DIAMETER) {

        this.EXTERNAL_ENCODER_RESOLUTION = EXTERNAL_ENCODER_RESOLUTION;

        this.SHAFT_RADIUS = SHAFT_DIAMETER / 2;
        FN = /*gravity*/ 9.80665 * (/*converted mass in g to kg*/ massInGrams * 1000);
    }

    /// @param kp Proportional
    /// <p>
    /// not using integral
    /// <p>
    /// @param kd Derivative
    /// <p>
    /// @param kf Holding Feedforward
    /// <p>
    /// @param kv Velocity Feedforward
    /// <p>
    /// @param ka Acceleration Feedforward
    /// <p>
    /// @param ks Static Friction
    /// <p>
    /// @param kPDFUnitsPerVolt delta PDF / delta volts
    public void setVelocityPDFVASCoefficients(double kp, double kd, double kf, double kv, double ka, double ks, double kPDFUnitsPerVolt) {

        this.kp = kp;
        this.kd = kd;
        this.kf = kf;
        this.kv = kv;
        this.ka = ka;
        this.ks = ks;
        this.kPDFUnitsPerVolt = kPDFUnitsPerVolt;
    }

    private double prevTime = 0, prevError = 0;

    /// @param velocity in ticks per second
    public void setVelocity(double velocity) {
        lastTargetVelocity = targetVelocity;
        targetVelocity = velocity;
    }

    public enum METHOD_OF_VELOCITY_CALCULATION {
        FRONTEND, INTERNAL
    }

    public void update(METHOD_OF_VELOCITY_CALCULATION methodOfVelocityCalculation) {

        double p;
        double d;
        double f;
        double v;
        double a;
        double s;

        double currTime = timer.milliseconds();
        double dt = currTime - prevTime;

        double error;

        //frontend is better for this - isAtVelocityAndStable()
        lastCurrentVelocity = currentVelocity;

        //setting current velocity in ticks per second (dt is in milliseconds so 'currentVelocity' is converted)
        currentVelocity = 1000 * (internalMotor.getCurrentPosition() / dt);

        if (methodOfVelocityCalculation == METHOD_OF_VELOCITY_CALCULATION.FRONTEND) error = targetVelocity - currentVelocity;
        else error = targetVelocity - internalMotor.getVelocity();

        //proportional
        p = kp * error;

        //derivative
        d = kd * (error - prevError) / dt;

        //positional feedforward for holding
        f = kf /* cos(0 degrees) = 1 */;

        //velocity feedforward
        v = kv * targetVelocity;

        //acceleration feedforward
        targetAcceleration = (targetVelocity - lastTargetVelocity) / dt;
        a = ka * targetAcceleration;

        //static friction
        double I = internalMotor.getCurrent(CurrentUnit.AMPS);
        double kt = (FN * SHAFT_RADIUS) / I;
        double T = ks * FN * SHAFT_RADIUS;
        s = (T / kt) * kPDFUnitsPerVolt;

        double PDFVAPower = p + d + (usingHoldingFeedforward ? f : 0) + v + a;
        internalMotor.setPower(PDFVAPower + (PDFVAPower >= 0 ? s : -s));

        prevError = error;
        prevTime = currTime;
    }

    /// IN TICKS PER SECOND
    /// <p>
    ///Calculated by external encoder using ticks
    public double getFrontendCalculatedVelocity() {
        return currentVelocity;
    }

    /// IN TICKS PER SECOND
    /// <p>
    ///Calculated by external encoder and motor internally
    public double getInternallyCalculatedVelocity() {
        return internalMotor.getVelocity();
    }

    public double getTargetAcceleration() {
        return targetAcceleration;
    }

    /// CONDITIONS:
    /// <p>
    /// Is at velocity within a certain margin of error.
    /// <p>
    /// Is stable within a certain margin of error.
    /// @param velocityMarginOfError Acceptable variation in velocity.
    /// @param stabilityMarginOfError Acceptable variation in stability.
    public boolean isAtVelocityAndStable(METHOD_OF_VELOCITY_CALCULATION methodOfVelocityCalculation, double velocityMarginOfError, double stabilityMarginOfError) {

        boolean motorIsAtVelocityAndStable = false;

        double currentVelocity; //different for each type of calculation
        if (methodOfVelocityCalculation == METHOD_OF_VELOCITY_CALCULATION.FRONTEND) currentVelocity = Math.abs(this.currentVelocity);
        else currentVelocity = Math.abs(internalMotor.getVelocity());

        if (Math.abs(targetVelocity) - currentVelocity <= velocityMarginOfError && currentVelocity - Math.abs(lastCurrentVelocity) < stabilityMarginOfError) motorIsAtVelocityAndStable = true;

        return motorIsAtVelocityAndStable;
    }

    private boolean usingHoldingFeedforward = true;

    /// Enables/disables holding feedforward
    /// @param state true (using) or false (not using)
    public void setHoldingFeedforwardState(boolean state) {
        usingHoldingFeedforward = state;
    }

}