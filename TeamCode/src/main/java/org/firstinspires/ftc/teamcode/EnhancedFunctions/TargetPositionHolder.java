package org.firstinspires.ftc.teamcode.EnhancedFunctions;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

@Autonomous (name = "TargetPositionHolder")
public class TargetPositionHolder {

    private ExecutorService executor = Executors.newCachedThreadPool();
    private double power;
    private volatile long waitTime = 5;
    public void changeWaitTimeBetweenRuns(long waitTime) { this.waitTime = waitTime; }

    //this never changes even if your motor has a different encoder resolution (this is what the code has been created with)
    public static double TRAINED_TICKS_PER_REV = 537.7;

    //tick margins
    public static double LOWER_MAX_POWER_LMARGIN = 2500;
    public static double HALF_POWER_LMARGIN = 1200;
    public static double MOD_POWER_LMARGIN = 900;
    public static double LOW_POWER_LMARGIN = 500;
    public static double MIN_POWER_LMARGIN = 35;

    private volatile double TICKS_PER_REV = 537.7; //THIS is your default motors encoder resolution (motor encoder resolution can be changed for each motor)
    public volatile double gearRatio = 1; // GEAR_RATIO input (motor) speed / output (motor) speed
    public volatile double holdPower = 0; // power set once motor has reached its TargetPosition
    public volatile double marginOfError = 5 ;
    public volatile double powerMultiplier = 1;
    public volatile double tuningPowerMultiplier = 1; // made on top of power multiplier adjustments

    private volatile Map<DcMotor, Boolean> motorLoopFlags = new ConcurrentHashMap<>(); //holds are motor flags
    private volatile Map<DcMotor, Future<?>> motorTasks = new ConcurrentHashMap<>(); //holds all motor tasks

    public void holdDcMotor(@NonNull DcMotor motor, double holdPosition, @NonNull VoltageSensor batteryVoltageSensor, Object... varargs) {

        //custom stats taken
        double[] varargsData = processVarargs(varargs);
        gearRatio = varargsData[0];
        TICKS_PER_REV = varargsData[1];
        holdPower = varargsData[2];
        marginOfError = varargsData[3];
        powerMultiplier = varargsData[4];
        tuningPowerMultiplier = varargsData[5];

        //motor powers et leveraging ternary operators

        double MAX_POWER = Math.min(1 / gearRatio, 1);
        // ternary operator
        double LOWER_MAX_POWER = ((MAX_POWER / 1.3) * powerMultiplier) > 1 ? ((1 * powerMultiplier <= 1) ? 1 : (1 * powerMultiplier)) : ((MAX_POWER / 1.3) * powerMultiplier);
        double HALF_POWER = ((MAX_POWER / 5.5) * powerMultiplier) > 1 ? ((1 * powerMultiplier <= 1) ? 1 : (1 * powerMultiplier)) : ((MAX_POWER / 2) * powerMultiplier);
        double MOD_POW = ((MAX_POWER / 7.5) * powerMultiplier) > 1 ? ((1 * powerMultiplier <= 1) ? 1 : (1 * powerMultiplier)) : ((MAX_POWER / 3.5) * powerMultiplier);
        double LOW_POW = ((MAX_POWER / 9) * powerMultiplier) > 1 ? ((1 * powerMultiplier <= 1) ? 1 : (1 * powerMultiplier)) : ((MAX_POWER / 8) * powerMultiplier);
        double MIN_POW = ((MAX_POWER / 10) * powerMultiplier) > 1 ? ((1 * powerMultiplier <= 1) ? 1 : (1 * powerMultiplier)) : ((MAX_POWER / 9.5) * powerMultiplier);
        double LOW_BATTERY_VOLTAGE_LOW_AND_MIN_POW = ((MAX_POWER / 5) * powerMultiplier) > 1 ? 1 : ((MAX_POWER / 7.5) * powerMultiplier);

        // tuningPowerMultiplier is applied after powerMultiplier is applied
        double MOD_POWER = (MOD_POW * tuningPowerMultiplier) > 1 ? ((1 * powerMultiplier <= 1) ? 1 : (1 * powerMultiplier)) : (MOD_POW * tuningPowerMultiplier);
        double LOW_POWER = (LOW_POW * tuningPowerMultiplier) > 1 ? ((1 * powerMultiplier <= 1) ? 1 : (1 * powerMultiplier)) : (LOW_POW * tuningPowerMultiplier);
        double MIN_POWER = (MIN_POW * tuningPowerMultiplier) > 1 ? ((1 * powerMultiplier <= 1) ? 1 : (1 * powerMultiplier)) : (MIN_POW * tuningPowerMultiplier);
        double LOW_BATTERY_VOLTAGE_LOW_AND_MIN_POWER = (LOW_BATTERY_VOLTAGE_LOW_AND_MIN_POW * tuningPowerMultiplier) > 1 ? 1 : (LOW_BATTERY_VOLTAGE_LOW_AND_MIN_POW * tuningPowerMultiplier);


        motorLoopFlags.put(motor, true);

        Future<?> motorTask = executor.submit(() -> {

            motor.setTargetPosition((int) holdPosition);

            try {
                while (Boolean.TRUE.equals(motorLoopFlags.getOrDefault(motor, false)) && !Thread.currentThread().isInterrupted()) {

                    //motor power is set through fuzzy logic (tuned)

                    power = setAppropriatePower(MAX_POWER, LOWER_MAX_POWER, HALF_POWER, MOD_POWER, LOW_POWER, MIN_POWER, LOW_BATTERY_VOLTAGE_LOW_AND_MIN_POWER, batteryVoltageSensor, motor);

                    if (Math.floor(Math.abs(motor.getCurrentPosition())) > (Math.abs(motor.getTargetPosition()) - marginOfError) && Math.floor(Math.abs(motor.getCurrentPosition())) < (Math.abs(motor.getTargetPosition()) + marginOfError)) {
                        motor.setPower(holdPower);
                    } else {
                        motor.setPower(power);
                        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }

                    Thread.sleep(waitTime);
                }
            } catch (Exception e) {
                Thread.currentThread().interrupt();
            } finally { clearFutureOfDcMotor(motor); }

        });
        motorTasks.putIfAbsent(motor, motorTask); //motor task is added to motorTasks

    }


    private double[] processVarargs(Object... varargs) { //processes custom stats

        double ticksPerRev = TICKS_PER_REV;
        double gearRatio = this.gearRatio;
        double holdPower = this.holdPower;
        double marginOfError = this.marginOfError;
        double powerMultiplier = this.powerMultiplier;
        double tuningPowerMultiplier = this.tuningPowerMultiplier;

        for (int i = 0; i < varargs.length; i += 2) {
            if (varargs[i] instanceof String && varargs[i + 1] instanceof Number) {
                String stringKey = (String) varargs[i];
                double value = ((Number) varargs[i + 1]).doubleValue();

                switch (stringKey) {
                    case "GEAR_RATIO":
                        gearRatio = value;
                        break;
                    case "TICKS_PER_REV":
                        ticksPerRev = value;
                        break;
                    case "HOLD_POWER":
                        holdPower = value;
                        break;
                    case "ALLOWABLE_MARGIN_OF_ERROR":
                        marginOfError = value;
                        break;
                    case "POWER_MULTIPLIER":
                        powerMultiplier = value;
                        break;
                    case "TUNING_POWER_MULTIPLIER":
                        tuningPowerMultiplier = value;
                    default:
                        //does nothing for unknown keys.
                        break;
                }
            }
        }
        return new double[] {gearRatio, ticksPerRev, holdPower, marginOfError, powerMultiplier, tuningPowerMultiplier};
    }

    /// ends the task of a motor
    public void clearFutureOfDcMotor(DcMotor motor) {
        if (motorTasks.containsKey(motor)) {
            motorLoopFlags.remove(motor);
            motorTasks.remove(motor);
        }
    }

    public void killExecutor() { //closes the TargetPositionHolder object
        for (DcMotor motor : motorTasks.keySet()) stopHoldingDcMotor(motor);

        executor.shutdown();
    }

    public void stopHoldingDcMotor(DcMotor motor) {
        Future<?> motorTask = motorTasks.get(motor);
        motorLoopFlags.put(motor, false);
        if (motorTask != null) {
            motorTask.cancel(true);
        }
        motor.setPower(0.0);
    }

    /// used by 'holdDcMotor()' sets appropriate power given data (tuned)
    public double setAppropriatePower(double MAX_POWER,
                                      double LOWER_MAX_POWER,
                                      double HALF_POWER,
                                      double MOD_POWER,
                                      double LOW_POWER,
                                      double MIN_POWER,
                                      double LOW_BATTERY_VOLTAGE_LOW_AND_MIN_POWER,
                                      VoltageSensor batteryVoltageSensor,
                                      DcMotor motor) {
        double powerAmount;
        double positionDifference = Math.abs(motor.getTargetPosition() - motor.getCurrentPosition());
        double currentBatteryVoltage = batteryVoltageSensor.getVoltage();
        //evaluates motor powers
        if (positionDifference <= TICKS_PER_REV * (MIN_POWER_LMARGIN / TRAINED_TICKS_PER_REV)) {
            if (currentBatteryVoltage <= 11.75) {
                powerAmount = LOW_POWER;
            }
            else if (currentBatteryVoltage <= 12) {
                powerAmount = LOW_BATTERY_VOLTAGE_LOW_AND_MIN_POWER;
            }
            else powerAmount = MIN_POWER;
        } else if (positionDifference <= TICKS_PER_REV * (LOW_POWER_LMARGIN / TRAINED_TICKS_PER_REV)) {
            if (currentBatteryVoltage <= 11.75) {
                powerAmount = LOW_BATTERY_VOLTAGE_LOW_AND_MIN_POWER;
            }
            else powerAmount = LOW_POWER;
        } else if (positionDifference <= TICKS_PER_REV * (MOD_POWER_LMARGIN / TRAINED_TICKS_PER_REV)) {
            powerAmount = MOD_POWER;
        } else if (positionDifference <= TICKS_PER_REV * (HALF_POWER_LMARGIN / TRAINED_TICKS_PER_REV)) {
            powerAmount = HALF_POWER;
        } else if (positionDifference <= TICKS_PER_REV * (LOWER_MAX_POWER_LMARGIN / TRAINED_TICKS_PER_REV)) {
            powerAmount = LOWER_MAX_POWER;
        }
        else powerAmount = MAX_POWER;

        return powerAmount;
    }
}
