package org.firstinspires.ftc.teamcode.EnhancedFunctions;

import static android.os.SystemClock.sleep;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.ArrayList;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.RejectedExecutionException;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;


@SuppressLint("DiscouragedApi")
@SuppressWarnings({"unused", "cast"})
public class UnboundedMotor {


    public class MotorAlreadyStartedException extends RejectedExecutionException {

        public MotorAlreadyStartedException(String message) {
            super(message);
        }
    }

    private boolean isStarted = false;

    private String deviceName;

    private static Integer SCHEDULE_RATE;
    private static String SCHEDULE_TYPE = "UNKNOWN";
    private static int SCHEDULE_BIAS;
    private ArrayList<Double> timeBetweenTicks = new ArrayList<>();
    private ArrayList<Double> timeWaited = new ArrayList<>();

    private ExecutorService asyncStarter; //for AutomaticSmartScheduling
    private volatile boolean process = true;
    private Future<?> runner;

    private double MAX_MILLIAMPS;

    private double power = 0;

    private int targetPosition = 0;

    public enum Direction { FORWARD, REVERSE }

    public enum RunMode { RUN_WITHOUT_ENCODER, RUN_USING_ENCODER, STOP_AND_RESET_ENCODER }

    private RunMode mode = RunMode.RUN_WITHOUT_ENCODER;

    private Direction direction = Direction.FORWARD; //default direction is forward

    private DcMotorEx motor;

    private ScheduledExecutorService executor;

    public abstract static class ScheduleType {}

    public static class ScheduleAtFixedRate extends ScheduleType {

        /// NOT RECOMMENDED!
        /// @apiNote Discouraged
        public ScheduleAtFixedRate(Integer rate) {
            isIllegalArgumentGivenToScheduler(rate);
            SCHEDULE_TYPE = "FIXED_RATE";
            SCHEDULE_RATE = rate;
        }
    }

    public static class ScheduleWithFixedDelay extends ScheduleType {

        public ScheduleWithFixedDelay(Integer rate) {
            isIllegalArgumentGivenToScheduler(rate);
            SCHEDULE_TYPE = "FIXED_DELAY";
            SCHEDULE_RATE = rate;
        }
    }

    public static class AutomaticSmartScheduling extends ScheduleType {

        public AutomaticSmartScheduling(int positiveBias) {
            isIllegalArgumentGivenToScheduler(positiveBias);
            SCHEDULE_TYPE = "AUTO";
            SCHEDULE_BIAS = positiveBias;
        }
    }

    /// @exception IllegalArgumentException scheduleRateArgument cannot be less than zero!
    public static void isIllegalArgumentGivenToScheduler(long scheduleRateArgument) {
        if (scheduleRateArgument < 1) throw new IllegalArgumentException("scheduleRateArgument cannot be less than zero!");
    }

    public UnboundedMotor(HardwareMap hardwareMap, String deviceName, double MAX_MILLIAMPS, int tolerance) {
        this.MAX_MILLIAMPS = MAX_MILLIAMPS; //max amps motor will take given

        this.deviceName = deviceName;
        motor = hardwareMap.get(DcMotorEx.class, deviceName);
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setTargetPositionTolerance(tolerance);

        executor = Executors.newSingleThreadScheduledExecutor();
        asyncStarter = Executors.newSingleThreadExecutor();
    }

    /** @see UnboundedMotor#UnboundedMotor(HardwareMap, String, double, int)
     * (double) MAX_MILLIAMPS and (int) tolerance are given defaults when the "new default" constructor is called
     * MAX_MILLIAMPS default: 5100
     * tolerance default: 2
    **/
    //new default constructor
    public UnboundedMotor(HardwareMap hardwareMap, String deviceName) {
        this(hardwareMap, deviceName, 5100, 2);
    }

    private Double getSumOfArrayListItems(ArrayList<Double> arrayList) {

        Double sum = 0.0;

        for (int i = 0; i < arrayList.size(); i++) {
            sum += arrayList.get(i);
        }

        return sum;
    }

    /// runs background actions concurrently
    public void start(ScheduleType rate/** all actions done in constructor renders calling methods redundant **/ ) {
        if (!isStarted) {
            if (SCHEDULE_TYPE.equals("FIXED_DELAY")) executor.scheduleWithFixedDelay(this::update,0, SCHEDULE_RATE, TimeUnit.MILLISECONDS);
            else if (SCHEDULE_TYPE.equals("FIXED_RATE")) executor.scheduleAtFixedRate(this::update, 0, SCHEDULE_RATE, TimeUnit.MILLISECONDS);
            else {
                runner = asyncStarter.submit(() -> {
                    try {
                        timeBetweenTicks.add(20.0); //~max time between ticks
                        Double prev_timeBetweenTick = 20.0;
                        Double curr_timeBetweenTick = 0.0;
                        double averageOfTimeInBetweenTicks;
                        double averageOfTimeWaited;
                        ElapsedTime timer = new ElapsedTime(); //timer for calculation time between ticks

                        while (process) {

                            update(); //running UnboundedMotor functions
                            averageOfTimeInBetweenTicks = (getSumOfArrayListItems(timeBetweenTicks) / timeBetweenTicks.size()); //calculating average
                            averageOfTimeWaited = getSumOfArrayListItems(timeWaited) / timeWaited.size(); //calculating average
                            timeWaited.add(0.0 + averageOfTimeInBetweenTicks); //adding data
                            sleep((long) (averageOfTimeInBetweenTicks + (averageOfTimeWaited / averageOfTimeInBetweenTicks))); /// waiting
                            sleep(SCHEDULE_BIAS); //waiting extra time

                            curr_timeBetweenTick = (timer.milliseconds() - SCHEDULE_BIAS); //calculating time between ticks in the latest instance
                            timeBetweenTicks.add(curr_timeBetweenTick); //adding data
                            timeBetweenTicks.set(0, 20.0 * (curr_timeBetweenTick / prev_timeBetweenTick)); //adjusting first value
                            prev_timeBetweenTick = curr_timeBetweenTick;
                            timer.reset(); //restarting timer
                        }
                    }
                    catch (Exception e) {
                        Thread.currentThread().interrupt();
                    }
                });
            }
        }
        else throw new MotorAlreadyStartedException("Motor has already been started and cannot be started again!");
    }

    ///method containing code that must be continuously run
    private void update() {

//        if (direction == Direction.FORWARD) motor.setDirection(DcMotor.Direction.FORWARD);
//        else motor.setDirection(DcMotor.Direction.REVERSE);

        if (getCurrent() > MAX_MILLIAMPS) power = 0; //if motor draws more amps than the maximum, motor is shut off (motor power set to 0)

        if (mode == RunMode.STOP_AND_RESET_ENCODER) {
            power = 0; //motor is stopped
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //motor is reset
            mode = RunMode.RUN_USING_ENCODER; // custom runmode is set back to RUN_USING_ENCODER
        }

        if (mode == RunMode.RUN_USING_ENCODER) {
            motor.setTargetPosition(targetPosition);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        motor.setPower(power); //motor power is set
    }

    private void endAsyncStarter() {
        process = false;
        if (runner != null) {
            runner.cancel(true);
        }
        asyncStarter.shutdown();
    }

    public void kill() { //called to close the motor and it's executor
        executor.shutdown();
        endAsyncStarter();
        motor.setPower(0);
    }

    public void setMaxCurrent(double MAX_MILLIAMPS) {
        this.MAX_MILLIAMPS = MAX_MILLIAMPS; //max amps drawable is set
    }

    public void setTargetPositionTolerance(int tolerance) {
        motor.setTargetPositionTolerance(tolerance);
    }

    public void setPower(double power) { if (getCurrent() <= MAX_MILLIAMPS) this.power = Math.min(power, 1); }

    public void setPosition(int position) {
        if (mode == RunMode.RUN_WITHOUT_ENCODER) throw new IllegalArgumentException("Cannot set target position when motor not using encoder!");
        targetPosition = position;
    }

    public void setMode(RunMode mode) { this.mode = mode; }

    public void setDirection(Direction direction) {
        this.direction = direction;
        //direction is set
        if (direction == Direction.FORWARD) motor.setDirection(DcMotor.Direction.FORWARD);
        else motor.setDirection(DcMotor.Direction.REVERSE);
    }

    public Direction getDirection() { return direction; }

    public RunMode getMode() { return mode; }

    public double getCurrent() { return motor.getCurrent(CurrentUnit.MILLIAMPS); }

    public int getTargetPositionTolerance() { return motor.getTargetPositionTolerance(); }

    public double getPower() { return motor.getPower(); }

    public double getCurrentPosition() { return motor.getCurrentPosition(); }

    public int getTargetPosition() { return targetPosition; }

}