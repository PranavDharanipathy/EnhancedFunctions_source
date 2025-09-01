package org.firstinspires.ftc.teamcode.EnhancedFunctions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

public class PDLoopManager {

    private ExecutorService executor = Executors.newCachedThreadPool();
    private volatile Map<DcMotor, Boolean> motorLoopFlags = new ConcurrentHashMap<>(); //holds are motor flags
    private volatile Map<DcMotor, Future<?>> pdLoops = new ConcurrentHashMap<>(); //holds all motor tasks
    private volatile long waitTime = 5;
    public void changeWaitTimeBetweenRuns(long waitTime) { this.waitTime = waitTime; }

    /**
     * Puts a motor into a concurrently running PD loop
     * @param kP - proportional
     * @param kD - derivative
    **/
    public void addPDLoop(DcMotor motor, double kP, double kD) {

        motorLoopFlags.put(motor, true);

        Future<?> pdLoop = executor.submit(() -> {

            try {

                ElapsedTime timer = new ElapsedTime();

                double prevTime = 0, prevError = 0;
                double p, d;

                while (Boolean.TRUE.equals(motorLoopFlags.getOrDefault(motor, false)) && !Thread.currentThread().isInterrupted()) {

                    double currTime = timer.milliseconds();
                    double error = motor.getTargetPosition() - motor.getCurrentPosition();

                    //proportional
                    p = kP * error;

                    //derivative
                    d = kD * (error - prevError) / (currTime - prevTime);

                    motor.setPower(p + d);

                    prevError = error;
                    prevTime = currTime;

                    Thread.sleep(waitTime);
                }
            }
            catch (Exception e) {
                Thread.currentThread().interrupt();
            }
            finally {
                clearFutureOfDcMotor(motor);
            }
        });

        pdLoops.putIfAbsent(motor, pdLoop);

    }

    /// ends the task of a motor
    public void clearFutureOfDcMotor(DcMotor motor) {
        if (pdLoops.containsKey(motor)) {
            motorLoopFlags.remove(motor);
            pdLoops.remove(motor);
        }
    }

    /// to close a motor's PD loop
    public void removePDLoop(DcMotor motor) {

        motorLoopFlags.put(motor, false);

        Future<?> motorTask = pdLoops.get(motor);
        if (motorTask != null) {
            motorTask.cancel(true);
        }
        motor.setPower(0.0);
    }

    /// closes the executor
    public void killManager() {

        for (DcMotor motor : pdLoops.keySet()) removePDLoop(motor);
        executor.shutdownNow();
    }

}
