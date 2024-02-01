package frc.robot.hardware.vendors.firstparties;

import lombok.extern.java.Log;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.Consumer;

/**
 * Provides a consistent time between cycles.
 * The cycleEnded() method must be called at the end of each robot cycle so we know when to update to the next time.
 */
@Log
public class Clock {

    private double startTime = 0.0;
    private double currentTime = 0.0;
    private double deltaT = 0.0;
    private boolean isSimulated = false;

    public Clock() {
    }

    public void update() {
        double t = (isSimulated) ? getJavaTime() : getRobotTime() - startTime;
        deltaT = t - currentTime;
        currentTime = t;
    }

    /**
     * @return A cycle-consistent time, in seconds.
     */
    public double now() {
        return currentTime;
    }

    /**
     * @return A cycle-consistent delta-time, in seconds.
     */
    public double deltaTime() {
        return deltaT;
    }

    /**
     * @return A cycle-consistent time, in milliseconds.
     */
    public double getCurrentTimeInMillis() {
        return now() * 1e3;
    }

    /**
     * @return A cycle-consistent time, in microseconds.
     */
    public double getCurrentTimeInMicros() {
        return now() * 1e6;
    }

    /**
     * @return A cycle-consistent time, in nanoseconds.
     */
    public double getCurrentTimeInNanos() {
        return now() * 1e9;
    }

    public void setTime(double time) {
        if (isSimulated) {
            startTime = 0.0;
            currentTime = time;
        } else {
            log.severe("Setting the current time is not allowed outside of simulation.");
        }
    }

    public Clock simulated() {
        startTime = getJavaTime();
        isSimulated = true;
        return this;
    }

    private static double getJavaTime() {
        return System.currentTimeMillis() / 1000.0;
    }

    private static double getRobotTime() {
        return Timer.getFPGATimestamp();
    }

    public void report(String name, Consumer<Void> toDo) {
        double start = Timer.getFPGATimestamp();
        toDo.accept(null);
        double end = Timer.getFPGATimestamp();
        SmartDashboard.putNumber(name, end - start);
    }
}
