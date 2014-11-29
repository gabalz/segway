package control.observe;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import control.RobotController;
import run.Robot;
import run.ThreadLogic;

/**
 * An observation thread which estimates the segway state
 * (pitch, roll, yaw and their velocities) excluding the position.
 */
public abstract class Observation extends ThreadLogic
{
    public Observation(RobotController controller, int delay,
                       int gyroSign, double gyroOffset)
    {
        this.controller = controller;
        this.delay = delay;
        this.gyroSign = gyroSign;
        this.gyroOffset = gyroOffset;
        log = null;
    }
    
    public final RobotController controller() { return controller; }
    public final Robot robot() { return controller.robot(); }
    
    public final int delay() { return delay; }
    public final int gyroSign() { return gyroSign; }
    
    public final double gyroOffset() { return gyroOffset; }
    public void setGyroOffset(double gyroOffset) { this.gyroOffset = gyroOffset; }
    
    //--------------------------------------------------------------------------
    
    /** @return time (ms) */
    public abstract long time();
    
    /** @return body pitch angle (rad) */
    public abstract double pitch();
    
    /** @return axle midpoint rotation (rad) */
    public abstract double roll();
    
    /** @return body yaw angle (rad) */
    public abstract double yaw();
    
    /** @return body pitch velocity (rad/sec) */
    public abstract double dPitch();
    
    /** @return axle midpoint velocity (rad/sec) */
    public abstract double dRoll();
    
    /** @return body yaw velocity (rad/sec) */
    public abstract double dYaw();
    
    //--------------------------------------------------------------------------
    
    public String logDescription()
    { return "time pitch roll yaw dPitch dRoll dYaw"; }
    
    public synchronized String logString()
    {
        return "" + time()
            + " " + pitch() + " " + roll() + " " + yaw()
            + " " + dPitch() + " " + dRoll() + " " + dYaw();
    }
    
    public void openLog(File file) throws IOException
    {
        log = new BufferedWriter(new FileWriter(file));
        log.write("# " + logDescription() + "\n");
    }
    
    public void log() throws IOException
    {
        if (log == null) return;
        log.write(logString() + "\n");
    }
    
    public void closeLog() throws IOException
    {
        if (log == null) return;
        log.flush();
        log.close();
        log = null;
    }
    
    //--------------------------------------------------------------------------
    
    private final int delay;
    private final RobotController controller;
    
    private final int gyroSign;
    private double gyroOffset;
    
    private BufferedWriter log;
}
