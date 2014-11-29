package control.observe;

import java.io.IOException;

import control.RobotController;

/**
 * Observation of the true simulator state (simulation only).
 */
public final class PerfectObservation extends Observation
{
    public PerfectObservation(RobotController controller, int freq,
                              int gyroSign, double gyroOffset)
    {
        super (controller, freq, gyroSign, gyroOffset);
        
        if (!controller.robot().isSimulated())
            throw new IllegalAccessError(
                  "The perfect observation is only available in simulation!");
    }
    
    //--------------------------------------------------------------------------
    
    @Override
    public synchronized long time() { return t; }
    
    @Override
    public synchronized double pitch() { return pitch; }
    
    @Override
    public synchronized double roll() { return roll; }
    
    @Override
    public synchronized double yaw() { return yaw; }
    
    @Override
    public synchronized double dPitch() { return dPitch; }
    
    @Override
    public synchronized double dRoll() { return dRoll; }
    
    @Override
    public synchronized double dYaw() { return dYaw; }
    
    //--------------------------------------------------------------------------
    
    @Override
    public void run()
    {
        while (true)
        {
            synchronized (this)
            {
                t = currentTimeMillis();
                
                pitch = robot().simDynState().pitch();
                roll = robot().simDynState().roll();
                yaw = robot().simDynState().yaw();
                
                dPitch = robot().simDynState().dPitch();
                dRoll = robot().simDynState().dRoll();
                dYaw = robot().simDynState().dYaw();
                
                try { log(); }
                catch (IOException e) { e.printStackTrace(System.err); }
            }
            msDelay(delay());
        }
    }
    
    //--------------------------------------------------------------------------
    
    private long t;
    private double pitch, roll, yaw;
    private double dPitch, dRoll, dYaw;
}
