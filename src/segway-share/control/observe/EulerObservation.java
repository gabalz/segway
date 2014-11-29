package control.observe;

import java.io.IOException;

import helper.Ratio;
import control.RobotController;

/**
 * Observation estimation by Euler differentiation and moving average smoothing.
 */
public final class EulerObservation extends Observation
{
    public EulerObservation(RobotController controller, int freq,
                            int gyroSign, double gyroOffset,
                            double a1, double a2, double b1,
                            double R, double W,
                            int historyLength)
    {
        super (controller, freq, gyroSign, gyroOffset);
        lrOrder = false;
        
        this.R = R;
        this.W = W;
        
        this.a1 = a1;
        this.a2 = a2;
        this.b1 = b1;
        
        this.historyLength = historyLength;
        assert (historyLength >= 0);
        
        dPitchH = new double[historyLength];
        dRollH = new double[historyLength];
        dYawH = new double[historyLength];
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
    
    private double readLeftRotCtr()
    { return robot().leftRotationCounter() * Ratio.DEG_TO_RAD; }
    
    private double readRightRotCtr()
    { return robot().rightRotationCounter() * Ratio.DEG_TO_RAD; }
    
    private double readGyro()
    { return gyroSign()*robot().readGyro()*Ratio.DEG_TO_RAD - gyroOffset(); }
    
    //--------------------------------------------------------------------------
    
    @Override
    public void run()
    {
        pitch = roll = yaw = 0;
        dPitch = dRoll = dYaw = 0;
        
        prevDGyro = 0;
        for (int i = 0; i < historyLength; ++i)
            dPitchH[i] = dRollH[i] = dYawH[i] = 0;
        
        prevT = t = currentTimeMillis();
        prevGyro = readGyro();
        prevLRotCtr = readLeftRotCtr();
        prevRRotCtr = readRightRotCtr();
        
        try { log(); }
        catch (IOException e) { e.printStackTrace(System.err); }

        while (true)
        {
            msDelay(delay());
            synchronized (this)
            {
                t = currentTimeMillis();
                double gyro = readGyro();
                double lRotCtr, rRotCtr;
                if (lrOrder)
                    { lRotCtr = readLeftRotCtr(); rRotCtr = readRightRotCtr(); }
                else
                    { rRotCtr = readRightRotCtr(); lRotCtr = readLeftRotCtr(); }
                lrOrder = !lrOrder;
                
                double dt = ((double)(t-prevT)) / 1000.0; // sec
                
                double dGyro = (gyro - prevGyro) / dt;
                double ddGyro = (dGyro - prevDGyro) / dt;
                double dPitchT = (ddGyro + a1*dGyro + a2*gyro) / b1;
                
                double dLRotCtr = (lRotCtr - prevLRotCtr) / dt;
                double dRRotCtr = (rRotCtr - prevRRotCtr) / dt;
                double dRollT = (dLRotCtr + dRRotCtr)/2 + dPitchT;
                double dYawT = (dRRotCtr - dLRotCtr)*R/W;

                if (historyLength > 0)
                {
                    dPitch = dPitchH[0];
                    dRoll = dRollH[0];
                    dYaw = dYawH[0];
                    for (int i = 1; i < historyLength; ++i)
                    {
                        dPitch += dPitchH[i];
                        dRoll += dRollH[i];
                        dYaw += dYawH[i];
                        
                        dPitchH[i-1] = dPitchH[i];
                        dRollH[i-1] = dRollH[i];
                        dYawH[i-1] = dYawH[i];
                    }
                    dPitchH[historyLength-1] = dPitchT;
                    dRollH[historyLength-1] = dRollT;
                    dYawH[historyLength-1] = dYawT;
                    
                    dPitch = (dPitch + dPitchT) / (historyLength+1);
                    dRoll = (dRoll + dRollT) / (historyLength+1);
                    dYaw = (dYaw + dYawT) / (historyLength+1);
                }
                else
                {
                    dPitch = dPitchT;
                    dRoll = dRollT;
                    dYaw = dYawT;
                }
                
                pitch += dPitch*dt;
                roll = (lRotCtr + rRotCtr)/2 + pitch;
                yaw = (rRotCtr - lRotCtr)*R/W;
                
                prevT = t;
                prevGyro = gyro;
                prevDGyro = dGyro;
                prevLRotCtr = lRotCtr;
                prevRRotCtr = rRotCtr;
                
                try { log(); }
                catch (IOException e) { e.printStackTrace(System.err); }
            }
        }
    }
    
    //--------------------------------------------------------------------------
    
    // segway parameters
    private final double R, W;
    
    // rate gyroscope parameters
    private final double a1, a2, b1;
    
    private final int historyLength;
    private final double[] dPitchH, dRollH, dYawH; // histories for averages
    
    private boolean lrOrder;
    
    private long t, prevT;
    private double prevLRotCtr, prevRRotCtr;
    private double prevGyro, prevDGyro;
    
    private double pitch, roll, yaw;
    private double dPitch, dRoll, dYaw;
}
