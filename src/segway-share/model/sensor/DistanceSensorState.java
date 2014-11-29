package model.sensor;

/**
 * Superclass of the internal state representation of the distance sensors.
 */
public class DistanceSensorState
{
    private double time = 0.0;
    private double distanceMeasurement = 0.0;

    //--------------------------------------------------------------------------

    /** @return time of the last update (sec) */
    public double time() { return time; }
    
    /** @return current distance sensor measurement (mm) */
    public double read() { return distanceMeasurement; }

    /** Reset to the initial state. */
    public void reset()
    { set(0.0, 0.0); }
    
    /** Set the distance measurement state attributes. */
    public void set(double time, double distanceMeasurement)
    {
        this.time = time;
        this.distanceMeasurement = distanceMeasurement;
    }
    
    /** @return copied distance sensor state (placed into "result") */
    public DistanceSensorState copy(DistanceSensorState result)
    {
        result.time = time;
        result.distanceMeasurement = distanceMeasurement;
        return result;
    }
}
