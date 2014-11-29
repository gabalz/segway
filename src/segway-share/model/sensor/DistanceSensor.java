package model.sensor;

import geom3d.Point3D;

import java.util.Random;

import model.scene.SceneModel;

/**
 * Model of a distance sensor.
 */
public abstract class DistanceSensor
{
    private final DistanceSensorConfig cfg;
    private final SceneModel scene;
    private final Random rng;    
    
    //--------------------------------------------------------------------------
    
    public DistanceSensor(DistanceSensorConfig cfg, SceneModel scene)
    {
        this.cfg = cfg;
        this.scene = scene;
        rng = new Random();
    }
    
    //--------------------------------------------------------------------------
    
    /** @return distance sensor configuration settings */
    public DistanceSensorConfig cfg() { return cfg; }
    
    /** @return random number generator */
    public Random rng() { return rng; }
    
    /** @return the used scene model */
    public SceneModel scene() { return scene; }

    /** @return the internal state of the distance sensor */
    public abstract DistanceSensorState state();
    
    //--------------------------------------------------------------------------
    
    /** Reset the sensor's state. */
    public void reset(long seed)
    {
        rng.setSeed(seed);
        state().reset();
    }

    /**
     * Update the state of the distance sensor.
     * @param time time (sec)
     * @param robotPosition position of the axle midpoint of the segway (mm)
     * @param robotPitch pitch angle of the segway (rad)
     * @param robotYaw yaw angle of the segway (rad)
     */
    public abstract void next(double time,
                              Point3D robotPosition,
                              double robotPitch,
                              double robotYaw);
    
    /** @return distance sensor measurement at the current state (mm) */
    public double read() { return state().read(); }
}
