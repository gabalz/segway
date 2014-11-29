package model.sensor;

import model.scene.SceneModelObject;
import model.sensor.DistanceSensorState;

/**
 * Internal state of the IR sensor.
 */
public final class IRSensorState extends DistanceSensorState
{
    private final DistanceMeasurement[] measurement;
    
    //--------------------------------------------------------------------------
    
    public IRSensorState(int numOfMeasurements)
    {
        measurement = new DistanceMeasurement[numOfMeasurements];
    }
    
    /** @return the measurement list taken by the IR sensor */
    public DistanceMeasurement[] measurement() { return measurement; }

    //--------------------------------------------------------------------------
    
    /**
     * A single measurement (by one ray) of a distance sensor.
     */
    public final static class DistanceMeasurement
    {
        private double distance = 0.0;
        private SceneModelObject hitObject = null;
        
        /** @return true if something is hit */
        public boolean isHit() { return hitObject != null; }
        
        /** @return real distance traveled by a sensor ray (mm) */
        public double distance() { return distance; }
        
        /** @return hit scene object (or null if nothing is hit) */
        public SceneModelObject hitObject() { return hitObject; }
        
        /** Set the measurement state. */
        public void set(double distance, SceneModelObject hitObject)
        {
            this.distance = distance;
            this.hitObject = hitObject;
        }
    }
}
