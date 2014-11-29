package model.sensor;

import geom3d.Point3D;
import helper.Config;
import helper.MissingConfigException;
import helper.Ratio;

import java.io.File;
import java.io.IOException;

import run.RunConfig;

/**
 * Distance sensor configuration.
 */
public class DistanceSensorConfig extends Config
{
    public DistanceSensorConfig(File file)
    throws IOException, MissingConfigException, ClassNotFoundException
    {
        super (file);
        
        cls = "model.sensor." + getStringConfig("class");
        
        position = new Point3D(getDoubleConfig("x"),
                               getDoubleConfig("y"),
                               getDoubleConfig("z"));
        
        pitch = getDoubleConfig("pitch") * Ratio.DEG_TO_RAD;
        yaw = getDoubleConfig("yaw") * Ratio.DEG_TO_RAD;

        orientation = Point3D.unitX().rotateY(pitch).rotateZ(yaw);
        
        maxValue = getDoubleConfig("max-value");
    }
    
    //--------------------------------------------------------------------------

    /** @return distance sensor class */
    public String sensorClass() { return cls; }
    
    /** @return relative position (mm) to the axle midpoint */
    public Point3D position() { return position; }
    
    /** @return orientation axis along the beam ray (unit vector) */
    public Point3D orientation() { return orientation; }
    
    /** @return pitch angle (rad) rotated around the sensor's position */
    public double pitch() { return pitch; }
    
    /** @return yaw angle (rad) rotated around the sensor's position */
    public double yaw() { return yaw; }
    
    /** @return maximum value which can be measured by the sensor */
    public double maxValue() { return maxValue; }
    
    //--------------------------------------------------------------------------
    
    private final String cls;
    private final Point3D position, orientation;
    private final double pitch, yaw;
    private final double maxValue;
}
