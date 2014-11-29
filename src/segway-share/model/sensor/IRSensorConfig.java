package model.sensor;

import geom3d.Point3D;
import helper.MissingConfigException;
import helper.Ratio;

import java.io.File;
import java.io.IOException;

import vecmat.Vector;
import model.sensor.DistanceSensorConfig;

/**
 * Configuration of an IR sensor.
 */
public final class IRSensorConfig extends DistanceSensorConfig
{
    private final Vector weights, pitchOffsets, yawOffsets;
    private final Point3D[] orientations;
    
    //--------------------------------------------------------------------------
    
    public IRSensorConfig(File file)
    throws IOException, MissingConfigException, ClassNotFoundException
    {
        super (file);
        
        weights = getVectorConfig("weights");
        pitchOffsets = getVectorConfig("pitch-offsets").mulL(Ratio.DEG_TO_RAD);
        yawOffsets = getVectorConfig("yaw-offsets").mulL(Ratio.DEG_TO_RAD);
        
        orientations = new Point3D[numOfSamples()];
        for (int i = 0; i < orientations.length; ++i)
        {
            orientations[i] = new Point3D(1, 0, 0)
                              .rotateY(pitch() + pitchOffsets().get(i))
                              .rotateZ(yaw() + yawOffsets().get(i));
        }
    }
    
    //--------------------------------------------------------------------------
    
    public int numOfSamples() { return weights.length(); }
    public Point3D[] orientations() { return orientations; }
    
    public Vector weights() { return weights; }
    public Vector pitchOffsets() { return pitchOffsets; }
    public Vector yawOffsets() { return yawOffsets; }
}
