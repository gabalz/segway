package model.sensor;

import model.sensor.IRSensorState.DistanceMeasurement;
import geom3d.Point3D;
import vecmat.Vector;
import model.scene.SceneModel;
import model.scene.SceneModel.DistanceResult;
import model.sensor.DistanceSensor;
import model.sensor.DistanceSensorConfig;
import model.sensor.DistanceSensorState;

/**
 * Model of an infrared sensor.
 */
public final class IRSensor extends DistanceSensor
{
    public IRSensor(DistanceSensorConfig cfg, SceneModel scene)
    {
        super (cfg, scene);
        
        state = new IRSensorState(cfg().numOfSamples());
        result = new DistanceResult();
    }
    
    //--------------------------------------------------------------------------

    @Override
    public IRSensorConfig cfg() { return (IRSensorConfig) super.cfg(); }
    
    @Override
    public DistanceSensorState state() { return state; }
    
    @Override
    public void next(double time,
                     Point3D robotPosition,
                     double robotPitch,
                     double robotYaw)
    {
        final IRSensorConfig cfg = cfg();
        final Point3D[] orientations = cfg.orientations();
        final Vector weights = cfg.weights();
        
        double distance = 0.0, d;
        final DistanceMeasurement[] samples = state.measurement();
        for (int i = 0; i < samples.length; ++i)
        {
            scene().realDistance(cfg.position(),
                                 orientations[i],
                                 cfg.maxValue(),
                                 robotPosition,
                                 robotPitch,
                                 robotYaw,
                                 result);            
            d = result.distance();
            d = irDistance(d) + rng().nextGaussian()*d/100.0;
            distance += weights.get(i) * d;
        }
        if (distance > cfg.maxValue()) distance = cfg.maxValue();
        state.set(time, distance);
    }
    
    public static double irDistance(double realDistance)
    {
        /*
        double d = 0.0;
        if (realDistance < 85)
        {
            d = (realDistance <= 55.0)
              ? -1.68 * realDistance + 185.0
              :  0.07 * realDistance +  88.38;
        }
        else
        {
            d = (realDistance <= 380.0)
              ? 1.07 * realDistance +  7.64
              : 0.97 * realDistance + 60.99;
        }
        */
        //return d;
        return realDistance;
    }
    
    //--------------------------------------------------------------------------

    private final IRSensorState state;
    private final DistanceResult result;
}
