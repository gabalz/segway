package run;

import helper.Config;
import helper.MissingConfigException;

import java.io.File;
import java.io.IOException;

import vecmat.Vector;

/**
 * General running configuration.
 */
public class RunConfig extends Config
{
    public RunConfig(File file)
    throws IOException, MissingConfigException
    {
        super (file);
        
        motionCfgFile = new File(getStringConfig("cfg-motion"));
        viewCfgFile = new File(getStringConfig("cfg-view"));
        simCfgFile = new File(getStringConfig("cfg-sim"));
        
        robotControllerClassName = getStringConfig("robot-controller");
        pcControllerClassName = getStringConfig("pc-controller");
        sceneMapFile = new File(getStringConfig("map-file"));
        
        String sensorConfigPathPrefix =
            getStringConfig("sensor-config-path-prefix");
        String sensorConfigPath = sensorConfigPathPrefix + File.separator;
        
        gyroSensorConfig = new File(sensorConfigPath + "gyro.cfg");
        
        distSensorConfigClass = getStringConfig("dist-config-class");
        Vector distSensorConfigIndices =
            getVectorConfig("dist-sensor-config-indices", Vector.create(0));
        distSensorConfig = new File[distSensorConfigIndices.length()];
        for (int i = 0; i < distSensorConfig.length; ++i)
            distSensorConfig[i] =
                new File(sensorConfigPath + "distance-"
                         + (int)distSensorConfigIndices.get(i) + ".cfg");
    }

    //--------------------------------------------------------------------------
    
    /** @return motion configuration file path */
    public File motionCfgFile()
    { return motionCfgFile; }
    
    /** @return view configuration file path */
    public File viewCfgFile()
    { return viewCfgFile; }
    
    /** @return simulator configuration file path */
    public File simCfgFile()
    { return simCfgFile; }
    
    /** @return class name of the robot controller */
    public String robotControllerClassName()
    { return robotControllerClassName; }
    
    /** @return class name of the pc controller */
    public String pcControllerClassName()
    { return pcControllerClassName; }
    
    /** @return scene map file */
    public File mapFile()
    { return sceneMapFile; }
    
    /** @return gyroscope sensor configuration file */
    public File gyroSensorConfig()
    { return gyroSensorConfig; }
    
    /** @return distance sensor configuration class */
    public String distanceSensorConfigClass()
    { return distSensorConfigClass; }
    
    /** @return distance sensor configuration files */
    public File[] distSensorConfigs()
    { return distSensorConfig; }
    
    //--------------------------------------------------------------------------
    
    private final File motionCfgFile;
    private final File viewCfgFile;
    private final File simCfgFile;
    
    private final String robotControllerClassName;
    private final String pcControllerClassName;
    private final File sceneMapFile;
    
    private final File gyroSensorConfig;
    private final String distSensorConfigClass;
    private final File[] distSensorConfig;
}
