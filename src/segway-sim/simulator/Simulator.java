package simulator;

import geom3d.Point3D;
import helper.Ratio;
import helper.Statistics;

import java.util.LinkedList;
import java.util.List;
import java.util.Random;

import model.motion.MotionConfig;
import model.motion.MotionModel;
import model.motion.State;
import model.scene.FixedPoint;
import model.scene.SceneModel;
import model.sensor.DistanceSensor;
import model.sensor.DistanceSensorConfig;
import model.sensor.GyroSensor;
import model.sensor.GyroSensorConfig;

import run.SimulatedThread;

public class Simulator
{
    public Simulator(SimConfig simCfg,
                     GyroSensorConfig gyroCfg,
                     DistanceSensorConfig[] distCfg,
                     MotionConfig motionModelCfg,
                     SceneModel sceneModel)
    throws Exception
    {
        cfg = simCfg;
        threads = new LinkedList<SimulatedThread>();
        isPaused = false;
        isStanding = false;

        rng = new Random();

        this.sceneModel = sceneModel;
        motionModel = new MotionModel(motionModelCfg, cfg.simDT());

        gyro = (GyroSensor)
               Class.forName(gyroCfg.sensorClass())
               .getConstructor(GyroSensorConfig.class)
               .newInstance(gyroCfg);
        
        distPosTmp = new Point3D();
        dist = new DistanceSensor[distCfg.length];
        for (int i = 0; i < dist.length; ++i)
            dist[i] = (DistanceSensor)
                      Class.forName(distCfg[i].sensorClass())
                           .getConstructor(DistanceSensorConfig.class,
                                           SceneModel.class)
                           .newInstance(distCfg[i], sceneModel);
    }
    
    public SimConfig cfg() { return cfg; }
    public Random rng() { return rng; }
    
    //--------------------------------------------------------------------------
    
    /** @return the current simulation time (sec) */
    public double time()
    { synchronized (this) { return state().time(); } }
    
    /** @return the current simulation time (msec) */
    public long currentTimeMillis()
    { return (long)(time()*Ratio.SEC_TO_MILLISEC); }
    
    /** @return current simulation state */
    public State state() { return motionModel.state(); }

    /** @return gyroscope sensor */
    public GyroSensor gyro() { return gyro; }
    
    /** @return distance sensors */
    public DistanceSensor[] dist() { return dist; }
    
    /** Reset the simulation (with a laid down robot). */
    public void reset()
    {
        rng.setSeed(cfg.seed() == 0 ? System.nanoTime() : cfg.seed());
        
        synchronized (this)
        {
            gyro.reset(rng.nextLong());
            
            Point3D p = null;
            List<FixedPoint> points = sceneModel.fixedPoints();
            if (points.size() > 0)
                p = points.get(rng.nextInt(points.size())).position();
            else
                p = new Point3D(sceneModel.floor().width()/2,
                                sceneModel.floor().height()/2,
                                0);
            
            // initial simulation time
            double t0 = cfg.isTime0() ? 0.0 : System.currentTimeMillis()/1000.0;
            
            motionModel.setDt(cfg.simDT());
            motionModel.setState(
                new State(motionModel.motionModelConfig(),
                          t0,
                          p.x() * Ratio.MM_TO_M +
                              Statistics.uniform(rng, 
                                                 cfg.initXPositionDevRange()),
                          p.y() * Ratio.MM_TO_M +
                              Statistics.uniform(rng,
                                                 cfg.initYPositionDevRange()),
                          -motionModel.maxPitch(), // robot is laid down
                          0.0,   // roll
                          Statistics.uniform(rng, cfg.initYawRange()), // yaw
                          0.0,   // dPitch
                          0.0,   // dRoll
                          0.0)); // dYaw
            notifyAll();
        }
    }

    /** @return true if the robot is supposed to be standing */
    public boolean isStanding() { return isStanding; }

    /** @return true if the robot body hits the ground */
    public boolean isGroundHit()
    {
        boolean isGroundHit = motionModel.isGroundHit();
        if (isGroundHit) isStanding = false;
        return isGroundHit;
    }
    
    /** Stand up the robot. */
    public void standUpRobot()
    {
        synchronized (this)
        {
            motionModel.setState(
                    new State(motionModel.motionModelConfig(),
                              time(),
                              state().x(),
                              state().y(),
                              Statistics.uniform(rng, cfg.initPitchRange()),
                              0.0, // roll
                              state().yaw(),
                              Statistics.uniform(rng, cfg.initPitchVelRange()),
                              0.0, // dRoll
                              Statistics.uniform(rng, cfg.initYawVelRange())));
            notifyAll();
            isStanding = true;
        }
    }
    
    /** Step the simulation using the specified control powers (milliV). */
    public void step(double leftPower, double rightPower)
    throws InterruptedException
    {
        synchronized (threads)
        {
            if (isPaused) threads.wait();
            
            int dt = calculateDT();
            synchronized (this)
            {
                long time = currentTimeMillis();
                final long targetTime = time + dt;
                while (time < targetTime)
                {
                    gyro.next(time,
                              state().dPitch() * Ratio.RAD_TO_DEG,
                              cfg.simDT());
                    
                    motionModel.step(leftPower, rightPower);
                    time = currentTimeMillis();
                }
                notifyAll();
            }
        }
    }
    
    public void pauseOrResume()
    {
        synchronized (threads)
        {
            isPaused = !isPaused;
            if (!isPaused) threads.notifyAll();
        }
    }
    
    //--------------------------------------------------------------------------
    
    /** @return the gyroscope reading of the current state (deg/sec) */
    public double readGyro()
    {
        synchronized (this) { return gyro().read(); }
    }
    
    /** @return the left rotation counter (deg) */
    public double readLeftRotationCounter()
    {
        synchronized (this)
        {
            final State state = state();
            return (state.leftRoll() - state.pitch()) * Ratio.RAD_TO_DEG;
        }
    }
    
    /** @return the right rotation counter (deg) */
    public double readRightRotationCounter()
    {
        synchronized (this)
        {
            final State state = state();
            return (state.rightRoll() - state.pitch()) * Ratio.RAD_TO_DEG;            
        }
    }
    
    /** @return the "i"th distance sensor reading of the current state (mm) */
    public double readDistance(int i)
    {
        final double R = motionModel.motionModelConfig().R;
        final DistanceSensor sensor = dist()[i];
        synchronized (this)
        {
            final State state = state();
            if (sensor.state().time() < state.time())
            {
                // axle midpoint position (mm)
                distPosTmp.set(state.x() * Ratio.M_TO_MM,
                               state.y() * Ratio.M_TO_MM,
                               R * Ratio.M_TO_MM);
                
                sensor.next(state.time(),
                            distPosTmp,
                            state.pitch(),
                            state.yaw());
            }
            return sensor.read();
        }
    }
    
    //--------------------------------------------------------------------------
    
    /** Register a simulated thread to be synchronized with this simulation. */
    public void registerThread(SimulatedThread thread)
    {
        synchronized (threads)
        {
            threads.addLast(thread);
            thread.setEnabled(true);
        }
    }
    
    /**
     * @return the minimum time step to match the next running time
     *         of any of the registered simulator entities
     */
    private int calculateDT()
    throws InterruptedException
    {
        long time = currentTimeMillis(), nextTime = 0;
        long minNextTime = Long.MAX_VALUE;

        for (SimulatedThread thread : threads)
        {
            while (true)
            {
                synchronized (thread)
                {
                    nextTime = thread.nextTime();
                    if (!thread.isRunning() || !thread.isEnabled() ||
                        time < nextTime) break;
                    thread.wait();
                }
            }
            
            if (thread.isRunning() && thread.isEnabled() &&
                nextTime < minNextTime && time < nextTime)
                minNextTime = nextTime;
        }
        
        if (minNextTime == Long.MAX_VALUE)
            return (int)(cfg.simDT()*Ratio.SEC_TO_MILLISEC);
        return (int)(minNextTime - time);
    }
    
    //--------------------------------------------------------------------------
    
    private boolean isPaused, isStanding;
    private final LinkedList<SimulatedThread> threads;
    
    private final Random rng;
    private final SimConfig cfg;
    private final MotionModel motionModel;
    private final SceneModel sceneModel;

    private final GyroSensor gyro;
    private final DistanceSensor[] dist;

    private final Point3D distPosTmp;
}
