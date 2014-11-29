package localize;

import geom3d.Point3D;
import helper.Ratio;
import helper.Statistics;
import localize.Particle;
import localize.ParticleCloud;
import model.motion.MotionConfig;
import model.scene.SceneModel;
import model.scene.SceneModel.DistanceResult;
import model.sensor.IRSensorConfig;
import model.sensor.IRSensor;

/**
 * The "standard" particle filter algorithm.
 */
public final class ParticleFilterAlgStd extends ParticleFilterAlg
{
    public static final int N = 2000; // fixed number of particles in a cloud
    
    public static final double STD_PSI = 0.01;
    public static final double STD_THETA_DRIVE = 0.25;
    public static final double STD_THETA_STEER = 0.1;
    public static final double EFF_RATIO = 0.2;

    public static final double IR_MIN_DENSITY = 1e-10;
    public static final int IR_USE_MIN = 70;
    public static final int IR_USE_MAX = 350;
    public static final double IR_STD = 50;
 
    //--------------------------------------------------------------------------

    public ParticleFilterAlgStd(long seed, MotionConfig mcfg, SceneModel scene)
    {
        // create 3 particle clouds (1 for visualization, 2 for working)
        super (seed, mcfg, scene, 3, N);
        
        distanceResult = new DistanceResult();
        distCfg = new IRSensorConfig[scene.distCfg().length];
        for (int i = 0; i < distCfg.length; ++i)
            distCfg[i] = (IRSensorConfig) scene.distCfg()[i];
        irUse = new boolean[distCfg.length];
        
        R = mcfg.R * Ratio.M_TO_MM;
        halfR = R / 2.0;
        pos = new Point3D(0, 0, mcfg.R * Ratio.M_TO_MM);
    }
    
    //--------------------------------------------------------------------------

    @Override
    public void init(double pitch, int[] ir)
    {        
        Particle p;
        ParticleCloud cloud = nextCloud();
        final double onePerN = 1.0/cloud.size();
        for (int i = 0; i < cloud.size(); ++i)
        {
            p = cloud.get(i);
            putParticleUniFixedPoint(p, 100);
            p.setWeight(onePerN);
        }
        setCloudAndEstimate(cloud, null);
        track(pitch * Ratio.RAD_TO_DEG, 0, 0, ir);
    }
    
    @Override
    public void track(double pitch, int dMrcL, int dMrcR, int[] ir)
    {
        // Recommended work flow:
        //      cV = particles() -> filtering process -> cA = nextCloud()
        //      optionally use cB = nextCloud(), i.e. for re-sampling
        //      compute the robot position estimate -> estP
        //      setCloudAndEstimate(cA or cB, estP)
        
        ParticleCloud cV = particles();
        ParticleCloud cA = nextCloud();
        
        double pitchRad = pitch * Ratio.DEG_TO_RAD;
        for (int j = 0; j < irUse.length; ++j)
        {
            irUse[j] = false;
            if (IR_USE_MIN < ir[j])
            {
                if (IR_USE_MAX < ir[j]) ir[j] = IR_USE_MAX;
                irUse[j] = true;
            }
        }

        double dMrcLRad = dMrcL * Ratio.DEG_TO_RAD;
        double dMrcRRad = dMrcR * Ratio.DEG_TO_RAD;
        
        int i, j;
        Particle pFrom, pTo, pEst = estimate();
        double w, wSum = 0;
        double psi, phi, dThetaSum, x , y;
        for (i = 0; i < cV.size(); ++i)
        {
            pTo = cA.get(i);
            pFrom = cV.get(i);
            w = pFrom.weight();
            
            // sampling based on the motion model
            
            psi = Statistics.gaussian(rng(), pitchRad, STD_PSI);
            dThetaSum = updateAngles(pFrom, pTo,
                                     psi, dMrcLRad, dMrcRRad,
                                     STD_THETA_DRIVE,
                                     STD_THETA_STEER);
            phi = yaw(pTo);
            x = pFrom.x() + halfR * dThetaSum * Math.cos(phi);
            y = pFrom.y() + halfR * dThetaSum * Math.sin(phi);
            pTo.setX(x);
            pTo.setY(y);
            
            // updating the weights
            wSum += updateParticleWeight(pTo, w, x, y, psi, phi, ir, irUse);
        }
        
        // normalizing and computing the position estimate particle
        
        double effR = 0.0;
        pEst = null;
        double wEst = 0;
        for (Particle p : cA)
        {
            w = p.weight() / wSum;
            p.setWeight(w);
            if (w > wEst) // searching the maximum likelihood particle
            {
                pEst = p;
                wEst = w;
            }
            effR += w * w;
        }
        
        // re-sampling (if necessary) and activating the updated particle cloud
        
        int N = cA.size();
        if (1.0 < effR * EFF_RATIO * N)
        {
            double threshold = 0.5/N, onePerN = 1.0/N;
            ParticleCloud cB = nextCloud();
            
            i = 0;
            Particle p;
            w = cA.get(0).weight();
            for (j = 0; j < N; ++j)
            {
                while (w < threshold && i < N-1) w += cA.get(++i).weight();
                threshold += onePerN;
                
                p = cB.get(j);
                p.set(cA.get(i));
                p.setWeight(onePerN);
            }
            setCloudAndEstimate(cB, pEst);
        }
        else setCloudAndEstimate(cA, pEst);
    }
    
    //--------------------------------------------------------------------------

    public double updateParticleWeight(Particle p,
                                       double w,
                                       double x, double y,
                                       double psi, double phi,
                                       int[] ir,
                                       boolean[] irUse)
    {
        pos.set(x, y, R + (scene().isOnCarpet(x, y)
                          ? scene().carpet().height() : 0.0));
        for (int j = 0; j < irUse.length; ++j)
        {
            if (!irUse[j]) continue;
            
            final IRSensorConfig cfg = distCfg[j];
            
            double d, dW, dIR, dWdIR;
            double mean = 0.0;
            for (int k = 0; k < cfg.numOfSamples(); ++k)
            {
                scene().realDistance(cfg.position(),
                                     cfg.orientations()[k],
                                     cfg.maxValue(),
                                     pos,
                                     psi,
                                     phi,
                                     distanceResult);
                
                dW = cfg.weights().get(k);
                d = distanceResult.distance();
                
                dIR = IRSensor.irDistance(d);
                dWdIR = dW * dIR;
                mean += dWdIR;
            }
            
            mean = Math.min(mean, cfg.maxValue());
            
            p.distance()[j] = mean;
            w *= pdf(ir[j], mean);
        }
        p.setWeight(w);
        return w;
    }
    
    private double updateAngles(Particle pFrom, Particle pTo,
                                double psi, double dMrcLRad, double dMrcRRad,
                                double stdDrive, double stdSteer)
    {
        double dNoise = Statistics.gaussian(rng(), 0.0, stdDrive);
        double sNoise = Statistics.gaussian(rng(), 0.0, stdSteer);
        double dThetaL = dMrcLRad + dNoise - sNoise;
        double dThetaR = dMrcRRad + dNoise + sNoise;
        
        pTo.setPitch(psi);
        pTo.setThetaL(pFrom.thetaL() + dThetaL);
        pTo.setThetaR(pFrom.thetaR() + dThetaR);
        return dThetaL + dThetaR;
    }
    
    private double pdf(double measurement, double mean)
    {
        double diffStd = (measurement - mean) / IR_STD;
        double density = Math.exp(-diffStd*diffStd/2.0);
        return Math.max(IR_MIN_DENSITY, density);
    }
    
    //--------------------------------------------------------------------------

    private final IRSensorConfig[] distCfg;
    private final DistanceResult distanceResult;
    private final boolean[] irUse;
    
    private final Point3D pos;
    private final double halfR, R;
}
