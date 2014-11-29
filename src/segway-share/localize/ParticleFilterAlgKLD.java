package localize;

import geom3d.Point3D;
import helper.MultiMap;
import helper.Ratio;
import helper.Statistics;

import java.util.HashMap;
import java.util.LinkedList;

import localize.Particle;
import localize.ParticleCloud;
import model.motion.MotionConfig;
import model.scene.Box;
import model.scene.SceneModel;
import model.scene.SceneModel.DistanceResult;
import model.sensor.DistanceSensorConfig;

/**
 * Particle filter algorithm with adaptive cloud size (KLD-sampling).
 */
public final class ParticleFilterAlgKLD extends ParticleFilterAlg
{
    public static final int N_MAX = 10000; // maximum number of particles in a cloud
    
    public static final double STD_PSI = 0.01;
    public static final double STD_THETA_DRIVE = 0.4;
    public static final double STD_THETA_STEER = 0.2;
    public static final double EFF_RATIO = 0.2;

    public static final double IR_MIN_DENSITY = 1e-10;
    public static final int IR_USE_MIN = 70;
    public static final int IR_USE_MAX = 350;
    public static final double IR_STD = 50;
    
    // bin precision along state coordinates
    public static final double PITCH_PREC = 1.0 * Ratio.DEG_TO_RAD;
    public static final double YAW_PREC = 5.0 * Ratio.DEG_TO_RAD;
    public static final double XY_PREC = 5.0; // mm
    public static final double EPSILON = 0.015; // K-L distance upper bound
    
    public static final double MIN_WHEEL_DIST = 0; // mm (0 means disabled)
    
    public static final double FIND_SALT = 1e-15;
    public static final double FIND_XY_PREC = 20; // mm
    public static final double FIND_YAW_PREC = 10; // deg
    
    //--------------------------------------------------------------------------
    
    private final Point3D pos = new Point3D();
    private final DistanceResult distanceResult = new DistanceResult();
    private final double[] incWeights = new double[N_MAX];
    private final double[] incWeightsNew = new double[N_MAX];
    private final long[] basis = new long[3];
    private final HashMap<Long, Boolean> bins = new HashMap<Long, Boolean>();
    private final LinkedList<Box> boxes = new LinkedList<Box>();
    
    private double accDMrcL, accDMrcR; // rad
    private final double dMrcThres; // rad
    
    private final double halfR, R; // mm
    
    public ParticleFilterAlgKLD(long seed, MotionConfig mcfg, SceneModel scene)
    {
        // create 2 particle clouds (1 for visualization, 1 for working)
        super (seed, mcfg, scene, 2, N_MAX);
        R = mcfg.R * Ratio.M_TO_MM;
        halfR = R / 2.0;
        dMrcThres = MIN_WHEEL_DIST / R;
        
        basis[0] = (long)(           360.0 * Ratio.DEG_TO_RAD / PITCH_PREC);
        basis[1] = (long)(basis[1] * 360.0 * Ratio.DEG_TO_RAD / YAW_PREC);
        basis[2] = (long)(basis[2] * scene.floor().width() / XY_PREC);
        
        double z = scene.segway().bodyHeight() + scene.segway().wheelRadius();
        for (Box box : scene.boxes())
            if (box.elevation() <= z) boxes.add(box);
                
        ParticleCloud pC = clouds()[0];
        pC.setSize(0);
        initViewedCloud(pC);
        
        accDMrcL = accDMrcR = 0.0;
        final double onePerN = 1.0/N_MAX;
        for (int j = 0; j < N_MAX; ++j)
            incWeights[j] = (j+1)*onePerN;
        incWeights[N_MAX-1] = 1.0; // just to be sure
    }
    
    //--------------------------------------------------------------------------
    
    @Override
    public void init(double pitch, int[] ir)
    {
        final ParticleCloud cloud = nextCloud();
        cloud.setSize(N_MAX);

        double w;
        Particle p;

        final double psi = pitch * Ratio.DEG_TO_RAD;
        final double yawStep = FIND_YAW_PREC * Ratio.DEG_TO_RAD;
        final int yawN = (int)(360.0 / FIND_YAW_PREC);

        int pI = 0, i;
        MultiMap<Double, Particle> ordered = new MultiMap<Double, Particle>();
        double x = FIND_XY_PREC, y = FIND_XY_PREC, phi;
        while (y < scene().floor().height())
        {
            if (!scene().isUnderAnyBox(x, y, boxes))
            {
                for (i = 0; i < yawN; ++i)
                {
                    phi = i * yawStep;
                    w = computeWeight(x, y, psi, phi, ir, null);
                    
                    // salting the weight to break ties
                    w += rng().nextGaussian() * FIND_SALT;

                    if (ordered.size() < N_MAX)
                    {
                        p = cloud.get(pI++);
                        set(p, w, x, y, psi, phi);
                        ordered.put(w, p);
                    }
                    else if (w > ordered.firstKey())
                    {
                        p = ordered.pollFirstValue();
                        set(p, w, x, y, psi, phi);
                        ordered.put(w, p);                        
                    }
                }
            }

            x += FIND_XY_PREC;
            if (x >= scene().floor().width())
            {
                x = FIND_XY_PREC;
                y += FIND_XY_PREC;
            }
        }

        final double onePerN = 1.0/N_MAX;
        for (i = 0; i < N_MAX; ++i)
            cloud.get(i).setWeight(onePerN);
        
        setCloudAndEstimate(cloud, null);
    }
    
    @Override
    public void track(double pitch, int dMrcL, int dMrcR, int[] ir)
    {
        final ParticleCloud cV = particles();
        if (cV.size() == 0)
        {
            init(pitch, ir);
            return;
        }
        final ParticleCloud cA = nextCloud();
                
        final double pitchRad = pitch * Ratio.DEG_TO_RAD;
        final double dMrcLRad = dMrcL * Ratio.DEG_TO_RAD;
        final double dMrcRRad = dMrcR * Ratio.DEG_TO_RAD;

        accDMrcL += dMrcLRad;
        accDMrcR += dMrcRRad;
        if (Math.abs(accDMrcL) < dMrcThres && Math.abs(accDMrcR) < dMrcThres)
            return; // robot's position & orientation did not changed too much
        
        long hash;
        Particle pFrom, pTo;
        double psi, phi, dThetaSum, x , y, w;
        
        int pIdx = 0, k = 0;
        double wSum = 0.0;
        bins.clear();
        do
        {
            pFrom = sampleParticle(cV);
            pTo = cA.get(pIdx);
            
            // sampling based on the motion model
            
            psi = Statistics.gaussian(rng(), pitchRad, STD_PSI);
            double dNoise = Statistics.gaussian(rng(), 0.0, STD_THETA_DRIVE);
            double sNoise = Statistics.gaussian(rng(), 0.0, STD_THETA_STEER);
            
            double dThetaL = accDMrcL + dNoise - sNoise;
            double dThetaR = accDMrcR + dNoise + sNoise;
            dThetaSum = dThetaL + dThetaR;
            
            pTo.setPitch(psi);
            pTo.setThetaL(pFrom.thetaL() + dThetaL);
            pTo.setThetaR(pFrom.thetaR() + dThetaR);
            
            phi = yaw(pTo);
            x = pFrom.x() + halfR * dThetaSum * Math.cos(phi);
            y = pFrom.y() + halfR * dThetaSum * Math.sin(phi);
            pTo.setX(x);
            pTo.setY(y);
            
            // updating the weights

            w = computeWeight(x, y, psi, phi, ir, pTo.distance());
            pTo.setWeight(w);
            wSum += w;
            incWeightsNew[pIdx] = wSum;
            
            // check whether we have enough particles
            
            hash = binHash(psi, phi, x, y);
            Boolean isEmpty = bins.put(hash, true);
            if (isEmpty == null) ++k;
            
            ++pIdx;
        }
        while (pIdx < N_MAX && pIdx < k/EPSILON);
        cA.setSize(pIdx);
        
        // normalizing and computing the position estimate particle
        
        Particle p, pEst = null;
        double wEst = 0;
        for (int i = 0; i < cA.size(); ++i)
        {
            p = cA.get(i);
            w = p.weight() / wSum;
            incWeights[i] = incWeightsNew[i] / wSum;
            p.setWeight(w);
            
            if (w > wEst) // searching the maximum likelihood particle
            {
                pEst = p;
                wEst = w;
            }
        }
        incWeights[cA.size()-1] = 1.0; // just to be sure
        
        accDMrcL = accDMrcR = 0.0;
        setCloudAndEstimate(cA, pEst);
    }
    
    //--------------------------------------------------------------------------
    
    private Particle sampleParticle(ParticleCloud cloud)
    {
        double w = rng().nextDouble(); // U(0,1)
        
        int iMin = 0, iMax = cloud.size()-1, iAvg;
        while (iMin+1 < iMax)
        {
            iAvg = (iMin + iMax) / 2;
            if (incWeights[iAvg] < w) iMin = iAvg; else iMax = iAvg;
        }        
        return cloud.get(iMax); 
    }
    
    private double density(double measurement, double distance)
    {
        double diffStd = (measurement - distance) / IR_STD;
        double density = Math.exp(-diffStd*diffStd/2.0) / IR_STD;
        return Math.max(IR_MIN_DENSITY, density);
    }

    private double computeWeight(double x, double y,
                                 double psi, double phi,
                                 int[] ir, double[] distance)
    {
        double w = 0.0;
        if (scene().isOnFloor(x, y) && !scene().isUnderAnyBox(x, y, boxes))
        {
            double dist;
            final DistanceSensorConfig[] distCfg = scene().distCfg();
            
            pos.set(x, y, R + (scene().isOnCarpet(x, y)
                              ? scene().carpet().height() : 0.0));
            w = 1.0;
            for (int i = 0; i < distCfg.length; ++i)
            {
                scene().realDistance(distCfg[i], pos, psi, phi, distanceResult);
                dist = distanceResult.distance();
                
                if (distance != null) distance[i] = dist;
                w *= density(ir[i], dist);
            }
        }
        return w;
    }
    
    private long binHash(double pitch, double yaw, double x, double y)
    {
        return (long)(y/XY_PREC)*basis[2]
             + (long)(x/XY_PREC)*basis[1]
             + (long)(yaw/YAW_PREC)*basis[0]
             + (long)(pitch/PITCH_PREC);
    }    
}
