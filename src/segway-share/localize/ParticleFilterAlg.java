package localize;

import java.util.Collection;

import geom3d.Point3D;
import helper.Statistics;
import localize.Particle;
import localize.ParticleCloud;
import localize.ParticleFilter;
import model.motion.MotionConfig;
import model.scene.Box;
import model.scene.SceneModel;

/**
 * Common part of the particle filter implementations.
 */
public abstract class ParticleFilterAlg extends ParticleFilter
{
    public ParticleFilterAlg(long seed,
                             MotionConfig mcfg,
                             SceneModel scene,
                             int nParticleClouds,
                             int nMaxParticles)
    {
        super (seed, mcfg, scene, nParticleClouds);
        
        for (int i = 0; i < nParticleClouds; ++i)
        {
            clouds()[i] = new ParticleCloud(nMaxParticles);
            initParticleCloud(clouds()[i]);
        }
    }

    private void initParticleCloud(ParticleCloud pC)
    {
        final int irN = scene().distCfg().length;
        for (int i = 0; i < pC.size(); ++i)
            pC.set(i, new Particle(irN));
    }
    
    //--------------------------------------------------------------------------
    
    public abstract void init(double pitch, int[] ir);
    public abstract void track(double pitch, int dMrcL, int dMrcR, int[] ir);
    
    //--------------------------------------------------------------------------
    
    protected void putParticleUniFixedPoint(Particle p, double range)
    {
        final int n = scene().fixedPoints().size();
        Point3D loc = scene().fixedPoints().get(rng().nextInt(n)).position();
        p.setX(loc.x() + Statistics.uniform(rng(), -range, range));
        p.setY(loc.y() + Statistics.uniform(rng(), -range, range));
        p.setPitch(0); p.setThetaL(0); p.setThetaR(0);
    }
    
    protected void putParticleUniAvoidBox(Particle p, Collection<Box> boxes)
    {
        do
        {
            set(p,
                p.weight(),
                Statistics.uniform(rng(), 0.0, scene().floor().width()),
                Statistics.uniform(rng(), 0.0, scene().floor().height()),
                Statistics.uniform(rng(), -0.1, 0.1),
                Statistics.uniform(rng(), -Math.PI, Math.PI));
        }
        while (scene().isUnderAnyBox(p.x(), p.y(), boxes));
    }
    
    //--------------------------------------------------------------------------
    
    protected void fillParticleUniFixedPoint(ParticleCloud pC, double range)
    {
        final double onePerN = 1.0/pC.size();
        
        Particle p;
        for (int i = 0; i < pC.size(); ++i)
        {
            p = pC.get(i);
            p.setWeight(onePerN);
            putParticleUniFixedPoint(p, range);
        }
    }
    
    protected void fillParticleUniAvoidBox(ParticleCloud pC,
                                           Collection<Box> boxes)
    {
        final double onePerN = 1.0/pC.size();
        
        Particle p;
        for (int i = 0; i < pC.size(); ++i)
        {
            p = pC.get(i);
            p.setWeight(onePerN);
            putParticleUniAvoidBox(p, boxes);
        }
    }
}
