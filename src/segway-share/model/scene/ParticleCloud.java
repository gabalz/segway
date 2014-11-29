package model.scene;

import geom3d.Parallelogram;
import geom3d.Point3D;
import localize.Particle;
import localize.ParticleFilter;

/**
 * Model of a particle cloud.
 */
public final class ParticleCloud extends AbstractSceneModelObject
{
    public ParticleCloud(ParticleFilter pf)
    {
        this.pf = pf;
        
        double s = 5; // particle rectangle scale
        double e = 5; // elevation (mm)
        Parallelogram rect = new Parallelogram(new Point3D(-s/2, -s/2, e),
                                               Point3D.unitX().mulL(s),
                                               Point3D.unitY().mulL(s));
        colorN = 100; // color gradient size
        rects = new Parallelogram[2*colorN];
        for (int i = 0; i < rects.length; ++i) rects[i] = rect;
        
        rectColors = new Color[2*colorN];
        float col1R = 25, col1G = 150, col1B = 150;
        float col2R = 240, col2G = 250, col2B = 125;
        float col3R = 250, col3G = 150, col3B = 50;
        for (int i = 0; i < colorN; ++i)
        {
            rectColors[i] = new Color(colorAvg(col1R, col2R, i, colorN),
                                      colorAvg(col1G, col2G, i, colorN),
                                      colorAvg(col1B, col2B, i, colorN));
            rectColors[colorN+i] = new Color(colorAvg(col2R, col3R, i, colorN),
                                             colorAvg(col2G, col3G, i, colorN),
                                             colorAvg(col2B, col3B, i, colorN));
        }
        pIndices = new int[1];
    }
    
    private float colorAvg(float low, float high, int i, int n)
    { return ((n-1-i)*low + i*high) / (n-1); }
    
    //--------------------------------------------------------------------------
    
    @Override
    public Parallelogram[] parallelogramObjects() { return rects; }
    
    @Override
    public Color[] parallelogramColors() { return rectColors; }
    
    @Override
    public boolean canBeHit() { return false; }
    
    @Override
    public int numDraws()
    {
        pc = pf.particles();
        final int N = (pc == null) ? 0 : pc.size();
        if (N > 0) onePerN = 1.0/N;
        return N;
    }

    @Override
    public int[] parallelogramIndices(int di)
    {
        final double w = p.weight();
        final int i = (int)(w * colorN) % colorN;
        pIndices[0] = (w < onePerN) ? i : colorN+i;
        return pIndices;
    }
    
    @Override
    public int numDynamicTransforms(int di)
    {
        p = pc.get(di);
        return 1;
    }
    
    @Override
    public Point3D translate(int di, int ti) { return pp; }

    //--------------------------------------------------------------------------
    
    private final class ParticlePoint extends Point3D
    {
        @Override
        public double x() { return p.x(); }
        
        @Override
        public double y() { return p.y(); }
    }
    private ParticlePoint pp = new ParticlePoint();
    
    //--------------------------------------------------------------------------
    
    private localize.ParticleCloud pc;
    private Particle p;
    private double onePerN;
    
    private final int colorN;
    private final int[] pIndices;
    private final ParticleFilter pf;
    private final Parallelogram[] rects;
    private final Color[] rectColors;
}
