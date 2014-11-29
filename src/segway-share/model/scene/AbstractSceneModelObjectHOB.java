package model.scene;

import geom3d.HalfLine;
import geom3d.Parallelepiped;
import geom3d.Point3D;

/**
 * Common implementation of scene model objects
 * which contain only one brick and can be hit.
 */
public abstract class AbstractSceneModelObjectHOB
                      extends AbstractSceneModelObject
{
    public AbstractSceneModelObjectHOB(Parallelepiped parallelepiped,
                                       Color color)
    {
        parallelepipeds = new Parallelepiped[]{parallelepiped};
        colors = new Color[]{color};
    }
    
    @Override
    public final Parallelepiped[] parallelepipedObjects()
    { return parallelepipeds; }
    
    @Override
    public final Color[] parallelepipedColors()
    { return colors; }

    //--------------------------------------------------------------------------
    
    @Override
    public final double hitAt(HalfLine ray, Point3D result)
    {
        double d = parallelepipedObjects()[0].intersection(ray, result);
        if (!Double.isInfinite(d)) d = Math.sqrt(d);
        return d;
    }
    
    //--------------------------------------------------------------------------
    
    private final Parallelepiped[] parallelepipeds;
    private final Color[] colors;
}
