package model.scene;

import geom3d.Brick;
import geom3d.Point3D;

/**
 * Model of the table carpet.
 */
public final class Carpet extends AbstractSceneModelObjectHOB
{
    public Carpet(double x, double y, double sizeX, double sizeY, double height)
    {
        super (new Brick(new Point3D(x, y, 0), sizeX, sizeY, height),
               new Color(30, 100, 30));
        
        this.height = height;
        xMin = x; xMax = x + sizeX;
        yMin = y; yMax = y + sizeY;
    }
    
    public double height() { return height; }
    
    public double xMin() { return xMin; }
    public double xMax() { return xMax; }
    public double yMin() { return yMin; }
    public double yMax() { return yMax; }

    //--------------------------------------------------------------------------
    
    private final double height, xMin, xMax, yMin, yMax;
}
