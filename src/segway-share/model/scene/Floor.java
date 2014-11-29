package model.scene;

import geom3d.Brick;
import geom3d.Point3D;

/**
 * Model of the floor of the scene.
 */
public final class Floor extends AbstractSceneModelObjectHOB
{
    public Floor(double width, double height, double thickness)
    {
        super (new Brick(new Point3D(0.0, 0.0, -thickness),
                         width, height, thickness),
                         new Color(65, 61, 53));
        
        this.thickness = thickness;
        this.width = width;
        this.height = height;
    }
    
    public double thickness() { return thickness; }
    public double width() { return width; }
    public double height() { return height; }
    
    //--------------------------------------------------------------------------
    
    private final double thickness, width, height;
}
