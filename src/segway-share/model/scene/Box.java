package model.scene;

import geom3d.Brick;
import geom3d.Point3D;
import geom3d.Parallelepiped.BoundingBallUsage;

/**
 * Model of a box.
 */
public final class Box extends AbstractSceneModelObjectHOB
{
    public Box(Point3D position,
               double dX, double dY, double dZ,
               double pitch, double yaw)
    {
        super (new Brick(new Point3D(0, 0, 0),
                         dX, dY, dZ,
                         BoundingBallUsage.USE)
                   .rotateY(pitch).rotateZ(yaw).translate(position),
               new Color(150, 125, 85));
        
        elevation = position.z();
    }
    
    public double elevation() { return elevation; }
    
    private final double elevation;
}
