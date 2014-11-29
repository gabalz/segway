package geom3d;

/**
 * A special parallelepiped having orthogonal neighbor sides.
 */
public class Brick extends Parallelepiped
{
    /**
     * Define a brick at "p" having (signed) side lengths "dX", "dY", "dZ".
     */
    public Brick(Point3D p, double dX, double dY, double dZ)
    {
        this (p, dX, dY, dZ, BoundingBallUsage.IGNORE);
    }
    
    /**
     * Define a brick at "p" having (signed) side lengths "dX", "dY", "dZ".
     * If useBoundingBall is true, a bounding ball is created and used to speed
     * up the intersection test with line segments if the brick passes
     * the surface test (see BOUNDING_BALL_SURF_AREA_RATIO).
     */
    public Brick(Point3D p, double dX, double dY, double dZ,
                 BoundingBallUsage bbUsage)
    {
        super (p,
               Point3D.unitX().mulL(dX),
               Point3D.unitY().mulL(dY),
               Point3D.unitZ().mulL(dZ),
               bbUsage);
    }
}
