package geom3d;

/**
 * A cylinder defined by a position "p", a line segment "u" and a radius "r".
 * 
 *      /---\ circle at "p"+"u" with radius "r"
 *      \---/
 *      |   |
 *      |   | line segment is described by vector "u"
 *      |   |
 *      \---/ circle at "p" with radius "r"
 */
public class Cylinder
{
    public Cylinder(Point3D p, Point3D u, double r)
    {
        this.p = p;
        this.u = u;
        this.r = r;
        center = u.copy().divL(2.0).addL(p);
    }

    public Point3D p() { return p; }
    public Point3D u() { return u; }
    public double radius() { return r; }
    public Point3D center() { return center; }
    
    //--------------------------------------------------------------------------
    
    /**
     * Rotate the cylinder around the x-axis in the yz-plane.
     * @param angle rotation angle (rad), positive rotates counterclockwise
     * @return rotated "this"
     */
    public Cylinder rotateX(double angle)
    {
        return rotateX(Math.cos(angle), Math.sin(angle));
    }
    
    /**
     * Rotate the cylinder around the x-axis in the yz-plane.
     * @param cosAngle cosine of the rotation angle
     * @param sinAngle sine of the rotation angle
     *        (positive rotates counterclockwise)
     * @return rotated "this"
     */
    public Cylinder rotateX(double cosAngle, double sinAngle)
    {
        p.rotateX(cosAngle, sinAngle);
        u.rotateX(cosAngle, sinAngle);
        center.rotateX(cosAngle, sinAngle);
        return this;
    }
    
    /**
     * Rotate the cylinder around the y-axis in the xz-plane.
     * @param angle rotation angle (rad), positive rotates counterclockwise
     * @return rotated "this"
     */
    public Cylinder rotateY(double angle)
    {
        return rotateY(Math.cos(angle), Math.sin(angle));
    }

    /**
     * Rotate the cylinder around the y-axis in the xz-plane.
     * @param cosAngle cosine of the rotation angle
     * @param sinAngle sine of the rotation angle
     *        (positive rotates counterclockwise)
     * @return rotated "this"
     */
    public Cylinder rotateY(double cosAngle, double sinAngle)
    {
        p.rotateY(cosAngle, sinAngle);
        u.rotateY(cosAngle, sinAngle);
        center.rotateY(cosAngle, sinAngle);
        return this;
    }
    
    /**
     * Rotate the cylinder around the z-axis in the xy-plane.
     * @param angle rotation angle (rad), positive rotates counterclockwise
     * @return rotated "this"
     */
    public Cylinder rotateZ(double angle)
    {
        return rotateZ(Math.cos(angle), Math.sin(angle));
    }

    /**
     * Rotate the cylinder around the z-axis in the xy-plane.
     * @param cosAngle cosine of the rotation angle
     * @param sinAngle sine of the rotation angle
     *        (positive rotates counterclockwise)
     * @return rotated "this"
     */
    public Cylinder rotateZ(double cosAngle, double sinAngle)
    {
        p.rotateZ(cosAngle, sinAngle);
        u.rotateZ(cosAngle, sinAngle);
        center.rotateZ(cosAngle, sinAngle);
        return this;
    }
    
    /**
     * Translate the cylinder by the given vector.
     * @param v translation vector
     * @return translated "this"
     */
    public Cylinder translate(Point3D v)
    {
        p.addL(v);
        center.addL(v);
        return this;
    }
    
    //--------------------------------------------------------------------------
    
    private final Point3D p, u, center;
    private final double r;
}
