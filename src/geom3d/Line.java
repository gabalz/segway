package geom3d;

/**
 * Representation of a 3D line.
 */
public class Line
{
    public static Line axisX()
    { return new Line(Point3D.origin(), Point3D.unitX()); }

    public static Line axisY()
    { return new Line(Point3D.origin(), Point3D.unitY()); }

    public static Line axisZ()
    { return new Line(Point3D.origin(), Point3D.unitZ()); }

    //--------------------------------------------------------------------------

    /**
     * Creates the line defined by its point (p0) and orientation (p1).
     */
    public Line(Point3D p, Point3D u)
    {
        this.p = p.copy();
        this.u = u.copy().normalize2();
    }

    /** @return copied line (placed into "result" */
    public Line copy(Line result)
    {
        p().copy(result.p());
        u().copy(result.u());
        return result;
    }
    
    /**
     * Copies the line (copies the point objects too).
     */
    public Line copy()
    {
        return new Line(p(), u());
    }

    //--------------------------------------------------------------------------

    public Point3D p() { return p; }
    public Point3D u() { return u; }

    /**
     * @return point p0 + t * p1 (placed into a new point)
     */
    public Point3D eval(double t)
    {
        return eval(t, new Point3D());
    }
    
    /**
     * @return point p + t * u (placed into "result")
     */
    public Point3D eval(double t, Point3D result)
    {
        return u().mul(t, result).addL(p());
    }

    //--------------------------------------------------------------------------

    /**
     * Rotate the line around the x-axis in the yz-plane.
     * @param angle rotation angle (rad), positive rotates counterclockwise
     * @return rotated "this"
     */
    public Line rotateX(double angle)
    {
        return rotateX(Math.cos(angle), Math.sin(angle));
    }

    /**
     * Rotate the line around the x-axis in the yz-plane.
     * @param cosAngle cosine of the rotation angle
     * @param sinAngle sine of the rotation angle
     *        (positive rotates counterclockwise)
     * @return rotated "this"
     */
    public Line rotateX(double cosAngle, double sinAngle)
    {
        p().rotateX(cosAngle, sinAngle);
        u().rotateX(cosAngle, sinAngle);
        return this;
    }

    /**
     * Rotate the line around the y-axis in the xz-plane.
     * @param angle rotation angle (rad), positive rotates counterclockwise
     * @return rotated "this"
     */
    public Line rotateY(double angle)
    {
        return rotateY(Math.cos(angle), Math.sin(angle));
    }

    /**
     * Rotate the line around the y-axis in the xz-plane.
     * @param cosAngle cosine of the rotation angle
     * @param sinAngle sine of the rotation angle
     *        (positive rotates counterclockwise)
     * @return rotated "this"
     */
    public Line rotateY(double cosAngle, double sinAngle)
    {
        p().rotateY(cosAngle, sinAngle);
        u().rotateY(cosAngle, sinAngle);
        return this;
    }

    /**
     * Rotate the line around the z-axis in the xy-plane.
     * @param angle rotation angle (rad), positive rotates counterclockwise
     * @return rotated "this"
     */
    public Line rotateZ(double angle)
    {
        return rotateZ(Math.cos(angle), Math.sin(angle));
    }

    /**
     * Rotate the line around the z-axis in the xy-plane.
     * @param cosAngle cosine of the rotation angle
     * @param sinAngle sine of the rotation angle
     *        (positive rotates counterclockwise)
     * @return rotated "this"
     */
    public Line rotateZ(double cosAngle, double sinAngle)
    {
        p().rotateZ(cosAngle, sinAngle);
        u().rotateZ(cosAngle, sinAngle);
        return this;
    }

    /**
     * Translate the line by the given vector.
     * @param v translation vector
     * @return translated "this"
     */
    public Line translate(Point3D v)
    {
        p().addL(v);
        return this;
    }

    //--------------------------------------------------------------------------

    @Override
    public String toString()
    {
        return "line{" + p() + "->" + u() + "}";
    }

    //--------------------------------------------------------------------------

    private final Point3D p, u;
}
