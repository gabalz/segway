package geom3d;

import vecmat.Matrix;

/**
 * Representation of a 3D plane.
 */
public class Plane
{
    public static Plane planeXY()
    { return new Plane(Point3D.origin(), Point3D.unitX(), Point3D.unitY()); }

    public static Plane planeXZ()
    { return new Plane(Point3D.origin(), Point3D.unitX(), Point3D.unitZ()); }

    public static Plane planeYZ()
    { return new Plane(Point3D.origin(), Point3D.unitY(), Point3D.unitZ()); }

    //--------------------------------------------------------------------------

    /**
     * Defines a plane by an offset point 
     * and two vectors showing its orientation.
     */
    public Plane(Point3D p, Point3D n0, Point3D n1)
    {
        this.p = p;
        this.n0 = n0.copy().normalize2();
        this.n1 = n1.copy().normalize2();
        assert (!n0().isParallel(n1()));
        
        M = Matrix.create(2, 2);
        updateM = true;
        
        Ip = new Point3D();
        IM = Matrix.create(3, 3);
        IMr = Matrix.create(3, 3);
        updateIM = true;
    }

    //--------------------------------------------------------------------------

    public Point3D p() { return p; }
    public Point3D n0() { return n0; }
    public Point3D n1() { return n1; }

    /**
     * @return p + t * n0 + s * n1 (placed into a new point)
     */
    public Point3D eval(double t, double s)
    {
        return eval(t, s, new Point3D());
    }
    
    /**
     * @return p + t * n0 + s * n1 (placed into "result")
     */
    public Point3D eval(double t, double s, Point3D result)
    {
        for (int i = 0; i < result.length(); ++i)
            result.set(i, p.get(i) + t * n0.get(i) + s * n1.get(i));
        return result;
    }

    //--------------------------------------------------------------------------

    /**
     * Rotate the plane around the x-axis in the yz-plane.
     * @param angle rotation angle (rad), positive rotates counterclockwise
     * @return rotated "this"
     */
    public Plane rotateX(double angle)
    {
        return rotateX(Math.cos(angle), Math.sin(angle));
    }

    /**
     * Rotate the plane around the x-axis in the yz-plane.
     * @param cosAngle cosine of the rotation angle
     * @param sinAngle sine of the rotation angle
     *        (positive rotates counterclockwise)
     * @return rotated "this"
     */
    public Plane rotateX(double cosAngle, double sinAngle)
    {
        p().rotateX(cosAngle, sinAngle);
        n0().rotateX(cosAngle, sinAngle);
        n1().rotateX(cosAngle, sinAngle);
        updateM = updateIM = true;
        return this;
    }

    /**
     * Rotate the plane around the y-axis in the xz-plane.
     * @param angle rotation angle (rad), positive rotates counterclockwise
     * @return rotated "this"
     */
    public Plane rotateY(double angle)
    {
        return rotateY(Math.cos(angle), Math.sin(angle));
    }

    /**
     * Rotate the plane around the y-axis in the xz-plane.
     * @param cosAngle cosine of the rotation angle
     * @param sinAngle sine of the rotation angle
     *        (positive rotates counterclockwise)
     * @return rotated "this"
     */
    public Plane rotateY(double cosAngle, double sinAngle)
    {
        p().rotateY(cosAngle, sinAngle);
        n0().rotateY(cosAngle, sinAngle);
        n1().rotateY(cosAngle, sinAngle);
        updateM = updateIM = true;
        return this;
    }

    /**
     * Rotate the plane around the z-axis in the xy-plane.
     * @param angle rotation angle (rad), positive rotates counterclockwise
     * @return rotated "this"
     */
    public Plane rotateZ(double angle)
    {
        return rotateZ(Math.cos(angle), Math.sin(angle));
    }

    /**
     * Rotate the plane around the z-axis in the xy-plane.
     * @param cosAngle cosine of the rotation angle
     * @param sinAngle sine of the rotation angle
     *        (positive rotates counterclockwise)
     * @return rotated "this"
     */
    public Plane rotateZ(double cosAngle, double sinAngle)
    {
        p().rotateZ(cosAngle, sinAngle);
        n0().rotateZ(cosAngle, sinAngle);
        n1().rotateZ(cosAngle, sinAngle);
        updateM = updateIM = true;
        return this;
    }

    /**
     * Translate the plane by the given vector.
     * @param v translation vector
     * @return translated "this"
     */
    public Plane translate(Point3D v)
    {
        p().addL(v);
        return this;
    }

    //--------------------------------------------------------------------------

    /**
     * Compute the projection matrix for this plane into its cache.
     * @return projection matrix (stored in cache of "this")
     */
    private Matrix getM()
    {
        if (updateM)
        {
            M.set(0, 0, n0.iprod(n0)); M.set(0, 1, n0.iprod(n1));
            M.set(1, 0, M.get(0, 1)); M.set(1, 1, n1.iprod(n1));
            M.inv2x2L();
            updateM = false;
        }
        return M;
    }
    
    /**
     * @return the orthogonally projected point to this plane
     *         (placed into "result")
     */
    public Point3D projection(Point3D p, Point3D result)
    {
        p.sub(p(), result);
        double b0 = result.iprod(n0),
               b1 = result.iprod(n1);
        Matrix M = getM();
        double t = M.get(0,0)*b0 + M.get(0,1)*b1,
               s = M.get(1,0)*b0 + M.get(1,1)*b1;
        return eval(t, s, result);
    }
    
    /**
     * @return the orthogonally projected point to this plane
     *         (placed into a new point)
     */
    public Point3D projection(Point3D p)
    {
        return projection(p, new Point3D());
    }

    /**
     * @return the orthogonally projected point to this plane
     *         (placed into "p")
     */
    public Point3D projectionR(Point3D p)
    {
        return projection(p, p);
    }
    
    //--------------------------------------------------------------------------
    
    private Matrix getIM()
    {
        if (updateIM)
        {
            IM.set(0, 1, n0().x()); IM.set(0, 2, n1().x());
            IM.set(1, 1, n0().y()); IM.set(1, 2, n1().y());
            IM.set(2, 1, n0().z()); IM.set(2, 2, n1().z());
            updateIM = false;
        }
        return IM;
    }
    
    /**
     * @return the intersection point with the given line
     *         or null if the line is parallel to this plane
     *         (placed into a new point)
     */
    public Point3D intersection(Line line)
    {
        return intersection(line, new Point3D());
    }
    
    /**
     * @return the intersection point with the given line
     *         or null if the line is parallel to this plane
     *         or the line is in the plane
     *         (placed into "result")
     */
    public Point3D intersection(Line line, Point3D result)
    {
        Point3D u = line.u();
        Matrix IM = getIM();
        IM.set(0, 0, -u.x()); IM.set(1, 0, -u.y()); IM.set(2, 0, -u.z());
        if (IM.inv3x3(IMr) == null) return null;
        line.p().sub(p(), result);
        IMr.mul(result, Ip);
        return line.eval(Ip.x(), result);
    }

    //--------------------------------------------------------------------------

    @Override
    public String toString()
    {
        return "plane{" + p() + ":" + n0() + "-" + n1() + "}";
    }

    //--------------------------------------------------------------------------

    private final Point3D p, n0, n1;
    private Matrix M; // projection matrix
    private boolean updateM, updateIM;
    
    private Point3D Ip; // intersection helper vector
    private Matrix IM, IMr; // intersection helper matrices
}
