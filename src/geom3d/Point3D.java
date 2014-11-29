package geom3d;

import java.util.Random;

import vecmat.Matrix;
import vecmat.Vector;

/**
 * Representation of a 3D point. 
 */
public class Point3D extends Vector
{
    public static Point3D origin() { return new Point3D(0.0, 0.0, 0.0); }
    public static Point3D unitX() { return new Point3D(1.0, 0.0, 0.0); }
    public static Point3D unitY() { return new Point3D(0.0, 1.0, 0.0); }
    public static Point3D unitZ() { return new Point3D(0.0, 0.0, 1.0); }

    //--------------------------------------------------------------------------

    public Point3D()
    {
        super (new double[]{0.0, 0.0, 0.0});
    }
    
    public Point3D(double x, double y, double z)
    {
        super (new double[]{x, y, z});
    }
    
    public Point3D(double[] xyz)
    {
        super (xyz);
        assert (xyz.length == 3);
    }

    @Override
    public Point3D copy()
    {
        return new Point3D(x(), y(), z());
    }

    public Point3D copy(Point3D result)
    {
        result.set(x(), y(), z());
        return result;
    }
    
    //--------------------------------------------------------------------------

    public double x() { return get(0); }
    public double y() { return get(1); }
    public double z() { return get(2); }
    
    public void setX(double x) { set(0, x); }
    public void setY(double y) { set(1, y); }
    public void setZ(double z) { set(2, z); }
    
    public void set(double x, double y, double z) { setX(x); setY(y); setZ(z); }

    //--------------------------------------------------------------------------

    /**
     * Rotate the point around the x-axis in the yz-plane.
     * @param angle rotation angle (rad), positive rotates counterclockwise
     * @return rotated "this"
     */
    public Point3D rotateX(double angle)
    {
        return rotateX(Math.cos(angle), Math.sin(angle));
    }

    /**
     * Rotate the point around the x-axis in the yz-plane.
     * @param cosAngle cosine of the rotation angle
     * @param sinAngle sine of the rotation angle
     *        (positive rotates counterclockwise)
     * @return rotated "this"
     */
    public Point3D rotateX(double cosAngle, double sinAngle)
    {
        double oldY = y(), oldZ = z();
        setY(oldY * cosAngle - oldZ * sinAngle);
        setZ(oldY * sinAngle + oldZ * cosAngle);
        return this;
    }

    /**
     * Rotate the point around the y-axis in the xz-plane.
     * @param angle rotation angle (rad), positive rotates counterclockwise
     * @return rotated "this"
     */
    public Point3D rotateY(double angle)
    {
        return rotateY(Math.cos(angle), Math.sin(angle));
    }

    /**
     * Rotate the point around the y-axis in the xz-plane.
     * @param cosAngle cosine of the rotation angle
     * @param sinAngle sine of the rotation angle
     *        (positive rotates counterclockwise)
     * @return rotated "this"
     */
    public Point3D rotateY(double cosAngle, double sinAngle)
    {
        double oldX = x(), oldZ = z();
        setZ(oldZ * cosAngle - oldX * sinAngle);
        setX(oldZ * sinAngle + oldX * cosAngle);
        return this;
    }

    /**
     * Rotate the point around the z-axis in the xy-plane.
     * @param angle rotation angle (rad), positive rotates counterclockwise
     * @return rotated "this"
     */
    public Point3D rotateZ(double angle)
    {
        return rotateZ(Math.cos(angle), Math.sin(angle));
    }

    /**
     * Rotate the point around the z-axis in the xy-plane.
     * @param cosAngle cosine of the rotation angle
     * @param sinAngle sine of the rotation angle
     *        (positive rotates counterclockwise)
     * @return rotated "this"
     */
    public Point3D rotateZ(double cosAngle, double sinAngle)
    {
        double oldX = x(), oldY = y();
        setX(oldX * cosAngle - oldY * sinAngle);
        setY(oldX * sinAngle + oldY * cosAngle);
        return this;
    }

    //--------------------------------------------------------------------------

    /**
     * @return true if the vectors specified by the given points
     *         are parallel to each other with "prec" precision
     */
    public boolean isParallel(Point3D p, double prec)
    {
        assert (0.0 < prec);
        double d = 0.0; // norm-1 of (a x b)
        d += Math.abs(y() * p.z() - z() * p.y());
        d += Math.abs(z() * p.x() - x() * p.z());
        d += Math.abs(x() * p.y() - y() * p.x());
        return d < prec;
    }
    
    /**
     * @return true if the vectors specified by the given points
     *         are parallel to each other with default precision
     */
    public boolean isParallel(Point3D p)
    {
        return isParallel(p, Constant.PREC);
    }

    //--------------------------------------------------------------------------
    
    /**
     * @return inner product between "this" and "p"
     */
    public double iprod(Point3D p)
    {
        return p.x()*x() + p.y()*y() + p.z()*z();
    }
    
    //--------------------------------------------------------------------------
    
    /**
     * @return cross product: "this" x "p" (placed into "result")
     */
    public Point3D xprod(Point3D p, Point3D result)
    {
        result.setX(y()*p.z() - z()*p.y());
        result.setY(z()*p.x() - x()*p.z());
        result.setZ(x()*p.y() - y()*p.x());
        return result;
    }
    
    /**
     * @return cross product: "this" x "p" (placed into a new point)
     */
    public Point3D xprod(Point3D p)
    {
        return xprod(p, new Point3D());
    }
    
    /**
     * @return squared 2-norm length of "this" x "p"
     */
    public double xprodLenSq(Point3D p)
    {
        double lx = y()*p.z() - z()*p.y();
        double ly = z()*p.x() - x()*p.z();
        double lz = x()*p.y() - y()*p.x();
        return lx*lx + ly*ly + lz*lz;
    }
    
    /**
     * @return 2-norm length of "this" x "p"
     */
    public double xprodLen(Point3D p)
    {
        return Math.sqrt(xprodLenSq(p));
    }
    
    //--------------------------------------------------------------------------
    
    /**
     * @return distance (1-norm) to point "p"
     */
    public double distance1(Point3D p)
    {
        return Math.abs(p.x()-x())
             + Math.abs(p.y()-y())
             + Math.abs(p.z()-z());
    }

    /**
     * @return square of the 2-norm distance to point "p"
     */
    public double distance2sq(Point3D p)
    {
        double dx = p.x()-x(), dy = p.y()-y(), dz = p.z()-z();
        return dx*dx + dy*dy + dz*dz;
    }

    /**
     * @return distance (2-norm) to point "p"
     */
    public double distance2(Point3D p)
    {
        return Math.sqrt(distance2sq(p));
    }
    
    /**
     * @return distance (inf-norm) to point "p"
     */
    public double distanceI(Point3D p)
    {
        return Math.max(Math.max(Math.abs(p.x()-x()), Math.abs(p.y()-y())),
                        Math.abs(p.z()-z()));
    }
    
    //--------------------------------------------------------------------------

    /**
     * @return true if the given point is equal to this one
     *         with "prec" precision
     */
    public boolean isEqual(Point3D p, double prec)
    {
        assert (0.0 < prec);
        return prec > Math.abs(p.x() - x())
            && prec > Math.abs(p.y() - y())
            && prec > Math.abs(p.z() - z());
    }

    /**
     * @return true if the given point is equal to this one
     *         with the default precision
     */
    public boolean isEqual(Point3D p)
    {
        return isEqual(p, Constant.PREC);
    }

    @Override
    public boolean equals(Object obj)
    {
        if (obj instanceof Point3D)
        {
            return isEqual((Point3D)obj);
        }
        return false;
    }

    //--------------------------------------------------------------------------

    @Override
    public String toString()
    {
        return "(" + x() + "," + y() + "," + z() + ")";
    }

    //--------------------------------------------------------------------------
    
    @Override
    public Point3D setToConstant(double c)
    { set(c, c, c); return this; }

    @Override
    public Point3D setToZero()
    { set(0.0, 0.0, 0.0); return this; }
    
    @Override
    public Point3D setToOne()
    { set(1.0, 1.0, 1.0); return this; }
    
    @Override
    public Point3D setToRand(Random rng)
    {
        set(rng.nextDouble(),
            rng.nextDouble(),
            rng.nextDouble());
        return this;
    }
    
    @Override
    public Point3D setToRandN(Random rng)
    {
        set(rng.nextGaussian(),
            rng.nextGaussian(),
            rng.nextGaussian());
        return this;
    }
    
    //--------------------------------------------------------------------------
    
    @Override
    public Point3D normalize1()
    { super.normalize1(); return this; }
    
    @Override
    public Point3D normalize2()
    { super.normalize2(); return this; }
    
    @Override
    public Point3D normalizeI()
    { super.normalizeI(); return this; }
    
    //--------------------------------------------------------------------------
    
    @Override
    public Point3D abs()
    { return abs(new Point3D()); }
    
    @Override
    public Point3D absL()
    { set(Math.abs(x()), Math.abs(y()), Math.abs(z())); return this; }
    
    //--------------------------------------------------------------------------
    
    @Override
    public Point3D sign(double zeroReplacement)
    { return sign(zeroReplacement, new Point3D()); }
    
    @Override
    public Point3D sign()
    { return sign(new Point3D()); }
    
    @Override
    public Point3D signL(double zeroReplacement)
    {
        set(sign(x(), zeroReplacement),
            sign(y(), zeroReplacement),
            sign(z(), zeroReplacement));
        return this;
    }
    
    @Override
    public Point3D signL()
    { super.signL(); return this; }
    
    //--------------------------------------------------------------------------
    
    @Override
    public Point3D neg()
    { return neg(new Point3D()); }
    
    @Override
    public Point3D negL()
    { set(-x(), -y(), -z()); return this; }
    
    //--------------------------------------------------------------------------
    
    @Override
    public Point3D add(Vector v)
    { return add(v, new Point3D()); }
        
    @Override
    public Point3D addL(Vector v)
    {
        assert (v.length() == 3);
        set(x() + v.get(0), y() + v.get(1), z() + v.get(2));
        return this;
    }
    
    @Override
    public Point3D add(double c)
    { return add(c, new Point3D()); }
    
    @Override
    public Point3D addL(double c)
    { set(x() + c, y() + c, z() + c); return this; }
    
    /** @return "this" + "p" (placed into "result") */
    public Point3D add(Point3D p, Point3D result)
    { result.set(x() + p.x(), y() + p.y(), z() + p.z()); return result; }
    
    /** @return "this" + "p" (placed into a new point) */
    public Point3D add(Point3D p)
    { return add(p, new Point3D()); }
    
    /** @return "this" + "p" (placed into "this") */
    public Point3D addL(Point3D p)
    { return add(p, this); }
    
    //--------------------------------------------------------------------------
    
    @Override
    public Point3D sub(Vector v)
    { return sub(v, new Point3D()); }
    
    @Override
    public Point3D subL(Vector v)
    {
        assert (v.length() == 3);
        set(x() - v.get(0), y() - v.get(1), z() - v.get(2));
        return this;
    }    
    
    @Override
    public Point3D sub(double c)
    { return sub(c, new Point3D()); }
    
    @Override
    public Point3D subL(double c)
    { set(x() - c, y() - c, z() - c); return this; }

    /** @return "this" - "p" (placed into "result") */
    public Point3D sub(Point3D p, Point3D result)
    { result.set(x() - p.x(), y() - p.y(), z() - p.z()); return result; }
    
    /** @return "this" - "p" (placed into a new point) */
    public Point3D sub(Point3D p)
    { return sub(p, new Point3D()); }
    
    /** @return "this" - "p" (placed into "this") */
    public Point3D subL(Point3D p)
    { return sub(p, this); }
    
    //--------------------------------------------------------------------------
    
    @Override
    public Point3D mul(double c)
    { return mul(c, new Point3D()); }
    
    @Override
    public Point3D mulL(double c)
    { set(x() * c, y() * c, z() * c); return this; }
    
    //--------------------------------------------------------------------------
    
    @Override
    public Point3D div(double c)
    { return div(c, new Point3D()); }
    
    @Override
    public Point3D divL(double c)
    { set(x() / c, y() / c, z() / c); return this; }
    
    //--------------------------------------------------------------------------
    
    @Override
    public Point3D mod(double m)
    { return mod(m, new Point3D()); }
    
    @Override
    public Point3D modL(double m)
    { set(x() % m, y() % m, z() % m); return this; }
    
    //--------------------------------------------------------------------------
    
    @Override
    public Point3D emul(Vector v)
    { return emul(v, new Point3D()); }
    
    @Override
    public Point3D emulL(Vector v)
    {
        assert (v.length() == 3);
        set(x() * v.get(0), y() * v.get(1), z() * v.get(2));
        return this;
    }
    
    /** @return "this" .* "p" (placed into "result") */
    public Point3D emul(Point3D p, Point3D result)
    { result.set(x() * p.x(), y() * p.y(), z() * p.z()); return result; }
    
    /** @return "this" .* p (placed into a new point) */
    public Point3D emul(Point3D p)
    { return emul(p, new Point3D()); }
    
    /** @return "this" .* p (placed into "this") */
    public Point3D emulL(Point3D p)
    { return emul(p, this); }
    
    //--------------------------------------------------------------------------
    
    @Override
    public Point3D mul(Matrix m)
    { super.mul(m); return this; }
    
    //--------------------------------------------------------------------------
    
    @Override
    public Point3D reciproc()
    { return reciproc(new Point3D()); }
    
    @Override
    public Point3D reciprocL()
    { set(1.0 / x(), 1.0 / y(), 1.0 / z()); return this; }
    
    //--------------------------------------------------------------------------
    
    @Override
    public double norm1()
    { return Math.abs(x()) + Math.abs(y()) + Math.abs(z()); }
    
    /** @return squared 2-norm */
    public double norm2sq()
    { return x()*x() + y()*y() + z()*z(); }
    
    @Override
    public double norm2()
    { return Math.sqrt(norm2sq()); }
    
    @Override
    public double normI()
    { return Math.max(Math.max(Math.abs(x()), Math.abs(y())), Math.abs(z())); }
}
