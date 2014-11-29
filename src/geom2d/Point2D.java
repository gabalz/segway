package geom2d;

import vecmat.Vector;

/**
 * Representation of a 2D point.
 */
public final class Point2D extends Vector
{
    public static Point2D origin() { return new Point2D(0.0, 0.0); }
    public static Point2D unitX() { return new Point2D(1.0, 0.0); }
    public static Point2D unitY() { return new Point2D(0.0, 1.0); }
    
    //--------------------------------------------------------------------------
    
    public Point2D()
    {
        super (new double[]{0.0, 0.0});
    }
    
    public Point2D(double x, double y)
    {
        super (new double[]{x, y});
    }
    
    public Point2D(double[] xy)
    {
        super (xy);
        assert (xy.length == 2);
    }
    
    @Override
    public Point2D copy()
    {
        return new Point2D(x(), y());
    }
    
    public Point2D copy(Point2D result)
    {
        result.set(x(), y());
        return result;
    }
    
    //--------------------------------------------------------------------------
    
    public double x() { return get(0); }
    public double y() { return get(1); }

    public void setX(double x) { set(0, x); }
    public void setY(double y) { set(1, y); }

    public void set(double x, double y) { setX(x); setY(y); }
    
    //--------------------------------------------------------------------------
    
    /**
     * Rotate the point around the origin.
     * @param angle rotation angle (rad), positive rotates counterclockwise
     * @return rotated "this"
     */
    public Point2D rotate(double angle)
    {
        return rotate(Math.cos(angle), Math.sin(angle));
    }
    
    /**
     * Rotate the point around the origin.
     * @param cosAngle cosine of the rotation angle
     * @param sinAngle sine of the rotation angle
     *        (positive rotates counterclockwise)
     * @return rotated "this"
     */
    public Point2D rotate(double cosAngle, double sinAngle)
    {
        double oldX = x(), oldY = y();
        setX(oldX * cosAngle - oldY * sinAngle);
        setY(oldX * sinAngle + oldY * cosAngle);
        return this;
    }
    
    //--------------------------------------------------------------------------
    
    /**
     * @return true if the given point is equal to this one
     *         with "prec" precision
     */
    public boolean isEqual(Point2D p, double prec)
    {
        assert (0.0 < prec);
        return prec > Math.abs(p.x() - x())
            && prec > Math.abs(p.y() - y());
    }

    /**
     * @return true if the given point is equal to this one
     *         with the default precision
     */
    public boolean isEqual(Point2D p)
    {
        return isEqual(p, Constant.PREC);
    }

    @Override
    public boolean equals(Object obj)
    {
        if (obj instanceof Point2D)
        {
            return isEqual((Point2D)obj);
        }
        return false;
    }
    
    //--------------------------------------------------------------------------
    
    /**
     * @return distance (1-norm) to point "p"
     */
    public double distance1(Point2D p)
    {
        return Math.abs(p.x()-x())
             + Math.abs(p.y()-y());
    }
    
    /**
     * @return square of the 2-norm distance to point "p"
     */
    public double distance2sq(Point2D p)
    {
        double dx = p.x()-x(), dy = p.y()-y();
        return dx*dx + dy*dy;
    }
    
    /**
     * @return distance (2-norm) to point "p"
     */
    public double distance2(Point2D p)
    {
        return Math.sqrt(distance2sq(p));
    }
    
    /**
     * @return distance (inf-norm) to point "p"
     */
    public double distanceI(Point2D p)
    {
        return Math.max(Math.abs(p.x()-x()), Math.abs(p.y()-y()));
    }
}
