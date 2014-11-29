package geom3d;

/**
 * Half-line (ray) starting from a point.
 */
public class HalfLine extends Line
{
    /**
     * Creates a half-line starting at p with orientation u.
     */
    public HalfLine(Point3D p, Point3D u)
    {
        super (p, u);
    }

    /**
     * Converts a line into a half-line.
     */
    public HalfLine(Line line)
    {
        this(line.p(), line.u());
    }

    /** @return copied half line (placed into a new object) */
    public HalfLine copy()
    {
        return new HalfLine(super.copy());
    }
    
    /** @return copied half line (placed into "result") */
    public HalfLine copy(HalfLine result)
    {
        super.copy(result);
        return result;
    }

    //--------------------------------------------------------------------------

    /** @return starting point of the half-line */
    public Point3D startPoint() { return p(); }

    /**
     * Evaluates the point on this half line at t.
     * If t is negative, <code>null</code> is returned.
     */
    @Override
    public Point3D eval(double t)
    {
        if (0.0 > t) { return null; }
        return super.eval(t);
    }

    //--------------------------------------------------------------------------
    
    @Override
    public HalfLine rotateX(double angle)
    {
        return rotateX(Math.cos(angle), Math.sin(angle)); 
    }
    
    @Override
    public HalfLine rotateX(double cosAngle, double sinAngle)
    {
        super.rotateX(cosAngle, sinAngle);
        return this;
    }
    
    @Override
    public HalfLine rotateY(double angle)
    {
        return rotateY(Math.cos(angle), Math.sin(angle));
    }
    
    @Override
    public HalfLine rotateY(double cosAngle, double sinAngle)
    {
        super.rotateY(cosAngle, sinAngle);
        return this;
    }
    
    @Override
    public HalfLine rotateZ(double angle)
    {
        return rotateZ(Math.cos(angle), Math.sin(angle));
    }
    
    @Override
    public HalfLine rotateZ(double cosAngle, double sinAngle)
    {
        super.rotateZ(cosAngle, sinAngle);
        return this;
    }
    
    @Override
    public HalfLine translate(Point3D v)
    {
        super.translate(v);
        return this;
    }
    
    //--------------------------------------------------------------------------

    @Override
    public String toString()
    {
        return "hline{" + p() + "->" + u() + "}";
    }
}
