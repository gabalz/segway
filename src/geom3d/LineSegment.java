package geom3d;

/**
 * A line segment between two points.
 */
public class LineSegment extends HalfLine
{
    /**
     * Create a d long line segment starting at p with orientation u.
     */
    public LineSegment(Point3D p, Point3D u, double d)
    {
        super (p, u);
        length = d;
    }
    
    /** @return the length of the line segment */
    public double length() { return length; }
    
    /** Set the length of the line segment. */
    public void setLength(double length)
    { this.length = length; }
    
    //--------------------------------------------------------------------------
    
    /** @return copied line segment (placed into a new object) */
    public LineSegment copy()
    {
        return new LineSegment(p(), u(), length());
    }
    
    /** @return copied line segment (placed into "result") */
    public LineSegment copy(LineSegment result)
    {
        super.copy(result);
        return result;
    }
    
    //--------------------------------------------------------------------------
    
    @Override
    public LineSegment rotateX(double angle)
    {
        return rotateY(Math.cos(angle), Math.sin(angle));
    }
    
    @Override
    public LineSegment rotateX(double cosAngle, double sinAngle)
    {
        super.rotateX(cosAngle, sinAngle);
        return this;
    }
    
    @Override
    public LineSegment rotateY(double angle)
    {
        return rotateY(Math.cos(angle), Math.sin(angle));
    }
    
    @Override
    public LineSegment rotateY(double cosAngle, double sinAngle)
    {
        super.rotateY(cosAngle, sinAngle);
        return this;
    }
    
    @Override
    public LineSegment rotateZ(double angle)
    {
        return rotateZ(Math.cos(angle), Math.sin(angle));
    }
    
    @Override
    public LineSegment rotateZ(double cosAngle, double sinAngle)
    {
        super.rotateZ(cosAngle, sinAngle);
        return this;
    }
    
    @Override
    public LineSegment translate(Point3D v)
    {
        super.translate(v);
        return this;
    }
    
    //--------------------------------------------------------------------------
    
    @Override
    public String toString()
    {
        return "sline{" + p() + "->" + u() + ":" + length() + "}";
    }
    
    //--------------------------------------------------------------------------
    
    private double length;
}
