package geom3d;

import vecmat.Matrix;

/**
 * A parallelogram spanned by point p and vectors u, v as
 * 
 *       p2
 *       /|
 *      / |
 *     /  |
 *  p1/   |
 *    |   |
 *    |   /p3
 *  u |  /
 *    | / v
 *    |/
 *    p = p0
 */
public class Parallelogram extends Plane
{
    /**
     * Define a rectangle between "p" and q = "p"+("u"+"v").
     */
    public Parallelogram(Point3D p, Point3D u, Point3D v)
    {
        super (p, u, v);
        
        this.u = u;
        this.v = v;

        points = new Point3D[4];
        points[0] = p.copy();
        points[1] = p.copy().addL(u);
        points[2] = points[1].copy().addL(v);
        points[3] = p.copy().addL(v);
        center = u.copy().addL(v).divL(2.0).addL(p);
        
        tmp = new Point3D();
        C = Matrix.create(2, 2);
        updateC = true;
    }
    
    public Point3D u() { return u; }
    public Point3D v() { return v; }

    /** @return all the 4 points */
    public Point3D[] points() { return points; }
    
    /** @return "i"th (0 <= i <= 3) point */
    public Point3D point(int i)
    {
        assert (0 <= i && i <= 3);
        return points[i];
    }
    
    /** @return center point = p + (u+v)/2 */
    public Point3D center() { return center; }
    
    //--------------------------------------------------------------------------
    
    @Override
    public Parallelogram rotateX(double angle)
    {
        return rotateX(Math.cos(angle), Math.sin(angle));
    }

    @Override
    public Parallelogram rotateX(double cosAngle, double sinAngle)
    {
        super.rotateX(cosAngle, sinAngle);
        u.rotateX(cosAngle, sinAngle);
        v.rotateX(cosAngle, sinAngle);
        for (Point3D p : points()) p.rotateX(cosAngle, sinAngle);
        center.rotateX(cosAngle, sinAngle);
        updateC = true;
        return this;
    }

    @Override
    public Parallelogram rotateY(double angle)
    {
        return rotateY(Math.cos(angle), Math.sin(angle));
    }

    @Override
    public Parallelogram rotateY(double cosAngle, double sinAngle)
    {
        super.rotateY(cosAngle, sinAngle);
        u.rotateY(cosAngle, sinAngle);
        v.rotateY(cosAngle, sinAngle);
        for (Point3D p : points()) p.rotateY(cosAngle, sinAngle);
        center.rotateY(cosAngle, sinAngle);
        updateC = true;
        return this;
    }

    @Override
    public Parallelogram rotateZ(double angle)
    {
        return rotateZ(Math.cos(angle), Math.sin(angle));
    }

    @Override
    public Parallelogram rotateZ(double cosAngle, double sinAngle)
    {
        super.rotateZ(cosAngle, sinAngle);
        u.rotateZ(cosAngle, sinAngle);
        v.rotateZ(cosAngle, sinAngle);
        for (Point3D p : points()) p.rotateZ(cosAngle, sinAngle);
        center.rotateZ(cosAngle, sinAngle);
        updateC = true;
        return this;
    }

    @Override
    public Parallelogram translate(Point3D v)
    {
        super.translate(v);
        for (Point3D p : points) p.addL(v);
        center.addL(v);
        return this;
    }
    
    //--------------------------------------------------------------------------
    
    private Matrix getC()
    {
        if (updateC)
        {
            C.set(0,0, u.iprod(u)); C.set(0,1, u.iprod(v));
            C.set(1,0, C.get(0,1)); C.set(1,1, v.iprod(v));
            C.inv2x2L();
            updateC = false;
        }
        return C;
    }
    
    @Override
    public Point3D intersection(Line line, Point3D result)
    {
        if (null == super.intersection(line, result)) return null;

        result.sub(p(), tmp);
        double b0 = tmp.iprod(u);
        double b1 = tmp.iprod(v);
        
        Matrix C = getC();
        double t = C.get(0,0)*b0 + C.get(0,1)*b1;
        if (t < 0.0 || t > 1.0) return null;
        
        double s = C.get(1,0)*b0 + C.get(1,1)*b1;
        if (s < 0.0 || s > 1.0) return null;
        
        return result;
    }
    
    /**
     * @return the intersection point with the given half line
     *         (placed into "result")
     *         or null if there is no such point
     */
    public Point3D intersection(HalfLine hline, Point3D result)
    {
        return null; // TODO
    }

    /**
     * @return the intersection point with the given half line
     *         (placed into a new point)
     *         or null if there is no such point
     */
    public Point3D intersection(HalfLine hline)
    {
        return intersection(hline, new Point3D());
    }
    
    //--------------------------------------------------------------------------
    
    private final Point3D[] points;
    private final Point3D u, v, center, tmp;
    private final Matrix C;
    private boolean updateC;
}
