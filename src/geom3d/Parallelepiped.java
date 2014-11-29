package geom3d;



/**
 * A parallelepiped spanned by point p and vectors u, v, w as
 *  
 *       ------------
 *      /|         /| v
 *     / p--------/-/
 *    / /        / /
 *   / /        / /
 *  ------------ / w
 *  |/         |/
 *  ------------
 *       u
 * 
 * The normals of the sides are pointing out from the object
 * defining the visible faces of the sides.
 */
public class Parallelepiped
{
    /**
     * If the ratio of the bounding ball and parallelepiped surface areas
     * is below this threshold, the bounding ball is created to imporove
     * performance of computing intersections with line segments.
     */
    public static final double BOUNDING_BALL_SURF_AREA_RATIO = 2.5;
    
    public static enum BoundingBallUsage
    {
        IGNORE,             // do not use
        USE,                // do use
        SURF_RATIO_TEST     // use if passes the surface ratio test
    }
    
    //--------------------------------------------------------------------------

    /**
     * Define a parallelepiped between "p" and "p"+("u"+"v"+"w").
     */
    public Parallelepiped(Point3D p, Point3D u, Point3D v, Point3D w)
    {
        this (p, u, v, w, BoundingBallUsage.IGNORE);
    }
    
    /**
     * Define a parallelepiped between "p" and "p"+("u"+"v"+"w").
     * If useBoundingBall is true, a bounding ball is created and used to speed
     * up the intersection test with line segments if the parallelepiped passes
     * the surface test (see BOUNDING_BALL_SURF_AREA_RATIO).
     */
    public Parallelepiped(Point3D p, Point3D u, Point3D v, Point3D w,
                          BoundingBallUsage bbUsage)
    {        
        tmp = new Point3D();
        normals = new Point3D[6];
        sides = new Parallelogram[6];
        
        Point3D center = u.copy().addL(v).addL(w).divL(2.0);
        bbR = center.norm2();
        bbRsq = bbR * bbR;
        center.addL(p);
        
        createSide(0, p.add(w), u.copy(), v.copy(), center); 
        createSide(1, p.add(u), v.copy(), w.copy(), center);
        createSide(2, p.add(v), w.copy(), u.copy(), center);
        createSide(3, p.copy(), u.copy(), v.copy(), center);
        createSide(4, p.copy(), w.copy(), v.copy(), center);
        createSide(5, p.copy(), u.copy(), w.copy(), center);
        
        if (bbUsage != BoundingBallUsage.IGNORE)
        {
            boolean use = true;
            if (bbUsage == BoundingBallUsage.SURF_RATIO_TEST)
            {
                double ppSurfArea = 2.0 * (u.xprodLen(v) +
                                           u.xprodLen(w) +
                                           v.xprodLen(w));
                double bbSurfArea = 4.8 * Math.PI * bbRsq;
                use = (bbSurfArea / ppSurfArea < BOUNDING_BALL_SURF_AREA_RATIO);                
            }
            bbP = use ? center : null;
        }
        else bbP = null;        
    }
    
    private void createSide(int i,
                            Point3D p, Point3D u, Point3D v,
                            Point3D center)
    {
        // tmp = (u+v)/2 + p - center
        u.add(v, tmp).divL(2.0).addL(p).subL(center);
        
        normals[i] = u.xprod(v).normalize2();
        if (0.0 > normals[i].iprod(tmp))
        {
            normals[i].negL();
            sides[i] = new Parallelogram(p, v, u);
        }
        else sides[i] = new Parallelogram(p, u, v);
    }
    
    //--------------------------------------------------------------------------
    
    public Point3D p() { return sides[5].p(); }
    public Point3D u() { return sides[0].u(); }
    public Point3D v() { return sides[0].v(); }
    public Point3D w() { return sides[5].v(); }
    
    /** @return all the 6 sides */
    public Parallelogram[] sides() { return sides; }
    
    /** @return "i"th (0 <= i <= 5) side */
    public Parallelogram side(int i)
    {
        assert (0 <= i && i <= 5);
        return sides[i];
    }
    
    /** @return all the 6 normals */
    public Point3D[] normals() { return normals; }
    
    /** @return the normal of the "i"th (0 <= i <= 5) side */
    public Point3D normal(int i)
    {
        assert (0 <= i && i <= 5);
        return normals[i];
    }

    //--------------------------------------------------------------------------
    
    /**
     * Rotate the parallelepiped around the x-axis in the yz-plane.
     * @param angle rotation angle (rad), positive rotates counterclockwise
     * @return rotated "this"
     */
    public Parallelepiped rotateX(double angle)
    {
        return rotateX(Math.cos(angle), Math.sin(angle));
    }
    
    /**
     * Rotate the parallelepiped around the x-axis in the yz-plane.
     * @param cosAngle cosine of the rotation angle
     * @param sinAngle sine of the rotation angle
     *        (positive rotates counterclockwise)
     * @return rotated "this"
     */
    public Parallelepiped rotateX(double cosAngle, double sinAngle)
    {
        for (int i = 0; i < 6; ++i)
        {
            sides[i].rotateX(cosAngle, sinAngle);
            normals[i].rotateX(cosAngle, sinAngle);
        }
        if (bbP != null) bbP.rotateX(cosAngle, sinAngle);
        return this;
    }
    
    /**
     * Rotate the parallelepiped around the y-axis in the xz-plane.
     * @param angle rotation angle (rad), positive rotates counterclockwise
     * @return rotated "this"
     */
    public Parallelepiped rotateY(double angle)
    {
        return rotateY(Math.cos(angle), Math.sin(angle));
    }

    /**
     * Rotate the parallelepiped around the y-axis in the xz-plane.
     * @param cosAngle cosine of the rotation angle
     * @param sinAngle sine of the rotation angle
     *        (positive rotates counterclockwise)
     * @return rotated "this"
     */
    public Parallelepiped rotateY(double cosAngle, double sinAngle)
    {
        for (int i = 0; i < 6; ++i)
        {
            sides[i].rotateY(cosAngle, sinAngle);
            normals[i].rotateY(cosAngle, sinAngle);
        }
        if (bbP != null) bbP.rotateY(cosAngle, sinAngle);
        return this;
    }
    
    /**
     * Rotate the parallelepiped around the z-axis in the xy-plane.
     * @param angle rotation angle (rad), positive rotates counterclockwise
     * @return rotated "this"
     */
    public Parallelepiped rotateZ(double angle)
    {
        return rotateZ(Math.cos(angle), Math.sin(angle));
    }

    /**
     * Rotate the parallelepiped around the z-axis in the xy-plane.
     * @param cosAngle cosine of the rotation angle
     * @param sinAngle sine of the rotation angle
     *        (positive rotates counterclockwise)
     * @return rotated "this"
     */
    public Parallelepiped rotateZ(double cosAngle, double sinAngle)
    {
        for (int i = 0; i < 6; ++i)
        {
            sides[i].rotateZ(cosAngle, sinAngle);
            normals[i].rotateZ(cosAngle, sinAngle);
        }
        if (bbP != null) bbP.rotateZ(cosAngle, sinAngle);
        return this;
    }
    
    /**
     * Translate the parallelepiped by the given vector.
     * @param v translation vector
     * @return translated "this"
     */
    public Parallelepiped translate(Point3D v)
    {
        for (Parallelogram side : sides()) side.translate(v);
        if (bbP != null) bbP.addL(v);
        return this;
    }
    
    //--------------------------------------------------------------------------
    
    /**
     * @return squared distance between the start and the hit points
     *         (Double.POSITIVE_INFINITY if there is no such point)
     *         and
     *         the intersection point with the given half line
     *         which is the closest to the origin of the half line
     *         (placed into "result")
     */
    public double intersection(HalfLine hline, Point3D result)
    {
        final Point3D p = hline.startPoint(), u = hline.u();

        double dist;
        Point3D normal;
        Parallelogram side;
        int hitCount = 0;
        double x = 0.0, y = 0.0, z = 0.0, d = Double.POSITIVE_INFINITY;
        for (int i = 0; i < 6; ++i)
        {
            side = sides[i];
            normal = normals[i];
            if (normal.iprod(u) < 0.0 &&
                normal.iprod(side.center().sub(p, tmp)) < 0.0 &&
                null != side.intersection((Line)hline, result))
            {
                dist = p.distance2sq(result);
                if (dist < d)
                {
                    d = dist;
                    x = result.x(); y = result.y(); z = result.z();
                }
                if (++hitCount >= 2) break;
            }
        }
        
        if (!Double.isInfinite(d)) result.set(x, y, z);
        return d;
    }
    
    /**
     * @return squared distance between the start and the hit points
     *         (Double.POSITIVE_INFINITY if there is no such point)
     *         and
     *         the intersection point with the given line segment
     *         which is the closest to the origin of the line segment
     *         (placed into "result")
     */
    public double intersection(LineSegment sline, Point3D result)
    {
        double d;
        if (bbP != null)
        {
            bbP.sub(sline.startPoint(), result);
            d = sline.length() + bbR;
            double dd = result.norm2sq();
            if (dd > d*d) return Double.POSITIVE_INFINITY;
            
            double t = result.iprod(sline.u());
            if (t < 0 || dd - t*t > bbRsq) return Double.POSITIVE_INFINITY;
        }
        
        d = intersection((HalfLine)sline, result);
        if (d > sline.length()) d = sline.length();
        return d;
    }
    
    //--------------------------------------------------------------------------
    
    @Override
    public String toString()
    {
        return "brick{" + p() + ":" + u() + "," + v() + "," + w() + "}";
    }
    
    //--------------------------------------------------------------------------
    
    private final Point3D tmp;
    private final Parallelogram[] sides;
    private final Point3D[] normals;
    
    private final double bbR, bbRsq; // bounding ball squared radius
    private final Point3D bbP; // bounding ball center point
}
