package geom2d;

import java.util.Collection;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.TreeMap;


/**
 * Representation of a 2D polygon.
 */
public class Polygon
{
    public Polygon(Point2D[] points)
    {
        this.points = points;
    }
    
    public Polygon(Collection<Point2D> points)
    {
        this (points, false);
    }
    
    public Polygon(Collection<Point2D> points, boolean copyPoints)
    {
        this.points = new Point2D[points.size()];
        
        int i = 0;
        for (Point2D p : points) this.points[i++] = copyPoints ? p.copy() : p;
    }
    
    //--------------------------------------------------------------------------
    
    public Point2D[] points() { return points; }
    public int size() { return points().length; }
    public Point2D point(int i)
    {
        assert (0 <= i && i < size());
        return points[i];
    }
    
    //--------------------------------------------------------------------------
    
    /**
     * @return a new polygon object with unique points
     *         (the points are not copied)
     */
    public Polygon removeRedundantPoints()
    {
        return removeRedundantPoints(false);
    }
    
    /**
     * @param copyPoints if true the points are copied, otherwise just referenced
     * @return a new polygon object with unique points
     */
    public Polygon removeRedundantPoints(boolean copyPoints)
    {
        HashSet<Point2D> s = new HashSet<Point2D>();
        for (Point2D p : points) if (!s.contains(p)) s.add(p);
        
        Point2D[] points = new Point2D[s.size()];
        {
            int i = 0;
            for (Point2D p : s) points[i++] = copyPoints ? p.copy() : p;
        }
        return new Polygon(points);
    }
    
    //--------------------------------------------------------------------------

    /**
     * Point inclusion in polygon test: PNPOLY algorithm
     * http://www.ecse.rpi.edu/Homepages/wrf/Research/Short_Notes/pnpoly.html
     * @return true if the polygon includes "p", false otherwise
     */
    public boolean testInclusion(Point2D p)
    {
        Point2D pi, pj;
        boolean c = false;
        for (int i = 0, j = points.length-1; i < points.length; j = i++)
        {
            pi = points[i];
            pj = points[j];
            if ( ((pi.y()>p.y()) != (pj.y()>p.y())) &&
                 (p.x() < (pj.x()-pi.x()) * (p.y()-pi.y())
                          / (pj.y()-pi.y()) + pi.x()) )
                c = !c;
        }
        return c;
    }
    
    //--------------------------------------------------------------------------

    /**
     * Convex hull computation: "Graham Scan" algorithm
     * http://softsurfer.com/Archive/algorithm_0109/algorithm_0109.htm
     * @return new polygon object of the convex hull (points are not copied)
     */
    public Polygon convexHull()
    {
        return convexHull(false);
    }
    
    /**
     * Convex hull computation: "Graham Scan" algorithm
     * http://softsurfer.com/Archive/algorithm_0109/algorithm_0109.htm
     * @param copyPoints if true the points are copied, otherwise just referenced
     * @return new polygon object of the convex hull
     */
    public Polygon convexHull(boolean copyPoints)
    {
        if (points.length <= 3) return removeRedundantPoints(copyPoints);
        
        // select the lowest rightmost point
        Point2D p0 = points[0];
        for (Point2D p : points)
            if (p.y() < p0.y() || (p.y() == p0.y() && p.x() > p0.x()))
                p0 = p;
        
        // order the rest of the points angularly about p0
        // (for ties, discard the closer points)
        TreeMap<Double, Point2D> ordered = new TreeMap<Double, Point2D>();
        for (Point2D p : points)
        {
            double d = p0.distance2(p);
            if (d < Constant.PREC) continue;
            double cosAngle = (p.x()-p0.x()) / d;
            
            double key = -cosAngle;
            Point2D pIn = ordered.get(key);
            if (pIn != null)
                if (d > p0.distance2(pIn)) ordered.put(key, p);
            else
                ordered.put(key, p);
        }
        
        LinkedList<Point2D> stack = new LinkedList<Point2D>();
        stack.addFirst(p0);
        stack.addFirst(ordered.pollFirstEntry().getValue());
        while (!ordered.isEmpty())
        {
            Point2D pt = ordered.firstEntry().getValue();
            
            Point2D p1 = stack.get(0);
            Point2D p2 = stack.get(1);
            
            if ( (pt.y()-p1.y())*(p2.x()-p1.x())
                    < (p2.y()-p1.y())*(pt.x()-p1.x()) )
                stack.addFirst(ordered.pollFirstEntry().getValue());
            else
                stack.pollFirst();
        }
        
        Point2D[] points = new Point2D[stack.size()];
        points = stack.toArray(points);
        return new Polygon(points);
    }
    
    //--------------------------------------------------------------------------
    
    private final Point2D[] points;
}
