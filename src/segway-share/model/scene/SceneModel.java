package model.scene;

import geom2d.Point2D;
import geom2d.Polygon;
import geom3d.LineSegment;
import geom3d.Parallelogram;
import geom3d.Point3D;
import helper.Ratio;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

import localize.ParticleFilter;
import model.motion.MotionConfig;
import model.motion.State;
import model.scene.tool.CanvasXY;
import model.scene.tool.Plot;
import model.scene.tool.SceneTool;
import model.scene.tool.Text;
import model.sensor.DistanceSensorConfig;

/**
 * Geometric model of the scene.
 * 
 * Orientation:   +z|  /+y
 *                  | /
 *                  |/
 *                  o------
 *                       +x
 * 
 * Descartes coordinate system
 * x, y, z labels according to right-hand rule
 * 
 * The bottom left corner of the floor is placed at the origin.
 */
public final class SceneModel
{
    public SceneModel(MotionConfig motionModelConfig,
                      File mapFile,
                      DistanceSensorConfig[] distCfg)
    throws IOException
    {
        assert (mapFile != null);
        this.distCfg = distCfg;

        plotTools = new HashSet<Plot>();
        canvasXYTools = new HashSet<CanvasXY>();
        textTools = new HashSet<Text>();
        
        boxProj = new HashMap<Box, Polygon>();
        p2tmp = new Point2D();

        startTime = -1;
        objects = new LinkedList<SceneModelObject>();
        objectsToHit = new LinkedList<SceneModelObject>();
        boxes = new LinkedList<Box>();
        fixedPoints = new LinkedList<FixedPoint>();
        selectedPoint = new DynamicPoint(this);
        
        background = new Color(75, 155, 200);
        load(mapFile);
        
        if (motionModelConfig != null)
        {
            segway = new Segway(motionModelConfig.R * Ratio.M_TO_MM,
                                motionModelConfig.w * Ratio.M_TO_MM,
                                motionModelConfig.W * Ratio.M_TO_MM,
                                motionModelConfig.H * Ratio.M_TO_MM,
                                motionModelConfig.D * Ratio.M_TO_MM);
            leftWheel = new Wheel(segway, +1);
            rightWheel = new Wheel(segway, -1);
        }
        else { segway = null; leftWheel = rightWheel = null; }

        add(floor);
        add(walls);
        if (carpet != null) add(carpet);
        for (Box box : boxes) add(box);
        for (FixedPoint p : fixedPoints) add(p);
        add(selectedPoint);
        
        if (motionModelConfig != null)
        {
            add(segway);
            add(leftWheel);
            add(rightWheel);
        
            laserBeam = new LaserBeam[distCfg.length];
            laserBeamHitPoint = new LaserBeamHitPoint[distCfg.length];
            for (int i = 0; i < distCfg.length; ++i)
            {
                laserBeam[i] = new LaserBeam(distCfg[i], segway, this);
                add(laserBeam[i]);
                
                laserBeamHitPoint[i] = new LaserBeamHitPoint(laserBeam[i]);
                add(laserBeamHitPoint[i]);
            }
        }
        else { laserBeam = null; laserBeamHitPoint = null; }
        
        particleCloud = null;
    }

    //--------------------------------------------------------------------------
    
    /** @return distance sensor configurations */
    public DistanceSensorConfig[] distCfg() { return distCfg; }
    
    /** @return elapsed time since the first update (sec) */
    public double elapsedTime() { return elapsedTime; }
    
    public Color background() { return background; }
    public Floor floor() { return floor; }
    public Carpet carpet() { return carpet; }
    public List<Box> boxes() { return boxes; }
    public List<FixedPoint> fixedPoints() { return fixedPoints; }
    public DynamicPoint selectedPoint() { return selectedPoint; }
    public Segway segway() { return segway; }
    public Wheel leftWheel() { return leftWheel; }
    public Wheel rightWheel() { return rightWheel; }
    public LaserBeam[] laserBeam() { return laserBeam; }

    public synchronized void addParticleFilter(ParticleFilter pf)
    {
        assert (particleCloud == null);
        particleCloud = new ParticleCloud(pf);
        add(particleCloud);
    }
    public ParticleCloud particleCloud() { return particleCloud; }

    //--------------------------------------------------------------------------
    
    public List<SceneModelObject> objects() { return objects; }
    private void add(SceneModelObject obj)
    {
        objects.addLast(obj);
        if (obj.canBeHit()) objectsToHit.addLast(obj);
    }
    
    public Set<Plot> plotTools() { return plotTools; }
    public Set<CanvasXY> canvasXYTools() { return canvasXYTools; }
    public Set<Text> textTools() { return textTools; }
    public synchronized void addTool(SceneTool tool)
    {
        if (tool instanceof Plot) plotTools.add((Plot)tool);
        else if (tool instanceof CanvasXY) canvasXYTools.add((CanvasXY)tool);
        else if (tool instanceof Text) textTools.add((Text)tool);
        else throw new IllegalArgumentException(
                       "Unknown scene tool type: " + tool + "!");
    }
    public synchronized void removeTool(SceneTool tool)
    {
        if (tool instanceof Plot) plotTools.remove(tool); 
        else if (tool instanceof CanvasXY) canvasXYTools.remove(tool);
        else if (tool instanceof Text) textTools.remove(tool);
        else throw new IllegalArgumentException(
                       "Unknown scene tool type: " + tool + "!");
    }

    //--------------------------------------------------------------------------
    
    /** Set the state of the scene model from the provided motion state. */
    public void update(State state)
    {
        double pitch = state.pitch();
        update(state.time(),
               state.x() * Ratio.M_TO_MM,
               state.y() * Ratio.M_TO_MM,
               pitch,
               state.yaw(),
               state.leftRoll() - pitch,
               state.rightRoll() - pitch);
    }
    
    /**
     * Set the state of the scene model.
     * @param time time (sec)
     * @param x x position of the robot (mm)
     * @param y y position of the robot (mm)
     * @param pitch robot body pitch angle (rad)
     * @param yaw robot body yaw angle (rad)
     * @param lRoll left wheel rotation counter (rad)
     * @param rRoll right wheel rotation counter (rad)
     */
    public synchronized void update(double time,
                                    double x, double y,
                                    double pitch, double yaw,
                                    double lRoll, double rRoll)
    {
        if (startTime < 0) startTime = time;
        elapsedTime = time - startTime;
        
        Point3D position = segway.position();
        position.setX(x);
        position.setY(y);
        position.setZ(isOnCarpet(x,y) ? carpet.height() : 0);
        segway.update(position, pitch, yaw);
        leftWheel.update(lRoll + pitch);
        rightWheel.update(rRoll + pitch);
        
        for (int i = 0; i < laserBeam.length; ++i)
        {
            laserBeam[i].update();
            laserBeamHitPoint[i].update();
        }
    }

    //--------------------------------------------------------------------------
    
    public boolean isOnFloor(double x, double y)
    {
        return 0.0 <= x && x <= floor.width() &&
               0.0 <= y && y <= floor.height();
    }
    
    public boolean isOnCarpet(double x, double y)
    {
        return carpet != null &&
               carpet.xMin() <= x && x <= carpet.xMax() &&
               carpet.yMin() <= y && y <= carpet.yMax();
    }
    
    public boolean isUnderBox(double x, double y, Box box)
    {
        Polygon proj = boxProj.get(box);
        if (proj == null)
        {
            LinkedList<Point2D> points = new LinkedList<Point2D>();
            for (Parallelogram side : box.parallelepipedObjects()[0].sides())
                for (Point3D pt : side.points())
                    points.add(new Point2D(pt.x(), pt.y()));
            proj = new Polygon(points);
            boxProj.put(box, proj);
        }
        p2tmp.set(x, y);
        return proj.testInclusion(p2tmp);
    }
    
    public boolean isUnderAnyBox(double x, double y, Collection<Box> boxes)
    {
        for (Box box : boxes)
            if (isUnderBox(x, y, box)) return true;
        return false;        
    }
    
    //--------------------------------------------------------------------------
    
    /**
     * Representation of a distance measurement result.
     */
    public static final class DistanceResult
    {
        public DistanceResult()
        {
            hitPoint = new Point3D();
            hitPointTmp = new Point3D();
            rayTmp = new LineSegment(Point3D.origin(), Point3D.unitX(), 1.0);
        }
        
        /** @return traveled ray distance (mm) */
        public double distance() { return distance; }
        
        /** @return hit point in the scene (mm), (valid when isHit() is true) */
        public Point3D hitPoint() { return hitPoint; }
        
        /** @return hit object in the scene (null if nothing is hit) */
        public SceneModelObject hitObject() { return hitObject; }
        
        /** @return true if the ray hits an object in the scene */
        public boolean isHit() { return (hitObject != null); }
        
        public void set(double distance,
                        Point3D hitPoint,
                        SceneModelObject hitObject)
        {
            this.distance = distance;
            this.hitObject = hitObject;
            if (hitPoint != this.hitPoint && isHit())
                hitPoint.copy(this.hitPoint);
        }
        
        private double distance;
        private final Point3D hitPoint;
        private SceneModelObject hitObject;
        
        // cache objects for calculations (internal use only)
        private final Point3D hitPointTmp;
        private final LineSegment rayTmp;
    }

    /**
     * Calculate a distance sensor hit point and return the traveled distance
     * of a distance sensor ray, the hitting point and the hit object in the
     * scene given the location and the orientation of the segway.
     * @param distCfg distance sensor configuration 
     * @param robotPosition axle midpoint position of the robot (mm)
     * @param robotPitch robot body pitch (rad)
     * @param robotYaw robot body yaw (rad)
     * @param result measurement result
     * @return traveled distance (mm), hit point (mm), hit object
     *         (placed into "result")
     */
    public DistanceResult realDistance(DistanceSensorConfig distCfg,
                                       Point3D robotPosition,
                                       double robotPitch,
                                       double robotYaw,
                                       DistanceResult result)
    {
        return realDistance(distCfg.position(),
                            distCfg.orientation(),
                            distCfg.maxValue(),
                            robotPosition,
                            robotPitch,
                            robotYaw,
                            result);
    }
    
    /**
     * Calculate a distance sensor hit point and return the traveled distance
     * of a distance sensor ray, the hitting point and the hit object in the
     * scene given the location and the orientation of the segway. 
     * @param sensorPosition sensor position relative to the axle midpoint (mm)
     * @param sensorOrientation sensor orientation for the zero robotPitch case
     * @param sensorMaxValue range of the distance sensor (mm)
     * @param robotPosition axle midpoint position of the robot (mm)
     * @param robotPitch robot body pitch (rad)
     * @param robotYaw robot body yaw (rad)
     * @param result measurement result
     * @return traveled distance (mm), hit point (mm), hit object
     *         (placed into "result")
     */
    public DistanceResult realDistance(Point3D sensorPosition,
                                       Point3D sensorOrientation,
                                       double sensorMaxValue,
                                       Point3D robotPosition,
                                       double robotPitch,
                                       double robotYaw,
                                       DistanceResult result)
    {
        final Point3D hitPointTmp = result.hitPointTmp;
        final LineSegment rayTmp = result.rayTmp;
        rayTmp.setLength(sensorMaxValue);
        
        double distance = sensorMaxValue;
        Point3D hitPoint = result.hitPoint();
        SceneModelObject hitObject = null;
        
        sensorPosition.copy(rayTmp.p());
        sensorOrientation.copy(rayTmp.u());
        rayTmp.rotateY(robotPitch).rotateZ(robotYaw);
        rayTmp.translate(robotPosition);
        
        double d;
        for (SceneModelObject obj : objectsToHit)
        {
            d = obj.hitAt(rayTmp, hitPointTmp);
            if (d < distance)
            {
                distance = d;
                hitObject = obj;
                hitPointTmp.copy(hitPoint);
            }
        }
        result.set(distance, hitPoint, hitObject);
        return result;        
    }
    
    //--------------------------------------------------------------------------
    
    private final static String DELIM = " ";
    private final static String COMMENT = "#";
    
    /** Load the scene from a file. */
    public void load(File mapFile) throws IOException
    {
        BufferedReader br = new BufferedReader(new FileReader(mapFile));
        try
        {
            String line;
            while (null != (line = br.readLine()))
            {
                if (line.isEmpty()) continue;
                String[] tokens = line.split(DELIM);
                
                String type = tokens[0];
                if (type.startsWith(COMMENT)) continue;
                
                if (type.equalsIgnoreCase("table"))
                {
                    double sizeX = distance(tokens[1]);
                    double sizeY = distance(tokens[2]);
                    double height = distance(tokens[3]);
                    double thickness = distance(tokens[4]);
                    
                    floor = new Floor(sizeX, sizeY, thickness);
                    walls = new Walls(floor, height, thickness);
                }
                else if (type.equalsIgnoreCase("carpet"))
                {
                    double x = distance(tokens[1]);
                    double y = distance(tokens[2]);
                    double sizeX = distance(tokens[3]);
                    double sizeY = distance(tokens[4]);
                    double height = distance(tokens[5]);
                    
                    carpet = new Carpet(x, y, sizeX, sizeY, height);
                }
                else if (type.equalsIgnoreCase("box"))
                {
                    double x = distance(tokens[1]);
                    double y = distance(tokens[2]);
                    double sizeX = distance(tokens[3]);
                    double sizeY = distance(tokens[4]);
                    double height = distance(tokens[5]);
                    double elevation = distance(tokens[6]);
                    double pitch = angle(tokens[7]);
                    double yaw = angle(tokens[8]);
                    
                    boxes.add(new Box(new Point3D(x, y, elevation),
                                      sizeX, sizeY, height,
                                      pitch, yaw));
                }
                else if (type.equalsIgnoreCase("point"))
                {
                    double x = distance(tokens[1]);
                    double y = distance(tokens[2]);
                    double elevation = distance(tokens[3]);
                    
                    fixedPoints.add(new FixedPoint(new Point3D(x, y, elevation)));
                }
                else { System.err.println("Unknown type: " + type + "!"); }
            }
        }
        finally { br.close(); }
    }
    
    private double distance(String s)
    { return Double.valueOf(s); }
    
    private double angle(String s)
    { return Double.valueOf(s) * Ratio.DEG_TO_RAD; }
    
    //--------------------------------------------------------------------------
    
    private Color background;
    private Floor floor;
    private Walls walls;
    private Carpet carpet;
    private LinkedList<Box> boxes;
    private LinkedList<FixedPoint> fixedPoints;
    private DynamicPoint selectedPoint;
    
    private Segway segway;
    private Wheel leftWheel, rightWheel;
    
    private LaserBeam[] laserBeam;
    private LaserBeamHitPoint[] laserBeamHitPoint;
    private final DistanceSensorConfig[] distCfg;
    
    private ParticleCloud particleCloud;
    
    private final HashMap<Box, Polygon> boxProj;
    private final Point2D p2tmp;
    
    private double startTime, elapsedTime;
    private final LinkedList<SceneModelObject> objects;
    private final LinkedList<SceneModelObject> objectsToHit;
    
    private final HashSet<Plot> plotTools;
    private final HashSet<CanvasXY> canvasXYTools;
    private final HashSet<Text> textTools;
}
