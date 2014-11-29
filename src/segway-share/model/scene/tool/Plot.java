package model.scene.tool;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.LinkedList;

import model.scene.Color;

/**
 * A plot window on the top of the whole scene view.
 * 
 * Parent window coordinate references:
 * 
 *      (0,1)----------(1,1)
 *        |              |
 *        |              |
 *        |              |
 *      (0,0)----------(1,0)
 *      
 * The coordinates are automatically scaled by
 * the width and height of the parent window.
 */
public final class Plot extends SceneTool
{
    public Plot(double xRange, int yDim, double yMin, double yMax)
    {
        this.xRange = xRange;
        this.yDim = yDim;
        this.yMin = yMin;
        this.yMax = yMax;
        setWindowXY(Placement.BOTTOM_RIGHT);
    }

    public double xRange() { return xRange; }
    public synchronized void setXRange(double xRange) { this.xRange = xRange; }
    
    public int yDim() { return yDim; }
    
    public double yMin() { return yMin; }
    public synchronized void setYMin(double yMin) { this.yMin = yMin; }
    
    public double yMax() { return yMax; }
    public synchronized void setYMax(double yMax) { this.yMax = yMax; }
    
    public double xTic() { return xTic; }
    public synchronized void setXTic(double xTic) { this.xTic = xTic; }
    
    public double yTic() { return yTic; }
    public synchronized void setYTic(double yTic) { this.yTic = yTic; }
    
    public float lineWidth() { return lineWidth; }
    public synchronized void setLineWidth(float lineWidth)
    { this.lineWidth = lineWidth; }

    public float border() { return border; }
    public synchronized void setBorder(float border)
    { this.border = border; }
    
    public float ticWidth() { return ticWidth; }
    public synchronized void setTicWidth(float ticWidth)
    { this.ticWidth = ticWidth; }
    
    public boolean isTicGrid() { return isTicGrid; }
    public synchronized void setTicGrid(boolean isTicGrid)
    { this.isTicGrid = isTicGrid; }
    
    //--------------------------------------------------------------------------
    
    public Color axisColor() { return axisColor; }
    public synchronized void setAxisColor(Color color)
    { axisColor = color; }
    
    public Color backgroundColor() { return backgroundColor; }
    public synchronized void setBackgroundColor(Color color)
    { backgroundColor = color; }
    
    public Color ticGridColor() { return ticGridColor; }
    public synchronized void setTicGridColor(Color color)
    { ticGridColor = color; }
    
    public int numOfYColors() { return yColors.length; }
    public Color yColor(int i) { return yColors[i]; }
    public synchronized void setYColor(int i, Color c) { yColors[i] = c; }
    public synchronized void setAllYColors(Color[] colors) { yColors = colors; }
    
    //--------------------------------------------------------------------------

    public static class Data
    {
        public Data(double x, double[] y)
        {
            this.y = new double[y.length];
            set(x, y);
        }
        
        public double x() { return x; }
        public double[] y() { return y; }
        
        private void set(double x, double[] y)
        {
            this.x = x;
            copy(y, this.y);
        }
        
        private double x;
        private final double[] y;
    }

    public LinkedList<Data> dataList() { return dataList; }
    public synchronized void add(double x, double[] y)
    throws IOException
    {
        assert (yDim <= y.length);
        
        Data d = null, tmp;
        double minX = x-xRange;
        while (!dataList.isEmpty() && dataList.peekFirst().x() < minX)
        {
            tmp = dataList.removeFirst();
            // reuse the removed data element if possible
            if (d == null && tmp.y.length == y.length) d = tmp;
        }
        if (d == null) d = new Data(x,y); d.set(x,y);
        dataList.addLast(d);
        
        if (log != null)
        {
            log.write("" + x);
            for (int i = 0; i < y.length; ++i) log.write(" " + y[i]);
            log.write(newLine);
            log.flush();
        }
    }
    
    //--------------------------------------------------------------------------
    
    public void createLogFile(File logFile, String xName, String[] yNames)
    throws IOException
    {
        if (log != null) return;
        log = new BufferedWriter(new FileWriter(logFile));
        
        if (xName != null && yNames != null)
        {
            log.write("# " + xName);
            for (String yName : yNames) log.write(" " + yName);
            log.write(newLine);
            log.flush();
        }
    }
    
    public synchronized void closeLogFile()
    throws IOException
    {
        if (log == null) return;
        try
        {
            log.flush();
            log.close();
        }
        finally { log = null; }
    }
    
    public String newLine() { return newLine; }
    public synchronized void setNewLine(String nl) { newLine = nl; }

    //--------------------------------------------------------------------------
    
    public enum Placement { TOP_LEFT, TOP_CENTER, TOP_RIGHT,
                            CENTER_LEFT, CENTER, CENTER_RIGHT,
                            BOTTOM_LEFT, BOTTOM_CENTER, BOTTOM_RIGHT }
    
    public float windowSizeX() { return windowSizeX; }
    public synchronized void setWindowSizeX(float wsx) { windowSizeX = wsx; }
    
    public float windowSizeY() { return windowSizeY; }
    public synchronized void setWindowSizeY(float wsy) { windowSizeY = wsy; }

    public float windowX() { return windowX; }
    public float windowY() { return windowY; }
    public synchronized void setWindowXY(float x, float y)
    {
        windowX = x;
        windowY = y;
    }
    public synchronized void setWindowXY(Placement placement)
    {
        final float OFFSET = 0.01f;
        switch (placement)
        {
            case TOP_LEFT :
                windowX = OFFSET;
                windowY = 1f-(windowSizeY+OFFSET);
                break;
            case TOP_CENTER :
                windowX = (1f-windowSizeX)/2f;
                windowY = 1f-(windowSizeY+OFFSET);
                break;
            case TOP_RIGHT :
                windowX = 1f-(windowSizeX+OFFSET);
                windowY = 1f-(windowSizeY+OFFSET);
                break;
            case CENTER_LEFT :
                windowX = OFFSET;
                windowY = (1f-windowSizeY)/2f;
                break;
            case CENTER :
                windowX = (1f-windowSizeX)/2f;
                windowY = (1f-windowSizeY)/2f;
                break;
            case CENTER_RIGHT :
                windowX = 1f-(windowSizeX+OFFSET);
                windowY = (1f-windowSizeY)/2f;
                break;
            case BOTTOM_LEFT :
                windowX = OFFSET;
                windowY = OFFSET;
                break;
            case BOTTOM_CENTER :
                windowX = (1f-windowSizeX)/2f;
                windowY = OFFSET;
                break;
            case BOTTOM_RIGHT :
                windowX = 1f-(windowSizeX+OFFSET);
                windowY = OFFSET;
                break;
        }
    }
    
    public void reDrawAll() { isReDrawAll = true; }
    
    //--------------------------------------------------------------------------
    // Internal use only!
    
    public boolean isReDrawAll() { return isReDrawAll; }
    public synchronized void clearReDrawAllFlag() { isReDrawAll = false; }
    
    private static void copy(double[] from, double[] to)
    {
        assert (from.length == to.length);
        for (int i = 0; i < to.length; ++i) to[i] = from[i];
    }
    
    //--------------------------------------------------------------------------
    
    private final int yDim;
    private final LinkedList<Data> dataList = new LinkedList<Data>();
    private boolean isReDrawAll = true;

    private double xRange;
    private double yMin, yMax;
    private double xTic = 1, yTic = 1;
    
    private float lineWidth = 1f;
    private float border = 0.0075f;
    private float ticWidth = 0.0075f;
    private boolean isTicGrid = false;
    
    private Color axisColor = new Color(200f/255f, 200f/255f, 200f/255f);
    private Color backgroundColor = new Color(0f, 0f, 0f, 0.75f);
    private Color ticGridColor = new Color(100f/255f, 100f/255f, 100f/255f);
    private Color[] yColors = new Color[]{
                              new Color(210f/255f, 100f/255f, 90f/255f),
                              new Color(50f/255f, 200f/255f, 100f/255f),
                              new Color(100f/255f, 100f/255f, 225f/255f),
                              new Color(200f/255f, 210f/255f, 90f/255f),
                              new Color(165f/255f, 90f/255f, 215f/255f),
                              new Color(30f/255f, 190f/255f, 200f/255f),
                              new Color(220f/255f, 170f/255f, 80f/255f),
                              new Color(230f/255f, 125f/255f, 210f/255f) };                              
    
    private String newLine = "\n";
    private BufferedWriter log = null;
    
    private float windowX, windowY;
    private float windowSizeX = 0.25f, windowSizeY = 0.15f;
}
