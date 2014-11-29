package visual.engine;

import java.util.LinkedList;

import model.scene.Color;
import model.scene.SceneModel;
import model.scene.tool.CanvasXY;
import model.scene.tool.Plot;
import model.scene.tool.Text;
import model.scene.tool.Plot.Data;

import org.lwjgl.opengl.GL11;

/**
 * Visualization of scene model tools.
 */
final class EngineTools
{
    public EngineTools(EngineCanvas canvas)
    {
        this.canvas = canvas;
    }

    //--------------------------------------------------------------------------
    
    public void draw()
    {
        final SceneModel scene = canvas.scene();
        
        GL11.glDisable(GL11.GL_LIGHTING);
        
        for (CanvasXY canvasXY : scene.canvasXYTools())
            if (canvasXY.isVisible()) drawCanvasXY(canvasXY);
        
        boolean isOrthoView = false;
        for (Plot plot : scene.plotTools())
            if (plot.isVisible())
            {
                if (!isOrthoView)
                {
                    isOrthoView = true;
                    canvas.enableOrthoView();
                }
                drawPlot(plot);
            }
        for (Text text : scene.textTools())
            if (text.isVisible())
            {
                if (!isOrthoView)
                {
                    isOrthoView = true;
                    canvas.enableOrthoView();
                }
                drawText(text);
            }
        if (isOrthoView) canvas.disableOrthoView();
        
        GL11.glEnable(GL11.GL_LIGHTING);
    }
    
    //--------------------------------------------------------------------------
    
    private void drawPlot(Plot plot)
    {
        synchronized (plot)
        {
            final float canvasWidth = canvas.currentWidth();
            final float canvasHeight = canvas.currentHeight();
            
            final float windowX = canvasWidth * plot.windowX();
            final float windowY = canvasHeight * plot.windowY();
            final float windowSizeX = canvasWidth * plot.windowSizeX();
            final float windowSizeY = canvasHeight * plot.windowSizeY();        
    
            // compute border, axis coordinates
            final float border = plot.border();
            final float borderX = border * canvasWidth;
            final float borderY = border * canvasHeight;

            final float originX = windowX+borderX*2f;
            final float originY = windowY+borderY*2f;
            final float xAxisEnd = windowX+windowSizeX-borderX;
            final float yAxisEnd = windowY+windowSizeY-borderY;

            final double xStep = (xAxisEnd-originX) / plot.xRange();
            final double yMin = plot.yMin(), yMax = plot.yMax();
            final double yStep = (yAxisEnd-originY) / (yMax-yMin);
            final double zeroY = originY - yMin*yStep;
            
            // compute tic parameters
            final float yTicStep = (float)(plot.yTic()*yStep);
            final float ticWh = plot.ticWidth()*canvasWidth/2f;
            final float yTicStart = originX-ticWh,
                        yTicEnd = originX+ticWh;
            
            final LinkedList<Data> dataList = plot.dataList();
            final double xRange = plot.xRange();
            
            final double xStart = dataList.isEmpty() ? 0 :
                Math.max(0.0, dataList.getLast().x()-xRange);
            final double xTic = plot.xTic();
            final float xTicStep = (float)(xTic*xStep);
            final float xTicStart = originY-ticWh,
                        xTicEnd = originY+ticWh;

            // initialize plotting
            final float lineWidth = plot.lineWidth();
            if (lineWidth != 1f) GL11.glLineWidth(lineWidth);

            GL11.glScissor((int)windowX, (int)windowY,
                           (int)windowSizeX, (int)windowSizeY);
            GL11.glEnable(GL11.GL_SCISSOR_TEST);
            
            // clear background
            final Color bgC = plot.backgroundColor();
            GL11.glColor4f(bgC.red(), bgC.green(), bgC.blue(), bgC.alpha());
            GL11.glRectf(windowX, windowY,
                         windowX+windowSizeX, windowY+windowSizeY);
            
            // plot tic grid
            if (plot.isTicGrid() && !dataList.isEmpty())
            {
                final Color ticGridColor = plot.ticGridColor();
                GL11.glColor3f(ticGridColor.red(),
                               ticGridColor.green(),
                               ticGridColor.blue());
                
                GL11.glBegin(GL11.GL_LINES);
                {
                    float xTicX = originX+(float)((xTic-(xStart % xTic))*xStep);
                    while (xTicX <= xAxisEnd)
                    {
                        GL11.glVertex2f(xTicX, originY);
                        GL11.glVertex2f(xTicX, yAxisEnd);
                        xTicX += xTicStep;
                    }
                    
                    int i = 0;
                    double yTicY;
                    while (true)
                    {
                        yTicY = zeroY + (i++)*yTicStep;
                        if (yTicY > yAxisEnd) break;

                        GL11.glVertex2d(originX, yTicY);
                        GL11.glVertex2d(xAxisEnd, yTicY);
                    }

                    i = -1;
                    while (true)
                    {
                        yTicY = zeroY + (i--)*yTicStep;
                        if (yTicY <= originY) break;

                        GL11.glVertex2d(originX, yTicY);
                        GL11.glVertex2d(xAxisEnd, yTicY);
                    }
                }
                GL11.glEnd();
            }
            
            // plot x,y-axis
            final Color axisC = plot.axisColor();
            GL11.glColor3f(axisC.red(), axisC.green(), axisC.blue());

            // plot y tics
            GL11.glBegin(GL11.GL_LINES);
            {
                GL11.glVertex2f(originX, yAxisEnd);
                GL11.glVertex2f(originX, windowY+borderY);
                GL11.glVertex2f(windowX+borderX, originY);
                GL11.glVertex2f(xAxisEnd, originY);

                double yTicY = zeroY;
                GL11.glVertex2d(yTicStart-ticWh, yTicY);
                GL11.glVertex2d(yTicEnd+ticWh, yTicY);

                int i = 1;
                while (true)
                {
                    yTicY = zeroY + (i++)*yTicStep;
                    if (yTicY > yAxisEnd) break;

                    GL11.glVertex2d(yTicStart, yTicY);
                    GL11.glVertex2d(yTicEnd, yTicY);
                }

                i = -1;
                while (yTicY > originY)
                {
                    yTicY = zeroY + (i--)*yTicStep;
                    if (yTicY <= originY) break;

                    GL11.glVertex2d(yTicStart, yTicY);
                    GL11.glVertex2d(yTicEnd, yTicY);
                }
            }
            GL11.glEnd();

            if (!dataList.isEmpty())
            {
                // plot x tics
                GL11.glBegin(GL11.GL_LINES);
                {
                    float xTicX = originX+(float)((xTic-(xStart % xTic))*xStep);
                    while (xTicX <= xAxisEnd)
                    {
                        GL11.glVertex2f(xTicX, xTicStart);
                        GL11.glVertex2f(xTicX, xTicEnd);
                        xTicX += xTicStep;
                    }
                }
                GL11.glEnd();
                
                // plot data
                GL11.glScissor((int)originX, (int)originY,
                               (int)(xAxisEnd-originX)+1,
                               (int)(yAxisEnd-originY)+1);
                
                final int yDim = plot.yDim();
                final int yColorsN = plot.numOfYColors();
                for (int i = 0; i < yDim; ++i)
                {
                    Color c = plot.yColor(i < yColorsN ? i : i%yColorsN);
                    GL11.glColor4f(c.red(), c.green(), c.blue(), c.alpha());
                    GL11.glBegin(GL11.GL_LINE_STRIP);
                    {
                        for (Data data : dataList)
                            GL11.glVertex2d(originX+xStep*(data.x()-xStart),
                                            zeroY+yStep*data.y()[i]);
                    }
                    GL11.glEnd();
                }
            }
            
            GL11.glDisable(GL11.GL_SCISSOR_TEST);
            if (lineWidth != 1f) GL11.glLineWidth(1f);
        }
    }
    
    //--------------------------------------------------------------------------
    
    private void drawCanvasXY(CanvasXY canvasXY)
    {
        final float z = canvasXY.z();
        final float lineWidth = canvasXY.width();
        
        if (lineWidth != 1.0) GL11.glLineWidth(lineWidth);
                    
        GL11.glBegin(GL11.GL_LINE);
        for (CanvasXY.Line line : canvasXY.shownLines())
        {
            Color c = line.color();                
            GL11.glColor3d(c.red(), c.green(), c.blue());
            GL11.glVertex3f(line.x1(), line.y1(), z);
            GL11.glVertex3f(line.x2(), line.y2(), z);
        }
        GL11.glEnd();
        
        GL11.glBegin(GL11.GL_LINE_LOOP);
        for (CanvasXY.Rect rect : canvasXY.shownRects())
        {
            if (rect.isFilled()) continue;
            
            Color c = rect.color();
            GL11.glColor3d(c.red(), c.green(), c.blue());
            GL11.glVertex3f(rect.x1(), rect.y1(), z);
            GL11.glVertex3f(rect.x1(), rect.y2(), z);
            GL11.glVertex3f(rect.x2(), rect.y2(), z);
            GL11.glVertex3f(rect.x2(), rect.y1(), z);
        }
        GL11.glEnd();
        
        GL11.glPushMatrix();
        GL11.glTranslatef(0f, 0f, z);
        for (CanvasXY.Rect rect : canvasXY.shownRects())
        {
            if (!rect.isFilled()) continue;
            
            Color c = rect.color();
            GL11.glColor3d(c.red(), c.green(), c.blue());
            GL11.glRectf(rect.x1(), rect.y1(), rect.x2(), rect.y2());
        }
        GL11.glPopMatrix();
        
        if (lineWidth != 1.0) GL11.glLineWidth(1f);
    }
    
    //--------------------------------------------------------------------------
    
    private void drawText(Text text)
    {
        final EngineFont font = EngineFont.provideFont(text.font());
        final Color c = text.color();
        
        float x = text.x();
        if (x < 0f) x += canvas.currentWidth();
        float y = text.y();
        if (y < 0f) y += canvas.currentHeight();
        
        GL11.glPushMatrix();
        GL11.glColor4f(c.red(), c.green(), c.blue(), c.alpha());
        GL11.glTranslatef(x, y, 0f);
        font.glPrint(text.text());
        GL11.glPopMatrix();
    }
    
    //--------------------------------------------------------------------------
    
    private final EngineCanvas canvas;
}
