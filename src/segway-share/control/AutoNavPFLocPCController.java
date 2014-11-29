package control;

import geom3d.Point3D;

import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.io.File;
import java.util.Set;

import javax.swing.event.MouseInputAdapter;

import localize.Particle;
import localize.ParticleFilterAlg;
import localize.ParticleFilterAlgKLD;

import model.motion.MotionConfig;
import model.scene.SceneModel;
import model.scene.tool.Plot;
import model.scene.tool.Text;
import model.scene.tool.Plot.Placement;
import run.PC;
import run.ThreadLogic;
import visual.View;

import comm.CommunicatorLogic;

import control.PCController;

/**
 * An autonomously navigating controller
 * based on a particle filtering localization.
 */
public final class AutoNavPFLocPCController extends PCController
{
    /** Bluetooth name of the NXT robot. */
    public static final String ROBOT_NAME = "harry";
    
    /** Delay between two visualization steps. */
    public static final int VISUAL_DELAY = 50;
    
    /** Create the used particle filter algorithm. */
    public static ParticleFilterAlg CREATE_PARTICLE_FILTER(long seed,
                                                           MotionConfig mcfg,
                                                           SceneModel scene)
    { return new ParticleFilterAlgKLD(seed, mcfg, scene); }
    
    //--------------------------------------------------------------------------
    
    public AutoNavPFLocPCController(PC pc,
                                    MotionConfig motionCfg,
                                    SceneModel scene,
                                    View view)
    throws Exception
    {
        super (pc, motionCfg, scene, view);
        mouseListener = new MouseListenerImpl();
        commLogic = new CommunicatorLogicImpl();
        dist = null;
        
        // remoteParticle = new Particle(scene.distCfg().length); // enabled
        pf = CREATE_PARTICLE_FILTER(19, motionCfg, scene);
        scene.addParticleFilter(pf);
        // You can disable particle cloud visualization by
        // scene.particleCloud().setEnabled(false);
        
        irPlotData = new double[2];
        irPlot = new Plot[scene.distCfg().length];
        for (int i = 0; i < irPlot.length; ++i)
        {
            irPlot[i] = new Plot(5, irPlotData.length,
                                 0, scene().distCfg()[i].maxValue()+1);
            irPlot[i].setYTic(100);
            //irPlot[i].createLogFile(new File("ir" + i + ".log"),
            //                        "time(sec)",
            //                        new String[]{"ir", "particle-dist"});
            scene.addTool(irPlot[i]);
        }
        if (0 < irPlot.length) irPlot[0].setWindowXY(Placement.BOTTOM_LEFT);
        if (1 < irPlot.length) irPlot[1].setWindowXY(Placement.BOTTOM_RIGHT);
        if (2 < irPlot.length) irPlot[2].setWindowXY(Placement.BOTTOM_CENTER);
        
        pitchPlotData = new double[1];
        pitchPlot = new Plot(5, pitchPlotData.length, -4, 4);
        pitchPlot.setYTic(1);
        pitchPlot.setTicGrid(true);
        //scene.addTool(pitchPlot);
        pitchPlot.setWindowXY(Placement.TOP_LEFT);
        
        timeText = new Text(10f, -22f);
        scene.addTool(timeText);
        
        cloudSizeText = new Text(10f, -44f);
        scene.addTool(cloudSizeText);
    }
    
    @Override
    public void initialize() throws Exception
    {
        super.initialize();
        isInit = true;
        
        pc().spawn("visual-updater", new VisualUpdater()).start();
        pc().createCommunicator(ROBOT_NAME, commLogic);
        
        dist = null;
        tStart = pc().currentTimeMillis();
    }
    
    @Override
    public void control() throws Exception
    {
        double pitch = 0.0;
        int dMrcL = 0, dMrcR = 0;
        boolean isNewData = false;
        synchronized (commLogic)
        {
            isNewData = commLogic.isNewData;
            if (isNewData)
            {
                pitch = commLogic.pitch;
                dMrcL = commLogic.dMrcL;
                dMrcR = commLogic.dMrcR;
                commLogic.dMrcL = commLogic.dMrcR = 0;
     
                if (dist == null) dist = new int[commLogic.dist.length];
                for (int i = 0; i < dist.length; ++i)
                    dist[i] = commLogic.dist[i];
            }
        }
        
        long t = pc().currentTimeMillis();
        if (isNewData)
        {
            double time = t/1000.0;
            timeText.setText("t: " + formatElapsedTime((int)(t-tStart)/1000));
            
            // regulate IR readings
            {
                int maxValue;
                for (int i = 0; i < dist.length; ++i)
                {
                    maxValue = (int)scene().distCfg()[i].maxValue();
                    if (dist[i] > maxValue) dist[i] = maxValue;
                }
            }
            
            if (isInit)
            {
                isInit = false;
                cloudSizeText.setText("Searching...");
                pf.init(pitch, dist);
            }
            else pf.track(pitch, dMrcL, dMrcR, dist);
            
            cloudSizeText.setText("N: " + pf.particles().size());
            Particle estP = pf.estimate();
            
            for (int i = 0; i < irPlot.length; ++i)
            {
                irPlotData[0] = dist[i];
                irPlotData[1] = estP.distance()[i];
                irPlot[i].add(time, irPlotData);
            }
            /*
            pitchPlotData[0] = pitch[1]; //(pitch[0] + pitch[1] + pitch[2])/3.0;
            pitchPlot.add(time, pitchPlotData);
            */
            
            if (!pc().isSimulated())
            {
                scene().update(time,
                               estP.x(),
                               estP.y(),
                               estP.pitch(),
                               pf.yaw(estP),
                               0.0, 0.0);
            }
        }
        int delay = 100 - (int)(pc().currentTimeMillis()-t);
        if (0 < delay) pc().msDelay(delay);
    }
    
    //--------------------------------------------------------------------------
    
    private class CommunicatorLogicImpl extends CommunicatorLogic
    {
        public CommunicatorLogicImpl() { isNewData = false; }
        
        @Override
        public void initalize() throws Exception
        {
            super.initalize();
            maxKeyCodes = channel().readByte();
            int len = channel().readByte();
            
            synchronized (this)
            {
                dist = new short[len];
                distTmp = new short[len];
                pitch = 0.0;
                dMrcL = dMrcR = 0;
            }
            isControlRound = true;
        }
        
        @Override
        public void logic() throws Exception
        {
            if (isControlRound)
            {
                Set<Short> activeKeys = activeKeys();
                synchronized (activeKeys)
                {
                    if (activeKeys.contains(KEY_ESCAPE))
                    {
                        channel().writeByte((byte)-1);
                        terminate();
                    }
                    else
                    {
                        byte size = (byte) activeKeys.size();
                        if (size > maxKeyCodes) size = maxKeyCodes;
                        
                        channel().writeByte(size);
                        for (short keycode : activeKeys)
                        {
                            channel().writeShort(keycode);
                            if (--size == 0) break;
                        }
                    }
                }
                channel().flush();                
            }
            else
            {
                channel().writeByte((byte)-2);
                channel().flush();
                
                double pitch = 0.01 * channel().readShort();
                short dL = channel().readShort();
                short dR = channel().readShort();
                for (int i = 0; i < distTmp.length; ++i)
                    distTmp[i] = channel().readShort();
                
                synchronized (this)
                {
                    this.pitch = pitch;
                    dMrcL += dL;
                    dMrcR += dR;
                    for (int i = 0; i < dist.length; ++i)
                        dist[i] = distTmp[i];
                    isNewData = true;
                }
            }
            isControlRound = !isControlRound;
            msDelay(50);
        }
        
        private short[] dist, distTmp;
        private int dMrcL, dMrcR;
        private double pitch;
        private boolean isControlRound, isNewData;
        private byte maxKeyCodes;
        
        private final static short KEY_ESCAPE = (short)KeyEvent.VK_ESCAPE;
    }
    
    //--------------------------------------------------------------------------
    
    private class VisualUpdater extends ThreadLogic
    {
        @Override
        public void run()
        {
            while (true)
            {
                if (!isPaused())
                {
                    if (pc().isSimulated())
                        scene().update(pc().simDynState());
                    view().updateCanvas();
                }
                msDelay(VISUAL_DELAY);
            }
        }
    }
    
    //--------------------------------------------------------------------------
    
    private class MouseListenerImpl extends MouseInputAdapter
    {
        @Override
        public void mouseClicked(MouseEvent e)
        {
            if (isPaused()) return;
            
            view().getPoint(e.getX(), e.getY(), p);
            if (scene().isOnFloor(p.x(), p.y()))
            {
                p.setZ(scene().isOnCarpet(p.x(), p.y())
                       ? (1+scene().carpet().height()) : 1);
                scene().selectedPoint().setPosition(p);
            }
            else scene().selectedPoint().setPosition(null);
            view().updateCanvas();
        }
        
        private Point3D p = new Point3D();
    }
    
    @Override
    public MouseListener mouseListener() { return mouseListener; }
    
    //--------------------------------------------------------------------------
    
    private class CustomKeyListener extends KeyListenerImpl
    {
        @Override
        public void keyPressed(KeyEvent e)
        {
            super.keyPressed(e);
            
            if (e.isControlDown())
            {
                switch (e.getKeyCode())
                {
                    case KeyEvent.VK_BACK_SPACE :
                        isInit = true;
                        break;
                }
            }
        }
    }
    
    @Override
    protected KeyListener createKeyListener()
    { return new CustomKeyListener(); }
    
    //--------------------------------------------------------------------------
    
    private boolean isInit;
    private int[] dist;
    
    private final double[] irPlotData;
    private final Plot[] irPlot;
    private final double[] pitchPlotData;
    private final Plot pitchPlot;
    
    private long tStart = 0;
    private final Text timeText;
    private final Text cloudSizeText;
    
    private final ParticleFilterAlg pf;
    private final CommunicatorLogicImpl commLogic;    
    private final MouseListenerImpl mouseListener;    
}
