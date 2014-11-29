package control;

import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.awt.event.MouseListener;
import java.util.HashSet;
import java.util.Set;
import java.util.concurrent.atomic.AtomicBoolean;

import model.motion.MotionConfig;
import model.scene.SceneModel;
import run.PC;
import visual.View;

/**
 * Shared implementation of a PC controller.
 */
public abstract class PCController extends Controller
{
    public PCController(PC pc,
                        MotionConfig motionCfg,
                        SceneModel scene,
                        View view)
    {
        this.pc = pc;
        this.motionCfg = motionCfg;
        this.scene = scene;
        this.view = view;
        
        isPaused = new AtomicBoolean(false);
        activeKeys = new HashSet<Short>();
        keyListener = createKeyListener();
    }
    
    /** @return interface to the "controlled pc" */
    public PC pc() { return pc; }
    
    /** @return the configuration of the motion model */
    public MotionConfig motionCfg() { return motionCfg; }
    
    /** @return the model of the scene */
    public SceneModel scene() { return scene; }
    
    /** @return the visualization of the scene */
    public View view() { return view; }
    
    /** @return active key codes */
    public Set<Short> activeKeys() { return activeKeys; }
    
    /** @return key listener of the PC controller */
    public KeyListener keyListener() { return keyListener; }
    
    /** @return mouse listener of the PC controller */
    public MouseListener mouseListener() { return null; }
    
    /** @return true if the pause flag is on */
    public boolean isPaused() { return isPaused.get(); }
    
    //--------------------------------------------------------------------------
    
    /** @return formatted elapsed time string */
    public static final String formatElapsedTime(int seconds)
    {
        String str = "", space = "";
        int h = seconds/3600, m = seconds/60, s = seconds%60;
        if (0 < h) { str += space + h + "h"; space = " "; }
        if (0 < m) { str += space + m + "m"; space = " "; }
        str += space + s + "s";
        return str;
    }
    
    //--------------------------------------------------------------------------
    
    protected class KeyListenerImpl implements KeyListener
    {
        @Override
        public void keyPressed(KeyEvent e)
        {
            if (e.getKeyCode() == KeyEvent.VK_SPACE)
            {
                isPaused.set(!isPaused.get());
                pc().pauseOrResume();
                view.updateCanvas();
                return;
            }
            if (e.isControlDown()) return;
            
            synchronized (activeKeys)
            { activeKeys.add((short)e.getKeyCode()); }
        }
        
        @Override
        public void keyReleased(KeyEvent e)
        {
            synchronized (activeKeys)
            { activeKeys.remove((short)e.getKeyCode()); }
        }
        
        @Override
        public void keyTyped(KeyEvent e)
        {}
    }

    protected KeyListener createKeyListener()
    { return new KeyListenerImpl(); }
    
    //--------------------------------------------------------------------------
    
    private AtomicBoolean isPaused;

    private final PC pc;
    private final HashSet<Short> activeKeys;
    private final KeyListener keyListener;
    
    private final MotionConfig motionCfg;
    private final SceneModel scene;
    private final View view;
}
