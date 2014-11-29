package model.scene.tool;

/**
 * Superclass of scene tool objects.
 */
public abstract class SceneTool
{
    /** @return true if the object is to be drawn */
    public boolean isVisible() { return isVisible; }

    /** Enable/disable the drawing of the object. */
    public synchronized void setVisible(boolean isVisible)
    { this.isVisible = isVisible; }
    
    //--------------------------------------------------------------------------
    
    private boolean isVisible = true;
}
