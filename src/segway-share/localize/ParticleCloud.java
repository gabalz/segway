package localize;

import java.util.Vector;

/**
 * Type of particle clouds.
 */
public class ParticleCloud extends Vector<Particle>
{
    private int size;
    
    public ParticleCloud(int capacity)
    {
        super (capacity);
        super.setSize(capacity);
        size = capacity;
    }
    
    @Override
    public synchronized int size() { return size; }
    
    @Override
    public synchronized void setSize(int newSize) { size = newSize; }
    
    //--------------------------------------------------------------------------
    
    private static final long serialVersionUID = 1L;
}
