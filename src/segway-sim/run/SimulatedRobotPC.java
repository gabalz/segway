package run;

import model.motion.State;
import model.sensor.DistanceSensorState;
import model.sensor.GyroSensor.GyroState;
import simulator.Simulator;

import comm.Communicator;
import comm.SimulatedChannel;
import comm.SimulatedCommunicator;

/**
 * Simulated thread implementing the shared features
 * of the Robot and PC interfaces. 
 */
public class SimulatedRobotPC extends SimulatedThread implements RobotPC
{
    public SimulatedRobotPC(Simulator sim,
                            SimulatedChannel channel,
                            String name,
                            ThreadLogic logic)
    {
        super (sim, name, logic);
        this.channel = channel;
        comm = null;
        
        dynState = sim.state().copy();
        gyroState = sim.gyro().state().copy(new GyroState());
        distState = new DistanceSensorState[sim.dist().length];
        for (int i = 0; i < distState.length; ++i)
            distState[i] = sim.dist()[i].state().copy(new DistanceSensorState());
    }
    
    //--------------------------------------------------------------------------
   
    @Override
    public Communicator comm() { return comm; }
    
    protected void setCommunicator(SimulatedCommunicator comm)
    { this.comm = comm; }
    
    protected SimulatedChannel channel() { return channel; }
    
    //--------------------------------------------------------------------------
    
    @Override
    public final boolean isSimulated() { return true; }
    
    @Override
    public final State simDynState()
    {
        synchronized (sim()) { sim().state().copy(dynState); }
        return dynState;
    }
    
    @Override
    public final GyroState simGyroState()
    {
        synchronized (sim()) { sim().gyro().state().copy(gyroState); }
        return gyroState;
    }
    
    @Override
    public final DistanceSensorState[] simDistState()
    {
        synchronized (sim())
        {
            for (int i = 0; i < distState.length; ++i)
            {
                sim().readDistance(i); // refresh the distance sensor state
                sim().dist()[i].state().copy(distState[i]);
            }
        }
        return distState;
    }
    
    //--------------------------------------------------------------------------
    
    private final SimulatedChannel channel;
    private SimulatedCommunicator comm;
    
    // These states do not have to be protected by synchronization
    // as a (robot/pc) controller runs exclusively with the simulator.
    
    private final State dynState;
    private final GyroState gyroState;
    private final DistanceSensorState[] distState;
}
