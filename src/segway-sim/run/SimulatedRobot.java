package run;

import simulator.Simulator;

import comm.CommunicatorLogic;
import comm.SimulatedChannel;
import comm.SimulatedCommunicator;

import control.RobotController;

/**
 * Simulated NXT Segway robot side.
 */
public final class SimulatedRobot extends SimulatedRobotPC implements Robot
{
    public SimulatedRobot(Simulator sim, SimulatedChannel channel)
    {
        super (sim, channel, "sim-nxt-robot", new SimulatedRobotLogic());
        SimulatedRobotLogic logic = (SimulatedRobotLogic)logicObject();
        logic.robot = this;
        
        leftPower = rightPower = 0;
        leftRotCtr = rightRotCtr = 0;
        
        distance = new int[sim.dist().length];
        for (int i = 0; i < distance.length; ++i) distance[i] = 0;
    }
    
    /** Set the robot controller. */
    public void setController(RobotController controller)
    {
        SimulatedRobotLogic logic = (SimulatedRobotLogic)logicObject();
        if (logic.controller != null)
            throw new IllegalStateException("Controller cannot be changed!");
        logic.controller = controller;
    }
    
    //--------------------------------------------------------------------------

    @Override
    public void resetLeftRotationCounter()
    {
        synchronized (mutexRotCtrL)
        { resetLeftRotCtr = leftRotationCounter(); }
    }
    
    @Override
    public void resetRightRotationCounter()
    {
        synchronized (mutexRotCtrR)
        { resetRightRotCtr = rightRotationCounter(); }
    }

    @Override
    public int leftRotationCounter()
    {
        synchronized (sim())
        { leftRotCtr = (int)(sim().readLeftRotationCounter()); }
        synchronized (mutexRotCtrL) { return leftRotCtr - resetLeftRotCtr; }
    }

    @Override
    public int rightRotationCounter()
    {
        synchronized (sim())
        { rightRotCtr = (int)(sim().readRightRotationCounter()); }
        synchronized (mutexRotCtrR) { return rightRotCtr - resetRightRotCtr; }
    }

    //--------------------------------------------------------------------------
    
    @Override
    public int readGyro()
    { synchronized (sim()) { return (int)(sim().readGyro()); } }

    @Override
    public int numDistances() { return distance.length; }
    
    @Override
    public int readDistance(int i)
    { synchronized (sim()) { return (int)(sim().readDistance(i)); } }
    
    //--------------------------------------------------------------------------
    
    /** @return applied power on the left motor (milliV) */
    public int leftPower()
    { synchronized (mutexPwrL) { return leftPower; } }
    
    /** @return applied power on the right motor (milliV) */
    public int rightPower()
    { synchronized (mutexPwrR) { return rightPower; } }
    
    @Override
    public void controlLeftMotor(int power)
    { synchronized (mutexPwrL) { leftPower = power; } }
    
    @Override
    public void controlRightMotor(int power)
    { synchronized (mutexPwrR) { rightPower = power; } }
    
    //--------------------------------------------------------------------------

    private static class SimulatedRobotLogic extends ThreadLogic
    {
        @Override
        public void run()
        {
            try
            {
                controller.initialize();
                robot.sim().standUpRobot();
                while (robot.isRunning() && !controller.isTerminated())
                    controller.control();
            }
            catch (Exception e) { e.printStackTrace(System.err); }
        }
        
        private RobotController controller = null;
        private SimulatedRobot robot = null;
    }

    //--------------------------------------------------------------------------
    
    @Override
    public void createCommunicator(CommunicatorLogic logic)
    {
        SimulatedCommunicator comm = new SimulatedCommunicator("sim-robot-comm",
                                                               "sim-robot",
                                                               logic,
                                                               sim(),
                                                               channel());
        setCommunicator(comm);
        channel().setThread(comm);
        comm.start();
    }
    
    //--------------------------------------------------------------------------
        
    private int leftPower, rightPower; // milliV
    private int leftRotCtr, rightRotCtr; // deg
    private int resetLeftRotCtr, resetRightRotCtr; // deg
    private int[] distance; // mm
    
    private final Integer mutexRotCtrL = 1, mutexRotCtrR = 2;
    private final Integer mutexPwrL = 3, mutexPwrR = 4;
}
