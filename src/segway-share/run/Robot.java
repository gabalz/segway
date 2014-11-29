package run;

import comm.CommunicatorLogic;

/**
 * Interface of a real/simulated Segway robot.
 */
public interface Robot extends RobotPC
{
    /** Reset the left rotation counter to zero. */
    void resetLeftRotationCounter();
    
    /** Reset the right rotation counter to zero. */
    void resetRightRotationCounter();
    
    /** @return left rotation counter (deg) */
    int leftRotationCounter();
    
    /** @return right rotation counter (deg) */
    int rightRotationCounter();
    
    //--------------------------------------------------------------------------
    
    /** @return gyroscope sensor reading (deg/sec) */
    int readGyro();
    
    /** @return number of distance sensors */
    int numDistances();
    
    /** @return the i-th distance sensor reading (mm) */
    int readDistance(int i);
    
    //--------------------------------------------------------------------------
    
    /** Apply "power" on the left motor (milliV). */
    void controlLeftMotor(int power);
    
    /** Apply "power" on the right motor (milliV). */
    void controlRightMotor(int power);
    
    //--------------------------------------------------------------------------
    
    /** Create and run a communicator thread. */
    void createCommunicator(CommunicatorLogic logic);
}
