package control;

import run.Robot;


/**
 * Abstract superclass of segway controllers. 
 */
public abstract class RobotController extends Controller
{
    public RobotController(Robot robot)
    {
        this.robot = robot;
        
        lPwr = rPwr = 0;
        pwrOrder = false;
    }
    
    /** @return interface to the controlled robot */
    public final Robot robot() { return robot; }
    
    /** @return power currently applied on the left motor (mV) */
    public final synchronized int leftPower() { return lPwr; }
    
    /** @return power currently applied on the right motor (mV) */
    public final synchronized int rightPower() { return rPwr; }
    
    //--------------------------------------------------------------------------
    
    /** @return power enforced into the [-100,100] range */
    protected final int limitPower(int power)
    {
        if (power > 100) return 100;
        if (power < -100) return -100;
        return power;
    }
    
    /** Apply the specified powers on the motors after regulation. */
    protected final void applyControl(double leftPower, double rightPower)
    {
        synchronized (this)
        {
            lPwr = limitPower((int)leftPower);
            rPwr = limitPower((int)rightPower);
        }
        
        if (pwrOrder)
        {
            robot().controlLeftMotor(lPwr);
            robot().controlRightMotor(rPwr);
        }
        else
        {
            robot().controlRightMotor(rPwr);
            robot().controlLeftMotor(lPwr);
        }
        pwrOrder = !pwrOrder;
    }
    
    //--------------------------------------------------------------------------
    
    private final Robot robot;

    private int lPwr, rPwr;
    private boolean pwrOrder;
}
