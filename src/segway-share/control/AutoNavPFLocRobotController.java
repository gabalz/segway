package control;

import java.awt.event.KeyEvent;

import vecmat.Vector;
import ode.CachedODE;
import ode.Euler;
import ode.ODESolver;
import run.Robot;
import run.ThreadLogic;

import comm.CommunicatorLogic;

import control.RobotController;

/**
 * Low-level robot controller navigated by an AutoNavPFLocPCController.
 */
public class AutoNavPFLocRobotController extends RobotController
{
    // number of samples used to measure the gyroscope bias
    static final int GYRO_BIAS_N = 200;
    
    // gyroscope sign (-1 or +1) set based on the sensor orientation
    static final int GYRO_SIGN = +1;
    
    // Main balancing constants.
    static final double WHEEL_RATIO = 0.8;
    static final double KGYROANGLE = 7.5;
    static final double KGYROSPEED = 1.15;
    static final double KPOS = 0.07;
    static final double KSPEED = 0.1;
    
    // This constant aids in drive control. When the robot starts moving 
    // because of user control, this constant helps get the robot leaning 
    // in the right direction. Similarly, it helps bring robot 
    // to a stop when stopping.
    static final double KDRIVE = -0.02;
    
    // Power differential used for steering based on difference of target
    // steering and actual motor difference.
    static final double KSTEER = 0.25;
    
    // This constant is in degrees/second for maximum speed. Note that
    // position and speed are measured as the sum of the two motors, in
    // other words, 600 would actually be 300 degrees/second for each motor.
    static final double CONTROL_SPEED = 600.0;
    
    // If robot power is saturated (over +/- 100) for over this time limit
    // then robot must have fallen. In milliseconds.
    static final int TIME_FALL_LIMIT = 500;
    
    // Target delay between two consecutive control steps (ms).
    static final int CONTROL_DELAY = 10;

    // Target delay between two consecutive observer steps (ms). 
    static final int OBSERVER_DELAY = 3;
    
    // Gyroscope model parameters.
    static final double A1 = 77.5, A2 = 1500, B1 = 1590;
    
    // Observer parameters.
    static final double L1 = 22.33, L2 = 18.0, L3 = 107.0;
    static final double SCALING_FACTOR = 1.0 - 0.0025;
    
    //--------------------------------------------------------------------------
    
    public AutoNavPFLocRobotController(Robot robot)
    {
        super (robot);
        commLogic = new CommunicatorLogicImpl();
        observer = new Observer();
    }

    @Override
    public void initialize() throws Exception
    {
        super.initialize();
        
        gyroOffset = 0.0;
        for (int i = 0; i < GYRO_BIAS_N; ++i)
        {
            gyroOffset += GYRO_SIGN * robot().readGyro();
            robot().msDelay(5);
        }
        gyroOffset /= GYRO_BIAS_N;
        gyroAngle = 0.0;
        
        mrcSumPrev = 0;
        mrcDeltaP1 = mrcDeltaP2 = mrcDeltaP3 = 0;
        
        motorPosition = 0.0;
        motorControlDrive = 0.0;
        motorControlSteer = 0.0;
        motorControlAccel = 0.0;
        motorDiffTarget = 0.0;
        
        prevTime = 0;
        tMotorPosOk = 0;
        
        robot().createCommunicator(commLogic);
    }
    
    @Override
    public void control() throws Exception
    {
        long time = robot().currentTimeMillis();
        if (prevTime == 0)
        {
            // 1st step
            prevTime = time;
            tMotorPosOk = time;
            robot().resetLeftRotationCounter();
            robot().resetRightRotationCounter();
            robot().spawn("observer", observer).start();
            // robot().spawn("datalogger", new DataLogger()).start();
            robot().msDelay(5);
            return;
        }
        
        double dt = (time - prevTime) * MILLISEC_TO_SEC;
        prevTime = time;
        
        double gyroValue = readGyro();
        gyroAngle += gyroValue * dt;
        
        int mrcLeft = robot().leftRotationCounter();
        int mrcRight = robot().rightRotationCounter();
        int mrcSum = mrcLeft + mrcRight;
        int motorDiff = mrcLeft - mrcRight;
        int mrcDelta = mrcSum - mrcSumPrev;
        motorPosition += mrcDelta;
        motorPosition -= motorControlDrive * dt;
        
        double motorControlDrive = 0.0;
        double motorControlSteer = 0.0;
        synchronized (commLogic)
        {
            motorControlDrive = this.motorControlDrive;
            motorControlSteer = this.motorControlSteer;
        }

        double motorSpeed =
            (mrcDelta + mrcDeltaP1 + mrcDeltaP2 + mrcDeltaP3) / (4.0 * dt);
        
        mrcDeltaP3 = mrcDeltaP2;
        mrcDeltaP2 = mrcDeltaP1;
        mrcDeltaP1 = mrcDelta;
        mrcSumPrev = mrcSum;
        
        int power = (int)( (KGYROSPEED * gyroValue
                            + KGYROANGLE * gyroAngle) / WHEEL_RATIO
                           + KPOS * motorPosition
                           + KDRIVE * motorControlDrive
                           + KSPEED * motorSpeed);
        
        if (Math.abs(power) < 100) tMotorPosOk = time;
        if (time - tMotorPosOk > TIME_FALL_LIMIT) { terminate(); return; }
        
        // steering
        motorDiffTarget += motorControlSteer * dt;
        int powerSteer = (int)(KSTEER * (motorDiffTarget - motorDiff));
        int lPower = power + powerSteer;
        int rPower = power - powerSteer;
        
        robot().controlLeftMotor(limitPower(lPower));
        robot().controlRightMotor(limitPower(rPower));
        
        int delay = CONTROL_DELAY + (int)(time - robot().currentTimeMillis());
        if (0 < delay) robot().msDelay(delay); else robot().msDelay(1);
    }
    
    private synchronized double readGyro()
    { return GYRO_SIGN * robot().readGyro() - gyroOffset; }
    
    //--------------------------------------------------------------------------
    
    /**
     * Luenberger observer to determine the pitch angular velocity.
     * The pitch velocity is also integrated (into the first component)
     * and mean corrected to get the pitch estimate.
     */
    private class Observer extends ThreadLogic
    {
        public Observer()
        {
            x = Vector.zero(4);
            xx = Vector.zero(4);
            odeSolver = new Euler(new ObserverODE(), 0.0);
        }
        
        @Override
        public void run()
        {
            prevT = currentTimeMillis();
            while (true)
            {
                update();
                msDelay(OBSERVER_DELAY);
            }
        }
        
        public synchronized double update()
        {
            t = currentTimeMillis();
            y = readGyro();
            
            odeSolver.setDt((t - prevT) / 1000.0);
            odeSolver.next(0.0, x, xx);
            
            prevT = t;
            Vector tmp = x; x = xx; xx = tmp;
            
            double x0 = x.get(0) * SCALING_FACTOR;
            x.set(0, x0);            
            return x0;
        }
        
        private class ObserverODE extends CachedODE
        {
            public ObserverODE() { super (4, 4); }
            
            @Override
            public Vector f(double t, Vector x)
            {
                Vector result = nextCachedVector();
                double x1 = x.get(1), x2 = x.get(2), x3 = x.get(3);
                double d = y - x2;
                result.set(0,    x1);
                result.set(1, A1*x1 + K2*x2 + K3*x3 + L1*d);
                result.set(2,                    x3 + L2*d);
                result.set(3, B1*x1 - A2*x2 - A1*x3 + L3*d);
                return result;
            }
            
            private static final double K2 = -A1*A2/B1, K3 = (A2-A1*A1)/B1;
        }
        
        private long t, prevT;
        private double y;
        private Vector x, xx;
        private final ODESolver odeSolver;
    }
    
    /*
    private class DataLogger extends ThreadLogic
    {
        @Override
        public void run()
        {
            FileWriter fw = null;
            try
            {
                fw = new FileWriter(new File("gyro-pitch.log"));
                fw.write("# time(sec) pitch(deg) gyro(deg) pitch-est(deg)\n");
                while (true)
                {
                    double pitch;
                    synchronized (observer) { pitch = observer.x.get(0); }
                    
                    State state = robot().simDynState();
                    if (5*60 < state.time()) break; 
                    fw.write("" + state.time()
                          + " " + (state.pitch() * Ratio.RAD_TO_DEG)
                          + " " + readGyro()
                          + " " + pitch
                                + "\n");
                    fw.flush();
                    msDelay(3);
                }
                System.out.println("datalogger ended");
            }
            catch (IOException e) { e.printStackTrace(System.err); }
            finally
            { if (fw != null) try { fw.close(); } catch (IOException e) {} }
        }
    }
    */
    
    //--------------------------------------------------------------------------
    
    // Maximum number of key codes to be received.
    private static final byte MAX_KEY_CODES = 3;
    
    private class CommunicatorLogicImpl extends CommunicatorLogic
    {
        @Override
        public void initalize() throws Exception
        {
            super.initalize();
            channel().writeByte(MAX_KEY_CODES);
            channel().writeByte((byte)robot().numDistances());
            channel().flush();
            
            d = new int[robot().numDistances()];
            mrcPrevL = robot().leftRotationCounter();
            mrcPrevR = robot().rightRotationCounter();
        }
        
        @Override
        public void logic() throws Exception
        {
            byte n = channel().readByte();
            switch (n)
            {
                case -1 :
                {
                    // termination request
                    robot().controlLeftMotor(0);
                    robot().controlRightMotor(0);
                    terminate();
                    break;
                }
                case -2 :
                {
                    final int dLen = d.length;
                    
                    // observation request
                    double pitch = 0.0;
                    int mrcL = 0, mrcR = 0;
                    for (int i = 0; i < dLen; ++i)
                    {
                        if (robot().isSimulated()) msDelay(6);
                        
                        d[i] = robot().readDistance(i);
                        pitch += observer.update();
                        mrcL += robot().leftRotationCounter();
                        mrcR += robot().rightRotationCounter();
                    }
                    pitch /= dLen;
                    mrcL /= dLen; mrcR /= dLen;
                    
                    short deltaL = (short)(mrcL - mrcPrevL);
                    short deltaR = (short)(mrcR - mrcPrevR);
                    
                    channel().writeShort((short)(100.0 * pitch));
                    channel().writeShort(deltaL);
                    channel().writeShort(deltaR);
                    for (int i = 0; i < dLen; ++i)
                        channel().writeShort((short)d[i]);
                    channel().flush();
                    
                    mrcPrevL = mrcL;
                    mrcPrevR = mrcR;
                    break;
                }
                default :
                {
                    // receiving controls
                    boolean controlAccelChanged = false;
                    boolean controlTurnChanged = false;
                    for (byte i = 0; i < n; ++i)
                        switch (channel().readShort())
                        {
                            case KeyEvent.VK_UP :
                                controlAccelChanged = true;
                                motorControlAccel += 0.25;
                                if (motorControlAccel > 1.0)
                                    motorControlAccel = 1.0;
                                break;
                            case KeyEvent.VK_DOWN :
                                controlAccelChanged = true;
                                motorControlAccel -= 0.25;
                                if (motorControlAccel < -1.0)
                                    motorControlAccel = -1.0;
                                break;
                            case KeyEvent.VK_LEFT :
                                controlTurnChanged = true;
                                motorControlTurn -= 0.1;
                                if (motorControlTurn < -1.0)
                                    motorControlTurn = -1.0;
                                break;
                            case KeyEvent.VK_RIGHT :
                                controlTurnChanged = true;
                                motorControlTurn += 0.1;
                                if (motorControlTurn > 1.0)
                                    motorControlTurn = 1.0;
                                break;
                        }
                    
                    if (!controlAccelChanged && 0.0 != motorControlAccel)
                    {
                        controlAccelChanged = true;
                        motorControlAccel -=
                            Math.signum(motorControlAccel) *
                            Math.min(Math.abs(motorControlAccel), 0.1);
                    }
                    
                    if (!controlTurnChanged && 0.0 != motorControlTurn)
                    {
                        controlTurnChanged = true;
                        motorControlTurn -=
                            Math.signum(motorControlTurn) *
                            Math.min(Math.abs(motorControlTurn), 0.5);
                    }
                    
                    synchronized (commLogic)
                    {
                        if (controlAccelChanged)
                            motorControlDrive =
                                motorControlAccel * CONTROL_SPEED;
                        if (controlTurnChanged)
                            motorControlSteer =
                                motorControlTurn * CONTROL_SPEED;
                    }
                    break;
                }
            }
        }
        
        private int mrcPrevL, mrcPrevR;
        private int[] d;
    }
    
    //--------------------------------------------------------------------------
    
    private static final double MILLISEC_TO_SEC = 1e-3;    
    private final CommunicatorLogicImpl commLogic;
    private final Observer observer;
    
    private long prevTime, tMotorPosOk;
    private int mrcSumPrev, mrcDeltaP1, mrcDeltaP2, mrcDeltaP3;
    private double gyroOffset, gyroAngle;
    
    private double motorControlDrive;
    private double motorControlSteer;
    private double motorDiffTarget;
    private double motorControlAccel;
    private double motorControlTurn;
    private double motorPosition;
}
