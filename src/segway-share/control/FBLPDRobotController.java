package control;

import helper.Ratio;

import java.awt.event.KeyEvent;
import java.io.File;
import java.io.IOException;

import run.Robot;

import comm.CommunicatorLogic;

import control.observe.EulerObservation;
import control.observe.Observation;
import control.observe.PerfectObservation;
import control.observe.UKFObservation;

public class FBLPDRobotController extends RobotController
{
    // number of samples used to measure the gyroscope bias
    static final int GYRO_BIAS_N = 100;
    
    // gyroscope sign (-1 or +1) set based on the sensor orientation
    static final int GYRO_SIGN = +1;
    
    // If robot power is saturated (over +/- 100) for over this time limit
    // then robot must have fallen. In milliseconds.
    static final int TIME_FALL_LIMIT = 500;

    // motion model parameters
    static final double g = 9.80665;
    static final double m = 0.0165;
    static final double R = 0.0216;
    static final double w = 0.022;
    static final double Jw = 0.5*m*R*R;
    static final double M = 0.55;
    static final double W = 0.15;
    static final double D = 0.045;
    static final double H = 0.158;
    static final double L = H/2;
    static final double Jpsi = 0.00805;
    static final double Jphi = 0.005;
    static final double psi0 = -2 * Ratio.DEG_TO_RAD; 
    static final double B = 0.18;
    static final double K = 20;
    static final double minPowerLow = -6;
    static final double minPowerHigh = 6;
    static final double minPowerTrunc = 3;

    // PD control parameters
    static final double KPITCHDAMP = 100;
    static final double KYAWDAMP = 60;
    static final double KROLLDAMP = 60;
    static final double KPITCHSPRING = KPITCHDAMP*KPITCHDAMP/4;
    
    // Target delay between two consecutive control steps (ms).
    static final int CONTROL_DELAY = 10;

    // Target delay between two consecutive observer steps (ms). 
    static final int OBSERVER_DELAY = 5;
    
    // Gyroscope model parameters.
    static final double A1 = 77.5, A2 = 1500, B1 = 1590;
    
    //--------------------------------------------------------------------------
    
    public FBLPDRobotController(Robot robot)
    {
        super (robot);
        commLogic = new CommunicatorLogicImpl();
        /*
        observation = new EulerObservation(this, OBSERVER_DELAY,
                                           GYRO_SIGN, 0.0,
                                           A1, A2, B1,
                                           R, W, 1);
        */
        /*
        observation = new UKFObservation(this, OBSERVER_DELAY,
                                         GYRO_SIGN, 0.0,
                                         A1, A2, B1,
                                         R, W);
        */
        ///*
        observation = new UKFObservation(this, OBSERVER_DELAY,
                                         GYRO_SIGN, 0.0,
                                         A1, A2, B1,
                                         R, W, L, g, K,
                                         B, m, M, Jw,
                                         Jpsi, Jphi, psi0);
        //*/
        perfObs = new PerfectObservation(this, OBSERVER_DELAY, GYRO_SIGN, 0.0);
        
        B2 = 2*B;
        MLL = M*L*L;
        MLR = M*L*R;
        MgL = M*g*L;
        WWB = W*W*B;
        KRW = K*R*W;
        RRMLL2 = 2*R*R*M*L*L;
        H11 = M*L*L + Jpsi;
        H22 = (2*m + M)*R*R + 2*Jw;
        H11MH22 = H11*H22;
        ht = m*R*R*W*W + W*W*Jw + 2*R*R*Jphi;
    }
    
    @Override
    public void initialize() throws Exception
    {
        super.initialize();
        
        gyroOffset = 0;
        for (int i = 0; i < GYRO_BIAS_N; ++i)
        {
            gyroOffset += GYRO_SIGN * robot().readGyro();
            robot().msDelay(5);
        }
        gyroOffset /= GYRO_BIAS_N;
        gyroOffset *= Ratio.DEG_TO_RAD;
        observation.setGyroOffset(gyroOffset);
        
        accel = turn = 0;
        dThetaRef = dPhiRef = 0;
        
        perfObs.openLog(new File("../log/obs-perf.log"));
        observation.openLog(new File("../log/obs-euler.log"));
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
            robot().spawn("observation", observation).start();
            robot().spawn("obs-perf", perfObs).start();
            robot().msDelay(5);
            return;
        }

        double psi, dPsi, dTheta, dPhi;
        synchronized (observation)
        {
            psi = observation.pitch();
            dPsi = observation.dPitch();
            dTheta = observation.dRoll();
            dPhi = observation.dYaw();
        }
        
        double psiShifted = psi - psi0;
        double sinPsi = Math.sin(psiShifted);
        double cosPsi = Math.cos(psiShifted);
        
        double dThetaSdPsi = dTheta - dPsi;
        double H12 = MLR*cosPsi;
        double h = RRMLL2*sinPsi*sinPsi + ht;
        
        double fh4 = MLL*dPhi*dPhi*sinPsi*cosPsi
                   + MgL*sinPsi
                   + B2*dThetaSdPsi
             , fh5 = MLR*dPsi*dPsi*sinPsi
                   - B2*dThetaSdPsi
             , fh6 = -RRMLL2*dPhi*dPsi*sinPsi*cosPsi
                   - WWB*dPhi;
        
        double Hdiv = H11MH22 - H12*H12;
        double f4 = (H22*fh4 - H12*fh5)/Hdiv;
        double f5 = (H11*fh5 - H12*fh4)/Hdiv;
        double f6 = fh6/h;
        
        double g4 = -K*(H12 + H22)/Hdiv;
        double g5 = K*(H11 + H12)/Hdiv;
        double g6 = KRW/h;
        
        double w1 = -KPITCHSPRING*psiShifted -KPITCHDAMP*dPsi;
        double w2, u;
        synchronized (commLogic)
        {
            w2 = -KYAWDAMP*(dPhi - dPhiRef);
            u = -KROLLDAMP*(dTheta - dThetaRef);
        }
        
        double p = f5 - g5*f4/g4;
        double q = g5/g4;
        w1 += (p - u) / q;
        
        double LgLf_det = 2*g4*g6;
        double v1 = (g6*(w1-f4) - g4*(w2-f6)) / LgLf_det * VOLT_TO_MILLIVOLT;
        double v2 = (g6*(w1-f4) + g4*(w2-f6)) / LgLf_det * VOLT_TO_MILLIVOLT;

        if (-minPowerTrunc > v1 && v1 > minPowerLow) v1 = minPowerLow;
        if (minPowerTrunc < v1 && v1 < minPowerHigh) v1 = minPowerHigh;
        if (-minPowerTrunc > v2 && v2 > minPowerLow) v2 = minPowerLow;
        if (minPowerTrunc < v2 && v2 < minPowerHigh) v2 = minPowerHigh;
        
        if (Math.abs(v1+v2)/2 < 100) tMotorPosOk = time;
        if (time - tMotorPosOk > TIME_FALL_LIMIT) { terminate(); return; }
        
        applyControl(v1, v2);
        //applyControl(20, 20);
        robot().msDelay(CONTROL_DELAY);
    }
    
    @Override
    public void terminate()
    {
        super.terminate();
        
        try
        {
            perfObs.closeLog();
            observation.closeLog();
        }
        catch (IOException e) { e.printStackTrace(System.err); }
    }
    
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
            channel().flush();
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
                default :
                {
                    // drive
                    boolean chgAccel = false;
                    boolean chgTurn = false;
                    
                    for (byte i = 0; i < n; ++i)
                        switch (channel().readShort())
                        {
                            case KeyEvent.VK_UP :
                                accel += 0.1;
                                chgAccel = true;
                                break;
                            case KeyEvent.VK_DOWN :
                                accel += -0.1;
                                chgAccel = true;
                                break;
                            case KeyEvent.VK_LEFT :
                                turn += 0.1;
                                chgTurn = true;
                                break;
                            case KeyEvent.VK_RIGHT :
                                turn += -0.1;
                                chgTurn = true;
                                break;
                        }
                    
                    if (!chgAccel && 0 != accel)
                    {
                        if (Math.abs(accel) < 0.1) accel = 0;
                        else accel -= 0.1*Math.signum(accel);
                    }
                    if (!chgTurn && 0 != turn)
                    {
                        if (Math.abs(turn) < 0.1) turn = 0;
                        else turn -= 0.1*Math.signum(turn);
                    }
                    
                    accel = Math.min(1, Math.max(accel, -1));
                    turn = Math.min(1, Math.max(turn, -1));
                    
                    synchronized (commLogic)
                    {
                        dThetaRef = accel * 300 * Ratio.DEG_TO_RAD;
                        dPhiRef = turn * 150 * Ratio.DEG_TO_RAD;
                    }
                    break;
                }
            }
        }
    }
    
    //--------------------------------------------------------------------------
    
    private static final double VOLT_TO_MILLIVOLT = 1000;
    
    private final CommunicatorLogicImpl commLogic;
    private final Observation observation;
    
    private Observation perfObs;

    private double accel, turn;
    private double dThetaRef, dPhiRef;
    
    private double MLL, MgL, MLR, RRMLL2, B2, WWB, KRW;
    private double H11, H22, H11MH22, ht;
    
    private double gyroOffset;
    private long prevTime, tMotorPosOk;
}
